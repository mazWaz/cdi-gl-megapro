#include "core/idle_rumble.h"

#include <Arduino.h>

#include "core/launch_control.h"
#include "core/safety.h"

namespace cdi::core::idle_rumble {
namespace {

// Cross-core: WS handlers (core 0) menulis config, live_stats (core 1)
// dan spark ISR (core 1) baca runtime state. volatile + word-atomic.
volatile bool                s_enabled       = false;
volatile cdi::IdleRumbleMode s_mode          = cdi::IdleRumbleMode::SUBTLE;
volatile cdi::rpm_t          s_rpmLo         = 1000;
volatile cdi::rpm_t          s_rpmHi         = 2000;
volatile float               s_maxRetardDeg  = 3.0f;
volatile uint8_t             s_skipFireN     = 7;       // fire 7, skip 1
volatile uint16_t            s_sustainMs     = 3000;
volatile uint16_t            s_minUptimeSec  = 60;

// Runtime
volatile bool      s_active        = false;
volatile float     s_currentRetard = 0.0f;
uint32_t           s_inBandSinceMs = 0;     // 0 = not in band
uint32_t           s_lastSampMs    = 0;

// Skip-fire pattern counter (touched only di spark ISR via shouldFireThisCycle
// chain). volatile for safety::shouldFire reentry.
volatile uint32_t  s_fireCounter   = 0;

// LCG random for jitter
uint32_t s_lcg = 0xBEEFC0DE;

float jitterRetard(float max){
    if (max <= 0.0f) return 0.0f;
    s_lcg = s_lcg * 1664525u + 1013904223u;
    const uint32_t r = (s_lcg >> 16) & 0xFF;     // 0-255
    return ((float)r / 255.0f) * max;            // 0..max°
}

} // anonymous

void begin() {
    Serial.println("[rumble] idle rumble module ready (off by default)");
}

void tick(cdi::rpm_t rpm) {
    const uint32_t now = millis();

    // ─── Gating layer 1: master enable + mode ─────────────────────
    if (!s_enabled || s_mode == cdi::IdleRumbleMode::OFF) {
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        return;
    }

    // ─── Gating layer 2: not yet warm (boot uptime) ───────────────
    // ESP baru boot → engine kemungkinan cold → high-idle fragile
    // → jangan kasih rumble sampai user sudah riding 60+ detik.
    if ((now / 1000u) < s_minUptimeSec) {
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        return;
    }

    // ─── Gating layer 3: no-signal failsafe / launch active ──────
    // Tidak mau compound cuts atau interfere dengan launch hold.
    if (cdi::core::launch::isActive()) {
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        return;
    }

    // ─── Gating layer 4: in idle RPM band? ────────────────────────
    const bool in_band = (rpm >= s_rpmLo && rpm <= s_rpmHi);
    if (!in_band) {
        // Exit segera tanpa sustain — saat user buka throttle, jangan
        // kasih bumpy acceleration karena skip-fire masih nyala.
        if (s_active) {
            Serial.printf("[rumble] disengage @ %u rpm (out of band)\n",
                          (unsigned)rpm);
        }
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        return;
    }

    // ─── Gating layer 5: sustain in band ──────────────────────────
    if (s_inBandSinceMs == 0) {
        s_inBandSinceMs = now;
    }
    if ((int32_t)(now - s_inBandSinceMs) < (int32_t)s_sustainMs) {
        // Belum cukup lama di band, belum engage
        s_active = false; s_currentRetard = 0.0f;
        return;
    }

    // ─── Engaged. Compute retard + skip pattern. ──────────────────
    if (!s_active) {
        Serial.printf("[rumble] engage @ %u rpm (mode=%d)\n",
                      (unsigned)rpm, (int)s_mode);
    }
    s_active = true;

    // Update retard pelan-pelan (10 ms tick rate dari sample) supaya
    // tidak bikin per-fire CPU spike di live_stats. live_stats baca
    // s_currentRetard apa adanya.
    if ((int32_t)(now - s_lastSampMs) >= 10) {
        float max = s_maxRetardDeg;
        // DRAG_BURBLE = heavier retard depth
        if (s_mode == cdi::IdleRumbleMode::DRAG_BURBLE) {
            max = s_maxRetardDeg * 2.0f;
            if (max > 12.0f) max = 12.0f;       // safety cap
        }
        s_currentRetard = jitterRetard(max);
        s_lastSampMs = now;
    }
}

// ── Spark ISR chain — called from safety::shouldFire ──────────────
// Pattern fire-N-skip-1. SUBTLE mode = no skip (return true always
// when active). AGGRESSIVE+DRAG_BURBLE = skip every (N+1)th fire.
bool shouldFireThisCycle() {
    if (!s_active) return true;       // not engaged, never block fire

    // SUBTLE: jitter retard only, never skip
    if (s_mode == cdi::IdleRumbleMode::SUBTLE) return true;

    // AGGRESSIVE / DRAG_BURBLE: pattern skip
    const uint8_t n = s_skipFireN;
    if (n == 0) return true;          // skip disabled

    s_fireCounter++;
    // Setiap (n+1)th fire = skip
    if ((s_fireCounter % (n + 1)) == 0) {
        return false;
    }
    return true;
}

// ── Public state queries ─────────────────────────────────────────
bool  isEnabled()         { return s_enabled; }
cdi::IdleRumbleMode mode(){ return s_mode; }
cdi::rpm_t rpmLo()        { return s_rpmLo; }
cdi::rpm_t rpmHi()        { return s_rpmHi; }
float maxRetardDeg()      { return s_maxRetardDeg; }
uint8_t skipPattern()     { return s_skipFireN; }
uint16_t sustainMs()      { return s_sustainMs; }
uint16_t minUptimeSec()   { return s_minUptimeSec; }
bool  isActive()          { return s_active; }
float currentRetardDeg()  { return s_active ? s_currentRetard : 0.0f; }

// ── Setters ──────────────────────────────────────────────────────
void setEnabled(bool en) {
    s_enabled = en;
    if (!en) { s_active = false; s_currentRetard = 0.0f; }
    Serial.printf("[rumble] enabled=%d\n", en ? 1 : 0);
}

void setMode(cdi::IdleRumbleMode m) {
    s_mode = m;
    s_active = false;                 // re-engage on next tick
    s_currentRetard = 0.0f;
    s_fireCounter = 0;
    Serial.printf("[rumble] mode=%d\n", (int)m);
}

void setRpmBand(cdi::rpm_t lo, cdi::rpm_t hi) {
    if (lo < 500)   lo = 500;
    if (hi <= lo+200) hi = lo + 200;
    if (hi > 4000)  hi = 4000;        // safety: rumble di luar idle band
    s_rpmLo = lo;
    s_rpmHi = hi;
}

void setMaxRetardDeg(float deg) {
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 10.0f) deg = 10.0f;     // hard cap, di atas ini risiko stall
    s_maxRetardDeg = deg;
}

void setSkipPattern(uint8_t fire_n) {
    if (fire_n > 0 && fire_n < 3) fire_n = 3;   // floor: skip 1-of-4 atau lebih jarang
    if (fire_n > 20) fire_n = 20;
    s_skipFireN = fire_n;
}

void setSustainMs(uint16_t ms) {
    if (ms < 500)   ms = 500;
    if (ms > 10000) ms = 10000;
    s_sustainMs = ms;
}

void setMinUptimeSec(uint16_t s) {
    if (s > 300) s = 300;
    s_minUptimeSec = s;
}

} // namespace cdi::core::idle_rumble
