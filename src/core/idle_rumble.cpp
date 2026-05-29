#include "core/idle_rumble.h"

#include <Arduino.h>
#include <math.h>

#include "core/launch_control.h"
#include "core/safety.h"

namespace cdi::core::idle_rumble {
namespace {

// Cross-core: WS handlers (core 0) menulis config, live_stats (core 1)
// dan spark ISR (core 1) baca runtime state. volatile + word-atomic.
volatile bool                s_enabled       = false;
volatile cdi::IdleRumbleMode s_mode          = cdi::IdleRumbleMode::SUBTLE;
volatile cdi::rpm_t          s_rpmLo         = 800;
volatile cdi::rpm_t          s_rpmHi         = 2500;
volatile float               s_maxRetardDeg  = 6.0f;
volatile uint8_t             s_skipFireN     = 4;       // fire 4, skip 1 (~20%)
volatile uint16_t            s_sustainMs     = 1500;
volatile uint16_t            s_minUptimeSec  = 15;

// Runtime
//
// Cross-core access pattern di sini:
//   - tick() jalan di core 1 (loop). Membaca + menulis hampir semua
//     runtime state.
//   - setEnabled / setMode dipanggil dari WS handler di core 0.
//     Beberapa reset state ini (s_inBandSinceMs=0, s_cooldownUntilMs=0,
//     s_minRpmInWindow=65535).
// Tanpa volatile, compiler bisa cache value di register pada tick()
// dan miss reset yang dilakukan core 0. Bug ini tidak crash sistem
// tapi bisa bikin behavior idle_rumble inconsistent setelah user
// toggle mode dari UI.
//
// Single-store 32-bit aligned writes pada Xtensa atomic di word level,
// jadi volatile saja cukup tanpa perlu mutex.
volatile bool      s_active        = false;
volatile float     s_currentRetard = 0.0f;
volatile uint32_t  s_inBandSinceMs = 0;     // 0 = not in band
uint32_t           s_lastSampMs    = 0;     // tick-internal, single core

// Skip-fire pattern counter (touched only di spark ISR via shouldFireThisCycle
// chain). volatile for safety::shouldFire reentry.
volatile uint32_t  s_fireCounter   = 0;

// LCG random untuk JITTER (loop side, tick()). Tidak shared dengan ISR
// supaya tidak race.
uint32_t s_lcg = 0xBEEFC0DE;

// LCG random untuk ISR side (BRAP_BRAP burst skip). Separate state
// dari s_lcg supaya tidak race ketika tick() dan ISR sama-sama update.
// volatile karena ISR + cuma kebaca/ditulis dari ISR.
volatile uint32_t s_isr_lcg = 0xDEADCAFE;

// ── Stall-guard cooldown ──
// Skip-heavy modes (AGGRESSIVE/DRAG_BURBLE/NGOROK/BRAP_BRAP) bisa drag
// idle RPM turun ke bawah rpmLo → exit band → disengage → recover →
// re-engage → loop death-spiral atau stall. Stall guard:
//   * Track minimum RPM dilihat dalam window in-band (1s)
//   * Kalau min < (rpmLo + 50), set cooldown 5 detik — selama cooldown
//     tidak akan engage lagi walau RPM normal kembali
//   * Cooldown reset kalau idle stabilo (min > rpmLo + 200) untuk 5s
//
// Net effect: kalau idle motor user marginal, rumble auto-back-off,
// engine recover, tidak stuck di death-spiral.
volatile uint32_t  s_cooldownUntilMs = 0;
volatile cdi::rpm_t s_minRpmInWindow = 65535;
uint32_t           s_minWinStartMs   = 0;   // tick-internal, single core

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
        //
        // EXIT-LOW stall-guard: kalau exit terjadi via low-RPM side
        // (rpm < rpmLo + 100), artinya rumble berkontribusi ke RPM
        // drop yang mendekati stall. Set cooldown 5 detik supaya
        // engine bisa recover stable sebelum re-engage.
        // Exit via HIGH-RPM (user gases up) tidak trigger cooldown.
        if (s_active && rpm < s_rpmLo + 100) {
            s_cooldownUntilMs = now + 5000;
            Serial.printf("[rumble] stall-guard cooldown 5s (exit-low @%u rpm)\n",
                          (unsigned)rpm);
        } else if (s_active) {
            Serial.printf("[rumble] disengage @ %u rpm (out of band)\n",
                          (unsigned)rpm);
        }
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        s_minRpmInWindow = 65535;     // reset window state
        return;
    }

    // ─── Gating layer 5: sustain in band ──────────────────────────
    if (s_inBandSinceMs == 0) {
        s_inBandSinceMs = now;
    }
    if ((int32_t)(now - s_inBandSinceMs) < (int32_t)s_sustainMs) {
        s_active = false; s_currentRetard = 0.0f;
        return;
    }

    // ─── Gating layer 6: stall-guard cooldown ─────────────────────
    if ((int32_t)(now - s_cooldownUntilMs) < 0) {
        // Recent stall avoidance — don't engage selama cooldown.
        s_active = false; s_currentRetard = 0.0f;
        return;
    }

    // Track minimum RPM dalam rolling 1-detik window untuk stall detect
    if (rpm < s_minRpmInWindow) s_minRpmInWindow = rpm;
    if ((int32_t)(now - s_minWinStartMs) >= 1000) {
        const cdi::rpm_t margin = (cdi::rpm_t)100;
        if (s_minRpmInWindow <= s_rpmLo + margin && s_active) {
            // RPM mendekati floor band → backoff untuk hindari stall.
            // Skip-heavy modes paling rentan; SUBTLE/NGEBASS no skip,
            // tapi heavy retard masih bisa drop RPM, jadi cooldown
            // universal (5 detik).
            s_cooldownUntilMs = now + 5000;
            Serial.printf("[rumble] stall-guard cooldown 5s (minrpm=%u floor=%u)\n",
                          (unsigned)s_minRpmInWindow, (unsigned)s_rpmLo);
            s_active = false; s_currentRetard = 0.0f;
            s_minRpmInWindow = 65535;
            s_minWinStartMs = now;
            return;
        }
        s_minRpmInWindow = 65535;
        s_minWinStartMs = now;
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
        float r = 0.0f;
        switch (s_mode) {
            case cdi::IdleRumbleMode::SUBTLE:
                r = jitterRetard(s_maxRetardDeg);          // halus: jitter 0..max
                break;
            case cdi::IdleRumbleMode::AGGRESSIVE: {
                // Lebih DALAM dari SUBTLE + floor 6° → terasa walau slider
                // retard di-set kecil. Pusat ~m, amplitudo jitter besar.
                const float m = (s_maxRetardDeg < 6.0f) ? 6.0f : s_maxRetardDeg;
                r = m * 0.45f + jitterRetard(m * 0.55f);
                break;
            }
            case cdi::IdleRumbleMode::DRAG_BURBLE: {
                float m = s_maxRetardDeg * 2.0f;
                if (m < 10.0f) m = 10.0f;
                if (m > 14.0f) m = 14.0f;
                r = m * 0.3f + jitterRetard(m * 0.7f);     // dalam, floor tinggi
                break;
            }
            case cdi::IdleRumbleMode::NGEBASS: {
                // Sine-wave modulation 1.5 Hz → engine pulse pelan
                // 'wub-wub-wub'. Phase dari millis() supaya konsisten.
                // Depth = user config × 1.5, capped di 10°. User bisa
                // dial in subtle NGEBASS (retard=2 → depth=3°) atau
                // dramatic (retard=6 → depth=9°).
                const float period_ms = 667.0f;   // 1.5 Hz
                const float t = ((float)(now % (uint32_t)period_ms)) / period_ms;
                const float sine = sinf(t * 2.0f * 3.14159265f);
                float depth = s_maxRetardDeg * 1.5f;
                if (depth > 10.0f) depth = 10.0f;
                if (depth < 2.0f)  depth = 2.0f;       // floor lebih tinggi supaya 'wub' jelas
                r = (sine * 0.5f + 0.5f) * depth;
                break;
            }
            case cdi::IdleRumbleMode::NGOROK: {
                // Random retard 0-4°, mostly low. Burst skip handled di
                // shouldFireThisCycle untuk efek 'snoring' tidak monoton.
                r = jitterRetard(s_maxRetardDeg < 5.0f ? 5.0f : s_maxRetardDeg);
                break;
            }
            case cdi::IdleRumbleMode::BRAP_BRAP: {
                // Heavy retard 4-8°, dikombinasi dengan frequent skip.
                const float base = 5.0f;
                const float jit  = (s_maxRetardDeg < 5.0f ? 5.0f : s_maxRetardDeg);
                r = base + jitterRetard(jit);
                if (r > 14.0f) r = 14.0f;
                break;
            }
            default:
                r = 0.0f;
                break;
        }
        if (r > 10.0f) r = 10.0f;   // respect the 10° anti-stall ceiling
                                    // (DRAG/BRAP floors could reach 14°) — audit LOW9
        s_currentRetard = r;
        s_lastSampMs = now;
    }
}

// ── Spark ISR chain — called from safety::shouldFire ──────────────
// Per-mode skip pattern. Active state sudah di-cek upstream.
// IRAM_ATTR: dipanggil dari CH1 GPIO ISR (lewat safety::shouldFire),
// jadi HARUS resident di IRAM — kalau flash di-erase (NVS/LittleFS),
// fetch instruksi dari flash di tengah ISR = crash (audit H1).
bool IRAM_ATTR shouldFireThisCycle() {
    if (!s_active) return true;       // not engaged, never block fire

    switch (s_mode) {
        case cdi::IdleRumbleMode::SUBTLE:
        case cdi::IdleRumbleMode::NGEBASS:
            // Jitter retard / sine modulation saja, tidak ada skip.
            return true;

        case cdi::IdleRumbleMode::AGGRESSIVE:
        case cdi::IdleRumbleMode::DRAG_BURBLE: {
            // Fire-N-skip-1 sederhana (N = s_skipFireN dari config).
            const uint8_t n = s_skipFireN;
            if (n == 0) return true;
            s_fireCounter++;
            return (s_fireCounter % (n + 1)) != 0;
        }

        case cdi::IdleRumbleMode::NGOROK: {
            // V-twin snoring emulation: fire 5, skip 2, repeat.
            // Pattern audible: 'BOM-BOM-BOM-BOM-BOM-(silent)-(silent)-...'
            // Cycle 7 cycles per pattern, dengan 5/7 fire rate (71%).
            s_fireCounter++;
            const uint32_t pos = s_fireCounter % 7u;
            return pos < 5u;          // fire on 0..4, skip on 5,6
        }

        case cdi::IdleRumbleMode::BRAP_BRAP: {
            // Drag-bike popping: skip 1-of-4 reguler + 5% random extra skip.
            // Pakai s_isr_lcg terpisah dari s_lcg (loop side) supaya
            // tidak ada race antara ISR dan tick() yang sama-sama update
            // RNG state.
            s_fireCounter++;
            if ((s_fireCounter & 0x3u) == 0u) return false;
            s_isr_lcg = s_isr_lcg * 1664525u + 1013904223u;
            if ((s_isr_lcg >> 24) < 12u) return false;
            return true;
        }

        default:
            return true;
    }
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
    if (!en) {
        s_active = false; s_currentRetard = 0.0f;
        s_inBandSinceMs = 0;
        s_cooldownUntilMs = 0;        // reset cooldown
        s_minRpmInWindow = 65535;
    }
    Serial.printf("[rumble] enabled=%d\n", en ? 1 : 0);
}

void setMode(cdi::IdleRumbleMode m) {
    s_mode = m;
    s_active = false;
    s_currentRetard = 0.0f;
    s_fireCounter = 0;
    s_cooldownUntilMs = 0;            // reset cooldown saat ganti mode
    s_minRpmInWindow = 65535;
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
