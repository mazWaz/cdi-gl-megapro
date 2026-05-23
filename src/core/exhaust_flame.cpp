#include "core/exhaust_flame.h"

#include <Arduino.h>

#include "core/safety.h"
#include "core/launch_control.h"
#include "core/alvp.h"
#include "core/spark_scheduler.h"

namespace cdi::core::flame {
namespace {

// ── Cross-core state ──
// Config writers di core 0 (WS handler), reader di core 1 (tick + ISR
// chain). Single-store 32-bit aligned writes pada Xtensa = word-atomic,
// jadi volatile saja cukup.
volatile bool           s_enabled = false;
volatile cdi::FlameMode s_mode    = cdi::FlameMode::OFF;

// Runtime
volatile bool      s_active            = false;
volatile float     s_currentRetard     = 0.0f;
volatile uint32_t  s_engageSinceMs     = 0;  // 0 = not engaging yet
volatile uint32_t  s_activeSinceMs     = 0;  // when active=true was set
volatile uint32_t  s_cooldownUntilMs   = 0;
uint32_t           s_lastSampMs        = 0;  // tick-internal

// ISR-only pattern counter
volatile uint32_t  s_fireCounter       = 0;

// LCG random — separate state untuk loop side (jitter retard)
uint32_t s_lcg = 0xF1AAEC0DE;

// ── Mode parameters ──
struct Params {
    uint16_t engage_delay_ms;
    uint16_t max_duration_ms;
    uint16_t cooldown_ms;
    uint16_t vbat_min_mv;
    float    retard_base_deg;
    float    retard_jitter_deg;
};

constexpr Params SAFE_PARAMS = {
    .engage_delay_ms   = 1000,
    .max_duration_ms   = 3000,
    .cooldown_ms       = 2000,
    .vbat_min_mv       = 11500,
    .retard_base_deg   = 5.0f,
    .retard_jitter_deg = 3.0f,    // total 5..8°
};
constexpr Params AGGRESSIVE_PARAMS = {
    .engage_delay_ms   = 1500,
    .max_duration_ms   = 5000,
    .cooldown_ms       = 5000,
    .vbat_min_mv       = 12000,
    .retard_base_deg   = 12.0f,
    .retard_jitter_deg = 8.0f,    // total 12..20°
};

// Engine warmup proxy: refuse engage sampai uptime > 30s. Tidak ada
// CHT sensor di build ini, jadi pakai uptime sebagai conservative
// guess bahwa exhaust sudah cukup panas.
constexpr uint32_t MIN_UPTIME_MS = 30000;

// RPM drop hysteresis di bawah main_limit untuk auto-disengage.
constexpr uint16_t RPM_DROP_HYST = 300;

// Headroom dari overrev — di atas threshold ini, overrev hard cut
// take priority, flame tidak engage.
constexpr uint16_t OVERREV_HEADROOM = 200;

const Params& currentParams() {
    return (s_mode == cdi::FlameMode::AGGRESSIVE) ? AGGRESSIVE_PARAMS : SAFE_PARAMS;
}

float jitterRetard(const Params& p) {
    s_lcg = s_lcg * 1664525u + 1013904223u;
    const uint32_t r = (s_lcg >> 16) & 0xFF;             // 0-255
    const float frac = (float)r / 255.0f;                // 0..1
    return p.retard_base_deg + frac * p.retard_jitter_deg;
}

void resetState() {
    s_active           = false;
    s_currentRetard    = 0.0f;
    s_engageSinceMs    = 0;
    s_activeSinceMs    = 0;
    s_fireCounter      = 0;
}

} // anonymous

void begin() {
    Serial.println("[flame] exhaust flame module ready (off by default)");
}

void tick(cdi::rpm_t rpm) {
    const uint32_t now = millis();

    // ─── Gating layer 1: master enable ────────────────────────────
    if (!s_enabled || s_mode == cdi::FlameMode::OFF) {
        resetState();
        return;
    }

    // ─── Gating layer 2: not warm yet ─────────────────────────────
    if (now < MIN_UPTIME_MS) {
        resetState();
        return;
    }

    // ─── Gating layer 3: spark must be armed ──────────────────────
    if (!cdi::core::spark::isArmed()) {
        resetState();
        return;
    }

    // ─── Gating layer 4: launch / alvp priority ───────────────────
    // Launch active = drag-start hold, jangan kompound dengan flame.
    // ALVP derated = battery low, jangan boros lagi pakai skip+retard.
    if (cdi::core::launch::isActive() || cdi::core::alvp::isDerated()) {
        resetState();
        return;
    }

    // ─── Gating layer 5: cooldown ─────────────────────────────────
    if ((int32_t)(now - s_cooldownUntilMs) < 0) {
        s_active        = false;
        s_currentRetard = 0.0f;
        s_engageSinceMs = 0;
        s_activeSinceMs = 0;
        return;
    }

    // ─── Gating layer 6: must be at main rev limit ────────────────
    // safety::isRevLimited() = RPM > effective main limit.
    // Tambahan: jangan engage kalau sudah dekat overrev (priority hard cut).
    const uint32_t mainLim   = cdi::core::safety::mainLimitRpm();
    const uint32_t overrev   = cdi::core::safety::overrevLimitRpm();
    const bool     atLimit   = cdi::core::safety::isRevLimited();
    const bool     headroom  = (rpm + OVERREV_HEADROOM) < overrev;

    if (!atLimit || !headroom) {
        // Released — clear engage timer. Don't enter cooldown unless we
        // were active (handled below via RPM-drop check).
        if (s_active && rpm + RPM_DROP_HYST < mainLim) {
            // RPM dropped below limit by hysteresis margin → graceful exit
            Serial.printf("[flame] disengage (rpm dropped to %u)\n", (unsigned)rpm);
        }
        s_active        = false;
        s_currentRetard = 0.0f;
        s_engageSinceMs = 0;
        s_activeSinceMs = 0;
        s_fireCounter   = 0;
        return;
    }

    // ─── At limit — engage delay (sustain) ────────────────────────
    const Params& p = currentParams();

    if (s_engageSinceMs == 0) {
        s_engageSinceMs = now;
    }
    if ((int32_t)(now - s_engageSinceMs) < (int32_t)p.engage_delay_ms) {
        // Still in delay window — not active yet
        s_active        = false;
        s_currentRetard = 0.0f;
        return;
    }

    // ─── Engaged — check duration cap ─────────────────────────────
    if (!s_active) {
        // Edge: just became active
        s_active        = true;
        s_activeSinceMs = now;
        s_fireCounter   = 0;
        Serial.printf("[flame] ENGAGE @ %u rpm (mode=%s, max=%ums)\n",
                      (unsigned)rpm,
                      (s_mode == cdi::FlameMode::AGGRESSIVE) ? "AGGRESSIVE" : "SAFE",
                      (unsigned)p.max_duration_ms);
    }

    if ((int32_t)(now - s_activeSinceMs) >= (int32_t)p.max_duration_ms) {
        // Cap reached → enter cooldown
        s_cooldownUntilMs = now + p.cooldown_ms;
        s_active          = false;
        s_currentRetard   = 0.0f;
        s_engageSinceMs   = 0;
        s_activeSinceMs   = 0;
        s_fireCounter     = 0;
        Serial.printf("[flame] cap hit → cooldown %ums\n", (unsigned)p.cooldown_ms);
        return;
    }

    // ─── Update retard sample (jitter every 10ms) ─────────────────
    if ((int32_t)(now - s_lastSampMs) >= 10) {
        s_currentRetard = jitterRetard(p);
        s_lastSampMs    = now;
    }
}

// ── Spark ISR chain — called dari safety::shouldFire ─────────────
bool shouldFireThisCycle() {
    if (!s_active) return true;

    s_fireCounter++;
    const uint32_t c = s_fireCounter;

    // SAFE: fire, fire, skip → 33% skip rate
    // AGGRESSIVE: fire, skip, skip → 67% skip rate
    if (s_mode == cdi::FlameMode::AGGRESSIVE) {
        return (c % 3u) == 0u;       // fire cycle 0, skip 1 & 2
    }
    return (c % 3u) != 2u;           // fire 0 & 1, skip 2
}

float currentRetardDeg() {
    return s_active ? s_currentRetard : 0.0f;
}

bool isEnabled()      { return s_enabled; }
cdi::FlameMode mode() { return s_mode; }
bool isActive()       { return s_active; }
bool isInCooldown()   { return (int32_t)(millis() - s_cooldownUntilMs) < 0; }

uint16_t activeElapsedMs() {
    if (!s_active) return 0;
    const uint32_t e = millis() - s_activeSinceMs;
    return (e > 65535u) ? 65535u : (uint16_t)e;
}
uint16_t maxDurationMs() { return currentParams().max_duration_ms; }

void setEnabled(bool en) {
    s_enabled = en;
    if (!en) {
        resetState();
        s_cooldownUntilMs = 0;
    }
    Serial.printf("[flame] enabled=%d\n", en ? 1 : 0);
}

void setMode(cdi::FlameMode m) {
    s_mode = m;
    resetState();
    s_cooldownUntilMs = 0;
    Serial.printf("[flame] mode=%d\n", (int)m);
}

} // namespace cdi::core::flame
