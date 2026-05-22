#include "core/safety.h"

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "core/rpm_calc.h"
#include "core/spark_scheduler.h"
#include "core/launch_control.h"
#include "core/alvp.h"

namespace cdi::core::safety {
namespace {

uint32_t s_mainLimit    = cdi::config::DEFAULT_REV_LIMIT_MAIN_RPM;
uint32_t s_overrevLimit = cdi::config::DEFAULT_REV_LIMIT_OVERREV_RPM;

bool s_revLimited        = false;
bool s_noSignal          = false;
bool s_overRevCut        = false;
// No-signal failsafe is OFF by default. Rev limiter + absolute RPM
// ceiling are still active. Enable explicitly via UI when riding.
bool s_noSignalEnabled   = false;

uint32_t s_overrevHits = 0;
constexpr uint32_t OVERREV_CONFIRM = 3;

// Latches once we've seen a genuinely RUNNING engine — defined as
// RPM ≥ RPM_LATCH_THRESHOLD sustained across `VALID_RPM_STREAK_TICKS`
// consecutive 100 ms safety ticks. Threshold is set above kick-start
// transient territory (300-400 RPM peaks during cranking) so failed
// start attempts don't latch and therefore don't engage the no-signal
// failsafe. A real idling engine (≥500 RPM sustained 3 s) does latch.
//
// Reset by clearFlags(). The no-signal failsafe only trips after
// this is true.
constexpr uint16_t RPM_LATCH_THRESHOLD     = 500;  // above kick transient
constexpr uint8_t  VALID_RPM_STREAK_TICKS  = 30;   // 30 × 100 ms = 3 s

bool     s_haveSeenValidRpm  = false;
uint8_t  s_validRpmStreak    = 0;

// Cut-mode configuration (main band only).
volatile cdi::CutMode s_mainCutMode = cdi::CutMode::SOFT_RETARD;
volatile float        s_mainRetardDeg = 10.0f;
volatile uint8_t      s_patternFireN  = 3;
volatile uint8_t      s_patternSkipN  = 1;

// Active state computed by tick() based on RPM band; read by spark ISR.
volatile cdi::CutMode s_activeCutMode = cdi::CutMode::OFF;
volatile float        s_activeRetardDeg = 0.0f;
volatile uint8_t      s_progressivePct  = 0;   // skip probability 0-100

// Pattern + PRNG state.
volatile uint32_t     s_patternCounter = 0;
volatile uint32_t     s_lcgState       = 0xDEADBEEF;

} // anonymous

void begin() {
    // Register the loop task with the task WDT.
    // (esp-idf v4 / arduino-esp32 v2.x signature)
    esp_task_wdt_init(cdi::config::TASK_WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);
    Serial.printf("[safety] task_wdt %u s · main_limit %u · overrev %u\n",
                  (unsigned)cdi::config::TASK_WDT_TIMEOUT_S,
                  (unsigned)s_mainLimit, (unsigned)s_overrevLimit);
}

void tick() {
    esp_task_wdt_reset();

    const uint32_t now_ms = millis();
    const cdi::micros_t now_us = (cdi::micros_t)micros();

    // ─── No-signal failsafe ───
    // Trip ONLY when we previously had VALID engine RPM and then lost
    // it (cable cut mid-ride, pulser failed, etc). Edge-only timestamps
    // can't be trusted as "signal present" — GPIO34/35 are input-only
    // and have NO internal pull-up, so a disconnected pulser pin
    // floating from noise could spuriously update lastCh1Us and
    // trigger a false failsafe at bench.
    //
    // rpm::current() applies sanity bounds (RPM_MIN_VALID..MAX_VALID)
    // and exponential smoothing, so it stays at 0 under noise. Use it
    // as the latch: once we've seen real RPM we know the engine ran;
    // after that, signal staleness == genuine fault.
    {
        cdi::rpm_t rpm_now = cdi::core::rpm::current();
        if (rpm_now >= RPM_LATCH_THRESHOLD) {
            if (s_validRpmStreak < 255) s_validRpmStreak++;
            if (s_validRpmStreak >= VALID_RPM_STREAK_TICKS && !s_haveSeenValidRpm) {
                s_haveSeenValidRpm = true;
                Serial.printf("[safety] engine latched as RUNNING "
                              "(%u rpm sustained %u ticks) — no-signal failsafe now armed\n",
                              (unsigned)rpm_now, (unsigned)s_validRpmStreak);
            }
        } else {
            // RPM dropped below the running threshold. Reset the
            // streak counter so a future re-run has to qualify again
            // from scratch. The latch itself stays set if it was
            // already set; only a fresh arm (clearFlags) drops it
            // again. That way the genuine "engine ran then stalled"
            // case still trips the no-signal failsafe at the next
            // safety tick.
            s_validRpmStreak = 0;
        }
    }
    if (s_noSignalEnabled && cdi::core::spark::isArmed() && s_haveSeenValidRpm) {
        cdi::micros_t last = cdi::core::rpm::lastCh1Us();
        if (last != 0 &&
            (now_us - last) > (cdi::micros_t)cdi::config::NO_SIGNAL_TIMEOUT_MS * 1000ULL) {
            cdi::core::spark::setArmed(false);
            s_noSignal = true;
            Serial.println("[safety] NO-SIGNAL failsafe → auto disarm "
                           "(valid RPM lost after being seen)");
        }
    }

    // ─── Rev limiter ───
    cdi::rpm_t rpm = cdi::core::rpm::current();

    // ─── Absolute RPM ceiling (catches multi-tooth pickup, noise) ───
    // Bypass all configurable limits — this is a "something is broken"
    // condition, not a "rider asked for high revs" one.
    if (rpm > cdi::config::ABSOLUTE_RPM_CEILING) {
        if (cdi::core::spark::isArmed()) {
            cdi::core::spark::setArmed(false);
            Serial.printf("[safety] RPM RUNAWAY %u rpm (> ceiling %u) → DISARM. "
                          "Cek pickup wiring / pilih preset benar (motor FI multi-tooth tidak kompatibel).\n",
                          (unsigned)rpm, (unsigned)cdi::config::ABSOLUTE_RPM_CEILING);
        }
        s_overRevCut = true;
        s_revLimited = true;
        s_activeCutMode   = cdi::CutMode::HARD_CUT;
        s_activeRetardDeg = 0.0f;
        s_progressivePct  = 100;
        return;   // skip the rest of the normal rev-limit ladder
    }

    // Effective limit determined by stacked overrides:
    //   - launch active → launch_rpm + HARD_CUT (highest priority, drag start)
    //   - ALVP derate → derate_rpm + SOFT_RETARD (low-voltage protection)
    //   - else        → configured main_limit + configured cut mode
    uint32_t effective_main = s_mainLimit;
    cdi::CutMode effective_mode = s_mainCutMode;
    bool launch_active = cdi::core::launch::isActive();
    if (cdi::core::alvp::isDerated()) {
        uint32_t lim = cdi::core::alvp::derateLimitRpm();
        if (lim < effective_main) {
            effective_main = lim;
            effective_mode = cdi::CutMode::SOFT_RETARD;
        }
    }
    if (launch_active) {
        effective_main = cdi::core::launch::launchRpm();
        effective_mode = cdi::CutMode::HARD_CUT;
    }

    if (rpm > s_overrevLimit) {
        s_overrevHits++;
        if (s_overrevHits >= OVERREV_CONFIRM && cdi::core::spark::isArmed()) {
            cdi::core::spark::setArmed(false);
            s_overRevCut = true;
            s_revLimited = true;
            Serial.printf("[safety] OVER-REV %u rpm → hard cut\n", (unsigned)rpm);
        }
        s_activeCutMode = cdi::CutMode::HARD_CUT;
        s_activeRetardDeg = 0.0f;
        s_progressivePct  = 100;
    } else if (rpm > effective_main) {
        s_overrevHits = 0;
        if (!s_revLimited) {
            Serial.printf("[safety] %s @ %u rpm · mode=%d\n",
                          launch_active ? "launch-cut" : "rev-limit",
                          (unsigned)rpm, (int)effective_mode);
        }
        s_revLimited = true;
        s_activeCutMode = effective_mode;
        switch (effective_mode) {
            case cdi::CutMode::SOFT_RETARD:
                s_activeRetardDeg = s_mainRetardDeg;
                s_progressivePct  = 0;
                break;
            case cdi::CutMode::HARD_CUT:
                // Force disarm immediately like overrev.
                if (cdi::core::spark::isArmed()) cdi::core::spark::setArmed(false);
                s_activeRetardDeg = 0.0f;
                s_progressivePct  = 100;
                break;
            case cdi::CutMode::PATTERN_CUT:
                s_activeRetardDeg = 0.0f;
                s_progressivePct  = 0;
                break;
            case cdi::CutMode::SPARK_PROGRESSIVE: {
                // skip pct ramps linearly across [main, overrev] band
                uint32_t band = s_overrevLimit - effective_main;
                if (band == 0) band = 1;
                uint32_t excess = rpm - effective_main;
                uint32_t pct = (excess * 100) / band;
                if (pct > 95) pct = 95;
                s_progressivePct = (uint8_t)pct;
                s_activeRetardDeg = 0.0f;
                break;
            }
            default:
                s_activeRetardDeg = 0.0f;
                s_progressivePct  = 0;
                break;
        }
    } else {
        s_overrevHits = 0;
        if (s_revLimited && rpm < effective_main - 200) {
            Serial.println("[safety] rev-limit released");
        }
        if (rpm < effective_main - 200) s_revLimited = false;
        s_activeCutMode   = cdi::CutMode::OFF;
        s_activeRetardDeg = 0.0f;
        s_progressivePct  = 0;
    }
}

void setRevLimits(uint32_t main_rpm, uint32_t overrev_rpm) {
    if (main_rpm < 1000) main_rpm = 1000;
    if (overrev_rpm < main_rpm + 200) overrev_rpm = main_rpm + 200;
    if (overrev_rpm > 20000) overrev_rpm = 20000;
    s_mainLimit    = main_rpm;
    s_overrevLimit = overrev_rpm;
    Serial.printf("[safety] limits updated · main=%u · overrev=%u\n",
                  (unsigned)main_rpm, (unsigned)overrev_rpm);
}

uint32_t mainLimitRpm()    { return s_mainLimit; }
uint32_t overrevLimitRpm() { return s_overrevLimit; }
bool     isRevLimited()    { return s_revLimited; }
bool     noSignal()        { return s_noSignal; }
bool     overRevCut()      { return s_overRevCut; }

void setMainCutMode(cdi::CutMode m) {
    s_mainCutMode = m;
    Serial.printf("[safety] cut mode = %d\n", (int)m);
}
cdi::CutMode mainCutMode() { return s_mainCutMode; }

void setMainRetardDeg(float d) {
    if (d < 0)   d = 0;
    if (d > 30)  d = 30;
    s_mainRetardDeg = d;
}
float mainRetardDeg() { return s_mainRetardDeg; }

void setMainPatternRatio(uint8_t fire_n, uint8_t skip_n) {
    if (fire_n < 1)  fire_n = 1;
    if (fire_n > 15) fire_n = 15;
    if (skip_n < 1)  skip_n = 1;
    if (skip_n > 15) skip_n = 15;
    s_patternFireN = fire_n;
    s_patternSkipN = skip_n;
}
uint8_t patternFireN() { return s_patternFireN; }
uint8_t patternSkipN() { return s_patternSkipN; }

float currentRetardDeg() { return s_activeRetardDeg; }

bool IRAM_ATTR shouldFire() {
    cdi::CutMode m = s_activeCutMode;
    if (m == cdi::CutMode::OFF || m == cdi::CutMode::SOFT_RETARD) return true;
    if (m == cdi::CutMode::HARD_CUT) return false;
    if (m == cdi::CutMode::PATTERN_CUT) {
        uint32_t c = s_patternCounter++;
        uint32_t period = (uint32_t)s_patternFireN + (uint32_t)s_patternSkipN;
        return (c % period) < s_patternFireN;
    }
    if (m == cdi::CutMode::SPARK_PROGRESSIVE) {
        // 32-bit LCG (Numerical Recipes), then mod 100.
        uint32_t s = s_lcgState * 1664525U + 1013904223U;
        s_lcgState = s;
        uint32_t r = (s >> 16) % 100;
        return r >= s_progressivePct;
    }
    return true;
}

void clearFlags() {
    s_noSignal         = false;
    s_overRevCut       = false;
    s_overrevHits      = 0;
    s_haveSeenValidRpm = false;   // fresh arm — wait for real signal again
    s_validRpmStreak   = 0;
}

void setNoSignalEnabled(bool en) {
    s_noSignalEnabled = en;
    if (!en) {
        // Drop sticky flag too so UI immediately reflects disabled state.
        s_noSignal = false;
    }
    Serial.printf("[safety] no-signal failsafe = %s\n", en ? "ENABLED" : "DISABLED");
}
bool noSignalEnabled() { return s_noSignalEnabled; }

} // namespace cdi::core::safety
