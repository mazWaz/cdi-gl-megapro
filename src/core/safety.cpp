#include "core/safety.h"

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "core/rpm_calc.h"
#include "core/spark_scheduler.h"
#include "core/launch_control.h"
#include "core/alvp.h"
#include "core/idle_rumble.h"
#include "core/exhaust_flame.h"

namespace cdi::core::safety {
namespace {

// Cross-core access: setRevLimits called from WS handlers (core 0)
// writes these; safety::tick on core 1 reads. Word-aligned 32-bit
// writes are atomic on Xtensa, but `volatile` is needed so the
// compiler does not cache stale values in a register past a
// context switch / function boundary.
volatile uint32_t s_mainLimit    = cdi::config::DEFAULT_REV_LIMIT_MAIN_RPM;
volatile uint32_t s_overrevLimit = cdi::config::DEFAULT_REV_LIMIT_OVERREV_RPM;

volatile bool s_revLimited        = false;
volatile bool s_overRevCut        = false;
// ALVP under-voltage full cut — SELF-RECOVERING (pulse-cut while vbat is
// below the disarm threshold; resumes automatically on recovery). Set by
// tick(), read by the spark ISR via shouldFire(). Never touches `armed`.
volatile bool s_alvpHardCut       = false;

uint32_t s_overrevHits = 0;
constexpr uint32_t OVERREV_CONFIRM = 3;

// (No-signal failsafe removed: after it was made self-recovering it was
// purely vestigial — no CH1 already means no spark, so a lost pickup
// signal stops firing on its own and resumes when it returns, with no
// disarm and no UI action. The RPM-latch machinery only fed it.)

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

// SOFT_RETARD hybrid-escalation state.
// SOFT_RETARD by itself (a 10° retard, all sparks still fire) does
// not arrest RPM on a carbureted engine under sustained throttle —
// power only drops ~5-10 %, not enough to overcome inertia + airflow.
// To preserve the smooth feel of SOFT_RETARD at first touch while
// still guaranteeing arrest, we escalate to PATTERN_CUT (which DOES
// skip sparks) if either:
//   (a) RPM keeps climbing > 100 rpm above entry-into-limit, or
//   (b) we've been stuck in the limiter for > SOFT_ESCALATE_MS.
// Reset on exit from rev-limit so the next over-rev event starts
// over from soft.
// Tightened for MotoGP-style crisp engagement: 400 ms / +50 rpm.
// Old 1500 ms / +100 rpm gave the engine over a second to climb
// past the limit before pattern-cut took over — audible as a soft
// "swell" past the stated limit instead of a hard "fffft".
constexpr uint32_t SOFT_ESCALATE_MS         = 400;
constexpr uint16_t SOFT_ESCALATE_RPM_RISE   = 50;
volatile uint32_t  s_softEntryMs   = 0;     // 0 = not in soft window
volatile cdi::rpm_t s_softEntryRpm = 0;
volatile bool      s_softEscalated = false;

} // anonymous

void begin() {
    // Register the loop task with the task WDT.
    // (esp-idf v4 / arduino-esp32 v2.x signature)
    esp_task_wdt_init(cdi::config::TASK_WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);   // monitor the loop task (firmware liveness)

    // Stop the Task WDT from monitoring the FreeRTOS IDLE tasks.
    //
    // By default (Arduino sdkconfig CHECK_IDLE_TASK) the WDT also
    // watches IDLE0/IDLE1. But bursty WiFi/lwIP/AsyncTCP activity on
    // core 0 (captive-portal DNS polling, client traffic) can keep the
    // core-0 network tasks busy long enough to starve IDLE0 — which
    // tripped the WDT and rebooted the board in a tight loop (backtrace:
    // prvIdleTask / esp_vApplicationIdleHook, abort on core 0), even
    // though the engine loop itself was perfectly healthy. Rebooting the
    // ignition for a transient network-stack hiccup is the wrong trade.
    // We keep the loop-task WDT — the only liveness signal that actually
    // matters for spark safety — and stop watching idle.
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    TaskHandle_t idle1 = xTaskGetIdleTaskHandleForCPU(1);
    if (idle0) esp_task_wdt_delete(idle0);
    if (idle1) esp_task_wdt_delete(idle1);
    Serial.printf("[safety] task_wdt %u s · main_limit %u · overrev %u\n",
                  (unsigned)cdi::config::TASK_WDT_TIMEOUT_S,
                  (unsigned)s_mainLimit, (unsigned)s_overrevLimit);
}

void tick() {
    esp_task_wdt_reset();

    const uint32_t now_ms = millis();

    // ALVP under-voltage full cut (self-recovering, NOT a disarm). While
    // vbat sits in DISARM_LOW we pulse-cut every spark via shouldFire();
    // firing resumes automatically once alvp leaves DISARM_LOW (hysteresis).
    s_alvpHardCut = cdi::core::alvp::isEnabled() &&
                    (cdi::core::alvp::state() == cdi::core::alvp::State::DISARM_LOW);

    // ─── Rev limiter ───
    // Use INSTANTANEOUS RPM (from latest period), not EMA-smoothed.
    // At high RPM and rapid acceleration the smoothed value lags 5-8
    // samples behind reality — a real overrev could exceed the limit
    // by 200-500 rpm before the smoothed reading catches up. For an
    // engine-protection limiter that's the wrong direction to err.
    cdi::rpm_t rpm;
    {
        const cdi::micros_t period = cdi::core::rpm::lastPeriodUs();
        if (period > 0) {
            uint32_t inst = (uint32_t)(60000000ULL / period);
            if (inst > 65535) inst = 65535;
            rpm = (cdi::rpm_t)inst;
        } else {
            rpm = cdi::core::rpm::current();   // fallback before first period
        }
    }

    // Smoothed RPM — confirmation gate for the MAIN rev-limit band.
    //
    // A single noisy CH1 period during the start-up catch (EMI from the
    // first combustion event, a short/double-triggered pulse) makes the
    // INSTANTANEOUS rpm momentarily read several thousand — we measured
    // ~11000 phantom rpm on a real KLX crank where the true speed was
    // <1600. With a LOW main limit (e.g. 6000) that phantom trips the
    // limiter and cuts spark exactly when the engine is trying to fire,
    // so the bike won't start — yet a high limit (11000) starts fine
    // because the phantom stays under it. The EMA-smoothed rpm cannot
    // physically reach a multi-thousand limit while cranking, so we
    // require it to ALSO confirm before engaging the main band.
    //
    // Overrev protection (below) deliberately keeps using instantaneous
    // for fast response — that's the engine-protection limit, and its
    // 3-sample confirm already rejects lone spikes before disarming.
    const cdi::rpm_t rpm_smooth = cdi::core::rpm::current();

    // (Absolute RPM ceiling removed — it was a dead + redundant guard.
    // ABSOLUTE_RPM_CEILING equalled RPM_MAX_VALID, so rpm_calc's period
    // clamp meant `rpm` could never exceed it (it never fired). Its
    // intended job is already covered: genuine over-rev by the self-
    // recovering overrev cut below; a broken/multi-tooth pickup by
    // rpm_calc rejecting the short phantom periods → no valid RPM → the
    // engine simply doesn't fire on un-trustable signal. Over-rev is a
    // self-recovering pulse-cut, not a sticky disarm.)

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
        // Hard ceiling — 100 % spark cut to arrest RPM, but
        // SELF-RECOVERING (pulse-cut, NOT a sticky disarm). The engine
        // bounces off the limiter like a real rev-limiter / MotoGP hard
        // cut and resumes the instant RPM drops back under the ceiling —
        // no WiFi re-arm needed.
        //
        // Why not disarm: a sticky disarm here stalls a running engine
        // mid-ride (dangerous in a corner / intersection) and forces a
        // manual re-arm. Hitting overrev is a transient event (aggressive
        // throttle, downshift, a phantom RPM blip) — not a reason to kill
        // ignition. The only sticky kills left are deliberate (panic
        // button → SAFE_HOLD) or genuinely incompatible hardware
        // (multi-tooth pickup → SAFE_HOLD during calibration). Pulse-cut
        // keeps the coil de-energized above the limit but recovers
        // automatically — just as safe as a disarm, far safer mid-ride.
        s_overrevHits++;   // kept for telemetry/diagnostics
        s_overRevCut = true;
        s_revLimited = true;
        if (s_overrevHits == OVERREV_CONFIRM) {
            Serial.printf("[safety] OVER-REV %u rpm → hard cut (self-recovering)\n",
                          (unsigned)rpm);
        }
        s_activeCutMode = cdi::CutMode::HARD_CUT;
        s_activeRetardDeg = 0.0f;
        s_progressivePct  = 100;
    } else if (rpm > effective_main && rpm_smooth > effective_main) {
        s_overrevHits = 0;
        s_overRevCut  = false;   // below overrev — clear transient flag
        if (!s_revLimited) {
            Serial.printf("[safety] %s @ %u rpm (smooth %u) · mode=%d\n",
                          launch_active ? "launch-cut" : "rev-limit",
                          (unsigned)rpm, (unsigned)rpm_smooth, (int)effective_mode);
        }
        s_revLimited = true;
        s_activeCutMode = effective_mode;
        switch (effective_mode) {
            case cdi::CutMode::SOFT_RETARD: {
                // First entry into the soft-retard window — latch
                // the start time + RPM so we can detect "still
                // climbing" or "stuck too long" on later ticks.
                if (s_softEntryMs == 0) {
                    s_softEntryMs   = now_ms;
                    s_softEntryRpm  = rpm;
                    s_softEscalated = false;
                }
                const bool still_climbing =
                    rpm > (cdi::rpm_t)(s_softEntryRpm + SOFT_ESCALATE_RPM_RISE);
                const bool too_long =
                    (now_ms - s_softEntryMs) > SOFT_ESCALATE_MS;
                if (still_climbing || too_long) {
                    if (!s_softEscalated) {
                        s_softEscalated = true;
                        Serial.printf("[safety] SOFT_RETARD escalated → PATTERN_CUT "
                                      "(%s, rpm=%u, entry=%u)\n",
                                      still_climbing ? "climbing" : "timeout",
                                      (unsigned)rpm, (unsigned)s_softEntryRpm);
                    }
                    s_activeCutMode   = cdi::CutMode::PATTERN_CUT;
                    s_activeRetardDeg = 0.0f;
                    s_progressivePct  = 0;
                } else {
                    s_activeRetardDeg = s_mainRetardDeg;
                    s_progressivePct  = 0;
                }
                break;
            }
            case cdi::CutMode::HARD_CUT:
                // PULSE-cut (shouldFire returns false) — do NOT
                // setArmed(false). For 2-step launch the rider holds
                // the clutch, expects spark to RESUME when rpm drops
                // back below launch_rpm. Permanent disarm here would
                // kill the engine after one launch attempt.
                // OVER-REV path above is the only place a sticky
                // disarm is appropriate (genuine fault, not a feature).
                s_activeRetardDeg = 0.0f;
                s_progressivePct  = 100;
                break;
            case cdi::CutMode::PATTERN_CUT:
                s_activeRetardDeg = 0.0f;
                s_progressivePct  = 0;
                break;
            case cdi::CutMode::SPARK_PROGRESSIVE: {
                // MotoGP-style tight ramp: cap the ramp band at
                // MAX_PROGRESSIVE_BAND rpm so the limiter "nails"
                // the main limit even when overrev is far away.
                //
                // Before this cap: with main=6000 / overrev=11500 the
                // 5500-rpm band gave only 1.8 % skip at 100 rpm over —
                // engine could climb 2000-3000 rpm before cut bit.
                // With the cap: 0 % at main, ~95 % within 300 rpm of
                // main, then HARD_CUT above. Audible as a flat
                // "fffft" at exactly the stated limit.
                constexpr uint32_t MAX_PROGRESSIVE_BAND = 300;
                uint32_t band = s_overrevLimit - effective_main;
                if (band > MAX_PROGRESSIVE_BAND) band = MAX_PROGRESSIVE_BAND;
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
        s_overRevCut  = false;   // below limits — clear transient flag
        // MotoGP-style crisp release: drop from 200 rpm to 75 rpm
        // so the limiter bounces tight against main instead of
        // releasing into a wide hysteresis valley. Combined with
        // the 300-rpm progressive band, this produces the rapid
        // engage/release cadence riders hear as a hard rev-limit.
        constexpr uint16_t REV_RELEASE_HYSTERESIS = 75;
        if (s_revLimited && rpm < effective_main - REV_RELEASE_HYSTERESIS) {
            Serial.println("[safety] rev-limit released");
        }
        if (rpm < effective_main - REV_RELEASE_HYSTERESIS) s_revLimited = false;
        s_activeCutMode   = cdi::CutMode::OFF;
        s_activeRetardDeg = 0.0f;
        s_progressivePct  = 0;
        // Out of rev-limit window — clear soft-retard escalation
        // memory so the next over-rev event starts fresh in SOFT.
        s_softEntryMs   = 0;
        s_softEscalated = false;
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
    // ALVP under-voltage: true 100% pulse-cut while below disarm voltage
    // (self-recovering — `armed` is untouched, firing resumes when vbat
    // recovers). Highest priority.
    if (s_alvpHardCut) return false;

    // Idle rumble skip-fire pattern (cuma aktif kalau engine sustained
    // di idle band + mode AGGRESSIVE/DRAG_BURBLE). Cek dulu sebelum
    // cut-mode logic supaya tidak kompound dengan rev-limit cut.
    if (!cdi::core::idle_rumble::shouldFireThisCycle()) return false;

    // Exhaust flame skip-fire pattern (aktif saat sustained di rev
    // limiter + flame mode enabled). Skip jalan PARALEL dengan rev-limit
    // cut yang sudah ada — flame engage hanya kalau safety::isRevLimited
    // true, jadi cut mode SOFT_RETARD/PATTERN_CUT/HARD_CUT juga aktif.
    // shouldFireThisCycle() flame return false → skip. Kalau true,
    // jatuh ke cut-mode logic di bawah (yang juga bisa decide skip).
    // Net effect: kalau salah satu return false, fire diskip. Aman.
    if (!cdi::core::flame::shouldFireThisCycle()) return false;

    cdi::CutMode m = s_activeCutMode;
    if (m == cdi::CutMode::OFF || m == cdi::CutMode::SOFT_RETARD) return true;
    if (m == cdi::CutMode::HARD_CUT) {
        // HARD_CUT burn-through floor.
        //
        // On a carbureted engine, 100 %-skip HARD_CUT lets raw fuel
        // accumulate in the exhaust manifold every cycle — when spark
        // resumes, the residual ignites in the header pipe ("POP!").
        // Sustained limiter use on the street can crack header welds
        // and is loud enough to spook other riders.
        //
        // For the safety-critical paths (genuine over-rev fault, or
        // explicit launch-control plateau) the rider expects a TRUE
        // 100 % kill — those bypass the floor. For the daily main
        // rev-limit case, we leak ~8 % of sparks. That's far below
        // what could sustain acceleration (RPM still nails the limit),
        // but enough to consume residual fuel before it builds up to
        // backfire energy.
        //
        // MotoGP equivalent: factory ECUs fuel-cut instead of spark-
        // cut. We can't (no injector control), so we simulate the
        // "spark-fires, no fuel" outcome by inverting it: most-sparks-
        // skipped, fuel still arrives.
        if (s_overRevCut || cdi::core::launch::isActive()) return false;
        constexpr uint32_t HARD_CUT_BURN_PCT = 8;
        uint32_t s = s_lcgState * 1664525U + 1013904223U;
        s_lcgState = s;
        return ((s >> 16) % 100) < HARD_CUT_BURN_PCT;
    }
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
    s_overRevCut       = false;
    s_overrevHits      = 0;
}

bool flashWriteSafe() {
    // Disarmed → no live spark path to disturb.
    if (!cdi::core::spark::isArmed()) return true;
    // Armed + engine turning → a flash stall could strand a live dwell.
    if (cdi::core::rpm::current() > 0) return false;
    // The EMA can read 0 on the first crank pulses before it settles, so
    // also treat a very recent CH1 edge as "still turning". 32-bit
    // modular diff (micros() wraps every ~71 min — see rpm_calc.cpp).
    const cdi::micros_t last = cdi::core::rpm::lastCh1Us();
    if (last == 0) return true;                  // never seen a pulse
    const uint32_t gap = (uint32_t)micros() - (uint32_t)last;
    return gap > 600000UL;                       // quiet ≥600 ms → quiescent
}

} // namespace cdi::core::safety
