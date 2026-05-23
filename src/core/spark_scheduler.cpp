#include "core/spark_scheduler.h"

#include <Arduino.h>

#include "config.h"
#include "pinmap.h"
#include "core/safety.h"
#include "core/quickshifter.h"

namespace cdi::core::spark {
namespace {

// Two dedicated hardware timers — independent of scope's timer (group 0,
// idx 0). Fire-on uses timer 1, fire-off uses timer 2.
hw_timer_t* s_fireOnTimer  = nullptr;
hw_timer_t* s_fireOffTimer = nullptr;

volatile bool     s_armed         = false;
volatile bool     s_autoArm       = false;
volatile bool     s_activeLow     = false;   // false=active-HIGH (default)
// (s_inductive removed — hardware fixed TCI, inductive() returns true const)
volatile uint32_t s_nextDelayUs   = 0;      // cached, updated by loop
// True only after live_stats has computed a delay from at least one
// valid period. Distinguishes "delay = 0 by computation (legitimate
// max-advance case)" from "delay = 0 because no computation has run
// yet (uninitialized)". Without this flag, the second CH1 ever in a
// session reads s_nextDelayUs == 0 and takes the direct-fire branch
// → spark lands at t_lead = 32° BTDC at cranking RPM → kickback.
// Cleared on long-stall re-anchor so a stalled-then-restarted engine
// also re-primes before firing.
volatile bool     s_delayPrimed   = false;
// Period that live_stats used to compute s_nextDelayUs. The ISR
// compares its measured inst_period against this and rejects the
// fire if they differ by more than 2× — meaning RPM swung enough
// in one cycle that the cached delay is meaningfully wrong for the
// current crank position. This matters during kickstart (where
// CH1-to-CH1 RPM can drop 825 → 33 between strokes) and during
// hard decel. Real-world data: kick CSV showed period 72ms → 1.8s
// in one cycle, which would have fired at 32° BTDC at 33 rpm with
// a delay computed for 825 rpm.
volatile uint32_t s_delayBasisPeriodUs = 0;
volatile uint32_t s_dwellUs            = cdi::config::DEFAULT_DWELL_US;  // configured (user)
volatile uint32_t s_effectiveDwellUs   = cdi::config::DEFAULT_DWELL_US;  // ISR-side actual
volatile float    s_advanceOffsetDeg = 0.0f;

volatile uint32_t s_fireCount     = 0;
volatile int32_t  s_lastJitterUs  = 0;
volatile cdi::micros_t s_scheduledFireUs = 0;
volatile bool     s_dwellInProgress = false;   // GPIO is in active (charging) state

constexpr uint32_t MIN_DELAY_US = 50;        // safety floor
// Ceiling must accommodate cranking RPM. At 30 rpm (period 2 s) a
// 30° delay = 167 ms, well above the old 50 ms cap that was clamping
// every cranking-speed fire to the wrong timing. 500 ms is large
// enough for 20 rpm cranking at full magnet_width delay.
constexpr uint32_t MAX_DELAY_US = 500000;    // 500 ms ceiling

inline void gpioHigh(uint8_t pin) { GPIO.out_w1ts = (1U << pin); }
inline void gpioLow(uint8_t pin)  { GPIO.out_w1tc = (1U << pin); }

// Logical "fire active" / "fire idle" — flipped by s_activeLow.
inline void IRAM_ATTR sparkActive(uint8_t pin) {
    if (s_activeLow) gpioLow(pin); else gpioHigh(pin);
}
inline void IRAM_ATTR sparkIdle(uint8_t pin) {
    if (s_activeLow) gpioHigh(pin); else gpioLow(pin);
}

void IRAM_ATTR isrFireOn() {
    cdi::micros_t now = (cdi::micros_t)micros();
    s_lastJitterUs = (int32_t)(now - s_scheduledFireUs);

    // Start charge phase. ACTIVE-HIGH: GPIO25 → 3V3, MOSFET ON.
    // ACTIVE-LOW:  GPIO25 → 0V,  PNP/PMOS ON.
    sparkActive(cdi::pins::SPARK_OUT);
    gpioHigh(cdi::pins::MODE_LED);    // visible LED stays active-HIGH
    s_dwellInProgress = true;

    // Arm fire-off timer for now + dwell.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOffTimer, s_effectiveDwellUs, false, 0);
#else
    timerAlarmDisable(s_fireOffTimer);
    timerRestart(s_fireOffTimer);
    timerAlarmWrite(s_fireOffTimer, s_effectiveDwellUs, false);
    timerAlarmEnable(s_fireOffTimer);
#endif

    // Stop fire-on timer to prevent retrigger.
#if ESP_ARDUINO_VERSION_MAJOR < 3
    timerAlarmDisable(s_fireOnTimer);
#endif
}

void IRAM_ATTR isrFireOff() {
    // End charge phase → coil collapse → spark on this edge.
    sparkIdle(cdi::pins::SPARK_OUT);
    gpioLow(cdi::pins::MODE_LED);
    s_dwellInProgress = false;
    s_fireCount++;

#if ESP_ARDUINO_VERSION_MAJOR < 3
    timerAlarmDisable(s_fireOffTimer);
#endif
}

} // anonymous

void begin() {
    pinMode(cdi::pins::SPARK_OUT, OUTPUT);
    pinMode(cdi::pins::MODE_LED,  OUTPUT);
    sparkIdle(cdi::pins::SPARK_OUT);
    gpioLow(cdi::pins::MODE_LED);

    if (!s_fireOnTimer) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        s_fireOnTimer = timerBegin(1000000);
        timerAttachInterrupt(s_fireOnTimer, &isrFireOn);
#else
        s_fireOnTimer = timerBegin(1, 80, true);
        timerAttachInterrupt(s_fireOnTimer, &isrFireOn, true);
#endif
    }
    if (!s_fireOffTimer) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        s_fireOffTimer = timerBegin(1000000);
        timerAttachInterrupt(s_fireOffTimer, &isrFireOff);
#else
        s_fireOffTimer = timerBegin(2, 80, true);
        timerAttachInterrupt(s_fireOffTimer, &isrFireOff, true);
#endif
    }

    Serial.println("[spark] scheduler ready · disarmed");
}

void end() {
    forceLow();
    s_armed = false;
}

bool setArmed(bool a) {
    if (!a) {
        forceLow();
        // Force re-prime on next arm cycle. If the user disarms
        // mid-ride and re-arms while the engine is cranking, we
        // want the same cold-start safety (no fire until a fresh
        // delay computation lands) rather than firing on whatever
        // stale value s_nextDelayUs holds from before disarm.
        // s_lastCh1IsrTs is reset by the ISR's own long-stall
        // branch the next time a CH1 arrives — the s_delayPrimed
        // gate below is sufficient to block any wrong-angle fire
        // in the meantime.
        s_delayPrimed = false;
    }
    s_armed = a;
    Serial.printf("[spark] armed = %s\n", a ? "TRUE" : "FALSE");
    return s_armed;
}
bool isArmed() { return s_armed; }

void setAutoArm(bool en) {
    s_autoArm = en;
    Serial.printf("[spark] auto-arm = %d\n", en ? 1 : 0);
}
bool autoArm() { return s_autoArm; }

void setNextDelayUs(uint32_t d, uint32_t basis_period_us) {
    if (d < MIN_DELAY_US) d = MIN_DELAY_US;
    if (d > MAX_DELAY_US) d = MAX_DELAY_US;
    s_nextDelayUs        = d;
    s_delayBasisPeriodUs = basis_period_us;
    // Live_stats called us, which means it has a valid period. From
    // here on the ISR is allowed to fire on s_nextDelayUs.
    s_delayPrimed        = true;
}

void setDwellUs(uint32_t d) {
    if (d < 500)  d = 500;
    if (d > 8000) d = 8000;
    s_dwellUs = d;
    // Effective tracks configured when no live cap is active.
    s_effectiveDwellUs = d;
}
uint32_t dwellUs()           { return s_dwellUs; }
uint32_t configuredDwellUs() { return s_dwellUs; }

void setEffectiveDwellUs(uint32_t d) {
    // Floor at 200µs (below which inductive saturation is too weak).
    if (d < 200) d = 200;
    // Absolute hardware ceiling matching setDwellUs() — protects the
    // coil regardless of caller intent. Previously this clamped to
    // s_dwellUs (the user-configured value) on the principle that
    // live caps should only shorten, never extend. That principle
    // silently nullified ALVP's dwell-boost: at low supply voltage
    // the system needs to extend dwell beyond configured to keep
    // spark energy above misfire floor, exactly the opposite of
    // "only shorten." live_stats already enforces the real safety
    // bounds upstream (40 % period thermal cap, advance-budget cap,
    // useful-spark floor), so the absolute ceiling here is the only
    // remaining backstop.
    if (d > 8000) d = 8000;
    s_effectiveDwellUs = d;
}
uint32_t effectiveDwellUs() { return s_effectiveDwellUs; }

void setAdvanceOffsetDeg(float deg) {
    if (deg < -10.0f) deg = -10.0f;
    if (deg >  10.0f) deg =  10.0f;
    s_advanceOffsetDeg = deg;
    Serial.printf("[spark] advance offset = %.2f deg\n", deg);
}
float advanceOffsetDeg() { return s_advanceOffsetDeg; }

void manualFire(uint32_t dwell_override_us) {
    // Optional dwell override for bench diagnostic — temporarily
    // swap s_dwellUs, schedule fire, restore after the fire-off ISR
    // would have completed. (Worst-case dwell + safety margin.)
    const uint32_t saved_effective = s_effectiveDwellUs;
    if (dwell_override_us > 0) {
        uint32_t d = dwell_override_us;
        if (d < 500)    d = 500;
        if (d > 20000)  d = 20000;     // allow up to 20 ms for diag
        s_effectiveDwellUs = d;
        Serial.printf("[spark] manual test fire (override dwell=%u us)\n", (unsigned)d);
    } else {
        // Use the same effective value the running pulser path uses.
        Serial.printf("[spark] manual test fire (dwell=%u us)\n", (unsigned)s_effectiveDwellUs);
    }

    s_scheduledFireUs = (cdi::micros_t)micros() + 100;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOnTimer, 100, false, 0);
#else
    timerAlarmDisable(s_fireOnTimer);
    timerRestart(s_fireOnTimer);
    timerAlarmWrite(s_fireOnTimer, 100, false);
    timerAlarmEnable(s_fireOnTimer);
#endif

    if (dwell_override_us > 0) {
        delay((dwell_override_us / 1000) + 5);
        s_effectiveDwellUs = saved_effective;
    }
}

// ISR-side absolute RPM ceiling — same value as safety::tick guard,
// but checked in ISR context for per-fire protection (safety::tick
// runs every 100 ms; at 13000 rpm that's ~22 cycles of latency
// before disarm). Period below this floor → RPM above ceiling →
// refuse to schedule.
//
// Lowered 16k → 13k after live engine log showed EMI bursts from
// coil firing producing spurious CH1 events at 4000-4500 µs periods
// (= 13-15k phantom RPM). 13000 rpm = 4615 µs threshold rejects
// those bursts cleanly while leaving 13 % margin above realistic
// commuter-4T mechanical ceiling (~11500 rpm overrev cut).
constexpr uint32_t MIN_PERIOD_HARD_US = 60000000UL / 13000;  // 13000 rpm ceiling
// 2-second period = ~30 rpm. If the gap since the last CH1 exceeds
// this, the engine has either stalled or never started, and any
// cached s_nextDelayUs from a previous run is meaningless. Treat
// the new edge as a fresh session: re-anchor s_lastCh1IsrTs and
// skip this fire (live_stats will compute a valid delay on the
// next cycle once a fresh period is available).
constexpr uint32_t MAX_PERIOD_HARD_US = 2000000UL;

static cdi::micros_t s_lastCh1IsrTs = 0;   // for ISR-local period check

void IRAM_ATTR onPulseCh1FromIsr(cdi::micros_t t_lead) {
    if (!s_armed) return;
    // Quickshifter active cut window — skip this fire entirely.
    if (cdi::core::quickshift::shouldCut()) return;
    // Cut-mode gate: pattern / progressive skip this cycle.
    if (!cdi::core::safety::shouldFire()) return;

    // ─── ISR-level absolute RPM ceiling guard ───
    // Tighter than safety::tick (100 ms cadence). Refuses to even
    // schedule a fire when the instantaneous period suggests we're
    // above the absolute ceiling — catches noise-induced or
    // mechanically-impossible RPM spikes before they ever drive
    // GPIO25 or charge the primary.
    // ─── First-pulse-after-arm safe skip ───
    // s_lastCh1IsrTs == 0 means this is the very first CH1 since
    // begin() or since the engine fully stopped (no period info
    // available, and s_nextDelayUs is whatever stale value loop
    // last computed — possibly 0). Firing now would land at
    // delay=0 → fire-on at t_lead = 32° BTDC at cranking RPM, a
    // textbook kickback geometry. Skip this one fire, record the
    // timestamp, and let the next CH1 fire with a known period.
    // One missed spark on first crank is invisible; a 32° fire at
    // 200 rpm during compression can break the user's ankle.
    if (s_lastCh1IsrTs == 0) {
        s_lastCh1IsrTs = t_lead;
        return;
    }
    // 32-bit modular subtraction — wraps correctly when the
    // underlying Arduino micros() counter rolls over every
    // ~71 minutes. See rpm_calc.cpp for the long-form explanation.
    const uint32_t inst_period = (uint32_t)t_lead - (uint32_t)s_lastCh1IsrTs;
    if (inst_period < MIN_PERIOD_HARD_US) {
        // RPM exceeds ceiling — skip this fire. Don't update
        // s_lastCh1IsrTs so the NEXT real CH1 (longer period
        // from us) reads correctly.
        return;
    }
    if (inst_period > MAX_PERIOD_HARD_US) {
        // Engine restarted after a long pause (stall / kickstart
        // pause / power-cycle of mechanical system but not the
        // ESP). Cached s_nextDelayUs reflects a previous RPM
        // regime — applying it now would fire at a wrong angle.
        // Re-anchor, force re-prime, and skip this fire; the next
        // live_stats iteration will set s_nextDelayUs based on the
        // new period and only then will the ISR be allowed to fire.
        s_lastCh1IsrTs = t_lead;
        s_delayPrimed  = false;
        return;
    }
    s_lastCh1IsrTs = t_lead;

    // ─── No fire until live_stats has primed a valid delay ───
    // setNextDelayUs() is only called when periodU > 0 in live_stats
    // (i.e. rpm_calc has computed at least one valid period). On a
    // fresh cold-crank that takes 2 CH1 events (1st = no period,
    // 2nd = first period). Without this gate the 2nd CH1 reads
    // s_nextDelayUs == 0 (the cold-init value) and the delay==0
    // branch below fires the coil at t_lead = 32° BTDC. At cranking
    // RPM that's a textbook kickback geometry. Cost of the gate is
    // one extra missed spark at startup; the saved cost is the
    // user's ankle.
    if (!s_delayPrimed) {
        return;
    }

    // ─── Period-drift gate ───
    // Analysis of real kick-crank captures (.temp/data_cek.csv)
    // showed period swinging 72 ms → 1.8 s between consecutive CH1
    // falls during kickstart. Even within the "valid" cranking
    // window (< MAX_PERIOD_HARD_US), an RPM swing of >2× in one
    // cycle means s_nextDelayUs was computed against a now-very-
    // wrong period — applying that delay to the new period places
    // the fire at an arbitrary crank angle. Skipping this fire and
    // letting live_stats recompute on the next iteration costs one
    // missed spark; firing on stale delay risks kickback when the
    // user gets the busi reinstalled.
    //
    // Guard against divide-by-zero / first-prime case where the
    // basis hasn't been written yet (still 0 from cold init).
    if (s_delayBasisPeriodUs > 0) {
        // ratio = inst_period / basis × 100, integer math
        // 50 < ratio < 200 ⇒ within 2× either direction
        const uint64_t ratio_x100 =
            (uint64_t)inst_period * 100ULL / (uint64_t)s_delayBasisPeriodUs;
        if (ratio_x100 < 50 || ratio_x100 > 200) {
            // Force re-prime so live_stats computes a fresh delay
            // for the new period before any further fire.
            s_delayPrimed = false;
            return;
        }
    }

    uint32_t delay = s_nextDelayUs;
    // 0 is a legitimate value — fire-on must happen right at CH1
    // (used when spark_delay ≤ dwell on TCI). Cap at MIN_DELAY_US
    // only as a hardware-timer minimum, not as a "skip this fire"
    // trigger. If delay == 0, fire-on directly here without the
    // timer round-trip.
    // ─── Skip if a previous cycle's dwell is still in progress ───
    // Forcing GPIO to idle here would cause an unintended fall edge
    // (= spark fires for TCI) at the wrong crank angle. Instead we
    // SKIP this fire entirely and let the previous fire-off complete
    // naturally. Engine sees a misfire on this cycle but no chaotic
    // timing — far less harmful than wrong-angle fire that could
    // detonate or burn valves.
    //
    // This should only happen when configured dwell > 40% of period
    // (despite the auto-cap in live_stats). Rare in normal use.
    if (s_dwellInProgress) {
        return;
    }

    if (delay == 0) {
        // Fire-on must happen RIGHT at CH1 (used when spark_delay
        // ≤ dwell on TCI at high advance). Drive GPIO directly and
        // arm fire-off timer for the dwell duration.
        cdi::micros_t now = (cdi::micros_t)micros();
        s_lastJitterUs = (int32_t)(now - t_lead);
        s_scheduledFireUs = t_lead;
        sparkActive(cdi::pins::SPARK_OUT);
        gpioHigh(cdi::pins::MODE_LED);
        s_dwellInProgress = true;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        timerAlarm(s_fireOffTimer, s_effectiveDwellUs, false, 0);
#else
        timerAlarmDisable(s_fireOffTimer);
        timerRestart(s_fireOffTimer);
        timerAlarmWrite(s_fireOffTimer, s_effectiveDwellUs, false);
        timerAlarmEnable(s_fireOffTimer);
#endif
        return;
    }
    if (delay < MIN_DELAY_US) delay = MIN_DELAY_US;

    s_scheduledFireUs = t_lead + delay;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOnTimer, delay, false, 0);
#else
    timerAlarmDisable(s_fireOnTimer);
    timerRestart(s_fireOnTimer);
    timerAlarmWrite(s_fireOnTimer, delay, false);
    timerAlarmEnable(s_fireOnTimer);
#endif
}

uint32_t totalFires()    { return s_fireCount; }
int32_t  lastJitterUs()  { return s_lastJitterUs; }

void setActiveLow(bool en) {
    s_activeLow = en;
    // Re-drive pin to the new idle state immediately so the line
    // doesn't sit in the wrong logical state between calls.
    sparkIdle(cdi::pins::SPARK_OUT);
    Serial.printf("[spark] polarity = ACTIVE-%s\n", en ? "LOW" : "HIGH");
}
bool activeLow() { return s_activeLow; }

// Hardware in this build is N-MOSFET TCI direct gate drive — fixed
// inductive topology. setInductive is kept as a no-op so legacy
// callers (config_store.load for old NVS blobs) still link. Any
// attempt to switch to capacitive/CDI is silently rejected; UI no
// longer exposes the toggle.
void setInductive(bool /*en*/) {
    // intentionally no-op
}
bool inductive() { return true; }

void forceLow() {
    // ─── TCI mid-dwell safety: do NOT force GPIO idle right now ───
    //
    // On inductive (TCI) ignition the HIGH→LOW transition on the
    // spark output is exactly what fires the spark — the coil
    // collapses on that edge. If forceLow runs in the middle of a
    // dwell (caller path: setArmed(false), reached via panic
    // button, overrev hard cut, ALVP DISARM_LOW, manual disarm,
    // SAFE_HOLD mode switch...), driving the pin idle right now
    // would fire the spark at the wrong crank angle — typically
    // somewhere arbitrary between t_lead and t_lead + dwell. On a
    // high-compression engine that wrong-angle spark near TDC can
    // detonate.
    //
    // Safe path: disable the fire-on timer so NO further dwell
    // cycles start, but leave GPIO and fire-off timer alone. The
    // already-armed fire-off ISR will land at its correctly-
    // computed angle, fire one final spark at the intended advance
    // for THIS cycle, then s_armed=false on the next CH1 prevents
    // any new fire. One correctly-timed parting spark is benign;
    // an arbitrary-angle interruption is not.
    //
    // Hardware always TCI in this build (s_inductive removed).
    if (s_dwellInProgress) {
#if ESP_ARDUINO_VERSION_MAJOR < 3
        if (s_fireOnTimer) timerAlarmDisable(s_fireOnTimer);
#endif
        return;
    }

    // Drive the spark pin to its IDLE-SAFE state (LOW for active-HIGH,
    // HIGH for active-LOW). Keeping the legacy function name so safety
    // callers don't need to change.
    sparkIdle(cdi::pins::SPARK_OUT);
    gpioLow(cdi::pins::MODE_LED);
    s_dwellInProgress = false;
#if ESP_ARDUINO_VERSION_MAJOR < 3
    if (s_fireOnTimer)  timerAlarmDisable(s_fireOnTimer);
    if (s_fireOffTimer) timerAlarmDisable(s_fireOffTimer);
#endif
}

} // namespace cdi::core::spark
