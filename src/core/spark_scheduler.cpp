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
volatile bool     s_inductive     = true;    // true=TCI (default), false=CDI/SCR
volatile uint32_t s_nextDelayUs   = 0;      // cached, updated by loop
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
    if (!a) forceLow();
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

void setNextDelayUs(uint32_t d) {
    if (d < MIN_DELAY_US) d = MIN_DELAY_US;
    if (d > MAX_DELAY_US) d = MAX_DELAY_US;
    s_nextDelayUs = d;
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
    // Ceiling at configured value — live cap can only make dwell
    // SHORTER, never longer than the user requested.
    if (d > s_dwellUs) d = s_dwellUs;
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
// runs every 100 ms; at 15000 rpm that's ~25 cycles of latency
// before disarm). Period below this floor → RPM above ceiling →
// refuse to schedule.
constexpr uint32_t MIN_PERIOD_HARD_US = 60000000UL / 16000;  // 16000 rpm ceiling

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
    if (s_lastCh1IsrTs != 0) {
        cdi::micros_t inst_period = t_lead - s_lastCh1IsrTs;
        if (inst_period < MIN_PERIOD_HARD_US) {
            // RPM exceeds ceiling — skip this fire. Don't update
            // s_lastCh1IsrTs so the NEXT real CH1 (longer period
            // from us) reads correctly.
            return;
        }
    }
    s_lastCh1IsrTs = t_lead;

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

void setInductive(bool en) {
    s_inductive = en;
    Serial.printf("[spark] ignition = %s (spark fires on %s edge)\n",
                  en ? "INDUCTIVE/TCI" : "CAPACITIVE/CDI",
                  en ? "FALL"          : "RISE");
}
bool inductive() { return s_inductive; }

void forceLow() {
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
