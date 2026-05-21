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
volatile uint32_t s_nextDelayUs   = 0;      // cached, updated by loop
volatile uint32_t s_dwellUs       = cdi::config::DEFAULT_DWELL_US;
volatile float    s_advanceOffsetDeg = 0.0f;

volatile uint32_t s_fireCount     = 0;
volatile int32_t  s_lastJitterUs  = 0;
volatile cdi::micros_t s_scheduledFireUs = 0;

constexpr uint32_t MIN_DELAY_US = 50;       // safety floor
constexpr uint32_t MAX_DELAY_US = 50000;    // 50 ms ceiling

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

    // Arm fire-off timer for now + dwell.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOffTimer, s_dwellUs, false, 0);
#else
    timerAlarmDisable(s_fireOffTimer);
    timerRestart(s_fireOffTimer);
    timerAlarmWrite(s_fireOffTimer, s_dwellUs, false);
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
    // no log — called every loop tick when dwell curve enabled.
}
uint32_t dwellUs() { return s_dwellUs; }

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
    const uint32_t saved_dwell = s_dwellUs;
    if (dwell_override_us > 0) {
        uint32_t d = dwell_override_us;
        if (d < 500)    d = 500;
        if (d > 20000)  d = 20000;     // allow up to 20 ms for diag
        s_dwellUs = d;
        Serial.printf("[spark] manual test fire (override dwell=%u us)\n", (unsigned)d);
    } else {
        Serial.printf("[spark] manual test fire (dwell=%u us)\n", (unsigned)s_dwellUs);
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

    // Restore configured dwell after the fire window has closed.
    // (Crude — relies on the timer-based ISR completing within
    // override+safety. Acceptable since this is a manual single-shot
    // diagnostic path, not the hot pulser path.)
    if (dwell_override_us > 0) {
        delay((dwell_override_us / 1000) + 5);
        s_dwellUs = saved_dwell;
    }
}

void IRAM_ATTR onPulseCh1FromIsr(cdi::micros_t t_lead) {
    if (!s_armed) return;
    // Quickshifter active cut window — skip this fire entirely.
    if (cdi::core::quickshift::shouldCut()) return;
    // Cut-mode gate: pattern / progressive skip this cycle.
    if (!cdi::core::safety::shouldFire()) return;

    uint32_t delay = s_nextDelayUs;
    if (delay < MIN_DELAY_US) return;

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

void forceLow() {
    // Drive the spark pin to its IDLE-SAFE state (LOW for active-HIGH,
    // HIGH for active-LOW). Keeping the legacy function name so safety
    // callers don't need to change.
    sparkIdle(cdi::pins::SPARK_OUT);
    gpioLow(cdi::pins::MODE_LED);
#if ESP_ARDUINO_VERSION_MAJOR < 3
    if (s_fireOnTimer)  timerAlarmDisable(s_fireOnTimer);
    if (s_fireOffTimer) timerAlarmDisable(s_fireOffTimer);
#endif
}

} // namespace cdi::core::spark
