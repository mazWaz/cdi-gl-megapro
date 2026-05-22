#include "core/spark_scheduler.h"

#include <Arduino.h>

#include "config.h"
#include "pinmap.h"
#include "core/safety.h"
#include "core/quickshifter.h"

namespace cdi::core::spark {
namespace {

// Three dedicated hardware timers — independent of scope's timer
// (group 0, idx 0).
//   timer 1 (fireOn)   — at fire moment: release charge, pulse spark
//   timer 2 (fireOff)  — N µs after fire: drop spark pin
//   timer 3 (chargeOn) — at fire-dwell: pull charge pin LOW (2-pin)
hw_timer_t* s_fireOnTimer   = nullptr;
hw_timer_t* s_fireOffTimer  = nullptr;
hw_timer_t* s_chargeOnTimer = nullptr;

volatile bool     s_armed         = false;
volatile bool     s_autoArm       = false;
volatile bool     s_activeLow     = false;   // false=active-HIGH (default) — spark trigger
volatile bool     s_useChargePin  = false;   // true=2-pin CDI mode
volatile uint32_t s_sparkPulseUs  = 200;     // SCR trigger pulse width
volatile uint32_t s_nextDelayUs   = 0;      // cached, updated by loop
volatile uint32_t s_dwellUs       = cdi::config::DEFAULT_DWELL_US;
volatile float    s_advanceOffsetDeg = 0.0f;

volatile uint32_t s_fireCount     = 0;
volatile int32_t  s_lastJitterUs  = 0;
volatile cdi::micros_t s_scheduledFireUs = 0;

// Previous CH1 timestamp (volatile because written in ISR). Used by
// 2-pin mode to predict next-fire timing and pre-charge cap in
// advance — so dwell can fit into the inter-fire gap instead of
// having to fit between THIS CH1 and THIS fire (which is too short
// at high RPM with target ~= max_advance_ref).
volatile cdi::micros_t s_prevCh1Ts = 0;
constexpr uint32_t PERIOD_STALE_US = 200000;   // > this = stalled, treat as cold start

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

// Charge pin is ALWAYS active-LOW (typical CDI step-up enable).
// LOW = MOSFET ON, cap charging, suara high-freq. HIGH = idle.
inline void IRAM_ATTR chargeActive() { gpioLow(cdi::pins::CHARGE_OUT);  }
inline void IRAM_ATTR chargeIdle()   { gpioHigh(cdi::pins::CHARGE_OUT); }

// "Charge start" — fired (T - dwell_us) before fire moment in 2-pin
// CDI mode. Pulls CHARGE_OUT LOW so the boost converter starts
// filling the capacitor.
void IRAM_ATTR isrChargeOn() {
    chargeActive();
#if ESP_ARDUINO_VERSION_MAJOR < 3
    timerAlarmDisable(s_chargeOnTimer);
#endif
}

// "Fire moment" — the spark moment.
//   1-pin mode  : start charge phase (SPARK active for dwell), spark
//                 fires on the FALLING edge handled by isrFireOff.
//   2-pin mode  : charge phase has already happened (CHARGE was LOW
//                 for dwell_us). Now release CHARGE → idle (HIGH) AND
//                 pulse SPARK → active (SCR gate trigger). Spark fires
//                 on this RISING edge.
void IRAM_ATTR isrFireOn() {
    cdi::micros_t now = (cdi::micros_t)micros();
    s_lastJitterUs = (int32_t)(now - s_scheduledFireUs);

    if (s_useChargePin) {
        // 2-pin: stop charging, trigger SCR gate.
        chargeIdle();
        sparkActive(cdi::pins::SPARK_OUT);
        gpioHigh(cdi::pins::MODE_LED);
        // Arm spark-off timer for now + spark pulse width.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        timerAlarm(s_fireOffTimer, s_sparkPulseUs, false, 0);
#else
        timerAlarmDisable(s_fireOffTimer);
        timerRestart(s_fireOffTimer);
        timerAlarmWrite(s_fireOffTimer, s_sparkPulseUs, false);
        timerAlarmEnable(s_fireOffTimer);
#endif
    } else {
        // 1-pin: start charge phase, schedule fire-off at dwell.
        sparkActive(cdi::pins::SPARK_OUT);
        gpioHigh(cdi::pins::MODE_LED);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        timerAlarm(s_fireOffTimer, s_dwellUs, false, 0);
#else
        timerAlarmDisable(s_fireOffTimer);
        timerRestart(s_fireOffTimer);
        timerAlarmWrite(s_fireOffTimer, s_dwellUs, false);
        timerAlarmEnable(s_fireOffTimer);
#endif
    }

#if ESP_ARDUINO_VERSION_MAJOR < 3
    timerAlarmDisable(s_fireOnTimer);
#endif
}

void IRAM_ATTR isrFireOff() {
    // 1-pin: end charge → primary collapse → spark on falling edge.
    // 2-pin: SCR gate pulse already latched the SCR; release pulse so
    //        the gate isn't held active during the next charge cycle.
    sparkIdle(cdi::pins::SPARK_OUT);
    gpioLow(cdi::pins::MODE_LED);
    s_fireCount++;

#if ESP_ARDUINO_VERSION_MAJOR < 3
    timerAlarmDisable(s_fireOffTimer);
#endif
}

} // anonymous

void begin() {
    pinMode(cdi::pins::SPARK_OUT,  OUTPUT);
    pinMode(cdi::pins::CHARGE_OUT, OUTPUT);
    pinMode(cdi::pins::MODE_LED,   OUTPUT);
    sparkIdle(cdi::pins::SPARK_OUT);
    chargeIdle();                          // 2-pin CDI: idle = step-up OFF
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
    if (!s_chargeOnTimer) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        s_chargeOnTimer = timerBegin(1000000);
        timerAttachInterrupt(s_chargeOnTimer, &isrChargeOn);
#else
        s_chargeOnTimer = timerBegin(3, 80, true);
        timerAttachInterrupt(s_chargeOnTimer, &isrChargeOn, true);
#endif
    }

    Serial.println("[spark] scheduler ready · disarmed · 2-pin mode available");
}

void end() {
    forceLow();
    s_armed = false;
}

bool setArmed(bool a) {
    if (!a) forceLow();
    s_armed = a;
    // Reset predictive-charge history so re-arm doesn't try to use a
    // stale period. First fire after re-arm will cold-start the cap.
    s_prevCh1Ts = 0;
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
    // Optional dwell override for bench diagnostic.
    const uint32_t saved_dwell = s_dwellUs;
    uint32_t effective_dwell = s_dwellUs;
    if (dwell_override_us > 0) {
        uint32_t d = dwell_override_us;
        if (d < 500)    d = 500;
        if (d > 20000)  d = 20000;     // up to 20 ms for diag
        s_dwellUs       = d;
        effective_dwell = d;
        Serial.printf("[spark] manual test fire (override dwell=%u us, 2-pin=%d)\n",
                      (unsigned)d, s_useChargePin ? 1 : 0);
    } else {
        Serial.printf("[spark] manual test fire (dwell=%u us, 2-pin=%d)\n",
                      (unsigned)s_dwellUs, s_useChargePin ? 1 : 0);
    }

    // Sequence: schedule fire-on at now + dwell + tiny lead (so we
    // can fit the charge phase first in 2-pin mode). In 1-pin mode
    // we just fire ~100µs from now as before.
    const uint32_t lead_us = s_useChargePin ? (effective_dwell + 100) : 100;
    s_scheduledFireUs = (cdi::micros_t)micros() + lead_us;

    if (s_useChargePin) {
        // Start charge immediately, fire after dwell elapses.
        chargeActive();
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOnTimer, lead_us, false, 0);
#else
    timerAlarmDisable(s_fireOnTimer);
    timerRestart(s_fireOnTimer);
    timerAlarmWrite(s_fireOnTimer, lead_us, false);
    timerAlarmEnable(s_fireOnTimer);
#endif

    if (dwell_override_us > 0) {
        delay((effective_dwell / 1000) + 5);
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

    // ── 2-pin CDI charge logic ──
    //
    // Goal: cap MUST be full at fire moment, AT ALL RPM, but module
    // MUST stay cool (no continuous charging).
    //
    // Strategy: PREDICTIVE pre-charge — when CH1 arrives, we know the
    // period (CH1 to prev CH1). The NEXT fire will happen at
    // (t_lead + period + delay). We schedule the chargeOn for that
    // next fire to start at (next_fire - dwell), which falls into
    // the IDLE gap between current fire and next fire.
    //
    // Duty cycle = dwell / period. At 11000 RPM (period 5454 µs)
    // and dwell 2500 µs → 46% duty — module sees half-time charging.
    // At idle 1500 RPM (period 40000 µs) → 6% duty. Cap full at fire
    // moment, module dingin between cycles.
    //
    // Special cases:
    //   * First fire since arm (prev_ts==0) — no period known yet.
    //     Best-effort: charge immediately (cap may be partial for
    //     THIS fire, full from fire #2 onwards).
    //   * Period stale (>200ms — engine stalled then re-fired) — same
    //     as first-fire, treat as cold start.
    //   * Period too short (<dwell+gap) — RPM beyond realistic;
    //     skip predictive (engine wouldn't sustain anyway).
    if (s_useChargePin) {
        if (s_prevCh1Ts == 0 ||
            (uint32_t)(t_lead - s_prevCh1Ts) > PERIOD_STALE_US) {
            // Cold start — charge cap now (best-effort for this fire)
            chargeActive();
        } else {
            // Predictive schedule for NEXT fire
            const uint32_t period = (uint32_t)(t_lead - s_prevCh1Ts);
            if (period > s_dwellUs + 300) {       // need room for dwell + spark pulse + gap
                const uint32_t next_charge_delay = period + delay - s_dwellUs;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
                timerAlarm(s_chargeOnTimer, next_charge_delay, false, 0);
#else
                timerAlarmDisable(s_chargeOnTimer);
                timerRestart(s_chargeOnTimer);
                timerAlarmWrite(s_chargeOnTimer, next_charge_delay, false);
                timerAlarmEnable(s_chargeOnTimer);
#endif
            }
            // else: period too short for dwell, cap stays partial
        }
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_fireOnTimer, delay, false, 0);
#else
    timerAlarmDisable(s_fireOnTimer);
    timerRestart(s_fireOnTimer);
    timerAlarmWrite(s_fireOnTimer, delay, false);
    timerAlarmEnable(s_fireOnTimer);
#endif

    s_prevCh1Ts = t_lead;
}

uint32_t totalFires()    { return s_fireCount; }
int32_t  lastJitterUs()  { return s_lastJitterUs; }

void setActiveLow(bool en) {
    s_activeLow = en;
    sparkIdle(cdi::pins::SPARK_OUT);
    Serial.printf("[spark] polarity = ACTIVE-%s\n", en ? "LOW" : "HIGH");
}
bool activeLow() { return s_activeLow; }

void setUseChargePin(bool en) {
    s_useChargePin = en;
    // Re-drive both pins to safe idle on mode switch.
    sparkIdle(cdi::pins::SPARK_OUT);
    chargeIdle();
    Serial.printf("[spark] mode = %s\n", en ? "2-pin (charge+trigger)" : "1-pin");
}
bool useChargePin() { return s_useChargePin; }

void setSparkPulseUs(uint32_t us) {
    if (us < 50)    us = 50;
    if (us > 2000)  us = 2000;
    s_sparkPulseUs = us;
}
uint32_t sparkPulseUs() { return s_sparkPulseUs; }

void forceLow() {
    // Drive both spark + charge pins to IDLE-SAFE state. (Function
    // name is legacy — covers more than just LOW now.)
    sparkIdle(cdi::pins::SPARK_OUT);
    chargeIdle();                          // step-up OFF
    gpioLow(cdi::pins::MODE_LED);
#if ESP_ARDUINO_VERSION_MAJOR < 3
    if (s_fireOnTimer)   timerAlarmDisable(s_fireOnTimer);
    if (s_fireOffTimer)  timerAlarmDisable(s_fireOffTimer);
    if (s_chargeOnTimer) timerAlarmDisable(s_chargeOnTimer);
#endif
}

} // namespace cdi::core::spark
