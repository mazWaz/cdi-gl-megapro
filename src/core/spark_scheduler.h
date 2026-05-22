// Spark output scheduler — precise µs-level timing for ignition.
//
// Flow:
//   1) Loop side computes `next_delay_us` after each RPM update from
//      the advance map (rpm → deg → delay). Cached via setNextDelayUs().
//   2) Pulser ISR (CH1 falling) calls `onPulseCh1FromIsr(t_lead)`. If
//      armed + sane RPM seen, schedules HW timer alarm to pulse GPIO25
//      HIGH at (t_lead + cached_delay), then LOW after dwell_us.
//   3) Fire-on timer ISR: GPIO25 HIGH + GPIO26 HIGH (visible LED), arms
//      fire-off timer for now+dwell. Captures jitter = (scheduled − actual).
//   4) Fire-off timer ISR: GPIO25 LOW + GPIO26 LOW. Increments fire_count.
//
// Safety:
//   - `armed` defaults FALSE. setArmed(true) must be called from UI to
//     enable any output. `armed` is forced FALSE on mode change OR when
//     no pulser activity for >NO_SIGNAL_TIMEOUT_MS (handled by safety).
//   - On disarm, both timers are stopped and GPIOs forced LOW.
//   - Out-of-range delay (>period) is clamped to safe value.
//
// All public functions are non-ISR-safe except `onPulseCh1FromIsr`.
#pragma once

#include <cstdint>
#include <esp_attr.h>     // IRAM_ATTR macro
#include "types.h"

namespace cdi::core::spark {

void begin();
void end();

// Safety gate. Returns the effective armed state after the call.
bool setArmed(bool armed);
bool isArmed();

// Persistent preference — when true, spark::setArmed(true) is called
// automatically at boot once config is loaded and IGNITION mode is
// active. Default false. Use case: production deployment where the
// ESP32 is powered by the ignition key, so user expects spark to
// "just work" without manual UI step.
void setAutoArm(bool en);
bool autoArm();

// Set the next-cycle fire delay (microseconds from CH1 falling edge to
// spark fire). Updated by live_stats every loop tick from the latest
// RPM + advance map lookup. Safe to call from any context.
void setNextDelayUs(uint32_t delay_us);

// Configure spark pulse width / primary charge time. This is the
// user-intended value persisted to NVS; live_stats may apply a
// smaller "effective" value at high RPM via setEffectiveDwellUs to
// prevent fire-off overlap with the next cycle (see comment in
// live_stats::tick).
void     setDwellUs(uint32_t dwell_us);
uint32_t dwellUs();          // user-configured
uint32_t configuredDwellUs(); // same — explicit name for clarity

// ISR-side effective dwell — set per cycle by live_stats. Capped
// against the configured value so user can never get LONGER dwell
// than they asked for, only equal or shorter.
void     setEffectiveDwellUs(uint32_t dwell_us);
uint32_t effectiveDwellUs();

// Global advance trim — added to every map lookup. Compensates for
// optocoupler/cable propagation delay. Range typically ±5°.
void setAdvanceOffsetDeg(float deg);
float advanceOffsetDeg();

// Fire one test spark 100 µs from now. Bypasses pulser/armed gate —
// caller is responsible for confirming bench-safe conditions. When
// `dwell_override_us` is non-zero, use that duration instead of the
// configured s_dwellUs — useful for bench diagnostic (e.g. 10 ms
// long-dwell test to confirm the MOSFET path is alive visually).
void manualFire(uint32_t dwell_override_us = 0);

// ── Output polarity ──
// false (default): ACTIVE-HIGH. Idle LOW, charge phase HIGH, fall
//                  edge fires spark. Matches direct N-MOSFET / NPN
//                  gate drive.
// true           : ACTIVE-LOW.  Idle HIGH, charge phase LOW, rise
//                  edge fires spark. Matches P-MOSFET / PNP gate
//                  drive, opto-isolated drivers wired LED-cathode-
//                  to-GPIO, or pre-built CDI modules with active-
//                  LOW trigger input.
// Changing this flips: idle state, manualFire pulse polarity, and
// the spark scheduler ISR writes. Hardware pull-resistor MUST
// match — pull-down for ACTIVE-HIGH, pull-up to 3V3 for ACTIVE-LOW.
void setActiveLow(bool en);
bool activeLow();

// ── Ignition topology ──
// true  (default) = inductive / TCI — GPIO HIGH charges primary
//                   coil through MOSFET; spark fires on the FALL
//                   edge at end of dwell. live_stats must subtract
//                   dwell_us from the CH1-to-spark delay so the
//                   actual fire-off transition lands at the desired
//                   crank angle.
// false           = capacitive / CDI / SCR — GPIO HIGH triggers an
//                   SCR gate; spark fires on the RISING edge.
//                   Dwell here is just the trigger-pulse width and
//                   should NOT be subtracted.
void setInductive(bool en);
bool inductive();

// ─── ISR entry — called from pulser CH1 falling-edge ISR ───
void IRAM_ATTR onPulseCh1FromIsr(cdi::micros_t t_lead);

// ─── Telemetry ───
uint32_t totalFires();
int32_t  lastJitterUs();   // scheduled - actual (signed; positive = late)

// ─── Force outputs LOW (failsafe). Called by safety.cpp ───
void forceLow();

} // namespace cdi::core::spark
