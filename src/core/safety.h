// Safety supervisor — runs periodically from loop().
//
// Responsibilities:
//   1. Task-watchdog feed. The main loop is registered with the
//      ESP-IDF task WDT; if `tick()` stops being called for
//      >TASK_WDT_TIMEOUT_S the device reboots — recovers from any
//      runaway tight loop or blocked task.
//   2. No-signal failsafe. If `spark::isArmed()` AND no CH1 pulser
//      event has arrived for >NO_SIGNAL_TIMEOUT_MS, force disarm
//      and set the `no_signal` flag. Re-arm requires explicit UI
//      action — auto-recovery would mask a real wiring fault.
//   3. Rev limiter. Two-tier:
//        - `main_limit_rpm`  → soft retard (live_stats subtracts
//          REV_LIMIT_RETARD_DEG from advance lookup).
//        - `overrev_limit_rpm` → hard cut (force disarm).
//      Hard cut requires explicit re-arm so the rider notices.
//   4. Brown-out detector. Enabled by Arduino-ESP32 by default;
//      logged on boot for verification.
//
// All public functions run in loop() / WS-callback context.
#pragma once

#include <cstdint>
#include <esp_attr.h>
#include "types.h"

namespace cdi::core::safety {

void begin();
void tick();           // call from loop ~every SAFETY_TICK_INTERVAL_MS

void setRevLimits(uint32_t main_rpm, uint32_t overrev_rpm);
uint32_t mainLimitRpm();
uint32_t overrevLimitRpm();

// ── Cut mode for the MAIN soft limit band ──
// (Overrev is always hard cut — safety can't be configured away.)
void     setMainCutMode(cdi::CutMode mode);
cdi::CutMode mainCutMode();
void     setMainRetardDeg(float deg);
float    mainRetardDeg();
void     setMainPatternRatio(uint8_t fire_n, uint8_t skip_n);
uint8_t  patternFireN();
uint8_t  patternSkipN();

// Status flags — read by live_stats / telemetry.
bool isRevLimited();   // RPM above main soft limit
bool noSignal();       // failsafe currently tripped
bool overRevCut();     // hard cut triggered

// ── ISR-callable: spark scheduler asks "should I fire THIS cycle?" ──
// Returns false to skip the current fire (PATTERN_CUT or PROGRESSIVE).
bool IRAM_ATTR shouldFire();

// Current retard° applied — read by live_stats when computing advance.
float currentRetardDeg();

// Clear sticky no_signal / overrev flags (called when user manually
// re-arms after a failsafe).
void clearFlags();

// ── No-signal failsafe toggle ──
// When disabled, the safety supervisor no longer auto-disarms on
// pulser signal loss. Rev limiter (main + overrev) and the absolute
// RPM ceiling guard stay active. Default: DISABLED — bench testing
// without a real pulser was the recurring frustration. Enable for
// real-road use so a mid-ride cable cut still stops firing.
void setNoSignalEnabled(bool en);
bool noSignalEnabled();

} // namespace cdi::core::safety
