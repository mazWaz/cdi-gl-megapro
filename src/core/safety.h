// Safety supervisor — runs periodically from loop().
//
// Responsibilities:
//   1. Task-watchdog feed. The main loop is registered with the
//      ESP-IDF task WDT; if `tick()` stops being called for
//      >TASK_WDT_TIMEOUT_S the device reboots — recovers from any
//      runaway tight loop or blocked task.
//   2. Rev limiter. Two-tier, both SELF-RECOVERING pulse-cut (never a
//      sticky disarm — see safety.cpp): `main_limit_rpm` (configurable
//      cut mode) and `overrev_limit_rpm` (hard cut). ALVP under-voltage
//      also routes through here as a self-recovering full cut.
//   3. Brown-out detector. Enabled by Arduino-ESP32 by default;
//      logged on boot for verification.
//
// (No-signal failsafe removed — vestigial once self-recovering: no CH1
//  already means no spark, so a lost pickup stops & resumes firing on
//  its own with no disarm.)
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
bool overRevCut();     // hard cut triggered

// ── ISR-callable: spark scheduler asks "should I fire THIS cycle?" ──
// Returns false to skip the current fire (PATTERN_CUT or PROGRESSIVE).
bool IRAM_ATTR shouldFire();

// Current retard° applied — read by live_stats when computing advance.
float currentRetardDeg();

// Clear sticky overrev flags (called when re-entering IGNITION / arm).
void clearFlags();

// True when a blocking flash erase/write (NVS persist, LittleFS snapshot,
// OTA) can run without risk of stranding the spark output. Flash ops
// stall the spark core during sector erases; landing mid-dwell would hold
// the coil primary energized for the erase duration. Safe iff the engine
// is not actively firing: disarmed, or armed but not turning (no CH1 for
// >600 ms). Callers must defer flash writes until this returns true.
bool flashWriteSafe();

} // namespace cdi::core::safety
