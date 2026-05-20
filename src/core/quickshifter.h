// Quickshifter — clutchless up-shift via ignition cut.
//
// Input: GPIO14 (cdi::pins::QUICKSHIFTER) with internal pull-up.
//        Active LOW from a strain-gauge / micro-switch on the shift
//        linkage. Triggered on FALLING edge.
//
// On trigger:
//   - If RPM ∈ [min_rpm, max_rpm], arm a cut window of
//     `cut_duration_ms` from now.
//   - During the cut window, `shouldCut()` returns true so the
//     spark scheduler skips fires (torque drops → gear slips up).
//   - Window ends naturally; no clutch needed.
//
// Guards prevent cutting at idle (would stall) or at over-rev.
#pragma once

#include <cstdint>
#include <esp_attr.h>
#include "types.h"

namespace cdi::core::quickshift {

void begin();
void end();

bool isEnabled();
void setEnabled(bool en);

uint16_t  cutDurationMs();
void      setCutDurationMs(uint16_t ms);

cdi::rpm_t minRpm();
cdi::rpm_t maxRpm();
void       setRpmGuard(cdi::rpm_t min_rpm, cdi::rpm_t max_rpm);

// IRAM-safe — called from spark::onPulseCh1FromIsr to decide skip.
bool IRAM_ATTR shouldCut();

// True when within an active cut window (for telemetry indicator).
bool isActive();

uint32_t totalShifts();

} // namespace cdi::core::quickshift
