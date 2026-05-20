// Launch control — 2-step rev limiter triggered by clutch input.
//
// Input: GPIO13 (cdi::pins::LAUNCH_INPUT) with internal pull-up.
//        Active LOW (clutch lever pulled / launch switch closed).
//
// When enabled AND input LOW AND mode is IGNITION:
//   safety considers `launch_rpm` as the effective main rev limit
//   (instead of the configured main_limit). This holds RPM at the
//   launch point during a drag-strip take-off.
//
// Hard cut is used at the launch limit (rider needs the bouncing
// sensation as the cue). Below the launch limit, ignition behaves
// normally so the rider can rev up to that point.
//
// poll() runs from loop() ~every 10ms (cheap debounce).
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::launch {

void begin();
void poll();

bool isEnabled();
void setEnabled(bool en);

cdi::rpm_t launchRpm();
void       setLaunchRpm(cdi::rpm_t rpm);

bool isActive();    // currently in launch state (clutch held)

} // namespace cdi::core::launch
