// Shift light driver — GPIO27 visual indicator.
//
// States:
//   < rpm_warn        → OFF
//   rpm_warn ≤ rpm < rpm_shift → flashing 5 Hz
//   rpm ≥ rpm_shift   → solid ON
//
// Disabled (setEnabled(false)) → output forced LOW.
//
// `tick()` is called from loop() ~every 50 ms (covers the 5 Hz flash).
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::shift_light {

void begin();
void tick();

bool isEnabled();
void setEnabled(bool en);

void       setThresholds(cdi::rpm_t rpm_warn, cdi::rpm_t rpm_shift);
cdi::rpm_t rpmWarn();
cdi::rpm_t rpmShift();

// 0 = OFF, 1 = flashing, 2 = solid — for telemetry.
uint8_t state();

} // namespace cdi::core::shift_light
