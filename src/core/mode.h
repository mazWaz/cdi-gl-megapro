// Operating-mode state machine.
//
// Owns the active `OperatingMode` and arbitrates which subsystems run:
//
//   SCOPE       — ADC sampler ON, pulser interrupts OFF, no spark.
//   IGNITION    — ADC sampler OFF, pulser interrupts ON, spark scheduling
//                 enabled (spark output stays HIGH-impedance until a
//                 valid pulser train + armed flag is observed by safety).
//   SAFE_HOLD   — both OFF, spark output forced LOW. Set automatically
//                 by safety on no-signal timeout or fault.
//   BOOT        — transient before `begin()`.
//
// All transitions happen from loop()/WS-callback context (never ISR).
// Transitioning into IGNITION is a one-way arm — code intentionally
// requires explicit confirmation from the UI to enter it.
#pragma once

#include "types.h"

namespace cdi::core::mode {

void           begin();                  // enter SCOPE
OperatingMode  current();
bool           set(OperatingMode m);     // transition with side effects
const char*    name(OperatingMode m);

} // namespace cdi::core::mode
