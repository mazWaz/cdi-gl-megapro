// Operating-mode state machine.
//
//   IGNITION    — pulser interrupts ON; spark AUTO-ARMED (market-CDI:
//                 on whenever powered in this mode, no manual arm step).
//                 `armed` follows the mode. Faults pulse-cut only.
//   SAFE_HOLD   — pulser ISR detached, spark forced OFF. Entered by the
//                 panic button (GPIO0), UI KILL, or multi-tooth detection.
//                 Left by explicit user action (UI NYALAKAN → IGNITION).
//   BOOT        — transient before `begin()`.
//
// All transitions happen from loop()/WS-callback context (never ISR).
#pragma once

#include "types.h"

namespace cdi::core::mode {

void           begin();                  // enter IGNITION
OperatingMode  current();
bool           set(OperatingMode m);     // transition with side effects
const char*    name(OperatingMode m);

} // namespace cdi::core::mode
