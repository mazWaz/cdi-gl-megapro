// Operating-mode state machine.
//
//   IGNITION    — pulser interrupts ON (drives both spark scheduling
//                 and live edge-stream broadcast). Spark output gated
//                 by a separate `spark::isArmed()` flag — arming is
//                 an explicit user action from the dashboard UI.
//   SAFE_HOLD   — pulser ISR detached, spark forced disarmed. Set by
//                 safety on no-signal timeout or fault. Recovered by
//                 explicit user action.
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
