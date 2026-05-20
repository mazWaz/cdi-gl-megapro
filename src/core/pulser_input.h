// Pulser falling-edge detection for IGNITION_MODE.
//
// `begin()` attaches IRAM interrupts on the two optocoupler outputs.
// Each falling edge produces a `PulserEvent` pushed onto a lock-free
// SPSC ring; the loop drains via `tryPop()` to compute RPM and
// schedule spark events.
//
// MUST NOT be active simultaneously with the scope-mode ADC sampler —
// the timer ISR + GPIO ISR competing for the same pins is undefined.
// The `core::mode` module enforces this transition.
//
// Public API runs in loop()/WS-callback context.
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::pulser {

void begin();
void end();
bool isAttached();

// Drain one pulser event from the ring. False if empty.
bool tryPop(PulserEvent& out);

// Total falling-edge interrupts received since boot (both channels).
uint32_t totalCount();

// Approximate ring occupancy (for telemetry diagnostics).
uint32_t pending();

} // namespace cdi::core::pulser
