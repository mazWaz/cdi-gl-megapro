// Pulser edge-detection for IGNITION_MODE.
//
// `begin()` attaches IRAM ANY-EDGE (CHANGE) interrupts on the two
// optocoupler outputs. Each edge produces a `PulserEvent` (with
// `level` field = 0 after FALLING / 1 after RISING) pushed onto
// TWO lock-free SPSC rings:
//
//   * ignition ring — drained by RPM calc / spark scheduler. Only
//     FALLING edges (level==0) are meaningful to the consumer.
//   * scope ring   — drained by scope::edge_capture for live waveform
//     visualization. Both edges included so duty / pulse-width can
//     be reconstructed in UI.
//
// The dual-ring split keeps ignition and scope decoupled: a slow or
// blocked scope consumer cannot back-pressure the spark scheduler.
//
// Public API runs in loop()/WS-callback context.
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::pulser {

void begin();
void end();
bool isAttached();

// Drain one ignition-side event (FALLING+RISING; consumer filters).
bool tryPop(PulserEvent& out);

// Drain one scope-side event. Independent ring; safe to consume from a
// different (non-ISR) caller than tryPop().
bool tryPopScope(PulserEvent& out);

// Total edge interrupts received since boot (both channels, both edges).
uint32_t totalCount();

// Approximate ignition-ring occupancy.
uint32_t pending();

// Approximate scope-ring occupancy.
uint32_t pendingScope();

// Number of events dropped because the scope ring was full (overrun).
// Useful as a diagnostic — non-zero means UI consumer is too slow.
uint32_t scopeOverruns();

// Tier-2 2B: cumulative pickup signal anomalies since boot (spurious
// CH1 + extra/lone CH2). A health indicator for pickup wiring/shielding;
// a rising count under load points at EMI on the pulser line.
uint32_t pickupAnomalies();

} // namespace cdi::core::pulser
