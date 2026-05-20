// RPM measurement from successive CH1 falling-edge timestamps.
//
// Period inversion: rpm = 60_000_000 / period_us (one pulse per rev).
// Sanity-gated against `config::RPM_{MIN,MAX}_VALID` to ignore noise.
// Exponential moving average dampens cycle-to-cycle jitter without
// adding observable latency.
//
// Decay: if the last CH1 event is older than 2× the max valid period
// (~1.2 s @ 100 rpm floor), RPM is forced to 0.
//
// All public functions are non-ISR — call from `live_stats::tick()`.
#pragma once

#include "types.h"

namespace cdi::core::rpm {

void      onPulseCh1(cdi::micros_t ts_us);   // feed CH1 falling-edge ts
void      tick(cdi::micros_t now_us);        // call ~every loop to decay stale RPM
cdi::rpm_t current();                         // smoothed
cdi::rpm_t raw();                             // last instantaneous
cdi::micros_t lastCh1Us();
cdi::micros_t lastPeriodUs();
void      reset();

} // namespace cdi::core::rpm
