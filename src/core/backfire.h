// Backfire / burble / anti-lag mode.
//
// Intentionally retards (or post-TDC fires) ignition timing so that
// unburnt fuel ignites in the exhaust manifold — produces loud pops
// and (with rich mixtures) visible flames. Performance-art, not a
// real performance gain on a single-cylinder NA engine; primarily a
// sound/flames effect.
//
// 3 trigger modes:
//   DECEL  — when RPM drops rapidly (>500 rpm in 100 ms) within
//            the configured rpm_range, fire window opens for
//            `duration_ms`.
//   BURBLE — continuously active while RPM ∈ rpm_range.
//   LAUNCH — active whenever launch_control::isActive(), i.e. the
//            characteristic 2-step popping during a drag start.
//
// `currentRetardDeg()` is added to the live-stats retard chain
// (after rev-limit retard, before global trim). Random pattern adds
// ±50% jitter to retard amount for an irregular sound.
//
// SAFETY: large retard angles raise exhaust valve & header temps.
// Disabled by default; user must opt in per profile.
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::backfire {

void begin();
void tick(cdi::rpm_t rpm);   // call ~every 100 ms from loop()

bool isEnabled();
void setEnabled(bool en);

cdi::BackfireTrigger trigger();
void                 setTrigger(cdi::BackfireTrigger t);

void setRpmRange(cdi::rpm_t lo, cdi::rpm_t hi);
cdi::rpm_t rpmLo();
cdi::rpm_t rpmHi();

void  setRetardDeg(float deg);
float retardDeg();

void     setDurationMs(uint16_t ms);
uint16_t durationMs();

void setRandomPattern(bool r);
bool randomPattern();

// Status / output used by live_stats.
bool  isActive();
float currentRetardDeg();   // 0 when not active

} // namespace cdi::core::backfire
