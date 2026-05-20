// RPM-compensated dwell curve.
//
// Without compensation, `spark::setDwellUs()` is a fixed value set
// from the Config UI. With this curve enabled, `live_stats::tick()`
// looks up the appropriate dwell for the current RPM and pushes it
// to the spark scheduler each cycle.
//
// Use case: HV stages with slow optocoupler / driver may need wider
// gate pulses at high RPM (less time available per cycle) to ensure
// the SCR fully turns on. Or some users want shorter dwell at idle
// to reduce coil heating.
//
// Storage is static — 8 breakpoints max, linear interpolation, clamp
// at ends. Disabled by default (manual dwell from Config rules).
#pragma once

#include <cstdint>
#include <ArduinoJson.h>

#include "types.h"

namespace cdi::core::dwell {

constexpr size_t MAX_DWELL_POINTS = 8;

struct Point {
    cdi::rpm_t rpm;
    uint16_t   dwell_us;
};

bool     isEnabled();
void     setEnabled(bool en);

// lookup with linear interpolation; clamps at ends.
// Returns the curve value regardless of enabled state — caller decides
// whether to apply it.
uint16_t lookup(cdi::rpm_t rpm);

bool     loadFromJson(const JsonArrayConst& arr);   // [[rpm,us], ...]
void     serialize(JsonArray out);
size_t   count();

void     loadDefault();   // flat 2500 µs across band

} // namespace cdi::core::dwell
