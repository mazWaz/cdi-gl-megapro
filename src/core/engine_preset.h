// Motor preset library — pre-tuned configurations for popular
// Indonesian motorcycles.
//
// Each preset bundles: trigger channel, magnet geometry, full advance
// map (12–16 points), rev limits, dwell. Selecting a preset overwrites
// the active configuration with these defaults; user can fine-tune
// afterward (custom flag will track deviations).
//
// All preset data lives in Flash (PROGMEM-friendly) — zero RAM cost.
#pragma once

#include <cstddef>
#include <cstdint>
#include "types.h"

namespace cdi::core::preset {

// 32 step matches the runtime advance map ceiling (config::MAX_ADVANCE_POINTS)
// and exceeds commercial aftermarket CDIs (BRT I-Max Neo 16, BRT Pro 24,
// Rextor Pro Drag 2 16). Each preset can now describe a smooth curve
// with ~200-500 rpm resolution across the full operating band.
constexpr size_t MAX_POINTS_PER_PRESET = 32;

struct Point {
    uint16_t rpm;
    float    deg;
};

struct Preset {
    const char* id;              // kebab-case key
    const char* category;        // "Honda 4T" / "Yamaha 4T" / "2-Stroke" / etc.
    const char* display;         // user-facing name
    uint8_t     trigger_channel; // 1 = CH1 leading, 2 = CH2 trailing
    float       magnet_width_deg;
    float       max_advance_deg; // reference advance angle at primary edge
    Point       points[MAX_POINTS_PER_PRESET];
    uint8_t     point_count;
    uint16_t    rev_main_rpm;
    uint16_t    rev_overrev_rpm;
    uint16_t    dwell_us;
    const char* notes;
};

// Number of presets in the library.
size_t count();

// Random-access preset by index.
const Preset* at(size_t i);

// Find preset by id (returns nullptr if not found).
const Preset* find(const char* id);

// Apply preset values to all relevant modules (advance map, safety,
// spark, etc.). Sets `s_currentId` so isModified() can compare later.
// Returns false if id not found.
bool apply(const char* id);

// Current loaded preset id (or "custom" if user-built).
const char* currentId();

// True if any setting has been changed since the last apply.
bool isModified();

void markModifiedFlag();  // call after any user-driven setter
void resetModifiedFlag(); // call after applying preset cleanly

} // namespace cdi::core::preset
