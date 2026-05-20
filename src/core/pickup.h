// Active pickup geometry — owns the two numbers the spark scheduler
// needs at runtime to translate (target advance, period) into a fire
// delay from the CH1 edge:
//
//   * max_advance_ref_deg — degrees BTDC at which CH1 (leading edge)
//                           fires. The earliest possible ignition
//                           moment without predictive scheduling.
//   * magnet_width_deg    — angular size of the magnet protrusion.
//                           Drives CH2 (trailing) timing reference
//                           (= max_advance_ref - magnet_width) and
//                           shows up in diagnostics; not consumed by
//                           the current single-edge scheduler.
//
// Initial values come from the active preset (preset::apply sets them).
// A custom pickup override layered on top, when present, replaces
// either or both — useful when the user has physically modified the
// pulser tonjolan or auto-calibrated against a real measurement.
//
// All getters are cheap and IRAM-safe; the spark hot path reads
// max_advance_ref every fire to compute its delay.
#pragma once

namespace cdi::core::pickup {

// Set/get the leading-edge advance reference (deg BTDC). Default at
// boot is the legacy compile-time constant until a preset is applied.
void  setMaxAdvanceRef(float deg);
float maxAdvanceRef();

// Set/get the magnet angular width.
void  setMagnetWidth(float deg);
float magnetWidth();

// Override flag — when true, the values stored here came from a user
// calibration (or manual entry) and should NOT be overwritten the
// next time a preset is re-applied. Cleared on explicit "reset to
// preset" action.
void setOverride(bool v);
bool hasOverride();

// Source label (for UI display): one of "preset" / "auto_cal" /
// "manual" / "imported". Plain pointer to static string.
void        setSource(const char* s);
const char* source();

// Derived base advance reference (CH2 trailing edge timing) — purely
// informational, not used by the scheduler.
inline float baseAdvanceRef() { return maxAdvanceRef() - magnetWidth(); }

} // namespace cdi::core::pickup
