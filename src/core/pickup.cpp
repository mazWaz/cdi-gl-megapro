#include "core/pickup.h"

#include <cstring>       // strncpy

#include "config.h"

namespace cdi::core::pickup {
namespace {

// Defaults from compile-time constants — overwritten as soon as
// preset::apply runs in setup(). These remain a safe fallback if
// preset application fails for any reason.
//
// `volatile` is required because these are read by live_stats on
// core 1 every spark cycle and written by WS handlers (setPickup*)
// on core 0. Without volatile the compiler is free to cache the
// register value, so a UI-side calibration change wouldn't take
// effect until the next context switch or function-boundary spill.
// 32-bit aligned writes on Xtensa are atomic at the word level, so
// no additional locking is needed for these primitives.
// The two float geometry numbers ARE read cross-core (live_stats core 1
// reads them per cycle; WS handlers core 0 write them) → volatile.
// The source string below is core-0/WS-only (UI display) and owns its
// storage, so it does not need volatile.
volatile float       s_max_advance_ref_deg = cdi::config::MAX_ADVANCE_FROM_CH1_DEG;
volatile float       s_magnet_width_deg    = cdi::config::MAGNET_ANGULAR_WIDTH_DEG;
volatile bool        s_override            = false;
// OWNED copy, not a borrowed pointer: config_store::applyJson passes a
// const char* that points into a transient JsonDocument freed when
// load() returns — storing that pointer would dangle (use-after-free in
// buildJson/getPickup, garbage persisted to NVS). Copy into a fixed
// buffer instead (audit H5).
char s_source_buf[16] = "preset";

} // anonymous

void  setMaxAdvanceRef(float deg) { s_max_advance_ref_deg = deg; }
float maxAdvanceRef()             { return s_max_advance_ref_deg; }

void  setMagnetWidth(float deg)   { s_magnet_width_deg = deg; }
float magnetWidth()               { return s_magnet_width_deg; }

void setOverride(bool v) { s_override = v; }
bool hasOverride()       { return s_override; }

void setSource(const char* s) {
    if (!s) s = "preset";
    strncpy(s_source_buf, s, sizeof(s_source_buf) - 1);
    s_source_buf[sizeof(s_source_buf) - 1] = '\0';
}
const char* source()                 { return s_source_buf; }

} // namespace cdi::core::pickup
