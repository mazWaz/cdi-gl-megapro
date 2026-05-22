#include "core/pickup.h"

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
volatile float       s_max_advance_ref_deg = cdi::config::MAX_ADVANCE_FROM_CH1_DEG;
volatile float       s_magnet_width_deg    = cdi::config::MAGNET_ANGULAR_WIDTH_DEG;
volatile bool        s_override            = false;
const char* volatile s_source              = "preset";

} // anonymous

void  setMaxAdvanceRef(float deg) { s_max_advance_ref_deg = deg; }
float maxAdvanceRef()             { return s_max_advance_ref_deg; }

void  setMagnetWidth(float deg)   { s_magnet_width_deg = deg; }
float magnetWidth()               { return s_magnet_width_deg; }

void setOverride(bool v) { s_override = v; }
bool hasOverride()       { return s_override; }

void        setSource(const char* s) { s_source = s ? s : "preset"; }
const char* source()                 { return s_source; }

} // namespace cdi::core::pickup
