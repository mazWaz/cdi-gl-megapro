#include "core/pickup.h"

#include "config.h"

namespace cdi::core::pickup {
namespace {

// Defaults from compile-time constants — overwritten as soon as
// preset::apply runs in setup(). These remain a safe fallback if
// preset application fails for any reason.
float       s_max_advance_ref_deg = cdi::config::MAX_ADVANCE_FROM_CH1_DEG;
float       s_magnet_width_deg    = cdi::config::MAGNET_ANGULAR_WIDTH_DEG;
bool        s_override            = false;
const char* s_source              = "preset";

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
