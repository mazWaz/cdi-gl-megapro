// Live operational telemetry aggregator.
//
// `tick()` drains pulser events, feeds RPM calc, and decays stale
// state. Safe to call frequently from loop().
//
// `snapshot()` returns a coherent struct used by ws_server to build
// the binary telemetry frame and by serial-side debug.
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::telemetry {

struct LiveStats {
    OperatingMode  mode;
    cdi::rpm_t     rpm;                 // smoothed
    cdi::rpm_t     rpm_raw;
    cdi::deg_x10_t target_advance_x10;  // 0.1° units, from advance map lookup
    cdi::deg_x10_t actual_advance_x10;  // 0.1° units — angle the spark ACTUALLY
                                        // fires at (from scheduled delay + dwell).
                                        // Diverges from target when dwell-capped
                                        // or near max_advance_ref. Diagnostic.
    uint32_t       pulser_count;
    uint32_t       pulser_pending;
    uint32_t       uptime_ms;
    uint32_t       free_heap;
    uint8_t        armed;               // spark armed flag
    uint32_t       fire_count;
    int16_t        last_jitter_us;
    uint8_t        safety_flags;        // bit0 rev_limited, bit1 no_signal, bit2 overrev_cut
    uint16_t       main_limit_rpm;
    uint16_t       overrev_limit_rpm;
    uint16_t       dwell_us;
    cdi::deg_x10_t advance_offset_x10;  // 0.1° units, signed
    uint8_t        cut_mode;
    uint8_t        retard_half_deg;     // 0.5° step
    uint8_t        pattern_fire_n;
    uint8_t        pattern_skip_n;
    uint8_t        shift_state;         // 0 off, 1 flash, 2 solid
    uint8_t        flags2;              // bit0 shift_enabled, bit1 dwell_curve_enabled
                                        // bit2 launch_enabled, bit3 qs_enabled
                                        // bit4 launch_active, bit5 qs_active
    uint16_t       shift_rpm_warn;
    uint16_t       shift_rpm_shift;
    uint16_t       launch_rpm;
    uint16_t       qs_cut_ms;
    uint32_t       qs_count;
    uint8_t        backfire_trigger;   // BackfireTrigger enum value
    uint8_t        flags3;             // bit0 bf_enabled, bit1 bf_active, bit2 bf_random
    uint16_t       bf_rpm_lo;
    uint16_t       bf_rpm_hi;
    uint8_t        bf_retard_half_deg;
    uint16_t       bf_duration_ms;
    uint16_t       vbat_mv;
    uint8_t        alvp_state;          // 0 NORMAL, 1 DERATE, 2 DISARM_LOW
    uint8_t        alvp_derate_v_x10;
    uint8_t        alvp_disarm_v_x10;
    uint8_t        flags4;              // bit0 alvp_enabled, bit1 auto_arm
    uint16_t       alvp_derate_rpm;
};

void       tick();
LiveStats  snapshot();
void       reset();

} // namespace cdi::telemetry
