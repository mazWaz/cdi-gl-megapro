// Shared types passed across module boundaries.
// Keep small (cache-friendly), POD only (memcpy-safe into ring buffers).
#pragma once

#include <cstdint>

namespace cdi {

// 64-bit microsecond timestamp from micros64() — wraps every 584,000 years.
using micros_t = uint64_t;

// RPM as uint16: range 0..65535, more than enough for any engine.
using rpm_t = uint16_t;

// Angle in tenths of degrees stored as int16 for compact ring entries
// (10 = 1.0°, range ±3276.7°). Convert to float only at the boundary.
using deg_x10_t = int16_t;

// ---------- Operating mode ----------
enum class OperatingMode : uint8_t {
    BOOT        = 0,   // initial state before setup completes
    SCOPE       = 1,   // ADC sampler running, no spark output
    IGNITION    = 2,   // pulser interrupts armed, spark scheduled
    SAFE_HOLD   = 3,   // failsafe: no spark, no sampling, waiting for recovery
};

// ---------- Pulser event ----------
// Pushed from IRAM ISR into SPSC ring; consumed by RPM calc.
enum class PulserChannel : uint8_t { CH1 = 0, CH2 = 1 };

struct PulserEvent {
    micros_t      ts_us;     // 8 B
    PulserChannel channel;   // 1 B
    uint8_t       _pad[7];   // align to 16 B for cache line friendliness
} __attribute__((packed));
static_assert(sizeof(PulserEvent) == 16, "PulserEvent must be 16 bytes");

// ---------- Advance map point ----------
struct AdvancePoint {
    rpm_t    rpm;      // 2 B
    deg_x10_t deg_x10; // 2 B — advance in tenths of a degree BTDC
} __attribute__((packed));
static_assert(sizeof(AdvancePoint) == 4, "AdvancePoint must be 4 bytes");

// ---------- Fire event (for telemetry & datalog) ----------
struct FireEvent {
    micros_t  ts_us;          // 8 B
    rpm_t     rpm;            // 2 B
    deg_x10_t target_deg_x10; // 2 B
    int16_t   jitter_us;      // 2 B — scheduled - actual fire time
    uint16_t  vbat_mv;        // 2 B
    uint16_t  dwell_us;       // 2 B
    uint8_t   flags;          // 1 B — bit0=launch, bit1=qs, bit2=backfire, bit3=rev_limit
    uint8_t   _pad[5];        // align to 24 B
} __attribute__((packed));
static_assert(sizeof(FireEvent) == 24, "FireEvent must be 24 bytes");

// FireEvent flag bits
namespace fire_flag {
    constexpr uint8_t LAUNCH_ACTIVE   = 1 << 0;
    constexpr uint8_t QS_CUT_ACTIVE   = 1 << 1;
    constexpr uint8_t BACKFIRE_ACTIVE = 1 << 2;
    constexpr uint8_t REV_LIMITED     = 1 << 3;
    constexpr uint8_t ALVP_DERATED    = 1 << 4;
    constexpr uint8_t SPARK_SKIPPED   = 1 << 5;
}

// ---------- Rev limiter cut mode ----------
enum class CutMode : uint8_t {
    OFF                 = 0,
    SOFT_RETARD         = 1,  // retard advance by configurable degrees
    HARD_CUT            = 2,  // skip spark entirely
    PATTERN_CUT         = 3,  // fire N then skip 1 (configurable ratio)
    SPARK_PROGRESSIVE   = 4,  // probability-based skip in transition band
};

// ---------- Backfire trigger ----------
enum class BackfireTrigger : uint8_t {
    OFF    = 0,
    DECEL  = 1,  // rpm dropping rapidly
    BURBLE = 2,  // continuous random retard in band
    LAUNCH = 3,  // active during 2-step launch
};

} // namespace cdi
