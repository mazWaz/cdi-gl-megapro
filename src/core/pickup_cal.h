// Pickup magnet-width auto-calibration (Phase B).
//
// Runs while the engine is at a steady idle. Drains the pulser scope
// ring and measures magnet width per revolution using a BIDIRECTIONAL
// period estimate (prev + next CH1 falls averaged) — eliminating the
// decel/accel bias that hand-crank sampling suffered from in earlier
// data sets.
//
// Each measured pulse must pass a steady-state filter (|prev_period
// - next_period| / mean < 5 %) and the running RPM must sit inside
// [800, 3000] (idle band). Pulses outside the window are silently
// dropped — they don't fail the run, they just don't count.
//
// After N clean measurements (default 50), the run completes and
// reports:
//   * median magnet_width_deg (robust to outliers)
//   * stddev / confidence-percent
//   * RPM at measurement
//   * recommendation: "matches preset within ±1°" or "differs by X°,
//     consider advance_offset compensation".
//
// Detects multi-tooth pickups (>2 falls per rev) and bails with
// ERR_MULTI_TOOTH — this CDI cannot drive toothed-wheel motors.
//
// Cannot detect: absolute TDC position (needs strobe), 2T vs 4T.
#pragma once

#include <cstdint>

namespace cdi::core::pickup_cal {

enum class State : uint8_t {
    IDLE             = 0,
    COLLECTING       = 1,
    DONE             = 2,
    ERR_TIMEOUT      = 3,
    ERR_MULTI_TOOTH  = 4,
};

struct Status {
    State    state;
    uint8_t  good_revs;        // valid measurements accepted
    uint8_t  target_revs;      // need this many
    uint32_t total_events;     // raw edge count seen
    uint32_t total_falls;
    uint32_t skipped_jitter;   // revs rejected by steady-state filter
    uint32_t skipped_rpm;      // revs rejected by RPM window
    float    width_mean_deg;
    float    width_median_deg;
    float    width_min_deg;
    float    width_max_deg;
    float    width_stddev_deg;
    float    confidence_pct;
    uint16_t rpm_mean;
    uint32_t elapsed_ms;
};

bool start();
void stop();
void tick();
Status status();

constexpr uint8_t  DEFAULT_TARGET_REVS = 50;
constexpr uint32_t DEFAULT_TIMEOUT_MS  = 30000;
constexpr uint16_t DEFAULT_MIN_RPM     = 800;
constexpr uint16_t DEFAULT_MAX_RPM     = 3000;
constexpr float    DEFAULT_JITTER_PCT  = 5.0f;

void setTarget(uint8_t revs);
void setTimeoutMs(uint32_t ms);
void setRpmWindow(uint16_t min_rpm, uint16_t max_rpm);
void setMaxJitterPct(float pct);

} // namespace cdi::core::pickup_cal
