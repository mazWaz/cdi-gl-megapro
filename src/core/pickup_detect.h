// Pickup geometry auto-detector.
//
// Drains the pulser scope ring while ignition is **disarmed** to measure
// pickup geometry directly from edge timestamps. Computes:
//
//   * magnet_width_deg     — angular size of the rotor magnet (CH1 fall → CH2 fall)
//   * width_min/max        — spread across captured revolutions (jitter indicator)
//   * stability_pct        — 100 × (1 − stddev/mean)
//   * revs_collected       — how many clean revolutions seen
//   * multi_tooth_flag     — true if more than 2 falling edges per revolution
//                            (signals Tier-3 toothed-wheel pickup; this CDI
//                            cannot drive such motors)
//
// What it CANNOT detect (needs external reference):
//   * absolute TDC offset  — magnet position vs piston TDC. Requires a strobe
//                            timing light measurement to calibrate.
//   * 2T vs 4T cycle       — pickup pattern is identical; user must pick.
//
// Algorithm: collect raw events, identify CH1-fall ↔ next-CH1-fall as one
// revolution period, find the CH2-fall in between, take the ratio. Drop revs
// where the pattern doesn't match (missing CH2, extra edges, out-of-range
// period). Need a minimum number of clean revs (default 20) before reporting.
//
// Caller flow (typical):
//   1. spark::setArmed(false)            // safety: never fire while detecting
//   2. detect::start()                   // resets state, opens collection window
//   3. user kicks engine, scope ring fills
//   4. detect::tick() called from loop, drains events, accumulates
//   5. UI polls detect::status() (or WS broadcast) until state == DONE
//   6. detect::result() returns final geometry → can be matched to preset library
//   7. detect::stop()                    // releases the scope-ring claim
#pragma once

#include <cstdint>

namespace cdi::core::detect {

enum class State : uint8_t {
    IDLE       = 0,   // no run in progress
    COLLECTING = 1,   // capturing edges
    DONE       = 2,   // enough revs collected; result is valid
    ERR_TIMEOUT = 3,  // not enough revs in the allotted time
    ERR_MULTI_TOOTH = 4,  // >2 falling edges per revolution
};

struct Status {
    State    state;
    uint8_t  revs_collected;     // 0..target
    uint8_t  revs_target;
    uint32_t total_events;       // raw edge count seen
    uint32_t total_falls;        // CH1+CH2 falling edge count
    float    width_mean_deg;     // running mean of magnet width
    float    width_min_deg;
    float    width_max_deg;
    float    stability_pct;      // 100 = perfect, <90 = noisy
    uint32_t period_us_mean;     // approximate (last good rev)
    uint32_t elapsed_ms;
};

// Start a new detection run. spark output is forced disarmed for safety.
// Returns false if pulser ISR isn't attached (mode != IGNITION).
bool start();

// Cancel detection mid-flight. Caller should also re-arm spark if desired
// (it stays disarmed automatically — we don't speak for the user there).
void stop();

// Pump from loop(). Drains scope ring → accumulator → may transition state.
void tick();

// Read current state (cheap, returns a static struct copy).
Status status();

// True when state == DONE — result is meaningful.
bool isDone();

// Default # of clean revolutions required before DONE (20).
constexpr uint8_t DEFAULT_REVS_TARGET = 20;

// Override the target rev count (called before start()).
void setTargetRevs(uint8_t n);

// Absolute timeout for the entire detection run (ms). Beyond this, state
// transitions to ERR_TIMEOUT regardless of revs collected.
constexpr uint32_t DEFAULT_TIMEOUT_MS = 30000;

void setTimeoutMs(uint32_t ms);

} // namespace cdi::core::detect
