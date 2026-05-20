#include "core/pickup_detect.h"

#include <Arduino.h>
#include <cmath>

#include "config.h"
#include "types.h"
#include "core/pulser_input.h"
#include "core/spark_scheduler.h"
#include "core/mode.h"

namespace cdi::core::detect {
namespace {

// Sanity bounds for one revolution period (µs). Below ~50 RPM or above
// ~15000 RPM we treat the rev as garbage and skip it.
constexpr uint32_t PERIOD_MIN_US = 60000000UL / 15000;   // ~4000 µs  (15000 RPM ceiling)
constexpr uint32_t PERIOD_MAX_US = 60000000UL / 50;      // 1.2 s     (50 RPM floor)

// Width sanity — magnets in the wild range ~5°..40°. Anything outside is
// almost certainly noise or a different pickup geometry.
constexpr float WIDTH_MIN_DEG = 5.0f;
constexpr float WIDTH_MAX_DEG = 50.0f;

// Per-revolution working state — we keep timestamps of falls seen since
// the last reference CH1 fall, then collapse on the next CH1 fall.
struct RevAccum {
    cdi::micros_t ch1_last_fall_ts = 0;
    cdi::micros_t ch2_fall_ts      = 0;
    uint8_t       extra_falls      = 0;   // anything beyond CH1+CH2 falls
    bool          have_ch1         = false;
    bool          have_ch2         = false;
};

// Accumulated results across revs.
struct Stats {
    uint8_t  revs       = 0;
    float    width_sum  = 0;
    float    width_sq_sum = 0;       // for stddev
    float    width_min  = 1e9f;
    float    width_max  = 0;
    uint32_t period_last = 0;
};

State    s_state            = State::IDLE;
uint8_t  s_revs_target      = DEFAULT_REVS_TARGET;
uint32_t s_timeout_ms       = DEFAULT_TIMEOUT_MS;
uint32_t s_start_ms         = 0;
uint32_t s_total_events     = 0;
uint32_t s_total_falls      = 0;
RevAccum s_acc;
Stats    s_stats;

void resetAll() {
    s_state        = State::IDLE;
    s_start_ms     = 0;
    s_total_events = 0;
    s_total_falls  = 0;
    s_acc          = RevAccum{};
    s_stats        = Stats{};
}

void onRevComplete(cdi::micros_t ch1_now_ts) {
    if (!s_acc.have_ch1)             return;       // no opening reference
    if (!s_acc.have_ch2)             return;       // missing CH2 → skip
    if (s_acc.extra_falls > 0) {
        // Multi-tooth pickup detected — abort whole run.
        s_state = State::ERR_MULTI_TOOTH;
        return;
    }
    const uint32_t period_us = (uint32_t)(ch1_now_ts - s_acc.ch1_last_fall_ts);
    if (period_us < PERIOD_MIN_US || period_us > PERIOD_MAX_US) return;

    const uint32_t mag_us = (uint32_t)(s_acc.ch2_fall_ts - s_acc.ch1_last_fall_ts);
    if (mag_us == 0 || mag_us >= period_us) return;   // sanity

    const float width_deg = 360.0f * (float)mag_us / (float)period_us;
    if (width_deg < WIDTH_MIN_DEG || width_deg > WIDTH_MAX_DEG) return;

    s_stats.revs++;
    s_stats.width_sum    += width_deg;
    s_stats.width_sq_sum += width_deg * width_deg;
    if (width_deg < s_stats.width_min) s_stats.width_min = width_deg;
    if (width_deg > s_stats.width_max) s_stats.width_max = width_deg;
    s_stats.period_last = period_us;

    if (s_stats.revs >= s_revs_target) {
        s_state = State::DONE;
    }
}

} // anonymous

bool start() {
    if (!cdi::core::pulser::isAttached()) return false;
    cdi::core::spark::setArmed(false);   // safety while detecting
    resetAll();
    s_state    = State::COLLECTING;
    s_start_ms = millis();
    Serial.println("[detect] started — kick engine to collect revs");
    return true;
}

void stop() {
    if (s_state == State::COLLECTING) {
        Serial.printf("[detect] cancelled, revs=%u/%u\n",
                      s_stats.revs, s_revs_target);
    }
    resetAll();
}

void tick() {
    if (s_state != State::COLLECTING) return;

    // Timeout check.
    if ((millis() - s_start_ms) > s_timeout_ms) {
        s_state = State::ERR_TIMEOUT;
        Serial.printf("[detect] timeout — only %u/%u revs collected\n",
                      s_stats.revs, s_revs_target);
        return;
    }

    PulserEvent ev;
    while (cdi::core::pulser::tryPopScope(ev)) {
        s_total_events++;
        if (ev.level != 0) continue;            // only falling edges matter
        s_total_falls++;

        if (ev.channel == cdi::PulserChannel::CH1) {
            // CH1 fall — close previous revolution if we already had one open.
            if (s_acc.have_ch1) {
                onRevComplete(ev.ts_us);
                if (s_state != State::COLLECTING) break;
            }
            // Open a new revolution.
            s_acc = RevAccum{};
            s_acc.ch1_last_fall_ts = ev.ts_us;
            s_acc.have_ch1         = true;
        } else if (ev.channel == cdi::PulserChannel::CH2) {
            if (!s_acc.have_ch1) continue;       // CH2 with no opening CH1
            if (s_acc.have_ch2) {
                // Second CH2 in the same window — likely multi-tooth.
                s_acc.extra_falls++;
            } else {
                s_acc.ch2_fall_ts = ev.ts_us;
                s_acc.have_ch2    = true;
            }
        }
    }
}

Status status() {
    Status s{};
    s.state          = s_state;
    s.revs_collected = s_stats.revs;
    s.revs_target    = s_revs_target;
    s.total_events   = s_total_events;
    s.total_falls    = s_total_falls;
    s.period_us_mean = s_stats.period_last;
    s.elapsed_ms     = (s_start_ms == 0) ? 0 : (millis() - s_start_ms);

    if (s_stats.revs > 0) {
        s.width_mean_deg = s_stats.width_sum / (float)s_stats.revs;
        s.width_min_deg  = s_stats.width_min;
        s.width_max_deg  = s_stats.width_max;
        // stddev → stability%. Cap at 100, floor at 0.
        if (s_stats.revs >= 2) {
            const float var = (s_stats.width_sq_sum / s_stats.revs)
                            - (s.width_mean_deg * s.width_mean_deg);
            const float sd  = (var > 0) ? sqrtf(var) : 0;
            const float stab = 100.0f * (1.0f - sd / s.width_mean_deg);
            s.stability_pct = (stab < 0) ? 0 : (stab > 100 ? 100 : stab);
        } else {
            s.stability_pct = 100.0f;
        }
    }
    return s;
}

bool isDone() { return s_state == State::DONE; }

void setTargetRevs(uint8_t n) {
    if (n < 3) n = 3;
    if (n > 100) n = 100;
    s_revs_target = n;
}

void setTimeoutMs(uint32_t ms) {
    if (ms < 1000)   ms = 1000;
    if (ms > 120000) ms = 120000;
    s_timeout_ms = ms;
}

} // namespace cdi::core::detect
