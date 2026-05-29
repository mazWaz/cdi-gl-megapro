#include "core/pickup_cal.h"

#include <Arduino.h>
#include <cmath>
#include <algorithm>

#include "config.h"
#include "types.h"
#include "core/pulser_input.h"
#include "core/spark_scheduler.h"
#include "core/rpm_calc.h"

namespace cdi::core::pickup_cal {
namespace {

// At most 100 measurements stored — enough for median + stats with
// good outlier rejection. Each measurement is 4 B float.
constexpr size_t MAX_MEASUREMENTS = 100;

// Width sanity bounds (degrees). Anything outside is treated as noise.
constexpr float WIDTH_MIN_DEG = 5.0f;
constexpr float WIDTH_MAX_DEG = 50.0f;

// Working state for a single revolution being assembled from edges.
struct RevBuilder {
    cdi::micros_t ch1_fall_ts = 0;
    cdi::micros_t ch2_fall_ts = 0;
    uint8_t       extra_falls = 0;
    bool          have_ch1    = false;
    bool          have_ch2    = false;
};

State    s_state              = State::IDLE;
uint8_t  s_target             = DEFAULT_TARGET_REVS;
uint32_t s_timeout_ms         = DEFAULT_TIMEOUT_MS;
uint16_t s_min_rpm            = DEFAULT_MIN_RPM;
uint16_t s_max_rpm            = DEFAULT_MAX_RPM;
float    s_max_jitter_pct     = DEFAULT_JITTER_PCT;

uint32_t s_start_ms           = 0;
uint32_t s_total_events       = 0;
uint32_t s_total_falls        = 0;
uint32_t s_skipped_jitter     = 0;
uint32_t s_skipped_rpm        = 0;

// We need PREVIOUS, CURRENT, NEXT CH1 timestamps and the CURRENT
// pulse's CH2 fall to compute bidirectional width. Keep a small
// rolling window.
RevBuilder    s_cur;
cdi::micros_t s_prev_ch1_ts   = 0;
bool          s_have_prev_ch1 = false;

float    s_widths[MAX_MEASUREMENTS];
uint32_t s_periods_us[MAX_MEASUREMENTS];
uint8_t  s_n                   = 0;

void resetAll() {
    s_state          = State::IDLE;
    s_start_ms       = 0;
    s_total_events   = 0;
    s_total_falls    = 0;
    s_skipped_jitter = 0;
    s_skipped_rpm    = 0;
    s_cur            = RevBuilder{};
    s_prev_ch1_ts    = 0;
    s_have_prev_ch1  = false;
    s_n              = 0;
}

// Called when a CH1 fall closes the current revolution. `prev_ch1_ts`
// is the CH1 fall that OPENED this revolution. `now_ch1_ts` is the
// CH1 fall that closes it (i.e. opens the next). We need a third
// timestamp — the CH1 fall AFTER `now_ch1_ts` — to do bidirectional.
// We defer the measurement: when current rev closes, we just stash
// the (gap, prev_period) pair, and finalize it on the NEXT close
// when we know next_period.
struct Pending {
    cdi::micros_t open_ts;
    cdi::micros_t gap_us;
    uint32_t      prev_period_us;   // close - open
    uint8_t       extra_falls;
    bool          valid;
};
Pending s_pending = {};

void finalizePending(uint32_t next_period_us) {
    if (!s_pending.valid) return;

    // Multi-tooth check — incompatible toothed-wheel pickup. THE genuine
    // safety case: the firmware has been firing on a geometry it cannot
    // drive (phantom multi-edge per rev), so cut spark immediately here.
    if (s_pending.extra_falls > 0) {
        cdi::core::spark::setArmed(false);
        s_state = State::ERR_MULTI_TOOTH;
        return;
    }

    const uint32_t prev = s_pending.prev_period_us;
    const uint32_t next = next_period_us;
    if (prev == 0 || next == 0) { s_pending.valid = false; return; }

    // Steady-state filter
    const uint32_t mean_period = (prev + next) / 2;
    const uint32_t diff        = (prev > next) ? (prev - next) : (next - prev);
    const float    jitter_pct  = 100.0f * (float)diff / (float)mean_period;
    if (jitter_pct > s_max_jitter_pct) {
        s_skipped_jitter++;
        s_pending.valid = false;
        return;
    }

    // RPM window
    const uint32_t rpm = 60000000UL / mean_period;
    if (rpm < s_min_rpm || rpm > s_max_rpm) {
        s_skipped_rpm++;
        s_pending.valid = false;
        return;
    }

    // Bidirectional width
    const float width = 360.0f * (float)s_pending.gap_us / (float)mean_period;
    if (width < WIDTH_MIN_DEG || width > WIDTH_MAX_DEG) {
        s_pending.valid = false;
        return;
    }

    if (s_n < MAX_MEASUREMENTS) {
        s_widths[s_n]     = width;
        s_periods_us[s_n] = mean_period;
        s_n++;
    }
    s_pending.valid = false;

    if (s_n >= s_target) s_state = State::DONE;
}

void onRevClose(cdi::micros_t closing_ch1_ts) {
    if (!s_have_prev_ch1) {
        // First CH1 fall — just record, can't measure yet.
        s_have_prev_ch1 = true;
        s_prev_ch1_ts   = closing_ch1_ts;
        return;
    }

    const uint32_t period_us = (uint32_t)(closing_ch1_ts - s_prev_ch1_ts);

    // First, complete any pending measurement (closing pulse provides
    // its next-period).
    finalizePending(period_us);
    if (s_state != State::COLLECTING) return;

    // Start a new pending measurement using THIS closed rev as the
    // "current pulse" — its gap & prev_period (= the period we just
    // computed) are known; next_period will arrive on next CH1 close.
    if (s_cur.have_ch1 && s_cur.have_ch2) {
        s_pending.open_ts        = s_cur.ch1_fall_ts;
        s_pending.gap_us         = (uint32_t)(s_cur.ch2_fall_ts - s_cur.ch1_fall_ts);
        s_pending.prev_period_us = period_us;
        s_pending.extra_falls    = s_cur.extra_falls;
        s_pending.valid          = true;
    } else {
        s_pending.valid = false;
    }

    // Roll: the closing fall becomes the new prev for next iteration.
    s_prev_ch1_ts = closing_ch1_ts;

    // Reset builder for the rev that opens at closing_ch1_ts.
    s_cur                = RevBuilder{};
    s_cur.ch1_fall_ts    = closing_ch1_ts;
    s_cur.have_ch1       = true;
}

float medianOf(float* arr, uint8_t n) {
    if (n == 0) return 0.0f;
    // Copy to scratch (we don't want to mutate s_widths).
    float scratch[MAX_MEASUREMENTS];
    for (uint8_t i = 0; i < n; i++) scratch[i] = arr[i];
    std::sort(scratch, scratch + n);
    if (n & 1) return scratch[n / 2];
    return 0.5f * (scratch[n/2 - 1] + scratch[n/2]);
}

} // anonymous

bool start() {
    if (!cdi::core::pulser::isAttached()) return false;
    // Do NOT disarm spark here. Calibration measures magnet geometry
    // purely from pulser edge timing (scope ring) — independent of whether
    // the firmware fires — and it REQUIRES the engine to keep idling at a
    // steady RPM for ~target revs. Cutting spark stalled the engine within
    // ~1 s; the decelerating revs then failed the steady-state + RPM-window
    // filters, so the run ALWAYS timed out with 0 good revs and left the
    // ignition off (the "kenapa kalibrasi mati" bug). Spark is force-
    // disarmed only on ERR_MULTI_TOOTH (finalizePending) — the one genuine
    // safety case (incompatible toothed pickup the CDI can't drive).
    resetAll();
    s_state    = State::COLLECTING;
    s_start_ms = millis();
    Serial.printf("[cal] start: target=%u revs, RPM[%u..%u], jitter<%.1f%% "
                  "(spark stays armed — engine must keep idling)\n",
                  s_target, s_min_rpm, s_max_rpm, s_max_jitter_pct);
    return true;
}

void stop() {
    if (s_state == State::COLLECTING) {
        Serial.printf("[cal] cancelled, %u/%u clean revs\n", s_n, s_target);
    }
    resetAll();
}

void tick() {
    if (s_state != State::COLLECTING) return;

    if ((millis() - s_start_ms) > s_timeout_ms) {
        Serial.printf("[cal] timeout, %u/%u clean revs (jitter=%u, rpm=%u dropped)\n",
                      s_n, s_target, s_skipped_jitter, s_skipped_rpm);
        s_state = State::ERR_TIMEOUT;
        return;
    }

    PulserEvent ev;
    while (cdi::core::pulser::tryPopScope(ev)) {
        s_total_events++;
        if (ev.level != 0) continue;
        s_total_falls++;

        if (ev.channel == cdi::PulserChannel::CH1) {
            onRevClose(ev.ts_us);
            if (s_state != State::COLLECTING) break;
        } else {
            // CH2 fall
            if (!s_cur.have_ch1) continue;
            if (s_cur.have_ch2) {
                s_cur.extra_falls++;
            } else {
                s_cur.ch2_fall_ts = ev.ts_us;
                s_cur.have_ch2    = true;
            }
        }
    }
}

Status status() {
    Status s{};
    s.state           = s_state;
    s.good_revs       = s_n;
    s.target_revs     = s_target;
    s.total_events    = s_total_events;
    s.total_falls     = s_total_falls;
    s.skipped_jitter  = s_skipped_jitter;
    s.skipped_rpm     = s_skipped_rpm;
    s.elapsed_ms      = (s_start_ms == 0) ? 0 : (millis() - s_start_ms);

    if (s_n > 0) {
        float sum = 0, mn = s_widths[0], mx = s_widths[0];
        uint64_t per_sum = 0;
        for (uint8_t i = 0; i < s_n; i++) {
            sum += s_widths[i];
            if (s_widths[i] < mn) mn = s_widths[i];
            if (s_widths[i] > mx) mx = s_widths[i];
            per_sum += s_periods_us[i];
        }
        s.width_mean_deg = sum / s_n;
        s.width_min_deg  = mn;
        s.width_max_deg  = mx;
        s.rpm_mean       = (uint16_t)(60000000ULL / (per_sum / s_n));

        // Median + stddev
        s.width_median_deg = medianOf(s_widths, s_n);

        if (s_n >= 2) {
            float var = 0;
            for (uint8_t i = 0; i < s_n; i++) {
                const float d = s_widths[i] - s.width_mean_deg;
                var += d * d;
            }
            var /= s_n;
            s.width_stddev_deg = sqrtf(var);
            const float c = 100.0f * (1.0f - s.width_stddev_deg / s.width_mean_deg);
            s.confidence_pct = (c < 0) ? 0 : (c > 100 ? 100 : c);
        } else {
            s.confidence_pct = 100.0f;
        }
    } else {
        // No good revs yet — report live RPM from the always-running
        // rpm_calc so the user can see whether the engine is inside
        // the calibration window and adjust throttle accordingly.
        s.rpm_mean = (uint16_t)cdi::core::rpm::current();
    }
    return s;
}

void setTarget(uint8_t n) {
    if (n < 5)   n = 5;
    if (n > MAX_MEASUREMENTS) n = MAX_MEASUREMENTS;
    s_target = n;
}
void setTimeoutMs(uint32_t ms) {
    if (ms < 2000)   ms = 2000;
    if (ms > 120000) ms = 120000;
    s_timeout_ms = ms;
}
void setRpmWindow(uint16_t mn, uint16_t mx) {
    if (mn < 100)  mn = 100;
    if (mx > 8000) mx = 8000;
    if (mx <= mn)  mx = mn + 100;
    s_min_rpm = mn;
    s_max_rpm = mx;
}
void setMaxJitterPct(float pct) {
    if (pct < 0.5f) pct = 0.5f;
    if (pct > 20.0f) pct = 20.0f;
    s_max_jitter_pct = pct;
}

} // namespace cdi::core::pickup_cal
