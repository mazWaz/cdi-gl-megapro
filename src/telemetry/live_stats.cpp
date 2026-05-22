#include "telemetry/live_stats.h"

#include <Arduino.h>

#include "core/mode.h"
#include "core/pulser_input.h"
#include "core/rpm_calc.h"
#include "core/advance_map.h"
#include "core/spark_scheduler.h"
#include "core/pickup.h"
#include "core/safety.h"
#include "core/dwell_curve.h"
#include "core/shift_light.h"
#include "core/launch_control.h"
#include "core/quickshifter.h"
#include "core/backfire.h"
#include "core/alvp.h"
#include "config.h"

constexpr float REV_LIMIT_RETARD_DEG = 10.0f;

namespace cdi::telemetry {

void tick() {
    PulserEvent ev;
    while (cdi::core::pulser::tryPop(ev)) {
        // Pulser ISR now reports both edges (CHANGE). RPM/spark only
        // care about the FALLING edge (level==0) to stay consistent
        // with the pre-refactor behaviour and the optocoupler conduct
        // semantics.
        if (ev.channel == cdi::PulserChannel::CH1 && ev.level == 0) {
            cdi::core::rpm::onPulseCh1(ev.ts_us);
        }
    }
    cdi::core::rpm::tick((cdi::micros_t)micros());

    // Update spark fire delay from latest period + advance map.
    //
    // IMPORTANT: use the INSTANTANEOUS RPM derived from the latest
    // valid period (not the EMA-smoothed RPM). Smoothing has a lag
    // of several CH1 cycles, which on a fast deceleration (e.g. kick
    // ends, RPM drops 3000 → 500) leaves the advance lookup believing
    // we're still at high RPM. The advance map returns the
    // high-RPM value (~21° BTDC) for a now-cranking engine that
    // wants ~8° BTDC — too advanced at low RPM → kickback during
    // compression. Instantaneous tracking eliminates the lag.
    //
    // IMPORTANT: when ignition is INDUCTIVE (default), the actual
    // spark fires on the FALL edge after dwell. The scheduler delay
    // points to the RISING edge (start of dwell), so we must
    // subtract dwell_us from the CH1→spark delay before sending it
    // to the scheduler. Otherwise the fall transition lands
    // dwell_us LATE → effective spark is retarded by dwell_deg
    // (e.g. 22° at 1500 RPM with 2.5 ms dwell — engine cannot run).
    cdi::micros_t periodU = cdi::core::rpm::lastPeriodUs();
    if (periodU > 0) {
        // Instantaneous RPM from the most recent valid period.
        cdi::rpm_t r_inst = (cdi::rpm_t)(60000000ULL / periodU);
        if (r_inst > 65535) r_inst = 65535;

        float adv = cdi::core::advance::active().lookup(r_inst);
        // Cut-mode retard (T9 — configurable per active cut mode).
        adv -= cdi::core::safety::currentRetardDeg();
        adv -= cdi::core::backfire::currentRetardDeg();
        // Global advance trim (T8, compensates HV stage propagation delay).
        adv += cdi::core::spark::advanceOffsetDeg();
        if (adv < cdi::config::ADVANCE_MIN_DEG) adv = cdi::config::ADVANCE_MIN_DEG;
        if (adv > cdi::config::ADVANCE_MAX_DEG) adv = cdi::config::ADVANCE_MAX_DEG;

        // Pickup's max_advance_ref is per-motor (set by preset::apply
        // or by an auto-cal override). Falls back to the compile-time
        // constant if neither has run yet.
        float delayDeg = cdi::core::pickup::maxAdvanceRef() - adv;
        if (delayDeg < 0)   delayDeg = 0;
        if (delayDeg > 360) delayDeg = 360;

        // Time from CH1 to the moment we want the spark to actually fire.
        const uint32_t spark_delay_us = (uint32_t)((delayDeg / 360.0f) * (float)periodU);

        // ── Auto-cap dwell to a safe fraction of period ──
        // At high RPM the configured dwell can exceed the time
        // available between CH1 events, which causes fire-off of
        // cycle N to overlap with fire-on of cycle N+1 — primary
        // coil ends up energized continuously, generates heat well
        // above its rated dissipation, and spark timing drifts
        // unpredictably. Cap effective dwell at 40% of period so
        // there's always headroom for the next charge cycle. The
        // user-configured value is preserved (in s_dwellUs) and
        // restored automatically when RPM drops.
        const uint32_t configured_dwell = cdi::core::spark::configuredDwellUs();
        const uint32_t safe_dwell_cap   = (uint32_t)((float)periodU * 0.4f);
        const uint32_t dwell_us         = (configured_dwell < safe_dwell_cap)
                                        ? configured_dwell
                                        : safe_dwell_cap;
        cdi::core::spark::setEffectiveDwellUs(dwell_us);

        // For inductive ignition, the GPIO HIGH→LOW transition (end
        // of dwell) is when the spark fires. Schedule fire-on so the
        // fire-off lands at spark_delay_us.
        uint32_t scheduler_delay_us;
        if (cdi::core::spark::inductive()) {
            scheduler_delay_us = (spark_delay_us > dwell_us)
                               ? (spark_delay_us - dwell_us)
                               : 0;
        } else {
            // Capacitive / CDI / SCR — spark fires on the rising
            // edge; dwell is just the trigger-pulse width, no
            // subtraction.
            scheduler_delay_us = spark_delay_us;
        }
        cdi::core::spark::setNextDelayUs(scheduler_delay_us);

        // T10: dwell compensation by RPM curve.
        if (cdi::core::dwell::isEnabled()) {
            uint16_t dwell = cdi::core::dwell::lookup(r_inst);
            cdi::core::spark::setDwellUs(dwell);
        }
    }
}

LiveStats snapshot() {
    LiveStats s;
    s.mode           = cdi::core::mode::current();
    s.rpm            = cdi::core::rpm::current();
    s.rpm_raw        = cdi::core::rpm::raw();
    float adv        = cdi::core::advance::active().lookup(s.rpm);
    // Display must match actual fired advance — same chain as tick().
    adv -= cdi::core::safety::currentRetardDeg();
    adv += cdi::core::spark::advanceOffsetDeg();
    if (adv < cdi::config::ADVANCE_MIN_DEG) adv = cdi::config::ADVANCE_MIN_DEG;
    if (adv > cdi::config::ADVANCE_MAX_DEG) adv = cdi::config::ADVANCE_MAX_DEG;
    s.target_advance_x10 = (cdi::deg_x10_t)(adv * 10.0f + 0.5f);
    s.pulser_count   = cdi::core::pulser::totalCount();
    s.pulser_pending = cdi::core::pulser::pending();
    s.uptime_ms      = millis();
    s.free_heap      = ESP.getFreeHeap();
    s.armed          = cdi::core::spark::isArmed() ? 1 : 0;
    s.fire_count     = cdi::core::spark::totalFires();
    int32_t j        = cdi::core::spark::lastJitterUs();
    if (j > 32767) j = 32767; if (j < -32768) j = -32768;
    s.last_jitter_us = (int16_t)j;
    uint8_t f = 0;
    if (cdi::core::safety::isRevLimited()) f |= 0x01;
    if (cdi::core::safety::noSignal())     f |= 0x02;
    if (cdi::core::safety::overRevCut())   f |= 0x04;
    s.safety_flags = f;
    s.main_limit_rpm    = (uint16_t)cdi::core::safety::mainLimitRpm();
    s.overrev_limit_rpm = (uint16_t)cdi::core::safety::overrevLimitRpm();
    s.dwell_us          = (uint16_t)cdi::core::spark::dwellUs();
    s.advance_offset_x10= (cdi::deg_x10_t)(cdi::core::spark::advanceOffsetDeg() * 10.0f);
    s.cut_mode          = (uint8_t)cdi::core::safety::mainCutMode();
    s.retard_half_deg   = (uint8_t)(cdi::core::safety::mainRetardDeg() * 2.0f + 0.5f);
    s.pattern_fire_n    = cdi::core::safety::patternFireN();
    s.pattern_skip_n    = cdi::core::safety::patternSkipN();
    s.shift_state       = cdi::core::shift_light::state();
    uint8_t f2 = 0;
    if (cdi::core::shift_light::isEnabled()) f2 |= 0x01;
    if (cdi::core::dwell::isEnabled())       f2 |= 0x02;
    if (cdi::core::launch::isEnabled())      f2 |= 0x04;
    if (cdi::core::quickshift::isEnabled())  f2 |= 0x08;
    if (cdi::core::launch::isActive())       f2 |= 0x10;
    if (cdi::core::quickshift::isActive())   f2 |= 0x20;
    s.flags2            = f2;
    s.shift_rpm_warn    = cdi::core::shift_light::rpmWarn();
    s.shift_rpm_shift   = cdi::core::shift_light::rpmShift();
    s.launch_rpm        = cdi::core::launch::launchRpm();
    s.qs_cut_ms         = cdi::core::quickshift::cutDurationMs();
    s.qs_count          = cdi::core::quickshift::totalShifts();
    s.backfire_trigger  = (uint8_t)cdi::core::backfire::trigger();
    uint8_t f3 = 0;
    if (cdi::core::backfire::isEnabled())     f3 |= 0x01;
    if (cdi::core::backfire::isActive())      f3 |= 0x02;
    if (cdi::core::backfire::randomPattern()) f3 |= 0x04;
    s.flags3            = f3;
    s.bf_rpm_lo         = cdi::core::backfire::rpmLo();
    s.bf_rpm_hi         = cdi::core::backfire::rpmHi();
    s.bf_retard_half_deg= (uint8_t)(cdi::core::backfire::retardDeg() * 2.0f + 0.5f);
    s.bf_duration_ms    = cdi::core::backfire::durationMs();
    s.vbat_mv           = cdi::core::alvp::vbatMv();
    s.alvp_state        = (uint8_t)cdi::core::alvp::state();
    s.alvp_derate_v_x10 = (uint8_t)(cdi::core::alvp::derateThresholdV() * 10.0f + 0.5f);
    s.alvp_disarm_v_x10 = (uint8_t)(cdi::core::alvp::disarmThresholdV() * 10.0f + 0.5f);
    s.flags4            = (cdi::core::alvp::isEnabled() ? 0x01 : 0)
                        | (cdi::core::spark::autoArm()  ? 0x02 : 0);
    s.alvp_derate_rpm   = cdi::core::alvp::derateLimitRpm();
    return s;
}

void reset() {
    cdi::core::rpm::reset();
}

} // namespace cdi::telemetry
