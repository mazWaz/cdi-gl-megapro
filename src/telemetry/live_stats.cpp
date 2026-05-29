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
#include "core/idle_rumble.h"
#include "core/exhaust_flame.h"
#include "core/alvp.h"
#include "config.h"

constexpr float REV_LIMIT_RETARD_DEG = 10.0f;

namespace cdi::telemetry {

// Last actual fired angle (°×10), recomputed every tick when a valid
// period exists. Read by snapshot()/datalog for diagnostic logging.
// Single 16-bit value — atomic on Xtensa, no lock needed.
static volatile cdi::deg_x10_t s_actualAdvX10 = 0;

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
        adv -= cdi::core::idle_rumble::currentRetardDeg();
        adv -= cdi::core::flame::currentRetardDeg();
        // Global advance trim (T8, compensates HV stage propagation delay).
        adv += cdi::core::spark::advanceOffsetDeg();

        // Per-feature MIN-advance override.
        // Default: clamp ke ADVANCE_MIN_DEG (0° = TDC). Combustion masih
        // selesai sebelum exhaust valve buka → tidak ada flame dari retard.
        //
        // Saat flame::isActive(), allow effective advance ke -20° ATDC
        // (post-TDC fire). Mekanisme:
        //   * Spark fire SETELAH TDC dengan piston turun
        //   * Combustion start saat valve mulai mendekati open
        //   * Sisa burn front bocor ke exhaust manifold panas
        //   * Visible flame keluar dari knalpot
        // Math: delay_deg = max_ref - adv. Untuk adv = -20°,
        //   delay = 32 - (-20) = 52° = valid (< 360°). Scheduler tidak
        //   peduli sign — cuma fire di scheduled micro.
        // Safety: -20° hard cap → combustion gak terjadi sebelum TDC
        //   (no kickback). Exhaust valve sees thermal load — yang sudah
        //   di-warn di UI disclaimer AGRESIF mode.
        const float min_adv_eff = cdi::core::flame::isActive()
            ? -20.0f
            : cdi::config::ADVANCE_MIN_DEG;
        if (adv < min_adv_eff) adv = min_adv_eff;
        if (adv > cdi::config::ADVANCE_MAX_DEG) adv = cdi::config::ADVANCE_MAX_DEG;

        // Pickup's max_advance_ref is per-motor (set by preset::apply
        // or by an auto-cal override). Falls back to the compile-time
        // constant if neither has run yet.
        float delayDeg = cdi::core::pickup::maxAdvanceRef() - adv;
        if (delayDeg < 0)   delayDeg = 0;
        if (delayDeg > 360) delayDeg = 360;

        // Time from CH1 to the moment we want the spark to actually fire.
        const uint32_t spark_delay_us = (uint32_t)((delayDeg / 360.0f) * (float)periodU);

        // ── Effective dwell selection ──
        //
        // Four constraints on dwell for an inductive ignition stage:
        //   1. Thermal: dwell ≤ 40 % of period so primary coil never
        //      stays energized between cycles.
        //   2. Advance preservation: spark fires at end of dwell;
        //      we need dwell ≤ spark_delay_us so the fire-off lands
        //      at the intended crank angle. If dwell > spark_delay,
        //      spark would land AFTER target → retarded (or even
        //      post-TDC) at higher RPM, which is exactly the regime
        //      where TCI dwell exceeds the (max_ref − target_advance)
        //      angular budget on a Honda-style 32 ° BTDC pulser.
        //   3. Spark energy floor: below ~200 µs primary doesn't
        //      charge enough for a useful spark. Honor the floor even
        //      if it means slightly retarded timing.
        //   4. RPM-compensated dwell curve (T10, optional): a per-RPM
        //      override that may only SHORTEN dwell. Composed in (min)
        //      BEFORE constraint 2 so the SINGLE final value flows into
        //      both the effective dwell and the scheduler-delay
        //      subtraction below.
        //
        // For capacitive (CDI/SCR) ignition, constraint #2 doesn't
        // apply — spark fires on the rising edge, dwell is just the
        // trigger pulse width.
        const uint32_t configured_dwell = cdi::core::spark::configuredDwellUs();
        const uint32_t safe_dwell_cap   = (uint32_t)((float)periodU * 0.4f);
        constexpr uint32_t MIN_USEFUL_DWELL = 200;   // µs

        // ALVP boost: at low supply voltage the coil needs longer to
        // saturate to a given primary current — extend dwell to keep
        // spark energy above the misfire floor. Multiplier is 1.0 when
        // ALVP is disabled or vbat is normal, 1.3 when derated. The
        // thermal cap below still bounds the result, so this can never
        // run the coil hotter than the duty-cycle ceiling.
        uint32_t dwell_us = (uint32_t)((float)configured_dwell *
                                       cdi::core::alvp::dwellMultiplier());
        if (dwell_us > safe_dwell_cap) dwell_us = safe_dwell_cap;

        // Constraint 4 — RPM-compensated dwell curve (T10).
        // MUST be folded in HERE, before the advance-budget cap and the
        // scheduler_delay computation, so the same final dwell value is
        // BOTH written as the effective dwell AND subtracted from
        // spark_delay below. Previously the curve was applied after
        // setNextDelayUs(), leaving scheduler_delay computed against the
        // pre-curve dwell: on a TCI stage the fire-off then landed early
        // by (pre_curve_dwell − curve_dwell) µs → spark OVER-ADVANCED
        // (e.g. ~7° at 2000 rpm with a 1000 µs idle-curve point, exactly
        // the documented "shorter dwell at idle" use case). The curve may
        // only ever SHORTEN dwell (take the min), so the rider can never
        // exceed their configured cap through it; toggling the curve off
        // restores the user's s_dwellUs intent untouched.
        if (cdi::core::dwell::isEnabled()) {
            const uint32_t curve_dwell = cdi::core::dwell::lookup(r_inst);
            if (curve_dwell < dwell_us) dwell_us = curve_dwell;
        }

        if (cdi::core::spark::inductive()) {
            // Pre-compute spark_delay to check the advance constraint.
            // Note: this is the same value used below — we factor it
            // out so the dwell cap can be aware of it.
            uint32_t target_spark_delay = (uint32_t)((delayDeg / 360.0f) * (float)periodU);
            if (dwell_us > target_spark_delay && target_spark_delay >= MIN_USEFUL_DWELL) {
                // Shorten dwell to match target advance. Stays above
                // the useful-spark floor.
                dwell_us = target_spark_delay;
            } else if (target_spark_delay < MIN_USEFUL_DWELL) {
                // Target advance is so close to max_ref that even
                // 200 µs dwell pushes spark past the intended angle.
                // Accept the retard — set dwell to the floor and let
                // the spark land a few degrees retarded from map.
                dwell_us = MIN_USEFUL_DWELL;
            }
        }
        if (dwell_us < MIN_USEFUL_DWELL) dwell_us = MIN_USEFUL_DWELL;
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
        // Pass periodU so the spark ISR's period-drift gate can
        // reject this delay if the next CH1's instantaneous period
        // is dramatically different — cranking RPM is unstable and
        // CH1-to-CH1 RPM can swing >2× during kickstart.
        cdi::core::spark::setNextDelayUs(scheduler_delay_us, (uint32_t)periodU);

        // ── Diagnostic: print effective spark angle to Serial once per
        // second so a USB-connected user can verify the firmware is
        // actually firing where the map says it should. Disable in
        // release by setting CORE_DEBUG_LEVEL=0 in platformio.ini.
        // Actual spark angle that will land = max_ref - (sched_delay+effective_dwell)/period × 360
        // Recompute EVERY tick so datalog captures the real fired angle
        // per sample (not just the 1 Hz serial print). Diverges from
        // target when dwell is capped or advance is near max_ref.
        const float fired_angle = cdi::core::pickup::maxAdvanceRef() -
            ((float)(scheduler_delay_us + dwell_us) / (float)periodU) * 360.0f;
        s_actualAdvX10 = (cdi::deg_x10_t)(fired_angle * 10.0f +
                                          (fired_angle >= 0 ? 0.5f : -0.5f));

        static uint32_t s_lastDiagMs = 0;
        const uint32_t now_ms = millis();
        if (now_ms - s_lastDiagMs >= 1000) {
            s_lastDiagMs = now_ms;
            Serial.printf("[ign] rpm=%u target=%.1f° actual=%.1f° dwell=%uµs(cfg=%u)\n",
                          (unsigned)r_inst, adv, fired_angle,
                          (unsigned)dwell_us, (unsigned)configured_dwell);
        }
    } else {
        // No valid period (engine stopped / pre-first-pulse) — clear
        // the diagnostic angle so the log doesn't show a stale fire
        // angle when nothing is firing.
        s_actualAdvX10 = 0;
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
    s.actual_advance_x10 = s_actualAdvX10;   // real fired angle (diagnostic)
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
