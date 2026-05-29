#include "core/rpm_calc.h"

#include <Arduino.h>     // IRAM_ATTR

#include "config.h"

namespace cdi::core::rpm {
namespace {

constexpr cdi::micros_t MIN_PERIOD_US = 60000000UL / cdi::config::RPM_MAX_VALID;
constexpr cdi::micros_t MAX_PERIOD_US = 60000000UL / cdi::config::RPM_MIN_VALID;

// volatile: ditulis core-1 (onPulseCh1/tick di loop), tapi dibaca
// lintas-core oleh edge_snapshot saver task (core 0, via flashWriteSafe)
// dan dari QS GPIO ISR (current()). Konvensi cross-core codebase (audit
// M1). Word-aligned → store/load atomik di Xtensa.
volatile cdi::micros_t s_lastCh1     = 0;
volatile cdi::micros_t s_lastPeriod  = 0;
volatile cdi::rpm_t    s_raw         = 0;
volatile cdi::rpm_t    s_smooth      = 0;

} // anonymous

void onPulseCh1(cdi::micros_t ts) {
    if (s_lastCh1 == 0) { s_lastCh1 = ts; return; }

    // Do the diff in uint32 modular arithmetic.
    //
    // Why: Arduino `micros()` returns uint32_t and wraps every
    // ~71 minutes. The ISR stuffs that into cdi::micros_t (uint64)
    // by zero-extension. Subtracting two zero-extended values in
    // uint64 underflows to ~1.8e19 µs across a wrap-boundary, even
    // though the true elapsed time is tiny. Casting both back to
    // uint32 before the subtraction lets the natural unsigned wrap
    // produce the correct small delta. This means we lose at most
    // one bad period per 71-minute mark, instead of permanently
    // freezing the RPM reading.
    const uint32_t period = (uint32_t)ts - (uint32_t)s_lastCh1;

    if (period < (uint32_t)MIN_PERIOD_US) {
        // Noise — short period. Keep s_lastCh1 anchored so a single
        // glitch doesn't poison the next real edge's period calc.
        return;
    }
    if (period > (uint32_t)MAX_PERIOD_US) {
        // Either engine timeout (kickstart pause, stall) or the
        // micros() rollover anomaly described above. Re-anchor
        // s_lastCh1 to the current edge so the next pulse measures
        // a fresh valid period. Clear s_lastPeriod too — otherwise
        // live_stats reads the stale value (from before the stall),
        // computes a delay based on the previous RPM regime, and
        // primes the spark ISR with a wrong-angle fire on the
        // second post-restart pulse.
        // Also clear s_raw / s_smooth so snapshot() can't keep
        // reporting a pre-stall RPM after the engine has actually
        // stopped — otherwise datalog & UI freeze at the last seen
        // value until tick()'s longer timeout fires.
        s_lastCh1    = ts;
        s_lastPeriod = 0;
        s_raw        = 0;
        s_smooth     = 0;
        return;
    }
    s_lastCh1 = ts;
    s_lastPeriod = period;

    uint32_t inst = (uint32_t)(60000000ULL / period);
    if (inst > 65535) inst = 65535;
    s_raw = (cdi::rpm_t)inst;

    // EMA: smooth = (7*smooth + new) / 8
    uint32_t blended = (uint32_t)s_smooth * 7 + inst;
    s_smooth = (cdi::rpm_t)(blended >> 3);
}

void tick(cdi::micros_t now_us) {
    if (s_lastCh1 == 0) return;
    // 32-bit modular diff — see comment in onPulseCh1.
    const uint32_t gap = (uint32_t)now_us - (uint32_t)s_lastCh1;

    // ── Two SEPARATE staleness horizons ──
    //
    // 1. DISPLAY (s_raw / s_smooth): zero quickly (NO_SIGNAL_TIMEOUT_MS,
    //    500 ms) so datalog & UI don't freeze on a pre-stall value
    //    after the engine actually stops. This is purely cosmetic /
    //    telemetry — nothing in the spark hot-path reads these.
    //
    // 2. SPARK TIMING (s_lastPeriod): keep the ORIGINAL long horizon
    //    (MAX_PERIOD_US * 2 ≈ 4 s). s_lastPeriod gates live_stats'
    //    delay priming (live_stats.cpp: `if (periodU > 0)`). Kick-start
    //    CH1 periods legitimately swing 72 ms → 1.8 s between falls
    //    (see spark_scheduler.cpp), so zeroing this at 500 ms gates
    //    spark priming mid-crank and the engine won't catch. Only a
    //    genuine multi-second stall should clear it.
    constexpr uint32_t DISPLAY_STALE_US =
        (uint32_t)cdi::config::NO_SIGNAL_TIMEOUT_MS * 1000UL;   // 500 ms
    if (gap > DISPLAY_STALE_US) {
        s_raw    = 0;
        s_smooth = 0;
    }
    if (gap > (uint32_t)(MAX_PERIOD_US * 2)) {
        s_lastPeriod = 0;   // gates live_stats delay computation
    }
}

// IRAM_ATTR: dipanggil dari quickshifter GPIO ISR (audit H2) — wajib
// IRAM-resident agar tidak fetch flash saat erase/write berlangsung.
cdi::rpm_t IRAM_ATTR current() { return s_smooth; }
cdi::rpm_t    raw()           { return s_raw; }
cdi::micros_t lastCh1Us()     { return s_lastCh1; }
cdi::micros_t lastPeriodUs()  { return s_lastPeriod; }

void reset() {
    s_lastCh1    = 0;
    s_lastPeriod = 0;
    s_raw        = 0;
    s_smooth     = 0;
}

} // namespace cdi::core::rpm
