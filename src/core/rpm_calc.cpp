#include "core/rpm_calc.h"

#include "config.h"

namespace cdi::core::rpm {
namespace {

constexpr cdi::micros_t MIN_PERIOD_US = 60000000UL / cdi::config::RPM_MAX_VALID;
constexpr cdi::micros_t MAX_PERIOD_US = 60000000UL / cdi::config::RPM_MIN_VALID;

cdi::micros_t s_lastCh1     = 0;
cdi::micros_t s_lastPeriod  = 0;
cdi::rpm_t    s_raw         = 0;
cdi::rpm_t    s_smooth      = 0;

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
        // micros() rollover anomaly described above. In both cases
        // re-anchor s_lastCh1 to the current edge so the next pulse
        // measures a fresh, valid period. tick() will zero s_raw /
        // s_smooth if the gap persists.
        s_lastCh1 = ts;
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
    if (gap > (uint32_t)(MAX_PERIOD_US * 2)) {
        s_raw    = 0;
        s_smooth = 0;
    }
}

cdi::rpm_t    current()       { return s_smooth; }
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
