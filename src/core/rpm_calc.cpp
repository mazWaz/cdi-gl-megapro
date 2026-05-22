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

    cdi::micros_t period = ts - s_lastCh1;

    if (period < MIN_PERIOD_US || period > MAX_PERIOD_US) {
        // Noise / hiccup — keep previous timestamp AND smoothed value.
        // Critical: do NOT advance s_lastCh1 to a noise event, otherwise
        // the next real edge computes period from the noise timestamp
        // (short period → also rejected) and the cascade poisons every
        // subsequent edge until rpm decays to zero. Keeping s_lastCh1
        // anchored to the last KNOWN-GOOD edge lets a single noise spike
        // self-recover on the next real pulse.
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
    if (now_us - s_lastCh1 > MAX_PERIOD_US * 2) {
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
