#include "core/pulser_input.h"

#include <Arduino.h>

#include "config.h"
#include "pinmap.h"
#include "util/ring_buffer.h"
#include "core/spark_scheduler.h"

namespace cdi::core::pulser {
namespace {

// Ignition ring is small — RPM consumer drains every loop tick.
constexpr size_t IGN_RING_N   = 32;
// Scope ring is larger because UI consumer runs at ~30 fps frame builder
// rate, so it must hold one frame's worth of edges (~30 ms × 4 edges/fire
// × 200 fire/s ≈ 24 edges) plus headroom.
constexpr size_t SCOPE_RING_N = 128;

using IgnRing   = cdi::util::SpscRing<PulserEvent, IGN_RING_N>;
using ScopeRing = cdi::util::SpscRing<PulserEvent, SCOPE_RING_N>;

IgnRing           s_ignRing;
ScopeRing         s_scopeRing;
volatile uint32_t s_total          = 0;
volatile uint32_t s_scopeOverruns  = 0;
volatile bool     s_attached       = false;

// Tier-2 2B: pickup signal-anomaly counter (EMI health indicator).
// Counts only UNAMBIGUOUS anomalies — a CH1 arriving sooner than the
// absolute RPM ceiling allows (spurious leading edge), and a CH2 with no
// pending CH1 (extra/lone trailing edge). Weak-signal MISSING CH2 is NOT
// counted, so a healthy bike reads ~0 even while cranking.
volatile uint32_t     s_pickupAnomalies = 0;
volatile bool         s_awaitingCh2     = false;
volatile cdi::micros_t s_lastCh1AnomTs  = 0;
constexpr uint32_t MIN_VALID_PERIOD_US  = 60000000UL / cdi::config::RPM_MAX_VALID;

// Pin-tied helpers so the ISR reads the current logical level after the
// edge — for CHANGE interrupts, digitalRead() at ISR entry returns the
// level the pin transitioned TO.
void IRAM_ATTR isrCh1() {
    const cdi::micros_t t = (cdi::micros_t)micros();
    const uint8_t lvl     = (uint8_t)digitalRead(cdi::pins::PULSER_CH1);
    PulserEvent ev;
    ev.ts_us   = t;
    ev.channel = cdi::PulserChannel::CH1;
    ev.level   = lvl;
    // Ignition ring carries only CH1 falling edges — the single
    // event class live_stats actually consumes for RPM/timing. CH1
    // rising edges and all CH2 events go only to the scope ring
    // (which the diagnostic UI drains). This keeps the hot-path
    // ring small enough that a few-ms loop pause can't overflow it
    // at high RPM (was ~1000 ev/s with all four edges, now ~270 ev/s
    // at 16k rpm — 8× headroom on the 32-entry ring).
    if (lvl == 0) {
        s_ignRing.pushFromIsr(ev);
        cdi::core::spark::onPulseCh1FromIsr(t);
        // Tier-2 2B: CH1 closer than the absolute RPM ceiling allows =
        // spurious leading edge (EMI). 32-bit modular diff (micros() wraps
        // ~71 min). Don't move the anchor on the spurious edge.
        if (s_lastCh1AnomTs != 0) {
            const uint32_t dt = (uint32_t)t - (uint32_t)s_lastCh1AnomTs;
            if (dt < MIN_VALID_PERIOD_US) { s_pickupAnomalies++; }
            else                          { s_lastCh1AnomTs = t; }
        } else {
            s_lastCh1AnomTs = t;
        }
        s_awaitingCh2 = true;   // a real pass now expects exactly one CH2
    }
    if (!s_scopeRing.pushFromIsr(ev)) s_scopeOverruns++;
    s_total++;
}

void IRAM_ATTR isrCh2() {
    const cdi::micros_t t = (cdi::micros_t)micros();
    const uint8_t lvl     = (uint8_t)digitalRead(cdi::pins::PULSER_CH2);
    PulserEvent ev;
    ev.ts_us   = t;
    ev.channel = cdi::PulserChannel::CH2;
    ev.level   = lvl;
    // CH2 falling edge feeds the spark scheduler's crank-assist path
    // (fire at the fixed mechanical base advance). Cheap no-op there
    // unless crank-assist is enabled AND armed this cycle. Still goes to
    // the scope ring for the diagnostic UI / pickup-cal as before.
    if (lvl == 0) {
        cdi::core::spark::onPulseCh2FromIsr(t);
        // Tier-2 2B: CH2 with no pending CH1 = extra/lone trailing edge
        // (EMI burst). Weak-signal MISSING CH2 is the opposite case and is
        // deliberately NOT counted → no false alarm on a healthy crank.
        if (!s_awaitingCh2) s_pickupAnomalies++;
        s_awaitingCh2 = false;
    }
    if (!s_scopeRing.pushFromIsr(ev)) s_scopeOverruns++;
    s_total++;
}

} // anonymous

void begin() {
    if (s_attached) return;
    pinMode(cdi::pins::PULSER_CH1, INPUT);
    pinMode(cdi::pins::PULSER_CH2, INPUT);
    attachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH1), isrCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH2), isrCh2, CHANGE);
    s_attached = true;
    Serial.println("[pulser] attached CHANGE on GPIO34 (CH1) + GPIO35 (CH2)");
}

void end() {
    if (!s_attached) return;
    detachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH1));
    detachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH2));
    s_attached = false;
    Serial.println("[pulser] detached");
}

bool     isAttached()    { return s_attached; }
bool     tryPop(PulserEvent& out)      { return s_ignRing.pop(out); }
bool     tryPopScope(PulserEvent& out) { return s_scopeRing.pop(out); }
uint32_t totalCount()    { return s_total; }
uint32_t pending()       { return (uint32_t)s_ignRing.size(); }
uint32_t pendingScope()  { return (uint32_t)s_scopeRing.size(); }
uint32_t scopeOverruns() { return s_scopeOverruns; }
uint32_t pickupAnomalies() { return s_pickupAnomalies; }

} // namespace cdi::core::pulser
