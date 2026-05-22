#include "core/quickshifter.h"

#include <Arduino.h>

#include "pinmap.h"
#include "core/rpm_calc.h"

namespace cdi::core::quickshift {
namespace {

// Cross-core: WS handlers on core 0 toggle s_enabled and set the
// thresholds; the GPIO ISR on core 1 and shouldCut() called from
// the spark ISR (also core 1) read them. Mark volatile so the
// compiler can't cache stale values past a function-call boundary.
// 16/32-bit aligned writes on Xtensa are atomic at the word level.
volatile bool       s_enabled    = false;
bool                s_attached   = false;   // touched only from main task
volatile uint16_t   s_cutMs      = 65;
volatile cdi::rpm_t s_minRpm     = 4000;
volatile cdi::rpm_t s_maxRpm     = 12000;

volatile uint32_t s_cutUntilMs = 0;
volatile uint32_t s_shiftCount = 0;

void IRAM_ATTR isrTrigger() {
    // RPM check uses cached snapshot read — rpm::current() is just
    // a memory load, ISR-safe.
    cdi::rpm_t r = cdi::core::rpm::current();
    if (r < s_minRpm || r > s_maxRpm) return;
    s_cutUntilMs = millis() + s_cutMs;
    s_shiftCount++;
}

} // anonymous

void begin() {
    pinMode(cdi::pins::QUICKSHIFTER, INPUT_PULLUP);
    // attachInterrupt only when enabled, to avoid spurious triggers.
}

void end() {
    if (s_attached) {
        detachInterrupt(digitalPinToInterrupt(cdi::pins::QUICKSHIFTER));
        s_attached = false;
    }
}

bool isEnabled() { return s_enabled; }

void setEnabled(bool e) {
    s_enabled = e;
    if (e && !s_attached) {
        attachInterrupt(digitalPinToInterrupt(cdi::pins::QUICKSHIFTER),
                        isrTrigger, FALLING);
        s_attached = true;
        Serial.println("[qs] attached FALLING on GPIO14");
    } else if (!e && s_attached) {
        detachInterrupt(digitalPinToInterrupt(cdi::pins::QUICKSHIFTER));
        s_attached = false;
        s_cutUntilMs = 0;
        Serial.println("[qs] detached");
    }
}

uint16_t cutDurationMs() { return s_cutMs; }
void setCutDurationMs(uint16_t ms) {
    if (ms < 20)  ms = 20;
    if (ms > 250) ms = 250;
    s_cutMs = ms;
}

cdi::rpm_t minRpm() { return s_minRpm; }
cdi::rpm_t maxRpm() { return s_maxRpm; }
void setRpmGuard(cdi::rpm_t lo, cdi::rpm_t hi) {
    if (lo < 1000)   lo = 1000;
    if (hi <= lo+500)hi = lo + 500;
    if (hi > 15000)  hi = 15000;
    s_minRpm = lo;
    s_maxRpm = hi;
}

bool IRAM_ATTR shouldCut() {
    if (!s_enabled) return false;
    uint32_t until = s_cutUntilMs;
    if (until == 0) return false;
    // Signed-difference comparison is wrap-safe across the 49.7-day
    // millis() rollover. `millis() < until` would latch true for an
    // entire wrap period if `until` straddles the boundary, holding
    // ignition cut for ~49 days.
    return (int32_t)(millis() - until) < 0;
}

bool isActive() {
    if (s_cutUntilMs == 0) return false;
    return (int32_t)(millis() - s_cutUntilMs) < 0;
}

uint32_t totalShifts() { return s_shiftCount; }

} // namespace cdi::core::quickshift
