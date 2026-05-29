#include "core/quickshifter.h"

#include <Arduino.h>

#include "pinmap.h"
#include "core/rpm_calc.h"
#include <esp_timer.h>   // esp_timer_get_time — IRAM-safe (unlike millis())

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

volatile uint32_t s_cutUntilUs = 0;   // µs (esp_timer); 0 = no window
volatile uint32_t s_shiftCount = 0;

// IRAM-safe microsecond clock: esp_timer_get_time() is IRAM_ATTR (unlike
// the flash-resident millis()/micros() wrappers in this build). Cast to
// uint32 (no 64-bit divide → no flash libgcc call); signed-diff handles
// the ~71-min wrap for the small (≤250 ms) cut window. Completes audit H2
// (the ISR previously called non-IRAM millis()).
static inline uint32_t IRAM_ATTR qsMicros() { return (uint32_t)esp_timer_get_time(); }

void IRAM_ATTR isrTrigger() {
    cdi::rpm_t r = cdi::core::rpm::current();   // IRAM (audit H2)
    if (r < s_minRpm || r > s_maxRpm) return;
    const uint32_t now = qsMicros();
    // Debounce: one cut + one shift-count per shift — ignore contact
    // bounce/chatter while a cut window is already active (audit M6).
    const uint32_t until = s_cutUntilUs;
    if (until != 0 && (int32_t)(now - until) < 0) return;
    s_cutUntilUs = now + (uint32_t)s_cutMs * 1000u;   // ms → µs
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
        s_cutUntilUs = 0;
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
    if (lo > 14500)  lo = 14500;     // clamp lo FIRST → hi=lo+500 ≤ 15000
    if (hi <= lo+500)hi = lo + 500;
    if (hi > 15000)  hi = 15000;     // (audit M6: avoid lo>hi inverted/never-arm)
    s_minRpm = lo;
    s_maxRpm = hi;
}

bool IRAM_ATTR shouldCut() {
    if (!s_enabled) return false;
    uint32_t until = s_cutUntilUs;
    if (until == 0) return false;
    // Signed-difference comparison is wrap-safe across the uint32 µs
    // rollover (~71 min) for the small cut window. IRAM-safe time source.
    return (int32_t)(qsMicros() - until) < 0;
}

bool isActive() {
    if (s_cutUntilUs == 0) return false;
    return (int32_t)(qsMicros() - s_cutUntilUs) < 0;
}

uint32_t totalShifts() { return s_shiftCount; }

} // namespace cdi::core::quickshift
