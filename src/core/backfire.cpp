#include "core/backfire.h"

#include <Arduino.h>

#include "core/launch_control.h"

namespace cdi::core::backfire {
namespace {

bool                 s_enabled    = false;
cdi::BackfireTrigger s_trigger    = cdi::BackfireTrigger::OFF;
cdi::rpm_t           s_rpmLo      = 3000;
cdi::rpm_t           s_rpmHi      = 7000;
float                s_retardDeg  = 15.0f;
uint16_t             s_duration   = 200;
bool                 s_random     = true;

// Runtime state.
bool       s_active        = false;
float      s_currentRetard = 0.0f;
uint32_t   s_activeUntilMs = 0;

// Decel detector state.
cdi::rpm_t s_lastRpm    = 0;
uint32_t   s_lastSampMs = 0;

uint32_t   s_lcg = 0xC0FFEE42;

float jitterRetard(float base) {
    // 32-bit LCG, map to 0.5..1.0 factor.
    s_lcg = s_lcg * 1664525U + 1013904223U;
    uint32_t r = (s_lcg >> 16) & 0xFF;
    float factor = 0.5f + ((float)r / 255.0f) * 0.5f;
    return base * factor;
}

} // anonymous

void begin() {}

bool isEnabled() { return s_enabled; }
void setEnabled(bool e) {
    s_enabled = e;
    if (!e) {
        s_active = false;
        s_currentRetard = 0.0f;
        s_activeUntilMs = 0;
    }
    Serial.printf("[backfire] enabled=%d\n", e ? 1 : 0);
}

cdi::BackfireTrigger trigger() { return s_trigger; }
void setTrigger(cdi::BackfireTrigger t) {
    s_trigger = t;
    s_activeUntilMs = 0;
}

void setRpmRange(cdi::rpm_t lo, cdi::rpm_t hi) {
    if (lo < 500)  lo = 500;
    if (hi <= lo + 200) hi = lo + 200;
    if (hi > 15000) hi = 15000;
    s_rpmLo = lo;
    s_rpmHi = hi;
}
cdi::rpm_t rpmLo() { return s_rpmLo; }
cdi::rpm_t rpmHi() { return s_rpmHi; }

void setRetardDeg(float d) {
    if (d < 0)   d = 0;
    if (d > 45)  d = 45;
    s_retardDeg = d;
}
float retardDeg() { return s_retardDeg; }

void setDurationMs(uint16_t ms) {
    if (ms < 50)   ms = 50;
    if (ms > 2000) ms = 2000;
    s_duration = ms;
}
uint16_t durationMs() { return s_duration; }

void setRandomPattern(bool r) { s_random = r; }
bool randomPattern()          { return s_random; }

bool  isActive()         { return s_active; }
float currentRetardDeg() { return s_currentRetard; }

void tick(cdi::rpm_t rpm) {
    if (!s_enabled || s_trigger == cdi::BackfireTrigger::OFF) {
        s_active = false; s_currentRetard = 0.0f;
        return;
    }

    bool in_range = (rpm >= s_rpmLo && rpm <= s_rpmHi);
    uint32_t now = millis();
    bool should_active = false;

    switch (s_trigger) {
        case cdi::BackfireTrigger::BURBLE: {
            // Random-pattern crackle: short bursts of retard interspersed
            // with normal fires. Previous implementation kept retard
            // active continuously across the entire RPM band, which
            // made the engine unrideable (constant power loss).
            //
            // Algorithm: every 100 ms re-roll the window. With ~25 %
            // probability open a `s_duration` retard window. Outside
            // that window engine runs normally. Result: irregular pops
            // at musical cadence while throttle is in the burble band.
            if (in_range) {
                if (now - s_lastSampMs >= 100) {
                    s_lcg = s_lcg * 1664525U + 1013904223U;
                    if (((s_lcg >> 24) & 0xFF) < 64) {   // 64/256 ≈ 25 %
                        s_activeUntilMs = now + s_duration;
                    }
                    s_lastSampMs = now;
                }
                should_active = (int32_t)(now - s_activeUntilMs) < 0;
            }
            break;
        }
        case cdi::BackfireTrigger::LAUNCH:
            should_active = in_range && cdi::core::launch::isActive();
            break;
        case cdi::BackfireTrigger::DECEL: {
            // sample RPM ~every 100ms; if drop > 500 → open window.
            // millis() rolls over every 49.7 days. Comparing absolute
            // timestamps (`now < s_activeUntilMs`) breaks at the
            // wrap boundary — once `now` rolls past UINT32_MAX/2 from
            // s_activeUntilMs the condition latches permanently true
            // or permanently false. Use signed-difference comparison
            // which stays correct across the wrap.
            if ((int32_t)(now - s_lastSampMs) >= 100) {
                int32_t drop = (int32_t)s_lastRpm - (int32_t)rpm;
                if (drop > 500 && in_range) {
                    s_activeUntilMs = now + s_duration;
                    Serial.printf("[backfire] decel trigger @ %u rpm (drop %d)\n",
                                  (unsigned)rpm, (int)drop);
                }
                s_lastRpm    = rpm;
                s_lastSampMs = now;
            }
            should_active = in_range && ((int32_t)(now - s_activeUntilMs) < 0);
            break;
        }
        default: should_active = false; break;
    }

    s_active = should_active;
    if (s_active) {
        s_currentRetard = s_random ? jitterRetard(s_retardDeg) : s_retardDeg;
    } else {
        s_currentRetard = 0.0f;
    }
}

} // namespace cdi::core::backfire
