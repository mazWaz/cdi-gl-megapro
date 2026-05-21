#include "core/panic_button.h"

#include <Arduino.h>

#include "pinmap.h"
#include "core/spark_scheduler.h"
#include "core/mode.h"

namespace cdi::core::panic {
namespace {

constexpr uint32_t LONG_PRESS_MS = 2000;   // hold for emergency kill
constexpr uint32_t DEBOUNCE_MS   = 30;

uint32_t s_pressedSince = 0;   // 0 = not pressed
bool     s_lastReading  = true;
uint32_t s_lastChangeMs = 0;
bool     s_tripped      = false;

} // anonymous

void begin() {
    pinMode(cdi::pins::BOOT_BTN, INPUT_PULLUP);
    Serial.println("[panic] boot button armed (≥2s hold = SAFE_HOLD)");
}

void poll() {
    const bool raw = digitalRead(cdi::pins::BOOT_BTN);   // LOW = pressed
    const uint32_t now = millis();

    // Debounce edge transitions.
    if (raw != s_lastReading) {
        s_lastChangeMs = now;
        s_lastReading  = raw;
    }
    if ((now - s_lastChangeMs) < DEBOUNCE_MS) return;

    // Stable state.
    if (!raw) {
        // Pressed (LOW). Start / continue press timer.
        if (s_pressedSince == 0) s_pressedSince = now;
        if (!s_tripped && (now - s_pressedSince) >= LONG_PRESS_MS) {
            s_tripped = true;
            cdi::core::spark::setArmed(false);
            cdi::core::mode::set(cdi::OperatingMode::SAFE_HOLD);
            Serial.println("[panic] BOOT BUTTON HELD ≥2s → SAFE_HOLD, spark disarmed");
        }
    } else {
        // Released.
        s_pressedSince = 0;
    }
}

bool tripped()   { return s_tripped; }
void clearTrip() { s_tripped = false; }

} // namespace cdi::core::panic
