#include "core/launch_control.h"

#include <Arduino.h>

#include "pinmap.h"

namespace cdi::core::launch {
namespace {

// volatile: ditulis WS handler (core 0), dibaca tick() + CH1 ISR
// (core 1) — sesuai konvensi cross-core codebase (audit H7).
volatile bool       s_enabled    = false;
volatile cdi::rpm_t s_launchRpm  = 5000;
volatile bool       s_active     = false;
uint32_t            s_lastChangeMs = 0;   // poll() (core-1 loop) only

constexpr uint32_t DEBOUNCE_MS = 30;

} // anonymous

void begin() {
    pinMode(cdi::pins::LAUNCH_INPUT, INPUT_PULLUP);
}

bool       isEnabled()  { return s_enabled; }
cdi::rpm_t launchRpm()  { return s_launchRpm; }
// IRAM_ATTR: read dari CH1 GPIO ISR via safety::shouldFire (audit H1).
bool IRAM_ATTR isActive() { return s_active; }

void setEnabled(bool e) {
    s_enabled = e;
    if (!e) s_active = false;
    Serial.printf("[launch] enabled=%d\n", e ? 1 : 0);
}

void setLaunchRpm(cdi::rpm_t r) {
    if (r < 1500)  r = 1500;
    if (r > 15000) r = 15000;
    s_launchRpm = r;
    Serial.printf("[launch] rpm=%u\n", (unsigned)r);
}

void poll() {
    if (!s_enabled) { s_active = false; return; }
    bool raw = (digitalRead(cdi::pins::LAUNCH_INPUT) == LOW);
    uint32_t now = millis();
    if (raw != s_active) {
        if (now - s_lastChangeMs > DEBOUNCE_MS) {
            s_active = raw;
            s_lastChangeMs = now;
            Serial.printf("[launch] %s\n", raw ? "ACTIVE" : "released");
        }
    } else {
        s_lastChangeMs = now;
    }
}

} // namespace cdi::core::launch
