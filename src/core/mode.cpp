#include "core/mode.h"

#include <Arduino.h>

#include "config.h"
#include "scope/adc_sampler.h"
#include "core/pulser_input.h"
#include "core/spark_scheduler.h"

namespace cdi::core::mode {
namespace {

OperatingMode s_mode = OperatingMode::BOOT;

// Mode transitions use scope::setPaused rather than end/begin to
// avoid the v2.x Arduino-ESP32 timer driver bug where re-attaching
// the ISR after timerEnd() fails with "timer_isr_callback_add register
// failed". The timer stays initialized for the device's lifetime;
// pausing simply gates the ADC reads inside the ISR.
void enterScope() {
    cdi::core::spark::setArmed(false);   // safety: no spark in scope mode
    cdi::core::pulser::end();
    cdi::scope::setPaused(false);
}

void enterIgnition() {
    cdi::scope::setPaused(true);
    cdi::core::pulser::begin();
    // armed stays whatever it was — UI must explicitly arm after IGNITION
}

void enterSafeHold() {
    cdi::core::spark::setArmed(false);
    cdi::scope::setPaused(true);
    cdi::core::pulser::end();
}

} // anonymous

void begin() {
    s_mode = OperatingMode::IGNITION;
    enterIgnition();
    Serial.println("[mode] IGNITION (default)");
}

OperatingMode current() { return s_mode; }

bool set(OperatingMode m) {
    if (m == s_mode) return true;
    switch (m) {
        case OperatingMode::SCOPE:     enterScope();     break;
        case OperatingMode::IGNITION:  enterIgnition();  break;
        case OperatingMode::SAFE_HOLD: enterSafeHold();  break;
        default:                       return false;
    }
    s_mode = m;
    Serial.printf("[mode] -> %s\n", name(m));
    return true;
}

const char* name(OperatingMode m) {
    switch (m) {
        case OperatingMode::BOOT:      return "BOOT";
        case OperatingMode::SCOPE:     return "SCOPE";
        case OperatingMode::IGNITION:  return "IGNITION";
        case OperatingMode::SAFE_HOLD: return "SAFE_HOLD";
    }
    return "?";
}

} // namespace cdi::core::mode
