#include "core/mode.h"

#include <Arduino.h>

#include "config.h"
#include "core/pulser_input.h"
#include "core/spark_scheduler.h"

namespace cdi::core::mode {
namespace {

// Written by mode::set (called from main loop / panic on core 1, and
// from WS handlers on core 0). Read by main loop, live_stats,
// ws_server, and the WS broadcast path. Volatile prevents the
// compiler from caching across function boundaries — a UI toggle
// to SAFE_HOLD must become visible to the IGNITION-gated loop
// branches on the next iteration. Enum + uint8 backing is atomic
// at the word level on Xtensa.
volatile OperatingMode s_mode = OperatingMode::BOOT;

void enterIgnition() {
    cdi::core::pulser::begin();
    // armed stays whatever it was — UI must explicitly arm
}

void enterSafeHold() {
    cdi::core::spark::setArmed(false);
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
        case OperatingMode::IGNITION:  return "IGNITION";
        case OperatingMode::SAFE_HOLD: return "SAFE_HOLD";
    }
    return "?";
}

} // namespace cdi::core::mode
