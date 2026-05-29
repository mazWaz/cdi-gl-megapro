#include "core/mode.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

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

// Serializes set() across the two tasks that call it — WS handler (core 0)
// and panic/pickup_cal pollers (core-1 loop). Without it, a UI arm and a
// concurrent panic SAFE_HOLD can interleave so the panic reads a stale
// s_mode and early-returns (kill swallowed → spark stays LIVE) — audit M9.
// A FreeRTOS mutex (not portMUX) so we can safely hold it across the
// attachInterrupt/detachInterrupt inside enterIgnition/enterSafeHold.
SemaphoreHandle_t s_modeMux = nullptr;

void enterIgnition() {
    cdi::core::pulser::begin();
    // Market-CDI behaviour: IGNITION mode IS "on" → arm automatically.
    // The rider never does a separate "arm" step — powering up (or
    // booting) in IGNITION makes spark live, exactly like a stock CDI.
    // The only OFF/kill is SAFE_HOLD (panic button GPIO0, or UI). Faults
    // pulse-cut (self-recovering) and never clear `armed`. ⚠ Spark is
    // LIVE in this mode — keep busi grounded/removed on the bench.
    cdi::core::spark::setArmed(true);
}

void enterSafeHold() {
    cdi::core::spark::setArmed(false);
    cdi::core::pulser::end();
}

} // anonymous

void begin() {
    if (!s_modeMux) s_modeMux = xSemaphoreCreateMutex();
    s_mode = OperatingMode::IGNITION;
    enterIgnition();
    Serial.println("[mode] IGNITION (default)");
}

OperatingMode current() { return s_mode; }

bool set(OperatingMode m) {
    // Atomic {read s_mode → side-effects → write s_mode} across both
    // calling tasks (audit M9), so a competing transition can't observe a
    // stale mode and no-op while this one is mid-flight.
    if (s_modeMux) xSemaphoreTake(s_modeMux, portMAX_DELAY);
    bool ok = true;
    if (m != s_mode) {
        switch (m) {
            case OperatingMode::IGNITION:  enterIgnition();  break;
            case OperatingMode::SAFE_HOLD: enterSafeHold();  break;
            default:                       ok = false;       break;
        }
        if (ok) {
            s_mode = m;
            Serial.printf("[mode] -> %s\n", name(m));
        }
    }
    if (s_modeMux) xSemaphoreGive(s_modeMux);
    return ok;
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
