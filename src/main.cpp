// Honda Megapro Programmable CDI — firmware entrypoint.
//
// Responsibilities: boot bring-up + loop pump. All real work is in
// the cdi::* modules under src/.

#include <Arduino.h>
#include <LittleFS.h>

#include "config.h"
#include "pinmap.h"
#include "core/mode.h"
#include "core/advance_map.h"
#include "core/spark_scheduler.h"
#include "core/safety.h"
#include "core/shift_light.h"
#include "core/dwell_curve.h"
#include "core/launch_control.h"
#include "core/quickshifter.h"
#include "core/backfire.h"
#include "core/alvp.h"
#include "core/engine_preset.h"
#include "core/rpm_calc.h"
#include "net/wifi_ap.h"
#include "net/http_server.h"
#include "net/ws_server.h"
#include "net/ota.h"
#include "storage/config_store.h"
#include "telemetry/live_stats.h"
#include "telemetry/datalog.h"

namespace cfg = cdi::config;
namespace pin = cdi::pins;

static uint32_t s_lastScopeFrameMs = 0;
static uint32_t s_lastTelemetryMs  = 0;
static uint32_t s_lastSafetyMs     = 0;

constexpr uint32_t TELEMETRY_INTERVAL_MS = 200;   // 5 fps

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.printf("\n[CDI] firmware v%u.%u.%u booting\n",
                  cfg::FW_VERSION_MAJOR, cfg::FW_VERSION_MINOR, cfg::FW_VERSION_PATCH);

    pinMode(pin::STATUS_LED, OUTPUT);
    digitalWrite(pin::STATUS_LED, LOW);

    if (!LittleFS.begin(false)) {
        Serial.println("[fs] LittleFS mount failed — UI assets will 404");
    }

    cdi::core::advance::active().loadDefaultMegapro();
    cdi::net::wifi_ap::begin();
    cdi::net::http_server::begin();
    cdi::net::ws_server::begin();
    cdi::net::ota::registerRoutes(cdi::net::http_server::server());
    cdi::core::spark::begin();                     // claim GPIO + hw timers (disarmed)
    cdi::core::safety::begin();                    // register WDT, init rev limits
    cdi::core::shift_light::begin();               // GPIO27 output
    cdi::core::dwell::loadDefault();               // 4-point default curve (disabled)
    cdi::core::launch::begin();                    // GPIO13 INPUT_PULLUP
    cdi::core::quickshift::begin();                // GPIO14 INPUT_PULLUP (ISR off)
    cdi::core::backfire::begin();                  // (disabled at boot)
    cdi::core::alvp::begin();                      // adc1 init for Vbat
    cdi::core::preset::apply("honda_megapro");     // default preset (overridden by load if NVS)
    cdi::storage::config::load();                  // load saved settings if any
    cdi::core::mode::begin();                      // default = IGNITION

    // Honor auto-arm preference (loaded from NVS).
    if (cdi::core::spark::autoArm() &&
        cdi::core::mode::current() == cdi::OperatingMode::IGNITION) {
        cdi::core::spark::setArmed(true);
        Serial.println("[spark] auto-armed from boot preference");
    }
}

void loop() {
    cdi::net::wifi_ap::poll();
    cdi::telemetry::tick();             // drain pulser events → update RPM

    const uint32_t now = millis();
    if (now - s_lastScopeFrameMs >= cfg::SCOPE_FRAME_INTERVAL_MS) {
        s_lastScopeFrameMs = now;
        cdi::net::ws_server::tickBroadcast();
    }
    if (now - s_lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        s_lastTelemetryMs = now;
        cdi::net::ws_server::tickTelemetry();
    }
    if (now - s_lastSafetyMs >= cfg::SAFETY_TICK_INTERVAL_MS) {
        s_lastSafetyMs = now;
        cdi::core::safety::tick();
    }
    cdi::core::shift_light::tick();          // cheap, can run every loop
    cdi::core::launch::poll();               // debounced digital read
    cdi::core::backfire::tick(cdi::core::rpm::current());
    cdi::core::alvp::tick();                   // sampled every 500ms internally
    cdi::telemetry::datalog::tick();           // sampled every 20ms internally
    cdi::storage::config::tick();              // debounced auto-save

    cdi::net::ws_server::cleanup();
}
