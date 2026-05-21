// Honda Megapro Programmable CDI — firmware entrypoint.
//
// Responsibilities: boot bring-up + loop pump. All real work is in
// the cdi::* modules under src/.

#include <Arduino.h>
#include <LittleFS.h>
#include <esp_system.h>

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
#include "core/pickup.h"
#include "core/pickup_cal.h"
#include "core/panic_button.h"
#include "core/rpm_calc.h"
#include "scope/edge_snapshot.h"
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

// Human-readable reset cause for the boot banner.
const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external reset";
        case ESP_RST_SW:        return "ESP.restart()";
        case ESP_RST_PANIC:     return "panic (crash)";
        case ESP_RST_INT_WDT:   return "INTERRUPT WDT";
        case ESP_RST_TASK_WDT:  return "TASK WDT";
        case ESP_RST_WDT:       return "other WDT";
        case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
        case ESP_RST_BROWNOUT:  return "BROWN-OUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "unknown";
    }
}

void setup() {
    // ── EARLIEST POSSIBLE GPIO25 SAFETY GUARD ──
    // The spark output pin must be driven LOW before ANYTHING else
    // touches the chip. Wiring (TCI/inductive): GPIO25 → resistor →
    // MOSFET/NPN gate → coil primary → +12V. A floating gate at
    // power-on (before pinMode() runs) lets the MOSFET conduct
    // briefly; when we finally drive the pin LOW, the falling edge
    // collapses the primary field and fires a spark. NOT what we
    // want before the bike is ready.
    //
    // ESP32 GPIO25 has no internal pull-down at boot. STRONGLY
    // recommend an external 10 kΩ from GPIO25 (or the MOSFET gate)
    // to GND in hardware — software alone can't beat the ~50 ms
    // gap between power and the first instruction.
    pinMode(pin::SPARK_OUT, OUTPUT);
    digitalWrite(pin::SPARK_OUT, LOW);
    pinMode(pin::MODE_LED, OUTPUT);
    digitalWrite(pin::MODE_LED, LOW);

    Serial.begin(115200);
    delay(100);

    const esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("\n[CDI] firmware v%u.%u.%u booting (reset: %s, heap=%u)\n",
                  cfg::FW_VERSION_MAJOR, cfg::FW_VERSION_MINOR, cfg::FW_VERSION_PATCH,
                  resetReasonStr(rr), (unsigned)ESP.getFreeHeap());
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_WDT) {
        Serial.println("[CDI] ⚠ previous boot was killed by WATCHDOG — review serial logs");
    } else if (rr == ESP_RST_BROWNOUT) {
        Serial.println("[CDI] ⚠ previous boot was killed by BROWN-OUT — check battery / wiring");
    } else if (rr == ESP_RST_PANIC) {
        Serial.println("[CDI] ⚠ previous boot crashed — investigate before riding");
    }

    pinMode(pin::STATUS_LED, OUTPUT);
    digitalWrite(pin::STATUS_LED, LOW);

    if (!LittleFS.begin(false)) {
        Serial.println("[fs] LittleFS mount failed — UI assets will 404");
    }
    cdi::scope::snapshot::begin();

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
    cdi::core::panic::begin();                     // hardware emergency kill on GPIO0
    cdi::core::preset::apply("honda_megapro");     // default preset (overridden by load if NVS)
    cdi::storage::config::begin();                 // spawn core-0 persist task + sync primitives
    cdi::storage::config::load();                  // load saved settings if any
    cdi::core::mode::begin();                      // default = IGNITION

    // Honor auto-arm preference (loaded from NVS).
    if (cdi::core::spark::autoArm() &&
        cdi::core::mode::current() == cdi::OperatingMode::IGNITION) {
        cdi::core::spark::setArmed(true);
        Serial.println("[spark] auto-armed from boot preference");
    }

    Serial.printf("[CDI] setup() complete · heap=%u · uptime=%lu ms\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned long)millis());
    Serial.println("[CDI] ready — connect to WiFi AP, open http://192.168.4.1/");
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
    cdi::core::panic::poll();                // boot-button long-press → SAFE_HOLD
    cdi::core::launch::poll();               // debounced digital read
    cdi::core::backfire::tick(cdi::core::rpm::current());
    cdi::core::alvp::tick();                   // sampled every 500ms internally
    cdi::telemetry::datalog::tick();           // sampled every 20ms internally
    cdi::core::pickup_cal::tick();             // auto-cal (drains scope ring when active)
    cdi::storage::config::tick();              // debounced auto-save

    cdi::net::ws_server::cleanup();
}
