#include "net/ota.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>

#include "core/mode.h"
#include "storage/config_store.h"

namespace cdi::net::ota {
namespace {

bool s_inProgress = false;

void uploadHandler(AsyncWebServerRequest* req,
                   String filename, size_t index,
                   uint8_t* data, size_t len, bool final) {
    if (index == 0) {
        // Refuse if not in SCOPE mode (engine monitoring active).
        if (cdi::core::mode::current() != cdi::OperatingMode::SCOPE) {
            req->send(409, "text/plain", "switch to SCOPE mode first");
            s_inProgress = false;
            return;
        }
        // Flush pending config to NVS before swapping firmware.
        cdi::storage::config::saveNow();

        Serial.printf("[ota] begin upload: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
            req->send(500, "text/plain", "Update.begin failed");
            s_inProgress = false;
            return;
        }
        s_inProgress = true;
    }

    if (!s_inProgress) return;

    if (len) {
        if (Update.write(data, len) != len) {
            Update.printError(Serial);
        }
    }

    if (final) {
        if (Update.end(true)) {
            Serial.printf("[ota] success · %u bytes total\n", (unsigned)(index + len));
        } else {
            Serial.println("[ota] Update.end failed:");
            Update.printError(Serial);
        }
        s_inProgress = false;
    }
}

void responseHandler(AsyncWebServerRequest* req) {
    // Called when multipart parsing is complete (after uploadHandler).
    if (Update.hasError()) {
        req->send(500, "text/plain", "OTA failed");
        return;
    }
    req->send(200, "text/plain", "OTA ok, rebooting");
    Serial.println("[ota] rebooting in 1s…");
    delay(1000);
    ESP.restart();
}

} // anonymous

void registerRoutes(AsyncWebServer& srv) {
    srv.on("/api/ota/upload", HTTP_POST,
           responseHandler,
           uploadHandler);
    Serial.println("[ota] route /api/ota/upload ready");
}

} // namespace cdi::net::ota
