#include "net/ota.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>

#include "core/spark_scheduler.h"
#include "storage/config_store.h"

namespace cdi::net::ota {
namespace {

// Outcome of the in-flight upload, decided in uploadHandler and acted
// on once in responseHandler. Keeping a single source of truth here
// fixes a bug where the old code called req->send() inside the upload
// handler AND unconditionally rebooted in the response handler — so a
// REJECTED upload (e.g. ignition armed, or Update.begin failure) still
// rebooted the board WITHOUT flashing, landing back on the old firmware
// and looking like "OTA silently did nothing".
enum class Result : uint8_t {
    NONE,            // no data received
    REJECTED_ARMED,  // refused: ignition armed
    BEGIN_FAIL,      // Update.begin failed (no OTA partition / busy)
    WRITE_FAIL,      // write or end/verify failed
    SUCCESS,         // fully written + verified
};

bool   s_inProgress = false;
Result s_result     = Result::NONE;

void uploadHandler(AsyncWebServerRequest* /*req*/,
                   String filename, size_t index,
                   uint8_t* data, size_t len, bool final) {
    if (index == 0) {
        s_inProgress = false;
        s_result     = Result::NONE;

        // Refuse OTA while ignition is armed — safety guard. Do NOT
        // send the response here; responseHandler owns the reply so we
        // never double-send or reboot on a rejected upload.
        if (cdi::core::spark::isArmed()) {
            s_result = Result::REJECTED_ARMED;
            Serial.println("[ota] refused — ignition armed");
            return;
        }
        // Flush pending config to NVS before swapping firmware.
        cdi::storage::config::saveNow();

        Serial.printf("[ota] begin upload: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
            s_result = Result::BEGIN_FAIL;
            return;
        }
        s_inProgress = true;
    }

    if (!s_inProgress) return;

    if (len) {
        if (Update.write(data, len) != len) {
            Update.printError(Serial);
            Update.abort();
            s_inProgress = false;
            s_result     = Result::WRITE_FAIL;
            return;
        }
    }

    if (final) {
        if (Update.end(true)) {
            Serial.printf("[ota] success · %u bytes total\n", (unsigned)(index + len));
            s_result = Result::SUCCESS;
        } else {
            Serial.println("[ota] Update.end failed:");
            Update.printError(Serial);
            s_result = Result::WRITE_FAIL;
        }
        s_inProgress = false;
    }
}

void responseHandler(AsyncWebServerRequest* req) {
    // Single point that replies AND decides whether to reboot. Reboot
    // ONLY on a genuine success — every other outcome reports a clear
    // error and leaves the running firmware untouched.
    switch (s_result) {
        case Result::SUCCESS:
            req->send(200, "text/plain", "OTA ok, rebooting");
            Serial.println("[ota] rebooting in 1s…");
            delay(1000);
            ESP.restart();
            break;
        case Result::REJECTED_ARMED:
            req->send(409, "text/plain", "disarm ignition first");
            break;
        case Result::BEGIN_FAIL:
            req->send(500, "text/plain", "Update.begin failed (OTA partition?)");
            break;
        case Result::WRITE_FAIL:
            req->send(500, "text/plain", "OTA write/verify failed — not flashed");
            break;
        case Result::NONE:
        default:
            req->send(400, "text/plain", "no firmware data received");
            break;
    }
}

} // anonymous

void registerRoutes(AsyncWebServer& srv) {
    srv.on("/api/ota/upload", HTTP_POST,
           responseHandler,
           uploadHandler);
    Serial.println("[ota] route /api/ota/upload ready");
}

} // namespace cdi::net::ota
