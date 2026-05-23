#include "net/ws_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "config.h"
#include "pinmap.h"
#include "net/http_server.h"
#include "net/wifi_ap.h"
#include "scope/edge_capture.h"
#include "scope/edge_snapshot.h"
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
#include "storage/config_store.h"
#include "telemetry/live_stats.h"
#include "telemetry/datalog.h"

namespace cdi::net::ws_server {
namespace {

AsyncWebSocket s_ws("/ws");

void handleText(AsyncWebSocketClient* client, const String& msg) {
    JsonDocument doc;
    if (deserializeJson(doc, msg)) return;
    const char* cmd = doc["cmd"] | "";
    if (!*cmd) return;

    if (!strcmp(cmd, "setMode")) {
        const char* m = doc["mode"] | "";
        cdi::OperatingMode target = cdi::OperatingMode::IGNITION;
        if (!strcmp(m, "ignition"))      target = cdi::OperatingMode::IGNITION;
        else if (!strcmp(m, "safehold")) target = cdi::OperatingMode::SAFE_HOLD;
        else {
            client->text("{\"type\":\"err\",\"msg\":\"unknown mode\"}");
            return;
        }
        cdi::core::mode::set(target);
        JsonDocument r;
        r["type"] = "mode";
        r["mode"] = cdi::core::mode::name(cdi::core::mode::current());
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getMap")) {
        JsonDocument r;
        r["type"] = "map";
        JsonArray arr = r["points"].to<JsonArray>();
        cdi::core::advance::active().serialize(arr);
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setMap")) {
        JsonArrayConst arr = doc["points"].as<JsonArrayConst>();
        if (arr.isNull()) {
            client->text("{\"type\":\"err\",\"msg\":\"missing points\"}");
            return;
        }
        // Pre-validate so we can return the SPECIFIC reason to the UI
        // instead of the generic "invalid map". Parse into a temp array
        // first, then run validateForSafety, then commit.
        cdi::core::advance::Point tmp[cdi::config::MAX_ADVANCE_POINTS];
        size_t n = 0;
        for (JsonVariantConst v : arr) {
            if (n >= cdi::config::MAX_ADVANCE_POINTS) {
                client->text("{\"type\":\"err\",\"msg\":\"max 32 titik\"}");
                return;
            }
            if (!v.is<JsonArrayConst>()) {
                client->text("{\"type\":\"err\",\"msg\":\"format salah (bukan [rpm,deg])\"}");
                return;
            }
            JsonArrayConst pair = v.as<JsonArrayConst>();
            if (pair.size() != 2) {
                client->text("{\"type\":\"err\",\"msg\":\"setiap titik harus [rpm,deg]\"}");
                return;
            }
            tmp[n].rpm = (cdi::rpm_t)(pair[0].as<int>());
            tmp[n].deg = pair[1].as<float>();
            // Range check per point with explicit message
            if (tmp[n].rpm < cdi::config::RPM_MIN_VALID ||
                tmp[n].rpm > cdi::config::RPM_MAX_VALID) {
                JsonDocument r;
                r["type"] = "err";
                char buf[80];
                snprintf(buf, sizeof(buf),
                         "titik %u: RPM %u di luar %u-%u",
                         (unsigned)(n+1), (unsigned)tmp[n].rpm,
                         (unsigned)cdi::config::RPM_MIN_VALID,
                         (unsigned)cdi::config::RPM_MAX_VALID);
                r["msg"] = buf;
                String out; serializeJson(r, out);
                client->text(out);
                return;
            }
            if (tmp[n].deg < cdi::config::ADVANCE_MIN_DEG ||
                tmp[n].deg > cdi::config::ADVANCE_MAX_DEG) {
                JsonDocument r;
                r["type"] = "err";
                char buf[80];
                snprintf(buf, sizeof(buf),
                         "titik %u: advance %.1f° di luar %.0f-%.0f°",
                         (unsigned)(n+1), tmp[n].deg,
                         cdi::config::ADVANCE_MIN_DEG,
                         cdi::config::ADVANCE_MAX_DEG);
                r["msg"] = buf;
                String out; serializeJson(r, out);
                client->text(out);
                return;
            }
            n++;
        }
        if (n == 0) {
            client->text("{\"type\":\"err\",\"msg\":\"map kosong\"}");
            return;
        }
        // Sort by RPM before safety-validation (validateForSafety
        // expects monotonic order).
        for (size_t i = 1; i < n; i++) {
            cdi::core::advance::Point key = tmp[i];
            size_t j = i;
            while (j > 0 && tmp[j-1].rpm > key.rpm) {
                tmp[j] = tmp[j-1];
                j--;
            }
            tmp[j] = key;
        }
        if (const char* err =
                cdi::core::advance::Map::validateForSafety(tmp, n)) {
            JsonDocument r;
            r["type"] = "err";
            char buf[120];
            snprintf(buf, sizeof(buf), "safety: %s", err);
            r["msg"] = buf;
            String out; serializeJson(r, out);
            client->text(out);
            return;
        }
        cdi::core::advance::Map fresh;
        if (!fresh.set(tmp, n)) {
            client->text("{\"type\":\"err\",\"msg\":\"set failed (internal)\"}");
            return;
        }
        cdi::core::advance::active() = fresh;
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"] = "mapApplied";
        r["count"] = (int)cdi::core::advance::active().count();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "loadDefaultMap")) {
        cdi::core::advance::active().loadDefaultMegapro();
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"] = "map";
        JsonArray arr = r["points"].to<JsonArray>();
        cdi::core::advance::active().serialize(arr);
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "reboot")) {
        client->text("{\"type\":\"ack\",\"msg\":\"rebooting\"}");
        cdi::storage::config::saveNow();   // flush pending changes
        delay(200);
        ESP.restart();
    }
    else if (!strcmp(cmd, "setArmed")) {
        bool a = doc["armed"] | false;
        auto m = cdi::core::mode::current();
        Serial.printf("[ws] setArmed req: armed=%d mode=%d\n", a ? 1 : 0, (int)m);
        if (a && m != cdi::OperatingMode::IGNITION) {
            client->text("{\"type\":\"err\",\"msg\":\"arm requires IGNITION mode\"}");
            Serial.println("[ws] setArmed REJECTED: not in IGNITION");
            return;
        }
        if (a) cdi::core::safety::clearFlags();   // fresh start on arm
        cdi::core::spark::setArmed(a);
        JsonDocument r;
        r["type"]  = "armed";
        r["armed"] = cdi::core::spark::isArmed();
        String out; serializeJson(r, out);
        client->text(out);
        Serial.printf("[ws] setArmed ack: now=%d\n", cdi::core::spark::isArmed() ? 1 : 0);
    }
    else if (!strcmp(cmd, "setDwell")) {
        uint32_t us = doc["us"] | cdi::config::DEFAULT_DWELL_US;
        cdi::core::spark::setDwellUs(us);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"] = "dwell";
        r["us"]   = cdi::core::spark::dwellUs();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setAdvanceOffset")) {
        float deg = doc["deg"] | 0.0f;
        cdi::core::spark::setAdvanceOffsetDeg(deg);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"] = "offset";
        r["deg"]  = cdi::core::spark::advanceOffsetDeg();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setAutoArm")) {
        bool en = doc["enabled"] | false;
        cdi::core::spark::setAutoArm(en);
        // Immediate save — user expects this to survive even an instant
        // physical reset, not waiting for the 1.5s debounce.
        cdi::storage::config::saveNow();
        JsonDocument r;
        r["type"]    = "autoArm";
        r["enabled"] = cdi::core::spark::autoArm();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "testFire")) {
        // Optional `dwell` override in µs for diagnostic spark (e.g.
        // 10000 = 10 ms long-dwell to make a visible arc on bench
        // when the configured dwell is too short to see).
        uint32_t override_dwell = doc["dwell"] | 0;
        Serial.printf("[ws] test fire requested (override=%u)\n", (unsigned)override_dwell);
        cdi::core::spark::manualFire(override_dwell);
        client->text("{\"type\":\"ack\",\"msg\":\"fired\"}");
    }
    else if (!strcmp(cmd, "setSparkPolarity")) {
        const bool al = doc["active_low"] | false;
        cdi::core::spark::setActiveLow(al);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]       = "sparkPolarity";
        r["active_low"] = cdi::core::spark::activeLow();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getSparkPolarity")) {
        JsonDocument r;
        r["type"]       = "sparkPolarity";
        r["active_low"] = cdi::core::spark::activeLow();
        r["inductive"]  = cdi::core::spark::inductive();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setIgnitionType")) {
        const bool ind = doc["inductive"] | true;
        cdi::core::spark::setInductive(ind);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]      = "sparkPolarity";
        r["inductive"] = cdi::core::spark::inductive();
        r["active_low"]= cdi::core::spark::activeLow();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setRevLimit")) {
        uint32_t main = doc["main"]    | cdi::core::safety::mainLimitRpm();
        uint32_t over = doc["overrev"] | cdi::core::safety::overrevLimitRpm();
        cdi::core::safety::setRevLimits(main, over);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"] = "revLimit";
        r["main"] = cdi::core::safety::mainLimitRpm();
        r["overrev"] = cdi::core::safety::overrevLimitRpm();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "clearFailsafe")) {
        cdi::core::safety::clearFlags();
        client->text("{\"type\":\"ack\",\"msg\":\"failsafe cleared\"}");
    }
    else if (!strcmp(cmd, "setNoSignalEnabled")) {
        const bool en = doc["enabled"] | false;
        cdi::core::safety::setNoSignalEnabled(en);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "noSignal";
        r["enabled"] = cdi::core::safety::noSignalEnabled();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getNoSignalEnabled")) {
        JsonDocument r;
        r["type"]    = "noSignal";
        r["enabled"] = cdi::core::safety::noSignalEnabled();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getWifi")) {
        JsonDocument r;
        r["type"]     = "wifi";
        r["ssid"]     = cdi::net::wifi_ap::ssid();
        r["password"] = cdi::net::wifi_ap::password();
        r["source"]   = cdi::net::wifi_ap::source();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setWifiSsid")) {
        const char* s = doc["ssid"] | "";
        const bool ok = cdi::net::wifi_ap::setSsid(s);
        JsonDocument r;
        r["type"]     = "wifi";
        r["ok"]       = ok;
        r["msg"]      = ok ? "SSID updated — reboot to apply"
                           : "invalid SSID (1-31 printable ASCII)";
        r["ssid"]     = cdi::net::wifi_ap::ssid();
        r["password"] = cdi::net::wifi_ap::password();
        r["source"]   = cdi::net::wifi_ap::source();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setWifiPassword")) {
        const char* pwd = doc["password"] | "";
        const bool ok = cdi::net::wifi_ap::setPassword(pwd);
        JsonDocument r;
        r["type"]     = "wifi";
        r["ok"]       = ok;
        r["msg"]      = ok ? "password updated — reboot to apply"
                           : "invalid password (8-63 printable ASCII)";
        r["ssid"]     = cdi::net::wifi_ap::ssid();
        r["password"] = cdi::net::wifi_ap::password();
        r["source"]   = cdi::net::wifi_ap::source();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "resetWifi")) {
        cdi::net::wifi_ap::resetToDefaults();
        JsonDocument r;
        r["type"]     = "wifi";
        r["ok"]       = true;
        r["msg"]      = "reset to platformio.ini defaults — reboot to apply";
        r["ssid"]     = cdi::net::wifi_ap::ssid();
        r["password"] = cdi::net::wifi_ap::password();
        r["source"]   = "default";
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "saveSnap")) {
        const String name = cdi::scope::snapshot::sanitize(doc["name"] | "snap");
        if (name.length() == 0) {
            client->text("{\"type\":\"err\",\"msg\":\"invalid name\"}");
            return;
        }
        // Async save — work happens on the snapshot saver task on
        // core 0. Returns immediately; WS reply tells UI the request
        // was queued. UI can poll listSnaps to see when it lands.
        const bool ok = cdi::scope::snapshot::saveAsync(name);
        JsonDocument r;
        r["type"]  = "snapSaved";
        r["name"]  = name;
        r["ok"]    = ok;
        r["count"] = (uint32_t)cdi::scope::snapshot::buffered();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "listSnaps")) {
        JsonDocument r;
        r["type"] = "snapList";
        JsonArray arr = r["snaps"].to<JsonArray>();
        cdi::scope::snapshot::list(arr);
        r["buffered"] = (uint32_t)cdi::scope::snapshot::buffered();
        r["capacity"] = (uint32_t)cdi::scope::snapshot::capacity();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "loadSnap")) {
        const String name = cdi::scope::snapshot::sanitize(doc["name"] | "");
        const int fsize = cdi::scope::snapshot::fileSize(name);
        if (fsize <= 0) {
            client->text("{\"type\":\"err\",\"msg\":\"snapshot not found\"}");
            return;
        }
        if (fsize > 32768) {   // sanity guard, max ~12 KB expected
            client->text("{\"type\":\"err\",\"msg\":\"snapshot too large\"}");
            return;
        }
        // Read file into a temporary buffer, parse, send as JSON of
        // {ts_us, ch, level} events. Large but manageable for a UI
        // overlay request (one-shot, not streamed).
        uint8_t* buf = (uint8_t*)malloc((size_t)fsize);
        if (!buf) { client->text("{\"type\":\"err\",\"msg\":\"oom\"}"); return; }
        const int n = cdi::scope::snapshot::load(name, buf, (size_t)fsize);
        if (n < 6 || buf[0] != cdi::scope::snapshot::FILE_MAGIC_0 ||
                    buf[1] != cdi::scope::snapshot::FILE_MAGIC_1) {
            free(buf);
            client->text("{\"type\":\"err\",\"msg\":\"bad snapshot file\"}");
            return;
        }
        uint16_t cnt = 0;
        memcpy(&cnt, &buf[4], 2);
        JsonDocument r;
        r["type"] = "snapData";
        r["name"] = name;
        r["count"] = cnt;
        JsonArray ev = r["events"].to<JsonArray>();
        const uint8_t* p = &buf[6];
        for (uint16_t i = 0; i < cnt; i++) {
            uint32_t ts; memcpy(&ts, p, 4);
            JsonObject o = ev.add<JsonObject>();
            o["ts"]    = ts;
            o["ch"]    = p[4];
            o["level"] = p[5];
            p += 6;
        }
        free(buf);
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "deleteSnap")) {
        const String name = cdi::scope::snapshot::sanitize(doc["name"] | "");
        const bool ok = cdi::scope::snapshot::remove(name);
        JsonDocument r;
        r["type"] = "snapDeleted";
        r["name"] = name;
        r["ok"]   = ok;
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getPresetList")) {
        // Return list as JSON array; may be large so use streaming response.
        JsonDocument r;
        r["type"] = "presetList";
        r["current"] = cdi::core::preset::currentId();
        r["modified"] = cdi::core::preset::isModified();
        JsonArray arr = r["list"].to<JsonArray>();
        for (size_t i = 0; i < cdi::core::preset::count(); i++) {
            const auto* p = cdi::core::preset::at(i);
            JsonObject o = arr.add<JsonObject>();
            o["id"]       = p->id;
            o["cat"]      = p->category;
            o["name"]     = p->display;
            o["trigger"]  = p->trigger_channel;
            o["magnet"]   = p->magnet_width_deg;
            o["max_adv"]  = p->max_advance_deg;
            o["points"]   = p->point_count;
            o["rev_main"] = p->rev_main_rpm;
            o["rev_over"] = p->rev_overrev_rpm;
            o["dwell"]    = p->dwell_us;
            o["notes"]    = p->notes;
        }
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setPreset")) {
        const char* id = doc["id"] | "";
        if (!cdi::core::preset::apply(id)) {
            client->text("{\"type\":\"err\",\"msg\":\"preset not found\"}");
            return;
        }
        cdi::storage::config::saveNow();
        JsonDocument r;
        r["type"]     = "preset";
        r["current"]  = cdi::core::preset::currentId();
        r["modified"] = false;
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getPresetStatus")) {
        JsonDocument r;
        r["type"]     = "preset";
        r["current"]  = cdi::core::preset::currentId();
        r["modified"] = cdi::core::preset::isModified();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getPresetMap")) {
        // Return the advance curve for any preset by id (defaults to
        // currently-active preset if id omitted). UI editor uses this
        // to overlay the "stock" reference curve for whichever motor
        // the user is currently tuning — Megapro stock for Megapro,
        // CB150R stock for CB150R, etc.
        const char* id = doc["id"] | cdi::core::preset::currentId();
        const auto* p = cdi::core::preset::find(id);
        if (!p) {
            client->text("{\"type\":\"err\",\"msg\":\"preset not found\"}");
            return;
        }
        JsonDocument r;
        r["type"]    = "presetMap";
        r["id"]      = p->id;
        r["name"]    = p->display;
        JsonArray arr = r["points"].to<JsonArray>();
        for (uint8_t i = 0; i < p->point_count; i++) {
            JsonArray pair = arr.add<JsonArray>();
            pair.add(p->points[i].rpm);
            pair.add(p->points[i].deg);
        }
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getPickup")) {
        JsonDocument r;
        r["type"]    = "pickup";
        r["max_ref"] = cdi::core::pickup::maxAdvanceRef();
        r["magnet"]  = cdi::core::pickup::magnetWidth();
        r["base"]    = cdi::core::pickup::baseAdvanceRef();
        r["override"]= cdi::core::pickup::hasOverride();
        r["source"]  = cdi::core::pickup::source();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "resetPickup")) {
        // Drop override and re-apply current preset's geometry
        cdi::core::pickup::setOverride(false);
        cdi::core::pickup::setSource("preset");
        cdi::core::preset::apply(cdi::core::preset::currentId());
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "pickup";
        r["max_ref"] = cdi::core::pickup::maxAdvanceRef();
        r["magnet"]  = cdi::core::pickup::magnetWidth();
        r["base"]    = cdi::core::pickup::baseAdvanceRef();
        r["override"]= false;
        r["source"]  = "preset";
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "startCal")) {
        uint8_t  revs  = doc["revs"]      | cdi::core::pickup_cal::DEFAULT_TARGET_REVS;
        uint32_t to    = doc["timeoutMs"] | cdi::core::pickup_cal::DEFAULT_TIMEOUT_MS;
        uint16_t rpmLo = doc["rpmMin"]    | cdi::core::pickup_cal::DEFAULT_MIN_RPM;
        uint16_t rpmHi = doc["rpmMax"]    | cdi::core::pickup_cal::DEFAULT_MAX_RPM;
        float    jit   = doc["jitterPct"] | cdi::core::pickup_cal::DEFAULT_JITTER_PCT;
        cdi::core::pickup_cal::setTarget(revs);
        cdi::core::pickup_cal::setTimeoutMs(to);
        cdi::core::pickup_cal::setRpmWindow(rpmLo, rpmHi);
        cdi::core::pickup_cal::setMaxJitterPct(jit);
        bool ok = cdi::core::pickup_cal::start();
        JsonDocument r;
        r["type"]  = "cal";
        r["state"] = ok ? "COLLECTING" : "ERR";
        r["msg"]   = ok ? "engine idle, hold RPM steady" : "pulser ISR not attached";
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "stopCal")) {
        cdi::core::pickup_cal::stop();
        JsonDocument r;
        r["type"]  = "cal";
        r["state"] = "IDLE";
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getCalStatus")) {
        const auto s = cdi::core::pickup_cal::status();
        JsonDocument r;
        r["type"] = "cal";
        const char* sn = "IDLE";
        switch (s.state) {
            case cdi::core::pickup_cal::State::IDLE:            sn = "IDLE"; break;
            case cdi::core::pickup_cal::State::COLLECTING:      sn = "COLLECTING"; break;
            case cdi::core::pickup_cal::State::DONE:            sn = "DONE"; break;
            case cdi::core::pickup_cal::State::ERR_TIMEOUT:     sn = "ERR_TIMEOUT"; break;
            case cdi::core::pickup_cal::State::ERR_MULTI_TOOTH: sn = "ERR_MULTI_TOOTH"; break;
        }
        r["state"]           = sn;
        r["good"]            = s.good_revs;
        r["target"]          = s.target_revs;
        r["total_events"]    = s.total_events;
        r["total_falls"]     = s.total_falls;
        r["skipped_jitter"]  = s.skipped_jitter;
        r["skipped_rpm"]     = s.skipped_rpm;
        r["width_mean"]      = s.width_mean_deg;
        r["width_median"]    = s.width_median_deg;
        r["width_min"]       = s.width_min_deg;
        r["width_max"]       = s.width_max_deg;
        r["width_stddev"]    = s.width_stddev_deg;
        r["confidence"]      = s.confidence_pct;
        r["rpm"]             = s.rpm_mean;
        r["elapsed_ms"]      = s.elapsed_ms;
        if (s.state == cdi::core::pickup_cal::State::DONE) {
            // Comparison vs preset geometry
            const float preset_w = cdi::core::pickup::magnetWidth();
            r["preset_width"]   = preset_w;
            r["delta_deg"]      = s.width_median_deg - preset_w;
        }
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "applyCal")) {
        // Take measured magnet width as the new pickup override.
        // max_advance_ref CANNOT be inferred from auto-cal; preserve
        // the preset value but flag override so subsequent preset
        // re-applies don't clobber the measured width.
        const auto s = cdi::core::pickup_cal::status();
        if (s.state != cdi::core::pickup_cal::State::DONE) {
            client->text("{\"type\":\"err\",\"msg\":\"calibration not complete\"}");
            return;
        }
        cdi::core::pickup::setMagnetWidth(s.width_median_deg);
        cdi::core::pickup::setOverride(true);
        cdi::core::pickup::setSource("auto_cal");
        cdi::storage::config::saveNow();    // immediate persist
        JsonDocument r;
        r["type"]    = "pickup";
        r["max_ref"] = cdi::core::pickup::maxAdvanceRef();
        r["magnet"]  = cdi::core::pickup::magnetWidth();
        r["base"]    = cdi::core::pickup::baseAdvanceRef();
        r["override"]= true;
        r["source"]  = "auto_cal";
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "startDatalog")) {
        cdi::telemetry::datalog::start();
        client->text("{\"type\":\"datalog\",\"recording\":true}");
    }
    else if (!strcmp(cmd, "stopDatalog")) {
        cdi::telemetry::datalog::stop();
        JsonDocument r;
        r["type"]     = "datalog";
        r["recording"]= false;
        r["entries"]  = cdi::telemetry::datalog::entryCount();
        r["duration"] = cdi::telemetry::datalog::durationMs();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getDatalogStatus")) {
        JsonDocument r;
        r["type"]     = "datalog";
        r["recording"]= cdi::telemetry::datalog::isRecording();
        r["entries"]  = cdi::telemetry::datalog::entryCount();
        r["capacity"] = cdi::telemetry::datalog::capacity();
        r["duration"] = cdi::telemetry::datalog::durationMs();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "clearDatalog")) {
        cdi::telemetry::datalog::clear();
        client->text("{\"type\":\"ack\",\"msg\":\"datalog cleared\"}");
    }
    else if (!strcmp(cmd, "setAlvp")) {
        bool en = doc["enabled"] | true;
        float dv = doc["derate"] | cdi::core::alvp::derateThresholdV();
        float xv = doc["disarm"] | cdi::core::alvp::disarmThresholdV();
        uint32_t lim = doc["derate_rpm"] | cdi::core::alvp::derateLimitRpm();
        cdi::core::alvp::setThresholds(dv, xv);
        cdi::core::alvp::setDerateLimitRpm(lim);
        cdi::core::alvp::setEnabled(en);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "alvp";
        r["enabled"] = cdi::core::alvp::isEnabled();
        r["derate"]  = cdi::core::alvp::derateThresholdV();
        r["disarm"]  = cdi::core::alvp::disarmThresholdV();
        r["derate_rpm"] = cdi::core::alvp::derateLimitRpm();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setBackfire")) {
        bool en = doc["enabled"] | false;
        uint8_t trig = doc["trigger"] | 0;
        uint32_t lo = doc["rpm_lo"] | cdi::core::backfire::rpmLo();
        uint32_t hi = doc["rpm_hi"] | cdi::core::backfire::rpmHi();
        float retard = doc["retard"] | cdi::core::backfire::retardDeg();
        uint32_t dur = doc["duration_ms"] | cdi::core::backfire::durationMs();
        bool rnd = doc["random"] | cdi::core::backfire::randomPattern();
        Serial.printf("[ws] setBackfire en=%d trig=%d range=%u-%u retard=%.1f dur=%u rnd=%d\n",
                      en?1:0, (int)trig, (unsigned)lo, (unsigned)hi, retard, (unsigned)dur, rnd?1:0);

        cdi::core::backfire::setTrigger((cdi::BackfireTrigger)trig);
        cdi::core::backfire::setRpmRange(lo, hi);
        cdi::core::backfire::setRetardDeg(retard);
        cdi::core::backfire::setDurationMs(dur);
        cdi::core::backfire::setRandomPattern(rnd);
        cdi::core::backfire::setEnabled(en);
        cdi::storage::config::markDirty();

        JsonDocument r;
        r["type"]      = "backfire";
        r["enabled"]   = cdi::core::backfire::isEnabled();
        r["trigger"]   = (uint8_t)cdi::core::backfire::trigger();
        r["rpm_lo"]    = cdi::core::backfire::rpmLo();
        r["rpm_hi"]    = cdi::core::backfire::rpmHi();
        r["retard"]    = cdi::core::backfire::retardDeg();
        r["duration_ms"] = cdi::core::backfire::durationMs();
        r["random"]    = cdi::core::backfire::randomPattern();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setLaunch")) {
        bool en = doc["enabled"] | false;
        uint32_t rpm = doc["rpm"] | cdi::core::launch::launchRpm();
        cdi::core::launch::setEnabled(en);
        cdi::core::launch::setLaunchRpm(rpm);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "launch";
        r["enabled"] = cdi::core::launch::isEnabled();
        r["rpm"]     = cdi::core::launch::launchRpm();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setQuickshifter")) {
        bool en = doc["enabled"] | false;
        uint32_t ms = doc["cut_ms"] | cdi::core::quickshift::cutDurationMs();
        uint32_t lo = doc["min_rpm"] | cdi::core::quickshift::minRpm();
        uint32_t hi = doc["max_rpm"] | cdi::core::quickshift::maxRpm();
        cdi::core::quickshift::setCutDurationMs(ms);
        cdi::core::quickshift::setRpmGuard(lo, hi);
        cdi::core::quickshift::setEnabled(en);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]     = "quickshifter";
        r["enabled"]  = cdi::core::quickshift::isEnabled();
        r["cut_ms"]   = cdi::core::quickshift::cutDurationMs();
        r["min_rpm"]  = cdi::core::quickshift::minRpm();
        r["max_rpm"]  = cdi::core::quickshift::maxRpm();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setShiftLight")) {
        bool en = doc["enabled"] | true;
        uint32_t warn  = doc["warn"]  | cdi::core::shift_light::rpmWarn();
        uint32_t shift = doc["shift"] | cdi::core::shift_light::rpmShift();
        cdi::core::shift_light::setEnabled(en);
        cdi::core::shift_light::setThresholds(warn, shift);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "shiftLight";
        r["enabled"] = cdi::core::shift_light::isEnabled();
        r["warn"]    = cdi::core::shift_light::rpmWarn();
        r["shift"]   = cdi::core::shift_light::rpmShift();
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setDwellCurve")) {
        bool en = doc["enabled"] | false;
        JsonArrayConst arr = doc["points"].as<JsonArrayConst>();
        if (!arr.isNull()) {
            cdi::core::dwell::loadFromJson(arr);
        }
        cdi::core::dwell::setEnabled(en);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]    = "dwellCurve";
        r["enabled"] = cdi::core::dwell::isEnabled();
        JsonArray out_arr = r["points"].to<JsonArray>();
        cdi::core::dwell::serialize(out_arr);
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "getDwellCurve")) {
        JsonDocument r;
        r["type"]    = "dwellCurve";
        r["enabled"] = cdi::core::dwell::isEnabled();
        JsonArray out_arr = r["points"].to<JsonArray>();
        cdi::core::dwell::serialize(out_arr);
        String out; serializeJson(r, out);
        client->text(out);
    }
    else if (!strcmp(cmd, "setCutMode")) {
        uint8_t m = doc["mode"] | 1;
        float retard = doc["retard"] | cdi::core::safety::mainRetardDeg();
        uint8_t fireN = doc["fire_n"] | cdi::core::safety::patternFireN();
        uint8_t skipN = doc["skip_n"] | cdi::core::safety::patternSkipN();
        cdi::core::safety::setMainCutMode((cdi::CutMode)m);
        cdi::core::safety::setMainRetardDeg(retard);
        cdi::core::safety::setMainPatternRatio(fireN, skipN);
        cdi::storage::config::markDirty();
        JsonDocument r;
        r["type"]   = "cutMode";
        r["mode"]   = (uint8_t)cdi::core::safety::mainCutMode();
        r["retard"] = cdi::core::safety::mainRetardDeg();
        r["fire_n"] = cdi::core::safety::patternFireN();
        r["skip_n"] = cdi::core::safety::patternSkipN();
        String out; serializeJson(r, out);
        client->text(out);
    }
}

void onEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
             AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        digitalWrite(cdi::pins::STATUS_LED, HIGH);
        JsonDocument doc;
        doc["type"] = "hello";
        doc["fw"]   = cdi::config::FW_VERSION;
        doc["mode"] = cdi::core::mode::name(cdi::core::mode::current());
        String out; serializeJson(doc, out);
        client->text(out);
    }
    else if (type == WS_EVT_DISCONNECT) {
        if (server->count() == 0) digitalWrite(cdi::pins::STATUS_LED, LOW);
    }
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            String msg;
            msg.reserve(len);
            for (size_t i = 0; i < len; i++) msg += (char)data[i];
            handleText(client, msg);
        }
    }
}

} // anonymous

void begin() {
    s_ws.onEvent(onEvent);
    cdi::net::http_server::server().addHandler(&s_ws);
    cdi::scope::edge::begin();
    Serial.println("[WS] handler attached at /ws (edge stream on)");
}

void tickBroadcast() {
    // Edge-event stream (opcode 0xA7) — live whenever pulser ISR is
    // attached (i.e. not in SAFE_HOLD). Drives the scope visualization
    // concurrently with ignition; no mode change required.
    //
    // Auto-cal owns the scope ring exclusively while COLLECTING: edge
    // broadcast pauses so the calibrator drains every event.
    //
    // We call buildFrame regardless of WS client count — it also feeds
    // the rolling RAM snapshot ring as a side effect, and we don't want
    // to lose events while no phone is connected.
    if (cdi::core::mode::current() == cdi::OperatingMode::SAFE_HOLD) return;
    if (cdi::core::pickup_cal::status().state ==
            cdi::core::pickup_cal::State::COLLECTING) return;
    if (!cdi::scope::edge::dueNow(millis())) return;

    size_t len = 0;
    const uint8_t* f = cdi::scope::edge::buildFrame(len);
    if (s_ws.count() == 0 || !f || len == 0) return;

    for (auto& c : s_ws.getClients()) {
        if (c.queueIsFull() ||
            c.queueLen() > cdi::config::WS_QUEUE_BACKPRESSURE_LIMIT) return;
    }
    s_ws.binaryAll((uint8_t*)f, len);
}

void tickTelemetry() {
    if (s_ws.count() == 0) return;
    // Telemetry frame is only 18 B, much smaller than scope (8 KB).
    // Use the looser `queueIsFull` check so it can sneak in even
    // when scope frames are saturating the queue.
    for (auto& c : s_ws.getClients()) {
        if (c.queueIsFull()) return;
    }

    auto t = cdi::telemetry::snapshot();
    uint8_t buf[71];
    buf[0]  = cdi::config::WS_MAGIC_TELEMETRY;
    buf[1]  = (uint8_t)t.mode;
    memcpy(&buf[2],  &t.rpm,                2);
    memcpy(&buf[4],  &t.rpm_raw,            2);
    memcpy(&buf[6],  &t.pulser_count,       4);
    memcpy(&buf[10], &t.uptime_ms,          4);
    memcpy(&buf[14], &t.free_heap,          4);
    memcpy(&buf[18], &t.target_advance_x10, 2);
    buf[20] = t.armed;
    memcpy(&buf[21], &t.fire_count,         4);
    memcpy(&buf[25], &t.last_jitter_us,     2);
    buf[27] = t.safety_flags;
    memcpy(&buf[28], &t.main_limit_rpm,      2);
    memcpy(&buf[30], &t.overrev_limit_rpm,   2);
    memcpy(&buf[32], &t.dwell_us,            2);
    memcpy(&buf[34], &t.advance_offset_x10,  2);
    buf[36] = t.cut_mode;
    buf[37] = t.retard_half_deg;
    buf[38] = t.pattern_fire_n;
    buf[39] = t.pattern_skip_n;
    buf[40] = t.shift_state;
    buf[41] = t.flags2;
    memcpy(&buf[42], &t.shift_rpm_warn,  2);
    memcpy(&buf[44], &t.shift_rpm_shift, 2);
    memcpy(&buf[46], &t.launch_rpm,      2);
    memcpy(&buf[48], &t.qs_cut_ms,       2);
    memcpy(&buf[50], &t.qs_count,        4);
    buf[54] = t.backfire_trigger;
    buf[55] = t.flags3;
    memcpy(&buf[56], &t.bf_rpm_lo,       2);
    memcpy(&buf[58], &t.bf_rpm_hi,       2);
    buf[60] = t.bf_retard_half_deg;
    memcpy(&buf[61], &t.bf_duration_ms,  2);
    memcpy(&buf[63], &t.vbat_mv,         2);
    buf[65] = t.alvp_state;
    buf[66] = t.alvp_derate_v_x10;
    buf[67] = t.alvp_disarm_v_x10;
    buf[68] = t.flags4;
    memcpy(&buf[69], &t.alvp_derate_rpm, 2);
    s_ws.binaryAll(buf, sizeof(buf));
}

void cleanup() { s_ws.cleanupClients(); }
int  clientCount() { return s_ws.count(); }

} // namespace cdi::net::ws_server
