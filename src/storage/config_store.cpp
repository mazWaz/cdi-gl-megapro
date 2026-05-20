#include "storage/config_store.h"

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include "core/advance_map.h"
#include "core/safety.h"
#include "core/spark_scheduler.h"
#include "core/shift_light.h"
#include "core/dwell_curve.h"
#include "core/launch_control.h"
#include "core/quickshifter.h"
#include "core/backfire.h"
#include "core/alvp.h"
#include "core/engine_preset.h"

namespace cdi::storage::config {
namespace {

// NVS-based persistence — survives uploadfs (which wipes LittleFS).
// Namespace name must be ≤15 chars.
constexpr const char* NVS_NS  = "cdicfg";
constexpr const char* NVS_KEY = "blob";
constexpr uint32_t    DEBOUNCE_MS = 1500;
constexpr uint8_t     SCHEMA_VERSION = 1;

bool     s_dirty       = false;
uint32_t s_lastDirtyMs = 0;

void buildJson(JsonDocument& doc) {
    doc["v"] = SCHEMA_VERSION;
    doc["preset_id"] = cdi::core::preset::currentId();
    doc["preset_modified"] = cdi::core::preset::isModified();

    // advance map
    JsonArray map = doc["advance_map"].to<JsonArray>();
    cdi::core::advance::active().serialize(map);

    // rev limits
    JsonObject rev = doc["rev"].to<JsonObject>();
    rev["main"]    = cdi::core::safety::mainLimitRpm();
    rev["overrev"] = cdi::core::safety::overrevLimitRpm();

    // cut mode
    JsonObject cut = doc["cut"].to<JsonObject>();
    cut["mode"]    = (uint8_t)cdi::core::safety::mainCutMode();
    cut["retard"]  = cdi::core::safety::mainRetardDeg();
    cut["fire_n"]  = cdi::core::safety::patternFireN();
    cut["skip_n"]  = cdi::core::safety::patternSkipN();

    // spark output
    JsonObject sp = doc["spark"].to<JsonObject>();
    sp["dwell_us"]       = cdi::core::spark::dwellUs();
    sp["advance_offset"] = cdi::core::spark::advanceOffsetDeg();
    sp["auto_arm"]       = cdi::core::spark::autoArm();

    // shift light
    JsonObject sh = doc["shift"].to<JsonObject>();
    sh["enabled"] = cdi::core::shift_light::isEnabled();
    sh["warn"]    = cdi::core::shift_light::rpmWarn();
    sh["shift"]   = cdi::core::shift_light::rpmShift();

    // dwell curve
    JsonObject dc = doc["dwell_curve"].to<JsonObject>();
    dc["enabled"] = cdi::core::dwell::isEnabled();
    JsonArray dcp = dc["points"].to<JsonArray>();
    cdi::core::dwell::serialize(dcp);

    // launch
    JsonObject lc = doc["launch"].to<JsonObject>();
    lc["enabled"] = cdi::core::launch::isEnabled();
    lc["rpm"]     = cdi::core::launch::launchRpm();

    // quickshifter
    JsonObject qs = doc["quickshift"].to<JsonObject>();
    qs["enabled"] = cdi::core::quickshift::isEnabled();
    qs["cut_ms"]  = cdi::core::quickshift::cutDurationMs();
    qs["min_rpm"] = cdi::core::quickshift::minRpm();
    qs["max_rpm"] = cdi::core::quickshift::maxRpm();

    // ALVP
    JsonObject av = doc["alvp"].to<JsonObject>();
    av["enabled"]    = cdi::core::alvp::isEnabled();
    av["derate"]     = cdi::core::alvp::derateThresholdV();
    av["disarm"]     = cdi::core::alvp::disarmThresholdV();
    av["derate_rpm"] = cdi::core::alvp::derateLimitRpm();

    // backfire
    JsonObject bf = doc["backfire"].to<JsonObject>();
    bf["enabled"]     = cdi::core::backfire::isEnabled();
    bf["trigger"]     = (uint8_t)cdi::core::backfire::trigger();
    bf["rpm_lo"]      = cdi::core::backfire::rpmLo();
    bf["rpm_hi"]      = cdi::core::backfire::rpmHi();
    bf["retard"]      = cdi::core::backfire::retardDeg();
    bf["duration_ms"] = cdi::core::backfire::durationMs();
    bf["random"]      = cdi::core::backfire::randomPattern();
}

void applyJson(const JsonDocument& doc) {
    // ── Apply preset first (sets defaults), then user overrides
    const char* pid = doc["preset_id"] | (const char*)nullptr;
    if (pid && *pid) {
        cdi::core::preset::apply(pid);   // base config from preset
    }
    bool modified = doc["preset_modified"] | false;

    // advance map (overrides preset if present)
    if (doc["advance_map"].is<JsonArrayConst>()) {
        cdi::core::advance::active().loadFromJson(doc["advance_map"].as<JsonArrayConst>());
    }

    // rev limits
    {
        JsonObjectConst rev = doc["rev"];
        if (!rev.isNull()) {
            uint32_t m = rev["main"]    | cdi::core::safety::mainLimitRpm();
            uint32_t o = rev["overrev"] | cdi::core::safety::overrevLimitRpm();
            cdi::core::safety::setRevLimits(m, o);
        }
    }
    // cut mode
    {
        JsonObjectConst cut = doc["cut"];
        if (!cut.isNull()) {
            uint8_t m   = cut["mode"]   | 1;
            float r     = cut["retard"] | 10.0f;
            uint8_t fn  = cut["fire_n"] | 3;
            uint8_t sn  = cut["skip_n"] | 1;
            cdi::core::safety::setMainCutMode((cdi::CutMode)m);
            cdi::core::safety::setMainRetardDeg(r);
            cdi::core::safety::setMainPatternRatio(fn, sn);
        }
    }
    // spark
    {
        JsonObjectConst sp = doc["spark"];
        if (!sp.isNull()) {
            cdi::core::spark::setDwellUs(sp["dwell_us"] | 2500);
            cdi::core::spark::setAdvanceOffsetDeg(sp["advance_offset"] | 0.0f);
            cdi::core::spark::setAutoArm(sp["auto_arm"] | false);
        }
    }
    // shift light
    {
        JsonObjectConst sh = doc["shift"];
        if (!sh.isNull()) {
            cdi::core::shift_light::setThresholds(sh["warn"] | 7500, sh["shift"] | 8500);
            cdi::core::shift_light::setEnabled(sh["enabled"] | true);
        }
    }
    // dwell curve
    {
        JsonObjectConst dc = doc["dwell_curve"];
        if (!dc.isNull()) {
            if (dc["points"].is<JsonArrayConst>()) {
                cdi::core::dwell::loadFromJson(dc["points"].as<JsonArrayConst>());
            }
            cdi::core::dwell::setEnabled(dc["enabled"] | false);
        }
    }
    // launch
    {
        JsonObjectConst lc = doc["launch"];
        if (!lc.isNull()) {
            cdi::core::launch::setLaunchRpm(lc["rpm"] | 5000);
            cdi::core::launch::setEnabled(lc["enabled"] | false);
        }
    }
    // quickshift
    {
        JsonObjectConst qs = doc["quickshift"];
        if (!qs.isNull()) {
            cdi::core::quickshift::setCutDurationMs(qs["cut_ms"] | 65);
            cdi::core::quickshift::setRpmGuard(qs["min_rpm"] | 4000, qs["max_rpm"] | 12000);
            cdi::core::quickshift::setEnabled(qs["enabled"] | false);
        }
    }
    // ALVP
    {
        JsonObjectConst av = doc["alvp"];
        if (!av.isNull()) {
            cdi::core::alvp::setThresholds(av["derate"] | 10.5f, av["disarm"] | 9.0f);
            cdi::core::alvp::setDerateLimitRpm(av["derate_rpm"] | 4000);
            cdi::core::alvp::setEnabled(av["enabled"] | true);
        }
    }
    // backfire
    {
        JsonObjectConst bf = doc["backfire"];
        if (!bf.isNull()) {
            cdi::core::backfire::setTrigger((cdi::BackfireTrigger)(bf["trigger"] | 0));
            cdi::core::backfire::setRpmRange(bf["rpm_lo"] | 3000, bf["rpm_hi"] | 7000);
            cdi::core::backfire::setRetardDeg(bf["retard"] | 15.0f);
            cdi::core::backfire::setDurationMs(bf["duration_ms"] | 200);
            cdi::core::backfire::setRandomPattern(bf["random"] | true);
            cdi::core::backfire::setEnabled(bf["enabled"] | false);
        }
    }

    if (modified) cdi::core::preset::markModifiedFlag();
}

} // anonymous

void begin() {
    // nothing — load() is called explicitly by main.cpp after modules init
}

void load() {
    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/true)) {
        Serial.println("[config] NVS open fail (read)");
        return;
    }
    // NOTE: do NOT use getBytesLength() here — that API only works for
    // BLOB entries. We write as STR via putString(), so isKey() is the
    // correct existence check.
    if (!prefs.isKey(NVS_KEY)) {
        Serial.println("[config] no saved blob; using compile defaults");
        prefs.end();
        return;
    }
    String json = prefs.getString(NVS_KEY, "");
    prefs.end();
    if (json.length() == 0) {
        Serial.println("[config] saved blob empty; using compile defaults");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.printf("[config] parse fail: %s\n", err.c_str());
        return;
    }
    uint8_t v = doc["v"] | 0;
    if (v != SCHEMA_VERSION) {
        Serial.printf("[config] schema v%u != %u, ignoring\n", v, SCHEMA_VERSION);
        return;
    }
    applyJson(doc);
    Serial.printf("[config] loaded %u bytes from NVS\n", (unsigned)json.length());
}

void markDirty() {
    s_dirty       = true;
    s_lastDirtyMs = millis();
}

void saveNow() {
    JsonDocument doc;
    buildJson(doc);
    String json;
    serializeJson(doc, json);

    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/false)) {
        Serial.println("[config] NVS open fail (write)");
        return;
    }
    size_t n = prefs.putString(NVS_KEY, json);
    prefs.end();
    s_dirty = false;
    Serial.printf("[config] saved %u bytes to NVS\n", (unsigned)n);
}

void tick() {
    if (!s_dirty) return;
    if (millis() - s_lastDirtyMs < DEBOUNCE_MS) return;
    saveNow();
}

} // namespace cdi::storage::config
