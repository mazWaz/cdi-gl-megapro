#include "storage/config_store.h"

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "core/advance_map.h"
#include "core/safety.h"
#include "core/spark_scheduler.h"
#include "core/shift_light.h"
#include "core/dwell_curve.h"
#include "core/launch_control.h"
#include "core/quickshifter.h"
#include "core/backfire.h"
#include "core/idle_rumble.h"
#include "core/exhaust_flame.h"
#include "core/alvp.h"
#include "core/engine_preset.h"
#include "core/pickup.h"

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

// ── Dual-core persistence ─────────────────────────────────────────
// NVS putString takes 10-50ms (flash erase + write). Doing that on
// the main loop (core 1) blocks pulser event drainage and degrades
// ignition jitter. We move the actual write to a task pinned on
// core 0 and use:
//   * binary semaphore `s_persistTrigger` — signal that work is
//     queued; the persist task blocks on it
//   * mutex `s_bufferMutex` — protects the shared pending-JSON
//     buffer between the main loop (producer) and the persist task
//     (consumer)
SemaphoreHandle_t s_persistTrigger = nullptr;
SemaphoreHandle_t s_bufferMutex    = nullptr;
String            s_pendingJson;        // protected by s_bufferMutex
TaskHandle_t      s_persistTask    = nullptr;

void persistTaskFn(void*) {
    Serial.printf("[config] persist task running on core %d\n", xPortGetCoreID());
    for (;;) {
        // Block forever until producer signals work is ready.
        if (xSemaphoreTake(s_persistTrigger, portMAX_DELAY) != pdTRUE) continue;

        // Snapshot the queued JSON under mutex, then release it
        // before doing the slow NVS write so producers can queue
        // the next one (coalesced — last write wins).
        String local;
        if (xSemaphoreTake(s_bufferMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            local = s_pendingJson;
            xSemaphoreGive(s_bufferMutex);
        } else {
            Serial.println("[config] persist: mutex timeout, skip");
            continue;
        }
        if (local.length() == 0) continue;

        // The actual long-running work — on core 0, so core 1's
        // pulser path is undisturbed.
        const uint32_t t0 = millis();
        Preferences prefs;
        if (!prefs.begin(NVS_NS, /*readOnly=*/false)) {
            Serial.println("[config] persist: NVS open fail (write)");
            continue;
        }
        const size_t n = prefs.putString(NVS_KEY, local);
        prefs.end();
        Serial.printf("[config] persist: %u bytes in %u ms (core %d)\n",
                      (unsigned)n, (unsigned)(millis() - t0), xPortGetCoreID());
    }
}

void buildJson(JsonDocument& doc) {
    doc["v"] = SCHEMA_VERSION;
    doc["preset_id"] = cdi::core::preset::currentId();
    doc["preset_modified"] = cdi::core::preset::isModified();

    // pickup override (set by auto-cal or manual entry)
    JsonObject pu = doc["pickup"].to<JsonObject>();
    pu["override"]    = cdi::core::pickup::hasOverride();
    pu["max_ref"]     = cdi::core::pickup::maxAdvanceRef();
    pu["magnet"]      = cdi::core::pickup::magnetWidth();
    pu["source"]      = cdi::core::pickup::source();

    // advance map
    JsonArray map = doc["advance_map"].to<JsonArray>();
    cdi::core::advance::active().serialize(map);

    // rev limits
    JsonObject rev = doc["rev"].to<JsonObject>();
    rev["main"]    = cdi::core::safety::mainLimitRpm();
    rev["overrev"] = cdi::core::safety::overrevLimitRpm();
    rev["no_sig"]  = cdi::core::safety::noSignalEnabled();

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
    sp["active_low"]     = cdi::core::spark::activeLow();
    sp["inductive"]      = cdi::core::spark::inductive();

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

    // idle rumble
    JsonObject ir = doc["idle_rumble"].to<JsonObject>();
    ir["enabled"]    = cdi::core::idle_rumble::isEnabled();
    ir["mode"]       = (uint8_t)cdi::core::idle_rumble::mode();
    ir["rpm_lo"]     = cdi::core::idle_rumble::rpmLo();
    ir["rpm_hi"]     = cdi::core::idle_rumble::rpmHi();
    ir["retard"]     = cdi::core::idle_rumble::maxRetardDeg();
    ir["skip_n"]     = cdi::core::idle_rumble::skipPattern();
    ir["sustain_ms"] = cdi::core::idle_rumble::sustainMs();
    ir["min_uptime"] = cdi::core::idle_rumble::minUptimeSec();

    // exhaust flame
    JsonObject fl = doc["flame"].to<JsonObject>();
    fl["enabled"] = cdi::core::flame::isEnabled();
    fl["mode"]    = (uint8_t)cdi::core::flame::mode();
}

void applyJson(const JsonDocument& doc) {
    // ── Restore pickup override BEFORE preset::apply, so that apply
    // respects the user's calibrated geometry instead of clobbering
    // it with factory defaults.
    {
        JsonObjectConst pu = doc["pickup"];
        if (!pu.isNull()) {
            const bool ov = pu["override"] | false;
            if (ov) {
                cdi::core::pickup::setMaxAdvanceRef(pu["max_ref"] | cdi::core::pickup::maxAdvanceRef());
                cdi::core::pickup::setMagnetWidth(pu["magnet"]     | cdi::core::pickup::magnetWidth());
                cdi::core::pickup::setSource(pu["source"]          | "preset");
                cdi::core::pickup::setOverride(true);
            }
        }
    }

    // ── Apply preset (sets defaults except pickup if overridden), then user overrides
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
            cdi::core::safety::setNoSignalEnabled(rev["no_sig"] | false);
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
            cdi::core::spark::setActiveLow(sp["active_low"] | false);
            cdi::core::spark::setInductive(sp["inductive"]  | true);
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
    // idle rumble
    {
        JsonObjectConst ir = doc["idle_rumble"];
        if (!ir.isNull()) {
            cdi::core::idle_rumble::setMode(
                (cdi::IdleRumbleMode)(ir["mode"] | (int)cdi::IdleRumbleMode::SUBTLE));
            cdi::core::idle_rumble::setRpmBand(ir["rpm_lo"] | 1000, ir["rpm_hi"] | 2000);
            cdi::core::idle_rumble::setMaxRetardDeg(ir["retard"] | 3.0f);
            cdi::core::idle_rumble::setSkipPattern(ir["skip_n"] | 7);
            cdi::core::idle_rumble::setSustainMs(ir["sustain_ms"] | 3000);
            cdi::core::idle_rumble::setMinUptimeSec(ir["min_uptime"] | 60);
            cdi::core::idle_rumble::setEnabled(ir["enabled"] | false);
        }
    }
    // exhaust flame
    {
        JsonObjectConst fl = doc["flame"];
        if (!fl.isNull()) {
            uint8_t m = fl["mode"] | (uint8_t)cdi::FlameMode::OFF;
            if (m > 2) m = 0;
            cdi::core::flame::setMode((cdi::FlameMode)m);
            cdi::core::flame::setEnabled(fl["enabled"] | false);
        }
    }

    if (modified) cdi::core::preset::markModifiedFlag();
}

} // anonymous

void begin() {
    // Sync primitives must exist before any saveNow() call.
    s_persistTrigger = xSemaphoreCreateBinary();
    s_bufferMutex    = xSemaphoreCreateMutex();
    if (!s_persistTrigger || !s_bufferMutex) {
        Serial.println("[config] FATAL: semaphore alloc fail; persistence DISABLED");
        return;
    }
    // Pin persist task to core 0 — opposite of Arduino's loopTask
    // (core 1), so NVS flash writes never compete with pulser /
    // spark-scheduler latency on core 1.
    BaseType_t ok = xTaskCreatePinnedToCore(
        persistTaskFn, "cdi_persist",
        /*stack words*/ 4096,
        /*arg*/         nullptr,
        /*priority*/    1,         // above idle, below loopTask
        &s_persistTask,
        /*core*/        0);
    if (ok != pdPASS) {
        Serial.println("[config] FATAL: persist task spawn fail");
    } else {
        Serial.println("[config] persist task pinned to core 0");
    }
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
    // Producer side (main loop / WS callback context, core 1):
    // build the JSON snapshot from module state, stash into the
    // shared buffer under mutex, signal the persist task to do the
    // actual NVS write off-core. Returns immediately even if the
    // physical write takes 10-50 ms.
    if (!s_persistTrigger || !s_bufferMutex) {
        Serial.println("[config] persist not initialized — saveNow skipped");
        return;
    }

    JsonDocument doc;
    buildJson(doc);
    String json;
    serializeJson(doc, json);

    // 5 ms is a generous 50× safety margin over the actual contended
    // critical section (~100 µs String copy in the persist task).
    // Originally 50 ms, which was wrong for a hot-loop caller — at
    // worst it would block the main loop for nearly a full safety
    // tick. If somehow contended longer than 5 ms, drop the snapshot
    // (debounce will re-fire on the next tick anyway).
    if (xSemaphoreTake(s_bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_pendingJson = json;       // last write wins (debounce-coalesce)
        xSemaphoreGive(s_bufferMutex);
    } else {
        Serial.println("[config] saveNow: mutex busy, drop this snapshot");
        return;
    }
    xSemaphoreGive(s_persistTrigger);    // wake persist task
    s_dirty = false;
}

void tick() {
    if (!s_dirty) return;
    if (millis() - s_lastDirtyMs < DEBOUNCE_MS) return;
    saveNow();
}

} // namespace cdi::storage::config
