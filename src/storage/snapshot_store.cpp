#include "storage/snapshot_store.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "scope/adc_sampler.h"

namespace cdi::storage::snap {
namespace {

constexpr const char* DIR = "/snap";
constexpr const char* EXT = ".bin";

String pathFor(const String& safe_name) {
    return String(DIR) + "/" + safe_name + EXT;
}

bool ensureMounted() {
    static bool s_mounted = false;
    if (s_mounted) return true;
    if (!LittleFS.begin(true)) {
        Serial.println("[snap] LittleFS mount failed");
        return false;
    }
    s_mounted = true;
    return true;
}

} // anonymous

bool init() {
    if (!ensureMounted()) return false;
    if (!LittleFS.exists(DIR)) LittleFS.mkdir(DIR);
    return true;
}

String sanitize(const char* raw) {
    if (!raw) return String();
    String s;
    for (size_t i = 0; raw[i] && i < 32; i++) {
        char c = raw[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '-' || c == '_';
        if (ok) s += c;
    }
    return s;
}

bool save(const String& safe_name,
          uint32_t rate_hz,
          const cdi::scope::Snapshot& snap) {
    if (safe_name.length() == 0) return false;
    if (!init()) return false;

    File f = LittleFS.open(pathFor(safe_name), "w");
    if (!f) return false;

    f.write((const uint8_t*)&rate_hz, sizeof(rate_hz));
    for (uint32_t i = 0; i < snap.buf_size; i++) {
        uint32_t idx = (snap.start_idx + i) % snap.buf_size;
        uint16_t c1 = snap.ch1[idx];
        uint16_t c2 = snap.ch2[idx];
        f.write((const uint8_t*)&c1, sizeof(c1));
        f.write((const uint8_t*)&c2, sizeof(c2));
    }
    f.close();
    return true;
}

int fileSize(const String& safe_name) {
    if (safe_name.length() == 0) return -1;
    String p = pathFor(safe_name);
    if (!LittleFS.exists(p)) return -1;
    File f = LittleFS.open(p, "r");
    if (!f) return -1;
    int sz = (int)f.size();
    f.close();
    return sz;
}

int load(const String& safe_name, uint8_t* buf, size_t buf_size) {
    if (safe_name.length() == 0 || !buf) return -1;
    String p = pathFor(safe_name);
    if (!LittleFS.exists(p)) return -1;
    File f = LittleFS.open(p, "r");
    if (!f) return -1;
    size_t want = f.size();
    if (want > buf_size) want = buf_size;
    int n = (int)f.read(buf, want);
    f.close();
    return n;
}

void remove(const String& safe_name) {
    if (safe_name.length() == 0) return;
    String p = pathFor(safe_name);
    if (LittleFS.exists(p)) LittleFS.remove(p);
}

void list(JsonArray& out) {
    if (!init()) return;
    File dir = LittleFS.open(DIR);
    if (!dir || !dir.isDirectory()) return;
    File f = dir.openNextFile();
    while (f) {
        String n = String(f.name());
        if (n.endsWith(EXT)) {
            n.replace(EXT, "");
            JsonObject o = out.add<JsonObject>();
            o["name"] = n;
            o["size"] = f.size();
            uint32_t rate = 0;
            f.read((uint8_t*)&rate, sizeof(rate));
            o["rate"] = rate;
        }
        f = dir.openNextFile();
    }
}

bool toCsv(const String& safe_name, String& out) {
    if (safe_name.length() == 0) return false;
    String p = pathFor(safe_name);
    if (!LittleFS.exists(p)) return false;
    File f = LittleFS.open(p, "r");
    if (!f) return false;

    uint32_t rate = 0;
    f.read((uint8_t*)&rate, sizeof(rate));
    uint32_t dt_us = rate > 0 ? (1000000UL / rate) : 100;

    out = "";
    out.reserve(16384);
    out += "time_us,ch1,ch2\n";
    uint32_t t = 0;
    while (f.available() >= 4) {
        uint16_t c1 = 0, c2 = 0;
        f.read((uint8_t*)&c1, sizeof(c1));
        f.read((uint8_t*)&c2, sizeof(c2));
        out += String(t); out += ',';
        out += String(c1); out += ',';
        out += String(c2); out += '\n';
        t += dt_us;
    }
    f.close();
    return true;
}

} // namespace cdi::storage::snap
