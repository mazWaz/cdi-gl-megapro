#include "scope/edge_snapshot.h"

#include <LittleFS.h>
#include <cstring>

namespace cdi::scope::snapshot {
namespace {

constexpr const char* SNAP_DIR = "/snap";

Event   s_ring[RING_CAPACITY];
size_t  s_write_idx = 0;     // next slot to fill
size_t  s_count     = 0;     // up to RING_CAPACITY

String pathFor(const String& name) {
    String p = SNAP_DIR;
    p += "/";
    p += name;
    p += ".bin";
    return p;
}

} // anonymous

void begin() {
    if (!LittleFS.exists(SNAP_DIR)) {
        LittleFS.mkdir(SNAP_DIR);
    }
}

void record(uint32_t ts_us, uint8_t ch, uint8_t level) {
    s_ring[s_write_idx].ts_us = ts_us;
    s_ring[s_write_idx].ch    = ch;
    s_ring[s_write_idx].level = level;
    s_write_idx = (s_write_idx + 1) % RING_CAPACITY;
    if (s_count < RING_CAPACITY) s_count++;
}

size_t buffered() { return s_count; }
size_t capacity() { return RING_CAPACITY; }

String sanitize(const char* raw) {
    String out;
    if (!raw) return out;
    out.reserve(32);
    for (size_t i = 0; raw[i] && i < 32; i++) {
        const char c = raw[i];
        if ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
             c == '-' || c == '_') out += c;
    }
    return out;
}

bool save(const String& name) {
    if (name.length() == 0) return false;
    if (s_count == 0)       return false;

    File f = LittleFS.open(pathFor(name), "w");
    if (!f) {
        Serial.printf("[snap] save fail: open %s\n", pathFor(name).c_str());
        return false;
    }

    // Header
    uint8_t hdr[4] = { FILE_MAGIC_0, FILE_MAGIC_1, FILE_VERSION, 0 };
    f.write(hdr, 4);

    uint16_t cnt = (uint16_t)s_count;
    f.write((uint8_t*)&cnt, 2);

    // Walk events in time order (oldest → newest).
    const size_t start = (s_count == RING_CAPACITY) ? s_write_idx : 0;
    for (size_t i = 0; i < s_count; i++) {
        const size_t idx = (start + i) % RING_CAPACITY;
        f.write((uint8_t*)&s_ring[idx], EVENT_BYTES);
    }
    f.close();
    Serial.printf("[snap] saved %u events → %s\n", (unsigned)s_count, pathFor(name).c_str());
    return true;
}

bool list(JsonArray out) {
    File root = LittleFS.open(SNAP_DIR);
    if (!root || !root.isDirectory()) return false;
    File f = root.openNextFile();
    while (f) {
        const char* fn = f.name();
        // Strip trailing ".bin" if present for the user-facing name.
        String name = fn;
        // ESP32 LittleFS returns basename without leading slash in some
        // versions and with full path in others — normalize both.
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        if (name.endsWith(".bin")) name = name.substring(0, name.length() - 4);

        // Read header to get event count cheaply.
        uint16_t cnt = 0;
        if (f.size() >= 6) {
            uint8_t hdr[6];
            f.seek(0);
            f.read(hdr, 6);
            if (hdr[0] == FILE_MAGIC_0 && hdr[1] == FILE_MAGIC_1) {
                memcpy(&cnt, &hdr[4], 2);
            }
        }
        JsonObject o = out.add<JsonObject>();
        o["name"]  = name;
        o["size"]  = (uint32_t)f.size();
        o["count"] = cnt;
        f = root.openNextFile();
    }
    return true;
}

int fileSize(const String& name) {
    if (name.length() == 0) return -1;
    File f = LittleFS.open(pathFor(name), "r");
    if (!f) return -1;
    const int sz = f.size();
    f.close();
    return sz;
}

int load(const String& name, uint8_t* buf, size_t cap) {
    if (!buf || cap == 0) return -1;
    File f = LittleFS.open(pathFor(name), "r");
    if (!f) return -1;
    const size_t sz = f.size();
    if (sz > cap) { f.close(); return -1; }
    const int n = f.read(buf, sz);
    f.close();
    return n;
}

bool toCsv(const String& name, String& out) {
    File f = LittleFS.open(pathFor(name), "r");
    if (!f) return false;
    if (f.size() < 6) { f.close(); return false; }

    uint8_t hdr[4];
    f.read(hdr, 4);
    if (hdr[0] != FILE_MAGIC_0 || hdr[1] != FILE_MAGIC_1) {
        f.close(); return false;
    }
    uint16_t cnt = 0;
    f.read((uint8_t*)&cnt, 2);

    out = String("time_us,ch,level\n");
    out.reserve((size_t)cnt * 16);   // ~ "4294967295,0,1\n"

    Event ev;
    for (uint16_t i = 0; i < cnt; i++) {
        if (f.read((uint8_t*)&ev, EVENT_BYTES) != EVENT_BYTES) break;
        out += String(ev.ts_us);
        out += ',';
        out += String((unsigned)ev.ch);
        out += ',';
        out += String((unsigned)ev.level);
        out += '\n';
    }
    f.close();
    return true;
}

bool remove(const String& name) {
    if (name.length() == 0) return false;
    return LittleFS.remove(pathFor(name));
}

} // namespace cdi::scope::snapshot
