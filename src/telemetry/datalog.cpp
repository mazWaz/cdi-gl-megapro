#include "telemetry/datalog.h"

#include <Arduino.h>

#include "telemetry/live_stats.h"

namespace cdi::telemetry::datalog {
namespace {

struct Entry {
    uint32_t ts_ms;
    uint16_t rpm;
    int16_t  advance_x10;
    int16_t  jitter_us;
    uint16_t vbat_mv;
    uint8_t  flags;          // bit0 armed, bit1 launch_active, bit2 qs, bit3 backfire
    uint8_t  state_byte;     // (cut_mode << 4) | (alvp_state & 0x0F)
} __attribute__((packed));
static_assert(sizeof(Entry) == 14, "Entry layout drift");

constexpr size_t CAP = 1500;  // 30s @ 50Hz
Entry    s_ring[CAP];
size_t   s_head     = 0;     // next write slot
size_t   s_count    = 0;     // valid entries (≤ CAP)

bool     s_recording = false;
uint32_t s_lastTickMs = 0;
uint32_t s_startMs   = 0;
constexpr uint32_t TICK_INTERVAL_MS = 20;   // 50 Hz

} // anonymous

void begin() {}

void start() {
    s_head = 0;
    s_count = 0;
    s_startMs = millis();
    s_lastTickMs = 0;
    s_recording = true;
    Serial.println("[datalog] start");
}

void stop() {
    s_recording = false;
    Serial.printf("[datalog] stop · %u entries\n", (unsigned)s_count);
}

bool     isRecording() { return s_recording; }
uint32_t entryCount()  { return (uint32_t)s_count; }
uint32_t capacity()    { return (uint32_t)CAP; }

uint32_t durationMs() {
    if (s_count < 2) return 0;
    size_t oldest = (s_head + CAP - s_count) % CAP;
    size_t newest = (s_head + CAP - 1) % CAP;
    return s_ring[newest].ts_ms - s_ring[oldest].ts_ms;
}

void clear() { s_head = 0; s_count = 0; }

void tick() {
    if (!s_recording) return;
    uint32_t now = millis();
    if (now - s_lastTickMs < TICK_INTERVAL_MS) return;
    s_lastTickMs = now;

    auto t = cdi::telemetry::snapshot();
    Entry& e = s_ring[s_head];
    e.ts_ms       = now - s_startMs;
    e.rpm         = t.rpm;
    e.advance_x10 = t.target_advance_x10;
    e.jitter_us   = t.last_jitter_us;
    e.vbat_mv     = t.vbat_mv;
    e.flags       = (t.armed ? 0x01 : 0)
                  | ((t.flags2 & 0x10) ? 0x02 : 0)   // launch_active
                  | ((t.flags2 & 0x20) ? 0x04 : 0)   // qs_active
                  | ((t.flags3 & 0x02) ? 0x08 : 0);  // backfire_active
    e.state_byte  = (uint8_t)((t.cut_mode & 0x0F) << 4) | (t.alvp_state & 0x0F);

    s_head = (s_head + 1) % CAP;
    if (s_count < CAP) s_count++;
}

bool fillCsv(String& out) {
    if (s_count == 0) return false;
    out = "";
    out.reserve((size_t)s_count * 60);
    out += "ts_ms,rpm,advance_deg,jitter_us,vbat_mv,armed,launch,qs,backfire,cut_mode,alvp_state\n";

    size_t oldest = (s_head + CAP - s_count) % CAP;
    for (size_t i = 0; i < s_count; i++) {
        const Entry& e = s_ring[(oldest + i) % CAP];
        char line[80];
        snprintf(line, sizeof(line), "%u,%u,%.1f,%d,%u,%d,%d,%d,%d,%u,%u\n",
                 (unsigned)e.ts_ms,
                 (unsigned)e.rpm,
                 (float)e.advance_x10 / 10.0f,
                 (int)e.jitter_us,
                 (unsigned)e.vbat_mv,
                 (e.flags & 0x01) ? 1 : 0,
                 (e.flags & 0x02) ? 1 : 0,
                 (e.flags & 0x04) ? 1 : 0,
                 (e.flags & 0x08) ? 1 : 0,
                 (unsigned)((e.state_byte >> 4) & 0x0F),
                 (unsigned)(e.state_byte & 0x0F));
        out += line;
    }
    return true;
}

} // namespace cdi::telemetry::datalog
