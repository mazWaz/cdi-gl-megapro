// Edge-stream snapshot for offline debugging.
//
// Maintains a rolling RAM ring of the most-recent edge events that
// passed through scope::edge::buildFrame. The user can freeze the
// ring at any moment ("Simpan") and persist it as a binary file in
// LittleFS — survives reboot, browsable / downloadable as CSV, or
// reloadable into the scope canvas as an overlay.
//
// Sizing: 2000 events × 6 B = 12 KB BSS. At a single-cylinder 4T
// idling near 1500 RPM (~100 fires/s × ~4 edges/fire = 400 ev/s)
// that's about 5 seconds of backlog — enough to see what happened
// around a misfire or RPM glitch.
//
// File layout (binary):
//   header [magic 0x53 0x4E ('SN'), version u8, _pad u8] (4 B)
//   count  u16
//   events count × {ts_us:u32, ch:u8, level:u8}        (6 B each)
//
// Filenames are kebab-sanitized into /snap/<name>.bin.
#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>
#include <ArduinoJson.h>

namespace cdi::scope::snapshot {

constexpr size_t  RING_CAPACITY  = 2000;
constexpr size_t  EVENT_BYTES    = 6;
constexpr uint8_t FILE_MAGIC_0   = 0x53;   // 'S'
constexpr uint8_t FILE_MAGIC_1   = 0x4E;   // 'N'
constexpr uint8_t FILE_VERSION   = 1;

struct Event {
    uint32_t ts_us;
    uint8_t  ch;
    uint8_t  level;
} __attribute__((packed));
static_assert(sizeof(Event) == EVENT_BYTES, "Event must be 6 bytes packed");

void   begin();                          // init LittleFS snap dir
void   record(uint32_t ts_us, uint8_t ch, uint8_t level);   // hot path
size_t buffered();                       // how many events currently in ring
size_t capacity();                       // RING_CAPACITY

// File ops — all paths kebab-sanitized at the boundary.
String sanitize(const char* raw);        // strip unsafe chars → kebab
bool   save(const String& name);         // snapshot ring → /snap/<name>.bin
bool   list(JsonArray out);              // {name, size_bytes, count}[]
int    fileSize(const String& name);     // -1 if missing
int    load(const String& name, uint8_t* buf, size_t cap);  // read into buf
bool   toCsv(const String& name, String& out);              // generate CSV
bool   remove(const String& name);

} // namespace cdi::scope::snapshot
