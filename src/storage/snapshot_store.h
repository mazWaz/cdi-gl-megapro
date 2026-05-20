// Persistent storage for scope-mode waveform snapshots.
//
// Files live under `/snap/<name>.bin` on LittleFS. Binary layout:
//   [4B uint32 LE sample_rate_hz] [N × (uint16 CH1, uint16 CH2)]
//
// Public API hides the directory layout and binary format so that
// callers (ws_server, http_server) only deal in sanitized names.
//
// Thread model: all calls happen from loop()/WS-callback context.
// Never call from ISR — LittleFS reads/writes touch flash.
#pragma once

#include <cstdint>
#include <Arduino.h>      // for String
#include <ArduinoJson.h>  // for JsonArray

namespace cdi::scope { struct Snapshot; }

namespace cdi::storage::snap {

// Mount LittleFS (if not already) and ensure `/snap/` exists.
// Returns true on success. Idempotent.
bool init();

// Reduce a user-supplied name to alnum + '-' + '_', max 32 chars.
// Empty result means "invalid"; callers should reject before save.
String sanitize(const char* raw);

// Persist the current scope ring buffer to `/snap/<safe_name>.bin`.
// `safe_name` must already be sanitized. Returns true on success.
bool save(const String& safe_name,
          uint32_t rate_hz,
          const cdi::scope::Snapshot& snap);

// File size on disk (bytes), or -1 if the file doesn't exist.
int fileSize(const String& safe_name);

// Read the entire file into `buf`. `buf_size` must be >= fileSize().
// Returns bytes read, or -1 on error.
int load(const String& safe_name, uint8_t* buf, size_t buf_size);

// Delete `/snap/<safe_name>.bin`. No-op if missing.
void remove(const String& safe_name);

// Append each existing snapshot as a JsonObject {name,size,rate} to `out`.
void list(JsonArray& out);

// Convert a snapshot file to CSV text (`time_us,ch1,ch2`). Returns
// false if the file is missing. `out` is appended/replaced.
bool toCsv(const String& safe_name, String& out);

} // namespace cdi::storage::snap
