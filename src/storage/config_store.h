// Persistent device configuration.
//
// All tunable parameters live in their owning module's RAM at runtime.
// This store snapshots the lot to `/config.json` on LittleFS so that
// they survive reboots, brown-outs, and OTA updates.
//
// Flow:
//   1. main.cpp setup() calls load() AFTER each module's begin()
//      but BEFORE mode::begin() — so loaded values are active by the
//      time the engine starts firing.
//   2. WS handlers that change settings call markDirty().
//   3. main.cpp loop() calls tick() periodically. A debounced timer
//      (~1.5 s after last change) flushes dirty state to flash —
//      avoids flash wear from rapid slider edits.
//   4. saveNow() forces an immediate flush (used before reboot).
#pragma once

namespace cdi::storage::config {

void begin();        // init (creates default file if missing)
void load();         // read JSON, distribute to modules
void markDirty();    // schedule a save
void tick();         // call from loop() — flush if dirty + debounce elapsed
void saveNow();      // synchronous flush

} // namespace cdi::storage::config
