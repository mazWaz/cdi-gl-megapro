// Over-the-air firmware update via HTTP multipart upload.
//
// Endpoint: POST /api/ota/upload
//   Form field "firmware" → .bin (typically firmware.bin from PIO build)
//
// Flow:
//   1. Reject if `cdi::core::mode::current() != SCOPE` — refuse to OTA
//      while engine is monitored (risk: dropping spark mid-firing).
//   2. ESP32 streams chunks into `Update.write()`, writing to the
//      inactive OTA partition.
//   3. On success, set the new partition active and reboot.
//   4. If new firmware crashes >3× on boot, ESP-IDF's app rollback
//      flag returns to the previous valid partition automatically.
//
// Front-end uses XHR with progress events to render a bar.
#pragma once

class AsyncWebServer;

namespace cdi::net::ota {

void registerRoutes(AsyncWebServer& srv);

} // namespace cdi::net::ota
