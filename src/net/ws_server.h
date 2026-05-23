// WebSocket server (single endpoint /ws).
//
// Hosts two binary streams + a JSON command protocol:
//   * scope edge events  (opcode 0xA7) — drives live scope UI
//   * telemetry          (opcode 0xB0) — 5 Hz state snapshot
//
// Must be initialized AFTER http_server::begin() because it attaches
// the AsyncWebSocket handler to the HTTP server instance.
//
// `tickBroadcast()` emits an edge frame (skipped if backpressure says
// any client is still draining its previous frame).
//
// `cleanup()` calls AsyncWebSocket::cleanupClients() — required to
// release closed connections.
#pragma once

namespace cdi::net::ws_server {

void begin();
void tickBroadcast();    // 20 fps scope frame (when SCOPE mode)
void tickTelemetry();    // 5 fps telemetry frame (always)
void tickFlame();        // 2 fps flame state JSON broadcast (replaces UI polling)
void cleanup();
int  clientCount();

} // namespace cdi::net::ws_server
