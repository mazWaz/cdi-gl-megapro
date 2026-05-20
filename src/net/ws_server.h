// WebSocket server (single endpoint /ws).
//
// Hosts the live binary frame stream (scope waveform) and a JSON
// text-command protocol for control (setRate, pause, snapshot
// save/load/list/delete).
//
// Must be initialized AFTER http_server::begin() because it attaches
// the AsyncWebSocket handler to the HTTP server instance.
//
// `tickBroadcast()` is called from loop() at a fixed cadence; it
// builds & sends the live scope frame (skipping if backpressure says
// any client is still draining its previous frame).
//
// `cleanup()` calls AsyncWebSocket::cleanupClients() — required to
// release closed connections.
#pragma once

namespace cdi::net::ws_server {

void begin();
void tickBroadcast();    // 20 fps scope frame (when SCOPE mode)
void tickTelemetry();    // 5 fps telemetry frame (always)
void cleanup();
int  clientCount();

} // namespace cdi::net::ws_server
