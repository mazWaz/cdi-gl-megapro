// HTTP server (AsyncWebServer wrapper).
//
// Owns the global AsyncWebServer instance on port AP_HTTP_PORT.
// Registers static routes for index.html, snapshot CSV download, and
// the OS-specific captive portal probe URLs (all return index.html so
// the HP browser auto-opens the UI).
//
// Other modules (ws_server) attach their own handlers via `server()`.
#pragma once

class AsyncWebServer;

namespace cdi::net::http_server {

// Build routes and start listening.
void begin();

// Underlying AsyncWebServer instance. Use to add custom handlers
// (e.g. ws_server attaches its AsyncWebSocket here).
AsyncWebServer& server();

} // namespace cdi::net::http_server
