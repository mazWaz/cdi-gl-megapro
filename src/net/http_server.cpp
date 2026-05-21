#include "net/http_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "config.h"
#include "telemetry/datalog.h"
#include "scope/edge_snapshot.h"

namespace cdi::net::http_server {
namespace {

AsyncWebServer s_server(cdi::config::AP_HTTP_PORT);

void logReq(AsyncWebServerRequest* req, const char* file) {
    Serial.printf("[HTTP] %s -> %s (%d B)\n",
                  req->url().c_str(), file,
                  LittleFS.exists(file) ? (int)LittleFS.open(file, "r").size() : -1);
}

void sendIndex(AsyncWebServerRequest* req) {
    logReq(req, "/index.html");
    req->send(LittleFS, "/index.html", "text/html");
}
void sendMap(AsyncWebServerRequest* req) {
    logReq(req, "/map.html");
    req->send(LittleFS, "/map.html", "text/html");
}
void sendScope(AsyncWebServerRequest* req) {
    logReq(req, "/scope.html");
    req->send(LittleFS, "/scope.html", "text/html");
}
void sendSettings(AsyncWebServerRequest* req) {
    logReq(req, "/settings.html");
    req->send(LittleFS, "/settings.html", "text/html");
}
void sendStyle(AsyncWebServerRequest* req) {
    logReq(req, "/style.css");
    AsyncWebServerResponse* r = req->beginResponse(LittleFS, "/style.css", "text/css");
    r->addHeader("Cache-Control", "no-cache");
    req->send(r);
}
void sendApp(AsyncWebServerRequest* req) {
    logReq(req, "/app.js");
    AsyncWebServerResponse* r = req->beginResponse(LittleFS, "/app.js", "application/javascript");
    r->addHeader("Cache-Control", "no-cache");
    req->send(r);
}

// ── Captive-portal probe responders ──────────────────────────────
//
// Each major OS pings a well-known URL to test whether the network
// has real internet. If we return ANYTHING other than the exact
// expected response, the OS thinks the network is a captive portal
// requiring sign-in and pops the "Sign in to network" banner —
// repeatedly on Android, every ~10 s. That banner is annoying and
// drops the user into a stripped-down mini-browser instead of full
// Chrome.
//
// We respond with the exact expected payload so the OS marks the
// network as "no internet" silently and lets the user navigate to
// 192.168.4.1 normally in real Chrome.
//
// Reference probe URLs:
//   Android  /generate_204 /gen_204                  → 204 empty
//   Windows  /ncsi.txt                               → "Microsoft NCSI"
//   Windows  /connecttest.txt                        → "Microsoft Connect Test"
//   iOS/Mac  /hotspot-detect.html /library/test/success.html
//                                                    → HTML containing "Success"
//   Mozilla  /success.txt                            → "success"
void probeEmpty(AsyncWebServerRequest* req) {
    // 204 No Content — silent "internet works"
    req->send(204);
}
void probeNcsi(AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "Microsoft NCSI");
}
void probeConnectTest(AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "Microsoft Connect Test");
}
void probeAppleSuccess(AsyncWebServerRequest* req) {
    req->send(200, "text/html",
              "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}
void probeMozillaSuccess(AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "success");
}

void handleSnapshotCsv(AsyncWebServerRequest* req) {
    if (!req->hasParam("name")) {
        req->send(400, "text/plain", "missing name"); return;
    }
    String name = cdi::scope::snapshot::sanitize(req->getParam("name")->value().c_str());
    if (name.length() == 0) {
        req->send(400, "text/plain", "invalid name"); return;
    }
    String csv;
    if (!cdi::scope::snapshot::toCsv(name, csv)) {
        req->send(404, "text/plain", "not found"); return;
    }
    AsyncWebServerResponse* r = req->beginResponse(200, "text/csv", csv);
    r->addHeader("Content-Disposition", "attachment; filename=\"" + name + ".csv\"");
    req->send(r);
}

void handleDatalogCsv(AsyncWebServerRequest* req) {
    String csv;
    if (!cdi::telemetry::datalog::fillCsv(csv)) {
        req->send(404, "text/plain", "datalog empty");
        return;
    }
    char ts[24];
    snprintf(ts, sizeof(ts), "datalog_%lu", (unsigned long)millis());
    AsyncWebServerResponse* resp = req->beginResponse(200, "text/csv", csv);
    String fn = String("datalog_") + String(millis()) + ".csv";
    resp->addHeader("Content-Disposition", "attachment; filename=\"" + fn + "\"");
    req->send(resp);
}

} // anonymous

void begin() {
    // Page routes — each HTML served with correct MIME.
    s_server.on("/",               HTTP_GET, sendIndex);
    s_server.on("/index.html",     HTTP_GET, sendIndex);
    s_server.on("/map.html",       HTTP_GET, sendMap);
    s_server.on("/scope.html",     HTTP_GET, sendScope);
    s_server.on("/settings.html",  HTTP_GET, sendSettings);

    // Shared assets — explicit MIME so phone browsers parse them.
    s_server.on("/style.css",      HTTP_GET, sendStyle);
    s_server.on("/app.js",         HTTP_GET, sendApp);

    // Dynamic endpoints.
    s_server.on("/datalog.csv",    HTTP_GET, handleDatalogCsv);
    s_server.on("/snapshot.csv",   HTTP_GET, handleSnapshotCsv);

    // OS captive-portal probes — respond with the exact payload each
    // OS expects so they DON'T pop a "Sign in to network" banner.
    // The user just opens a real browser to http://192.168.4.1/.
    s_server.on("/generate_204",              HTTP_GET, probeEmpty);
    s_server.on("/gen_204",                   HTTP_GET, probeEmpty);
    s_server.on("/hotspot-detect.html",       HTTP_GET, probeAppleSuccess);
    s_server.on("/library/test/success.html", HTTP_GET, probeAppleSuccess);
    s_server.on("/ncsi.txt",                  HTTP_GET, probeNcsi);
    s_server.on("/connecttest.txt",           HTTP_GET, probeConnectTest);
    s_server.on("/success.txt",               HTTP_GET, probeMozillaSuccess);
    s_server.on("/canonical.html",            HTTP_GET, probeAppleSuccess);
    s_server.on("/redirect",                  HTTP_GET, probeEmpty);

    // Unknown paths → serve index.html so a legitimate browser typing
    // a wrong URL still lands on the dashboard (won't trigger captive
    // banner because the explicit probe routes above already won).
    s_server.onNotFound(sendIndex);

    s_server.begin();
    Serial.println("[HTTP] server started (multi-page)");
}

AsyncWebServer& server() { return s_server; }

} // namespace cdi::net::http_server
