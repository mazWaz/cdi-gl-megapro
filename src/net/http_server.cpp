#include "net/http_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "config.h"
#include "storage/snapshot_store.h"
#include "telemetry/datalog.h"

namespace cdi::net::http_server {
namespace {

namespace snap = cdi::storage::snap;

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

void handleDownload(AsyncWebServerRequest* req) {
    if (!req->hasParam("name")) {
        req->send(400, "text/plain", "missing name");
        return;
    }
    String name = snap::sanitize(req->getParam("name")->value().c_str());
    if (name.length() == 0) {
        req->send(400, "text/plain", "invalid name");
        return;
    }
    String csv;
    if (!snap::toCsv(name, csv)) {
        req->send(404, "text/plain", "not found");
        return;
    }
    AsyncWebServerResponse* resp = req->beginResponse(200, "text/csv", csv);
    resp->addHeader("Content-Disposition", "attachment; filename=\"" + name + ".csv\"");
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
    s_server.on("/download",       HTTP_GET, handleDownload);
    s_server.on("/datalog.csv",    HTTP_GET, handleDatalogCsv);

    // OS captive-portal probes — return index.html so HP auto-opens UI.
    s_server.on("/generate_204",              HTTP_GET, sendIndex);
    s_server.on("/gen_204",                   HTTP_GET, sendIndex);
    s_server.on("/hotspot-detect.html",       HTTP_GET, sendIndex);
    s_server.on("/library/test/success.html", HTTP_GET, sendIndex);
    s_server.on("/ncsi.txt",                  HTTP_GET, sendIndex);
    s_server.on("/connecttest.txt",           HTTP_GET, sendIndex);
    s_server.on("/redirect",                  HTTP_GET, sendIndex);
    s_server.on("/canonical.html",            HTTP_GET, sendIndex);
    s_server.on("/success.txt",               HTTP_GET, sendIndex);

    // Anything else also goes to index → keeps captive portal happy.
    s_server.onNotFound(sendIndex);

    s_server.begin();
    Serial.println("[HTTP] server started (multi-page)");
}

AsyncWebServer& server() { return s_server; }

} // namespace cdi::net::http_server
