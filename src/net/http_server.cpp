#include "net/http_server.h"

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

#include "config.h"
#include "telemetry/datalog.h"
#include "scope/edge_snapshot.h"
#include "ui_pages.h"     // gzipped UI files embedded as PROGMEM

namespace cdi::net::http_server {
namespace {

AsyncWebServer s_server(cdi::config::AP_HTTP_PORT);

// ─── Serve embedded gzipped UI files from PROGMEM ─────────────────
// Pattern dipinjam dari WLED. Filesystem (LittleFS) dihilangkan untuk
// UI assets — semua di-embed di firmware binary via PROGMEM.
//   * OTA firmware update = otomatis update UI sekaligus
//   * Gzip ratio ~28% = 47 KB total (vs 164 KB raw)
//   * No LittleFS corruption risk
//   * Browser auto-decompress karena Content-Encoding: gzip
//
// `beginResponse_P` adalah AsyncWebServer helper untuk PROGMEM data
// — tidak copy ke heap, langsung stream dari flash.
void sendEmbeddedPage(AsyncWebServerRequest* req,
                      const cdi::ui::pages::PageEntry& p) {
    AsyncWebServerResponse* r =
        req->beginResponse_P(200, p.mime, p.data, p.len);
    r->addHeader("Content-Encoding", "gzip");
    r->addHeader("Cache-Control", "max-age=300");   // 5 min, cukup untuk session
    req->send(r);
    Serial.printf("[HTTP] %s -> embed %s (%u B gz)\n",
                  req->url().c_str(), p.path, (unsigned)p.len);
}

// Generic handler: lookup request path di kPages[], serve kalau match.
void handleEmbedded(AsyncWebServerRequest* req) {
    const char* url = req->url().c_str();
    for (size_t i = 0; i < cdi::ui::pages::kPageCount; i++) {
        const auto& p = cdi::ui::pages::kPages[i];
        if (strcmp(url, p.path) == 0) {
            sendEmbeddedPage(req, p);
            return;
        }
    }
    // Fallback: kalau path tidak dikenali, serve index "/" → dashboard.
    for (size_t i = 0; i < cdi::ui::pages::kPageCount; i++) {
        const auto& p = cdi::ui::pages::kPages[i];
        if (strcmp(p.path, "/") == 0) {
            sendEmbeddedPage(req, p);
            return;
        }
    }
    req->send(404, "text/plain", "no embedded page");
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
    // ─── Embedded UI routes ──────────────────────────────────────
    // Register handler untuk tiap path di kPages[]. Generic
    // handleEmbedded() lookup by path dan serve dari PROGMEM.
    for (size_t i = 0; i < cdi::ui::pages::kPageCount; i++) {
        s_server.on(cdi::ui::pages::kPages[i].path, HTTP_GET, handleEmbedded);
    }
    // Alias: /index.html → "/" (some browsers/bookmarks pakai literal name)
    s_server.on("/index.html", HTTP_GET, handleEmbedded);

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

    // Unknown paths → serve embedded index (handleEmbedded fallback
    // to "/" kalau path tidak match). Won't trigger captive banner
    // karena probe routes di atas sudah match dulu.
    s_server.onNotFound(handleEmbedded);

    s_server.begin();
    Serial.println("[HTTP] server started (multi-page)");
}

AsyncWebServer& server() { return s_server; }

} // namespace cdi::net::http_server
