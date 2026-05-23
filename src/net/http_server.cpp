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
    // ETag check: kalau browser kirim If-None-Match yang match, return
    // 304 Not Modified — zero body, save 4-16 KB bandwidth per request.
    // ETag bentuk: "abc12345" (8 hex chars from SHA1(gz)). Generated
    // at build time per file, stable across rebuilds dengan content sama.
    if (req->hasHeader("If-None-Match")) {
        const auto* hdr = req->getHeader("If-None-Match");
        if (hdr && hdr->value().equals(p.etag)) {
            AsyncWebServerResponse* r304 = req->beginResponse(304);
            r304->addHeader("ETag", p.etag);
            req->send(r304);
            return;
        }
    }
    AsyncWebServerResponse* r =
        req->beginResponse_P(200, p.mime, p.data, p.len);
    r->addHeader("Content-Encoding", "gzip");
    r->addHeader("Cache-Control", "max-age=300, must-revalidate");
    r->addHeader("ETag", p.etag);
    req->send(r);
    Serial.printf("[HTTP] %s -> embed %s (%u B gz, etag=%s)\n",
                  req->url().c_str(), p.path, (unsigned)p.len, p.etag);
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
// Setiap OS modern probes URL well-known untuk detect internet.
// Strategi lama: respond dengan exact success payload supaya banner
// tidak muncul → user harus manually ketik 192.168.4.1. Strategi
// sekarang (per user request): TRIGGER captive portal supaya browser
// AUTO-OPEN ke dashboard kita.
//
// Cara: response 302 redirect ke "/" (dashboard). OS lihat respond
// ≠ expected success → flag network "needs sign-in" → trigger sistem:
//   * Android : notification "Sign in to network" → tap → buka browser
//   * iOS/Mac : auto-open captive browser ke URL → follow redirect
//   * Windows : notif "No internet, limited" → klik → buka browser
//
// Reference probe URLs per-OS:
//   Android  /generate_204, /gen_204            (Google connectivity check)
//   Windows  /ncsi.txt, /connecttest.txt        (NCSI)
//   iOS/Mac  /hotspot-detect.html, /library/test/success.html
//   Apple    /canonical.html                    (newer iOS)
//   Mozilla  /success.txt, /canonical.html       (Firefox)
//
// Helper: 302 redirect ke dashboard. Browser follow → dashboard load.

void portalRedirect(AsyncWebServerRequest* req) {
    AsyncWebServerResponse* r = req->beginResponse(302, "text/plain",
                                                   "captive portal redirect");
    r->addHeader("Location", "http://192.168.4.1/");
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
    Serial.printf("[HTTP] captive probe %s -> 302 redirect to /\n",
                  req->url().c_str());
}

// Apple-specific: macOS captive browser kadang lebih happy dengan HTML
// yang JELAS bukan "Success" + ada meta-refresh ke portal. 302 sometimes
// gets cached oleh OS sebelum captive browser open. HTML approach lebih
// reliable di iOS 14+.
void portalAppleHtml(AsyncWebServerRequest* req) {
    const char* body =
        "<!DOCTYPE html><html><head>"
        "<meta http-equiv=\"refresh\" content=\"0; url=http://192.168.4.1/\">"
        "<title>CDI Portal</title></head>"
        "<body><a href=\"http://192.168.4.1/\">CDI//MGPRO portal</a></body>"
        "</html>";
    AsyncWebServerResponse* r = req->beginResponse(200, "text/html", body);
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
    Serial.printf("[HTTP] apple probe %s -> portal HTML\n",
                  req->url().c_str());
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
    // (ESP32Async fork track semua headers otomatis — tidak perlu
    // collectHeaders explicit untuk If-None-Match.)

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

    // OS captive-portal probes — TRIGGER captive portal di semua OS
    // dengan respond ≠ expected success. Browser akan auto-open
    // ke dashboard kita.

    // Android (Google Chrome connectivity check):
    s_server.on("/generate_204",              HTTP_GET, portalRedirect);
    s_server.on("/gen_204",                   HTTP_GET, portalRedirect);

    // Apple (iOS, macOS, iPadOS):
    s_server.on("/hotspot-detect.html",       HTTP_GET, portalAppleHtml);
    s_server.on("/library/test/success.html", HTTP_GET, portalAppleHtml);

    // Windows NCSI:
    s_server.on("/ncsi.txt",                  HTTP_GET, portalRedirect);
    s_server.on("/connecttest.txt",           HTTP_GET, portalRedirect);

    // Firefox / Mozilla:
    s_server.on("/success.txt",               HTTP_GET, portalRedirect);
    s_server.on("/canonical.html",            HTTP_GET, portalAppleHtml);
    s_server.on("/redirect",                  HTTP_GET, portalRedirect);

    // Unknown paths → serve embedded index (handleEmbedded fallback
    // to "/" kalau path tidak match). Won't trigger captive banner
    // karena probe routes di atas sudah match dulu.
    s_server.onNotFound(handleEmbedded);

    s_server.begin();
    Serial.println("[HTTP] server started (multi-page)");
}

AsyncWebServer& server() { return s_server; }

} // namespace cdi::net::http_server
