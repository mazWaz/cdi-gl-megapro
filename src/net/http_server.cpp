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
// Tujuan: bikin OS detect "no internet, captive portal" + biarkan
// user pilih "Tetap terhubung tanpa internet" → setelah itu OS stop
// spam probe. Browser bisa di-buka manual ke 192.168.4.1.
//
// Strategi: return 200 OK + HTML portal page (BUKAN exact success
// payload, BUKAN 302 redirect). Behaviour per-OS:
//
//   Android: probe gagal match expected "204 empty" → notif "Sign
//            in to network" muncul SEKALI. User tap → buka browser
//            ke portal URL. Dismiss notif → Android tanya "Tetap
//            terhubung walau tanpa internet?". User pilih ya →
//            jaringan di-whitelist, no more probes.
//
//   iOS/Mac: captive browser auto-open ke portal URL. User browse
//            atau dismiss. OS mark network sebagai captive-aware.
//
//   Windows: "No internet, limited" indicator di tray. User can
//            still browse manually.
//
// 302 redirect SEBELUMNYA terlalu agresif — Android terus re-probe
// karena response berubah tiap try, never settles ke "captive done"
// state. 200 + static HTML lebih cooperative.

void portalProbe(AsyncWebServerRequest* req) {
    // Static HTML — small payload, identical setiap request supaya
    // OS bisa hash-compare dan settle on "captive done" state.
    const char* body =
        "<!DOCTYPE html><html lang=\"id\"><head>"
        "<meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<meta http-equiv=\"refresh\" content=\"0; url=http://192.168.4.1/\">"
        "<title>CDI//MGPRO Portal</title>"
        "<style>"
        "body{font-family:monospace;background:#0a0d12;color:#e8e6df;"
        "margin:0;padding:40px 20px;text-align:center}"
        "h1{color:#ffb84d;letter-spacing:0.2em;font-size:18px}"
        "a{color:#ffb84d;font-size:16px;display:inline-block;"
        "margin-top:20px;padding:14px 22px;border:1px solid #ffb84d;"
        "text-decoration:none;letter-spacing:0.15em}"
        "p{color:#8b95a7;font-size:11px;letter-spacing:0.08em;"
        "text-transform:uppercase;margin:8px 0}"
        "</style></head>"
        "<body>"
        "<h1>▌ CDI//MGPRO</h1>"
        "<p>jaringan internal — tidak ada internet</p>"
        "<a href=\"http://192.168.4.1/\">▶ Buka Dashboard</a>"
        "</body></html>";
    AsyncWebServerResponse* r = req->beginResponse(200, "text/html", body);
    // Cache portal HTML SECARA SHORT — OS akan revalidate setelah 10s.
    // no-store ditinggalkan supaya OS detect content tidak berubah.
    r->addHeader("Cache-Control", "max-age=10");
    req->send(r);
    Serial.printf("[HTTP] captive probe %s -> portal HTML (200)\n",
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

    // OS captive-portal probes — single handler returns 200 + HTML
    // portal page. Cooperative behavior: Android dapat opsi "tetap
    // terhubung" setelah probe gagal beberapa kali; iOS/Mac open
    // captive browser SEKALI lalu network di-mark captive-aware.

    // Android (Google Chrome connectivity check):
    s_server.on("/generate_204",              HTTP_GET, portalProbe);
    s_server.on("/gen_204",                   HTTP_GET, portalProbe);

    // Apple (iOS, macOS, iPadOS):
    s_server.on("/hotspot-detect.html",       HTTP_GET, portalProbe);
    s_server.on("/library/test/success.html", HTTP_GET, portalProbe);

    // Windows NCSI:
    s_server.on("/ncsi.txt",                  HTTP_GET, portalProbe);
    s_server.on("/connecttest.txt",           HTTP_GET, portalProbe);

    // Firefox / Mozilla:
    s_server.on("/success.txt",               HTTP_GET, portalProbe);
    s_server.on("/canonical.html",            HTTP_GET, portalProbe);
    s_server.on("/redirect",                  HTTP_GET, portalProbe);

    // Unknown paths → serve embedded index (handleEmbedded fallback
    // to "/" kalau path tidak match). Won't trigger captive banner
    // karena probe routes di atas sudah match dulu.
    s_server.onNotFound(handleEmbedded);

    s_server.begin();
    Serial.println("[HTTP] server started (multi-page)");
}

AsyncWebServer& server() { return s_server; }

} // namespace cdi::net::http_server
