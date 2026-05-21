#include "net/wifi_ap.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <cstring>

#include "config.h"

namespace cdi::net::wifi_ap {
namespace {

const IPAddress kApIp(192, 168, 4, 1);
const IPAddress kApMask(255, 255, 255, 0);

constexpr const char* NVS_NS       = "cdiwifi";
constexpr const char* KEY_SSID     = "ssid";
constexpr const char* KEY_PWD      = "pwd";
constexpr size_t      SSID_BUF     = 32;       // including NUL
constexpr size_t      PWD_BUF      = 64;

DNSServer s_dns;
char      s_ssid[SSID_BUF] = {0};
char      s_password[PWD_BUF] = {0};
bool      s_overrideSsid = false;
bool      s_overridePwd  = false;

bool validSsid(const char* s) {
    if (!s) return false;
    const size_t n = strlen(s);
    if (n < 1 || n >= SSID_BUF) return false;
    for (size_t i = 0; i < n; i++) {
        const char c = s[i];
        if (c < 0x20 || c == 0x7F) return false;
    }
    return true;
}

bool validPassword(const char* p) {
    if (!p) return false;
    const size_t n = strlen(p);
    if (n < 8 || n >= PWD_BUF) return false;
    for (size_t i = 0; i < n; i++) {
        const char c = p[i];
        if (c < 0x20 || c == 0x7F) return false;
    }
    return true;
}

void loadCredentials() {
    // Start with compile-time defaults.
    strncpy(s_ssid,     cdi::config::AP_SSID,     SSID_BUF - 1);
    strncpy(s_password, cdi::config::AP_PASSWORD, PWD_BUF  - 1);
    s_ssid[SSID_BUF - 1] = 0;
    s_password[PWD_BUF - 1] = 0;
    s_overrideSsid = false;
    s_overridePwd  = false;

    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/true)) {
        Serial.println("[AP] NVS open fail (read) — using compile-time defaults");
        return;
    }
    if (prefs.isKey(KEY_SSID)) {
        char buf[SSID_BUF] = {0};
        prefs.getString(KEY_SSID, buf, SSID_BUF);
        if (validSsid(buf)) {
            strncpy(s_ssid, buf, SSID_BUF - 1);
            s_ssid[SSID_BUF - 1] = 0;
            s_overrideSsid = true;
        }
    }
    if (prefs.isKey(KEY_PWD)) {
        char buf[PWD_BUF] = {0};
        prefs.getString(KEY_PWD, buf, PWD_BUF);
        if (validPassword(buf)) {
            strncpy(s_password, buf, PWD_BUF - 1);
            s_password[PWD_BUF - 1] = 0;
            s_overridePwd = true;
        }
    }
    prefs.end();
    Serial.printf("[AP] credentials loaded — ssid:%s pwd:%s\n",
                  s_overrideSsid ? "override" : "default",
                  s_overridePwd  ? "override" : "default");
}

} // anonymous

void begin() {
    loadCredentials();

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(kApIp, kApIp, kApMask);

    if (validPassword(s_password)) {
        WiFi.softAP(s_ssid, s_password);
    } else {
        Serial.printf("[AP] FATAL: password invalid; falling back to OPEN AP\n");
        WiFi.softAP(s_ssid);
    }

    Serial.print("[AP] SSID=");
    Serial.print(s_ssid);
    Serial.print("  IP=");
    Serial.print(WiFi.softAPIP());
    Serial.print("  password=");
    Serial.println(s_password);
    Serial.println("[AP] open http://192.168.4.1/ in Chrome");

    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(cdi::config::AP_DNS_PORT, "*", kApIp);
}

void poll() {
    s_dns.processNextRequest();
}

const IPAddress& apIp() { return kApIp; }
const char*      ssid()     { return s_ssid; }
const char*      password() { return s_password; }
const char*      source()   {
    return (s_overrideSsid || s_overridePwd) ? "override" : "default";
}

bool setSsid(const char* s) {
    if (!validSsid(s)) return false;
    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/false)) return false;
    prefs.putString(KEY_SSID, s);
    prefs.end();
    // Mirror to RAM for getter consistency; new AP value applies on next boot.
    strncpy(s_ssid, s, SSID_BUF - 1);
    s_ssid[SSID_BUF - 1] = 0;
    s_overrideSsid = true;
    Serial.printf("[AP] SSID override saved (effective after reboot)\n");
    return true;
}

bool setPassword(const char* pwd) {
    if (!validPassword(pwd)) return false;
    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/false)) return false;
    prefs.putString(KEY_PWD, pwd);
    prefs.end();
    strncpy(s_password, pwd, PWD_BUF - 1);
    s_password[PWD_BUF - 1] = 0;
    s_overridePwd = true;
    Serial.printf("[AP] password override saved (effective after reboot)\n");
    return true;
}

void resetToDefaults() {
    Preferences prefs;
    if (!prefs.begin(NVS_NS, /*readOnly=*/false)) return;
    prefs.remove(KEY_SSID);
    prefs.remove(KEY_PWD);
    prefs.end();
    // Mirror to RAM so getters reflect the defaults immediately.
    strncpy(s_ssid,     cdi::config::AP_SSID,     SSID_BUF - 1);
    strncpy(s_password, cdi::config::AP_PASSWORD, PWD_BUF  - 1);
    s_ssid[SSID_BUF - 1] = 0;
    s_password[PWD_BUF - 1] = 0;
    s_overrideSsid = false;
    s_overridePwd  = false;
    Serial.println("[AP] credentials reset to platformio.ini defaults (effective after reboot)");
}

} // namespace cdi::net::wifi_ap
