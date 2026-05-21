#include "net/wifi_ap.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_random.h>
#include <cstring>

#include "config.h"

namespace cdi::net::wifi_ap {
namespace {

const IPAddress kApIp(192, 168, 4, 1);
const IPAddress kApMask(255, 255, 255, 0);

constexpr const char* PWD_NS  = "cdiwifi";
constexpr const char* PWD_KEY = "pwd";
constexpr size_t      PWD_LEN = 10;       // WPA2-PSK requires ≥ 8
constexpr size_t      PWD_BUF = 32;       // including NUL

DNSServer s_dns;
char      s_password[PWD_BUF] = {0};

void generateRandomPassword(char* out, size_t len) {
    // URL-safe alphanumeric — easy to type on a phone keyboard.
    // 32 chars × 8 positions = 32^10 ≈ 10^15 → not brute-forceable
    // over WiFi RF range (NIST recommends ≥ 8 chars).
    static const char kAlphabet[] =
        "abcdefghijkmnopqrstuvwxyz"   // skip 'l' (looks like 1)
        "ABCDEFGHJKLMNPQRSTUVWXYZ"    // skip 'I', 'O'
        "23456789";                   // skip '0', '1'
    const uint32_t alphaLen = sizeof(kAlphabet) - 1;
    for (size_t i = 0; i < len; i++) {
        out[i] = kAlphabet[esp_random() % alphaLen];
    }
    out[len] = 0;
}

void loadOrCreatePassword() {
    Preferences prefs;
    if (!prefs.begin(PWD_NS, /*readOnly=*/false)) {
        Serial.println("[AP] NVS open fail; using ephemeral random password");
        generateRandomPassword(s_password, PWD_LEN);
        return;
    }
    if (prefs.isKey(PWD_KEY)) {
        prefs.getString(PWD_KEY, s_password, PWD_BUF);
        prefs.end();
        if (strlen(s_password) >= 8) {
            Serial.println("[AP] password loaded from NVS");
            return;
        }
        // Stored value invalid — regenerate.
        Serial.println("[AP] stored password invalid; regenerating");
    } else {
        Serial.println("[AP] first boot — generating WPA2 password");
    }
    generateRandomPassword(s_password, PWD_LEN);
    prefs.putString(PWD_KEY, s_password);
    prefs.end();
}

} // anonymous

void begin() {
    loadOrCreatePassword();

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(kApIp, kApIp, kApMask);
    WiFi.softAP(cdi::config::AP_SSID, s_password);

    Serial.print("[AP] SSID=");
    Serial.print(cdi::config::AP_SSID);
    Serial.print("  IP=");
    Serial.print(WiFi.softAPIP());
    Serial.print("  password=");
    Serial.println(s_password);
    Serial.println("[AP] Connect with above credentials, then open http://192.168.4.1/");

    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(cdi::config::AP_DNS_PORT, "*", kApIp);
}

void poll() {
    s_dns.processNextRequest();
}

const IPAddress& apIp() { return kApIp; }
const char*      ssid()     { return cdi::config::AP_SSID; }
const char*      password() { return s_password; }

bool setPassword(const char* pwd) {
    if (!pwd) return false;
    const size_t n = strlen(pwd);
    if (n < 8 || n >= PWD_BUF) return false;   // WPA2 min, buffer max
    // Reject control characters / spaces (some phones strip them).
    for (size_t i = 0; i < n; i++) {
        const char c = pwd[i];
        if (c < 0x20 || c == 0x7F) return false;
    }
    Preferences prefs;
    if (!prefs.begin(PWD_NS, /*readOnly=*/false)) return false;
    prefs.putString(PWD_KEY, pwd);
    prefs.end();
    // Update RAM copy too; new AP value applies after next reboot.
    strncpy(s_password, pwd, PWD_BUF - 1);
    s_password[PWD_BUF - 1] = 0;
    Serial.printf("[AP] password updated (effective after reboot)\n");
    return true;
}

} // namespace cdi::net::wifi_ap
