#include "net/wifi_ap.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <cstring>

#include "config.h"

namespace cdi::net::wifi_ap {
namespace {

const IPAddress kApIp(192, 168, 4, 1);
const IPAddress kApMask(255, 255, 255, 0);

DNSServer s_dns;

bool validPassword(const char* p) {
    if (!p) return false;
    const size_t n = strlen(p);
    if (n < 8 || n > 63) return false;
    for (size_t i = 0; i < n; i++) {
        const char c = p[i];
        if (c < 0x20 || c == 0x7F) return false;
    }
    return true;
}

} // anonymous

void begin() {
    const char* ssid = cdi::config::AP_SSID;
    const char* pwd  = cdi::config::AP_PASSWORD;

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(kApIp, kApIp, kApMask);

    if (!validPassword(pwd)) {
        Serial.printf("[AP] FATAL: CDI_AP_PASSWORD invalid (len=%u). "
                      "WPA2-PSK requires 8-63 printable ASCII chars. "
                      "Edit platformio.ini and rebuild. Falling back to OPEN AP.\n",
                      (unsigned)(pwd ? strlen(pwd) : 0));
        WiFi.softAP(ssid);
    } else {
        WiFi.softAP(ssid, pwd);
    }

    Serial.print("[AP] SSID=");
    Serial.print(ssid);
    Serial.print("  IP=");
    Serial.print(WiFi.softAPIP());
    if (validPassword(pwd)) {
        Serial.print("  password=");
        Serial.println(pwd);
    } else {
        Serial.println("  password=(open — invalid build flag)");
    }
    Serial.println("[AP] Connect with above credentials, then open http://192.168.4.1/");

    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(cdi::config::AP_DNS_PORT, "*", kApIp);
}

void poll() {
    s_dns.processNextRequest();
}

const IPAddress& apIp() { return kApIp; }
const char*      ssid()     { return cdi::config::AP_SSID; }
const char*      password() { return cdi::config::AP_PASSWORD; }

bool setPassword(const char* pwd) {
    // Compile-time credentials — runtime change not supported.
    (void)pwd;
    return false;
}

} // namespace cdi::net::wifi_ap
