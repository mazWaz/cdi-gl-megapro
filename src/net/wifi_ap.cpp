#include "net/wifi_ap.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>

#include "config.h"

namespace cdi::net::wifi_ap {
namespace {

const IPAddress kApIp(192, 168, 4, 1);
const IPAddress kApMask(255, 255, 255, 0);

DNSServer s_dns;

} // anonymous

void begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(kApIp, kApIp, kApMask);
    if (cdi::config::AP_PASSWORD) {
        WiFi.softAP(cdi::config::AP_SSID, cdi::config::AP_PASSWORD);
    } else {
        WiFi.softAP(cdi::config::AP_SSID);
    }

    Serial.print("[AP] SSID=");
    Serial.print(cdi::config::AP_SSID);
    Serial.print("  IP=");
    Serial.println(WiFi.softAPIP());

    s_dns.setErrorReplyCode(DNSReplyCode::NoError);
    s_dns.start(cdi::config::AP_DNS_PORT, "*", kApIp);
}

void poll() {
    s_dns.processNextRequest();
}

const IPAddress& apIp() { return kApIp; }

} // namespace cdi::net::wifi_ap
