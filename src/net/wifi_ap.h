// SoftAP + DNS captive portal.
//
// `begin()` brings up the AP with SSID from config.h and starts a DNS
// server that resolves every query to the AP IP. The HTTP layer is
// responsible for serving index.html in response to the various OS
// captive-portal probe URLs.
//
// AP credentials (SSID + password) are compile-time constants from
// platformio.ini build flags (-D CDI_AP_SSID, -D CDI_AP_PASSWORD).
// To rotate the password: edit platformio.ini, `pio run -t upload`.
// setPassword() is kept for API stability but always returns false.
//
// `poll()` must be called frequently from loop() to drive DNS reply
// pump (DNSServer is non-async).
#pragma once

#include <cstdint>
#include <Arduino.h>

class IPAddress;

namespace cdi::net::wifi_ap {

void begin();
void poll();

// IP address of the AP (always 192.168.4.1).
const IPAddress& apIp();

// Active SSID (compile-time constant for now).
const char* ssid();

// Current AP password (loaded from NVS on begin). Lifetime: program.
const char* password();

// Always returns false — password is compile-time. Kept so WS code
// can call it uniformly without conditional logic.
bool setPassword(const char* pwd);

} // namespace cdi::net::wifi_ap
