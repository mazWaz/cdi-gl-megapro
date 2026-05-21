// SoftAP + DNS captive portal.
//
// `begin()` brings up the AP with SSID from config.h and starts a DNS
// server that resolves every query to the AP IP. The HTTP layer is
// responsible for serving index.html in response to the various OS
// captive-portal probe URLs.
//
// AP password is stored in NVS under namespace "cdiwifi" / key "pwd".
// On first boot (no stored value) a random 10-char alphanumeric
// password is generated, persisted to NVS, and printed to Serial so
// the user can read it over USB. Subsequent boots reuse the stored
// password. The user can change it via the WS protocol; the new
// value persists immediately but only takes effect on the next boot.
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

// Replace the stored password. Validates min length 8 (WPA2-PSK).
// Returns false on invalid input. New value takes effect after reboot.
bool setPassword(const char* pwd);

} // namespace cdi::net::wifi_ap
