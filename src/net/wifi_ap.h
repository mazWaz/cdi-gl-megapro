// SoftAP + DNS captive portal.
//
// `begin()` brings up the AP and starts a DNS server that resolves
// every query to the AP IP. The HTTP layer responds to OS captive-
// portal probe URLs with the exact expected payload so the banner
// doesn't spam.
//
// Credentials layering:
//   1. Build flags  CDI_AP_SSID / CDI_AP_PASSWORD in platformio.ini
//      act as the FACTORY DEFAULT — used when no NVS override is
//      stored.
//   2. NVS namespace "cdiwifi" keys "ssid" / "pwd" carry an
//      optional runtime override — set via the UI (setSsid /
//      setPassword) and persisted across reboots.
//
// To recover from a forgotten password / wrong SSID the user can
// erase the NVS partition over USB (`pio run -t erase`) — that
// drops back to the compile-time defaults.
//
// `poll()` must be called frequently from loop() to drive the DNS
// reply pump.
#pragma once

#include <cstdint>
#include <Arduino.h>

class IPAddress;

namespace cdi::net::wifi_ap {

void begin();
void poll();

const IPAddress& apIp();

// Active SSID — runtime override if present, else compile-time default.
const char* ssid();
// Active password — same fallback chain. Lifetime: program.
const char* password();

// Source of currently active values: "default" | "override".
const char* source();

// Replace the stored SSID. Validates 1-31 printable ASCII chars.
// Returns false on invalid input. Takes effect after reboot.
bool setSsid(const char* s);

// Replace the stored password. Validates 8-63 printable ASCII chars
// (WPA2-PSK rules). Takes effect after reboot.
bool setPassword(const char* pwd);

// Drop all NVS overrides and fall back to platformio.ini defaults
// on next boot.
void resetToDefaults();

} // namespace cdi::net::wifi_ap
