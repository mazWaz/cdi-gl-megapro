// SoftAP + DNS captive portal.
//
// `begin()` brings up the AP with SSID from config.h and starts a DNS
// server that resolves every query to the AP IP. The HTTP layer is
// responsible for serving index.html in response to the various OS
// captive-portal probe URLs.
//
// `poll()` must be called frequently from loop() to drive DNS reply
// pump (DNSServer is non-async).
#pragma once

#include <cstdint>

class IPAddress;

namespace cdi::net::wifi_ap {

void begin();
void poll();

// IP address of the AP (always 192.168.4.1).
const IPAddress& apIp();

} // namespace cdi::net::wifi_ap
