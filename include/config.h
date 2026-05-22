// Compile-time configuration for the Honda Megapro Programmable CDI firmware.
// All values must be `constexpr` — no instantiation, no runtime cost.
// Hardware pin assignments live in `pinmap.h`. Shared types live in `types.h`.
#pragma once

#include <cstdint>

namespace cdi::config {

// ---------- Firmware identity ----------
constexpr uint32_t FW_VERSION_MAJOR = 0;
constexpr uint32_t FW_VERSION_MINOR = 3;
constexpr uint32_t FW_VERSION_PATCH = 0;
constexpr uint32_t FW_VERSION       = (FW_VERSION_MAJOR << 16) |
                                      (FW_VERSION_MINOR << 8)  |
                                      (FW_VERSION_PATCH);

// ---------- Scope (edge-event stream only — no ADC) ----------
constexpr uint32_t SCOPE_FRAME_INTERVAL_MS = 30;  // ~33 fps edge frame broadcast

// ---------- Ignition-mode (real-time engine logic) ----------
constexpr uint32_t PULSER_RING_SIZE     = 64;     // SPSC ring, ISR→consumer
constexpr uint32_t TELEMETRY_RING_SIZE  = 32;
constexpr uint32_t MAX_ADVANCE_POINTS   = 32;     // per profile
constexpr uint32_t MAX_PROFILE_SLOTS    = 8;      // /profiles/profile_1..8.json

// Megapro pulser geometry (verified from manual_crank.csv analysis)
constexpr float    MAGNET_ANGULAR_WIDTH_DEG = 18.0f;
constexpr float    MAX_ADVANCE_FROM_CH1_DEG = 32.0f;
constexpr float    BASE_ADVANCE_FROM_CH2_DEG = 14.0f;

// Defaults — overridden by active profile after load
constexpr uint32_t DEFAULT_REV_LIMIT_MAIN_RPM    = 10500;
constexpr uint32_t DEFAULT_REV_LIMIT_OVERREV_RPM = 11500;
constexpr uint32_t DEFAULT_REV_LIMIT_LAUNCH_RPM  = 5000;
constexpr uint32_t DEFAULT_DWELL_US              = 2500;

// Sanity bounds — anything outside is rejected as noise/error.
// MIN must be low enough to accept kick / electric-start cranking
// (typical 50-300 rpm). Anything below 30 rpm is almost certainly
// a stuck pulser line.
constexpr uint32_t RPM_MIN_VALID = 30;
constexpr uint32_t RPM_MAX_VALID = 15000;
constexpr float    ADVANCE_MIN_DEG = 0.0f;
constexpr float    ADVANCE_MAX_DEG = 45.0f;

// ---------- Safety ----------
constexpr uint32_t NO_SIGNAL_TIMEOUT_MS  = 500;   // pulser silent → disarm
constexpr uint32_t SAFETY_TICK_INTERVAL_MS = 100;
// Tightened from 5 s to 2 s: if firmware crashes mid-dwell (GPIO
// stuck HIGH = primary coil energized continuously), 5 s of full
// primary current would noticeably heat the coil. 2 s is still long
// enough for legitimate loop work (heavy NVS / LittleFS ops run on
// core 0 anyway), short enough that coil thermal damage is minimal.
constexpr uint32_t TASK_WDT_TIMEOUT_S    = 2;

// Absolute RPM ceiling — above this we assume something is broken
// (multi-tooth pickup mistakenly connected, electrical noise on
// CH1, math bug, etc) and disarm instantly. 16000 RPM > redline of
// every single-cylinder bike in the preset library, but still well
// inside what rpm_calc can sanely report (RPM_MAX_VALID 15000).
constexpr uint32_t ABSOLUTE_RPM_CEILING = 16000;
constexpr float    ALVP_DERATE_BELOW_V   = 10.5f;
constexpr float    ALVP_DISARM_BELOW_V   = 9.0f;
constexpr uint32_t ALVP_LOW_DURATION_MS  = 2000;  // hysteresis

// ---------- Network ----------
// SSID + password come from build flags in platformio.ini (-D CDI_AP_SSID,
// -D CDI_AP_PASSWORD). Fallback defaults below if the flags aren't set.
#ifndef CDI_AP_SSID
#define CDI_AP_SSID "CDI-Megapro"
#endif
#ifndef CDI_AP_PASSWORD
#define CDI_AP_PASSWORD "ganti-password"
#endif
constexpr const char* AP_SSID     = CDI_AP_SSID;
constexpr const char* AP_PASSWORD = CDI_AP_PASSWORD;
constexpr uint16_t    AP_DNS_PORT  = 53;
constexpr uint16_t    AP_HTTP_PORT = 80;

// ---------- WebSocket protocol ----------
constexpr uint8_t WS_MAGIC_SCOPE_EDGE = 0xA7;   // edge-event stream
constexpr uint8_t WS_MAGIC_TELEMETRY  = 0xB0;
constexpr uint8_t WS_MAGIC_FIRE_EVENT = 0xB1;
constexpr uint8_t WS_QUEUE_BACKPRESSURE_LIMIT = 2;

// Edge-stream frame: capacity sized for headroom
// (~24 edges/frame at 10000 RPM single-cyl 4T).
constexpr uint32_t SCOPE_EDGE_FRAME_INTERVAL_MS = 30;
constexpr uint32_t SCOPE_EDGE_FRAME_MAX_EVENTS  = 64;

// ---------- Datalog ----------
constexpr uint32_t DATALOG_SAMPLE_HZ      = 50;
constexpr uint32_t DATALOG_DURATION_S     = 60;
constexpr uint32_t DATALOG_RING_ENTRIES   = DATALOG_SAMPLE_HZ * DATALOG_DURATION_S;
constexpr uint32_t DATALOG_BYTES_PER_ENTRY = 24;

} // namespace cdi::config
