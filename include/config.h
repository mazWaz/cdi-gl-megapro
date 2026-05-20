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

// ---------- Scope-mode (ADC sampling) ----------
constexpr uint32_t BUF_SIZE_SCOPE       = 2048;   // samples per channel
constexpr uint32_t SCOPE_RATE_DEFAULT   = 5000;   // Hz
constexpr uint32_t SCOPE_RATE_MIN       = 500;
constexpr uint32_t SCOPE_RATE_MAX       = 10000;  // higher rates starve ISR
constexpr uint32_t SCOPE_FRAME_INTERVAL_MS = 50;  // 20 fps WS broadcast

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

// Sanity bounds — anything outside is rejected as noise/error
constexpr uint32_t RPM_MIN_VALID = 100;
constexpr uint32_t RPM_MAX_VALID = 15000;
constexpr float    ADVANCE_MIN_DEG = 0.0f;
constexpr float    ADVANCE_MAX_DEG = 45.0f;

// ---------- Safety ----------
constexpr uint32_t NO_SIGNAL_TIMEOUT_MS  = 500;   // pulser silent → disarm
constexpr uint32_t SAFETY_TICK_INTERVAL_MS = 100;
constexpr uint32_t TASK_WDT_TIMEOUT_S    = 5;
constexpr float    ALVP_DERATE_BELOW_V   = 10.5f;
constexpr float    ALVP_DISARM_BELOW_V   = 9.0f;
constexpr uint32_t ALVP_LOW_DURATION_MS  = 2000;  // hysteresis

// ---------- Network ----------
constexpr const char* AP_SSID    = "CDI-Megapro";
constexpr const char* AP_PASSWORD = nullptr;       // open
constexpr uint16_t    AP_DNS_PORT = 53;
constexpr uint16_t    AP_HTTP_PORT = 80;

// ---------- WebSocket protocol ----------
constexpr uint8_t WS_MAGIC_SCOPE_LIVE = 0xA5;   // raw ADC samples (legacy)
constexpr uint8_t WS_MAGIC_SCOPE_SNAP = 0xA6;   // saved raw ADC snapshot
constexpr uint8_t WS_MAGIC_SCOPE_EDGE = 0xA7;   // edge-event stream
constexpr uint8_t WS_MAGIC_TELEMETRY  = 0xB0;
constexpr uint8_t WS_MAGIC_FIRE_EVENT = 0xB1;
constexpr uint8_t WS_QUEUE_BACKPRESSURE_LIMIT = 2;

// Edge-stream frame: 30 ms cadence, capacity sized for headroom
// (~24 edges/frame at 10000 RPM single-cyl 4T).
constexpr uint32_t SCOPE_EDGE_FRAME_INTERVAL_MS = 30;
constexpr uint32_t SCOPE_EDGE_FRAME_MAX_EVENTS  = 64;

// ---------- Datalog ----------
constexpr uint32_t DATALOG_SAMPLE_HZ      = 50;
constexpr uint32_t DATALOG_DURATION_S     = 60;
constexpr uint32_t DATALOG_RING_ENTRIES   = DATALOG_SAMPLE_HZ * DATALOG_DURATION_S;
constexpr uint32_t DATALOG_BYTES_PER_ENTRY = 24;

} // namespace cdi::config
