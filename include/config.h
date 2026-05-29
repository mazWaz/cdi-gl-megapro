// Compile-time configuration for the Honda Megapro Programmable CDI firmware.
// All values must be `constexpr` — no instantiation, no runtime cost.
// Hardware pin assignments live in `pinmap.h`. Shared types live in `types.h`.
#pragma once

#include <cstdint>

namespace cdi::config {

// ---------- Firmware identity ----------
constexpr uint32_t FW_VERSION_MAJOR = 0;
constexpr uint32_t FW_VERSION_MINOR = 5;
constexpr uint32_t FW_VERSION_PATCH = 5;
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
// 5 ms gives ~70% primary saturation on typical motorcycle coil
// (3-5 mH, 0.5-1 Ω; τ = L/R ≈ 5 ms). Spark energy strong enough
// for cold cranking. Auto-shortened by live_stats at high RPM to
// preserve target advance (see live_stats::tick).
constexpr uint32_t DEFAULT_DWELL_US              = 5000;

// Sanity bounds — anything outside is rejected as noise/error.
// MIN must be low enough to accept kick / electric-start cranking
// (typical 50-300 rpm). Anything below 30 rpm is almost certainly
// a stuck pulser line.
// MAX_VALID was 15000 but live log on real engine showed EMI bursts
// from coil firing producing spurious CH1 events at 4000-4500 µs
// periods (= 13-15k phantom RPM) that lay above any realistic Honda
// Megapro / GL Pro / commuter-4T physical ceiling (stock redline
// 10500, modified ~13000). 13000 rpm = 4615 µs period gives:
//   - clean reject of all observed EMI bursts (>= 4000 µs threshold)
//   - margin above OEM overrev cut at 11500 rpm (= 5217 µs)
//   - 13 % headroom for the most aggressive aftermarket builds
constexpr uint32_t RPM_MIN_VALID = 30;
constexpr uint32_t RPM_MAX_VALID = 13000;
constexpr float    ADVANCE_MIN_DEG = 0.0f;
constexpr float    ADVANCE_MAX_DEG = 45.0f;

// Crank-assist (opt-in, default OFF): below this RPM the spark scheduler
// fires off the CH2 trailing edge (fixed mechanical base advance, period-
// independent) instead of the CH1+delay path whose drift gate rejects
// most cranking sparks. Threshold sits BELOW idle (cranking is ~200-600
// rpm, idle ≥1000) so idle/running keep the CH1 advance-map path — earlier
// 1500 overlapped the idle band, hijacking idle timing & idle-rumble retard.
constexpr uint32_t CRANK_MODE_RPM = 700;

// ---------- Safety ----------
constexpr uint32_t NO_SIGNAL_TIMEOUT_MS  = 500;   // pulser silent → disarm
constexpr uint32_t SAFETY_TICK_INTERVAL_MS = 100;
// Loop-task watchdog. Set to 5 s (was briefly 2 s).
//
// The 2 s value rebooted the board whenever the loop blocked on the
// WiFi/WebSocket stack for >2 s — which happens under marginal power
// (bench USB) or a stalled WiFi client, where AsyncTCP/lwip back-
// pressures a synchronous send. That produced a boot loop on the bench
// even though the engine was perfectly fine.
//
// Coil-thermal safety does NOT depend on this short timeout: the spark
// fire-OFF is driven by a dedicated hardware-timer ISR (spark_scheduler),
// which de-energizes the coil independently of the loop. A stalled loop
// therefore cannot strand the primary HIGH. 5 s still catches a genuine
// firmware hang while tolerating transient network stalls.
//
// CAVEAT — flash writes: the fire-off timer ISR is NOT IRAM-allocated,
// and a flash erase/program (NVS persist, LittleFS snapshot, OTA) stalls
// the spark core entirely for the duration of each erase. If one lands
// mid-dwell it CAN strand the primary HIGH for up to a sector-erase
// (~tens of ms), independent of this WDT. Firmware mitigates by deferring
// all flash writes while the engine is firing (safety::flashWriteSafe);
// the ultimate backstop is a hardware max-dwell / current-limit on the
// coil driver — see README wiring notes.
constexpr uint32_t TASK_WDT_TIMEOUT_S    = 5;

// (ABSOLUTE_RPM_CEILING removed — was a dead/redundant guard: it equalled
// RPM_MAX_VALID so rpm_calc's clamp meant it never fired, and over-rev is
// already handled by the self-recovering overrev cut while a broken/
// multi-tooth pickup is rejected as noise by rpm_calc → no fire. See
// safety::tick.)
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
