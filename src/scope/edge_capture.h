// Edge-event capture for live scope visualization.
//
// Drains the pulser scope ring (filled by `core::pulser` ISR on every
// CH1/CH2 edge) and emits binary WS frames (opcode 0xA7) at a fixed
// cadence. UI reconstructs a step-function waveform from the timestamp
// + level events — far cheaper than ADC sampling, and accurate to ~1 µs.
//
// Coexists with ignition: same ISR feeds both ignition logic and scope
// stream via two independent SPSC rings, so a slow UI consumer can
// never back-pressure spark scheduling.
//
// Frame layout (little-endian):
//   [0]      magic 0xA7
//   [1..4]   seq counter      (uint32)
//   [5..8]   first ts_us      (uint32, low 32 of micros base)
//   [9..10]  n (event count)  (uint16)
//   [11..]   n × {ts_offset_us:uint16, ch:uint8, level:uint8}
//
// `ts_offset_us` is delta from frame's first ts_us — keeps frame size
// small even with 1 µs resolution. UI reconstructs absolute time by
// adding offset to first ts_us.
#pragma once

#include <cstdint>
#include <cstddef>

namespace cdi::scope::edge {

void begin();

// Drain pending scope events and build one binary frame. Returns
// pointer to internal static buffer + size, or nullptr if no events
// since last call (no frame to send).
const uint8_t* buildFrame(size_t& out_len);

// Called by WS server when ready to broadcast. Internal cadence is
// gated by `SCOPE_EDGE_FRAME_INTERVAL_MS`.
bool dueNow(uint32_t now_ms);

uint32_t framesEmitted();
uint32_t lastFrameEventCount();

} // namespace cdi::scope::edge
