// Scope-mode 2-channel ADC sampler.
//
// Owns a circular buffer fed by a hardware-timer ISR that calls
// `adc1_get_raw()` for both pulser channels. Producers: the ISR.
// Consumers: WS frame builder (`scope_frame`), snapshot save in storage.
//
// Lifetime:
//   begin(rate)   in setup() — claims hw_timer + attaches ISR.
//   setRate(hz)   safe to call any time; only updates alarm period.
//   setPaused(b)  ISR continues running but skips the buffer write.
//   end()         release timer (used when switching to IGNITION_MODE).
//
// Thread safety:
//   - ISR writes; consumers read via `snapshot()` which captures the
//     write index atomically. Pointers returned point at live volatile
//     storage — the data is racing with the ISR but the ring layout
//     guarantees the oldest BUF_SIZE samples are valid going forward
//     from `start_idx`.
//   - All public API is callable from loop() / WS callback context.
//     NOT callable from ISR.
#pragma once

#include <cstdint>

namespace cdi::scope {

struct Snapshot {
    uint32_t start_idx;         // ring write index at time of snapshot
    uint32_t buf_size;          // == cdi::config::BUF_SIZE_SCOPE
    const volatile uint16_t* ch1;
    const volatile uint16_t* ch2;
};

// Start sampling at `rate_hz`. Configures ADC1 channels, attaches ISR.
// Safe to call once; second call is a no-op (use setRate to change rate).
void begin(uint32_t rate_hz);

// Stop sampling and release the hardware timer. After this, the ISR
// no longer fires. Call again `begin()` to restart.
void end();

// Update sample period without re-attaching the ISR. Clamps to
// [SCOPE_RATE_MIN, SCOPE_RATE_MAX] from config.h. Returns the clamped
// rate that was actually applied.
uint32_t setRate(uint32_t hz);

// Current sample rate in Hz.
uint32_t getRate();

// Pause/resume buffer writes. The timer ISR keeps firing but skips
// the ADC read+store when paused — keeps timer stable for fast resume.
void setPaused(bool paused);
bool isPaused();

// Capture a coherent snapshot of the ring buffer state. The returned
// pointers are stable (point at static storage); only `start_idx`
// changes between calls. Iterate as:
//
//   auto s = cdi::scope::snapshot();
//   for (uint32_t i = 0; i < s.buf_size; i++) {
//       uint32_t idx = (s.start_idx + i) % s.buf_size;
//       uint16_t c1 = s.ch1[idx];
//       uint16_t c2 = s.ch2[idx];
//   }
Snapshot snapshot();

} // namespace cdi::scope
