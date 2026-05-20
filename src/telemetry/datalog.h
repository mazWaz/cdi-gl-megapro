// In-RAM ring datalog — snapshots live telemetry at 50 Hz for
// post-test analysis (jitter histogram, RPM trace, advance° curve).
//
// Capacity: 1500 entries × 16 B = 24 KB BSS. At 50 Hz that's 30
// seconds of recording before the oldest entry is overwritten.
//
// State machine:
//   IDLE        — not recording, ring empty (or last session)
//   RECORDING   — tick() pushes a sample every 20 ms; oldest sample
//                 is overwritten when ring is full (sliding window).
//   STOPPED     — recording halted; ring retains last N seconds for
//                 CSV download.
//
// CSV columns: ts_ms,rpm,advance_deg,jitter_us,vbat_mv,armed,
//              cut_mode,safety_flags,backfire_active,alvp_state.
#pragma once

#include <cstdint>

class String;

namespace cdi::telemetry::datalog {

void begin();
void tick();    // call from loop — 20ms sampling rate internally

void start();
void stop();

bool        isRecording();
uint32_t    entryCount();   // current valid entries (0 if just started)
uint32_t    capacity();     // ring slots
uint32_t    durationMs();   // span covered by current ring contents

// Build CSV from current ring contents. Returns false if empty.
bool fillCsv(String& out);

void clear();

} // namespace cdi::telemetry::datalog
