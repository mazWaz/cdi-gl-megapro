// Idle rumble — bikin idle suara "lopey" / "drag-bike burble" via
// dua mekanisme yang dijalankan secara paralel:
//
//   1) Jitter retard: spark di-retard random per fire (±X°), tapi
//      tetap di range aman (mis. 1-5° max). Engine torque variasi
//      cycle-to-cycle → RPM modulasi ±20-50 → audible rumble.
//
//   2) Skip-fire pattern (mode AGGRESSIVE+): pattern fire-N-skip-1.
//      Mis. fire 7, skip 1 = irregular firing rhythm "BOM-BOM-..-BOM".
//
// Activation gated dengan:
//   * RPM dalam idle band (default 1000-2000)
//   * Sustained min sustain_ms (default 3 detik) — anti flicker
//   * Min uptime ESP since boot (default 60 detik) — warm-up window
//   * Auto-disable saat no-signal failsafe / ALVP DERATE / launch active
//
// Public API non-ISR-safe except shouldFireThisCycle() (called from
// spark ISR via safety::shouldFire chain).
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::idle_rumble {

void begin();

// Tick dipanggil dari main loop ~1 kHz. Param `rpm` = raw (instan)
// supaya respon cepat ke perubahan throttle.
void tick(cdi::rpm_t rpm);

// ── Config ──
void setEnabled(bool en);
bool isEnabled();

void setMode(cdi::IdleRumbleMode m);
cdi::IdleRumbleMode mode();

// Idle band (rpm) — di luar band, rumble auto-disengage.
void setRpmBand(cdi::rpm_t lo, cdi::rpm_t hi);
cdi::rpm_t rpmLo();
cdi::rpm_t rpmHi();

// Max retard depth (degrees). Actual per-fire = random 0..max.
void setMaxRetardDeg(float deg);
float maxRetardDeg();

// Skip pattern: fire `fire_n` consecutive, skip 1. n=0 disables skip.
void setSkipPattern(uint8_t fire_n);
uint8_t skipPattern();

// Sustain ms in idle band sebelum engage (default 3000 ms).
void setSustainMs(uint16_t ms);
uint16_t sustainMs();

// Min uptime sec sejak boot sebelum bisa engage (default 60 s).
void setMinUptimeSec(uint16_t s);
uint16_t minUptimeSec();

// ── Runtime ──
bool isActive();                      // sedang engage (gates UI display)
float currentRetardDeg();             // dipanggil live_stats per fire
bool  shouldFireThisCycle();          // dipanggil safety::shouldFire chain

} // namespace cdi::core::idle_rumble
