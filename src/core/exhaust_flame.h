// Exhaust flame — produce visible exhaust flame saat sustained
// di rev limiter. Mekanisme dua-tahap:
//
//   1) Skip pattern: drop sebagian spark di limiter → unburned
//      air-fuel mixture lewat exhaust valve → ignite di header panas.
//   2) Retard injection: cycle yang fire dapat heavy retard (5-20°)
//      → combustion bergeser ke exhaust stroke → flame keluar saat
//      valve buka.
//
// Dua intensity:
//   SAFE       — fire-fire-skip + retard 5-8°,  max 3s, cooldown 2s
//   AGGRESSIVE — fire-skip-skip + retard 12-20°, max 5s, cooldown 5s
//
// Activation gated:
//   * Master enable + mode (SAFE/AGGRESSIVE) explicit dari user
//   * Engine uptime ≥ 30s (warmup proxy untuk exhaust temp)
//   * Spark armed, tidak launch active, tidak ALVP derated
//   * RPM > main_limit selama engage_delay (1.0s SAFE / 1.5s AGGRESSIVE)
//   * RPM < overrev_limit - 200 (jangan ganggu overrev hard cut)
//
// Auto-disengage:
//   * RPM drop > 300 di bawah main_limit
//   * Duration > cap → cooldown
//   * Cooldown active
//
// Hooks (sama pattern dengan idle_rumble):
//   * shouldFireThisCycle()  → called dari safety::shouldFire chain (ISR)
//   * currentRetardDeg()     → called dari live_stats per CH1 cycle
//   * tick(rpm)              → dari main loop ~1 kHz
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::flame {

void begin();

// Tick dari main loop. Param rpm = raw (instan) untuk respon cepat.
void tick(cdi::rpm_t rpm);

// ── Config ──
void setEnabled(bool en);
bool isEnabled();

void setMode(cdi::FlameMode m);
cdi::FlameMode mode();

// ── Runtime state ──
bool isActive();            // sedang dump flame
bool isInCooldown();        // habis cap, lagi recovery
uint16_t activeElapsedMs(); // berapa ms sudah engage (untuk UI countdown)
uint16_t maxDurationMs();   // berapa ms cap untuk mode aktif

// ── Spark ISR chain (called dari safety::shouldFire) ──
bool shouldFireThisCycle();

// ── live_stats injection (called per CH1 fire) ──
float currentRetardDeg();

} // namespace cdi::core::flame
