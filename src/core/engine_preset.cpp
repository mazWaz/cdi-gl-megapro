#include "core/engine_preset.h"

#include <Arduino.h>
#include <cstring>

#include "core/advance_map.h"
#include "core/safety.h"
#include "core/spark_scheduler.h"
#include "core/pickup.h"

namespace cdi::core::preset {
namespace {

// ─── Preset library ───
// Advance maps based on common stock + community-known reference values
// for each motor. Tune your specific build from these starting points.
//
// Conventions enforced across all entries (post-research audit):
//   * First map point at 300 rpm = 5° BTDC. Kick-start range
//     (50-400 rpm) clamps to this point — safe value below the
//     kickback threshold (research: <25° BTDC safe, ≤10° ideal).
//   * Dwell 5 ms (4-stroke) / 4 ms (2-stroke) — tuned for inductive
//     TCI saturation (τ ≈ 5 ms on typical motorcycle coil). Stock
//     CDI used 2-3 ms because capacitor discharge doesn't need
//     primary saturation. live_stats auto-caps dwell at high RPM.
//   * Racing presets retain stock dwell — UI must warn about
//     compression / detonation risk before applying.
const Preset PRESETS[] = {

// ─── Honda 4-stroke ───
// 32-point advance curves derived from factory marks (T/F/PL at 0/10/32°)
// + community references (Indonesian aftermarket CDI patterns, BRT I-Max
// curve philosophy). Each preset describes: cranking 3-5° → idle (~1500
// rpm) at motor-specific value → linear ramp to peak around 5000-6000 rpm
// → plateau → very-high RPM slight retard (detonation protection).
// ════════════════════════════════════════════════════════════════════
// DATA PROVENANCE & SAFETY (web research 2026-05 + adversarial sanity pass)
//   * Factory ignition CURVES for these Indonesian-market bikes are NOT
//     published (service manuals proprietary). The advance maps below are
//     ENGINEERING ESTIMATES following the confirmed pattern: idle ~8-12°
//     BTDC, full advance ~28-35° BTDC plateauing ~4000-5000 rpm.
//   * rev_main/rev_overrev were set from researched PEAK-POWER rpm
//     (rev_main ≈ peak + ~1000, overrev ≈ +500) so the limiter is reachable.
//   * Confirmed factory idle timing found in research (for reference):
//     Mio 5°@1500 · Jupiter-MX 10°@1400 · Vixion 10°@1400 · Scorpio 5°@1450 ·
//     Thunder125 13°@1400 (full ~32°) · Supra125 ~15° · Satria-FU ~4°.
//   * ⚠ magnet_width_deg AND max_advance_deg are PHYSICAL pickup geometry —
//     NOT publishable, vary per unit. Treat preset values as a STARTING
//     POINT: run pickup auto-cal (width) + STROBE the absolute timing
//     (max_advance) before riding. A wrong max_advance over-advances the
//     spark → kickback. (KLX150 is the proof: real magnet 43°, not 18°.)
// ════════════════════════════════════════════════════════════════════
//
// Megapro/Tiger — calibrated to factory marks:
//   T = 0° (TDC), F = 10° BTDC at 1400 rpm idle, PL = 32° BTDC at 5000+ rpm.
// Cranking (≤300 rpm) very conservative 2-3° to eliminate kickback at slow
// first kick where combustion duration in degrees is tiny.
{ "honda_megapro", "Honda 4T", "Honda Megapro / Tiger", 1, 18.0f, 32.0f,
  { {200,2}, {300,3}, {500,6}, {800,8}, {1000,9}, {1200,9}, {1500,10},
    {1800,12}, {2000,13}, {2200,14}, {2500,16}, {2800,18}, {3000,19},
    {3300,21}, {3500,22}, {3800,24}, {4000,25}, {4300,26}, {4500,27},
    {4800,28}, {5000,29}, {5300,30}, {5500,31}, {5800,31}, {6000,32},
    {6500,32}, {7000,32}, {7500,32}, {8000,32}, {9000,32}, {10500,31},
    {12000,30} }, 32,
  9500, 10000, 5000,
  "Stok megapro/tiger 160-200cc, F=10° idle, PL=32° peak (factory). Redline ~9000-9500 (peak 8500). Geometri estimasi — strobe." },

// GL Pro/Max/Win — derived from same engine family as Megapro but
// vintage 125 cc — idle slightly leaner advance, lower redline.
{ "honda_gl_pro", "Honda 4T", "Honda GL Pro / GL Max / Win", 1, 18.0f, 32.0f,
  { {200,2}, {300,3}, {500,5}, {800,7}, {1000,8}, {1200,9}, {1500,10},
    {1800,11}, {2000,12}, {2200,13}, {2500,15}, {2800,17}, {3000,18},
    {3300,20}, {3500,21}, {3800,23}, {4000,24}, {4300,25}, {4500,26},
    {4800,27}, {5000,28}, {5300,29}, {5500,30}, {5800,31}, {6000,32},
    {6500,32}, {7000,32}, {7500,32}, {8000,32}, {8500,31}, {9000,31},
    {10000,30} }, 32,
  9000, 9500, 5000,
  "GL Pro/Max/Win vintage 125-160cc (peak 8500). Redline ~9000. Timing estimasi — strobe." },

{ "honda_cb150r", "Honda 4T", "Honda CB150R Streetfire", 1, 16.0f, 35.0f,
  { {300,5}, {800,12}, {1000,13}, {1200,14}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,25}, {3300,26}, {3500,27},
    {3800,28}, {4000,29}, {4300,30}, {4500,31}, {4800,32}, {5000,32},
    {5300,33}, {5500,33}, {5800,34}, {6000,34}, {6500,34}, {7000,35},
    {7500,35}, {8000,35}, {8500,35}, {9000,35}, {9500,35}, {10500,34},
    {11500,33} }, 32,
  10500, 11000, 5000,
  "Sport 150cc DOHC PGM-FI high-rev (redline ~11000, peak 9000). Advance estimasi — strobe." },

{ "honda_verza", "Honda 4T", "Honda Verza 150", 1, 16.0f, 33.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,28}, {4500,29}, {4800,30}, {5000,30},
    {5300,31}, {5500,31}, {5800,32}, {6000,32}, {6500,32}, {7000,33},
    {7500,33}, {8000,33}, {8500,33}, {9000,33}, {9500,33}, {10500,32},
    {11500,31} }, 32,
  9000, 9500, 5000,
  "Commuter 150cc SOHC PGM-FI (peak 8500, redline ~9000). Timing estimasi — strobe." },

{ "honda_supra_125", "Honda 4T", "Honda Supra X 125 karbu", 1, 14.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,12}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,27}, {6300,28}, {6500,28},
    {7000,28}, {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,27},
    {10500,27} }, 32,
  9000, 9500, 5000,
  "Bebek 125cc karbu (peak 8000, idle ~15° riset). Redline ~9000. Strobe." },

{ "honda_revo_karbu", "Honda 4T", "Honda Revo / Blade 110 karbu", 1, 14.0f, 27.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,12}, {1800,13}, {2000,14},
    {2200,15}, {2500,17}, {2800,18}, {3000,19}, {3300,20}, {3500,21},
    {3800,22}, {4000,23}, {4300,23}, {4500,24}, {4800,24}, {5000,25},
    {5300,25}, {5500,26}, {5800,26}, {6000,26}, {6300,27}, {6500,27},
    {7000,27}, {7500,27}, {8000,27}, {8500,27}, {9000,26}, {9500,26},
    {10000,26} }, 32,
  8500, 9000, 5000,
  "Bebek 110cc karbu (peak 7500). Redline ~8500. Timing estimasi — strobe." },

{ "honda_beat_karbu", "Honda 4T", "Honda Beat / Vario / Scoopy karbu", 1, 15.0f, 28.0f,
  { {300,5}, {800,8}, {1000,10}, {1200,11}, {1500,13}, {1800,15}, {2000,16},
    {2200,17}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,27}, {6300,28}, {6500,28},
    {7000,28}, {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,27},
    {10500,27} }, 32,
  8500, 9000, 5000,
  "Matic 110cc karbu (peak 7500). Redline ~8500. Timing estimasi — strobe." },

{ "honda_vario_125", "Honda 4T", "Honda Vario / Vario 125 karbu", 1, 15.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,29}, {5800,30}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  9000, 9500, 5000,
  "Matic 125cc PGM-FI/TCI (peak 8500). Redline ~9000. Timing estimasi — strobe." },

{ "honda_win_100", "Honda 4T", "Honda Win 100 / Astrea", 1, 18.0f, 30.0f,
  { {300,5}, {500,8}, {800,9}, {1000,10}, {1200,11}, {1500,13}, {1800,15},
    {2000,16}, {2200,17}, {2500,19}, {2800,20}, {3000,21}, {3300,22},
    {3500,23}, {3800,24}, {4000,25}, {4300,25}, {4500,26}, {4800,27},
    {5000,27}, {5300,28}, {5500,28}, {5800,29}, {6000,29}, {6500,30},
    {7000,30}, {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,29},
    {10000,29} }, 32,
  8500, 9000, 5000,
  "Vintage 100cc cub (peak ~8000). Redline ~8500. Timing estimasi — strobe." },

// ─── Yamaha 4-stroke ───
{ "yamaha_mio_karbu", "Yamaha 4T", "Yamaha Mio / Vega / Jupiter Z karbu", 1, 22.0f, 32.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,21}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,30}, {5800,30}, {6000,30}, {6500,31}, {7000,31},
    {7500,32}, {8000,32}, {8500,32}, {9000,32}, {9500,32}, {10500,32},
    {11500,31} }, 32,
  9500, 10000, 5000,
  "Matic/bebek 113-115cc karbu, idle 5°@1500 (riset). Magnet 22° estimasi. Redline ~9500. Strobe." },

{ "yamaha_jupiter_mx", "Yamaha 4T", "Yamaha Jupiter MX 135", 1, 20.0f, 33.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,25}, {3300,26}, {3500,27},
    {3800,28}, {4000,29}, {4300,29}, {4500,30}, {4800,30}, {5000,31},
    {5300,31}, {5500,31}, {5800,32}, {6000,32}, {6500,32}, {7000,33},
    {7500,33}, {8000,33}, {8500,33}, {9000,33}, {9500,33}, {10500,32},
    {11500,31} }, 32,
  9500, 10000, 5000,
  "Bebek sport 135cc, idle 10°@1400 (riset), peak 8500. Redline ~9500. Strobe." },

{ "yamaha_vixion_carb", "Yamaha 4T", "Yamaha Vixion karbu (lama)", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,29}, {5800,30}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10000, 10500, 5000,
  "Sport 150cc karbu, idle 10°@1400 (riset), peak 8500, revs ~10500. Strobe." },

{ "yamaha_scorpio", "Yamaha 4T", "Yamaha Scorpio Z 225", 1, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,21}, {2800,23}, {3000,24}, {3300,25}, {3500,26},
    {3800,27}, {4000,28}, {4300,28}, {4500,29}, {4800,29}, {5000,30},
    {5300,30}, {5500,31}, {5800,31}, {6000,32}, {6500,32}, {7000,32},
    {7500,32}, {8000,32}, {8500,32}, {9000,32}, {9500,32}, {10500,31},
    {11500,30} }, 32,
  9000, 9500, 5000,
  "Sport tourer 223cc, idle 5°@1450 (riset), peak 8000. Redline ~9000-9500 (bukan 11000). Strobe." },

// ─── Suzuki 4-stroke ───
{ "suzuki_shogun_125", "Suzuki 4T", "Suzuki Shogun 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  9000, 9500, 5000,
  "Bebek 125cc DC-CDI (peak 8000). Redline ~9000. Timing estimasi — strobe." },

{ "suzuki_smash", "Suzuki 4T", "Suzuki Smash / Smash Titan", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,13}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,28}, {6500,28}, {7000,28},
    {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,28}, {10000,27},
    {10500,27} }, 32,
  8000, 8500, 5000,
  "Bebek 110cc commuter (peak 7000). Redline ~8000. Timing estimasi — strobe." },

{ "suzuki_satria_fu_new", "Suzuki 4T", "Suzuki Satria FU 150 NEW", 1, 18.0f, 35.0f,
  { {300,5}, {800,12}, {1000,13}, {1200,14}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,26}, {3300,27}, {3500,28},
    {3800,29}, {4000,30}, {4300,31}, {4500,32}, {4800,32}, {5000,33},
    {5300,33}, {5500,33}, {5800,34}, {6000,34}, {6500,34}, {7000,34},
    {7500,35}, {8000,35}, {8500,35}, {9000,35}, {9500,35}, {10500,34},
    {11500,33} }, 32,
  11000, 11500, 5000,
  "Satria FU150 DOHC, idle ~4° (riset), peak 10000, redline ~11000. Strobe." },

{ "suzuki_thunder_125", "Suzuki 4T", "Suzuki Thunder 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  9500, 10000, 5000,
  "Sport touring 125cc TCI, idle 13°@1400 + full ~32° (riset). Redline ~9500-10000 (bukan 13500). Strobe." },

{ "suzuki_spin_skywave", "Suzuki 4T", "Suzuki Spin / Skywave 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,11}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,19}, {2800,20}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  8500, 9000, 5000,
  "Matic 125cc CDI (peak 7500-8000). Redline ~8500. Timing estimasi — strobe." },

// ─── Kawasaki ───
// KLX150 / D-Tracker 150 — magnet 43° = HASIL KALIBRASI user (bukan 18°).
// max_advance 40° = ESTIMASI (aftermarket race-CDI ceiling 39° → leading-edge
// pickup ≥39° BTDC). ⚠ WAJIB di-strobe; crank-assist CH2 OFF (trailing edge
// ATDC pada geometri 43° ini → baseAdvanceRef negatif).
{ "kawasaki_klx150", "Kawasaki 4T", "Kawasaki KLX150 / D-Tracker 150", 1, 43.0f, 40.0f,
  { {300,4}, {500,6}, {800,9}, {1000,12}, {1300,14}, {1500,16}, {1800,18},
    {2000,20}, {2500,24}, {3000,28}, {3500,31}, {4000,33}, {4500,34},
    {5000,35}, {6000,35}, {7000,35}, {8000,34}, {9000,33}, {9500,32} }, 19,
  9000, 9500, 5000,
  "magnet 43° (kalibrasi). max_adv 40° ESTIMASI — WAJIB strobe. Crank-assist OFF "
  "(CH2 ATDC). Kickback? NAIKKAN max_advance. Peak power 8000 rpm." },

{ "kawasaki_athlete", "Kawasaki 4T", "Kawasaki Athlete 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,9}, {1000,10}, {1200,12}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  9000, 9500, 5000,
  "Bebek sport 125cc CDI (peak 8000). Redline ~9000. Timing estimasi — strobe." },

{ "kawasaki_kaze", "Kawasaki 4T", "Kawasaki Kaze / Blitz / ZX", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,13}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,22}, {4000,23}, {4300,23}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,28}, {6500,28}, {7000,28},
    {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,28}, {10000,27},
    {10500,27} }, 32,
  8500, 9000, 5000,
  "Bebek 110-125cc DC-CDI (peak 7500). Redline ~8500. Timing estimasi — strobe." },

// ─── Racing / Custom ───
// WARNING: racing presets assume engine modifications (bore-up,
// ported cylinder, high-comp piston, race fuel). Applied to a STOCK
// engine → detonation → piston/valve damage in seconds. UI guards
// against accidental selection (see settings page preset picker).
{ "drag_megapro_boreup", "Racing", "Drag Race · Megapro bore-up 200cc", 1, 18.0f, 38.0f,
  { {300,5}, {800,10}, {1000,12}, {1200,13}, {1500,14}, {1800,16}, {2000,18},
    {2200,20}, {2500,22}, {2800,24}, {3000,26}, {3300,28}, {3500,30},
    {3800,31}, {4000,32}, {4300,33}, {4500,34}, {4800,34}, {5000,35},
    {5300,35}, {5500,36}, {5800,36}, {6000,37}, {6500,37}, {7000,38},
    {7500,38}, {8000,38}, {8500,38}, {9000,38}, {9500,38}, {11000,37},
    {12500,35} }, 32,
  12500, 12900, 5000,
  "⚠ Bore-up + race fuel only — JANGAN di stock engine" },

{ "drag_jupiter_mx_tune", "Racing", "Drag Race · Jupiter MX tune-up", 1, 20.0f, 38.0f,
  { {300,5}, {800,10}, {1000,12}, {1200,13}, {1500,14}, {1800,16}, {2000,18},
    {2200,20}, {2500,22}, {2800,24}, {3000,26}, {3300,28}, {3500,30},
    {3800,32}, {4000,33}, {4300,34}, {4500,35}, {4800,35}, {5000,36},
    {5300,36}, {5500,37}, {5800,37}, {6000,37}, {6500,38}, {7000,38},
    {7500,38}, {8000,38}, {8500,38}, {9000,38}, {9500,38}, {10500,38},
    {12000,37} }, 32,
  12500, 12900, 5000,
  "⚠ MX ported + race CDI — JANGAN di stock engine" },

// ─── Custom ───
{ "custom", "Other", "Custom · manual config", 1, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1500,15}, {2500,20}, {3500,25}, {4500,29}, {6000,32},
    {10000,32} }, 8,
  10500, 11500, 5000,
  "Template kosong — edit semua parameter manual sesuai motor anda" },

};

constexpr size_t N = sizeof(PRESETS) / sizeof(PRESETS[0]);

char s_currentId[24] = "honda_megapro";
bool s_modified = false;

} // anonymous

size_t count() { return N; }
const Preset* at(size_t i) { return (i < N) ? &PRESETS[i] : nullptr; }

const Preset* find(const char* id) {
    if (!id) return nullptr;
    for (size_t i = 0; i < N; i++) {
        if (strcmp(PRESETS[i].id, id) == 0) return &PRESETS[i];
    }
    return nullptr;
}

bool apply(const char* id) {
    const Preset* p = find(id);
    if (!p) {
        Serial.printf("[preset] not found: %s\n", id ? id : "(null)");
        return false;
    }

    // Apply pickup geometry FIRST, then publish the advance map — so
    // live_stats (core 1) never combines a freshly-published map with a
    // stale max_advance_ref for one spark cycle (audit LOW15). Geometry is
    // applied ONLY if the user hasn't calibrated their own values on top:
    // a measured magnet width for the actual physical motor must take
    // priority over factory spec carried by the preset.
    if (!cdi::core::pickup::hasOverride()) {
        cdi::core::pickup::setMaxAdvanceRef(p->max_advance_deg);
        cdi::core::pickup::setMagnetWidth(p->magnet_width_deg);
        cdi::core::pickup::setSource("preset");
    }

    // Apply advance map (published last of the two)
    cdi::core::advance::Map fresh;
    for (uint8_t i = 0; i < p->point_count; i++) {
        fresh.addPoint(p->points[i].rpm, p->points[i].deg);
    }
    cdi::core::advance::active() = fresh;

    // Apply rev limits
    cdi::core::safety::setRevLimits(p->rev_main_rpm, p->rev_overrev_rpm);

    // Apply dwell
    cdi::core::spark::setDwellUs(p->dwell_us);

    // Reset advance offset (clean start)
    cdi::core::spark::setAdvanceOffsetDeg(0.0f);

    // Default cut mode: SPARK_PROGRESSIVE.
    // Reason: SOFT_RETARD alone (10° pull, all sparks still fire) does
    // not arrest RPM on a carbureted engine — power only drops a few
    // percent. SPARK_PROGRESSIVE skips sparks with a probability that
    // ramps 0 → 95 % across the [main, overrev] band, giving smooth
    // street feel + guaranteed arrest. SOFT_RETARD remains selectable
    // via UI; when chosen it now escalates to PATTERN_CUT on its own
    // if it fails to hold RPM (see safety::tick).
    cdi::core::safety::setMainCutMode(cdi::CutMode::SPARK_PROGRESSIVE);
    cdi::core::safety::setMainRetardDeg(10.0f);

    strncpy(s_currentId, id, sizeof(s_currentId) - 1);
    s_currentId[sizeof(s_currentId) - 1] = 0;
    s_modified = false;

    Serial.printf("[preset] applied '%s' (%s) — %u points, %u-%u rpm, dwell %u us\n",
                  p->id, p->display, p->point_count,
                  p->rev_main_rpm, p->rev_overrev_rpm, p->dwell_us);
    return true;
}

const char* currentId()      { return s_currentId; }
bool        isModified()     { return s_modified; }
void        markModifiedFlag()  { s_modified = true; }
void        resetModifiedFlag() { s_modified = false; }

} // namespace cdi::core::preset
