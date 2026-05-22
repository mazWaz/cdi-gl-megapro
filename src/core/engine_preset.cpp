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
  10500, 11500, 5000,
  "Stok megapro/tiger 160-200cc, F=10° idle, PL=32° peak (factory)" },

// GL Pro/Max/Win — derived from same engine family as Megapro but
// vintage 125 cc — idle slightly leaner advance, lower redline.
{ "honda_gl_pro", "Honda 4T", "Honda GL Pro / GL Max / Win", 1, 18.0f, 32.0f,
  { {200,2}, {300,3}, {500,5}, {800,7}, {1000,8}, {1200,9}, {1500,10},
    {1800,11}, {2000,12}, {2200,13}, {2500,15}, {2800,17}, {3000,18},
    {3300,20}, {3500,21}, {3800,23}, {4000,24}, {4300,25}, {4500,26},
    {4800,27}, {5000,28}, {5300,29}, {5500,30}, {5800,31}, {6000,32},
    {6500,32}, {7000,32}, {7500,32}, {8000,32}, {8500,31}, {9000,31},
    {10000,30} }, 32,
  10000, 11000, 5000,
  "GL series vintage, idle 10° BTDC, peak 32° BTDC factory" },

{ "honda_cb150r", "Honda 4T", "Honda CB150R Streetfire", 1, 16.0f, 35.0f,
  { {300,5}, {800,12}, {1000,13}, {1200,14}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,25}, {3300,26}, {3500,27},
    {3800,28}, {4000,29}, {4300,30}, {4500,31}, {4800,32}, {5000,32},
    {5300,33}, {5500,33}, {5800,34}, {6000,34}, {6500,34}, {7000,35},
    {7500,35}, {8000,35}, {8500,35}, {9000,35}, {9500,35}, {10500,34},
    {11500,33} }, 32,
  11000, 12000, 5000,
  "Sport 150cc DOHC, high-rev, advance lebih agresif" },

{ "honda_verza", "Honda 4T", "Honda Verza 150", 1, 16.0f, 33.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,28}, {4500,29}, {4800,30}, {5000,30},
    {5300,31}, {5500,31}, {5800,32}, {6000,32}, {6500,32}, {7000,33},
    {7500,33}, {8000,33}, {8500,33}, {9000,33}, {9500,33}, {10500,32},
    {11500,31} }, 32,
  10500, 11500, 5000,
  "Commuter 150cc, advance moderate" },

{ "honda_supra_125", "Honda 4T", "Honda Supra X 125 karbu", 1, 14.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,12}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,27}, {6300,28}, {6500,28},
    {7000,28}, {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,27},
    {10500,27} }, 32,
  9500, 10500, 5000,
  "Bebek 125cc karbu, range advance kecil" },

{ "honda_revo_karbu", "Honda 4T", "Honda Revo / Blade 110 karbu", 1, 14.0f, 27.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,12}, {1800,13}, {2000,14},
    {2200,15}, {2500,17}, {2800,18}, {3000,19}, {3300,20}, {3500,21},
    {3800,22}, {4000,23}, {4300,23}, {4500,24}, {4800,24}, {5000,25},
    {5300,25}, {5500,26}, {5800,26}, {6000,26}, {6300,27}, {6500,27},
    {7000,27}, {7500,27}, {8000,27}, {8500,27}, {9000,26}, {9500,26},
    {10000,26} }, 32,
  9000, 10000, 5000,
  "Bebek 110cc karbu, idle commute" },

{ "honda_beat_karbu", "Honda 4T", "Honda Beat / Vario / Scoopy karbu", 1, 15.0f, 28.0f,
  { {300,5}, {800,8}, {1000,10}, {1200,11}, {1500,13}, {1800,15}, {2000,16},
    {2200,17}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,27}, {6300,28}, {6500,28},
    {7000,28}, {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,27},
    {10500,27} }, 32,
  9500, 10500, 5000,
  "Matic 110cc karbu, advance halus untuk idle smooth" },

{ "honda_vario_125", "Honda 4T", "Honda Vario / Vario 125 karbu", 1, 15.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,29}, {5800,30}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10000, 11000, 5000,
  "Matic 125cc, advance moderate" },

{ "honda_win_100", "Honda 4T", "Honda Win 100 / Astrea", 1, 18.0f, 30.0f,
  { {300,5}, {500,8}, {800,9}, {1000,10}, {1200,11}, {1500,13}, {1800,15},
    {2000,16}, {2200,17}, {2500,19}, {2800,20}, {3000,21}, {3300,22},
    {3500,23}, {3800,24}, {4000,25}, {4300,25}, {4500,26}, {4800,27},
    {5000,27}, {5300,28}, {5500,28}, {5800,29}, {6000,29}, {6500,30},
    {7000,30}, {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,29},
    {10000,29} }, 32,
  9000, 10000, 5000,
  "Vintage 100cc, advance lebar untuk efisiensi" },

// ─── Yamaha 4-stroke ───
{ "yamaha_mio_karbu", "Yamaha 4T", "Yamaha Mio / Vega / Jupiter Z karbu", 1, 22.0f, 32.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,21}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,30}, {5800,30}, {6000,30}, {6500,31}, {7000,31},
    {7500,32}, {8000,32}, {8500,32}, {9000,32}, {9500,32}, {10500,32},
    {11500,31} }, 32,
  10500, 11500, 5000,
  "Matic/bebek 113-115cc karbu, magnet lebar 22°" },

{ "yamaha_jupiter_mx", "Yamaha 4T", "Yamaha Jupiter MX 135", 1, 20.0f, 33.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,25}, {3300,26}, {3500,27},
    {3800,28}, {4000,29}, {4300,29}, {4500,30}, {4800,30}, {5000,31},
    {5300,31}, {5500,31}, {5800,32}, {6000,32}, {6500,32}, {7000,33},
    {7500,33}, {8000,33}, {8500,33}, {9000,33}, {9500,33}, {10500,32},
    {11500,31} }, 32,
  11000, 12000, 5000,
  "Bebek sport 135cc DOHC, advance agresif" },

{ "yamaha_vixion_carb", "Yamaha 4T", "Yamaha Vixion karbu (lama)", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,29}, {5800,30}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10500, 11500, 5000,
  "Sport 150cc karbu, model lama sebelum injection" },

{ "yamaha_byson_carb", "Yamaha 4T", "Yamaha Byson karbu (pulser NEGATIF)", 2, 20.0f, 30.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,11}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10500, 11500, 5000,
  "Naked 150cc, pulser NEGATIF — TUKAR KABEL pulser fisik" },

{ "yamaha_scorpio", "Yamaha 4T", "Yamaha Scorpio Z 225", 1, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,21}, {2800,23}, {3000,24}, {3300,25}, {3500,26},
    {3800,27}, {4000,28}, {4300,28}, {4500,29}, {4800,29}, {5000,30},
    {5300,30}, {5500,31}, {5800,31}, {6000,32}, {6500,32}, {7000,32},
    {7500,32}, {8000,32}, {8500,32}, {9000,32}, {9500,32}, {10500,31},
    {11500,30} }, 32,
  10000, 11000, 5000,
  "Sport tourer 225cc, advance lebih konservatif" },

// ─── Suzuki 4-stroke ───
{ "suzuki_shogun_125", "Suzuki 4T", "Suzuki Shogun 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10000, 11000, 5000,
  "Bebek 125cc DOHC karbu" },

{ "suzuki_smash", "Suzuki 4T", "Suzuki Smash / Smash Titan", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,13}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,23}, {4000,24}, {4300,24}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,28}, {6500,28}, {7000,28},
    {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,28}, {10000,27},
    {10500,27} }, 32,
  9500, 10500, 5000,
  "Bebek 110cc commuter" },

{ "suzuki_satria_fu_old", "Suzuki 4T", "Suzuki Satria FU 150 OLD (pulser NEGATIF)", 2, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,13}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,25}, {3300,26}, {3500,27},
    {3800,28}, {4000,29}, {4300,29}, {4500,30}, {4800,30}, {5000,31},
    {5300,31}, {5500,31}, {5800,32}, {6000,32}, {6500,32}, {7000,32},
    {7500,32}, {8000,32}, {8500,32}, {9000,32}, {9500,32}, {10500,32},
    {11500,31} }, 32,
  11000, 12000, 5000,
  "FU OLD, pulser NEGATIF — TUKAR KABEL pulser fisik" },

{ "suzuki_satria_fu_new", "Suzuki 4T", "Suzuki Satria FU 150 NEW", 1, 18.0f, 35.0f,
  { {300,5}, {800,12}, {1000,13}, {1200,14}, {1500,16}, {1800,18}, {2000,19},
    {2200,20}, {2500,22}, {2800,24}, {3000,26}, {3300,27}, {3500,28},
    {3800,29}, {4000,30}, {4300,31}, {4500,32}, {4800,32}, {5000,33},
    {5300,33}, {5500,33}, {5800,34}, {6000,34}, {6500,34}, {7000,34},
    {7500,35}, {8000,35}, {8500,35}, {9000,35}, {9500,35}, {10500,34},
    {11500,33} }, 32,
  11500, 12500, 5000,
  "FU New, advance lebih agresif untuk RPM tinggi" },

{ "suzuki_thunder_125", "Suzuki 4T", "Suzuki Thunder 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10500, 11500, 5000,
  "Sport touring 125cc" },

{ "suzuki_spin_skywave", "Suzuki 4T", "Suzuki Spin / Skywave 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,11}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,19}, {2800,20}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10000, 11000, 5000,
  "Matic 125cc" },

// ─── Kawasaki ───
{ "kawasaki_klx150", "Kawasaki 4T", "Kawasaki KLX 150 / D-Tracker", 1, 18.0f, 30.0f,
  { {300,5}, {800,10}, {1000,11}, {1200,12}, {1500,15}, {1800,17}, {2000,18},
    {2200,19}, {2500,20}, {2800,22}, {3000,23}, {3300,24}, {3500,25},
    {3800,26}, {4000,27}, {4300,27}, {4500,28}, {4800,28}, {5000,29},
    {5300,29}, {5500,29}, {5800,30}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10500, 11500, 5000,
  "Trail 150cc SOHC karbu" },

{ "kawasaki_athlete", "Kawasaki 4T", "Kawasaki Athlete 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,9}, {1000,10}, {1200,12}, {1500,14}, {1800,16}, {2000,17},
    {2200,18}, {2500,20}, {2800,21}, {3000,22}, {3300,23}, {3500,24},
    {3800,25}, {4000,26}, {4300,26}, {4500,27}, {4800,27}, {5000,28},
    {5300,28}, {5500,29}, {5800,29}, {6000,30}, {6500,30}, {7000,30},
    {7500,30}, {8000,30}, {8500,30}, {9000,30}, {9500,30}, {10500,29},
    {11500,28} }, 32,
  10000, 11000, 5000,
  "Bebek sport 125cc" },

{ "kawasaki_kaze", "Kawasaki 4T", "Kawasaki Kaze / Blitz / ZX", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1000,9}, {1200,10}, {1500,13}, {1800,14}, {2000,15},
    {2200,16}, {2500,18}, {2800,19}, {3000,20}, {3300,21}, {3500,22},
    {3800,22}, {4000,23}, {4300,23}, {4500,25}, {4800,25}, {5000,26},
    {5300,26}, {5500,27}, {5800,27}, {6000,28}, {6500,28}, {7000,28},
    {7500,28}, {8000,28}, {8500,28}, {9000,28}, {9500,28}, {10000,27},
    {10500,27} }, 32,
  9500, 10500, 5000,
  "Bebek 110-125cc" },

// ─── 2-stroke ───
// 2T fires every revolution at compression (no wasted spark) — coil
// thermal load is higher than 4T wasted-spark. Dwell shorter (4 ms).
// Research-confirmed curve shape: ramp to peak ~3000-4000 rpm, plateau
// briefly, then PROGRESSIVELY RETARD at high RPM (10-20°/1000rpm) to
// prevent detonation as combustion timing window shrinks faster than
// for 4T (no fresh charge between fires).
// 2-stroke curves: peak around 3000-5000 rpm, then DEEP RETARD at
// high RPM (research-validated 10-15°/1000rpm above peak) to prevent
// detonation. 2T has no fresh charge between fires, so heat builds
// in piston crown — retard pushes peak pressure AFTER TDC,
// transferring heat to exhaust port instead of piston crown.
{ "yamaha_rx_king", "2-Stroke", "Yamaha RX-King 135", 1, 22.0f, 24.0f,
  { {200,1}, {300,2}, {500,4}, {800,6}, {1000,8}, {1200,9}, {1500,10},
    {1800,11}, {2000,12}, {2200,13}, {2500,15}, {2800,16}, {3000,18},
    {3300,19}, {3500,21}, {3800,22}, {4000,23}, {4300,23}, {4500,24},
    {4800,24}, {5000,24}, {5500,24}, {6000,23}, {6500,22}, {7000,21},
    {7500,19}, {8000,17}, {8500,15}, {9000,12}, {9500,10}, {10000,8},
    {11000,5} }, 32,
  9000, 10000, 4000,
  "2T legend — peak 24° @ 4500-5000, retard 15°/1000rpm above 7000" },

{ "yamaha_f1zr", "2-Stroke", "Yamaha F1ZR", 1, 22.0f, 24.0f,
  { {200,1}, {300,2}, {500,4}, {800,6}, {1000,8}, {1200,9}, {1500,10},
    {1800,11}, {2000,12}, {2200,13}, {2500,15}, {2800,16}, {3000,18},
    {3300,19}, {3500,21}, {3800,22}, {4000,23}, {4300,23}, {4500,24},
    {4800,24}, {5000,24}, {5500,24}, {6000,23}, {6500,22}, {7000,21},
    {7500,19}, {8000,17}, {8500,15}, {9000,12}, {9500,10}, {10000,8},
    {10500,6} }, 32,
  9500, 10500, 4000,
  "Bebek 2T 110cc, retard agresif di high RPM" },

{ "suzuki_rgr_150", "2-Stroke", "Suzuki RGR 150 / Sport", 1, 20.0f, 25.0f,
  { {200,1}, {300,2}, {500,4}, {800,6}, {1000,8}, {1200,10}, {1500,11},
    {1800,12}, {2000,13}, {2200,14}, {2500,16}, {2800,17}, {3000,19},
    {3300,20}, {3500,21}, {3800,22}, {4000,23}, {4300,24}, {4500,25},
    {4800,25}, {5000,25}, {5500,25}, {6000,24}, {6500,23}, {7000,22},
    {7500,20}, {8000,18}, {8500,16}, {9000,14}, {9500,12}, {10000,10},
    {11000,7} }, 32,
  10500, 11500, 4000,
  "2T sport 150cc, peak 25° @ 4500-5500, retard 12°/1000 above 7000" },

{ "honda_nsr_150", "2-Stroke", "Honda NSR 150 SP / RR", 1, 20.0f, 26.0f,
  { {200,1}, {300,2}, {500,4}, {800,6}, {1000,8}, {1200,9}, {1500,10},
    {1800,12}, {2000,13}, {2200,14}, {2500,16}, {2800,17}, {3000,19},
    {3300,20}, {3500,22}, {3800,23}, {4000,24}, {4300,25}, {4500,26},
    {4800,26}, {5000,26}, {5500,26}, {6000,25}, {6500,24}, {7000,23},
    {7500,21}, {8000,19}, {8500,17}, {9000,15}, {9500,13}, {10500,10},
    {11500,7} }, 32,
  11000, 12000, 4000,
  "2T sport DOHC, peak 26° @ 4500-5500, retard 13°/1000 above 7000" },

{ "kawasaki_ninja_150rr", "2-Stroke", "Kawasaki Ninja 150 RR", 1, 20.0f, 26.0f,
  { {200,1}, {300,2}, {500,4}, {800,6}, {1000,8}, {1200,9}, {1500,10},
    {1800,12}, {2000,13}, {2200,14}, {2500,16}, {2800,17}, {3000,19},
    {3300,20}, {3500,22}, {3800,23}, {4000,24}, {4300,25}, {4500,26},
    {4800,26}, {5000,26}, {5500,26}, {6000,25}, {6500,24}, {7000,23},
    {7500,22}, {8000,20}, {8500,18}, {9000,16}, {9500,14}, {10500,11},
    {12000,8} }, 32,
  11500, 12500, 4000,
  "2T sport, peak 26° @ 4500-5500, retard 11°/1000 above 7500" },

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
  12500, 13500, 5000,
  "⚠ Bore-up + race fuel only — JANGAN di stock engine" },

{ "drag_jupiter_mx_tune", "Racing", "Drag Race · Jupiter MX tune-up", 1, 20.0f, 38.0f,
  { {300,5}, {800,10}, {1000,12}, {1200,13}, {1500,14}, {1800,16}, {2000,18},
    {2200,20}, {2500,22}, {2800,24}, {3000,26}, {3300,28}, {3500,30},
    {3800,32}, {4000,33}, {4300,34}, {4500,35}, {4800,35}, {5000,36},
    {5300,36}, {5500,37}, {5800,37}, {6000,37}, {6500,38}, {7000,38},
    {7500,38}, {8000,38}, {8500,38}, {9000,38}, {9500,38}, {10500,38},
    {12000,37} }, 32,
  12500, 13500, 5000,
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

    // Apply advance map
    cdi::core::advance::Map fresh;
    for (uint8_t i = 0; i < p->point_count; i++) {
        fresh.addPoint(p->points[i].rpm, p->points[i].deg);
    }
    cdi::core::advance::active() = fresh;

    // Apply pickup geometry — but ONLY if the user hasn't already
    // calibrated their own values on top. A measured magnet width
    // for the actual physical motor must take priority over factory
    // spec carried by the preset.
    if (!cdi::core::pickup::hasOverride()) {
        cdi::core::pickup::setMaxAdvanceRef(p->max_advance_deg);
        cdi::core::pickup::setMagnetWidth(p->magnet_width_deg);
        cdi::core::pickup::setSource("preset");
    }

    // Apply rev limits
    cdi::core::safety::setRevLimits(p->rev_main_rpm, p->rev_overrev_rpm);

    // Apply dwell
    cdi::core::spark::setDwellUs(p->dwell_us);

    // Reset advance offset (clean start)
    cdi::core::spark::setAdvanceOffsetDeg(0.0f);

    // Reset cut mode to safe default (soft retard 10°)
    cdi::core::safety::setMainCutMode(cdi::CutMode::SOFT_RETARD);
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
