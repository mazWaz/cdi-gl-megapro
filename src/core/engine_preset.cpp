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
{ "honda_megapro", "Honda 4T", "Honda Megapro / Tiger", 1, 18.0f, 32.0f,
  { {300,5}, {500,8}, {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,20},
    {3000,23}, {3500,25}, {4000,27}, {4500,29}, {5000,30}, {6000,32},
    {7500,32}, {10000,32} }, 15,
  10500, 11500, 5000,
  "Stok megapro 160cc, ignition single-coil dual-edge via 2 opto" },

{ "honda_gl_pro", "Honda 4T", "Honda GL Pro / GL Max / Win", 1, 18.0f, 32.0f,
  { {300,5}, {500,8}, {800,10}, {1200,12}, {1500,14}, {2000,17}, {2500,20},
    {3000,22}, {3500,25}, {4000,27}, {4500,29}, {5000,30}, {6000,32},
    {8000,32}, {10000,30} }, 15,
  10000, 11000, 5000,
  "Honda 125cc OHV vintage, mirip megapro tapi redline lebih rendah" },

{ "honda_cb150r", "Honda 4T", "Honda CB150R Streetfire", 1, 16.0f, 35.0f,
  { {300,5}, {800,12}, {1500,16}, {2000,19}, {2500,22}, {3000,25}, {3500,27},
    {4000,29}, {4500,31}, {5000,32}, {6000,34}, {7000,35}, {8000,35},
    {9500,35}, {11000,33} }, 15,
  11000, 12000, 5000,
  "Sport 150cc DOHC, high-rev, advance lebih agresif" },

{ "honda_verza", "Honda 4T", "Honda Verza 150", 1, 16.0f, 33.0f,
  { {300,5}, {800,10}, {1500,14}, {2000,17}, {2500,20}, {3000,23}, {3500,25},
    {4000,27}, {4500,29}, {5000,30}, {6000,32}, {7500,33}, {9500,33} }, 13,
  10500, 11500, 5000,
  "Commuter 150cc, advance moderate" },

{ "honda_supra_125", "Honda 4T", "Honda Supra X 125 karbu", 1, 14.0f, 28.0f,
  { {300,5}, {800,8}, {1200,10}, {1500,12}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {5500,27}, {6500,28},
    {8500,28} }, 14,
  9500, 10500, 5000,
  "Bebek 125cc karbu, range advance kecil" },

{ "honda_revo_karbu", "Honda 4T", "Honda Revo / Blade 110 karbu", 1, 14.0f, 27.0f,
  { {300,5}, {800,8}, {1200,10}, {1500,12}, {2000,14}, {2500,17}, {3000,19},
    {3500,21}, {4000,23}, {4500,24}, {5000,25}, {5500,26}, {6500,27},
    {8000,27} }, 14,
  9000, 10000, 5000,
  "Bebek 110cc karbu, idle commute" },

{ "honda_beat_karbu", "Honda 4T", "Honda Beat / Vario / Scoopy karbu", 1, 15.0f, 28.0f,
  { {300,5}, {800,8}, {1200,11}, {1500,13}, {2000,16}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {5500,27}, {6500,28},
    {8500,28} }, 14,
  9500, 10500, 5000,
  "Matic 110cc karbu, advance halus untuk idle smooth" },

{ "honda_vario_125", "Honda 4T", "Honda Vario / Vario 125 karbu", 1, 15.0f, 30.0f,
  { {300,5}, {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10000, 11000, 5000,
  "Matic 125cc, advance moderate" },

{ "honda_win_100", "Honda 4T", "Honda Win 100 / Astrea", 1, 18.0f, 30.0f,
  { {300,5}, {500,8}, {1000,10}, {1500,13}, {2000,16}, {2500,19}, {3000,21},
    {3500,23}, {4000,25}, {4500,27}, {5000,28}, {6000,30}, {8000,30} }, 13,
  9000, 10000, 5000,
  "Vintage 100cc, advance lebar untuk efisiensi" },

// ─── Yamaha 4-stroke ───
{ "yamaha_mio_karbu", "Yamaha 4T", "Yamaha Mio / Vega / Jupiter Z karbu", 1, 22.0f, 32.0f,
  { {300,5}, {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,21}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,32},
    {10000,32} }, 14,
  10500, 11500, 5000,
  "Matic/bebek 113-115cc karbu, magnet lebar 55mm" },

{ "yamaha_jupiter_mx", "Yamaha 4T", "Yamaha Jupiter MX 135", 1, 20.0f, 33.0f,
  { {300,5}, {800,10}, {1200,13}, {1500,16}, {2000,19}, {2500,22}, {3000,25},
    {3500,27}, {4000,29}, {4500,30}, {5000,31}, {6000,32}, {7000,33},
    {8500,33}, {10500,32} }, 15,
  11000, 12000, 5000,
  "Bebek sport 135cc DOHC, advance agresif" },

{ "yamaha_vixion_carb", "Yamaha 4T", "Yamaha Vixion karbu (lama)", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,22},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10500, 11500, 5000,
  "Sport 150cc karbu, model lama sebelum injection" },

{ "yamaha_byson_carb", "Yamaha 4T", "Yamaha Byson karbu (pulser NEGATIF)", 2, 20.0f, 30.0f,
  { {300,5}, {800,8}, {1200,11}, {1500,14}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10500, 11500, 5000,
  "Naked 150cc, pulser NEGATIF — TUKAR KABEL pulser fisik (leading-edge ke GPIO34)" },

{ "yamaha_scorpio", "Yamaha 4T", "Yamaha Scorpio Z 225", 1, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,21}, {3000,24},
    {3500,26}, {4000,28}, {4500,29}, {5000,30}, {6000,32}, {8000,32},
    {10000,32} }, 14,
  10000, 11000, 5000,
  "Sport tourer 225cc, advance lebih konservatif" },

// ─── Suzuki 4-stroke ───
{ "suzuki_shogun_125", "Suzuki 4T", "Suzuki Shogun 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10000, 11000, 5000,
  "Bebek 125cc DOHC karbu" },

{ "suzuki_smash", "Suzuki 4T", "Suzuki Smash / Smash Titan", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1200,10}, {1500,13}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {6000,28}, {8500,28} }, 13,
  9500, 10500, 5000,
  "Bebek 110cc commuter" },

{ "suzuki_satria_fu_old", "Suzuki 4T", "Suzuki Satria FU 150 OLD (pulser NEGATIF)", 2, 18.0f, 32.0f,
  { {300,5}, {800,10}, {1200,13}, {1500,16}, {2000,19}, {2500,22}, {3000,25},
    {3500,27}, {4000,29}, {4500,30}, {5000,31}, {6000,32}, {7000,32},
    {8500,32}, {10500,32} }, 15,
  11000, 12000, 5000,
  "FU OLD, pulser NEGATIF — TUKAR KABEL pulser fisik (leading-edge ke GPIO34)" },

{ "suzuki_satria_fu_new", "Suzuki 4T", "Suzuki Satria FU 150 NEW", 1, 18.0f, 35.0f,
  { {300,5}, {800,12}, {1500,16}, {2000,19}, {2500,22}, {3000,26}, {3500,28},
    {4000,30}, {4500,32}, {5000,33}, {6000,34}, {7500,35}, {9000,35},
    {11000,33} }, 14,
  11500, 12500, 5000,
  "FU New, advance lebih agresif untuk RPM tinggi" },

{ "suzuki_thunder_125", "Suzuki 4T", "Suzuki Thunder 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,10}, {1200,12}, {1500,15}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10500, 11500, 5000,
  "Sport touring 125cc" },

{ "suzuki_spin_skywave", "Suzuki 4T", "Suzuki Spin / Skywave 125", 1, 20.0f, 30.0f,
  { {300,5}, {800,8}, {1200,11}, {1500,14}, {2000,17}, {2500,19}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {8500,30} }, 13,
  10000, 11000, 5000,
  "Matic 125cc" },

// ─── Kawasaki ───
{ "kawasaki_klx150", "Kawasaki 4T", "Kawasaki KLX 150 / D-Tracker", 1, 18.0f, 30.0f,
  { {300,5}, {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 14,
  10500, 11500, 5000,
  "Trail 150cc SOHC karbu" },

{ "kawasaki_athlete", "Kawasaki 4T", "Kawasaki Athlete 125", 1, 18.0f, 30.0f,
  { {300,5}, {800,9}, {1200,12}, {1500,14}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {8500,30} }, 13,
  10000, 11000, 5000,
  "Bebek sport 125cc" },

{ "kawasaki_kaze", "Kawasaki 4T", "Kawasaki Kaze / Blitz / ZX", 1, 18.0f, 28.0f,
  { {300,5}, {800,8}, {1200,10}, {1500,13}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,23}, {4500,25}, {5000,26}, {6000,28}, {8500,28} }, 13,
  9500, 10500, 5000,
  "Bebek 110-125cc" },

// ─── 2-stroke ───
// 2T fires every revolution at compression (no wasted spark), so coil
// thermal load is higher. Slightly shorter dwell (4 ms) than 4T 5 ms.
{ "yamaha_rx_king", "2-Stroke", "Yamaha RX-King 135", 1, 22.0f, 24.0f,
  { {300,3}, {1000,8}, {1500,10}, {2000,12}, {2500,14}, {3000,16}, {3500,18},
    {4000,19}, {4500,20}, {5000,21}, {6000,22}, {7000,23}, {8000,24},
    {9500,24} }, 14,
  9000, 10000, 4000,
  "2-Stroke legend, advance kecil karena combustion cepat" },

{ "yamaha_f1zr", "2-Stroke", "Yamaha F1ZR", 1, 22.0f, 24.0f,
  { {300,3}, {1000,8}, {1500,10}, {2000,12}, {2500,14}, {3000,16}, {3500,18},
    {4000,19}, {4500,20}, {5000,21}, {6000,22}, {7000,23}, {8000,24},
    {9500,24} }, 14,
  9500, 10500, 4000,
  "Bebek 2T 110cc" },

{ "suzuki_rgr_150", "2-Stroke", "Suzuki RGR 150 / Sport", 1, 20.0f, 25.0f,
  { {300,3}, {1000,8}, {1500,11}, {2000,13}, {2500,15}, {3000,17}, {3500,19},
    {4000,21}, {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8500,25},
    {10000,24} }, 14,
  10500, 11500, 4000,
  "2-Stroke sport 150cc" },

{ "honda_nsr_150", "2-Stroke", "Honda NSR 150 SP / RR", 1, 20.0f, 26.0f,
  { {300,3}, {1500,10}, {2000,13}, {2500,15}, {3000,17}, {3500,19}, {4000,21},
    {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8000,26}, {9500,26},
    {11000,24} }, 14,
  11000, 12000, 4000,
  "2-Stroke sport 150cc DOHC" },

{ "kawasaki_ninja_150rr", "2-Stroke", "Kawasaki Ninja 150 RR", 1, 20.0f, 26.0f,
  { {300,3}, {1500,10}, {2000,13}, {2500,15}, {3000,17}, {3500,19}, {4000,21},
    {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8500,26}, {10500,26},
    {12000,24} }, 14,
  11500, 12500, 4000,
  "2-Stroke sport, advance moderate" },

// ─── Racing / Custom ───
// WARNING: racing presets assume engine modifications (bore-up,
// ported cylinder, high-comp piston, race fuel). Applied to a STOCK
// engine → detonation → piston/valve damage in seconds. UI guards
// against accidental selection (see settings page preset picker).
{ "drag_megapro_boreup", "Racing", "Drag Race · Megapro bore-up 200cc", 1, 18.0f, 38.0f,
  { {300,5}, {1500,14}, {2000,18}, {2500,22}, {3000,26}, {3500,30}, {4000,32},
    {4500,34}, {5000,35}, {5500,36}, {6000,37}, {7000,38}, {8000,38},
    {9500,38}, {11500,36} }, 15,
  12500, 13500, 5000,
  "⚠ Bore-up + race fuel only, advance agresif untuk drag — JANGAN di stock engine" },

{ "drag_jupiter_mx_tune", "Racing", "Drag Race · Jupiter MX tune-up", 1, 20.0f, 38.0f,
  { {300,5}, {1500,14}, {2000,18}, {2500,22}, {3000,26}, {3500,30}, {4000,33},
    {4500,35}, {5000,36}, {5500,37}, {6500,38}, {8000,38}, {10000,38},
    {12000,36} }, 14,
  12500, 13500, 5000,
  "⚠ MX dengan ported cyl + race CDI — JANGAN di stock engine" },

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
