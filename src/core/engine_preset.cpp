#include "core/engine_preset.h"

#include <Arduino.h>
#include <cmath>
#include <cstring>

#include "core/advance_map.h"
#include "core/safety.h"
#include "core/spark_scheduler.h"

namespace cdi::core::preset {
namespace {

// ─── Preset library ───
// Advance maps based on common stock + community-known reference values
// for each motor. Tune your specific build from these starting points.
const Preset PRESETS[] = {

// ─── Honda 4-stroke ───
// Magnet width 22° confirmed dari spec Honda Tiger 110mm rotor:
// arc 22mm @ 0.95mm/° = 22°. Base advance (CH2 trailing) = 32-22 = 10° BTDC.
// Sumber: hondatigerefi.blogspot, rivaldoharyoprakoso 2017
{ "honda_megapro", "Honda 4T", "Honda Megapro / Tiger", 1, 22.0f, 32.0f,
  { {500,8}, {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,20},
    {3000,23}, {3500,25}, {4000,27}, {4500,29}, {5000,30}, {6000,32},
    {7500,32}, {10000,32} }, 14,
  10500, 11500, 2500,
  "Stok megapro/tiger 160cc/200cc GL series, magnet 22° (PL 32° → F 10° BTDC)" },

{ "honda_gl_pro", "Honda 4T", "Honda GL Pro / GL Max / Win", 1, 22.0f, 32.0f,
  { {500,8}, {800,10}, {1200,12}, {1500,14}, {2000,17}, {2500,20},
    {3000,22}, {3500,25}, {4000,27}, {4500,29}, {5000,30}, {6000,32},
    {8000,32}, {10000,30} }, 14,
  10000, 11000, 2500,
  "Honda 125cc OHV vintage, mirip megapro tapi redline lebih rendah" },

{ "honda_cb150r", "Honda 4T", "Honda CB150R Streetfire", 1, 16.0f, 35.0f,
  { {800,12}, {1500,16}, {2000,19}, {2500,22}, {3000,25}, {3500,27},
    {4000,29}, {4500,31}, {5000,32}, {6000,34}, {7000,35}, {8000,35},
    {9500,35}, {11000,33} }, 14,
  11000, 12000, 2300,
  "Sport 150cc DOHC, high-rev, advance lebih agresif" },

{ "honda_verza", "Honda 4T", "Honda Verza 150", 1, 16.0f, 33.0f,
  { {800,10}, {1500,14}, {2000,17}, {2500,20}, {3000,23}, {3500,25},
    {4000,27}, {4500,29}, {5000,30}, {6000,32}, {7500,33}, {9500,33} }, 12,
  10500, 11500, 2400,
  "Commuter 150cc, advance moderate" },

{ "honda_supra_125", "Honda 4T", "Honda Supra X 125 karbu", 1, 14.0f, 28.0f,
  { {800,8}, {1200,10}, {1500,12}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {5500,27}, {6500,28},
    {8500,28} }, 13,
  9500, 10500, 2400,
  "Bebek 125cc karbu, range advance kecil" },

{ "honda_revo_karbu", "Honda 4T", "Honda Revo / Blade 110 karbu", 1, 14.0f, 27.0f,
  { {800,8}, {1200,10}, {1500,12}, {2000,14}, {2500,17}, {3000,19},
    {3500,21}, {4000,23}, {4500,24}, {5000,25}, {5500,26}, {6500,27},
    {8000,27} }, 13,
  9000, 10000, 2200,
  "Bebek 110cc karbu, idle commute" },

{ "honda_beat_karbu", "Honda 4T", "Honda Beat / Vario / Scoopy karbu", 1, 15.0f, 28.0f,
  { {800,8}, {1200,11}, {1500,13}, {2000,16}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {5500,27}, {6500,28},
    {8500,28} }, 13,
  9500, 10500, 2200,
  "Matic 110cc karbu, advance halus untuk idle smooth" },

{ "honda_vario_125", "Honda 4T", "Honda Vario / Vario 125 karbu", 1, 15.0f, 30.0f,
  { {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10000, 11000, 2300,
  "Matic 125cc, advance moderate" },

{ "honda_win_100", "Honda 4T", "Honda Win 100 / Astrea", 1, 18.0f, 30.0f,
  { {500,8}, {1000,10}, {1500,13}, {2000,16}, {2500,19}, {3000,21},
    {3500,23}, {4000,25}, {4500,27}, {5000,28}, {6000,30}, {8000,30} }, 12,
  9000, 10000, 2500,
  "Vintage 100cc, advance lebar untuk efisiensi" },

// ─── Yamaha 4-stroke ───
{ "yamaha_mio_karbu", "Yamaha 4T", "Yamaha Mio / Vega / Jupiter Z karbu", 1, 22.0f, 32.0f,
  { {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,21}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,32},
    {10000,32} }, 13,
  10500, 11500, 2400,
  "Matic/bebek 113-115cc karbu, magnet lebar 55mm" },

{ "yamaha_jupiter_mx", "Yamaha 4T", "Yamaha Jupiter MX 135", 1, 20.0f, 33.0f,
  { {800,10}, {1200,13}, {1500,16}, {2000,19}, {2500,22}, {3000,25},
    {3500,27}, {4000,29}, {4500,30}, {5000,31}, {6000,32}, {7000,33},
    {8500,33}, {10500,32} }, 14,
  11000, 12000, 2300,
  "Bebek sport 135cc DOHC, advance agresif" },

{ "yamaha_vixion_carb", "Yamaha 4T", "Yamaha Vixion karbu (lama)", 1, 20.0f, 30.0f,
  { {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,22},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10500, 11500, 2400,
  "Sport 150cc karbu, model lama sebelum injection" },

{ "yamaha_byson_carb", "Yamaha 4T", "Yamaha Byson karbu (pulser NEGATIF)", 2, 20.0f, 30.0f,
  { {800,8}, {1200,11}, {1500,14}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10500, 11500, 2400,
  "Naked 150cc, pakai pulser NEGATIF (trigger CH2 trailing)" },

{ "yamaha_scorpio", "Yamaha 4T", "Yamaha Scorpio Z 225", 1, 18.0f, 32.0f,
  { {800,10}, {1200,13}, {1500,15}, {2000,18}, {2500,21}, {3000,24},
    {3500,26}, {4000,28}, {4500,29}, {5000,30}, {6000,32}, {8000,32},
    {10000,32} }, 13,
  10000, 11000, 2600,
  "Sport tourer 225cc, advance lebih konservatif" },

// ─── Suzuki 4-stroke ───
{ "suzuki_shogun_125", "Suzuki 4T", "Suzuki Shogun 125", 1, 20.0f, 30.0f,
  { {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10000, 11000, 2400,
  "Bebek 125cc DOHC karbu" },

{ "suzuki_smash", "Suzuki 4T", "Suzuki Smash / Smash Titan", 1, 18.0f, 28.0f,
  { {800,8}, {1200,10}, {1500,13}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,24}, {4500,25}, {5000,26}, {6000,28}, {8500,28} }, 12,
  9500, 10500, 2300,
  "Bebek 110cc commuter" },

{ "suzuki_satria_fu_old", "Suzuki 4T", "Suzuki Satria FU 150 OLD (pulser NEGATIF)", 2, 18.0f, 32.0f,
  { {800,10}, {1200,13}, {1500,16}, {2000,19}, {2500,22}, {3000,25},
    {3500,27}, {4000,29}, {4500,30}, {5000,31}, {6000,32}, {7000,32},
    {8500,32}, {10500,32} }, 14,
  11000, 12000, 2300,
  "Underbone sport 150cc DOHC, pulser NEGATIF (trigger CH2)" },

{ "suzuki_satria_fu_new", "Suzuki 4T", "Suzuki Satria FU 150 NEW", 1, 18.0f, 35.0f,
  { {800,12}, {1500,16}, {2000,19}, {2500,22}, {3000,26}, {3500,28},
    {4000,30}, {4500,32}, {5000,33}, {6000,34}, {7500,35}, {9000,35},
    {11000,33} }, 13,
  11500, 12500, 2200,
  "FU New, advance lebih agresif untuk RPM tinggi" },

{ "suzuki_thunder_125", "Suzuki 4T", "Suzuki Thunder 125", 1, 18.0f, 30.0f,
  { {800,10}, {1200,12}, {1500,15}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10500, 11500, 2400,
  "Sport touring 125cc" },

{ "suzuki_spin_skywave", "Suzuki 4T", "Suzuki Spin / Skywave 125", 1, 20.0f, 30.0f,
  { {800,8}, {1200,11}, {1500,14}, {2000,17}, {2500,19}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {8500,30} }, 12,
  10000, 11000, 2400,
  "Matic 125cc" },

// ─── Kawasaki ───
{ "kawasaki_klx150", "Kawasaki 4T", "Kawasaki KLX 150 / D-Tracker", 1, 18.0f, 30.0f,
  { {800,10}, {1200,12}, {1500,15}, {2000,18}, {2500,20}, {3000,23},
    {3500,25}, {4000,27}, {4500,28}, {5000,29}, {6000,30}, {7500,30},
    {9500,30} }, 13,
  10500, 11500, 2500,
  "Trail 150cc SOHC karbu" },

{ "kawasaki_athlete", "Kawasaki 4T", "Kawasaki Athlete 125", 1, 18.0f, 30.0f,
  { {800,9}, {1200,12}, {1500,14}, {2000,17}, {2500,20}, {3000,22},
    {3500,24}, {4000,26}, {4500,27}, {5000,28}, {6000,30}, {8500,30} }, 12,
  10000, 11000, 2400,
  "Bebek sport 125cc" },

{ "kawasaki_kaze", "Kawasaki 4T", "Kawasaki Kaze / Blitz / ZX", 1, 18.0f, 28.0f,
  { {800,8}, {1200,10}, {1500,13}, {2000,15}, {2500,18}, {3000,20},
    {3500,22}, {4000,23}, {4500,25}, {5000,26}, {6000,28}, {8500,28} }, 12,
  9500, 10500, 2400,
  "Bebek 110-125cc" },

// ─── 2-stroke ───
{ "yamaha_rx_king", "2-Stroke", "Yamaha RX-King 135", 1, 22.0f, 24.0f,
  { {1000,8}, {1500,10}, {2000,12}, {2500,14}, {3000,16}, {3500,18},
    {4000,19}, {4500,20}, {5000,21}, {6000,22}, {7000,23}, {8000,24},
    {9500,24} }, 13,
  9000, 10000, 1800,
  "2-Stroke legend, advance kecil karena combustion cepat" },

{ "yamaha_f1zr", "2-Stroke", "Yamaha F1ZR", 1, 22.0f, 24.0f,
  { {1000,8}, {1500,10}, {2000,12}, {2500,14}, {3000,16}, {3500,18},
    {4000,19}, {4500,20}, {5000,21}, {6000,22}, {7000,23}, {8000,24},
    {9500,24} }, 13,
  9500, 10500, 1800,
  "Bebek 2T 110cc" },

{ "suzuki_rgr_150", "2-Stroke", "Suzuki RGR 150 / Sport", 1, 20.0f, 25.0f,
  { {1000,8}, {1500,11}, {2000,13}, {2500,15}, {3000,17}, {3500,19},
    {4000,21}, {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8500,25},
    {10000,24} }, 13,
  10500, 11500, 1900,
  "2-Stroke sport 150cc" },

{ "honda_nsr_150", "2-Stroke", "Honda NSR 150 SP / RR", 1, 20.0f, 26.0f,
  { {1500,10}, {2000,13}, {2500,15}, {3000,17}, {3500,19}, {4000,21},
    {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8000,26}, {9500,26},
    {11000,24} }, 13,
  11000, 12000, 1900,
  "2-Stroke sport 150cc DOHC" },

{ "kawasaki_ninja_150rr", "2-Stroke", "Kawasaki Ninja 150 RR", 1, 20.0f, 26.0f,
  { {1500,10}, {2000,13}, {2500,15}, {3000,17}, {3500,19}, {4000,21},
    {4500,22}, {5000,23}, {6000,24}, {7000,25}, {8500,26}, {10500,26},
    {12000,24} }, 13,
  11500, 12500, 1900,
  "2-Stroke sport, advance moderate" },

// ─── Racing / Custom ───
{ "drag_megapro_boreup", "Racing", "Drag Race · Megapro bore-up 200cc", 1, 18.0f, 38.0f,
  { {1500,14}, {2000,18}, {2500,22}, {3000,26}, {3500,30}, {4000,32},
    {4500,34}, {5000,35}, {5500,36}, {6000,37}, {7000,38}, {8000,38},
    {9500,38}, {11500,36} }, 14,
  12500, 13500, 2300,
  "Bore-up + tuning, advance agresif untuk drag" },

{ "drag_jupiter_mx_tune", "Racing", "Drag Race · Jupiter MX tune-up", 1, 20.0f, 38.0f,
  { {1500,14}, {2000,18}, {2500,22}, {3000,26}, {3500,30}, {4000,33},
    {4500,35}, {5000,36}, {5500,37}, {6500,38}, {8000,38}, {10000,38},
    {12000,36} }, 13,
  12500, 13500, 2300,
  "MX dengan ported cyl + race CDI" },

// ─── Custom ───
{ "custom", "Other", "Custom · manual config", 1, 18.0f, 32.0f,
  { {800,10}, {1500,15}, {2500,20}, {3500,25}, {4500,29}, {6000,32},
    {10000,32} }, 7,
  10500, 11500, 2500,
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

size_t suggestByMagnetWidth(float measured_deg, float tolerance_deg,
                            const char** out_ids, size_t max_results) {
    if (!out_ids || max_results == 0) return 0;
    // Simple insertion sort by absolute deviation. With ~30 presets and
    // max_results ~5 this is trivial.
    struct Cand { float dev; const char* id; };
    Cand best[8] = {};
    size_t cap = (max_results > 8) ? 8 : max_results;
    for (size_t k = 0; k < cap; k++) best[k] = {1e9f, nullptr};

    for (size_t i = 0; i < N; i++) {
        const float dev = fabsf(PRESETS[i].magnet_width_deg - measured_deg);
        if (dev > tolerance_deg) continue;
        // Insert into best[] sorted by dev ascending.
        for (size_t k = 0; k < cap; k++) {
            if (dev < best[k].dev) {
                for (size_t j = cap - 1; j > k; j--) best[j] = best[j-1];
                best[k] = { dev, PRESETS[i].id };
                break;
            }
        }
    }
    size_t out = 0;
    for (size_t k = 0; k < cap; k++) {
        if (!best[k].id) break;
        out_ids[out++] = best[k].id;
    }
    return out;
}

} // namespace cdi::core::preset
