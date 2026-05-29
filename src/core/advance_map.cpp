#include "core/advance_map.h"

#include <Arduino.h>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace cdi::core::advance {
namespace {

Map s_active;

// Spinlock — protects s_active across cores. WS handlers on core 0
// modify the map via Map::set/loadFromJson/loadDefaultMegapro while
// live_stats::tick on core 1 reads via Map::lookup per spark cycle.
// portMUX gives mutual exclusion across cores AND against same-core
// preemption, with near-zero overhead (~100 cycles when uncontended).
// Critical section length: ~100 µs for full 32-point copy, ~400 ns
// for one lookup — utilisation negligible.
portMUX_TYPE s_mapMux = portMUX_INITIALIZER_UNLOCKED;

bool inRange(cdi::rpm_t rpm, float deg) {
    return rpm >= cdi::config::RPM_MIN_VALID && rpm <= cdi::config::RPM_MAX_VALID
        && deg >= cdi::config::ADVANCE_MIN_DEG && deg <= cdi::config::ADVANCE_MAX_DEG;
}

} // anonymous

const char* Map::validateForSafety(const Point* pts, size_t n) {
    if (!pts || n == 0) return "empty map";
    // First point should be conservative at cranking
    if (pts[0].rpm <= 500 && pts[0].deg > 8.0f) {
        return "first point >8° BTDC at low RPM — kickback risk";
    }
    // Hard cap on extreme advance
    for (size_t i = 0; i < n; i++) {
        if (pts[i].deg > 45.0f) return "advance >45° BTDC — detonation guaranteed";
        if (pts[i].deg < 0.0f)  return "advance <0° (post-TDC) — combustion fail";
    }
    // Monotonic-non-decreasing RPM. Equal RPMs are allowed (lookup
    // takes the first match, harmless) so a user who drags a point to
    // exactly the same RPM as its neighbor doesn't get rejected — they
    // probably intended a step in the curve.
    for (size_t i = 1; i < n; i++) {
        if (pts[i].rpm < pts[i-1].rpm) return "RPM tidak naik berurutan";
    }
    // Idle band sanity (1000-1800 rpm should be ≤ 18° BTDC for stock fuel)
    for (size_t i = 0; i < n; i++) {
        if (pts[i].rpm >= 1000 && pts[i].rpm <= 1800 && pts[i].deg > 18.0f) {
            return "idle-band advance >18° — detonation risk at stock fuel";
        }
    }
    // No huge jumps between consecutive points (engine stumble).
    // Loosened from 8°/10° to 12°/12° so users have room to drag points
    // around in the UI editor without each edit immediately failing
    // validation. The thresholds still catch obvious typos (one
    // out-of-pattern point in an otherwise smooth curve) while
    // permitting legitimate aggressive curve shapes like deep retard
    // at peak RPM. Real-world detonation risk only kicks in around
    // 15-18° jumps on stock fuel.
    for (size_t i = 1; i < n; i++) {
        const float jump = pts[i].deg - pts[i-1].deg;
        if (jump > 12.0f || jump < -12.0f) {
            return "lompatan derajat >12° antar titik berurutan";
        }
    }
    return nullptr;   // all clear
}

void Map::clear() { count_ = 0; }

bool Map::set(const Point* pts, size_t n) {
    if (n == 0 || n > cdi::config::MAX_ADVANCE_POINTS) return false;
    for (size_t i = 0; i < n; i++) {
        if (!inRange(pts[i].rpm, pts[i].deg)) return false;
    }
    // Engine-safety validation — reject obviously dangerous maps.
    if (const char* err = validateForSafety(pts, n)) {
        Serial.printf("[advance] map rejected: %s\n", err);
        return false;
    }
    // Sort into a LOCAL buffer OUTSIDE the spinlock. The insertion sort
    // is O(n²) for up to 32 points; running it inside portENTER_CRITICAL
    // (as it used to) held interrupts disabled on the spinning core for
    // the whole sort — hundreds of operations — which could stall the
    // pulser CH1 ISR on core 1 long enough to jitter one spark cycle
    // (~6° at high RPM) whenever a map edit on core 0 collided with a
    // fire. Build the sorted result here, then publish with a copy-only
    // critical section (~n word-copies, sub-µs).
    Point sorted[cdi::config::MAX_ADVANCE_POINTS];
    for (size_t i = 0; i < n; i++) sorted[i] = pts[i];
    for (size_t i = 1; i < n; i++) {
        Point key = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1].rpm > key.rpm) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = key;
    }
    // Cross-core publish: live_stats::tick (core 1) reads points_/count_
    // via lookup() concurrently with this write (called from a WS handler
    // on core 0). The spinlock prevents the reader from seeing a torn
    // copy mid-update and firing at a wrong crank angle for one cycle.
    portENTER_CRITICAL(&s_mapMux);
    for (size_t i = 0; i < n; i++) points_[i] = sorted[i];
    count_ = n;
    portEXIT_CRITICAL(&s_mapMux);
    return true;
}

bool Map::addPoint(cdi::rpm_t rpm, float deg) {
    if (count_ >= cdi::config::MAX_ADVANCE_POINTS) return false;
    if (!inRange(rpm, deg)) return false;
    portENTER_CRITICAL(&s_mapMux);
    size_t i = count_;
    while (i > 0 && points_[i - 1].rpm > rpm) {
        points_[i] = points_[i - 1];
        i--;
    }
    points_[i] = { rpm, deg };
    count_++;
    portEXIT_CRITICAL(&s_mapMux);
    return true;
}

float Map::lookup(cdi::rpm_t rpm) const {
    // Brief spinlock against concurrent map mutation on core 0.
    // Critical section is just a few interpolation operations
    // (~400 ns @ 240 MHz) — overhead per fire is negligible.
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&s_mapMux));
    float result;
    if (count_ == 0) {
        result = cdi::config::BASE_ADVANCE_FROM_CH2_DEG;
    } else if (rpm <= points_[0].rpm) {
        result = points_[0].deg;
    } else if (rpm >= points_[count_ - 1].rpm) {
        result = points_[count_ - 1].deg;
    } else {
        result = points_[count_ - 1].deg;
        for (size_t i = 0; i + 1 < count_; i++) {
            if (rpm >= points_[i].rpm && rpm <= points_[i + 1].rpm) {
                float t = (float)(rpm - points_[i].rpm) /
                          (float)(points_[i + 1].rpm - points_[i].rpm);
                result = points_[i].deg + t * (points_[i + 1].deg - points_[i].deg);
                break;
            }
        }
    }
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&s_mapMux));
    return result;
}

bool Map::loadFromJson(const JsonArrayConst& arr) {
    Point tmp[cdi::config::MAX_ADVANCE_POINTS];
    size_t n = 0;
    for (JsonVariantConst v : arr) {
        if (n >= cdi::config::MAX_ADVANCE_POINTS) return false;
        if (!v.is<JsonArrayConst>()) return false;
        JsonArrayConst pair = v.as<JsonArrayConst>();
        if (pair.size() != 2) return false;
        tmp[n].rpm = (cdi::rpm_t)(pair[0].as<int>());
        tmp[n].deg = pair[1].as<float>();
        n++;
    }
    return set(tmp, n);
}

void Map::serialize(JsonArray out) const {
    for (size_t i = 0; i < count_; i++) {
        JsonArray pair = out.add<JsonArray>();
        pair.add(points_[i].rpm);
        pair.add(points_[i].deg);
    }
}

void Map::loadDefaultMegapro() {
    // Honda Megapro factory-calibrated curve (32 points).
    // Mirrors engine_preset.cpp `honda_megapro` exactly so the
    // editor's "Muat stok" button gives the same result as applying
    // the preset from the preset picker. The pre-pass-2 7-point
    // version lived here from initial scaffolding and went stale
    // when the preset library was added — unifying the two prevents
    // user confusion about "which stock is the real stock".
    static const Point megapro[] = {
        { 200, 2.0f },  { 300, 3.0f },  { 500, 6.0f },  { 800, 8.0f },
        {1000, 9.0f },  {1200, 9.0f },  {1500,10.0f},  {1800,12.0f},
        {2000,13.0f},  {2200,14.0f},  {2500,16.0f},  {2800,18.0f},
        {3000,19.0f},  {3300,21.0f},  {3500,22.0f},  {3800,24.0f},
        {4000,25.0f},  {4300,26.0f},  {4500,27.0f},  {4800,28.0f},
        {5000,29.0f},  {5300,30.0f},  {5500,31.0f},  {5800,31.0f},
        {6000,32.0f},  {6500,32.0f},  {7000,32.0f},  {7500,32.0f},
        {8000,32.0f},  {9000,32.0f},  {10500,31.0f}, {12000,30.0f},
    };
    set(megapro, sizeof(megapro) / sizeof(megapro[0]));
}

Map& Map::operator=(const Map& other) {
    if (this == &other) return *this;
    portENTER_CRITICAL(&s_mapMux);
    for (size_t i = 0; i < other.count_; i++) {
        points_[i] = other.points_[i];
    }
    count_ = other.count_;
    portEXIT_CRITICAL(&s_mapMux);
    return *this;
}

Map& active() { return s_active; }

} // namespace cdi::core::advance
