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
    // Monotonic RPM
    for (size_t i = 1; i < n; i++) {
        if (pts[i].rpm <= pts[i-1].rpm) return "RPM not monotonically increasing";
    }
    // Idle band sanity (1000-1800 rpm should be ≤ 18° BTDC for stock fuel)
    for (size_t i = 0; i < n; i++) {
        if (pts[i].rpm >= 1000 && pts[i].rpm <= 1800 && pts[i].deg > 18.0f) {
            return "idle-band advance >18° — detonation risk at stock fuel";
        }
    }
    // No huge jumps between consecutive points (engine stumble)
    for (size_t i = 1; i < n; i++) {
        const float jump = pts[i].deg - pts[i-1].deg;
        if (jump > 8.0f || jump < -10.0f) {
            return "consecutive points differ >8° — uneven curve, engine stumble";
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
    // Cross-core critical section: live_stats::tick (core 1) reads
    // points_/count_ via lookup() concurrently with this write
    // (called from WS handler on core 0). Without the spinlock the
    // reader could see a torn copy mid-update and fire spark at a
    // wrong crank angle for one cycle.
    portENTER_CRITICAL(&s_mapMux);
    for (size_t i = 0; i < n; i++) points_[i] = pts[i];
    count_ = n;
    // sort by rpm ascending (insertion-sort is plenty for n <= 32)
    for (size_t i = 1; i < count_; i++) {
        Point key = points_[i];
        size_t j = i;
        while (j > 0 && points_[j - 1].rpm > key.rpm) {
            points_[j] = points_[j - 1];
            j--;
        }
        points_[j] = key;
    }
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
    // Honda Megapro stock-ish curve, derived from manual_crank.csv
    // analysis + plan reference (15° base, 32° max).
    static const Point megapro[] = {
        {  800, 10.0f },
        { 1500, 15.0f },
        { 2500, 20.0f },
        { 3500, 25.0f },
        { 4500, 29.0f },
        { 6000, 32.0f },
        {10000, 32.0f },
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
