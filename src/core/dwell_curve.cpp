#include "core/dwell_curve.h"

#include <Arduino.h>     // portMUX / portENTER_CRITICAL

namespace cdi::core::dwell {
namespace {

Point  s_points[MAX_DWELL_POINTS];
size_t s_count   = 0;
bool   s_enabled = false;

// Guards the publish (set) vs the per-fire read (lookup): set() runs on
// the WS/persist task (core 0) while lookup() runs in live_stats::tick
// on core 1 — without this the in-place sort exposes a non-monotonic
// array to the reader (audit H6). Mirrors advance_map's s_mapMux.
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

bool valid(cdi::rpm_t rpm, uint16_t us) {
    return us >= 500 && us <= 8000;
}

bool set(const Point* pts, size_t n) {
    if (n == 0 || n > MAX_DWELL_POINTS) return false;
    for (size_t i = 0; i < n; i++) {
        if (!valid(pts[i].rpm, pts[i].dwell_us)) return false;
    }
    // Build + sort into a LOCAL buffer; publish atomically under the
    // spinlock so a concurrent lookup() never sees a mid-permutation
    // (non-monotonic) array.
    Point sorted[MAX_DWELL_POINTS];
    for (size_t i = 0; i < n; i++) sorted[i] = pts[i];
    for (size_t i = 1; i < n; i++) {
        Point key = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j-1].rpm > key.rpm) {
            sorted[j] = sorted[j-1];
            j--;
        }
        sorted[j] = key;
    }
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < n; i++) s_points[i] = sorted[i];
    s_count = n;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

} // anonymous

bool isEnabled()       { return s_enabled; }
void setEnabled(bool e){ s_enabled = e; }
size_t count()         { return s_count; }

uint16_t lookup(cdi::rpm_t rpm) {
    // Snapshot the curve under the spinlock so the interpolation runs on
    // a coherent, monotonic copy even if set() publishes concurrently
    // from core 0 (audit H6). Cheap: ≤8 points.
    Point  pts[MAX_DWELL_POINTS];
    size_t n;
    portENTER_CRITICAL(&s_mux);
    n = s_count;
    for (size_t i = 0; i < n; i++) pts[i] = s_points[i];
    portEXIT_CRITICAL(&s_mux);

    if (n == 0) return 2500;
    if (rpm <= pts[0].rpm) return pts[0].dwell_us;
    if (rpm >= pts[n-1].rpm) return pts[n-1].dwell_us;
    for (size_t i = 0; i + 1 < n; i++) {
        if (rpm >= pts[i].rpm && rpm <= pts[i+1].rpm) {
            float t = (float)(rpm - pts[i].rpm) /
                      (float)(pts[i+1].rpm - pts[i].rpm);
            float us = pts[i].dwell_us + t * (pts[i+1].dwell_us - pts[i].dwell_us);
            return (uint16_t)(us + 0.5f);   // round, not truncate (audit LOW12)
        }
    }
    return pts[n-1].dwell_us;
}

bool loadFromJson(const JsonArrayConst& arr) {
    Point tmp[MAX_DWELL_POINTS];
    size_t n = 0;
    for (JsonVariantConst v : arr) {
        if (n >= MAX_DWELL_POINTS) return false;
        if (!v.is<JsonArrayConst>()) return false;
        JsonArrayConst pair = v.as<JsonArrayConst>();
        if (pair.size() != 2) return false;
        tmp[n].rpm      = (cdi::rpm_t)(pair[0].as<int>());
        tmp[n].dwell_us = (uint16_t)(pair[1].as<int>());
        n++;
    }
    return set(tmp, n);
}

void serialize(JsonArray out) {
    for (size_t i = 0; i < s_count; i++) {
        JsonArray pair = out.add<JsonArray>();
        pair.add(s_points[i].rpm);
        pair.add(s_points[i].dwell_us);
    }
}

void loadDefault() {
    static const Point flat[] = {
        { 1000, 2500 },
        { 3000, 2500 },
        { 6000, 3000 },
        {10000, 3500 },
    };
    set(flat, sizeof(flat)/sizeof(flat[0]));
}

} // namespace cdi::core::dwell
