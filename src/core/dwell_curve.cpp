#include "core/dwell_curve.h"

namespace cdi::core::dwell {
namespace {

Point  s_points[MAX_DWELL_POINTS];
size_t s_count   = 0;
bool   s_enabled = false;

bool valid(cdi::rpm_t rpm, uint16_t us) {
    return us >= 500 && us <= 8000;
}

bool set(const Point* pts, size_t n) {
    if (n == 0 || n > MAX_DWELL_POINTS) return false;
    for (size_t i = 0; i < n; i++) {
        if (!valid(pts[i].rpm, pts[i].dwell_us)) return false;
    }
    for (size_t i = 0; i < n; i++) s_points[i] = pts[i];
    s_count = n;
    // insertion sort by rpm
    for (size_t i = 1; i < s_count; i++) {
        Point key = s_points[i];
        size_t j = i;
        while (j > 0 && s_points[j-1].rpm > key.rpm) {
            s_points[j] = s_points[j-1];
            j--;
        }
        s_points[j] = key;
    }
    return true;
}

} // anonymous

bool isEnabled()       { return s_enabled; }
void setEnabled(bool e){ s_enabled = e; }
size_t count()         { return s_count; }

uint16_t lookup(cdi::rpm_t rpm) {
    if (s_count == 0) return 2500;
    if (rpm <= s_points[0].rpm) return s_points[0].dwell_us;
    if (rpm >= s_points[s_count-1].rpm) return s_points[s_count-1].dwell_us;
    for (size_t i = 0; i + 1 < s_count; i++) {
        if (rpm >= s_points[i].rpm && rpm <= s_points[i+1].rpm) {
            float t = (float)(rpm - s_points[i].rpm) /
                      (float)(s_points[i+1].rpm - s_points[i].rpm);
            float us = s_points[i].dwell_us + t * (s_points[i+1].dwell_us - s_points[i].dwell_us);
            return (uint16_t)us;
        }
    }
    return s_points[s_count-1].dwell_us;
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
