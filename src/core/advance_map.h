// Ignition advance map — RPM → degrees-BTDC lookup.
//
// Stored as a sorted-by-RPM array of (rpm, degrees) breakpoints with
// linear interpolation in between. Caller-side clamping to the first/
// last breakpoint outside the table range (so RPM=0 returns the lowest
// point's value, RPM>>max returns the highest).
//
// Storage is static (BSS) — no heap, no String. JSON I/O lives at the
// boundary; the runtime representation is plain ints/floats.
//
// All public methods are non-ISR.
#pragma once

#include <ArduinoJson.h>

#include "config.h"
#include "types.h"

namespace cdi::core::advance {

struct Point {
    cdi::rpm_t rpm;
    float      deg;       // degrees BTDC
};

class Map {
public:
    void  clear();
    bool  set(const Point* pts, size_t n);    // overwrite (sorted on insert)
    bool  addPoint(cdi::rpm_t rpm, float deg);
    float lookup(cdi::rpm_t rpm) const;        // linear interp, clamp at ends
    size_t count() const { return count_; }
    const Point& at(size_t i) const { return points_[i]; }

    bool loadFromJson(const JsonArrayConst& arr); // [[rpm,deg], ...]
    void serialize(JsonArray out) const;

    // Pre-baked Honda Megapro stock curve (15° idle → 32° max @ 6000+).
    void loadDefaultMegapro();

    // Validate that the points form a reasonable advance curve for
    // engine safety. Returns nullptr if OK, else a static error
    // message describing the violation. Checks:
    //   * First point ≤ 8° BTDC at any RPM ≤ 500 (cranking safety)
    //   * No point in idle band (1000-1800 rpm) > 18° BTDC
    //   * No two consecutive points differ by > 8° (engine stumble)
    //   * Max value never exceeds 45° BTDC (extreme detonation guard)
    //   * Points monotonically non-decreasing in RPM
    static const char* validateForSafety(const Point* pts, size_t n);

    // Atomic copy assignment — protected by the module's portMUX
    // spinlock so a concurrent lookup on the other core can't read
    // a half-updated points_ array. Required because the WS handler
    // (core 0) often does `active() = fresh;` while live_stats on
    // core 1 reads the same instance every spark cycle.
    Map& operator=(const Map& other);
    Map() = default;
    Map(const Map&) = default;   // construction is always single-thread

private:
    Point  points_[cdi::config::MAX_ADVANCE_POINTS];
    size_t count_ = 0;
};

// Global active map — read every time spark scheduler needs `advance`.
Map& active();

} // namespace cdi::core::advance
