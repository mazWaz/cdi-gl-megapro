#include "scope/edge_capture.h"

#include <Arduino.h>
#include <cstring>

#include "config.h"
#include "types.h"
#include "core/pulser_input.h"
#include "scope/edge_snapshot.h"

namespace cdi::scope::edge {
namespace {

// Frame: 1 (magic) + 4 (seq) + 4 (first_ts) + 2 (n) + N*(2+1+1)
constexpr size_t HEADER_BYTES        = 11;
constexpr size_t BYTES_PER_EVENT     = 4;
constexpr size_t FRAME_BUFFER_BYTES  =
    HEADER_BYTES + cdi::config::SCOPE_EDGE_FRAME_MAX_EVENTS * BYTES_PER_EVENT;

uint8_t  s_frame[FRAME_BUFFER_BYTES];
uint32_t s_seq             = 0;
uint32_t s_framesEmitted   = 0;
uint32_t s_lastEventCount  = 0;
uint32_t s_lastBuildMs     = 0;

} // anonymous

void begin() {
    s_seq            = 0;
    s_framesEmitted  = 0;
    s_lastEventCount = 0;
    s_lastBuildMs    = 0;
}

bool dueNow(uint32_t now_ms) {
    return (now_ms - s_lastBuildMs) >= cdi::config::SCOPE_EDGE_FRAME_INTERVAL_MS;
}

const uint8_t* buildFrame(size_t& out_len) {
    s_lastBuildMs = millis();

    // Drain up to MAX events from the scope ring.
    PulserEvent ev;
    uint16_t n = 0;
    uint32_t  first_ts_lo = 0;

    while (n < cdi::config::SCOPE_EDGE_FRAME_MAX_EVENTS) {
        if (!cdi::core::pulser::tryPopScope(ev)) break;

        // Use low 32 bits of micros — wraps every ~71 minutes, fine for
        // a ~30 ms frame window.
        const uint32_t ts_lo = (uint32_t)(ev.ts_us & 0xFFFFFFFFu);

        // Tap into the rolling snapshot ring — every event the WS sees
        // also lands in RAM, so user can "freeze" it at any moment.
        cdi::scope::snapshot::record(ts_lo, (uint8_t)ev.channel, ev.level);

        if (n == 0) {
            first_ts_lo = ts_lo;
            // Header
            s_frame[0] = cdi::config::WS_MAGIC_SCOPE_EDGE;
            memcpy(&s_frame[1], &s_seq, 4);
            memcpy(&s_frame[5], &first_ts_lo, 4);
            // n placeholder at [9..10] — filled after loop
        }

        uint32_t offset_u32 = ts_lo - first_ts_lo;
        // Clamp to 16-bit (65 ms window — plenty for 30 ms frame)
        uint16_t offset = (offset_u32 > 0xFFFFu) ? 0xFFFFu : (uint16_t)offset_u32;

        size_t p = HEADER_BYTES + n * BYTES_PER_EVENT;
        memcpy(&s_frame[p], &offset, 2);
        s_frame[p + 2] = (uint8_t)ev.channel;
        s_frame[p + 3] = ev.level;
        n++;
    }

    s_lastEventCount = n;

    if (n == 0) {
        out_len = 0;
        return nullptr;
    }

    memcpy(&s_frame[9], &n, 2);
    out_len = HEADER_BYTES + n * BYTES_PER_EVENT;
    s_seq++;
    s_framesEmitted++;
    return s_frame;
}

uint32_t framesEmitted()       { return s_framesEmitted; }
uint32_t lastFrameEventCount() { return s_lastEventCount; }

} // namespace cdi::scope::edge
