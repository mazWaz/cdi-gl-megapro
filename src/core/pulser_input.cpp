#include "core/pulser_input.h"

#include <Arduino.h>

#include "pinmap.h"
#include "util/ring_buffer.h"
#include "core/spark_scheduler.h"

namespace cdi::core::pulser {
namespace {

constexpr size_t RING_N = 32;   // power of 2, holds 31 events
using Ring = cdi::util::SpscRing<PulserEvent, RING_N>;

Ring              s_ring;
volatile uint32_t s_total    = 0;
volatile bool     s_attached = false;

void IRAM_ATTR isrCh1() {
    cdi::micros_t t = (cdi::micros_t)micros();
    PulserEvent ev;
    ev.ts_us   = t;
    ev.channel = cdi::PulserChannel::CH1;
    s_ring.pushFromIsr(ev);
    s_total++;
    // Tell spark scheduler — IRAM-safe, no float, no malloc.
    cdi::core::spark::onPulseCh1FromIsr(t);
}

void IRAM_ATTR isrCh2() {
    PulserEvent ev;
    ev.ts_us   = (cdi::micros_t)micros();
    ev.channel = cdi::PulserChannel::CH2;
    s_ring.pushFromIsr(ev);
    s_total++;
}

} // anonymous

void begin() {
    if (s_attached) return;
    pinMode(cdi::pins::PULSER_CH1, INPUT);
    pinMode(cdi::pins::PULSER_CH2, INPUT);
    attachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH1), isrCh1, FALLING);
    attachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH2), isrCh2, FALLING);
    s_attached = true;
    Serial.println("[pulser] attached FALLING on GPIO34 (CH1) + GPIO35 (CH2)");
}

void end() {
    if (!s_attached) return;
    detachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH1));
    detachInterrupt(digitalPinToInterrupt(cdi::pins::PULSER_CH2));
    s_attached = false;
    Serial.println("[pulser] detached");
}

bool     isAttached() { return s_attached; }
bool     tryPop(PulserEvent& out) { return s_ring.pop(out); }
uint32_t totalCount() { return s_total; }
uint32_t pending()    { return (uint32_t)s_ring.size(); }

} // namespace cdi::core::pulser
