#include "scope/adc_sampler.h"

#include <Arduino.h>
#include <driver/adc.h>

#include "config.h"
#include "pinmap.h"

namespace cdi::scope {
namespace {

// Ring buffer storage — BSS, static, never freed. Sized from config.
constexpr uint32_t BUF_SIZE = cdi::config::BUF_SIZE_SCOPE;
volatile uint16_t s_buf1[BUF_SIZE];
volatile uint16_t s_buf2[BUF_SIZE];
volatile uint32_t s_writeIdx = 0;
portMUX_TYPE      s_bufMux   = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t*       s_timer    = nullptr;
volatile uint32_t s_rate_hz  = 0;
volatile bool     s_paused   = false;

void IRAM_ATTR isrSample() {
    if (s_paused) return;
    uint16_t v1 = adc1_get_raw(cdi::pins::PULSER_CH1_ADC);
    uint16_t v2 = adc1_get_raw(cdi::pins::PULSER_CH2_ADC);
    portENTER_CRITICAL_ISR(&s_bufMux);
    uint32_t idx = s_writeIdx % BUF_SIZE;
    s_buf1[idx] = v1;
    s_buf2[idx] = v2;
    s_writeIdx++;
    portEXIT_CRITICAL_ISR(&s_bufMux);
}

uint32_t clampRate(uint32_t hz) {
    if (hz < cdi::config::SCOPE_RATE_MIN) hz = cdi::config::SCOPE_RATE_MIN;
    if (hz > cdi::config::SCOPE_RATE_MAX) hz = cdi::config::SCOPE_RATE_MAX;
    return hz;
}

} // anonymous

void begin(uint32_t rate_hz) {
    if (s_timer) return;  // already running

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(cdi::pins::PULSER_CH1_ADC, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(cdi::pins::PULSER_CH2_ADC, ADC_ATTEN_DB_12);

    s_rate_hz = clampRate(rate_hz);
    uint32_t ticks = 1000000UL / s_rate_hz;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    s_timer = timerBegin(1000000);
    timerAttachInterrupt(s_timer, &isrSample);
    timerAlarm(s_timer, ticks, true, 0);
#else
    s_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(s_timer, &isrSample, true);
    timerAlarmWrite(s_timer, ticks, true);
    timerAlarmEnable(s_timer);
#endif

    Serial.printf("[scope] sampling @ %u Hz\n", (unsigned)s_rate_hz);
}

void end() {
    if (!s_timer) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerEnd(s_timer);
#else
    timerAlarmDisable(s_timer);
    timerEnd(s_timer);
#endif
    s_timer = nullptr;
}

uint32_t setRate(uint32_t hz) {
    hz = clampRate(hz);
    s_rate_hz = hz;
    uint32_t ticks = 1000000UL / hz;

    if (!s_timer) return hz;  // not running; rate stored for next begin()

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAlarm(s_timer, ticks, true, 0);
#else
    // Update period only; toggling enable would re-register the ISR and
    // fail with "timer_isr_callback_add register failed" on core v2.x.
    timerAlarmWrite(s_timer, ticks, true);
#endif
    Serial.printf("[scope] rate -> %u Hz\n", (unsigned)hz);
    return hz;
}

uint32_t getRate()           { return s_rate_hz; }
void     setPaused(bool p)   { s_paused = p; }
bool     isPaused()          { return s_paused; }

Snapshot snapshot() {
    Snapshot s;
    s.buf_size = BUF_SIZE;
    s.ch1 = s_buf1;
    s.ch2 = s_buf2;
    portENTER_CRITICAL(&s_bufMux);
    s.start_idx = s_writeIdx;
    portEXIT_CRITICAL(&s_bufMux);
    return s;
}

} // namespace cdi::scope
