#include "core/alvp.h"

#include <Arduino.h>
#include <driver/adc.h>

#include "pinmap.h"
#include "core/spark_scheduler.h"

namespace cdi::core::alvp {
namespace {

bool       s_enabled       = false;   // default off — needs divider wired
float      s_derateV       = 10.5f;
float      s_disarmV       =  9.0f;
cdi::rpm_t s_derateLimit   = 4000;

uint32_t   s_lastSampleMs  = 0;
uint32_t   s_stateSinceMs  = 0;
State      s_state         = State::NORMAL;
State      s_pendingState  = State::NORMAL;

// 8-sample moving average of raw ADC.
static constexpr size_t WIN = 8;
uint16_t   s_buf[WIN] = {0};
uint8_t    s_idx      = 0;
uint16_t   s_lastMv   = 0;

constexpr uint32_t SAMPLE_INTERVAL_MS = 500;
constexpr uint32_t HYSTERESIS_MS      = 2000;

uint16_t readAvg() {
    s_buf[s_idx] = (uint16_t)adc1_get_raw(cdi::pins::VBAT_SENSE_ADC);
    s_idx = (s_idx + 1) % WIN;
    uint32_t sum = 0;
    for (size_t i = 0; i < WIN; i++) sum += s_buf[i];
    return (uint16_t)(sum / WIN);
}

uint16_t rawToMv(uint16_t raw) {
    // raw [0..4095] → 0..3.3V at pin → × divider ratio for battery side
    float pin_v = (float)raw / (float)cdi::pins::ADC_FULL_SCALE * cdi::pins::ADC_VREF_V;
    float bat_v = pin_v * cdi::pins::VBAT_DIVIDER_RATIO;
    return (uint16_t)(bat_v * 1000.0f);
}

State computeState(float vbat) {
    if (vbat < s_disarmV) return State::DISARM_LOW;
    if (vbat < s_derateV) return State::DERATE;
    return State::NORMAL;
}

} // anonymous

void begin() {
    adc1_config_channel_atten(cdi::pins::VBAT_SENSE_ADC, ADC_ATTEN_DB_12);
    // Pre-fill window with one read to avoid spurious low at boot.
    uint16_t r0 = (uint16_t)adc1_get_raw(cdi::pins::VBAT_SENSE_ADC);
    for (size_t i = 0; i < WIN; i++) s_buf[i] = r0;
    Serial.println("[alvp] adc init · disabled until vbat divider wired");
}

bool  isEnabled()         { return s_enabled; }
void  setEnabled(bool e)  { s_enabled = e; Serial.printf("[alvp] enabled=%d\n", e ? 1 : 0); }
float derateThresholdV()  { return s_derateV; }
float disarmThresholdV()  { return s_disarmV; }

void setThresholds(float d, float dis) {
    if (d < 7.0f) d = 7.0f;
    if (d > 15.0f) d = 15.0f;
    if (dis < 6.0f) dis = 6.0f;
    if (dis >= d) dis = d - 0.5f;
    s_derateV = d;
    s_disarmV = dis;
    Serial.printf("[alvp] thresholds: derate=%.1fV disarm=%.1fV\n", d, dis);
}

void setDerateLimitRpm(cdi::rpm_t rpm) {
    if (rpm < 1000)  rpm = 1000;
    if (rpm > 12000) rpm = 12000;
    s_derateLimit = rpm;
}
cdi::rpm_t derateLimitRpm() { return s_derateLimit; }

uint16_t vbatMv()  { return s_lastMv; }
State    state()   { return s_state; }
bool     isDerated(){ return s_enabled && s_state == State::DERATE; }

void tick() {
    uint32_t now = millis();
    if (now - s_lastSampleMs < SAMPLE_INTERVAL_MS) return;
    s_lastSampleMs = now;

    uint16_t raw = readAvg();
    s_lastMv = rawToMv(raw);

    if (!s_enabled) {
        s_state = State::NORMAL;
        return;
    }

    float vbat = s_lastMv / 1000.0f;
    State target = computeState(vbat);

    if (target != s_pendingState) {
        s_pendingState = target;
        s_stateSinceMs = now;
    } else if (target != s_state && (now - s_stateSinceMs) >= HYSTERESIS_MS) {
        Serial.printf("[alvp] state change %d → %d @ %.2fV\n",
                      (int)s_state, (int)target, vbat);
        s_state = target;
        if (s_state == State::DISARM_LOW && cdi::core::spark::isArmed()) {
            cdi::core::spark::setArmed(false);
            Serial.println("[alvp] DISARM_LOW → spark disarmed");
        }
    }
}

} // namespace cdi::core::alvp
