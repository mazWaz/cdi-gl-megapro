#include "core/shift_light.h"

#include <Arduino.h>

#include "pinmap.h"
#include "core/rpm_calc.h"

namespace cdi::core::shift_light {
namespace {

bool        s_enabled  = true;
cdi::rpm_t  s_rpmWarn  = 7500;
cdi::rpm_t  s_rpmShift = 8500;

uint8_t     s_state    = 0;     // 0/1/2
bool        s_flashOn  = false;
uint32_t    s_lastFlashMs = 0;

constexpr uint32_t FLASH_HALFPERIOD_MS = 100;   // 5 Hz

inline void writePin(bool h) {
    digitalWrite(cdi::pins::SHIFT_LIGHT, h ? HIGH : LOW);
}

} // anonymous

void begin() {
    pinMode(cdi::pins::SHIFT_LIGHT, OUTPUT);
    writePin(false);
}

bool       isEnabled() { return s_enabled; }
cdi::rpm_t rpmWarn()   { return s_rpmWarn; }
cdi::rpm_t rpmShift()  { return s_rpmShift; }
uint8_t    state()     { return s_state; }

void setEnabled(bool e) {
    s_enabled = e;
    if (!e) {
        writePin(false);
        s_state = 0;
    }
}

void setThresholds(cdi::rpm_t warn, cdi::rpm_t shift) {
    if (warn  < 500)   warn  = 500;
    if (shift <= warn) shift = warn + 200;
    if (shift > 15000) shift = 15000;
    s_rpmWarn  = warn;
    s_rpmShift = shift;
}

void tick() {
    if (!s_enabled) { writePin(false); s_state = 0; return; }

    cdi::rpm_t rpm = cdi::core::rpm::current();
    if (rpm >= s_rpmShift) {
        writePin(true);
        s_state = 2;
    } else if (rpm >= s_rpmWarn) {
        // flashing
        uint32_t now = millis();
        if (now - s_lastFlashMs >= FLASH_HALFPERIOD_MS) {
            s_lastFlashMs = now;
            s_flashOn = !s_flashOn;
            writePin(s_flashOn);
        }
        s_state = 1;
    } else {
        writePin(false);
        s_state = 0;
    }
}

} // namespace cdi::core::shift_light
