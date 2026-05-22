// Automatic Low Voltage Protection (ALVP).
//
// Reads battery voltage via ADC on GPIO32 with an external 1:4
// voltage divider (12V → 3V at pin). Three states:
//
//   NORMAL     — vbat ≥ derate_v: nothing to do
//   DERATE     — disarm_v ≤ vbat < derate_v: safety treats this as an
//                additional rev limit (default 4000 rpm) so the rider
//                feels power drop. Indicator on UI.
//   DISARM_LOW — vbat < disarm_v: spark output force-disarmed.
//                Battery too weak to charge HV cap reliably, and
//                continuing to fire could damage the ignition coil.
//
// Hysteresis: state changes require the condition to hold for
// `HYSTERESIS_MS` (~2 s) to avoid oscillation during cranking dips.
//
// Tick from loop ~every 500 ms (sampling rate). ADC reads use 8-sample
// moving average to suppress noise.
#pragma once

#include <cstdint>
#include "types.h"

namespace cdi::core::alvp {

enum class State : uint8_t {
    NORMAL     = 0,
    DERATE     = 1,
    DISARM_LOW = 2,
};

void begin();
void tick();

bool isEnabled();
void setEnabled(bool en);

// Thresholds (volts). disarm_v must be < derate_v.
void  setThresholds(float derate_v, float disarm_v);
float derateThresholdV();
float disarmThresholdV();

void       setDerateLimitRpm(cdi::rpm_t rpm);
cdi::rpm_t derateLimitRpm();

uint16_t vbatMv();      // last measured value × 1000
State    state();

// Used by safety to clamp main limit when DERATE active.
bool isDerated();

// Multiplier applied to spark dwell when DERATE active. At low
// supply voltage, the coil takes longer to saturate to a given
// primary current — extending dwell partially compensates so the
// spark energy stays above misfire threshold. Returns 1.0 normal,
// 1.3 when derated. live_stats applies this to the configured
// dwell before passing it to the scheduler.
float dwellMultiplier();

} // namespace cdi::core::alvp
