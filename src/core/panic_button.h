// Boot button as a hardware emergency kill.
//
// GPIO0 on the DevKit doubles as BOOT (held LOW during reset puts the
// chip into UART download mode), but during normal operation it's
// just a free input with internal pull-up. We poll it from loop and
// detect a sustained press (≥ 2 s) — at which point we force
// SAFE_HOLD and disarm spark output unconditionally.
//
// Reasoning: if the firmware glitches, WiFi is unreachable, or the
// rider just wants spark off RIGHT NOW with no phone in hand, a
// hardware-rooted kill path matters. Watchdog reboot is 5 s; this is
// faster and explicit.
//
// A short tap (< 500 ms) is reserved as a future "switch profile"
// trigger but is currently a no-op (no spurious mode flip from a
// finger graze).
#pragma once

#include <cstdint>

namespace cdi::core::panic {

void begin();
void poll();   // call from loop — debounce + state machine

// True only while a long-press is being held past the trip threshold;
// cleared on button release so a SECOND emergency press re-fires the
// kill (the hardware kill must never be single-shot — audit C1). Used
// for per-press de-bounce of the action, not as a UI-latched state.
bool tripped();
void clearTrip();

} // namespace cdi::core::panic
