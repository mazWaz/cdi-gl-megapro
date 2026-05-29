# CDI//MGPRO — Programmable CDI for Honda Megapro / GL series (ESP32)

ESP32-based capacitor discharge ignition (CDI) controller with WiFi
captive portal tuning UI. Designed for single-cylinder 4-stroke
karbu motorcycles using a single VRS coil + 2 anti-parallel
optocoupler pulser pickup.

**Pre-production firmware.** Functional on bench with signal sim;
real-motor / road validation incomplete. See "Safety" section.

---

## Status

```
✓  Core: pulser ISR · RPM calc · advance map · spark scheduler · safety
✓  Features: shift light · dwell curve · launch · QS · backfire · ALVP
✓  Storage: NVS persistence · datalog · OTA · snapshot
✓  UI: dashboard · map editor · scope · settings (Pit-Lane Terminal aesthetic)
✓  30-motor preset library (Honda / Yamaha / Suzuki / Kawasaki / 2-stroke)
✓  Auto-calibration at idle (magnet width via bidirectional steady-state)
✓  Dual-core: persist + snapshot tasks on core 0 (mutex + binary semaphore)
✓  Hardware emergency kill (GPIO0 long-press → SAFE_HOLD)
✓  WPA2 password protection (auto-gen first boot, NVS-persisted)
✓  Multi-tooth/EMI rejection (short phantom periods dropped → no fire)
─  Real motor validation, strobe timing, EMI testing — see Safety
```

---

## Hardware

### GPIO Assignment (ESP32 DOIT DevKit V1)

| Pin    | Function                  | Direction | Notes |
|--------|---------------------------|-----------|-------|
| GPIO34 | Pulser CH1 (leading edge) | INPUT     | Pulled HIGH via 10 kΩ; opto pulls LOW |
| GPIO35 | Pulser CH2 (trailing edge)| INPUT     | Same                                  |
| GPIO25 | Spark trigger out         | OUTPUT    | HIGH ~2.5 ms → SCR gate (opto-iso)    |
| GPIO26 | Mode LED                  | OUTPUT    | green = IGNITION armed                |
| GPIO2  | Status LED (onboard)      | OUTPUT    | blink = WS client connected           |
| GPIO27 | Shift light               | OUTPUT    | HIGH when RPM > threshold             |
| GPIO13 | Launch input (clutch)     | INPUT_PULLUP | tarik clutch = LOW = 2-step active |
| GPIO14 | Quickshifter sensor       | INPUT_PULLUP | shift sensor → cut ignition X ms   |
| GPIO33 | Gear position sense       | ADC1_CH5  | voltage divider from gear switch      |
| GPIO32 | Vbat sense                | ADC1_CH4  | **REQUIRED** 1:4 divider 12V→3V       |
| GPIO0  | Boot button / panic kill  | INPUT_PULLUP | hold ≥2 s → SAFE_HOLD              |

### Required External Circuit

```
12V battery ──┬─[7805/buck]→ 5V → ESP32 USB pin
              │
              ├─[1:4 divider]→ GPIO32 (Vbat sense; REQUIRED for ALVP)
              │
              └─[OPTO IN]→ pulser primary → ground
                         (anti-parallel pair: leading + trailing)
                 OPTO OUT (open collector) → GPIO34 / GPIO35
                          (10 kΩ pull-up to 3.3V on each)

ESP32 GPIO25 → opto-isolator → SCR gate → HV stage (existing module)
                                       → ignition coil → spark plug
```

**Wiring this firmware does NOT do for you**:
- 12V→3.3V regulation with sufficient filtering for the spark-discharge EMI environment
- Opto-isolation on GPIO25 → SCR gate (mandatory, do not direct-drive)
- Shielding of pulser cable run (low-mV VRS signal vulnerable to RF)
- Conformal coating / IP65 enclosure for vibration + moisture
- **Hardware max-dwell / current-limit backstop on the coil driver**
  (e.g. a monostable that forces the gate LOW after ~8–10 ms, or a
  current-limited driver). Software cannot guarantee the primary is
  de-energized in every case: a flash erase (NVS/LittleFS/OTA) stalls
  the spark core for the erase duration, and the firmware only *mitigates*
  this by deferring flash writes while the engine fires. A hardware
  backstop is the only thing that protects the coil against a stranded
  dwell from any firmware stall/crash, full stop.

---

## Build & Flash

```bash
# Prerequisite: PlatformIO Core in PATH
pio run                    # compile
pio run -t upload          # flash firmware only (NVS preserved)
pio run -t uploadfs        # flash UI assets (wipes /snap/ snapshots!)
pio run -t monitor         # serial @ 115200
```

NVS partition (config, WiFi password, pickup calibration) survives
both `upload` and `uploadfs`. LittleFS partition (HTML/JS/CSS +
`/snap/` snapshots) is wiped by `uploadfs`. For routine code
iteration use `upload` only.

---

## WiFi Credentials

SSID and password live in `platformio.ini` as compile-time build flags:

```ini
build_flags =
    -D CDI_AP_SSID='"CDI-Megapro"'
    -D CDI_AP_PASSWORD='"ganti-password"'    ; WPA2-PSK, 8-63 ASCII chars
```

**Change them before flashing.** Defaults are intentionally weak so
nobody runs production with stock credentials.

To rotate: edit the file → `pio run -t upload`. Both values are read
from these defines on every boot — no NVS state to manage, no
runtime change path in the UI (intentional: single source of truth,
no chance of locking yourself out via a typo over WiFi).

If you push this repo to a public git remote, the password is
exposed in commit history. Treat the repo as private if that
matters to you.

## First Boot

1. Connect USB serial @ 115200 baud
2. Power on, watch boot log:
   ```
   [CDI] firmware v0.3.0 booting (reset: power-on, heap=…)
   [snap] saver task running on core 0
   [config] persist task running on core 0
   [AP] SSID=CDI-Megapro  IP=192.168.4.1  password=ganti-password
   [AP] Connect with above credentials, then open http://192.168.4.1/
   ```
3. Connect phone WiFi → `CDI-Megapro` → enter the password from `platformio.ini`
4. Open browser → `http://192.168.4.1/` (auto-redirects via captive portal)
6. Dashboard appears, mode = IGNITION → **spark is LIVE automatically**
   (market-CDI behaviour: armed whenever powered in IGNITION, no manual
   arm step). **Keep busi grounded/removed on the bench.** The only OFF
   is the panic button (hold BOOT/GPIO0 ≥2 s) or the SAFE_HOLD button.
7. Settings → Motor Preset → pick your motor
8. Settings → Pickup → "Kalibrasi sekarang" once engine idle (optional,
   refines magnet width; runs with spark live, auto-kills only on
   multi-tooth)
9. (No arm step.) Faults pulse-cut and self-recover — they never disarm,
   so you never have to re-arm from the UI mid-ride.

---

## Safety

This firmware drives the gate of an SCR that releases tens of
kilovolts into an ignition coil. **Mis-timed sparks can damage
piston, valves, or the rider.** Read this section before powering
the device with a real motor connected.

### What's Validated

- Pulser ISR latency on bench with signal generator (<10 µs)
- WS protocol + UI flow on phone, multiple page sessions
- NVS persistence across reboot + uploadfs cycles
- Auto-calibration algorithm against synthetic data
- Dual-core task isolation under WS broadcast load

### What's NOT Validated

- **Spark fired on a real engine running** at any RPM
- **Timing accuracy with a strobe light** on factory TDC mark
- **EMI/RFI tolerance** during actual HV discharge cycles
- **WDT recovery** from a real firmware hang (only configured, not exercised)
- **Thermal performance** above 40 °C ambient
- **Vibration durability** of breadboard/header connections
- **Battery brown-out behavior** without ALVP wired

### Mandatory Safety Practices

| Phase | Practice |
|-------|----------|
| Bench  | Remove spark plug from cylinder. Either run with plug grounded to engine block, or use a dummy load (1 Ω 100 W power resistor). |
| Bench  | Verify with oscilloscope on GPIO25 that pulse width matches `dwell_us` setting before connecting HV stage. |
| Bench  | Set rev limit MAIN to a conservative value (e.g. 6000 RPM) until you've seen its behaviour. |
| Bench  | Test no-signal failsafe: disconnect pulser cable while spark armed, confirm output goes LOW within 500 ms. |
| Bench  | Test panic button: hold BOOT (GPIO0) ≥ 2 s, confirm mode jumps to SAFE_HOLD on serial + UI. |
| Motor  | First start: engine on stand, busi grounded externally, ignition armed via UI. Listen for clean fire. |
| Motor  | Verify timing with strobe light on factory TDC mark at idle (~1500 RPM) before opening throttle. |
| Riding | First 5+ rides in safe area (empty lot, side street). Datalog enabled, review afterwards. |
| Always | **Keep CDI lama / stock CDI accessible** — bring it on first rides in case of firmware fail. |
| Always | Hardware emergency: hold GPIO0 ≥ 2 s → SAFE_HOLD. |

### Known Risks

- **WiFi exposed if password leaks**: anyone with the WPA2 key can
  arm/disarm/retune mid-ride. Don't share the password, change if
  compromised.
- **Multi-tooth FI motors (CB150R FI, Vario FI, R15, etc) are NOT
  compatible**. Firmware detects this during auto-calibration
  (ERR_MULTI_TOOTH → disarm), and at runtime the extra teeth produce
  periods too short to be real (implied RPM above `RPM_MAX_VALID`) that
  `rpm_calc` rejects as noise → no valid RPM → the engine simply won't
  fire. But **do not assume protection** — use only on single-magnet
  2-edge pickups (karbu single-cyl 4T).
- **No regulatory certification**. WiFi 2.4 GHz device requires
  Postel / SDPPI compliance in Indonesia for commercial sale.

---

## Architecture

```
include/
  config.h     pinmap.h     types.h
src/
  core/        pulser_input · rpm_calc · advance_map · spark_scheduler
               safety · mode · shift_light · dwell_curve · launch_control
               quickshifter · backfire · alvp · engine_preset · pickup
               pickup_cal · panic_button
  net/         wifi_ap · http_server · ws_server · ota
  storage/     config_store (NVS, persist task on core 0)
  scope/       edge_capture · edge_snapshot (saver task on core 0)
  telemetry/   live_stats · datalog
  util/        ring_buffer (SPSC lockfree)
data/          index.html · scope.html · map.html · settings.html
               app.js · style.css
```

### Threading Model

- **Core 1 (APP_CPU, real-time)**: pulser ISR (CH1+CH2 CHANGE), spark
  hw_timer ISR, Arduino loop() — drains pulser ring, computes RPM,
  schedules fire, runs safety tick.
- **Core 0 (PRO_CPU, background)**: WiFi/lwIP/AsyncTCP (Arduino
  default), `cdi_persist` task (NVS writes), `cdi_snap` task
  (LittleFS snapshot writes).

Cross-core synchronization uses FreeRTOS binary semaphores
(signaling) and mutexes (shared buffer protection). Pulser →
consumer paths use lock-free SPSC ring buffers from `util/ring_buffer.h`.

### Persistence Layout

| Where     | What | Survives |
|-----------|------|----------|
| NVS `cdiwifi/pwd` | WPA2 password | uploadfs, OTA, factory reset = NVS erase |
| NVS `cdicfg/blob` | Module settings JSON (preset, rev limits, etc) | same |
| LittleFS `/snap/*.bin` | Captured edge-stream snapshots | reboot, but **wiped by uploadfs** |
| OTA partitions | Firmware (active + inactive) | OTA swap, rollback on triple boot fail |

---

## License

This is a personal hobby/research project. No warranty, express or
implied. The author accepts no liability for damage to motorcycle,
property, or person resulting from use of this firmware. Use at
your own risk.
