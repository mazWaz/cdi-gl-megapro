# CLAUDE.md

Guidance for Claude Code (and any contributor) working in this repository.
Read this fully before changing anything in `src/core/`, the ISR/spark path, or
persistence — this is **safety-critical, real-time ignition firmware**.

---

## ⚠️ Safety first (read this)

This firmware drives an **ignition coil** on a running motorcycle engine. A
wrong-angle or stranded spark can damage the piston/valves or injure the rider
via kickback.

- **Spark is LIVE on boot.** The "market-CDI" model auto-arms in `IGNITION`
  mode (no manual arm step). On the bench, **ground or remove the spark plug
  (busi)** — or use a dummy load — before connecting the HV stage.
- **All timing changes are UNVALIDATED on hardware** until checked with a
  timing light/strobe + oscilloscope. Code review (even the multi-agent audit)
  verifies *logic*, not real combustion timing.
- The only sticky kills are the **panic button (GPIO0, hold ≥2 s)** and a
  **multi-tooth pickup detected during calibration** → `SAFE_HOLD`. Everything
  else (over-rev, ALVP under-voltage, rev-limit) is a **self-recovering
  pulse-cut**, never a disarm.
- A hardware **max-dwell backstop** (monostable / current-limited coil driver)
  is mandatory — firmware alone cannot guarantee the coil de-energizes if a
  flash erase stalls the spark core mid-dwell (see README wiring notes).

---

## What this is

Programmable inductive/TCI **CDI ignition** for a Honda Megapro / GL-class
single-cylinder 4-stroke (and a 30-bike Indonesian preset library), running on
an **ESP32 DOIT DevKit v1**. It reads a VRS pulser (dual edge: CH1 leading +
CH2 trailing), computes per-cycle advance from an RPM→degrees map, and fires the
coil via microsecond hardware-timer ISRs. A WiFi SoftAP serves a multi-page web
UI (dashboard, map editor, edge scope, settings) over HTTP + WebSocket.

Firmware version lives in `include/config.h` (`FW_VERSION_*`, currently 0.5.5).

---

## Build / Flash / Monitor

PlatformIO project. Single env: **`esp32doit-devkit-v1`** (board same name,
platform `espressif32`, framework `arduino` = **arduino-esp32 2.0.17**,
`framework-arduinoespressif32 @ 3.20017`). Partition `default.csv`, filesystem
**LittleFS**, monitor **115200**.

```bash
pio run                                  # compile (runs build_ui.py pre-build)
pio run -t upload                        # flash firmware (auto-chains uploadfs on a fresh board)
pio run -t upload --upload-port COM4     # explicit port (Windows)
pio run -t uploadfs                      # flash LittleFS image — WIPES /snap/ snapshots
pio run -t monitor                       # serial @ 115200
```

- **`scripts/build_ui.py` (pre-build hook):** gzips everything in `web/` and
  regenerates `include/ui_pages.h` (PROGMEM arrays + ETag/MIME table). **After
  editing any file in `web/`, you MUST rebuild firmware** — the UI is embedded,
  not on the filesystem. `ui_pages.h` is generated; never hand-edit it.
- **`scripts/auto_uploadfs.py` (post-upload hook):** after a firmware `upload`
  it waits ~2 s and chains `uploadfs` so a blank board is provisioned in one
  command. Non-fatal if it fails (firmware auto-formats LittleFS on first boot).
- **WiFi credentials** are compile-time build flags `-D CDI_AP_SSID` /
  `-D CDI_AP_PASSWORD` in `platformio.ini` (defaults `CDI-Megapro` /
  `ganti-password`). ⚠ The password is plaintext in git history if the repo is
  public. A runtime NVS override exists (`setWifiSsid`/`setWifiPassword` WS cmds).

### Build/flash gotchas (Windows)
- `pio run -t upload` may end with a `UnicodeEncodeError` printing `→`/`≥` on a
  cp1252 console. This is the **post-script failing to PRINT**, *after* the
  firmware was written and the hash verified — **firmware is fine**. Confirm
  success by grepping the output for `Hash of data verified` / `[SUCCESS]`.
- **Do NOT pipe an upload through `head`** — closing the pipe early can
  interrupt the flash mid-write. Use `tail` or a filtered `grep`.
- NVS config (presets, calibration, WiFi override) **survives** firmware upload;
  it is only cleared by an explicit NVS erase. Changing a firmware *default*
  does not change an already-persisted NVS value — re-apply via the UI.

---

## Architecture: dual-core, real-time

ESP32 has two cores; the split is deliberate and safety-relevant.

**Core 1 — real-time (Arduino `loopTask` + all engine ISRs):**
- `loop()` — drains the pulser ring → RPM/advance/dwell (`telemetry::tick`),
  polls modules, pushes WS frames (time-gated), runs `safety::tick` (100 ms).
- **GPIO ISRs:** `isrCh1` (GPIO34 CHANGE), `isrCh2` (GPIO35 CHANGE),
  `isrTrigger` (GPIO14 FALLING, quickshifter).
- **HW-timer ISRs:** `isrFireOn` (start dwell / coil charge) and `isrFireOff`
  (end dwell → **spark fires** as the coil collapses).

**Core 0 — background (must tolerate 10–50 ms flash stalls):**
- WiFi / AsyncTCP / lwIP framework tasks.
- `cdi_persist` task (`config_store.cpp`) — NVS writes, off the spark core.
- `cdi_snap` task (`edge_snapshot.cpp`) — LittleFS scope-snapshot writes.

**Spark fire path (all `IRAM_ATTR`):**
```
isrCh1 (GPIO34) ─▶ spark::onPulseCh1FromIsr
                      ├─ quickshift::shouldCut()        (cut window?)
                      ├─ safety::shouldFire()           (ALVP / rumble / flame / launch / cut-mode)
                      │     ├─ idle_rumble::shouldFireThisCycle()
                      │     ├─ flame::shouldFireThisCycle()
                      │     └─ launch::isActive()
                      ├─ crank-assist branch (rpm < CRANK_MODE_RPM → fire off CH2)
                      └─ arm fire-on timer at (CH1 + scheduler_delay)
isrFireOn  (timer)  ─▶ GPIO25 HIGH  (coil charging, dwell starts)
isrFireOff (timer)  ─▶ GPIO25 LOW   (SPARK)
```
`telemetry::live_stats::tick()` (core-1 loop) computes the per-cycle delay/dwell
and publishes them to the scheduler via `setNextDelayUs` / `setEffectiveDwellUs`
/ crank-assist arming. The ISRs only consume those published values.

### Repo map
```
include/        types.h · config.h · pinmap.h · ui_pages.h (GENERATED)
src/main.cpp    boot bring-up + loop pump (see boot order below)
src/core/       engine logic (see table)
src/net/        wifi_ap · http_server · ws_server · ota
src/scope/      edge_capture (live WS scope) · edge_snapshot (LittleFS capture)
src/storage/    config_store (NVS JSON, debounced ~1.5 s, core-0 task)
src/telemetry/  live_stats (advance/dwell compute + WS snapshot) · datalog (RAM ring)
src/util/       ring_buffer.h (lock-free SPSC, ISR↔loop)
web/            SPA source (index/map/scope/settings .html, app.js, style.css)
scripts/        build_ui.py (pre-build) · auto_uploadfs.py (post-upload)
docs/           dual-edge-ignition.md · 2-stroke-support-research.md
```

| `src/core/` module | namespace | purpose |
|---|---|---|
| `spark_scheduler` | `cdi::core::spark` | µs ignition timing: CH1+delay → fire-on/off HW timers → GPIO25 |
| `pulser_input` | `cdi::core::pulser` | CH1/CH2 edge ISRs; SPSC rings (ignition + scope) |
| `rpm_calc` | `cdi::core::rpm` | RPM = 60e6/period, EMA smoothing, stale-decay |
| `advance_map` | `cdi::core::advance` | RPM→deg BTDC interp table (portMUX-guarded) |
| `dwell_curve` | `cdi::core::dwell` | RPM→dwell µs table (portMUX-guarded) |
| `safety` | `cdi::core::safety` | rev-limit/overrev cut, cut-modes, `flashWriteSafe`, WDT |
| `mode` | `cdi::core::mode` | IGNITION/SAFE_HOLD/BOOT state machine (mutex-serialized) |
| `pickup` / `pickup_cal` | `cdi::core::pickup[_cal]` | active geometry + magnet-width auto-cal |
| `alvp` | `cdi::core::alvp` | battery derate/disarm (ADC GPIO32) |
| `launch_control` | `cdi::core::launch` | 2-step launch limiter (GPIO13) |
| `quickshifter` | `cdi::core::quickshift` | clutchless cut (GPIO14 ISR) |
| `backfire` / `exhaust_flame` / `idle_rumble` | `cdi::core::{backfire,flame,idle_rumble}` | sound/flame features (retard + skip) |
| `panic_button` | `cdi::core::panic` | GPIO0 hardware emergency kill |
| `shift_light` | `cdi::core::shift_light` | GPIO27 RPM indicator |
| `engine_preset` | `cdi::core::preset` | 30-bike preset library |

### Boot order (`main.cpp::setup()`)
1. GPIO25/26 forced LOW (spark off) **first**. 2. Serial. 3. OTA partition
diag. 4. LittleFS mount + snapshot init. 5. default advance map. 6. WiFi AP +
HTTP + OTA routes. 7. **engine modules** (spark/safety/shift/dwell/launch/QS/
backfire/rumble/flame/alvp/panic, `preset::apply`) — spark still DISARMED.
8. `config_store::begin/load` (NVS). 9. `mode::begin` → IGNITION. 10.
`ws_server::begin` (deferred until after engine modules so a reconnecting phone
can't arm before timers exist). 11. auto-arm if enabled → **spark LIVE**.

---

## Hardware (`include/pinmap.h`)

| GPIO | Name | Function |
|---|---|---|
| 0 | BOOT_BTN | panic kill (hold ≥2 s → SAFE_HOLD) |
| 2 | STATUS_LED | HIGH when a WS client is connected |
| 13 | LAUNCH_INPUT | clutch switch (INPUT_PULLUP, active LOW) |
| 14 | QUICKSHIFTER | shift sensor (INPUT_PULLUP, ISR FALLING) |
| 25 | **SPARK_OUT** | TCI coil drive; HIGH = dwell, falling edge = spark |
| 26 | MODE_LED | HIGH during dwell (visible spark indicator) |
| 27 | SHIFT_LIGHT | RPM indicator |
| 32 | VBAT_SENSE | battery ADC1_CH4, 1:4 divider |
| 34 | **PULSER_CH1** | magnet leading edge (input-only, ext 10k pull-up, opto pulls LOW) |
| 35 | **PULSER_CH2** | magnet trailing edge (input-only) |

**Pulser geometry / key config (`include/config.h`):** `MAGNET_ANGULAR_WIDTH_DEG
=18`, `MAX_ADVANCE_FROM_CH1_DEG=32`, `BASE_ADVANCE_FROM_CH2_DEG=14`,
`RPM_MIN_VALID=30`, `RPM_MAX_VALID=13000` (→ max measurable ≈13001 rpm),
`ADVANCE_MIN/MAX_DEG=0/45`, `DEFAULT_DWELL_US=5000`, `DEFAULT_REV_LIMIT_MAIN/
OVERREV=10500/11500`, `CRANK_MODE_RPM=700` (below idle — crank-assist fires off
CH2 only during true cranking), `TASK_WDT_TIMEOUT_S=5`, `ALVP_DERATE/DISARM=
10.5/9.0 V`. ADC: `ADC_VREF_V=3.55` (DB_11 full-scale), divider 4.0.

---

## CRITICAL conventions & invariants

> These are the rules an audit found violations of. Breaking them causes
> crashes mid-fire, wrong-angle sparks, or a swallowed safety kill. Follow them.

### 1. IRAM rule — ISR-reached code must be `IRAM_ATTR`
arduino-esp32 2.0.17 here does **not** set `CONFIG_ARDUINO_ISR_IRAM`. During an
NVS/LittleFS/OTA **flash erase/program**, the instruction cache is disabled and
the core stalls; any **non-IRAM function reached from an ISR faults** (cache
miss) → crash mid-rotation. Every function on the spark fire path is
`IRAM_ATTR`: `isrCh1/isrCh2`, `onPulseCh1/Ch2FromIsr`, `isrFireOn/Off`,
`sparkActive/Idle`, `safety::shouldFire`, `idle_rumble::shouldFireThisCycle`,
`flame::shouldFireThisCycle`, `launch::isActive`, `quickshift::shouldCut`,
`rpm::current` (read by the QS ISR). **If you add anything reachable from an
ISR, mark it (and its callees) `IRAM_ATTR`** and use only integer ops (no FPU,
no `Serial`, no flash strings).

### 2. ISR-safe time source
In this build **`millis()` and `micros()` are NON-IRAM** (flash wrappers). The
IRAM-safe monotonic clock is **`esp_timer_get_time()`** (IDF, `IRAM_ATTR`). When
you need a timestamp inside an IRAM ISR, use `(uint32_t)esp_timer_get_time()` —
truncate to 32 bits (a 64-bit divide pulls in a flash libgcc routine!) and
compare with wrap-safe signed differences `(int32_t)(now - then) < 0`. See
`quickshifter.cpp::qsMicros()`. (The pulser ISR's `micros()` is a known residual
exception; do not add new `millis()/micros()` calls to ISR paths.)

### 3. `volatile` for cross-core shared state
Any variable written on one core and read on another (or read in an ISR) must be
`volatile` (documented in `safety.cpp` and `alvp.cpp`). Word-aligned ≤32-bit
loads/stores are atomic on Xtensa, so `volatile` is sufficient for scalars — it
just stops the compiler caching a stale value past a function boundary. This is
a hard convention; non-`volatile` cross-core state is a bug even if it "works
today".

### 4. `portMUX` for the advance/dwell curves
`advance_map` and `dwell_curve` are read every fire (core 1) and rewritten from
WS handlers (core 0). Pattern: **build+sort into a local buffer OUTSIDE the
lock**, then publish under `portENTER_CRITICAL(&s_mux)` (copy only), and have
`lookup()` snapshot under the same lock. **Never** hold a `portMUX` across an
allocation, a blocking call, or a JSON build (release, then build).

### 5. `flashWriteSafe()` gates every flash write
NVS (`config_store`) and LittleFS (`edge_snapshot`, snapshot delete) writes, and
WiFi-credential writes, must be gated by `safety::flashWriteSafe()` (true only
when disarmed, or rpm 0 and no CH1 edge for >600 ms). A flash erase mid-dwell
strands GPIO25 HIGH. WS handlers that write flash reject with "matikan mesin
dulu" while the engine turns.

### 6. Market-CDI behavior
`IGNITION` mode auto-arms (`mode::enterIgnition` → `spark::setArmed(true)`).
Faults pulse-cut via `shouldFire()` returning false and **never touch `armed`**
(self-recovering). Over-rev hard-cut engages on the **first** instantaneous
sample above the ceiling (fast backstop; the progressive main-limiter bites
below it). Sticky `SAFE_HOLD` only from panic button or multi-tooth detect.
`mode::set()` is serialized by a FreeRTOS mutex so a UI arm can't race-swallow a
panic kill.

### Never do
1. Call NVS/LittleFS writes from an ISR.
2. Hold `portMUX` across malloc/free/blocking/JSON-build, or take a FreeRTOS
   mutex inside a `portENTER_CRITICAL` section.
3. Write `s_activeCutMode`/`s_activeRetardDeg`/`s_progressivePct` from anywhere
   but `safety::tick()` (single-writer; ISR reads them lock-free, mode published
   LAST after its companion fields).
4. Add a `millis()/micros()` (or FPU, or `Serial`) call to an IRAM/ISR path.
5. Hand-edit `include/ui_pages.h` (regenerated by `build_ui.py`).

---

## Web UI + WebSocket

- **Pipeline:** `web/*` → `build_ui.py` (gzip + PROGMEM) → `include/ui_pages.h`,
  served by `http_server` with `Content-Encoding: gzip`. Rebuild firmware after
  any `web/` edit. Pages: `index` (dashboard ON/KILL + live telemetry), `map`
  (draggable advance-map editor, 4 localStorage slots P1–P4, landscape-forced,
  draggable rev-limit lines), `scope` (CH1/CH2 edge scope + snapshots),
  `settings` (preset picker, calibration, WiFi, rev limits, dwell, features).
- **WS endpoint `/ws` (binary out, JSON-text in).** Magic bytes:
  `0xB0` telemetry (≈73-byte frame, 5 Hz), `0xA7` edge-event stream (~33 Hz),
  `0xB1` fire event (reserved). `app.js` decodes and drives an event bus.
- **Main JSON commands:** `setMode`/`setArmed`/`testFire` (bench-only, refused
  while armed), `getMap`/`setMap`/`loadDefaultMap`, `setPreset`/`getPresetList`
  (broadcasts `preset`+`map`), `getPickup`/`setPickup`/`resetPickup`,
  `startCal`/`stopCal`/`getCalStatus`/`applyCal`, `setRevLimit`, `setCutMode`,
  `setDwellCurve`, `setBackfire`/`setIdleRumble`/`setFlame`, `setLaunch`,
  `setQuickshifter`, `setShiftLight`, `setAlvp`, `setCrankAssist`,
  `setWifiSsid`/`setWifiPassword`/`resetWifi` (flash-gated),
  `saveSnap`/`listSnaps`/`loadSnap`/`deleteSnap`, datalog cmds, `reboot`.

---

## Persistence
- **NVS (`config_store`):** all runtime params as a debounced JSON blob
  (~1.5 s after a change), written on the core-0 `cdi_persist` task, gated by
  `flashWriteSafe()`. Survives firmware upload.
- **LittleFS:** scope snapshots under `/snap/` (`edge_snapshot`). Wiped by
  `uploadfs`. Auto-formats on first boot.

---

## Contributing
- Work on a feature branch (e.g. `fix/...`), not `main`. The audit work lives on
  `fix/timing-path-audit`.
- After editing safety-critical/ISR code: `pio run` must succeed, then flash +
  serial boot-check (no `guru`/`backtrace`/exception, `setup() complete`), and
  ideally adversarially re-verify the change.
- End commit messages with:
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- Persistent notes for Claude sessions live in
  `.claude/projects/.../memory/` (`core-audit-2026-05.md`,
  `cdi-megapro-safety-state.md`). `.temp/` is gitignored scratch.

## See also
- `README.md` — wiring/circuit topology, first-boot walkthrough, full safety
  section, license/no-warranty.
- `docs/dual-edge-ignition.md` — CH2 crank-assist (Tier-1, opt-in) + EMI gate
  design and validation status.
- `docs/2-stroke-support-research.md` — future 2-stroke compatibility research.
