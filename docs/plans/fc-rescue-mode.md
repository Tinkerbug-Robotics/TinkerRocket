# FC rescue mode: update a flight computer that cannot finish booting

**Reported 2026-10-16.** A mounted V9 sat on "Starting sensors" after a battery
power cycle that followed an FC OTA. The app showed the boot step and nothing
more: iOS put a full-sheet "Initializing…" overlay over Settings, Firmware
included, so the image could not be replaced over the air. With the rocket
assembled, USB means taking the airframe apart, so OTA is the only practical
way back.

Two separate gaps, fixed separately.

## 1. The app hid the update

**iOS.** `SettingsView` laid `initializingOverlay` over the whole sheet whenever
`telemetry.state == "INITIALIZATION"`. That is what the OC's zeroed
`rocket_state` reads until the FC sends its first `NonSensorData`, i.e. for all
of `setup_fc()`, forever if setup never finishes, and often with the rail off
too, so the overlay also covered the rocket-off view and its firmware row.

**Android.** The firmware screen is a dashboard tool and was never gated, but
`SettingsScreen` returned early with an "Initializing…" notice, which hid the
out computer's own radio and network settings with everything else.

**Rule now** (`SettingsAccess` on both platforms, one shared case table in
`tests_cpp/fixtures/app_behavior/settings_access.json`):

| Connected rocket | FC settings | OC settings, firmware, app-only fields |
|---|---|---|
| FC running | yes | yes |
| Rail off | locked | yes |
| Rail on, FC not running (booting, stuck, silent, rescue mode) | locked | yes |

- *FC settings*: calibrations, IMU mounting and rate, camera, servo control,
  PID, roll and guidance, fin layout, pyro channels and tests. Edits made while
  the FC is not running queue in the OC (#366) and land late, out of order or
  not at all, so these stay locked.
- *OC settings*: LoRa telemetry on/off (cmd 68) and the network ID (cmd 41)
  live in the OC's NVS and never touch the FC.
- *Firmware*: the OC flashes itself; an FC relay is offered too, and the update
  says whether the FC answered. When OTA_BEGIN gets no answer the failure now
  names the flight computer and the way out, instead of "Device did not accept
  OTA_BEGIN within 20s".
- *App-only*: the descent profile (and Android's notes) never leave the app.

The notice that replaces the FC settings names what the FC is doing (the boot
step, or rescue mode) and points at the update. Android's notice links to the
dashboard's Firmware tool.

## 2. The FC could not take an update until setup finished

An FC image arrives app → BLE → OC → I2C `OTA_BEGIN_PENDING` → FC erases its
slot and answers READY over I2S → the I2S link reverses and the OC pumps the
image → `OTA_FINISH_CMD` → verify, `ready_to_boot`, restart. The FC polls the OC
for those commands only from `loop_fc()`. During `setup_fc()` the links are up
(I2C master, I2S sender, OTA parser task all exist before `FCB_NVS`) but nobody
polls, so a setup that stalls or crash-loops can never be updated.

App rollback covers part of it: a new image marks itself valid only after 10 s
of `loop_fc()` with the OC link proven (#1123), and a reset before that boots
the previous image. It misses an image that validated once and later stops
booting (a cold boot that differs from the warm one, as on the bench), and an
image that stalls without resetting never triggers it at all.

### Options considered

**Serve the relay during `setup_fc()`**, from a task polling the OC while setup
runs. Rejected:

- It would poll during every normal boot. The OC serves its command queue in
  order, so the boot-time listener would consume the commands queued for the
  booting FC (the connect-time sync that #366 and #1105 deliberately deliver
  across a power-on), changing every normal boot to fix the abnormal one.
- It runs the update concurrently with whatever setup is doing, in the boot
  path of a flight computer.
- It does nothing for a crash loop: each boot's window is shorter than an
  update.

**Rescue mode plus a setup watchdog** (chosen). One isolated path covers both
stalls and crash loops, and a normal boot is unchanged apart from two
bookkeeping writes to RTC memory and a one-shot timer.

### Design (`flight_computer/main/fc_rescue_policy.h`, host-tested)

- **Setup watchdog.** An `esp_timer` armed right after the pyro pins are made
  safe. If `setup_fc()` is still running after **120 s** (a healthy boot is
  25 s; the longest legitimate one, a dead GNSS module at its ~35 s deadline,
  is about 50 s), the FC records the stall and restarts. On a not-yet-validated
  image that restart is also what lets the bootloader roll it back.
- **Counting.** A record in RTC memory (survives panic, watchdog and software
  resets, not a power-on) counts consecutive boots that never ran `loop_fc()`
  for **60 s** (longer than an FC update takes end to end). It belongs to one
  image (the running partition), so a rollback or an update starts fresh, and
  an installed update clears it before restarting.
- **Candidate.** The next boot is a rescue candidate when the previous one
  stalled, or after **3** unproven boots in a row (one panic on the pad must
  not take the vehicle out of its flight software).
- **Never at the cost of a flight.** A normal boot can carry on a flight only
  if the OC serves it a valid in-flight snapshot (#846, #1176); without one it
  starts cold. So a candidate asks the OC first, with the same snapshot ask the
  normal boot uses (extracted to `fcAskOcForFlightSnapshot()`, unchanged), and
  enters rescue mode only when the OC answers a status query and hands back no
  flight. A silent OC or a flight in progress means a normal boot, which is
  counted, so the board keeps trying.
- **Rescue mode.** `setup_fc()` returns right after the links and the OTA
  receiver, and `app_main` starts `fcRescueTask` instead of the flight task.
  It polls the OC exactly as `loop_fc()` does, pushes `FC_IDENTITY` so the app
  can check images and read the version back, and serves `OTA_BEGIN_PENDING`,
  `OTA_FINISH_CMD` and `OTA_ABORT_CMD` through the same handlers `loop_fc()`
  now calls (extracted, unchanged). Every other command is dropped and logged.
  No sensors, GNSS, servos, camera, state machine, or pyro code run, so nothing
  can arm or fire. It parks the camera pins safe, reconciles an inherited power
  hold (the OC is alive and holds no flight, so the hold is released), and
  reports `FCB_RESCUE` (64) every 500 ms, which the OC forwards as `"bs"`.
- **Leaving.** An update restarts into the new image with the counts cleared. A
  power cycle (power-on, brownout, reset pin, USB) always tries a normal boot.

Timeline for the reported case on rescue-capable firmware: stall → about 2 min
→ restart → rescue mode in a few seconds → the app shows "Rescue mode — ready
for a firmware update" → flash a working image.

### What this does not fix

- **The board that is stuck today.** Its firmware predates rescue mode and
  nothing here reaches it. Rollback does not apply either if that image already
  validated. It needs USB once; after that, an image with rescue mode can be
  rescued over the air.
- A stall before the links come up (I2C or I2S init), or a broken OC link: no
  update path exists then in any mode.
- A hang with interrupts disabled is the interrupt watchdog's (it resets, and
  the reset counts).

## Verification

- `tests_cpp/test_fc_rescue_policy.cpp`: which boots count, what clears them,
  power cycles, rollbacks, the flight gate, and that rescue mode serves only
  the three OTA commands and none of the one-shot actuating ones.
- iOS `SettingsAccessTests`, `OTABeginFailureTests`, `FcBootProgressTests`;
  Android `SettingsAccessTest`, `OtaBeginFailureTest`, `TelemetryDataTest`.
- **Bench, not yet run.** Build with `-DTR_TEST_SETUP_FAIL=1` (stall) or `=2`
  (panic) and flash over USB (an OTA'd test image rolls back on its first
  restart instead). Expect: stuck on "Starting sensors" for 120 s, a restart,
  `[RESCUE] RESCUE MODE` on the console and "Rescue mode" in the app, then an
  FC update from the app completes and the normal image boots and validates.
  With `=2`, rescue mode after the third panic. Also check a normal boot logs
  no `[RESCUE]` lines beyond "boot proven" a minute into the flight loop.
