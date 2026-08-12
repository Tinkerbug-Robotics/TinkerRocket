# Android screen sweep — 2026-08-10

# BENCH PLAN — Android app vs TR-R-e424 (USB-powered, BLE)

## 0. Before you connect

Do these first; several of them change what the rest of the plan is safe to do.

1. **Confirm FC firmware is at or after `52e8db5` (2026-07-30).** Before that commit, simulated flights **energize pyro channels for real** at apogee. `git log --oneline 52e8db5`. If you cannot confirm, disconnect the charges and skip §5.3 entirely.
2. **Turn servo control OFF** if you want a fin-free session: Settings → "Servo control" toggle (`SettingsScreen.kt:192-194` → cmd 14 → `servo_en` NVS, `flight_computer/main.cpp:4773-4785`). Fins physically deflect through a whole simulated flight and neither app warns about it.
3. **Lock phone rotation.** `AndroidManifest.xml:21-23` declares MainActivity with no `configChanges` and no `screenOrientation`; every nav flag is plain `remember`, so a rotation recreates the Activity and drops you out of whatever screen you are on — mid-calibration included.
4. **Do not use the system back gesture anywhere in this app.** There is no `BackHandler` and no NavHost in the whole codebase — the only hit is a comment at `MainActivity.kt:144`. Back falls through to `ComponentActivity`'s default `finish()`. Use only the on-screen `←` buttons.
5. **The top-bar chevron is DISCONNECT while you are on tab 0** — `ConnectedTopBar.kt:58`: `if (tab != 0) onTab(0) else onDisconnect()`. One tap, no confirmation.
6. Set phone media volume during a voice test phrase, not between callouts (`TtsSpeaker.kt:24-26, :117` — audio focus is per-utterance).
7. Have a serial console on the FC. Several of the failure modes below are only visible there.

---

## 1. SAFE — offline, nothing connected

### 1.1 Saved Flights — **SAFE**
`SavedFlightsScreen.kt`. No `DeviceSession` reference at all; only reachable from the disconnected scanner (`MainActivity.kt:210`).

- Tap **Saved Flights** from the scanner. Tap **Chart** on a row. Tap **Share** on a row (opens the OS share sheet, CSV only).
- Look for: rows titled with the raw filename (`flight_20260729_163740`) and dated with **download time, not flight time** (`FlightCache.kt:82`); a size that disagrees with the Files screen for the same file (SI here, KiB there); no year in the date; no max-alt/max-speed on the card even though the `.json` sidecar is on the phone.
- **Do not** expect to find a base-station LoRa log here. They are never listed — see defect #5.
- Nothing to avoid. No delete exists anywhere on Android.

### 1.2 Flight Chart + Path (2D/3D) — **SAFE**
`FlightChartScreen.kt`, `FlightTrajectoryScreen.kt`, `Trajectory3DCanvas.kt`. Signature takes a `File` and no session; grep for `session.`/`Commands.`/`send` returns nothing.

- Taps: **Columns** → tick series → **Done**; pinch/drag/double-tap on the plot; **Reset**; **Path** → **● 2D** / **● 3D**; **← Files**.
- Look for: **Columns** has no "Apogee Flag (Master)", no "Apogee Detector: …", no "Pyro n Fired" rows (defect #14); tapping **Path** on a large log before the parse finishes gives a blank pane with no spinner and no error; the 2D track has no apogee marker and no basemap while the 3D one does; the trace bleeds left over the Y-axis labels at deep zoom; `← Files` says "Files" even when you came from Saved Flights.
- Nothing to avoid.

### 1.3 Device Manager ("My Devices") — **SAFE to view only**
`DeviceManagerScreen.kt`. Only reachable when nothing is connected (`MainActivity.kt:146`), so every edit here is **queued**, not sent.

- Taps for this phase: **My Devices** → tap the TR-R-e424 row → read the detail → **← Devices** → **← Back**.
- Look for: no `⚠` mismatch badge; the Rocket ID and network ID shown match what you expect.
- **DO NOT TAP YET** — deferred to §4: Rename…, the Rocket ID `−`/`+`, "Set to …", "Move all devices to ID n". **DO NOT TAP AT ALL** — "Forget This Device" (§6).

---

## 2. SAFE — connected, read-only

Connect to TR-R-e424 directly (not via the base station).

### 2.1 Voice announcer (top-bar speaker icon) — **SAFE**
Pure sink; no BLE call sites in `TtsSpeaker.kt`, `ConnectedTopBar.kt`, `FlightAnnouncer.kt`.

- Taps: **long-press** the speaker icon (speaks "Voice check. Announcements are enabled." without changing the enabled flag). Then single-tap to enable — it says "Voice ready" and goes green.
- Look for: callout **rate** — Android never calls `setSpeechRate`, so callouts run at whatever the phone's accessibility TTS rate is (defect #24). Set the volume rocker *while the phrase is playing*.
- **Do not** tap the chevron immediately left of it (that is disconnect).

### 2.2 Simulation screen — **SAFE to open**
`ToolsScreens.kt:146-267`. No `LaunchedEffect`, no `DisposableEffect`, no init. Opening and leaving send nothing.

- Taps: Tools → **Simulate**. Type into Mass g / Thrust N / Burn s / Descent m/s (per-keystroke writes to phone SharedPreferences only). **← Dashboard**.
- Look for: the full QWERTY keyboard on numeric fields; the button gate is only `connected && inputsValid && !launching` — no on-pad check.
- **DO NOT TAP "🔥 Launch Simulation" → "Launch" in this phase.** That is §5.3.
- Note: while you sit on this screen, phone-fix (cmd 47) pushes to a base station stop (`DashboardScreen.kt:75` early-returns past the pusher at `:88-109`).

### 2.3 Frequency Scan — **SAFE to open** (base-station link only)
`ToolsScreens.kt:335-447`. No effects; entry transmits nothing. Hidden on direct links (`DashboardScreen.kt:430-433`).

- Taps: Tools → **Freq scan**. Read Scan Range / Channels / Est. Duration. Edit the four fields (phone-local). **← Dashboard**.
- Look for: **Est. Duration reads ~1.7 s on defaults for a scan that takes tens of seconds** — the firmware runs 5 passes (`base_station/main.cpp:2690`) and the UI formula counts one.
- **DO NOT TAP "Start Scan" in this phase.** That is §4.2 and it is fleet-wide.

---

## 3. WRITES CONFIG — phone-side only, nothing reaches the rocket

### 3.1 Drift Cast — **WRITES CONFIG (phone/profile only)**
`DriftCastScreen.kt`. The composable takes no session (`:76`); nothing here reaches the radio.

- **The hazard is silent and undisclosed:** every parseable keystroke in **Drogue ft/s**, **Main ft/s**, **Main alt ft** rewrites `drogueRateFps` / `mainRateFps` / `mainDeployAltAglFt` in the **active rocket profile's JSON on disk** (`:130-156` → `RocketProfileStore.kt:81-87`). Mid-edit values commit — typing `1` on the way to `18` is written. iOS prints a caption saying so (`DriftCastView.swift:817-822`); Android prints nothing.
- **Write down the three current values before touching them.** There is no confirmation and no undo. These are the same values the live landing predictor and Settings → Recovery use. They are **not** pushed to the FC (`ActiveRocketSyncer.kt:242-316` has no recovery group).
- Safe taps: Set Launch / Set Landing, map taps, Use GPS, Launch/Landing lat-lon, Apogee ft, Max steer °, **Calculate guidance point** (one HTTPS GET to open-meteo; no BLE).
- Look for: on a fresh install the map may sit at the MapLibre default (whole world) and never re-centre — the only `moveCamera` is at `:177` inside the map factory; a failed recompute leaves the **previous** FEASIBLE card and 3D scene on screen next to the error, with the landing pin already moved; launch pad is **blue** on the map and **red** in the 3D card 200 lines below; there is no launch-time picker (winds are always current-hour); no wind-layer table; no "Send to Unit".

### 3.2 Device Manager → network name — **WRITES CONFIG (phone only)**
- Taps: **Change network name…** → type → **Save** (`DeviceManagerScreen.kt:279` → SharedPreferences `network_identity`).
- Look for: every known device instantly re-flags as mismatched (`⚠`) and the "Move all devices to ID n" button appears. **Nothing has been sent to any device yet.**
- **Do not** tap Save with an empty field — the dialog closes and silently does nothing (`:178-181`).

---

## 4. WRITES CONFIG — reaches rocket NVS

### 4.1 Device Manager identity edits — **WRITES CONFIG (deferred to next connect)**
These queue now and fire **automatically ~1 s after the edited device next connects**, with no further prompt: `DeviceSession.kt:346` (cmd 20 REQUEST_CONFIG) → `:554-561` → drains pending into cmd 40/41/42, each writing the rocket's `identity` NVS namespace.

- **Rename… → Save** → cmd 40. Firmware also calls `ble_app.setName()` (`out_computer/main.cpp:7563`) — **the rocket's BLE advertising name changes and it stops appearing as "TR-R-e424" in the scanner.** The field is unlimited but the value is silently clamped to 20 **bytes** (`KnownDeviceStore.kt:285`).
- **Rocket ID `−` / `+`** → cmd 42, NVS `rid`. This is the address the base station targets for relayed commands. An unset (0) ID becomes 1 on the first tap in either direction.
- **"Set to …" / "Move all devices to ID n"** → cmd 41, NVS `nid`. **No confirmation on either platform.** A rocket on a different `nid` than its base station goes deaf on LoRa. The firmware does no range check on cmd 41 (`out_computer/main.cpp:7569`).
- **Your abort button is "Cancel queued changes"** (`:458-460`) — app-side only, and it cannot undo anything already pushed.
- Recommended: queue **one** change, reconnect, confirm on the FC serial console, then proceed. Do not queue a rename and an ID change together on the first pass.

### 4.2 Frequency Scan → Start Scan — **WRITES CONFIG (fleet-wide, both ends)**
This is the single most under-labelled control in the app. Read all of this before tapping.

- One tap (`ToolsScreens.kt:390-406`, cmd 60) causes the base station to sweep and then **persist a new hop channel mask to its own NVS** (`base_station/main.cpp:2785-2795`) **and uplink cmd 15, which every rocket that hears it commits to its own NVS** (`out_computer/main.cpp:4098, 4135-4137`).
- **Both uplinks are addressed `target_rid = 0xFF` = broadcast to all rockets in the network** (`base_station/main.cpp:2877` and `:3130`). This is *not* scoped to TR-R-e424. **Power down every other TinkerRocket in range first.**
- In hopping mode it first sends cmd 16 HOP_PAUSE, parking the rocket off-hop for a window **capped at 60 s** (`RocketComputerTypes.h:252`). **Telemetry will be dead for up to a minute.** Backing out, disconnecting, or force-quitting the app does **not** shorten it; only a re-sent cmd 16 changes the deadline, and it only extends it.
- **Nothing on screen warns you of any of this before the tap** — the explanatory text at `:426-433` renders only *after* the config has been written.
- Expect the spinner to run 10–45 s, not the 1.7 s the screen predicts.
- **A rejected scan looks identical to a working one**: the three firmware rejection paths (`base_station/main.cpp:5147, :5152, :5156`) send nothing back; the app spins for 75 s then silently clears. **Do not tap Start Scan a second time** — the second tap is also rejected while the fleet is parked.
- Do not enter step or dwell above 65535: it truncates mod-65536 on the wire (`Commands.kt:575-579`); iOS clamps.

---

## 5. ACTUATES HARDWARE

### 5.1 Servo Test — **ACTUATES HARDWARE ON OPEN**
`ToolsScreens.kt:69-136`. **Merely opening this screen moves the servos and changes the flight computer's mode.** Clear hands and tools from the fin cans before you tap the tile.

- `LaunchedEffect(Unit) { send() }` at `:79` fires cmd 24 with all-zero angles the instant the screen composes. No connection, power, or state check runs first — the button at `DashboardScreen.kt:423-424` has no `enabled` condition at all (iOS gates it on READY/PRELAUNCH, `DashboardView.swift:2525`).
- On the FC this sets `servo_test_active`, and **the flight state machine and `servicePyroChannels()` are both suspended for as long as it is set** (`flight_computer/main.cpp:6459-6461` vs `:6466-6571`). The firmware refuses cmd 24 only in INFLIGHT and MAG_CALIBRATION — **LANDED is accepted.**
- There is **no timeout and no link-loss watchdog** on `servo_test_active`. The only clears are the explicit cmd 25 and the #363 launch-detect failsafe.
- Taps: the four sliders (0.5° steps; every step re-sends the whole 4-angle vector), **Center All** (sends `[0,0,0,0]` — **it does NOT exit servo-test mode**), **← Dashboard** (sends cmd 25; expect a second, larger anti-backlash movement and ~6 s of held servos).
- **Exit only via "← Dashboard."** Any other exit is unsafe or unproven:
  - System back — no `BackHandler`; it hits the Activity default. Whether that disposes the composition (and therefore sends cmd 25) is **not determined from source** — see defect #3.
  - Top-bar tab change — does dispose and does send cmd 25.
  - Disconnect — disposes, but the cmd-25 write cannot reach the rocket and **its failure is swallowed** (`DeviceSession.kt:848-850`, `:1065-1068`). **If the link drops with this screen open, treat the rocket as still in servo test** until you power-cycle the FC or reconnect and cycle the screen.
- Not reachable on a base-station link on Android (`DashboardScreen.kt:421-426`), though iOS offers it and the BS firmware relays it fine.
- **Related trap:** Settings → fin-layout **jog** (`SettingsScreen.kt:373-374`, `FinLayoutEditor.kt:209-213`) sends the same cmd 24. It has **no exit-time stop on either platform** — if you jog and leave Settings without tapping **Stop**, the FC stays in `servo_test_active` with the state machine suspended.

### 5.2 Magnetometer Calibration — **ACTUATES HARDWARE / WRITES CONFIG**
`MagCalScreen.kt`. Opening is safe (no command on entry). Everything after **Start Calibration** is not.

- **Start Calibration / Recalibrate** (cmd 50): caches the current NVS offsets, then **zeroes the IIS2MDC OFFSET registers — the magnetometer runs uncalibrated from this moment** — enters `MAG_CALIBRATION`, stows servos, and inhibits launch detection, EKF init, automatic pyro servicing and all ground-test commands (`flight_computer/main.cpp:4940-5011, 6957-6982`).
- **The firmware has no automatic exit from SAMPLING or REVIEW.** `main.cpp:6957-6959`: "No state-machine transitions out — driven entirely by BLE commands." **The only safe exit is the top-left "✕ Cancel"** (cmd 51, restores the prior offsets).
- **Four ways to strand the FC** (defect #2): system back (finishes the Activity — you leave the app entirely, connection pinned by the FGS), any top-bar tab icon, the top-bar chevron (disconnect), and any rotation / dark-mode / split-screen change. None of them sends abort. Recovery: reopen Tools → Mag cal (the FC republishes status at 5 Hz) and tap **✕ Cancel**.
- Buttons and what they cost:
  - **Compute Fit** (cmd 54) — RAM only, safe.
  - **Retry** (cmd 53) — safe, chip offsets stay zeroed.
  - **Verify** (cmd 52) — programs the proposed offsets into the chip **and opens a 60 s window that will PERSIST the cal to NVS on its own if the gates pass and you do nothing** (`main.cpp:163, 3843-3882`). Neither app mentions the timer. **Do not walk away from this screen during verify.** If rotation stopped, the opposite branch fires: coverage fails, the chip offsets are re-zeroed, and the FC drops back to REVIEW still inside MAG_CALIBRATION.
  - **Accept and Save** (cmd 56) — gated evaluation, then permanent NVS write.
  - **"Save anyway"** — **the same button, relabelled**, dispatching cmd 58 FORCE_APPLY, which skips every gate re-check and writes NVS unconditionally (`main.cpp:5159-5215`). **Read the label before you tap.**
  - **Watch out:** if you re-enter the screen mid-verify, the phone's local min/max are `null` again, so the button reads **"Save anyway"** and dispatches the *ungated* command — for a reason that has nothing to do with the calibration (defect #11).
  - **Abort (restore prior cal)** (cmd 51) — the safe way out.
- Look for: verify-failure text that says only "Verify: |B| too high" with no measured number (iOS prints the value and the remediation); the first second of Verifying showing three green "—" rows and two red 0/N counters.

### 5.3 Simulated flight — **ACTUATES HARDWARE, NO ABORT**
`ToolsScreens.kt:202-242`. Do this **last**. Confirm §0.1 and §0.2 first.

- **Android has no way to stop a running simulation.** `BleCommandId.SIM_STOP = 7` is declared and never sent by any app code; `markSimLaunched()` feeds a latch no composable reads. There is no sim banner and no stop control anywhere on the Android dashboard. iOS has both (`DashboardView.swift:646-657`). **Once you tap Launch, the rocket flies the whole synthetic trajectory.** Your only recovery is power-cycling the rail or picking up an iPhone.
- **Fins physically deflect** through boost and coast. Everything downstream of `SensorCollector` sees the sim as real data; the INFLIGHT branch reaches `setServoAngles` (`flight_computer/main.cpp:6798`) and `controlWithGainSchedule` (`:6856`) with **no `isSimActive()` guard**. Neither app's warning text mentions this — the dialog only promises the pyros stay cold.
- Pyros dry-fire **only** on FC firmware ≥ `52e8db5`; the ARM rail is held low (`main.cpp:1133, 1178`).
- Taps: **🔥 Launch Simulation** → dialog "Simulated flight" → **Launch**. Sequence is cmd 9 TIME_SYNC → cmd 5 SIM_CONFIG → (300 ms direct / 1000 ms BS) → cmd 6 SIM_START. Flight state is reset and re-armed; a log file is written.
- **Do not tap "← Dashboard" during the config→start delay** — the coroutine is scope-bound and dies, leaving the FC configured but never started, silently (defect #17).
- **Do not tap Launch twice.** There is no eligibility gate at all (`:245`) — no on-pad check, no already-simming check. A second Launch restarts the trajectory under a state machine still holding INFLIGHT.
- Note: an Android-driven sim can never reach the OC's cmd-7 log-close path (`out_computer/main.cpp:7030-7035`), so the log is only closed by the flight's natural end.

---

## 6. DESTRUCTIVE — DO NOT TAP

### Device Manager → "Forget This Device" → **"Forget Device"** — **DESTRUCTIVE**
`DeviceManagerScreen.kt:523-527` → `KnownDeviceStore.kt:355-358`. Deletes the whole known-device record — cached name, network/rocket ID, last-seen, **and any queued edits** — irreversibly, with no undo. The rocket keeps its own name and IDs (nothing is sent).

**Do not tap this during the bench session.** If you queued an identity edit you regret, use **"Cancel queued changes"** instead.

Also note the on-screen copy at `:476-477` ("it will be treated as new the next time it connects") is inherited iOS wording — Android has **no** provisioning sheet, so nothing prompts; the record silently re-materialises from the identity readback.

---

# DEFECT LIST

Ranked most severe first. Severity = consequence for the physical rocket and its stored state. REFUTED findings are dropped (Trajectory3D "BS logs render no trajectory" — superseded by #5; Simulation "confirm dialog claims iOS parity" — iOS has the same alert at `DashboardView.swift:2559-2570`). Where the same defect appeared on several screens I have merged it once.

## HIGH

**1. Servo Test has no entry gate and actuates on open; opening it suspends the FC state machine and pyro servicing.**
`ToolsScreens.kt:79` (+ `DashboardScreen.kt:423-424`), consequence at `flight_computer/main.cpp:6459-6461` vs `:6466-6571`.
Repro: run Tools → Simulate → Launch, let the sim land (LANDED), tap "Servo test" — the screen opens, cmd 24 goes out, FC enters `servo_test_active` with its state machine suspended until you tap "← Dashboard". iOS refuses to open the sheet.

**2. Four ways to leave Mag Cal mid-calibration without sending MAG_CAL_ABORT; the FC is stranded in MAG_CALIBRATION with the magnetometer offsets zeroed.**
`MagCalScreen.kt:100-117`; `MainActivity.kt:147, 151-158`; `ConnectedTopBar.kt:58-69`; `AndroidManifest.xml:21-23`.
Repro: Start Calibration, tumble, then tap the Map icon in the top bar (or rotate the phone). Dashboard banner reads MAG_CAL indefinitely; no "[MAGCAL] abort" on the FC console.

**3. No `BackHandler` exists anywhere in the app, so the system back gesture never runs any screen's exit path.**
Only hit is the comment at `MainActivity.kt:144`; nav is plain `remember` booleans (`DashboardScreen.kt:70-72`, `MainActivity.kt:198`, `SavedFlightsScreen.kt:51`).
Repro: from any pushed screen, use the system back gesture — the app closes to the launcher instead of popping.
**NEEDS A CHECK ON THE DEVICE:** on Servo Test specifically, it is not determinable from source whether the Activity default disposes the composition (firing the `DisposableEffect` at `ToolsScreens.kt:80-82` and sending cmd 25) or merely backgrounds the task, leaving the rocket in `servo_test_active` with no UI. **The settling observation:** open Servo Test, press system back, then check the FC console for the SERVO_TEST_STOP line and watch for the anti-backlash stow movement. If neither occurs, back is the worst exit in the app.

**4. Android can never send SIM_STOP — a launched simulation cannot be aborted from the phone.**
`BleCommandId.kt:37` declared, zero senders; `ToolsScreens.kt:232` sets a latch no composable reads; `DeviceSession.kt:255, 967-979`.
Repro: launch a sim from Android, scroll the whole dashboard — no banner, no stop control, while the FC flies the trajectory.

**5. Base-station LoRa logs cannot be downloaded into a usable form at all, and are invisible in Saved Flights.**
`FilesScreen.kt:570-586` has no `isBaseStation` branch; `CsvGenerator.kt:95` throws on ASCII input; `FlightCache.kt:63` filters BinaryCache to `.bin`. iOS branches at `BLEDevice.swift:1465` → `FileCache.swift:292-303`.
Repro: connect to the base station, download one of its own logs — status reads "Saved .bin; CSV failed", the row's Chart/Share stay greyed forever, and the log never appears in Saved Flights.

## MEDIUM

**6. Start Scan's channel-mask rewrite is a fleet-wide broadcast, not scoped to the connected rocket.**
`base_station/main.cpp:2877` and `:3130`, both `target_rid = 0xFF`; rockets persist at `out_computer/main.cpp:4135-4137`.
Repro: with a second TinkerRocket powered in range, tap Start Scan once — both rockets commit the new mask to NVS. No UI names this.

**7. A coordinated scan parks the rocket for up to 60 s and nothing in the app can shorten it.**
`base_station/main.cpp:3098-3131`, `out_computer/main.cpp:4148-4177`, cap `RocketComputerTypes.h:252`.
Repro: tap Start Scan on a hopping fleet, then back out / disconnect / force-quit — telemetry stays dead until the deadline expires on its own.

**8. Servo Test's exit-time SERVO_TEST_STOP is fire-and-forget and its failure is swallowed.**
`ToolsScreens.kt:81` → `DeviceSession.kt:848-850` (job discarded) → `:1065-1068` (`runCatching{}.isSuccess`, Boolean thrown away).
Repro: open Servo Test, pull power on the BLE link — the composable disposes, the write cannot land, nothing retries, and the FC keeps `servo_test_active`.

**9. Settings fin-layout jog puts the FC into the same `servo_test_active` mode with no exit-time stop (both platforms).**
`SettingsScreen.kt:373-374` + `FinLayoutEditor.kt:209-213`; no `DisposableEffect` in the file. iOS twin `FinLayoutView.swift:148, :208-210`.
Repro: Settings → fin layout → jog ±10° → leave Settings without tapping Stop. FC state machine stays suspended.

**10. Leaving Mag Cal during VERIFYING can commit a permanent NVS calibration unattended, or silently re-zero the chip offsets.**
`flight_computer/main.cpp:163, 3839-3910`. Neither app surfaces the 60 s timer.
Repro: tap Verify, tumble to pass the gates, then tap the Map icon. 60 s later the FC writes `mag_cal` NVS with no screen up to report it.

**11. Re-entering Mag Cal mid-VERIFY silently relabels the primary button and swaps the gated command for the ungated one.**
`MagCalScreen.kt:72-73, 336-342, 365-366`; FORCE_APPLY accepted at `flight_computer/main.cpp:5159-5215`.
Repro: during VERIFYING, switch tabs and come back — three rows show red because the phone forgot, and the only button offered is "Save anyway" (cmd 58, no gate re-check).

**12. Simulation has no launch-eligibility gate; a second Launch mid-sim is accepted.**
`ToolsScreens.kt:245` (`connected && inputsValid && !launching`) vs iOS `DashboardView.swift:2426-2434, 2461`. SIM_START handler has no lockout check (unlike GROUND_TEST_START at `flight_computer/main.cpp:4880-4884`).
Repro: launch a sim, reopen Simulate, tap Launch again — `startSim` runs again while the state machine still holds INFLIGHT.

**13. Neither app warns that fin servos physically actuate during a simulated flight.**
`ToolsScreens.kt:209-215, 257-265` scope the safety claim to pyro; the INFLIGHT branch has no `isSimActive()` guard (`flight_computer/main.cpp:6798, 6856`).
Repro: launch a sim with `servo_en` set and watch the fin cans deflect through the profile.

**14. Column picker cannot reach the apogee-detector, pyro or master-flag columns any modern log contains.**
`CsvParser.kt:69-72` group table + group-only rendering at `FlightChartScreen.kt:420-422`; writer emits them at `CsvGenerator.kt:540-564`. Shared with iOS (`CSVParser.swift:58-62`).
Repro: open any flight → Columns → the Flags section has three rows; `head -1` on the same CSV shows "Apogee Flag (Master)", "Pyro 1 Fired", etc. Fix is to **add** the new names, not swap the two legacy ones (they still match pre-May-2026 logs).

**15. Drift Cast rewrites the active rocket profile on every keystroke with no disclosure.**
`DriftCastScreen.kt:301-305` + `:129-156`; iOS prints the caption at `DriftCastView.swift:817-822`.
Repro: type in "Main alt ft" — `mainDeployAltAglFt` in the profile JSON is changed permanently, no confirmation, no undo, and the same value drives the live landing predictor.

**16. Mag Cal verify-failure messages drop the measured |B| and iOS's remediation text.**
`MagCalScreen.kt:453-465` (bare labels) vs `MagCalStatus.swift:122-148`. The value is in the frame Android already parsed (`MagCalScreen.kt:357`).
Repro: fail a verify near steel — Android says "Verify: |B| too high"; iOS says "…reached 84.3 µT (above 70 µT cap). Try moving away from interference."

**17. Backing out during the sim config→start delay leaves the rocket configured but never started.**
`ToolsScreens.kt:221-236` in a `rememberCoroutineScope()`; iOS uses `asyncAfter` (`SimulationView.swift:174-178`).
Repro: on a BS link (1 s window) tap Launch then immediately "← Dashboard" — BS log shows the sim config with no following sim start, and the rocket stays READY.

**18. "Est. Duration" understates the scan ~5× (shared with iOS).**
`ToolsScreens.kt:360` / `FrequencyScanView.swift:236` vs `LORA_NOISE_SCAN_PASSES = 5` (`base_station/main.cpp:2690`).
Repro: defaults show 1.7 s; the spinner runs 10–45 s.

**19. Tapping "Path" during the CSV parse gives a blank pane, and a parse error is invisible there.**
`FlightChartScreen.kt:182-192` returns before the spinner (`:200-207`) and error text (`:195-199`).
Repro: open the largest log and tap "Path" within the first second — empty space under the 2D/3D row, indistinguishable from a log with no position data.

**20. Downloads are keyed by bare filename with no size guard, so an identically-named log from a second device silently overwrites the first.**
`FilesScreen.kt:316-318, :576`; iOS guards with `isFlightCached(_:expectedSize:)` (`FileCache.swift:334-342`).
Repro: download `rocket_data_000.bin` from board A, then from board B — A's cached flight is gone, one row in Saved Flights.
**NEEDS A CHECK ON THE DEVICE:** effectively a legacy-firmware exposure only (timestamped names collide only within the same UTC second) and untested on hardware. **The settling observation:** flash legacy firmware on two boards, download the same sequential name from each, and compare the on-disk `.bin` sizes.

**21. Device Manager's "Forget Device" consequence copy is imported from iOS and is not true on Android.**
`DeviceManagerScreen.kt:476-477`; `markProvisioned` (`KnownDeviceStore.kt:271-274`) has no caller in `app/`; grep for "provision" in the Android app returns zero.
Repro: forget a device and reconnect — nothing prompts; the record silently re-materialises from the config_identity readback.

**22. System back exits the app from any Device Manager level (instance of #3, called out because it is the only two-level screen in the offline branch alongside Saved Flights).**
`DeviceManagerScreen.kt:73`, `MainActivity.kt:198`.

**23. A single TTS error latches the pre-flight voice indicator red for the whole app process.**
`TtsSpeaker.kt:68, :95` (never cleared) + tint priority at `ConnectedTopBar.kt:99-104`; iOS clears on every successful session call (`FlightAnnouncer.swift:468, :496`).
Repro: force-stop the system TTS engine, long-press the speaker — red. Let the engine respawn; speech works again, icon stays red. Only an app restart clears it.

**24. Callout speech rate is never pinned — in-flight callouts play at the phone's accessibility TTS rate.**
`TtsSpeaker.kt:72-104` (no `setSpeechRate`) vs `FlightAnnouncer.swift:382, :446`.
Repro: set the system TTS rate to 2×, long-press the speaker icon — the test phrase plays at 2×.

**25. Audio focus is per-utterance, so the volume rocker only sets callout loudness while a callout plays.**
`TtsSpeaker.kt:117` vs iOS's up-front `speech.activate()` (`FlightAnnouncer.swift:126-128, :237`). The `requestAudioFocus` return value is discarded.
Repro: enable voice, press volume between callouts — you are adjusting the ring stream.

**26. No staleness window on the TTS busy flag — a lost engine callback mutes the periodic altitude/descent callouts.**
`TtsSpeaker.kt:56, :108, :117-123` (speak() return discarded) vs iOS `busyStaleAfter = 15` (`FlightAnnouncer.swift:392-402`). Skippable path returns early at `FlightAnnouncer.kt:274-277`.
Repro: force-stop the TTS engine mid-callout — 5 s altitude and 10 s descent callouts stop; burnout/apogee/landed still speak (they bypass `isBusy`). Cleared by the next successful critical callout, or by toggling voice off/on.

**27. The top-bar chevron disconnects with no confirmation and is one tap from the voice toggle.**
`ConnectedTopBar.kt:58-64`, `MainActivity.kt:154-157`, teardown at `DeviceSession.kt:370-385` (fails in-flight downloads, clears cal status and the guidance echo).

**28. A log with no EKF fix draws a fake zero-extent track instead of an empty state.**
`Trajectory3DCanvas.kt:302-307` only rejects non-finite values, but positions are decoded from i32 centimetres (`NonSensorData.kt:28-31`) and can never be NaN. iOS rejects `lat == 0 || lon == 0` (`FlightTrajectoryView.swift:50`).
Repro: a USB-powered indoor run with no GNSS lock — the 2D view shows a launch dot and a landing dot on top of each other with a scale bar, and 3D shows a full grid with three markers at the origin and "0 m AGL".

**29. Android chart is permanently capped at 100 Hz, so deep zoom cannot resolve what iOS resolves.**
`FlightChartScreen.kt:79, :109-113` (`PREVIEW_SAMPLE_HZ = 100.0`) vs iOS full-rate `CSVParser.parse(url:)` (`FlightChartView.swift:352`). `ChartWindow.kt:99` still promises "true 1 kHz resolution at deep zoom".
Repro: zoom High-G Z on the boost phase on both phones — Android's markers stop getting denser at a 10 ms grid. Nothing read off the screen is *wrong*, just 10 ms coarse. Fix is a ledger entry + the stale doc comment, not restoring full-rate parsing.

**30. A failed Drift Cast recompute leaves the previous guidance result on screen next to the error.**
`DriftCastScreen.kt:318-346` never assigns `result = null`; iOS clears it before the Task (`DriftCastView.swift:1085-1087`).
Repro: compute a cast, move the landing pin, go airplane mode, recompute — the error sits above a still-green FEASIBLE card for the *old* landing site.

**31. Drift Cast map camera is set once at map construction and never again — no fit-to-result, and possibly a whole-world view on a fresh install.**
Only `moveCamera` in the file is `DriftCastScreen.kt:177`; iOS has `fitMapToResults` (`DriftCastView.swift:1107, 1118-1152`).
Repro (fit-to-result, unconditional): set a landing point 2 km downwind, compute — the guidance pin and descent track can sit entirely off the 300 dp map.
**NEEDS A CHECK ON THE DEVICE** for the fresh-install half: it is a race between the first fused GPS fix and map readiness, and MapLibre's default camera is not stated in source. **The settling observation:** clear app storage, open Drift Cast, and see whether the map comes up framed or at a world view — and confirm it never re-centres once the fields populate.

**32. Servo Test is unreachable on a base-station link on Android; iOS offers it and the BS firmware relays it.**
`DashboardScreen.kt:421-426` (a shared guard also covering "Mag cal") vs `DashboardView.swift:2509-2525` outside the `!isBaseStation` block; relay at `base_station/main.cpp:4787-4803`. The wire format is already correct, so the screen would work if unhidden.

**33. No local delete for downloaded flights anywhere on Android; iOS has per-flight and bulk delete.**
`SavedFlightsScreen.kt:108`; nothing in `app/` or `core/` deletes from BinaryCache/CSVCache. iOS `FlightLogsView.swift:91-117`, `FileCache.swift:394-407`.
Repro: multi-MB flights accumulate in `filesDir` with no in-app recourse; the only out is Settings → Clear storage, which also destroys rocket profiles (`AppContainer.kt:45-48`).

**34. A `.bin` whose CSV conversion failed is listed but cannot be shared, contradicting the module's own doc.**
`SavedFlightsScreen.kt:116` gates Share on `hasCsv`; `FlightCache.kt:58-61` says such a row "can be shared and re-converted"; `shareCsv` returns early without a CSV (`:88-89`). `file_paths.xml` already grants the whole `filesDir`, so this needs UI only.

**35. Saved Flights titles rows by raw filename and dates them by download time, disagreeing with Android's own Files screen.**
`SavedFlightsScreen.kt:102, :104`, `FlightCache.kt:51, :82` vs the ported `displayTitle` at `FilesScreen.kt:464-470` (ledger line 54). The correct logic already exists unused at `FlightFiles.kt:146-175`. If you fix the title, port the type badge too — the filename is currently the only thing distinguishing a `flight_` row from a `lora_` row.

## LOW

**36.** `← Files` label is hardcoded on the chart screen, which is also mounted from Saved Flights and from Trajectory — `FlightChartScreen.kt:170` (hosts `FilesScreen.kt:79`, `SavedFlightsScreen.kt:53`). Names the wrong screen on the offline path.
**37.** Servo Test caption reads "Range ±60°–60°" for every rocket — `ToolsScreens.kt:128-134` applies `abs()` to both endpoints after a `±`; fin travel is always symmetric (`RocketProfile.kt:118-119`), so there is no profile on which it reads sensibly.
**38.** Numeric fields open the full QWERTY keyboard app-wide — `ToolsScreens.kt:269-275` (`SimField`, used by 4 sim + 4 freq-scan fields), `DriftCastScreen.kt:400-409` (9 fields). iOS uses `.decimalPad`/`.numbersAndPunctuation`. On Drift Cast, one stray character greys Calculate with no message saying which field.
**39.** A blanked freq-scan field persists as an empty string and returns as an unscannable range — `ToolsScreens.kt:342-352`; survives app restart, shows "Channels 1857 (max 128)".
**40.** Step/dwell > 65535 truncate mod-65536 on the wire — `Commands.kt:575-579`, `ToolsScreens.kt:395`; iOS clamps (`FrequencyScanView.swift:109-110`). Reachable on both fields.
**41.** A rejected scan is indistinguishable from a lost result: 75 s of spinner then silence — `DeviceSession.kt:988-1002`, firmware rejections log only (`base_station/main.cpp:5145-5157`). Shared with iOS.
**42.** The 75 s scan wedge timeout is a fixed constant, so legal parameters (128 ch × 120 ms × 5) exceed it — `DeviceSession.kt:1116` vs `base_station/main.cpp:3104`.
**43.** Nothing warns before the tap that Start Scan writes NVS on both ends — the explanatory copy renders only after (`ToolsScreens.kt:408-433`). Shared with iOS.
**44.** Verify gate rows are binary green/red: three "—" placeholders render green before any sample, and 0/32 + 0/100 render red — `MagCalScreen.kt:358-362, 476-485` vs iOS's three-way (`MagCalView.swift:471-510`).
**45.** Android's profile import drops iOS's non-empty unit-ID guard — `ActiveRocketSyncer.kt:353-370` vs `MagCalView.swift:109-111`; can store a cal tagged with the empty string and raise a spurious BoardMismatch next connect.
**46.** Two risk labels in the original Mag Cal inventory were internally inconsistent (units menu marked writes-config though phone-only; "Re-run calibration" marked none though it re-zeroes the chip offsets) — `ConnectedTopBar.kt:123-150`, `MagCalScreen.kt:146-153`. Corrected in the plan above.
**47.** No apogee marker in the Android 2D ground track though the 3D view has one — `FlightTrajectoryScreen.kt:87-94` vs `Trajectory3DCanvas.kt:213-217` and iOS `FlightTrajectoryView.swift:125-131`.
**48.** No basemap under the post-flight track (2D canvas, 3D wireframe grid) — `Trajectory3DCanvas.kt:118-137` vs iOS `.hybrid` MKMapView + ArcGIS drape (`FlightTrajectoryView.swift:105, 281-315`). The Canvas-instead-of-SceneKit decision *is* recorded (`docs/plans/624-android-port.md:23`); the missing imagery is not, and the plan's decision 2 (`:160`) points the other way.
**49.** No stats bar under the Android trajectory (Max Alt / Duration / GPS Points) — `FlightChartScreen.kt:182-191` vs `FlightTrajectoryView.swift:705-737`.
**50.** Apogee AGL is measured from the track minimum, not from launch — `Trajectory3DCanvas.kt:208, :216` vs `FlightTrajectoryView.swift:447-448`. The whole ground plane and drop-line sit at the minimum too (`:116, 125-131, 221-224`), so fixing only the label leaves the picture disagreeing with the number. Metric-only is sanctioned by ledger line 50.
**51.** The 2D ground track has no gestures at all — `FlightTrajectoryScreen.kt:38`; it is the default mode, and iOS's is a live MKMapView.
**52.** No "Clear All" in the column picker — `FlightChartScreen.kt:410-446` vs `ColumnPickerView.swift:46-52`; each individual untick also resets zoom.
**53.** Zoomed trace draws outside the plot rect over the Y-axis labels — `FlightChartScreen.kt:368-390` (no clip) with the ±1 edge point from `ChartWindow.kt:113-114`. Bounded to one sample interval, visible at extreme zoom.
**54.** No X-axis label — `FlightChartScreen.kt:343-354` vs `.chartXAxisLabel("Time (s)")` (`FlightChartView.swift:210`). Separate from the sanctioned SI-units question. Neither platform labels Y.
**55.** Legend always renders on Android; iOS hides it for a single series — `FlightChartScreen.kt:232-247` vs `FlightChartView.swift:211`. Do not "fix" before #54 — the legend is currently the only thing naming the plotted column.
**56.** Picker rows show the raw header with units; iOS strips the suffix — `FlightChartScreen.kt:439` vs `ColumnPickerView.swift:32, 66-71`. Arguably better on Android while there is no axis label; record the decision.
**57.** Drift Cast's infeasible verdict is amber and reads "NOT FEASIBLE"; iOS is red and "INFEASIBLE" — `DriftCastScreen.kt:358-365` vs `DriftCastView.swift:888-897`. `docs/design-language.md:49` makes amber "degraded" — a no-go is not degraded.
**58.** Drift Cast 2D map and 3D card disagree about which colour is the pad, on one scrolling screen — `DriftCastScreen.kt:428-449` (launch blue) vs `Trajectory3DCanvas.kt:269-281` (launch red, matching iOS). Report as internal inconsistency; the map-vs-iOS half is plausibly inside the un-re-skinned sanction at ledger line 51.
**59.** Drift Cast has no launch-time picker — `DriftCastScreen.kt:323-326` hardcodes `System.currentTimeMillis()`; iOS defaults to the next hour and threads it through (`DriftCastView.swift:595-601, :1100`). Neither app *displays* which hour was used.
**60.** Drift Cast results drop the wind-layer table and ground elevation — `DriftCastScreen.kt:371` reduces the profile to a count; the Kotlin model carries the data (`DriftCast.kt:26-29`).
**61.** No "Send to Unit" on Android Drift Cast — `DriftCastScreen.kt:76` takes no session. The protocol and session layers are already ported and unit-tested (`Commands.kt:198-199`, `DeviceSession.kt:1009-1034`); only the UI is missing. cmd 28 is RAM-only on the FC in any case.
**62.** No basemap picker, offline-map access or offline pill on the Drift Cast map — `DriftCastScreen.kt:223-229, 451-467` hardcodes USGS_IMAGERY_TOPO; `MapScreen.kt:250-257` already has the control.
**63.** "Use GPS" writes whichever point is selected (iOS writes launch only), and Android does not auto-advance the tap mode — `DriftCastScreen.kt:264-273, 233-244` vs `DriftCastView.swift:1003-1013, 1031-1042`. Android's is arguably better; record it as a decision.
**64.** Drift Cast's profile seed at `:121-127` does not call `persist()`, so the prefs and the visible fields can disagree.
**65.** Phone-fix (cmd 47) pushes to the base station stop while any Tools screen is open — `DashboardScreen.kt:74-77` early-returns past the pusher at `:88-109`; iOS presents these as sheets and keeps pushing. Self-heals via `resetFixThrottle()` on return. Matters most on Freq Scan, where you legitimately sit for tens of seconds.
**66.** Network-name dialog accepts an empty name and closes silently — `DeviceManagerScreen.kt:178-181, :279`; iOS disables Save (`DeviceManagerView.swift:66-68`). App-side only, so a mistaken empty Save is a no-op.
**67.** Rename field has no input limit despite its "max 20 chars" label, and the real clamp is 20 **bytes** — `DeviceManagerScreen.kt:492-497` + `KnownDeviceStore.kt:284-294`. Shared with iOS.
**68.** Swipe-to-forget was not ported — `DeviceManagerScreen.kt:189-191` vs `DeviceManagerView.swift:141-145`, where it deletes with **no** confirmation. Recommend back-porting the confirmation to iOS rather than adding an unconfirmed swipe to Android.
**69.** `FleetManager.connectedDevice(unitId)` can never return non-null in production — `onIdentityReadback` (`FleetManager.kt:606-620`) has no production caller, so `FleetDevice.unitId` stays `""`. Device Manager routes around it, but a mis-typed `deviceType` also never gets corrected by the firmware's `dt`, which affects voice-announcer attach (`MainActivity.kt:119`) and relay framing (`FleetManager.kt:711`) for the whole session.
**70.** Queued startup TTS utterance is spoken after voice is turned off — `TtsSpeaker.kt:100-103, 110-115`; nothing clears `pending`. Android-only (iOS needs no async TTS init).
**71.** Utterance IDs are generated but never compared — `TtsSpeaker.kt:62, 80-98, 120-123`; a late terminal callback from a flushed utterance would clear `busy` and abandon audio focus mid-speech.
**NEEDS A CHECK ON THE DEVICE:** platform callback ordering around `QUEUE_FLUSH` is not determinable from source. **The settling observation:** trigger a burnout callout while a periodic altitude callout is mid-sentence and listen for the new phrase being ducked or overlapped.
**72.** `TextToSpeech` is never shut down — `AppContainer.kt:66-72` (no `shutdown()` anywhere). Deliberate app-scoped design, but it is why #23's latched error can only be cleared by killing the process.
**73.** Stale KDoc + dangling `@OptIn` in `DashboardScreen.kt:704-715` documenting the voice toggle that now lives at `ConnectedTopBar.kt:95-120`. Source-only.
**74.** Drift Cast file header at `:64-69` says "2D scope — the 3D trajectory view trails to Phase 9" while `:374-383` renders the 3D card. Source-only.
**75.** `ChartWindow.kt:99` still promises "true 1 kHz resolution at deep zoom", which #29 made false.
**76.** `Trajectory3D.kt:105-129` and `Trajectory3DCanvas.kt:41` claim "iOS-parity initial framing … 15° elevation"; iOS's camera is atan(0.4) ≈ 21.8° at 1.077× the slant range (`FlightTrajectoryView.swift:466-474`). Azimuth, 50° FOV and the 100 m extent floor **do** match.
**77.** Freq-scan result chart has no y-axis ticks or axis titles — `ToolsScreens.kt:454-492` vs `FrequencyScanView.swift:291-294`. Bars are plotted against the same implicit 0–100 scale, so geometry is comparable; the operator just cannot read an absolute dB margin. The summary-row colours are inside the un-re-skinned sanction.
**78.** Saved Flights sizes use SI while Files uses KiB for the same file — `SavedFlightsScreen.kt:122-126` vs `FilesScreen.kt:472-477`. (`FirmwareUpdateScreen.kt:305` is a third variant: KiB math under a lowercase `kB` label.)
**79.** Saved Flights date has no year — `SavedFlightsScreen.kt:130` (`"MMM d, HH:mm"`, hardcoded `Locale.US`). Compounds #33, since the list can only grow.
**80.** The `.json` flight summary sidecar is written and never read — `FilesScreen.kt:580` writes it, nothing reads it; iOS surfaces max alt/speed/burnout/apogee on the row and in the detail (`FlightLogsView.swift:176-198`, `FlightDetailView.swift:52-61`).
**81.** Ledger gaps to file, no code change: the EKF-vs-GPS trajectory coordinate source (`Trajectory3D.kt:19-23` vs `docs/android-parity-ledger.md:10-13`); the 100 Hz chart cap; the Servo Test BS guard; Drift Cast's missing send flow; Mag Cal (absent from both docs entirely).
**82.** Recorded so it is not re-filed later: iOS's "auto-selected channel" banner (`FrequencyScanView.swift:40-50`) is dead code — `hasAutoSelectedChannel` is never set true. Its absence on Android is correct, not a gap.
