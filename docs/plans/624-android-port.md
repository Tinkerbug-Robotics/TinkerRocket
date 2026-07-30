# TinkerRocket Android Port — Plan

*Status: IN PROGRESS — tracking issue [#624](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/624). **Phases 0–8 complete; Checkpoint A COMPLETE as of 2026-07-29** (matrix in `624-checkpoint-a-matrix.md` — every box passed or waived with a reason; the compass walk-around rides with the outing). Release signing is DONE (key minted, CI signs on `android-v*` tags, pipeline proven end-to-end 2026-07-30). Remaining for v1.0: the **shadow-phone field outing** (calendar-gated), now purely confirmatory. Phase 9 trailing parity is post-v1.0.*

Goal: a native Android app with feature parity to the iOS TinkerRocketApp, **maintained concurrently and permanently** alongside it. iOS is the behavioral reference; the firmware (`TR_RocketComputerTypes.h` + `TR_BLE_To_APP`) is the wire authority. One person maintains all of it, evenings/weekends.

This plan was produced from a full subsystem-by-subsystem read of the iOS app (~30k LOC Swift, 94 files), the firmware protocol surface, and the existing CI guards, followed by an adversarial gap review. Facts below about specific behaviors (command layouts, demux ladders, state machines) come from source, not memory.

---

## 1. Settled decisions

| Decision | Choice | Why (short) |
|---|---|---|
| Approach | Native Kotlin + Jetpack Compose; iOS untouched | Least risk to a working flight-critical app; the wire protocol is the contract |
| Location | **This monorepo**, `TinkerRocketAndroid/` | Every parity mechanism (shared fixtures, one checker over both apps, path-filtered CI, one PR per protocol change) depends on co-location |
| Modules | `:core:protocol` (pure JVM, zero Android deps) → `:core:session` (pure JVM, fleet/device domain over abstract transport) → `:core:ble` (Android GATT impl) → `:app` (Compose + platform glue) | Everything that can silently corrupt commands or flight data is JVM-testable on CI in ms, no emulator. The pure-JVM boundary *is* the future KMP option, exercised or not |
| KMP | Door open, not walked through | Adopting now = rewriting mature test-pinned Swift decoders for zero user value. Revisit on protocol-v2 rework, a third platform, or demonstrated drift despite the golden corpus |
| DI | Manual `AppContainer` + constructor injection (no Hilt/Koin) | ~10 app-scoped singletons don't justify KSP/AGP coupling for a permanent solo project. *(Supersedes a Hilt mention in an earlier phasing draft.)* |
| BLE | Raw `BluetoothGatt` behind a hand-rolled serial GATT op queue (~300 LOC), wrapped by a `BleTransport` interface | The app must own exactly the layer libraries abstract: one-outstanding-op scheduling, MTU-517-before-setup, one-write-per-callback OTA pacing, deliberate NON-pinning of connection priority (firmware owns conn-param policy, #519/#524), autoConnect endgame, wire-tap recording. Fallback if vendor quirks get ugly: Nordic's mature Java `android-ble-library` behind the same interface |
| Maps | MapLibre GL Native, raster-only USGS styles, **localhost read-through tile proxy** serving the byte-identical `OfflineTiles/<src>/<z>/<x>/<y>.tile` flat-file layout from `noBackupFilesDir` | Google Maps: ToS forbids tile caching (kills the offline feature) + key/billing at a no-signal field. MapLibre's own OfflineManager: evictable SQLite — the exact silent-data-loss iOS designed around. osmdroid is the named plan-B if the spike fails |
| Charts | One custom Compose Canvas chart (flight chart + freq scan), no Vico | Zoom/pan/LTTB windowing is already app-owned logic; a library's zoom model fights it |
| 3D | All three SceneKit views → 2D-projected Compose Canvas (fixed-camera mag-cal sphere; one shared orbit-camera trajectory component). No Filament | Scenes are static/fixed-camera; value is orientation + imagery, not shading. Removes "3D engine" from the risk register entirely |
| Serialization | kotlinx.serialization; **hand-written JsonObject mapper for telemetry**; stored JSON schema-identical to iOS Codable (Apple-epoch dates, uppercase UUIDs, omit-nil) | Telemetry leniency is asymmetric and must be replicated *exactly* (see §2.4). Schema parity keeps backup/restore and KMP doors open |
| minSdk | 31 (Android 12) | Collapses BLE permissions to `BLUETOOTH_SCAN`+`neverForLocation`/`BLUETOOTH_CONNECT`, no legacy location-for-scanning path. Install base is maintainer + club members |
| Distribution | Signed APK on GitHub Releases + trivial in-app update check (GitHub API → compare versionName → open browser) | New personal Play accounts require 20 testers × 14 days closed testing before production — absurd for this fleet. Nothing in the design is Play-incompatible if that ever changes |
| Devices | Pixel 8a (~$240 used) as reference, ordered at Phase 2 exit; Galaxy A15 5G (~$150 new) later as the floor/OEM-power-manager case | See Checkpoints A/B. A $15 USB BT dongle is bought first for Spike S1 |

---

## 2. Parity infrastructure — the backbone of "permanent"

This is Phase 0 and it lands **before any Kotlin decoder is written**, so the port is written against goldens, not prose. It also permanently upgrades iOS↔firmware parity, which today rests on hand-mirrored decoders and discipline.

### 2.1 Golden-vector corpus (`tests_cpp/fixtures/wire/`)

- A new host binary `wire_fixture_gen` in `tests_cpp/` constructs instances of the **real packed structs** from `RocketComputerTypes.h` and dumps byte-exact fixtures + `.expected.json` sidecars. The C compiler's layout IS the wire layout — no transcription step exists to get wrong. Extends the exact pattern `test_tr_flightlog_wire_format.cpp` already established.
- Families: log-frame payloads (NonSensor 43/44/48/50/65 ladder, FlightSettings 188/200/208/210/219, GNSS/IMU/baro/mag/power incl. legacy), framed streams (good/bad-CRC-resync/truncated-tail), file_ops frames (0xAA scan, 0xCA mag-cal 22/26/32/36, 0xCB, 0xCC, 0xCD — fixtures include the discriminator byte, resolving the 26-vs-27 ambiguity permanently in `manifest.json`), all ~20 command payloads (app→device fixtures are parsed *back* through the firmware structs to prove firmware reads what apps write), telemetry/config JSON incl. trim (`tr:1`) and flexInt torture cases, guidance-verdict replay scripts, and a small synthetic `.bin`→CSV+sidecar golden.
- A freshness gtest re-runs generation in-memory and asserts byte-equality with committed files — a header change that alters any layout fails C++ CI until fixtures regenerate.
- **Known hole + mitigation**: telemetry JSON is behavior (`TR_BLE_To_APP.cpp`), not a struct — the emitter can't produce it. Mitigation: host-compile the JSON builder into `wire_fixture_gen` if the extraction is tractable; otherwise a mandatory bench-capture regression step in the protocol checklist. Without this, the highest-churn message in the system has no freshness gate.
- Consumption: C++ gtest via `TR_WIRE_FIXTURES_DIR`; Swift via a blue folder reference into the test bundle + a manifest-walking `GoldenVectorTests.swift` (existing hand-written tests are KEPT — they encode intent); Kotlin via a Gradle `systemProperty` path, pure-JVM on `ubuntu-latest`. One fixture regeneration re-verifies every decoder in the fleet.

### 2.2 Wire-code checker extension (`tools/check_ble_command_ids.py`)

- Kotlin command IDs live in exactly one file (`BleCommandId.kt`, `const val` style, `_OC`/`_BS` suffixes making the cmd-50 dual-meaning explicit); the checker gains a `KOTLIN_CONST_RE` and a **hard-fail Swift↔Kotlin command-set diff** — the new failure mode this port introduces, unguarded today. `ALLOWED_DIVERGENCE` set seeded with everything, burned down as Android implements commands.
- Guard the guard: `SWIFT_RE` only matches integer literals at send sites; add a minimum-match-count assertion (fail if < ~30 sites) so a partial Swift refactor to named constants can't silently shrink the compared set, and note the constraint in `BLEDevice.swift`.
- Enforce app-numbers ⊆ OC-dispatch ∪ BS-dispatch ∪ {70,71,72} (OTA lives in `TR_BLE_To_APP.cpp`, invisible to the main.cpp grep — hard-coded trio with a comment).
- Phase-2 (post-stabilization): `--constants-parity` mode diffing `RocketComputerTypes.h` constants (SH_* shifts, flag masks, GUID codes) against Kotlin/Swift mirrors — **and firmware `config.h` defaults against both apps' RocketProfile defaults**. The syncer pushes defaults over FC NVS; a default typo in either app silently re-tunes rockets.

### 2.3 Behavior parity + the moving target

Wire bytes are not enough. The hand-ported twin logic (guidance-send verdicts, flight-readiness table, announcer policy, syncer state machine, staleness/power gates, relay targeting) can drift when a bugfix lands on one platform only. Two mechanisms:

- **Behavior fixtures** for the pinned state machines (guidance verdict replay scripts, readiness truth table, flexInt matrix) consumed by both XCTest and JUnit — same corpus mechanism as §2.1.
- **Parity ledger** (`docs/android-parity-ledger.md`): during the port, any change under `TinkerRocketApp/Models/` (or later, the Kotlin core dirs) must either touch the twin or add a dated ledger entry. This is also the answer to the moving-target problem — iOS development continues weekly during a year-long port; the ledger is the running divergence record that makes "parity at v1.0" a checkable claim instead of a feeling. A CI job flags PRs touching one side's core without a ledger entry or twin change (path-based heuristic; noisy is fine, it's a reminder not a gate).

### 2.4 Source-verified behavioral contracts the port must honor

- **Telemetry JSON leniency is asymmetric**: integer keys accept int/float/string (`flexInt`); float/string keys are STRICT — a float key emitted as a string throws and the whole frame is discarded. Replicate exactly; more lenient is a divergence too (#293/#571 shipped bugs in both directions).
- **Connect choreography matches iOS ordering**: time-sync (cmd 9) fires the instant the command characteristic is discovered — *before* the file_ops/file_transfer CCCD writes — then 1.0 s → cmd 20 + cmd 45 re-pin. Sim filenames depend on early clock sync; treat the sequence as a timing contract.
- MTU must complete before setup writes (cmds 26/65/34/12 don't fit MTU 23; no long-writes).
- Scanner: service-UUID filter only, never name (#547); advertised name from the scan record, not the stack's cached name.
- Everything little-endian; JVM ByteBuffer default is big-endian — every buffer gets an explicit `order()`; u32 fields in `Long`/`UInt` (time_us wraps signed Int at ~35.8 min = a pad-wait bug).
- Port the "wrong-looking" code verbatim where it's regression-pinned: SensorConverter's hardcoded FS constants, the RocketProfile decoder that deliberately ignores `finMinDeg`/`finMaxDeg` (#449 — mapping them resurrects the 2× fin bug), LandingPredictor's 0.5 threshold (the code, not the comment).

### 2.5 Protocol-change checklist + CI

- `docs/protocol-change-checklist.md`: the ordered touch sequence (header → gtest pins → firmware dispatches/JSON builder → regen fixtures → iOS → Android → checker → `Data_Analysis/*.py` sweep → bench), plus a change-type → surface matrix. `tools/preflight_protocol.sh` runs checker + cpp tests + both apps' unit tests locally.
- New `.github/workflows/android-tests.yml`: JVM-only (`:core:protocol` + `:core:session` tests, lint, assemble) on `ubuntu-latest`, triggered by `TinkerRocketAndroid/**` AND `tests_cpp/fixtures/wire/**`. No emulator job. `ios-tests.yml` gains the fixtures path; `wire-codes.yml` gains the Kotlin paths. Release CI signs and attaches the APK on tag push (keystore in repo secrets, **offline backup of the keystore is part of Phase 0 exit**).

---

## 3. Architecture notes (what makes the port correct, not just present)

- **Single-writer concurrency**: iOS's zero-lock correctness comes from CoreBluetooth delivering on the main queue. Android analog: one dedicated single-threaded fleet dispatcher owns ALL session/fleet state; GATT binder callbacks only post events into a Channel; notification `byte[]` copied immediately (vendor buffer reuse). Confined dispatcher, not `Main`, so the pipeline runs during service-only operation.
- **Fleet/session split preserved exactly**: new `DeviceSession` per connection (destroyed on disconnect — a dozen documented iOS bugs live in this split: #140/#290/#375/#377/#385/#390/#394); survivors on the fleet singleton (`lastValidRocketFixes` keyed `(nid,rid)`, bsFocus, OTA registry keyed by MAC, KnownDeviceStore). A **connection-generation counter** replaces iOS object-identity so attach edges fire on "same id, new connection".
- **Foreground service** (`foregroundServiceType="connectedDevice"`) started on first connect. Ordering constraint: `BLUETOOTH_CONNECT` must already be granted before FGS start (SecurityException on 14+) — first-run sequence is permission → connect → startForegroundService. OTA and cmd-8 power-on (30–90 s NAND flush) extend the must-stay-alive condition. `FLAG_KEEP_SCREEN_ON` while any device is connected (#385).
- **BT adapter lifecycle**: `BluetoothAdapter.ACTION_STATE_CHANGED` receiver → tear down sessions cleanly, requeue scanner on re-enable, FGS behavior defined. (iOS poweredOn/poweredOff handling has no free Android analog; a field BT cycle must not strand the fleet state machine.)
- **Reconnect endgame**: 8 attempts with `min(8, 2^(n-1))` backoff, then re-scan-then-autoConnect — never autoConnect blind from a MAC (fails on stacks without a cached address type; device must have been seen in a scan since boot).
- **Replay seam at the GATT-op level** (`BleTransport` in `:core:session`): every demux ladder, delay, gate, and state machine runs identically in real and replay modes. Three feeds: scripted FakeFirmware (answers cmd 20, echoes cmd 28, pages file lists, serves chunks, speaks OTA); timestamped wire-tap session logs recorded by `RealBleTransport` and replayed; telemetry synthesized from parsed `.bin` flight logs (e.g. the 7/16 HIL log "flown" live in the emulator).
- **Recomposition scoping**: pre-sliced `StateFlow`s per card (`.map{}.distinctUntilChanged()`), `@Immutable` telemetry, shared 1 Hz tick for staleness/age labels, mag-cal frames on a `SharedFlow` (conflated StateFlow may skip verify-accumulator edges).
- **Settings text fields**: local `TextFieldValue` + commit-on-focus-loss; never bound to hot flows (the Android analog of #361 — also protects against the syncer/echo clobbering mid-edit IME composition).
- **Data off the phone**: per-file share via FileProvider, **plus a bulk "export all flight data" flow to a SAF-picked tree/Downloads** — iOS Documents is Finder-visible and feeds the `Data_Analysis/*.py` workflow; Android `filesDir` is adb-only, so without bulk export the primary post-flight analysis path breaks.
- **Crash diagnostics for a sideloaded app**: uncaught-exception handler writing a local log with a share action (no Play Console feed exists in this distribution model).
- **Accessibility + back-handling minimum**: contentDescription on safety-critical controls (pyro tiles, power gate); predictive-back/gesture-dismiss and process death are explicit abort paths for PyroTestView's countdown (iOS relies on `onDisappear` for the #560 cmd-23 toggle discipline).
- **MSL altitude**: `Location.getMslAltitudeMeters()` on API 34+; below, decide between a bundled coarse EGM96 table vs showing ellipsoid with a caveat (open decision; the ~30 m geoid delta silently corrupts the rocket-vs-phone altitude readout otherwise).
- **Dependency policy**: pin everything; one scheduled upgrade window per year (AGP/Kotlin/Compose-BOM/MapLibre together), otherwise no bumps. Written down so it's a policy, not ambient drag.
- CompanionDeviceManager was evaluated and rejected for v1: per-device association UX fits poorly with a multi-device rotating fleet; revisit if OEM killing proves worse than the Field Setup screen can handle.

---

## 4. Phases

Estimating unit: 1 evening ≈ 2–3 h. Cadence ~3 sessions/week. Every phase ends compile-verified + committed at a hardware-free-verifiable seam where possible; bench seams are explicit PAUSE-for-go points.

| # | Phase | Scope (condensed) | Exit criteria | Evenings |
|---|---|---|---|---|
| 0 | Scaffold + parity infra | Gradle modules, CI, keystore + offline backup, `wire_fixture_gen` + freshness gtest + first fixture families, checker Kotlin extension, iOS golden CSV snapshot from the 7/16 HIL log | CI green on Kotlin module consuming C++-emitted fixtures; checker fails on a planted divergent ID | 6–9 |
| S1 | **Spike: emulator-BLE reality** ($15 USB dongle) | Emulator + Bumble dongle-bridge → scan/connect a real bench OC, MTU 517, CCCD, telemetry notifications, cmd 20 round-trip; netsim fallback evaluated | Documented verdict: (a) bridge works → phone waits till Phase 2 exit; (b) doesn't → FakeFirmware-only + Pixel ordered at Phase 3 start. Throughput recorded | 2–4 |
| 1 | `:core:protocol` | MessageParser (CRC16, resync/stop asymmetry), all SensorTypes ladders, SensorConverter verbatim, TelemetryData hand-serializer (asymmetric leniency), bitfields, CSV gen/parse + LTTB, FileCache logic, centralized command encoders | All golden vectors green; `bin→CSV` of HIL log semantically identical to checked-in iOS output (numeric equality with a documented rounding-tie allowlist; byte-identical only if Swift-matching formatters prove cheap); encoders byte-identical to fixtures | 11–15 |
| 2 | `:core:session` + `:core:ble` | Serial op queue, single-dispatcher marshaling, connect choreography (iOS ordering), scanner, reconnect ladder + autoConnect endgame, fleet/session split + generation counter, demux ladders, relay routing + sticky focus, KnownDeviceStore, FGS, guidance verdict machine verbatim + tests, wire-tap | JVM tests green (queue/demux/verdicts/reconnect against FakeFirmware); **bench seam**: emulator↔real OC + BS — telemetry decodes, config readback, relayed rocket in roster, delete-burst survives queue, reconnect after power-cycle | 16–22 |
| 3 | App shell + Dashboard slice | NavHost, AppContainer, attach coordinator, Dashboard core (entry state machine, #377 power gate, #382 display inversion, staleness overlay, health/GNSS/signal/storage, pyro tiles with cont-&&-live fail-safe, relay-aware controls), replay harness drives it; **emulator perf probe** (frame timing at replayed 10–20 Hz × 2 devices — pulled early per gap review) | Emulator demo: replay drives connect→telemetry→stale→power-off; no dropped frames at target rates on emulator profile | 10–14 |
| 4 | Downloads + files + logs | Download engine (arrival-order append, 3 s stall, EOF|ABORT, dup-EOF), file-list page flow (write→0.5 s→read), FilePageNavigator, FileCache on `filesDir`, CSV off-main, FlightLogs, FlightDetail + ColumnPicker (dual header dialects), share + **bulk export** | Bench seam: real flight log downloaded via bridge; CSV parity vs iOS for same `.bin`; JVM chunk/stall/abort tests green | 7–10 |
| 5 | Profiles + Settings + Syncer + DeviceManager + provisioning | RocketProfile (defaults pinned vs checked-in firmware-derived constants file; #449 fin-key exclusion), stores, ActiveRocketSyncer (one-shot trigger, 14-write burst, 0.8 s optimistic sync, role-flip re-attach), Settings (EditGroup commit-on-blur, txPower debounce, 3 s link-mode hold, pyro triple-write), DeviceManager, DeviceProvisioningSheet, onboarding + **Field Setup checklist screen** (battery-optimization deep-link, OEM instructions, green checks) | Defaults-parity test green; syncer unit-tested vs FakeFirmware; bench seam: push profile to bench rocket via OC, verify readback echo; edit on Android → confirm via iOS/serial | 14–18 |
| ★A | **Pixel 8a checkpoint** (order at Phase 2 exit) | Runtime permissions, scan-throttle, real MTU 517 (untrimmed telemetry, cmd 26/65 accepted), op queue under fire, reconnect matrix incl. status-133 + FGS survival backgrounded, download throughput, **S3: OTA pacing probe**, 10 Hz dashboard jank profile, heading arrow vs compass, TTS ducking | Bug-fix tail budgeted | 5–8 |
| 6 | Maps + location + prediction (**Spike S2 first**: MapLibre + tile proxy + style-swap layer-reinstall + camera-reason gating, 2–3 ev) | TileSource/TileMath (z/y/x URL vs z/x/y disk transposition test), OfflineTileCache, TileDownloader, SaveArea/OfflineMaps, RocketMapView (latched fix, geodesic uncertainty polygon, diff-based overlays), LocationManager (fused + rotation-vector heading + declination + MSL, **denied-state UI**), landing predictor orchestration, DriftCast 2D + form | Emulator with mocked GPS: offline area downloaded, airplane-mode map renders, replayed flight drives prediction pin | 13–17 |
| 7 | Cal + utility + charts | FinLayout, Simulation, FreqScan (hoppingMode gate), ServoTest, Canvas chart (LTTB re-decimation, safeDomain, real plot bounds), FlightTrajectory 2D, MagCal (APPLIED-trap semantics, every-frame verify edges) + Canvas sphere | Chart demo vs golden CSV; mag-cal demo vs recorded cal replay; bench seam: real mag cal + freq scan end-to-end | 11–15 |
| — | **Shadow-phone field outing** | Android as second phone at a real launch — zero-stakes field validation of connect/dashboard/map/voice. **NOT both on the same BS**: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` on OC *and* BS, so one phone per board is a hard limit. Run iPhone→BS (flight primary, relay path) and Android→OC direct (pad-side ground tests) — which also covers both link paths instead of one. Force-quitting the iOS app hands its board over normally; only scripted kills (`devicectl process terminate`) fail to stick, because iOS relaunches the app under state restoration | Field notes; fixes | 2–4 |
| 8 | OTA | One write per `onCharacteristicWrite`, CONNECTION_PRIORITY_HIGH during pump only, chunk = MTU−3−7, fleet-keyed session surviving reboot/recreation/process-death, FC-vs-device version source for rollback detection, SAF pick | State machine unit-tested vs scripted status stream; **Pixel seam**: full OTA of bench OC + FC-relay OTA + rollback path, throughput recorded | 7–10 |
| 9 | Trailing parity | Shared 3D trajectory Canvas (orbit camera), fleet chrome, PyroTest (Camera2 high-speed w/ 60 fps CameraX fallback tier; every abort path honors cmd-23 discipline), DriftCast 3D, announcer polish, in-app update checker, accessibility pass | — | 16–22 |
| ★B | **Galaxy A15 5G checkpoint** | One UI power management (45-min pad wait, screen off), vendor stack throughput floor, buffer-reuse defense, mid-range CPU perf, camera capability query | — | 2–4 |

**Screen order rationale** — the launch-day loop defines v1.0: scan/connect → pad dashboard (readiness, power gate, continuity, staleness) → flight (telemetry, voice, prediction) → recovery (map pin, arrow) → post-flight (download, chart). Settings/syncer is non-negotiable in v1.0: the app is the config source of truth; a syncer-less Android app silently flies stale NVS settings. Relay routing (cmd-50 targeting, sticky focus) ships in Phases 2–3 — at a real launch the phone talks through the BS LoRa relay most of the time; only the multi-rocket UI chrome trails. OTA trails: firmware can be flashed at home via iOS/USB indefinitely.

**v1.0 = Phases 0–7 + Checkpoint A + shadow outing.** Trailing: OTA (first to land), 3D views, PyroTest video, DriftCast send flow (guided flight is gated behind FIRST-GUIDED-FLIGHT anyway), fleet chrome, iOS-profile import.

**Spike verdicts**
- **S1 (emulator BLE)**: mooted — the Pixel 8 arrived before the spike ran; Phase 2's bench seam validated the real stack directly.
- **S2 (MapLibre + tile proxy)**: **PASSED 2026-07-27** on the Pixel 8. Raster styles render through the localhost read-through proxy (byte-identical iOS flat-file cache layout); style swap re-installs overlays in the `setStyle` callback; `REASON_API_GESTURE` gates follow mode; airplane-mode cold start renders the cached area. Two real findings, both fixed in `:core:maps`/`MapScreen`: (1) `InetAddress.getLoopbackAddress()` binds `::1` on Android while clients dial `127.0.0.1` — bind IPv4 loopback explicitly; (2) MapLibre fail-fasts all HTTP when the OS reports offline, so `MapLibre.setConnected(true)` is mandatory — safe because every source URL is loopback and the proxy owns real offline semantics. No osmdroid fallback needed.

---

## 5. Permanent dual-maintenance operating model

- **Versioning**: both apps use the same `MAJOR.MINOR` when at feature parity for a protocol generation; platform-specific patch releases float. APK `versionName` mirrors the iOS marketing version. Each release notes **min supported firmware** — the app↔firmware compatibility statement lives in the release notes and a `COMPAT.md` table (app version × FC/OC/BS firmware), because the phasing guarantees the two apps will NOT update together for ~a year.
- **Feature lag discipline**: when a firmware change touches a feature only one app has, the parity ledger gets an entry and the lagging app hides the surface behind a capability check (firmware version from `config_identity`/`fc_identity`), never a crash or a silent wrong behavior.
- **Bugfix mirroring**: a bug fixed in one app's core logic = ledger entry + twin fix or an explicit "iOS-only, why" note. The CI path-heuristic reminder (§2.3) backstops memory.
- **Release process**: tag → CI builds/signs/attaches APK + runs full parity gate. iOS release process unchanged.
- **Dependency cadence**: §3 — one pinned upgrade window/year.

---

## 6. Risk register (top items)

| Risk | Mitigation |
|---|---|
| Emulator-BLE premise fails (S1) | Known by week 2 for $15; pivot = FakeFirmware harness + earlier Pixel |
| Android vendor stack fights the op-queue model (status-133 storms, throughput) | Op queue is ~300 LOC behind `BleTransport`; Nordic Java lib is a drop-in fallback; Checkpoint A probes early; L2CAP CoC (#526 branch) is the long-term download escape hatch |
| MapLibre proxy/offline semantics don't hold (S2) | Standalone spike before Phase 6; osmdroid named fallback |
| OEM power manager kills a pad-wait session | FGS + Field Setup checklist + keep-screen-on during pad ops; Checkpoint B validates on One UI specifically |
| Telemetry JSON drift (no freshness gate on the highest-churn message) | Host-compile the builder into the emitter if tractable; else mandatory bench-capture step in the checklist |
| Solo-maintainer drift between the two apps | Golden vectors + hard-fail checker + behavior fixtures + parity ledger + CI reminder — §2 exists for exactly this |
| Estimate blow-out | Spikes convert the two biggest unknowns (BLE, maps) into early known cost; ±30% stated below |

---

## 7. Effort — honest numbers

Per-phase sums above give ~105–145 evenings to v1.0 and ~130–180 to full parity. The adversarial review judged the underlying per-phase figures ~1.3–1.6× optimistic against the LOC evidence (net-new design layers with no Swift to transcribe: op queue, tile proxy, FGS, Canvas charts + two Canvas-3D components, Camera2 high-speed) plus previously unbudgeted items now folded in (onboarding/Field Setup, provisioning, bulk export, update checker, signing CI, crash logging, accessibility, ledger upkeep).

**Planning numbers: v1.0 ≈ 300–450 h (~9–14 months at ~8–10 h/week); full parity ≈ 450–650 h (~14–20 months). ±30%.**

The consolation: Phase 0's infrastructure permanently hardens the iOS↔firmware contract too, and every phase ends at a committed, testable seam — the project is useful long before it's done, and pausable at any seam without rot.

---

## 8. Open decisions (user)

1. **Approve Spike S1** ($15 USB BT dongle) and the trigger rule: Pixel 8a ordered at Phase 2 exit, or immediately if S1 fails.
2. ~~**Default basemap**~~ **DECIDED 2026-07-27**: `usgsImageryTopo` stays the default and the ONLY offline/cacheable path; OpenFreeMap Liberty (keyless vector, online-only, ODbL attribution shown while active) joined the source cycle for international browsing. Save Area stays USGS-only. Post-flight FlightMapView rebases onto the USGS stack.
3. **v1.0 scope confirmations**: OTA trailing OK? MagCal in Phase 7 OK? Single-BS+single-rocket chrome for the first field season OK (relay routing present)?
4. ~~**EGM96 fallback**~~ **DECIDED 2026-07-27**: no bundled geoid table — platform MSL on API 34+ (both fleet devices), ellipsoid altitude with a caveat below.
5. **L2CAP CoC**: if `ble/526-l2cap-coc` merges before Phase 4, grow `BleTransport` with a channel path in v1, or GATT-only first?
6. ~~**Telemetry JSON fixtures**~~ **DECIDED 2026-07-27**: bench-captured frames committed to git approved (scrub coordinates); capture task tracked in #624.
7. **iOS golden-walk retrofit depth**: all families in Phase 0 (biggest coverage gain: Swift command encoders are currently byte-unpinned) vs per-family as they next change.
8. ~~File a tracking issue~~ **DONE 2026-07-27**: [#624](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/624); doc renamed.
