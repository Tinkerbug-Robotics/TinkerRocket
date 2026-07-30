# Android Parity Ledger

The running divergence record between the iOS and Android apps during (and
after) the port — android-port plan §2.3. Two kinds of entries:

- **Feature lag** — a capability one app has and the other doesn't yet.
- **Behavior divergence** — twin logic that intentionally or accidentally
  differs (a bugfix landed on one side, a policy change, a platform quirk).

Rules: any change under `TinkerRocketApp/TinkerRocketApp/Models/` or
`TinkerRocketAndroid/core/` that alters shared behavior must either touch the
twin in the same PR or add a dated entry here. Entries are removed when parity
is restored. "Parity at v1.0" is checkable ⇔ this file is honest.

## Feature lag (Android behind iOS — the port burn-down)

| Since | Area | Status |
|---|---|---|
| 2026-07-23 | `:core:protocol` COMPLETE — decoders, encoders, converter, telemetry, CSV pipeline all golden-pinned (bin→CSV byte-identical to iOS) | Phase 1 done |
| 2026-07-23 | Everything above the protocol layer: `:core:session` device/fleet logic, `:core:ble`, all UI | Phases 2–9 of docs/plans/624-android-port.md |

## Behavior divergences

| Since | Divergence | Rationale |
|---|---|---|
| 2026-07-23 | Disconnect suppression is **per-device** (`pendingUserDisconnects` set); iOS uses one global `userInitiatedDisconnect` flag that can suppress the WRONG device's reconnect in a multi-device fleet | The global flag is an iOS latent bug, not a behavior to preserve |
| 2026-07-23 | Download API returns typed `DownloadResult.Busy` for a second concurrent download; iOS silently overwrites the in-flight completion handler (leaking it) | Silent handler loss is a defect; byte/wire behavior unchanged |
| 2026-07-23 | Reconnect endgame is **re-scan → sighting-gated autoConnect**; iOS parks a CoreBluetooth `connect()` forever | Android `autoConnect=true` from a bare MAC can silently never fire (plan §3); the CB parked connect has no analog |
| 2026-07-27 | FreqScan's fixed-frequency **Apply** (transactional relay + verify + rollback) is iOS-only; Android shows a "use iOS" note | The fleet runs hopping mode (#150 default) where Apply is refused by design — the mask auto-applies. Port the transactional flow when a fixed-frequency need reappears |
| 2026-07-23 | Scan restart within 15 s is NOT truncated (epoch-guarded timeout); iOS's unguarded timer truncates a restarted scan | The iOS behavior is a timer-hygiene bug; a restarted scan deserves its full window |
| 2026-07-23 | KnownDeviceStore device list sorts plain case-insensitive; iOS uses `localizedCaseInsensitiveCompare` | Locale-dependent ordering is cosmetic; plain ordering is deterministic across devices |
| 2026-07-30 | Virtual Rocket is on BOTH platforms (Android `DemoMode.kt`, iOS `VirtualRocket.swift` — same script, frame for frame; one flight per SIM_START via the standard Simulation UI, SIM_STOP honored). iOS runs it in the Simulator (no CoreBluetooth). Divergence: the pre-staged downloadable sample flight is Android-only — iOS has no transport fake to serve file ops | Files-in-virtual: Android-only | Android's demo rides FakeFirmware (a full transport); iOS's rides a command tap + direct demux feed, which covers everything except the file pipeline. Follow-up noted: suppress the provisioning sheet for the virtual device (registry hygiene) |
| 2026-07-30 | In-app update checker (GitHub Releases → scanner banner → browser) is Android-only | Distribution differs by design (plan §1): iOS updates ride the App Store; Android sideloads from GitHub Releases, so the app itself must notice a new `android-v*` tag. Pure logic in `UpdateCheck.kt` (JVM-tested); fetch is throttled to one hit/day and silent on every failure — no signal at a launch site is normal |

## Feature lag (session layer, deferred to Phase 3)

| Since | Area | Status |
|---|---|---|
| ~~2026-07-23~~ | ~~RocketRoster merge + the 10 RocketRosterTests cases~~ | **PORTED 2026-07-27** — RocketRoster.kt + all 10 cases green; single-BS relay UI live (multi-device fleet chrome remains Phase 9) |

## UI parity (both directions — see docs/design-language.md)

| Since | Item | Direction | Status |
|---|---|---|---|
| 2026-07-27 | Flight-event flag chips (LAUNCH/BURNOUT/APOGEE/LANDED/LOG illuminating) | Android → iOS | **done 2026-07-30** (`FlightEventFlagsView`, render-tested) |
| 2026-07-27 | Sensor-health dot row (per-subsystem green/amber/red/gray dots) | Android → iOS | **done 2026-07-30** (`HealthDotRow` replaces the labeled grid). The go/no-go banner completed the round trip the same day — Android now renders `flightReadiness` too (`ReadinessBanner`), so the health cards match in both directions |
| 2026-07-27 | iOS dashboard sections not yet on Android (storage bar, camera/logging controls, voice indicator) | iOS → Android | **done 2026-07-30**: StatusCard (badges + active file + BS auto-close countdown), ControlsCard (relay-aware camera/log toggles + BS CSV toggle), StorageCard (0xCC/0xCD variants, #315 auto-evict note), go/no-go ReadinessBanner — and the LOG chip dropped in the same change, re-converging the flag rows. Voice indicator had already landed with the announcer (#645). Verified live on both link types (bench OC: 22 flights / 305 MB used; bench BS: countdown 5:00, 451 MB free) |

## Test-coverage parity

| Since | Item | Direction | Status |
|---|---|---|---|
| 2026-07-28 | OTA timing contract — shared golden `tests_cpp/fixtures/app_behavior/ota_timeouts.json`, both suites assert it plus the crossesRelay invariant | both | done |
| 2026-07-28 | `OTASession` flow tests. Android has 11 (`OtaSessionTest`) against a scripted peripheral; iOS has none, because `OTASession` reaches concrete `BLEFleet`/`BLEDevice`. Needs a protocol over the 9 members it touches + the fleet lookup behind a closure — the Android design. | Android → iOS | **done 2026-07-30**: `OTALink` protocol (BLEDevice conforms retroactively, zero changes) + `linkLookup` closure init; 9 flow cases ported (`OTASessionFlowTests`) running the REAL timeout table at 1/200 time-scale — full flow incl. the reconnect-swap property, rollback, mid-pump verify-fail, all three timeout branches incl. the FC-window-outlasts-local invariant, size gate, cancel-aborts. Wire-byte and conn-priority assertions stay Android-side by design (encoding is golden-pinned in BLEDevice; iOS has no priority boost). 304/304 suite green |
| 2026-07-29 | Announcer flight-profile tests. The #643 port put callout policy behind an `AnnouncerSpeech` seam, so Android has 15 state-machine cases iOS could not run — its policy was welded to `AVSpeechSynthesizer`. | Android → iOS | **done 2026-07-30**: iOS `FlightAnnouncer` split into policy + `SystemSpeech` engine behind the same `AnnouncerSpeech` shape as Android (isBusy/speak/stop + session status), with injectable clock and unit-system; 17 policy cases ported (`FlightAnnouncerPolicyTests`) — burnout counter incl. jitter tolerance + speed floor, 5 s altitude cadence + deadband, apogee edge + unknown-alt, first-descent-at-5 s-then-10 s, landed haversine + state fallback, STALE gate, disabled tracking, busy-skip vs interrupt, PRELAUNCH reset, enable/disable lifecycle, imperial. Dispatch (#138) + wording (#235) tests unchanged. 321/321 suite. The #390 non-focused-silence case remains Android-only (it lives in dispatch, where iOS's relay path differs structurally) |
