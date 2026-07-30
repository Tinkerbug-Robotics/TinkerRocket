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

## Feature lag (session layer, deferred to Phase 3)

| Since | Area | Status |
|---|---|---|
| ~~2026-07-23~~ | ~~RocketRoster merge + the 10 RocketRosterTests cases~~ | **PORTED 2026-07-27** — RocketRoster.kt + all 10 cases green; single-BS relay UI live (multi-device fleet chrome remains Phase 9) |

## UI parity (both directions — see docs/design-language.md)

| Since | Item | Direction | Status |
|---|---|---|---|
| 2026-07-27 | Flight-event flag chips (LAUNCH/BURNOUT/APOGEE/LANDED/LOG illuminating) | Android → iOS | pending back-port |
| 2026-07-27 | Sensor-health dot row (per-subsystem green/amber/red/gray dots) | Android → iOS | pending back-port (user-requested) |
| 2026-07-27 | iOS dashboard sections not yet on Android (storage bar, camera/logging controls, voice indicator) | iOS → Android | Phase 4/5 screens |

## Test-coverage parity

| Since | Item | Direction | Status |
|---|---|---|---|
| 2026-07-28 | OTA timing contract — shared golden `tests_cpp/fixtures/app_behavior/ota_timeouts.json`, both suites assert it plus the crossesRelay invariant | both | done |
| 2026-07-28 | `OTASession` flow tests. Android has 11 (`OtaSessionTest`) against a scripted peripheral; iOS has none, because `OTASession` reaches concrete `BLEFleet`/`BLEDevice`. Needs a protocol over the 9 members it touches (`sendOtaBegin/Chunk/Finish/Abort`, `otaStatus`, `isConnected`, `otaMaxChunkSize`, `firmwareVersion`, `fcFirmwareVersion`) + the fleet lookup behind a closure — which is the Android design (`sessionLookup: () -> DeviceSession?`), so this converges the two rather than adding a parallel shape. Then the 11 cases port over. | Android → iOS | **Phase 9** (user-scheduled 2026-07-28) |
| 2026-07-29 | Announcer flight-profile tests. The #643 port put callout policy behind an `AnnouncerSpeech` seam, so Android has 15 state-machine cases (burnout counter, apogee edge, 5 s/10 s cadences, STALE gate, busy-skip vs interrupt, PRELAUNCH reset) that iOS cannot run — its policy is welded to `AVSpeechSynthesizer`. Same convergence story as OTASession: give iOS the seam, then the cases port. iOS keeps its 4 dispatch tests (#138) + 3 wording tests (#235); Android has both plus a non-focused-rocket-silent case (#390) iOS lacks. | Android → iOS | **Phase 9** |
