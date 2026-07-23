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
| 2026-07-23 | Everything above the protocol layer: `:core:session` device/fleet logic, `:core:ble`, all UI | Phases 2–9 of docs/plans/android-port.md |

## Behavior divergences

*(none)*
