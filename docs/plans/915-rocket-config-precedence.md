# 915 — The rocket keeps its own settings

Issue: [#915](https://github.com/Tinkerbug-Robotics/TinkerRocket-Hardware/issues/915)

A rocket that has been configured should stay configured, and a phone should be
able to connect to any rocket in the field without changing it. That is a
precedence decision, not a persistence one — the retention already works.

## What the investigation found

**Retention is not the gap.** The flight computer persists nearly its entire
config to NVS and reloads it at boot: servo biases 1–4, hz, pulse endpoints, fin
travel, PID, gain schedule, all five roll-control parameters, guidance enable
plus all thirteen PN parameters, fin layout (four azimuths and both reverse
masks), roll waypoints, pyro, IMU rate, sounds, servo-enable, camera, mag cal and
sensor cal. The one exception is the IMU mounting orientation, which the FC does
*not* store — the out computer stores it instead and re-pushes it through the
status-query self-heal when the FC comes up in `ORIENT_MODE_DEFAULT`.

**The gap was precedence.** Within about a second of connecting, the app pushed
fourteen config frames (not eleven — the log in the issue is truncated; pyro,
sounds and IMU rate follow), overwriting whatever the rocket had with whichever
profile happened to be active. It did this *after* receiving the rocket's
identity, and while already knowing which profile had last flown on that board.

**`lastUsedUnitID` was already the binding, unused.** It bound a profile to a
board's hardware id and drove a "last flown as…" suggestion the user had to
accept — while the push went out regardless. Reading that binding instead of
ignoring it is most of the fix.

**The out computer's config cache is display-only.** It feeds the readback JSON
and is never re-pushed to the FC, so it can drift from what the vehicle actually
holds. The exception, again, is orientation.

**`nid=180 → 0` is not an ongoing discard.** It is the one-shot identity NVS
v0→v1 schema migration, which fires once per device.

## The rule

The rocket is authoritative on connect.

1. On connect, the profile bound to the connected board becomes active.
2. A board the app has never seen is adopted as a new profile, seeded from the
   rocket's own reported settings and named from its unit name.
3. The rocket's reported values are adopted into that profile. Where the two
   disagreed, the profile changes and the app says which groups changed.
4. **Connecting writes nothing to the vehicle.** The whole profile goes out only
   on an explicit act: switching the active profile onto a connected rocket, or
   *Send All Settings*. A single-field edit still self-applies its group (#144).

Calibration is deliberately unchanged. Mag and sensor cal were already
board-tagged and refused to cross to a board they were not captured on, which is
the same guarantee by a different route; changing a well-tested safety path that
already satisfies the goal would have been churn.

## What the app still cannot see

The readback echoes about half the editable surface. Not reported: servo trim
2–4, fin travel, fin layout, roll waypoints, the PN guidance parameters, sounds.
Those are shown from the profile and labelled as unverifiable rather than
presented as confirmed. Two comparison rules keep adoption honest:

- Values are compared at the precision they cross the wire at, so a profile Kp of
  `0.12` does not "differ" from a rocket reporting `0.1200` on every connect.
- The #253 sentinels (`rcap`/`kpang` ≤ 0, `iwind` < 0) mean "firmware default",
  and `RocketConfig` then holds the *app's* defaults rather than the vehicle's.
  Adoption skips those three fields when the sentinel came back, so a
  deliberately-tuned profile is not overwritten with a number nobody chose.

One defect fell out of this: `RocketConfig`'s PID defaults were `0.08 / 0.005 /
0.003 / ±10`, matching neither the firmware nor `RocketProfile`. Harmless while
they were only a display fallback; a silent re-tune once adoption reads them.
Aligned to config.h on both platforms, the same fix #407 and #561 already made
for the servo fields.

## The firmware follow-up (shipped second)

Ordered after the app change deliberately — that change stands on its own, and
this is what retires the "cannot verify" list.

### 1. The FC now persists its IMU orientation setting

Every other config group wrote NVS in its command handler; `ORIENT_CONFIG_PENDING`
did not. It now stores the SETTING (`orient`/`set`, the same namespace and key
the OC and the mini already use) and restores it at boot, ahead of the snapshot
recovery — which still wins, because a mid-flight reboot has to come back in the
frame the EKF state was estimated in.

Only the setting is written, never the active frame: the pad-gravity auto-detect
re-snaps that on its own, and persisting an auto-detected result would silently
convert a rocket the user left on AUTO into a manually pinned one — pinning
disables the very detect that chose it.

**Two memories can now disagree**, so the report carries provenance
(`F_ORIENT_FROM_NVS`). The OC's status-query self-heal — which existed *because*
the FC could not remember — is skipped once the FC says it has its own record,
and the `imu_orient` readback reports the FC's setting rather than the OC's
cache. Without the bit, reflashing either board alone would silently lose
whichever memory the other overwrote: an FC that genuinely holds AUTO is
indistinguishable from one that has never been told.

### 2. `ConfigReportData`, pushed FC→OC

169 bytes on `CONFIG_REPORT_MSG` (0xFB), composed from the very structs the app
writes — `ServoConfigData`, `FinConfigData`, `GuidanceConfigData`,
`RollProfileData` — so a field that moves in one moves here with it.

Not an extension of `FlightSettingsData`: that is 219 bytes against a
`MAX_PAYLOAD` of 224, and it is a flight-log record emitted at
PRELAUNCH→INFLIGHT, a different job from a pre-flight readback.

**Pushed, not requested.** The FC emits it at boot, on every accepted change to
a reported field, and every 5 s while not INFLIGHT. A request/response would have
cost a second message code — and this space now has exactly two values left
(0xFC, 0xFD) — and it would not have self-healed an OC that rebooted on its own,
because the OC has no way to know it missed one.

Deliberately **not logged**: the OC handles it before the log enqueue, like
`FC_IDENTITY`. It is pre-flight state, `FlightSettingsData` already records what
actually flew, and keeping it out spares the `Data_Analysis` parsers a message
type they hardcode no length for.

### 3. Three readback frames

`config_servo` (trim 2-4, fin travel, fin layout, sounds), `config_guid` (the
thirteen PN / station-keep parameters) and `config_roll` (the waypoints), each
small enough for the notify MTU. `config_roll` is sent even when there are zero
waypoints, because "rate-only" and "this rocket can't tell you" must not look
alike to the app.

This took the connect burst from six frames to nine against an eight-deep
readback queue — the ninth enqueue evicted the oldest, which is the main
`config` frame. Cap raised to 12; it has to move with the burst.

The mini needs none of this: it has no servo, fin, guidance or roll hardware, so
its `config` frame was already trimmed and those groups are genuinely absent
rather than merely unreported.

### 4. The app adopts them

`unreportedGroups` is now derived from which frames actually arrived, so it
empties on new firmware and stays honest on old firmware and on the mini. Two
distinctions the adoption has to keep straight:

- **null vs empty waypoints.** null is "we don't know" and must leave the
  profile's roll profile alone; empty is "we know, and this rocket flies
  rate-only" and must clear it.
- **Fin azimuth → ring slot must FLOOR, not round.** The "×" layout's azimuths
  (45/135/225/315) land exactly on rounding ties, where Kotlin breaks to even
  and Swift breaks away from zero — the two platforms derived *different* fin
  mappings from the same rocket. Flooring has no tie to break.

An asymmetric rocket-side fin cal collapses to its span, because #449 made the
profile store one travel number. That is a limitation of the profile model, not
of the readback.

## Bench validation (V9 board, 2026-08-25)

OC `9c:13:9e:28:ab:5c` + FC `80:f1:b2:d0:94:a7`, both on this branch, with a
freshly installed app.

**Orientation survives an FC-only reflash — confirmed.** The FC comes up

    I (194)  FC: IMU orientation setting: +X (from NVS)
    I (6027) FC: Board→rocket orientation: +X (code 0, mode 1, residual 0.00°)

One line, mode 1 (MANUAL), straight from its own NVS. The issue's log showed
mode 0 (DEFAULT) at boot and only reached mode 1 seconds later when something
else pushed it.

**The report reaches the app — confirmed.** The FC logs

    [CFG] Config report sent: orient=+X(nvs) sounds=off bias=[-125,-60,20,-55]
          fin=[0,90,180,270] rev=0x0/0x0 guid=on wp=0

and a phone with NO saved profiles came away showing exactly those four trims.
Trims 2-4 can only come from `config_servo`; trim 1 rides `sb1` in the main
`config` frame — so the whole burst landed, which also clears the queue-depth
change (at the old cap of 8 the ninth enqueue would have evicted `config` and
servo 1 would have read 0).

**Not tested: the OC-only-reflash case.** The provenance bit only does
observable work when the two boards' orientation records DISAGREE, and today
both hold +X. Arranging a genuine divergence means erasing one board's NVS,
which the bench rule forbids (it holds cal and LoRa config). The logic is
covered by construction and by the FC's `(from NVS)` / `(board default)` line,
but it has not been exercised against a real disagreement.

**A bug the bench found that no test would have.** Switching between two
rockets: put a second profile on the V9, connect to the V8 (correct), come back
to the V9 — and the selection had reverted. `lastUsedUnitID` bound a profile to
a board but nothing ever RELEASED one, so two profiles claimed the same board
and the lookup takes the first match in a list sorted by NAME. Which profile a
rocket came back on was decided alphabetically. The other rocket looked fine
only because a single profile had ever claimed it — one board is not enough to
see this, which is exactly why it survived the unit tests.

Binding is exclusive now, through one `store.bind()` that owns the invariant,
plus a heal for stores the broken build already wrote: when more than one
profile claims a board, prefer the most recently updated and release the rest.
Without the heal the fix would have looked like it had not worked, because the
wrong profile was already stuck where it was stuck.

**The precedence rule itself held throughout.** Over the whole switching run
the app sent the rocket three commands — time sync, a sensor-cal READ, and a
readback request. Zero config writes, against thirteen frames in under two
seconds from the old app on the same board that morning.

**Expected bench noise:** `[ORIENT] pad gravity 93.9° off nose with MANUAL
orientation +X` — the board is lying flat, so gravity is perpendicular to the
nose axis and MANUAL means the FC will not auto-correct. Correct behaviour,
and the warning that would matter on a pad.

## Still open

- The OC-only-reflash divergence case above.
- Two free message codes left in the OC↔FC space. The next one needs an
  escape/extended encoding, not a thirteenth constant.
