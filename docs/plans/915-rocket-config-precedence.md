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

## #1231 — the deployment configuration joins the report (2026-09-11)

Split out of #1117 by the #1225 triage as the one change that makes #1117,
#1131 and #1078 *detectable*. Each of those had its own fix; none of them gave
the operator a way to see the deployment configuration the FC actually holds,
because nothing on any screen was sourced from the FC's live `pyro_config`:
the app's `config_pyro` readback was the OC echoing its own cache, the #915
report had no pyro member, `FLIGHT_SETTINGS_MSG` is log-only, and the gated
scorecard bits read `SH_NA`.

### The wire

`ConfigReportData` v2: `PyroConfigData pyro` appended after `roll`, 193 bytes,
`F_PYRO_FROM_NVS` (bit 2) set when the FC's copy is a stored record. Appended,
not inserted, so **v1 is a byte-exact prefix of v2** and the OC accepts both —
copying a v1 frame by `offsetof(pyro)` and serving it with no pyro block —
rather than refusing it. The FC image is relayed through the OC, so an OC
updated ahead of its FC is the normal OTA order; the strict version check the
v1 handler had would have put every #915 group back on the app's can't-verify
list for that whole window. The prefix is pinned by a `static_assert` and the
host layout test.

The FC marks the report dirty on an applied pyro frame (it was the one config
handler that did not), and reports the LIVE struct `servicePyroChannels()`
reads, not a copy of the last frame.

### What the OC serves, and from where

`config_pyro` is built from the FC's report whenever the rail is on and a v2
report is held; the OC's cache stands in only with the rail off or under a
pre-#1231 FC. The frame carries `src` (`"fc"` / `"oc"`) and, FC-sourced only,
`fnv`. Two things the OC deliberately does **not** do:

- **It does not overwrite its cache from the report.** The cache is what the
  phone last pushed and what the rail-off readback serves (#1131); the report
  is what the FC holds. When they differ the OC logs one line per changed
  report — `FC deployment config differs from OC cache` — and the app is
  shown the FC's copy, with its source.
- **It does not self-heal.** The orientation precedent re-pushes the OC's
  record when the FC says it has never been told; doing the same for the
  deployment configuration would be the OC silently writing what fires. That
  is a flight-safety behaviour choice, not a visibility fix, and is left for
  the owner to decide separately. With `fnv` on the wire the app can at least
  say "the flight computer has no stored deployment config" instead of
  rendering four channels that look switched off.

The report-dirty path now sends four frames (the three extras plus
`config_pyro`), which could land in the same `loop_oc` pass as a connect burst
and overflow the 12-deep readback ring; `sendCurrentConfig()` clears the dirty
flag before its own snapshots so the two never stack.

### The apps

Both decode `src`/`fnv` into `RocketConfig.pyroSource` and
`pyroStoredOnFlightComputer`, carry them across a `config` rebuild the way the
pyro fields already were, and render one quiet caption under the pyro card
only when the tiles are *not* the FC's own stored configuration.

The #1078 optimistic mirror had to yield. With FC-sourced tiles, a write the
FC never applied produces no echo and an unchanged report — so a mirror that
painted the new values in would re-create exactly the invisible divergence
this issue exists to remove. The mirror now applies only when there is no echo
to wait for: the OC cache, an OC that predates the key, or the rail off.

### Bench validation (V9 pair, 2026-09-11)

All five #1211 checks ran on the V9 pair (`tests/bench/1231_phase*.txt`, driven by
`tools/bench_session.py`; boot lines via `tools/bench_capture_boot.py`, which
resets the board deliberately since the harness attaches without a reset).

- **Mixed versions** (new OC, pre-#1231 FC): the 169-byte v1 report was
  accepted, `config_servo`/`config_guid`/`config_roll` were still served, and
  `config_pyro` carried `"src":"oc"` with `Queued pyro config readback (…, from
  OC cache)` on the OC console — rail on, and after a rail-off reboot.
- **FC-sourced readback**: the new FC loaded A from NVS, its first report logged
  `pyro(nvs)=[1/0/1.0 1/1/150.0 …]`, and the readback carried `"src":"fc","fnv":true`.
  An edit was applied (`[PYRO CFG] …`), re-reported, and re-published to the
  phone unsolicited, with no retry line.
- **Erased FC NVS**: `NVS pyro: none (all four disabled)`, report `pyro(dflt)=[…]`,
  readback `"fnv":false` with all four disabled. The OC's orientation self-heal
  re-pushed `+X` to the erased FC at the same time; pyro was left at its default,
  the asymmetry this design leaves open on purpose.
- **Hand-written FC record** (ch1 off, ch2 300 m, ch3 2.0 s, written with
  `nvs_partition_gen.py` + `esptool write_flash 0x9000`) while the OC cache held
  something else: the readback showed the FC's record, and so did the **Android
  app's pyro tiles** (bench build on the Pixel) with no caption. After the app's own
  Power off, the tiles showed the OC's copy under "Flight computer is off. Showing
  the out computer's stored copy.", and returned to the FC's record on Power on.
- **Dropped frame** (`-DTR_TEST_CFG_DROP=1` OC image, BLE cmd 200 `03 ce`): three
  deliveries dropped 250 ms apart, `Cmd 0xCD cleared after 3 deliveries`, the hook
  disarmed itself, no `config_pyro` was published, the OC logged `FC deployment
  config differs from OC cache — FC(nvs)=[C] OC=[E]` on the next 5 s report, and a
  cmd-20 readback still showed C with `"src":"fc"`. The same edit on a healthy link
  landed, logged `matches OC cache again`, and was re-published.

Two things the bench corrected on the way. The divergence line is now evaluated on
every report and logged on the *transition* into divergence (a cache write the FC
never received would otherwise never be logged, since the FC's copy does not
change). And the first version of the drop hook decremented once per staging
attempt; the FC reads the staged buffer up to three times per poll, so three
armed drops were spent inside one poll — it now keys on the OC's own delivery
counter and disarms when the command is retired. One sequencing rule for the
scripts: never arm the hook while the previous command may still be inside its
three-delivery window.
