# Checkpoint A — Pixel 8 hardware pass

*Android port [#624](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/624). Gate on
v1.0 alongside the shadow-phone outing; every code phase (0–8) is merged.*

The point of this checkpoint is to find problems at a desk instead of at a launch site. Most
of it has been touched incidentally by phase bench seams — what has never happened is a
deliberate pass with numbers written down. Budget is mostly the bug-fix tail, not the runs.

Grouped into sittings that each end at a stopping point. Nothing here depends on a later
group, so they can be run in any order.

**Rig**: Pixel 8 (`43230DLJH000MG`), Rocket Computer V6 (`a1cae0f0`), BaseStation V4.
**One phone per board** — `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` on both OC and BS, so a
second phone cannot connect at all. Force-quitting the iOS app releases its board normally.
The automation gotcha: `devicectl device process terminate --kill` does *not* stick — iOS
relaunches the app under state restoration and it re-grabs the peripheral, so a scripted
kill leaves the board invisible to the other phone. Bluetooth off is the reliable lever when
driving this from a script.

---

## S3 — OTA pacing probe ✅ DONE 2026-07-28

Closed out ahead of the rest by the #627 work; recorded here so the checkpoint isn't
re-run needlessly.

| | result |
|---|---|
| OC direct OTA (Android) | 746.7 kB, ≤11 s, **~68 kB/s** at 502 B/chunk, MTU 512 |
| OC direct OTA (iOS) | 764.6 kB, 67.1 s, **11.4 kB/s** — 6x slower, same board |
| FC relay OTA | capped at **12 kB/s** (`fcRelayMaxBytesPerSec`); above that the OC's NimBLE mbuf pool exhausts and the link wedges (#627) |
| Rollback path | verified — re-flashing an identical image reports RollbackDetected |
| Survives reboot + session recreation | yes (fleet-keyed OTA session, #140) |

---

## Group 1 — Permissions and cold-start

- [ ] Fresh install, all permissions denied → app explains rather than dead-ends
- [ ] Grant BLUETOOTH_SCAN/CONNECT only (deny location) → scanning still works; the
      direction-to-rocket arrow degrades with a stated reason, never blocks
- [ ] Deny POST_NOTIFICATIONS → FGS still runs; no crash on the notification path
- [ ] Revoke a permission from Settings while connected → app recovers, no crash
- [ ] Cold start with Bluetooth OFF → clear prompt, recovers when enabled

## Group 2 — Scan throttle and discovery

- [ ] 6+ scan starts inside 30 s → Android's undocumented 5-scans-per-30-s throttle does not
      leave the UI stuck "scanning" forever
- [ ] Scan with both OC and BS powered → both appear, correct type badges
- [ ] Scan with target board powered off → empty state is honest, not a spinner
- [ ] Record: time-to-first-advertisement (Phase 2 measured **0.65 s** via svc-UUID filter)

## Group 3 — Link under load

- [ ] Confirm MTU negotiates 517→**512** and telemetry arrives **untrimmed** (`"tr"` flag
      absent — see the BLE JSON budget note)
- [ ] cmd 26 and cmd 65 accepted at full MTU
- [ ] Delete-burst: queue 10+ file deletes back-to-back → op queue serializes, nothing lost
- [ ] Profile sync burst (~13–15 commands) → all land; verify by readback, not by UI state
- [ ] Sustained 10 Hz telemetry for 10 min → no leak, no growing latency

## Group 4 — Reconnect matrix

The one I'd least want to discover in the field.

- [ ] Board power-cycled mid-session → reconnect ladder (1,2,4,8×5 s) recovers
- [ ] Walk out of range → return → reconnects without user action
- [ ] **status-133** (Android's generic GATT failure) → single retry path works, no wedge
- [ ] Airplane mode on → off → recovers
- [ ] Phone reboot with board live → app reconnects on next launch
- [ ] **FGS survival**: connected, background the app, screen off **45 min** → still
      connected and logging (this is the pad-wait case)
- [ ] Measure OC current during that wait. #519 puts the BT LP clock on the 32 k crystal
      (`CONFIG_RTC_CLK_SRC_EXT_CRYS`), so the board light-sleeps *while connected* — expect
      ~1 mA, not the ~22 mA a `-bench` build draws. A regression here silently costs pad
      endurance, and nothing else would catch it
- [ ] Same, but with battery optimization ON for the app → document what actually happens;
      this is what the Field Setup checklist screen exists to prevent

## Group 5 — Throughput and files

- [ ] Download a real flight log; record kB/s and total time
- [ ] Compare against iOS on the same `.bin` (iOS baseline ~35 kB/s post-#524)
- [ ] Stall path: interrupt mid-download → 3 s stall timer fires, state is honest
- [ ] On-device bin→CSV of the same log → byte-identical to the iOS golden
- [ ] Bulk export / share sheet works for a real file

## Group 6 — UI performance

- [ ] 10 Hz dashboard, 2 rockets in roster → frame timing; look for jank, not averages
- [ ] Chart screen with a full flight CSV → pinch/pan stays responsive during
      re-decimation (200 ms debounce)
- [ ] Map with offline tiles + prediction overlays at 1 Hz → no dropped frames
- [ ] Screen-on power draw during a simulated 45 min pad wait

## Group 7 — Sensors and audio

- [ ] Heading arrow vs a physical compass, 8 headings → declination applied, wrap-around
      correct at N
- [ ] Arrow behaviour when location permission is denied → stated, not silently wrong
- [ ] TTS announcements duck background audio and restore it
- [ ] Announcements still fire with the screen off

---

## Exit

- [ ] Every box above ticked or explicitly waived with a reason
- [ ] Numbers recorded in this file (throughput, frame timing, time-to-first-advert)
- [ ] Bugs filed; blockers fixed; non-blockers triaged onto Phase 9 or the ledger
- [ ] Plan doc §4 updated — the shadow outing's "both on the same BS" is **wrong** and needs
      correcting to one-phone-per-board (iPhone→BS, Android→OC)
