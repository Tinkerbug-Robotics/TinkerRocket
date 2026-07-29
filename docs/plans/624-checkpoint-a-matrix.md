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

## Group 4 — Reconnect matrix ✅ DONE 2026-07-29

The one I'd least want to discover in the field. Result: clean except cold start.

- [x] Board power-cycled mid-session → **PASS**, reconnected in under 6 s (faster than a 6 s
      sampling interval could catch a disconnected state)
- [x] Walk out of range → return → **PASS**, reconnected with no user action. Drop confirmed
      by direct observation at the phone; see the probe note below for why the instrument
      missed it
- [ ] **status-133** → NOT OBSERVED. Opportunistic; it cannot be forced deliberately, so it
      stays a watch-for-it during other work rather than a step
- [x] Airplane mode on → off → **PASS**, back inside 10 s. Usefully, the ladder is visible
      and honest in the UI: `Reconnecting (5/8)...`
- [ ] Phone reboot with board live → **FAIL** → issue #633. Mid-session reconnect is solid;
      this is specifically cold start. iOS scans on BT power-on (`BLEFleet.swift:397`) plus
      state restoration, so it reconnects itself; Android's `scan()` is only ever called from
      the Scan button, and the sighting-gated autoConnect endgame never engages from cold
- [x] **FGS survival** → **PASS**: 47 min 56 s, 95 samples, 0 process deaths, 0 GATT drops,
      deep Doze held for every sample, PID stable
- [ ] OC current during the wait → NOT MEASURED (needs a meter; the app's own battery figure
      is the rail, not the OC's own draw)
- [x] Battery optimization ON → **PASS**, covered by the run above — the app was never
      whitelisted

### Method notes — two traps that produced wrong answers first time

**The connection probe must be the GATT record, not the MAC.** Grepping the device address
out of `dumpsys bluetooth_manager` matches `Remote Client: <mac>`, a registration entry that
**persists across disconnects** — it is structurally incapable of reading 0, and it silently
reported "no drops" through a real, physically-confirmed disconnect. Validate any probe in
both directions before trusting it. What actually works:

```sh
adb shell dumpsys bluetooth_manager | grep -c "Connection<conn_id.*<MAC-suffix>"
# 0 when disconnected, non-zero when connected (several stale conn_ids linger — treat as bool)
```

**A plugged-in phone never enters Doze.** The first FGS run was charging throughout, so it
never faced the thing most likely to kill a backgrounded service, and the result was
worthless. Simulate battery while keeping USB for adb:

```sh
adb shell dumpsys battery unplug          # AC/USB powered -> false
adb shell dumpsys deviceidle force-idle   # "Now forced in to deep idle mode"
...run the test...
adb shell dumpsys deviceidle unforce && adb shell dumpsys battery reset   # ALWAYS restore
```

**Don't run FGS tests soon after a reboot.** A post-reboot backup sweep
(`BackupManagerService: Killing agent host process`) tears down processes wholesale —
including `system` — and killed the app at 19:53 in one run. That looked exactly like an FGS
failure and was not one.

**Prove the perturbation landed.** An early "board power-cycle" run showed the app staying
connected, which could equally have meant the reset never happened — esptool cannot always
sync with an OC in light sleep. Use an observable tell-tale: the rocket power rail is ON
before and OFF after, because `pwr_pin_on` starts OFF on every OC boot.

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
