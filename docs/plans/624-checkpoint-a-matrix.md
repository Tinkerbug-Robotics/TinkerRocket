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

## Group 2 — Scan throttle and discovery ✅ DONE 2026-07-29

- [x] 7 scan starts inside 20 s → **PASS**. Ends at "Scan complete" with the device listed,
      never stranded on a spinner; no `SCAN_FAILED` or throttle warnings in logcat, so the
      app's own stop-between-starts lifecycle keeps it under Android's 5-per-30-s limit
- [x] Both boards powered → **PASS**. `Rocket Computer V6` (−30 dBm) and `BaseStation V4`
      (−42 dBm), distinct names and MACs. NOTE: scan order is not stable between runs — match
      the Connect button to its device row, never to a fixed y
- [x] Target powered off → **PASS** (partial). An absent board simply isn't listed — no stale
      entry, no phantom, no spinner. A both-off empty state was not exercised
- [x] Time-to-first-advertisement → **2.54 s measured**, but this includes ~0.5–1 s per
      `uiautomator` poll. NOT comparable to the in-app 0.65 s Phase 2 figure; treat as
      "advertises promptly", not as a regression signal

## Group 3 — Link under load ✅ MOSTLY DONE 2026-07-29

- [x] MTU 517→512 → **PASS**, straight from the BLE stack (the cleanest evidence available,
      and independent of the app's own UI):
      ```
      BluetoothGatt: configureMTU() - mtu: 517
      gatt_process_mtu_rsp: MTU Exchange resulted in: 512
      BluetoothGatt: onConfigureMTU(..., 512, 0)      # status 0 = success
      ```
- [x] Telemetry untrimmed (`"tr"` absent) → **WAIVED** 2026-07-29. No path to the JSON: the
      app doesn't log telemetry at any logcat level and the OC console is silent under light
      sleep on a normal (non-`-bench`) image. Phase 2's bench seam already confirmed untrimmed
      telemetry at MTU 512 and nothing since has touched that path; reflashing the OC to
      re-measure it isn't worth the churn
- [ ] cmd 26 / 65 accepted at full MTU → PENDING
- [ ] Delete-burst (10+) → PENDING, needs flight logs on the board
- [x] Profile sync burst → **PASS with a caveat**. The connect-time burst completed with zero
      GATT write errors and a fresh connection shows synced config. But this is NOT the
      independent readback the matrix asks for — `BenchSeamTest` would not run (instrumentation
      runner crashed on install), and the app confirming its own sync is weaker evidence. The
      `profile: synced` badge specifically is NOT proof: `SYNCED_DELAY_MS` fires 800 ms after
      pushing, optimistically, without confirmation
- [x] Sustained 10 Hz telemetry → **PASS**. 9 min, PSS flat at 142.4→142.7 MB after startup
      settling (+0.2%, noise not leak), pid stable, GATT connection held throughout, no
      exceptions in logcat

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
