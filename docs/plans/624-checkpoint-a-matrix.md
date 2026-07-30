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

## Group 1 — Permissions and cold-start ✅ DONE 2026-07-29 (found a crash)

- [x] All permissions denied → **PASS**. No crash; the app explains itself — "Bluetooth access
      needed / TinkerRocket finds and talks to your flight computers over Bluetooth.
      Nearby-devices permission is required." with a Grant button. Correctly gates on BLE only
- [x] Scan/connect granted, location denied → **PASS on both halves**. Scanning is unaffected
      (both boards found) — that's `neverForLocation` and minSdk 31 paying off. The bearing card
      degrades with a stated reason and an in-place remedy: "Location permission is needed to
      show direction and distance to the rocket." + `Grant location`
- [x] POST_NOTIFICATIONS denied → **PASS**. `Background started FGS: Allowed`, connection
      established, no `SecurityException` on the notification path — the OS suppresses the
      notification rather than throwing, which is what the pad wait depends on
- [x] Revoke while connected → **PASS**. Android kills the process on revoke BY DESIGN (not a
      crash); relaunch recovers cleanly with no crash loop.
      PLATFORM NOTE: `BLUETOOTH_SCAN` and `BLUETOOTH_CONNECT` share the `NEARBY_DEVICES` group,
      so revoking CONNECT alone gets it auto-re-granted on the next request (it comes back
      WITHOUT the `USER_SET` flag — that's the tell). "Connected with CONNECT denied" isn't
      reachable that way; revoke both, which the first item covers
- [x] Cold start with Bluetooth OFF → **CRASH FOUND, then FIXED**. Launch was fine, but tapping
      **Scan** with the adapter off killed the app:
      ```
      E AndroidRuntime: BleTransportException: BLE scanner unavailable (adapter off?)
      I ActivityManager: Process ... has died: prcp CRE      # exit-info: APP CRASH(EXCEPTION)
      ```
      `FleetManager.scan()` caught only `TimeoutCancellationException`, so a scanner-flow
      failure escaped the `launch` and reached the default handler. Now caught: scanning stops,
      the reason is surfaced ("BLE scanner unavailable (adapter off?)"), the spinner clears, and
      a later scan works once the adapter returns — verified on-device, same pid throughout

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
- [x] cmd 26 / 65 accepted at full MTU → **PASS** (2026-07-29). Negotiated MTU is **512**, not
      517 — the OC's NimBLE preferred-MTU cap wins over Android's 517 request
      (`dumpsys bluetooth_manager`: `mtu: 512` on the OC's GATT client row); write payload
      ceiling 509 B. Exercised via the Settings self-apply path on the live link: add/remove a
      roll waypoint (cmd 26, 76 B, twice) and guidance-enable on/off (cmd 65, 45 B, twice).
      All four frames are `withResponse = true` writes and all ATT-acked — a failed
      `writeCommand` forces the sync badge to `Failed`, and it stayed `synced` throughout,
      with zero GATT errors in logcat. NimBLE runs the characteristic access callback *before*
      responding, so an ATT ack means the OC application layer (the #517 command ring) had the
      full frame. Honest scope: FC-side apply (NVS persist, field validation) is the
      iOS-shared firmware path and was NOT independently re-verified — FC serial isn't
      attached, and opening OC serial resets the boards. At the default MTU 23 these frames
      could not fit one write (cap 20 B), which is what this box existed to rule out.
- [x] Delete-burst → **MECHANISM PROVEN 2026-07-29** via the #634 multi-select (3-file burst
      of disposable sim logs; a 10+ burst would need 10 disposable files). The OC's serial
      confirms all three cmd-3s arrived **in the order sent** and each acknowledged
      `Delete '...': OK`, whole burst in 223 ms, list resynced to page 0 afterward. Unit test
      pins the ordering + stop-on-failure; the op-queue serialisation is what the hardware
      run adds. CAUTION for future runs: pagination undercounts silently — a fixed page-count
      crawl missed 5 files; crawl until Next disappears and diff against a full inventory
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
- [x] **status-133** → WAIVED (2026-07-29): never observed across the whole checkpoint —
      dozens of connects, walk-away drops, Doze cycles, reboots. It cannot be forced
      deliberately; the reconnect ladder that would handle it is the same one airplane-mode
      and walk-away exercised. Stays a watch-for-it at the field outing.
- [x] Airplane mode on → off → **PASS**, back inside 10 s. Usefully, the ladder is visible
      and honest in the UI: `Reconnecting (5/8)...`
- [x] Phone reboot with board live → **WAS #633, NOW FIXED**. Mid-session reconnect was always
      solid; cold start wasn't. iOS gets state restoration plus a re-scan on BT power-on
      (`BLEFleet.swift:397`); Android's `scan()` was only ever called from the Scan button, so
      a crash, OEM kill or reboot left the user tapping Scan then Connect. `resumeLastSession()`
      now scans on launch when the last session ended connected, waits for a real advertisement,
      then autoConnects — bounded at 20 s, unlike the mid-flight endgame's deliberate forever.
      Verified: relaunch after a kill reconnected in **under 5 s with zero taps**; an explicit
      user disconnect clears the hint and relaunch correctly stays put
- [x] **FGS survival** → **PASS**: 47 min 56 s, 95 samples, 0 process deaths, 0 GATT drops,
      deep Doze held for every sample, PID stable
- [x] OC current during the wait → WAIVED (2026-07-29): needs a bench meter on battery
      (#519's rule — USB masks it), and the OC light-sleep figure (~1 mA connected) is
      already established by #519's own measurements. Nothing Android-specific left in this
      box — the phone-side pad-wait cost is measured (Group 2 Doze + Group 6 screen-on).
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

## Group 5 — Throughput and files ✅ MOSTLY DONE 2026-07-29

Log generated on the bench by a **simulated flight** (Simulation tool → 20 g / 40 N / 1.5 s
burn), not a real one — the sim runs on the real FC and writes a real log, so it exercises
the whole download/CSV/export pipeline without flying. Sim logs are HIL: injected altitude
and velocity, GNSS frozen. Fine for pipeline tests, NOT for judging flight-data quality.

- [x] Download throughput → **PASS**. `flight_20260729_163740.bin`, **10,544,908 B verified
      on disk** against the board's reported 10544.9 kB — exact. Download alone completed
      within 90 s (**≥117 kB/s**); **84.8 kB/s end-to-end** including on-device CSV
      generation. iOS baseline is ~35 kB/s, so Android reads roughly 2.4x faster — the
      opposite direction from OTA writes, where iOS is 6x slower (#627)
- [x] Compare against iOS on the same `.bin` → **PASS — BYTE-IDENTICAL** (2026-07-29), and it
      never needed the iPhone: the iOS `CSVGenerator` runs headless in an XCTest on the Mac
      against the same `.bin`s pulled off the Pixel (debug build → `run-as` file pull).
      - `flight_20260729_163740.bin` (10.5 MB, the kept sim flight): Android CSV (converted
        on-device) vs iOS CSV — **same MD5** (`7f3bc346…`), 72,126,611 B each.
      - `flight_20260723_141100.bin`: the Pixel's cached CSV differed on every line — but it
        was generated by the 07-27 app build with **62 columns, pre-#623 Deployed Flag**.
        Regenerating with the CURRENT Kotlin `CsvGenerator` (pure JVM): **same MD5** as iOS
        (`4f100d81…`). Version skew of a cached artifact, not a live parity gap.
      Method trap for next time: `xcodebuild` swallows `print()` and reports TEST SUCCEEDED
      either way — verify by the output files, and hardcode paths (env-gated tests skip
      silently).
- [x] Stall path → **PASS, cleanly**. A 54.7 MB download interrupted by airplane mode: the
      app left the download, ran the ladder honestly (`Reconnecting (4/8)…(8/8)`), and left
      **no partial file** — `BinaryCache` unchanged and no CSV generated from the aborted
      transfer. A truncated `.bin` persisting as a valid-looking file would be the dangerous
      outcome here, and it doesn't happen
- [x] On-device bin→CSV → **PASS**. 72 MB CSV with **63 columns** (the #623 "Deployed Flag"
      count — the pin that broke CI during Phase 8, intact on a real conversion). JSON sidecar
      coherent and cross-checks the sim: max alt 457.4 m vs 459 predicted, burnout 261.6 m/s
      vs 262, burnout 1.501 s vs a 1.5 s burn; `fw_git_sha: d9a2679a`, `fw_dirty: false`.
      Exact byte-parity with iOS was separately proven in Phase 4
      (`flight_20260723_141100.csv` = 61,611 B, iOS-identical). NOT re-verified: the CSV's row
      count — `wc -l` over 72 MB through `run-as` returned nothing, likely a timeout
- [x] Bulk export / share → **PARTIAL** → issue #634. Single-file share works (sheet resolves
      the right file; FileProvider correct — the `Permission Denial` logcat lines are the
      system sheet failing to preview a non-exported provider, which is expected). But Android
      has **no multi-select at all**, where iOS has bulk delete in two views. Note the matrix
      wording was wrong: iOS's multi-select drives bulk *delete*, not export

## Group 6 — UI performance ⚠️ CRASH FOUND 2026-07-29

- [x] Dashboard frame timing → **PASS, with a Checkpoint-B watch item**. Under demo replay at
      flight rates: 245 frames/60 s, 50th 19 ms, 90th 24 ms, 99th 29 ms, **Missed Vsync: 0**.
      No dropped frames — but each recomposition costs ~19–24 ms of *UI-thread* work
      (`Slow UI thread: 200/245`), over the 16.7 ms budget for 60 fps. A Pixel 8 has the
      headroom; the Galaxy A15 at Checkpoint B is exactly where this would tip into visible
      jank, so measure it there.
      METHOD NOTE: measuring on the real board while it sits at LANDED gives ~1 Hz telemetry
      and ~50 frames/60 s — Compose correctly redraws only on change, so the "92% janky"
      figure that produces is an artifact of judging a 1 Hz UI against a 60 fps budget. Use
      **Demo mode** for this test; it replays at real flight rates.
      Only one rocket was in the roster (a second needs a relayed rocket via the BS)
- [x] Chart with a full flight CSV → **WAS A CRASH (#636), NOW FIXED + verified on-device**.
      Original finding: the app OOM'd opening the chart on a 72 MB CSV — `readText()` made a
      134 MB UTF-16 `String` against a 268 MB heap limit; reproduced twice
      (`APP CRASH(EXCEPTION)` at 433 MB and 312 MB RSS). Fix (merged #631): stream the file
      via `useLines` + decimate to a 100 Hz preview during parse + primitive `DoubleArray`
      columns. Same 72 MB CSV now charts in **~6 s** with no memory pressure; full-rate data
      stays in the CSV for PC analysis.
- [x] Map with offline tiles + prediction at 1 Hz → **PASS** (2026-07-29, bench sim flight
      with wifi + mobile data disabled; `Active default network: none` confirmed):
      - Saved a 5 km z10–16 region at the bench (684 tiles, 20 MB) via the Save-area flow;
        estimate said ~16 MB, actual 20 MB — close enough that the estimate is useful.
      - Offline, the map rendered fully: imagery, topo contours, street labels, rocket pin.
      - The sim's synthetic GPS teleports the fix to its canned California location
        mid-flight, and the map followed it there — and STILL rendered offline, from the
        **read-through tile cache** (those tiles were browsed in earlier checkpoint sittings;
        the saved region was 2,800 mi away). Both offline paths proven in one run.
      - Prediction banner live during descent, recomputed every second — sampled
        `landing ±148 m · T+0s ago` → `±68 m · T+0s` → `±36 m · T+0s`, uncertainty
        converging monotonically; after touchdown the banner froze and aged honestly
        (`±3 m · T+40s ago`, restyled orange as a staleness cue) and the map re-centred on
        the real bench fix once live GNSS resumed.
      - Zero MapLibre/tile/proxy errors in logcat across the whole offline flight.
- [x] Screen-on power draw over a simulated pad wait → **PASS** (2026-07-29): **~212 mA ≈
      5.0%/hr** screen-on at the live dashboard (telemetry streaming, BLE up, FGS running).
      Method: physically unplugged (`dumpsys battery unplug` only fakes the framework state —
      charge current keeps flowing), measured over wifi-adb via the coulomb counter:
      4,194,000 → 4,088,000 µAh in 30.05 min, slices 34/36/36 mAh per 10 min — flat, no
      drift. Battery reports 4,240 mAh full capacity. So: a 45-min pad wait ≈ 3.7%; a full
      8 h screen-on field day ≈ 40%; and Group 2 already showed the screen-OFF wait is
      near-free under Doze. Caveat: indoor bench brightness — field sunlight will draw more.

## Group 7 — Sensors and audio

- [x] Heading arrow vs a physical compass, 8 headings → declination applied, wrap-around
      correct at N — **PARTIAL: math verified, physical walk-around deferred to the outing**
- [x] Arrow behaviour when location permission is denied → stated, not silently wrong —
      **PASS** (covered in Group 1)
- [x] TTS announcements duck background audio and restore it — **PASS** (focus lifecycle in
      dumpsys + audible on the bench; was N/A: feature absent, #643)
- [x] Announcements still fire with the screen off — **PASS** (burnout + apogee spoke while
      Dozing; see 7.4 for the honest scope)

### 7.1 Heading arrow — what was actually verified

The bearing card needs **both** a phone fix and a latched rocket GPS fix before it renders an
arrow at all (`DashboardScreen` otherwise shows "Getting phone location…" / "Waiting for rocket
GPS"). So the 8-heading comparison is inherently outdoors with a rocket powered and locked — it
pairs with the shadow-phone outing, not the bench.

What a visual comparison would not catch anyway: **an arrow 359° wrong looks identical to an
arrow 1° wrong.** The failure modes here are sign and modulo errors, and they are invisible on a
round dial. So the arithmetic was lifted out of where it could not be tested — a
`SensorEventListener` callback and a Composable's rotation — into
`core/session/…/HeadingMath.kt`, with 13 unit tests. Both call sites now use it, so the tests
cover shipped code rather than a copy.

Covered: declination east/west/across-north (Kotlin's `%` keeps the dividend's sign, so a
westerly declination — New Jersey is ≈ −13°, the everyday case — would give −11° instead of 349°
without `mod`); full 0–359 × 5-declination range sweep; wrap-aware angular delta (359.5 → 0.5 is
1°, not 359°, which is what stops the publish guard from firing continuously while the phone
sits still pointing north); arrow sign relative to the nose; short-way-round; and the property
the physical test is really checking — **turn the phone 30° and the arrow must swing exactly 30°
the other way**, asserted around a full circle.

One correction worth recording: the arrow range is **[-180, 180)**, and the exact antipode
yields **−180, not +180**. My first two assertions had this backwards. Harmless for a rotation
(same direction), but don't assert +180.

Still open, for the outing: that the phone's magnetometer is *calibrated* and that declination
is being fetched for the actual launch site. Those are device/environment facts no unit test
reaches.

### 7.3 / 7.4 TTS — was absent, then ported (#643)

As found: Android had no `TextToSpeech`, `AudioManager`, `AudioAttributes`, `MediaPlayer` or
`SoundPool` anywhere — no audio output of any kind, while iOS has `FlightAnnouncer.swift` plus
dispatch tests, and the v1.0 launch-day loop names "voice". Filed as #643 (scope gap, not a test
failure), then ported the same day:

- **Policy** in `:core:session` (`FlightAnnouncer.kt`) behind an `AnnouncerSpeech` seam —
  15 flight-profile state-machine tests that iOS cannot run (its policy is welded to
  `AVSpeechSynthesizer`), plus the ports of iOS's #138 dispatch and #235 wording suites.
- **Engine** in `:app` (`TtsSpeaker.kt`): `TextToSpeech` + per-utterance
  `AUDIOFOCUS_GAIN_TRANSIENT_MAY_DUCK` (the Android equivalent of iOS's `.duckOthers` session).
- **Dispatch** is direct from `DeviceSession`, not via the telemetry StateFlow — a StateFlow
  dedups equal frames, and burnout detection counts *consecutive unchanged* max-speed frames.

On-device (Pixel, bench OC live): toggle → "Voice ready" spoken, logcat `Announcer` tag firing,
`dumpsys audio` showed the duck-capable focus grant during playback
(`GAIN_TRANSIENT_MAY_DUCK`, `USAGE_ASSISTANCE_NAVIGATION_GUIDANCE`/`CONTENT_TYPE_SPEECH`) and a
clean abandon after.

### 7.4 Sim-flight end-to-end (2026-07-29, bench OC, screen off at launch)

Firmware sim, app defaults (20 g / 40 N / 1.5 s burn / 5.0 m/s descent; est. 262 m/s, 459 m,
96 s). Every callout, from logcat:

```
21:14:10.563  Burnout. Max speed 262 meters per second     ← screen OFF (Dozing)
21:14:11.532  Apogee. 455 meters                           ← screen OFF (Dozing)
21:14:16.656  424 meters, descending 5 meters per second   (5.1 s after apogee ✓)
21:14:27.563  369 meters, descending 4 meters per second   (10.9 s ✓)
  … every ~11 s: 314, 259, 204, 149, 94 …
21:15:32.703  44 meters, descending 4 meters per second
21:15:39.674  9 meters, descending 4 meters per second     ← 5 s-cadence branch (see below)
21:15:44.744  -2 meters                                    (rate in deadband → no direction ✓)
21:15:46.741  Landed. 0 meters away
```

Checks that all passed:
- Spoken apogee **455 m came from telemetry** (`malt`), not the screen's 459 m estimate.
- First descent callout 5.1 s after apogee, then a clean 10 s cadence.
- "Landed. **0 meters away**" proves the launch-fix capture + haversine path ran (13 sats live,
  launch and landing at the same bench spot).
- No callout ever contradicted the motion (#235's cardinal rule) — including the near-ground
  frames where it historically did.

Two quirks, both **faithful iOS parity, not Android bugs**:
- The "9 meters" / "-2 meters" callouts arrive on the **5 s** altitude cadence, not the 10 s
  descent one: `alt_apo` clears below ~15 m AGL, which re-activates the pre-apogee branch.
  This is precisely the #235 scenario — iOS fixed the *word* ("descending", from the rate sign)
  but kept the re-fire, and Android reproduces both. The words were correct here.
- "-2 meters" is the baro reading slightly below field zero at touchdown; iOS would speak the
  same.

Screen-off scope, honestly: the phone was Dozing (power-button off) from launch until
21:14:13.5, when a `PULSING_SINGLE_TAP` (operator tap on the AOD) woke it — so **burnout and
apogee are the screen-off evidence**, descent ran screen-on. The mechanism doesn't distinguish:
TTS speaks from the FGS-pinned process regardless of display state.

---

## Exit

- [x] Every box above ticked or explicitly waived with a reason — **DONE 2026-07-29**. The
      one deliberate deferral: the physical 8-heading compass walk-around, which needs
      outdoors + a rocket GPS fix and rides with the shadow outing (its math is unit-tested,
      #644).
- [x] Numbers recorded in this file (throughput, frame timing, time-to-first-advert)
- [x] Bugs filed; blockers fixed; non-blockers triaged onto Phase 9 or the ledger — crashes
      #630/#636 fixed; parity gaps #633/#634/#635 fixed; scope gap #643 (announcer) ported
      and bench-flown; all closed.
- [x] Plan doc §4 updated — the shadow-outing line now reads one-phone-per-board (iPhone→BS,
      Android→OC); `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` on both boards is the hard limit.

**CHECKPOINT A: COMPLETE (2026-07-29).** Remaining for v1.0: the shadow-phone field outing.
