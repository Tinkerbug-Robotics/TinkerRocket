# Bench HIL tests

Hardware-in-the-loop regression tests that exercise real firmware on real
boards over BLE.  Run manually before flight days and on PRs that touch
LoRa / base station / logging code.

## `test_lora_log_capture.py` (issue #137)

End-to-end test that verifies the BS captures a complete LoRa CSV across a
simulated flight cycle.

### What it does

```
   laptop ─ BLE ─ base station ─ LoRa ─ flight computer (SIM mode)
```

1. Connects to the BS via BLE
2. Sends time-sync (cmd 9) so the new log uses a timestamped name
3. Optionally clears existing `lora_*.csv` for a clean slate
   (`--delete-existing`)
4. Sends sim config (cmd 5) + sim start (cmd 6) — the BS relays both to
   the flight computer via LoRa uplink
5. The flight computer's `SensorCollectorSim` feeds synthetic GNSS / IMU /
   pressure to the state machine, driving:
   `READY → PRELAUNCH → POWERED → COASTING → DESCENT → LANDED` (LANDED is
   terminal — post_flight_lockout holds until reboot/power-cycle, #317)
6. The rocket transmits real LoRa packets the whole time; the BS receives
   them and writes to SD
7. After the cycle, downloads the newest `lora_*.csv` via BLE
8. Asserts the file is complete, parseable, and shows the expected state
   sequence

### Hardware setup

| Item                | Required        | Notes                              |
|---------------------|-----------------|------------------------------------|
| Base station        | Powered, on SD  | LoRa antenna connected             |
| Flight computer     | Powered         | USB or 1-cell LiPo                 |
| Distance            | <5 m            | bench-to-bench is fine             |
| Laptop BLE          | Built-in or USB | tested on macOS + Linux            |

No GPS antenna or external sensors needed — the sim path bypasses real
sensors and provides synthetic data instead.

### Software requirements

```bash
pip install bleak    # tested with bleak 0.21+
```

Python 3.10 or newer.

### Two ways to run

#### A. Fully scripted (Python drives everything)

```bash
# Auto-discover BS named "TinkerBaseStation":
python3 tests/bench/test_lora_log_capture.py

# Specific BS by MAC:
python3 tests/bench/test_lora_log_capture.py --address AA:BB:CC:DD:EE:FF

# Wipe old logs first (recommended for repeated runs):
python3 tests/bench/test_lora_log_capture.py --delete-existing

# Shorter sim (faster turnaround):
python3 tests/bench/test_lora_log_capture.py --sim-burn 1.5 --sim-thrust 30
```

The Python script handles BLE pairing, time-sync, sim config, sim start,
state-machine observation, log download, and assertions.  No iOS device
needed.  Recommended for repeated runs and pre-flight regression checks.

#### B. iOS-driven (you launch the sim from the app, Python just verifies)

The Python script connects after the sim is done, grabs the resulting
log, and asserts on it.  Use this when you want to:

- Exercise the **same code path operators use in the field** (iOS app →
  cmd 5/6 → BS → LoRa relay → rocket).  This catches the
  iOS-side encoding bug that an all-Python test would miss.
- Manually choose sim parameters via the polished iOS Simulation view
  rather than CLI flags.
- Verify a specific past log without re-running the sim (see `--file`).

**Steps:**

1. Power up the BS (SD inserted) and the flight computer.  Let both boot.
2. Open the iOS app on your phone.
3. Pair with the BS in the app's BLE scanner.  On connect the app
   automatically sends time-sync (cmd 9) — confirm via the dashboard
   that battery/RSSI fields populate.
4. (Optional) From the Files view, delete any old `lora_*.csv` to
   simplify the post-run check.  Equivalent to `--delete-existing`.
5. Open the Simulation view (gear icon → Simulation, or the Sim tile on
   the dashboard).
6. Enter sim parameters and tap **Launch Sim**.  The app sends cmd 5
   (config) + cmd 6 (start) via the BS LoRa uplink.
7. Watch the dashboard.  The rocket state field should sweep:
   `READY → PRELAUNCH → INFLIGHT → LANDED`.  Total ~30-60 s
   depending on burn / descent parameters.  The rocket stays LANDED
   until rebooted (#317) — that is the expected end state.
8. After the state shows LANDED, **wait ~10 s** for the BS to flush
   the LANDED-close fsync to disk.  The BS will auto-open a *second*
   log file ("log B") for post-landing telemetry — that file stays
   open until 5 min of LoRa silence elapses
   (`LOG_SILENCE_TIMEOUT_MS = 5 * 60 * 1000`).  The iOS toggle button
   will reflect that: it stays at "Stop Logging" until log B closes.
   The Python verifier targets log A (the file active during INFLIGHT,
   not the post-landing log B) via `Capture.inflight_active_file`.
9. **Disconnect** the iOS app from the BS — either kill the app or
   toggle airplane mode briefly.  The BS only accepts one BLE
   connection at a time, so the Python verifier needs the slot free.
10. Run the verifier:

    ```bash
    python3 tests/bench/test_lora_log_capture.py --skip-sim
    ```

    Pass = exit 0, prints `[pass] BS LoRa log captured the simulated
    flight end-to-end` plus a row-count and peak-altitude summary.

11. To re-verify a specific file (e.g. after the bench is gone and you
    saved the log file separately, or when iterating on assertion
    logic):

    ```bash
    python3 tests/bench/test_lora_log_capture.py --skip-sim \
        --file lora_20260512_143000.csv
    ```

#### Help

```bash
python3 tests/bench/test_lora_log_capture.py --help
```

### What the assertions catch

| Assertion                              | Bug it would have caught (#137 context) |
|----------------------------------------|----------------------------------------|
| Newest `lora_*.csv` exists             | "no log on disk after flight"          |
| File size > 200 bytes                  | "empty file, only header"              |
| State sequence has PRELAUNCH/INFLIGHT/LANDED | "Journey 75 pattern" (log opens at LANDED only) |
| Peak `max_alt` > threshold             | rocket telemetry not actually captured |
| No INFLIGHT inter-row gap > 5 s        | silence-close-during-INFLIGHT (Fix A regression) |

### Limits / what this test doesn't cover

- **SD card flakiness mid-session** — would need a glitching tool or a
  flaky-by-design test card.  The current path catches "SD never worked
  this session," not "SD wedged at minute 5."
- **Multi-rocket BS state interactions** — only one flight computer in the
  loop.  Multi-rocket regressions need a second rocket on the bench.
- **LoRa range loss** — both boards are close together.  To simulate
  long-range altitude dropouts, power-cycle the flight computer mid-sim
  (manual, not scripted).
- **GNSS lock semantics** — `SensorCollectorSim` provides a synthetic
  `num_sats=4` immediately, so the READY→PRELAUNCH transition fires within
  a few seconds.  Real-world GPS-cold-start regressions aren't caught here.

### Exit codes

- `0`   — all assertions passed
- `3`   — BLE scan found no base station
- `4`   — connected but received no telemetry within 10 s
- `5-7` — state-machine timeout (PRELAUNCH / INFLIGHT / LANDED)
- `9`   — BS didn't respond to file-list request
- `10`  — no `lora_*.csv` on the SD after the cycle
- `11`  — newest log is suspiciously small
- `12`  — file download timed out
- `20+` — CSV content assertions failed (see stderr for which)
- `130` — interrupted (Ctrl-C)
