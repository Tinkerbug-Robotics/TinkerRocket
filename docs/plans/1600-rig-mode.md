# Rig mode in the flight image (design, #1600)

## Why

The rig replay works (#1620, flown r1–r6). But it runs a bench-only build, and that has two costs.

- **The image flown on the rig isn't the image that flies.** A `TR_SIM_REPLAY` build compiles the flight's sensors into the firmware. Every rig result describes a build that never goes on a rocket.
- **Every bench session ends in a reflash.** If someone forgets, the FC is left on a replay image. On 2026-10-13 it was.

In rig mode, the flight image carries the *ability* to replay. The flight's data lives in its own flash partition, and the bench arms exactly one replay, for the next boot.

## What stays the same

- The replay file: the TRRP v1 format, written by `tools/sim_replay_blob.py`.
- The playback and its clock (`SimReplay`). It follows the receiver's GPS time on the pad and freezes 5 s before ignition.
- Running as a sim: the dry-fire gate holds, and the log and telemetry say sim.
- The rig side, and the scorers in `tools/rig_replay/`.

## Design

### 1. Storage: a `replay` partition

Add one line to `partitions.csv`:

```
replay,   data, 0x40,    0x610000,0x1F0000,
```

- This is the top 1.94 MB of the 8 MB the table declares.
- rp5_0829 from t = −30 s is 1.3 MB, so the partition holds about 150 s at the same rate.
- `tools/check_flash_declarations.py` still passes, because the table ends at exactly 8 MB.
- **Layout:** sector 0 (4 KB) holds the arm record, and the replay file starts at 0x1000.
- **Adding it:** a new partition table needs one full USB flash; OTA can't add it. That's fine, since rig mode is for bench units.

### 2. Arming: one boot, by the bench, over USB

The arm record contains the magic `TRRA`, a version, the file's length and CRC-32, and a 16-byte name.

- **Arming:** `tools/rig_replay/arm.py PORT FILE` writes the file (only if it changed) and then the arm record, using `parttool.py write_partition`. `run.sh` then resets the FC.
- **At boot:** the flight image reads sector 0. If it's armed, the boot proceeds in this order:
  1. Erase the record first. One arming gives one replay, whatever happens next, and never a second boot.
  2. Check the CRC.
  3. Map the file with `esp_partition_mmap`.
  4. Start the replay exactly as a `TR_SIM_REPLAY` build does today.
- **Every normal boot:** nothing happens. The cost is one 4 KB read, and a board whose table has no `replay` partition logs "no rig mode" once.
- **Why USB:** at the bench the FC is already on USB for its console. Physical access is the authorization. No BLE or LoRa command can arm it, so nothing in the app or the radio link can put a rocket into a sim.

### 3. Interlock: the rig's sky or nothing

Even when armed, the replay plays only against the flight's own GPS time.

- **The rule:** the receiver's first fix must fall inside the file's window, from its start − 60 s to its end. Otherwise the replay stops at once, the sim ends, and the FC carries on as a flight computer.
- **At a field:** the real sky's GPS time is days or years from the file's. A replay armed by mistake can't survive the first fix.
- **Before the first fix:** the FC holds the replay's pad, as today, so telemetry says SIM. A board armed by mistake shows SIM in the app until its receiver's first fix, about a minute after a cold start, and then drops it.
- **Restored flights:** as now, the replay never starts over one (`rocket_state != INFLIGHT`).

### 4. What the flight image gains

- `SensorCollectorSim::startReplay()` reads the partition instead of the build's `_binary_sim_replay_bin` symbol. `SimReplay::load(ptr, len)` already validates the file.
- The arm record's parse and consume, written as pure code and host-tested.
- The GPS-window check in the replay clock, also pure and host-tested.
- **On the tools side:** `tools/rig_replay/arm.py`, and `run.sh` calls it.
- `TR_SIM_REPLAY` builds stay for one release, then go.

### 5. Flight-image risk

| What | Risk | Bound |
|---|---|---|
| The boot read of sector 0 | none in flight; a missing partition is "no rig mode" | 4 KB, once |
| The erase at boot | only when armed | one sector |
| The replay path | the code r1–r6 ran | unchanged |
| A mistaken arming | the rocket boots in sim on the pad | ended by the first fix (§3), shown as SIM in the app until then; the dry-fire gate holds |

## The bench, in rig mode

1. Once per flight: `python3 tools/sim_replay_blob.py SENSORS_CSV SCENARIO_JSON sim_replay.bin`.
2. Each run: `python3 tools/rig_replay/arm.py PORT sim_replay.bin`.
3. `tools/rig_replay/run.sh PORT 46 r7`.
4. Download and score as now.

The FC keeps the flight image throughout.

## Tests

- **Host:**
  - The arm record: armed, unarmed, bad CRC, short, and consumed (a second boot doesn't replay).
  - The GPS-window interlock: inside, outside, and before the first fix.
  - `SimReplay` from a mapped buffer, using the existing tests.
- **Bench:**
  - Arm and run r7 on the flight image.
  - Boot unarmed: no sim.
  - Arm with a file whose window doesn't match the rig's (another flight's): the replay stops at the first fix.

## Decisions for the owner

1. The `replay` partition on the Beetle (M1) only, or on every board's table?
2. USB-only arming, as proposed, or also a BLE command for a sealed cage? The OC answers through the cage at about −82 dBm, so it would be possible, at the price of a remote way into sim mode.
3. Retire the `TR_SIM_REPLAY` builds once rig mode has flown on the rig?

**Effort:** about 1–2 days: the partition, `arm.py`, the firmware's load path and interlock, the tests, and one rig run.
