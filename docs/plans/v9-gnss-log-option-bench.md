# V9 GNSS log option: what is not bench-tested

**Written 2026-10-17, branch `fc/v9-gnss-log-option`**, built on the V9 GNSS
auto-detect work (`fc/gnss-autodetect`). Not for the Goblin's 2026-10-24
flight, which flies the mosaic-only image from `fc/mosaic-g5`. Nothing here has
run on hardware: host tests, app tests and builds only.

## What it is

A setting in the app, under the IMU logging rate: **GNSS Logging, Detailed or
Reduced** (BLE cmd 79). It matters only with a Septentrio mosaic-G5 on the V9.

- **Detailed** (the default, today's behavior): the flight log keeps every SBF
  block the mosaic sends, and the IMU logs at up to 4k (3840 Hz). 8k Dynamic
  flies as 4k Dynamic, and the setting is kept.
- **Reduced**: the log keeps PVTGeodetic at 20 Hz, the status and navigation
  blocks, and MeasEpoch and MeasExtra once a second (the epoch on each whole
  GPS second). The IMU gets the board's own limit, so 8k Dynamic flies.

Either way the receiver streams everything to the FC, which parses all of it,
and to its own microSD card. Only the FC's log copy is cut, in the flight loop
(`TR_RocketComputerTypes/gnss_log_policy.h`). The receiver's setup is the same.

The OC and the FC each keep the setting in NVS (`gnsslog`/`set`), as with the
IMU rate. The OC reports it as `glog` in the config readback, moves `irmax`
with it, and sends the readback again when `irmax` moves, so the app's rate
menu follows. The FC ignores a change while INFLIGHT. The app locks the picker
in flight, and on firmware whose readback has no `glog`.

## Budget (host tests, `test_i2s_link_budget`)

The V9 in a guided coast, the SBF budgeted at 130 signals:

| Flight | IMU | FC->OC link | Log, against the OC's ~310 kB/s flash drain |
|---|---|---|---|
| u-blox | 7680 Hz | 66.6 % | 282 kB/s, 91 % |
| mosaic, Detailed | 3840 Hz | 66.2 % | 257 kB/s, 83 % |
| mosaic, Reduced | 7680 Hz | 69.1 % | 291 kB/s, 94 % |
| mosaic, every block at 7680 Hz (not allowed) | 7680 Hz | 92.1 % | 373 kB/s, 120 % |

The log model gives the Goblin's measured 221 kB/s (3840 Hz with the full SBF,
2026-10-16) within 1 kB/s. The ~310 kB/s drain is from a July bench on an
earlier board. The V9's own has not been measured, and Reduced at 8k sits at
94 % of it.

## Not bench-tested

1. **The round trip with the mosaic.** The V8 run (below) covers the command,
   the OC to FC frame, both NVS copies across a power cycle and `glog` in the
   readback, all with the u-blox. With the mosaic, `irmax` should also move
   (3840 to 7680) and the 8k rates appear in the app's menu a moment after
   Reduced is picked. The app itself was not used: the bench drove BLE from
   the Mac.
2. **8k with the mosaic.** The FC at 7680 Hz while the mosaic driver parses
   the full 20 Hz SBF has never run. Watch the loop rate, `[GAP DIAG]
   imu_q_drops`, the FC's I2S enqueue drops, the OC's `rx_ovf` and ring peak,
   and the 30 s `mosaic:` status line (ring drops, UART overflows).
3. **The OC's flash at 8k.** A minute or more of 8k Dynamic with Reduced
   against the V9's real NAND drain: LogBufferStats, ring peak, no drops.
4. **The cut on a live stream.** `extract_gnss_sbf.py` on that log should show
   MeasEpoch at 1.00 Hz with a MeasExtra for each, and PVTGeodetic at 20 Hz.
   The host test runs a recorded epoch through the cut, not a live receiver.
5. **Switching on the pad.** The IMU rate changes live when the limit moves,
   as with cmd 67. The INFLIGHT refusal is untested.
6. **Mixed versions.** An app without the setting never sends cmd 79, so the
   rocket stays Detailed. This app with older firmware locks the picker.

## Results on the V8, 2026-10-17

The V8 ("RC V8") stands in for a u-blox V9: the same SAM-M10Q on the same pins,
but no mosaic and no 8k (it tops out at 3840 Hz). So it tests the setting's
round trip, not the cut or 8k. Images from this branch over USB (the V8 has one
USB port and a switch between the OC and the FC): OC
393fed6c-dirty-v8+20261017-1104 (MAC e0:72:a1:ca:e4:24), FC auto image
393fed6c-dirty-v8+20261017-1106 (`-DTR_BOARD_V8=1 -DTR_GNSS_AUTO=1`, MAC
30:ed:a0:e3:6a:5b). BLE from the Mac (`tools/bench_ble_cmd.py`), FC console
over USB. Before: OC ...-0940 and FC ...-0938 (the auto-detect bench images),
readback with no `glog`.

| Step | Result |
|---|---|
| New OC, old FC | readback `"irate":0,"irmax":3840,"glog":0` |
| New FC, first boot | `GNSS log: detailed (every SBF block)`, `[GNSS] Receiver: u-blox; IMU rates up to 3840 Hz`, loop ~980/s |
| cmd 79 = 1 | FC: the frame (`AA 55 AA 55 8F 01 01 ...`), then `[CFG] GNSS log: reduced (persisted)`; readback `glog` 1; `irmax` stays 3840 (u-blox) |
| cmd 79 = 2 | refused by the OC: `glog` stays 1, nothing reaches the FC |
| Power cycle (cmd 8 off, 24 s, on: the OC resets, the FC rail drops) | OC readback `glog` 1; FC boot `GNSS log: reduced (a mosaic-G5's raw measurements once a second)` |
| cmd 79 = 0 | FC `[CFG] GNSS log: detailed (persisted)`; readback `glog` 0 |

The V8 stays on these images, at the user's choice, with the setting back on
Detailed.

## Bench steps

The Goblin's V9 with the mosaic, FC console on USB (check the USB serial
number, the MAC, before flashing), FC and OC images from this branch.

1. Boot on the default. FC console: `GNSS log: detailed (every SBF block)`,
   then `[GNSS] Receiver: mosaic-G5; IMU rates up to 3840 Hz`. App: Detailed,
   no 8k rates.
2. Pick Reduced. FC: `[CFG] GNSS log: reduced (persisted)` and `[CFG] with
   the mosaic-G5, IMU rates up to 7680 Hz now: ...`. OC: `GNSS log reduced
   with the mosaic-G5: IMU rates up to 7680 Hz`. App: the 8k rates appear.
3. Pick 8k Dynamic and record a minute (items 2 to 4 above).
4. Power-cycle. Both keep Reduced: the FC prints `GNSS log: reduced ...` at
   boot.
5. Pick Detailed. The 8k rates go away, and 8k Dynamic flies as 4k Dynamic
   (`irate` 0). Pick Reduced again: 8k Dynamic is back.

## Not done

- The flight log does not record the setting itself; a reader sees it in the
  SBF (MeasEpoch at 1 Hz). FlightSettingsData has no flag bit left, and a new
  field would change its length for every log reader.
