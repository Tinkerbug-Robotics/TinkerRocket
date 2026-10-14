# Rig replay: a flight on the bench, sensors and GNSS together

The rig replay flies one flight on the bench, end to end (#1600, #1620):

- The flight computer runs a `TR_SIM_REPLAY` build. It plays the flight's IMU, barometer and magnetometer through the sensor sim's path, timed by the receiver's GPS time.
- At the same time the COCOM rig (HackRF/PortaPack into a Faraday cage) plays the same flight's GNSS signals to the Beetle's LC86G.
- So the raw-GNSS filter, the physics checks and the flight logic all see one consistent flight, and the result can be scored against its truth.

For how it works, see "The rig replay" in [docs/plans/1600-realtime-raw-gnss.md](../../docs/plans/1600-realtime-raw-gnss.md).

| File | What it does |
|---|---|
| `run.sh` | One run: resets the FC and logs its console, transmits the flight's IQ file, then returns the radio to Mayhem |
| `console.py` | Resets an ESP32-S3 over USB and logs its console with host timestamps |
| `score_console.py` | Scores the console's `[REPLAY]` lines (the navigation filter's ENU, once a second) against the truth, by phase |
| `score_log.py` | Scores the OC's flight log against the truth: the raw-GNSS filter's records (0x96), the logged navigation state, the receiver's own fixes, and the physics checks (0x9A) |
| `baro_port_check.py` | The barometer's error through the flight beside the filter's static-port model |
| `truth.py` | Loads a flight's truth and scenario for the scorers |

## What you need

- **The gnss repo** ([TinkerRocket-GNSS](https://github.com/Tinkerbug-Robotics/TinkerRocket-GNSS)), `cocom-rig/sdr`: this repo's `gnss` submodule, or set `COCOM_SDR` to it.
  - `hackrf_tx_ram/` built (`build.sh`) and `ensure_hackrf.py`.
  - The flight's files from `sim_flight_3d.py`: `scenarios/sim_<id>_sensors.csv.gz`, `scenarios/real_sim_<id>_sky3d_pad180.json`, and the IQ file `c8/signalsim_real_sim_<id>_sky3d_all_45_w_p180.C8` (SignalSim; about 9.5 GB).
  - `sim_flight_3d.py` and the `-N DB@SECONDS` outage come with TinkerRocket-GNSS#7.
- **ESP-IDF 6.0**, the HackRF tools, and Python with numpy and pyserial (plus bleak for the download).
- **The Beetle in the cage**, its FC on USB. Its OC must be reachable over BLE afterwards to download the log.

## A run

1. **Make the replay file** from the flight's sensors and scenario:
   ```
   python3 tools/sim_replay_blob.py $COCOM_SDR/scenarios/sim_rp5_0829_sensors.csv.gz \
       $COCOM_SDR/scenarios/real_sim_rp5_0829_sky3d_pad180.json /abs/path/sim_replay.bin
   ```
2. **Build the replay image** (bench only). Then revert `dependencies.lock`.
   ```
   cd tinkerrocket-idf/projects/flight_computer
   idf.py -B build_replay -DTR_BOARD_M1=1 -DTR_SIM_REPLAY_BLOB=/abs/path/sim_replay.bin build
   ```
3. **Flash the FC, and only the FC.** The Beetle's FC and OC both enumerate on the same USB port, so check which chip is there before writing:
   - `ioreg -p IOUSB -l -w0 | grep 'USB Serial Number'` shows the chip's MAC without opening the port.
   - Then: `idf.py -B build_replay -p PORT flash`.
4. **Fly it:**
   ```
   TX_NOISE=30@220 tools/rig_replay/run.sh PORT 46 r1
   ```
   - This takes about 6.5 minutes.
   - `TX_NOISE` is optional: here it buries the signal from file second 220, which is +40 s, under rp5_0829's main.
   - `DRY_RUN=1` prints the commands without running them.
   - Don't run builds or other heavy jobs while it transmits: an underrun slips the whole sky.
5. **Download the OC's log** over BLE: `tools/ble_flight_download.py fetch OUTDIR FILE`, with `TARGET_NAME` set to the unit's name. It takes about 9 minutes for 18 MB through the cage.
6. **Score it:**
   ```
   python3 tools/rig_replay/score_console.py rig_runs/run_r1/fc_console.log
   python3 tools/rig_replay/score_log.py OUTDIR/flight_YYYYMMDD_HHMMSS.bin
   ```
7. **Put a flight image back on the FC.** The replay image is for the bench only.

## Reading the results

- **The console's "up" is relative.** It's the filter's height above its own pad reference, so an absolute height error on the pad only shows in `score_log.py`, in the raw filter's ellipsoidal records.
- **The receiver isn't repeatable on the rig.** In 2026-10 the LC86G lost the rig's sky at different points in different runs (on the pad, at launch, under the drogue), with no command from the FC. Check each run's fix count over time (`[REPLAY] ... fixes=`) before comparing two runs.
- **One run per boot.** The replay plays from boot as a sim, with the pyros in dry-fire, then holds the landed state until a reboot.
