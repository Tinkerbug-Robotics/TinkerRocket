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
| `score_heading.py` | The heading from thrust (0x9F): each fit on the flight's time, its 1-sigma, the hand-off and its cost; with `--ref`, two runs' turns side by side |
| `truth.py` | Loads a flight's truth and scenario for the scorers |
| `run_v9.sh` | One run on the V9 with the mosaic-G5 (below): the MAC check, the rig marked busy, the FC reset and its console, the sky conducted, the run voided if the replay clock is late |
| `score_v9.py` | Scores a V9 run: its console alone, plus the OC's log if downloaded, plus that log through the offline harness on request |

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
   python3 tools/rig_replay/score_log.py OUTDIR/flight_YYYYMMDD_HHMMSS.bin --blob /abs/path/sim_replay.bin
   ```
   - `--blob` is the replay file the image was built with: the log's clock is put on the flight's time by where its accelerometer crosses 20 m/s² at ignition. Without it, that edge is worked out from the flight's CSV as `sim_replay_blob.py` writes it with its default `--from`.
   - Another flight: `--flight sim_<id>`, or `--csv` and `--scenario`.
7. **Put a flight image back on the FC.** The replay image is for the bench only.

## The heading from thrust

The fit (TR_RawGnssShadow/HeadingFit.h) runs in observe mode in every LC86G image; its records (0x9F) are in the
OC's log. Two bench flags go with the replay image (step 2):

- `-DTR_HEADING_FIT_BENCH=1` times the fit at the raw-GNSS task's start: the console's `[RAWGNSS] heading fit
  bench:` lines, a few seconds before the filter starts.
- `-DTR_BENCH_HEADING_ERROR_DEG=N` turns the pad's heading N deg (the declination's), for the fit to find. With
  `-DTR_HEADING_FIT=2` (act) the filter is handed the fit's heading too, once its 1-sigma is 10 deg.

The same replay flown with and without the error: the fits' turns should differ by -N, whatever the
magnetometer's own error:
```
python3 tools/rig_replay/score_heading.py OUTDIR_B/flight_....bin --ref OUTDIR_A/flight_....bin --error 40 \
    --blob /abs/path/sim_replay.bin
```

## Reading the results

- **The console's "up" is relative.** It's the filter's height above its own pad reference, so an absolute height error on the pad only shows in `score_log.py`, in the raw filter's ellipsoidal records.
- **The receiver isn't repeatable on the rig.** In 2026-10 the LC86G lost the rig's sky at different points in different runs (on the pad, at launch, under the drogue), with no command from the FC. Check each run's fix count over time (`[REPLAY] ... fixes=`) before comparing two runs.
- **One run per boot.** The replay plays from boot as a sim, with the pyros in dry-fire, then holds the landed state until a reboot.

## The V9 with the mosaic-G5

The Goblin's V9 flies the raw-GNSS filter as its one navigation filter, fed by a Septentrio mosaic-G5 (the
mosaic-only image, `-DTR_GNSS_MOSAIC=1`). Its rig replay is the same idea as the Beetle's, with these differences:

- **The replay's clock comes from the receiver's timed raw epochs**, ten a second (`ReplayClock::onEpoch`). The
  mosaic's driver hands no timed fix. An epoch counts only when the driver hands it over timed: the 1PPS line valid,
  the receiver FINETIME, an edge paired within 120 s, and (with the time-tag fix, `MosaicTimeTag.h`) a clock bias
  carried to it and the tag confirmed by the 1PPS. An untimed or held epoch carries `mcu_us` and `tag_dtr_ms` 0 and
  sets nothing. After a cold start the first timed epoch comes about 70 s after the sky starts (10/17 pad check), so
  around file second 70, t = -110 s. The clock must arm before T-5 s (file second 175), or the flight never starts.
  The clock takes each epoch's GPS instant (the receiver's time of week less the clock bias the driver's tag took
  off, `TimedRawEpoch::tag_dtr_ms`), so the replayed IMU sits on the sky's GPS time.
- **The freeze is checked against the 1PPS edge.** Before the time-tag fix the driver's clock line followed a sky
  that came late (a transmitter underrun) only after about a minute, while the edges follow it at once. With the fix
  the line steps onto its edges and, after such a slip, the tag is held (the epochs untimed) until the 1PPS confirms
  two seconds in a row: a slip shows as a few seconds of untimed epochs, not as wrong tags. The check stays as the
  freeze's guard: the first 1PPS check of a second from the freeze on (by T-2 s) confirms the freeze, and where the
  edge puts that second more than 200 us from the frozen clock, the clock moves onto the edge, on the pad
  (`ReplayClock::onTagCheck`). It takes the driver's last residual with its own second (`PpsTagStats::last_s`), a
  second handed over timed; a held second's check is the driver's alone. The console says which: `RIG REPLAY clock:
  the freeze checked against the 1PPS edge ... kept` or `... MOVED`, with the driver's check counts at that moment
  (and, with the fix, the seconds held back). A hold across T-5 s keeps the clock following the old sky until the
  first timed epoch after the release, which freezes it on the new sky's time; after T-2 s that freeze is
  unchecked, and `score_v9.py` marks it. Epochs the filter got untimed between the freeze and the landing are a mark
  too: it flew those without GNSS.
- **The receiver cold-starts at every boot** (`erst,Soft,PVTData+SatData` after its setup). This is on in every
  replay build of the mosaic image, through the bench switch `TR_BENCH_MOSAIC_COLD_START`. The receiver then sets
  itself up a second time, about 5 s after boot, and `run_v9.sh` transmits once it has.
- **The ionosphere: the rig sky's own, carried in the build.** The rig plays a recorded day, 2026-08-29, and its
  files never broadcast GPSIon: subframe 4 page 18 comes once every 12.5 min, and the three flights' files (TOW
  573606 on, 241-257 s) hold pages 21-25 and 1-4 only. So the build carries the scenario's own Klobuchar, the eight
  broadcast integers of its RINEX header (`-DTR_SIM_REPLAY_GPS_IONO=$(tools/rig_replay/rinex_gps_iono.py
  RINEX)`: `14,3,-1,-2,51,6,-2,-5` for the day-241 file all three flights use), which SignalSim delays every signal
  by. The sim hands it to the filter as the receiver's GPSIon, dated the sky's day. The replay's rule
  (`RawIonoRule::offerRigReplay`) takes no set before the sky's day is known (the NVS set read at boot) and none
  dated more than a day from it (a pad calibration's from today), and an app set never displaces the receiver's.
  The boot line `RIG REPLAY ionosphere:` says what the image carries; `score_v9.py` flags a set in use at launch
  that is not the sky's. Only the pseudoranges carry it, a few metres; the Doppler and the attitude do not.
- **Lines to score.** `[REPLAY-NAV]` prints 50 times a second from launch to burnout, 10 a second after, and once a
  second on the ground, stamped with the navigation state's own instant (the IMU tick it was carried to). It
  carries the attitude, velocity and position, the four apogee voters, the call, the backstop, the GPS admission,
  the raw vote guard and the transonic lockout. `[REPLAY-VOTE]` prints every change of those flags (at most 50 a
  second, with a count of any not printed), so a vote of a few milliseconds is seen and each voter's latency is
  the loop's own. So the console alone scores a run.
- **The high-g channel.** Where the low-g rail hands over (rp5_0829 rides it for 48 ms at 18.8 g, eagleclaw_0517
  for 22 ms), the converter takes the high-g channel less the V9's NVS high-g bias; the replayed high-g counts
  carry that bias (`SimHighGBias.h`), so the flight's own force comes out.
- **Conducted, not radiated.** The PortaPack Mayhem (HackRF) feeds the mosaic's SMA through the pad stack and a DC
  block. The mosaic-go powers its antenna port. The receiver's own USB is never connected.

### Build and stage (bench only)

```
python3 tools/sim_replay_blob.py $COCOM_SDR/scenarios/sim_rp5_0829_sensors.csv.gz \
    $COCOM_SDR/scenarios/real_sim_rp5_0829_sky3d_pad180.json /abs/sim_replay_rp5_0829.bin
cd tinkerrocket-idf/projects/flight_computer
idf.py -B build_rig_rp5_0829 -DTR_BOARD_V9=1 -DTR_GNSS_MOSAIC=1 -DTR_SIM_REPLAY_BLOB=/abs/sim_replay_rp5_0829.bin \
    -DTR_BENCH_MOSAIC_CN0=1 \
    -DTR_SIM_REPLAY_GPS_IONO=$(python3 ../../../tools/rig_replay/rinex_gps_iono.py $COCOM_SDR/signalsim/EphData/BRDC_2026241_MN.rnx) \
    reconfigure
cmake --build build_rig_rp5_0829 -j 4
```

- The image is about 2.1 MB with the 1.28 MB blob, inside the 3 MB `ota_0` slot.
- Build the CPU profile variant with `-DTR_LOOP_PROFILE=1` (`[LP]`). Then, after `reconfigure`, append
  `CONFIG_FREERTOS_USE_TRACE_FACILITY=y` and `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` to the build dir's
  `sdkconfig` (`[CPU]`). Both perturb timing, so keep them out of the scored runs.
- The build is marked `-bench`. It plays a canned flight from boot as a sim, with every pyro dry.
- It is refused on the mosaic image built without the filter (no epochs to clock from) and on any board but the M1
  and the mosaic image.
- The staged images, their blobs and the exact commands are in `firmware-ota/v9-rig-replay/<flight>/`.

### A run

1. **Flash the FC only, after the MAC check**, with the owner's OK. `ioreg -p IOUSB -l -w0 | grep 'USB Serial
   Number'` must show the FC, 80:F1:B2:D0:94:A7. `run_v9.sh` never flashes.
2. **Fly it:** `tools/rig_replay/run_v9.sh sim_rp5_0829 7 r1`
   - The arguments are the flight, the conducted TX gain and the run's tag. On 10/17, `-x 3` read GPS 40 dB-Hz on
     the pad.
   - The script refuses while a build runs, and while `RIG_STATE` says another session has the rig.
   - It finds the FC's port from the USB registry without opening any (`usb_ports.py`: the port under the device
     whose serial is the FC's MAC; the PortaPack the only other device allowed), again before the reset and after
     the boot, and marks the rig active.
   - It resets the FC and logs its console (`tools/bench_capture_boot.py`). It checks that the image carries this
     flight, waits for the cold start to be queued and the receiver's second setup, and checks the IMU reads at the
     flight's rate (about 3780 a second: 4k Dynamic) and GNSS logging is Detailed. Then it transmits.
   - The run is **VOID** if the replay clock is not on the receiver's by file second 170, or says MISSED; if the
     console capture ends or the FC boots again before the file is done; or if the transmitter underran (or fell
     short on the host) after the freeze, failed, or stopped short of the file (`tx_log.py` puts each underrun on
     the file's timeline). The transmission stops at a VOID found during it (`KEEP_ON_VOID=1` keeps it going).
   - After the file it keeps 120 s of console, then leaves the radio idle and the rig idle, on any exit, and says
     the FC still runs the RIG REPLAY image: **flash the flight image back before any flight.**
   - The run folder (default `gnss-attitude-study/rig-replay/v9-runs/run_TAG` beside the repository, never inside a
     checkout: the console carries the rig pad's position) holds `fc_console.txt`, `hackrf.txt`, `tx.json`,
     `run.json` (image version and ionosphere, IMU rate, blob SHA-256, gain, times, the transmitter's verdict, void)
     and `VOID` when void.
   - `DRY_RUN=1` prints the plan after the checks.
3. **Download the OC's log** if the run needs more than the console. Use `tools/ble_flight_download.py`, with
   `TARGET_NAME` set to "RC V9". At about 221 kB/s of log, a run is 20-30 MB, about 10-15 min over BLE. **Delete the
   rig logs from the Goblin's OC before 10/23.**
4. **Score it:**
   ```
   python3 tools/rig_replay/score_v9.py RUN_DIR [--log OC_LOG [--harness build/raw_replay_mosaic]]
   ```
   - First whether the run counts: run_v9.sh's verdict, the transmitter's slips after the freeze, a second boot
     before the landing, and any second the 1PPS tag check judged over 200 us from the freeze's check to the
     landing (the filter's own tags off the sky and the IMU while the driver's line lagged a slip). A VOID run's
     flight is not scored; `--score-void` scores it for diagnosis only.
   - The attitude error per phase is the plan's (e = 2 acos|q_est . q_true|): p90, p99 and max for boost, coast
     and descent, plus the pitch error within 2 s of the apogee.
   - Velocity and position errors per phase.
   - Each apogee voter from the `[REPLAY-VOTE]` changes: its first vote, its latency against the true apogee, the
     true vertical velocity then, and any vote while still climbing at more than 2 m/s (G5's hard bar). From an
     image without those lines G5 is not judged: 10 Hz samples miss a 6 ms vote.
   - The epoch age and build, the filter's step and core-0 share, `[CPU]`/`[LP]`, the loop and IMU rates.
   - The PPS tag check, what the receiver tracked through each phase, and the replay clock.
   - With `--log`: the on-board records (0x96) on the same metrics, each voter's NonSensor flag, the receiver's own
     PVT against the truth, its clock drift (about +0.1 ppm on the rig) and its bias at ignition.
   - With `--harness`: the log through `raw_replay_mosaic` from the on-board filter's own start
     (`extract.py --sbf --rig`), aided and unaided (att_gnss_off), scored against the plan's section 4 bars (the
     attitude p90 per phase, the pitch near apogee, the call's latency, vz at the true apogee, G5's hard bar, no
     boost/coast p90 more than 5 deg above the twin) and against the on-board records. Both twins start from the
     aided filter's state about 0.4 s before launch (the OC log has no pad), so this is GNSS's effect in flight only,
     not on the pad alignment.
5. **Afterwards** (plan risk 10):
   - Cold-start the receiver once more, then give it 15 min of real sky before the Goblin's pad. It otherwise
     carries the rig's day, place and ephemerides.
   - Put a flight image back on the FC.
   - Set the IMU back to 4k Dynamic and GNSS logging to Detailed.

### The rig's own limits on these runs

- **C/N0 tops out at about 38-40 dB-Hz whatever the gain.** The real sky reads GPS 44-46 at mid and high
  elevation. The receiver keeps GPS through the boost but loses most satellites under the drogue on these skies.
  Read `score_v9.py`'s tracked table before blaming the filter for a descent.
- **Transmitter underruns.** The LC86G runs logged 2-7 underruns a run, some of 10-18 ms. One after the freeze voids
  the run. One in the minute before it leaves the driver's line lagging into the flight, which the 1PPS tag check
  then shows and the scorer voids on. Keep the Mac quiet (no builds) while transmitting.
- **The pad is not in the OC log.** It records from launch, with about 0.4 s of prelaunch ring. The pad's numbers
  come from the console.
- **The skies are not the Goblin's motor.** These are the three flights with full truth: rp5_0829, eagleclaw_0517,
  and gtv67_0509 (the windy one).
