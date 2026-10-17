# raw_replay: the Beetle's raw-GNSS navigation filter, offline

`raw_replay` replays one OC flight log (`.bin`) through the Beetle's navigation filter: `RawGnssShadow`,
`TR_RawGnssEKF`, `RawMeasBuilder` and the LC86G's RTCM parser, compiled from the firmware's own sources without
changes. It feeds them the way the flight computer's raw task does (`flight_computer/main/main.cpp`, raw-GNSS
section), and every bench flag of the image is a run-time argument. One run takes about 0.15 s on a Mac, so a
whole matrix of configurations over all the rig logs takes seconds.

## Build

```
cmake -S tools/raw_replay -B build_raw_replay && cmake --build build_raw_replay
```

Built from this tree, it replays the design flown since #1637: the vertical check, its boost window and its latch
are compiled in (`RAW_REPLAY_VCHECK` and `RAW_REPLAY_LATCH`, on by default), and the firmware's own defaults apply
unless a flag below changes them.

To replay with older firmware, for example the rig's r20-r26 code (`aef947c5`, before the receiver profiles and the
carrier), export that tree and build against it, turning off what it does not have:

```
git archive aef947c5 tinkerrocket-idf/components/TR_RawGnssShadow tinkerrocket-idf/components/TR_RawGnssEKF \
    tinkerrocket-idf/components/TR_GNSSRaw tinkerrocket-idf/components/TR_GNSSReceiverLC86_Serial \
    tinkerrocket-idf/components/TR_RocketComputerTypes tinkerrocket-idf/projects/flight_computer/main/baro_gate_policy.h \
    tests_cpp/host_shim | tar -x -C /tmp/aef947c5
cmake -S tools/raw_replay -B build_aef -DSRC_ROOT=/tmp/aef947c5 -DRAW_REPLAY_OLD=ON -DRAW_REPLAY_VCHECK=OFF \
    -DRAW_REPLAY_LATCH=OFF && cmake --build build_aef
```

## Use

```
python3 tools/raw_replay/extract.py OC_LOG run.events --blob sim_replay.bin   # once per log
build_raw_replay/raw_replay --in run.events --out run.csv --t-base-s <the events file's META t_base_s> [flags]
```

| flag | firmware build flag | meaning |
|---|---|---|
| `--heading-mode 0/1/2` | `TR_HEADING_FIT` | off, observe, act |
| `--pad-sigma-deg X` | `TR_BENCH_HEADING_PAD_SIGMA_DEG` | act's heading 1-sigma from launch (default 30) |
| `--handoff-deg X` | `TR_BENCH_HEADING_HANDOFF_DEG` | the fit's hand-off threshold; -1 = never (default 10) |
| `--gate-full 0/1` | `TR_BENCH_HEADING_GATE_FULL` | act's GNSS reaches all three axes from launch |
| `--tilt-sigma-deg X` | `TR_BENCH_LAUNCH_TILT_SIGMA_DEG` | act's launch tilt-sigma floor (0 = off) |
| `--carrier 0/1/2` | `TR_BENCH_CARRIER_MODE` | carrier range rates off, observe, fuse (default: the LC86G profile's, observe) |
| `--half-cycle-repair 0/1` | `TR_BENCH_HALF_CYCLE_REPAIR` | repair or reject a half-cycle step |
| `--mag-error-deg X` | `TR_BENCH_HEADING_ERROR_DEG` | magnetometer heading error, added to the declination |
| `--recorded-mag-error-deg X` | | what the log's own image was built with (taken out first) |

Other options: `--no-match-ages`, `--age-lead-ms`, `--epoch-delay-ms` (epoch timing, below), `--reanchor` (take the
pad context's own state and covariance at the launch tick, within a step), `--fc-unit-us` (an emulated FC clock so
epochs are worked a slice a step), `--pad lat,lon,h` (the NED origin; default the rig scenario's pad), `--every N`.

A calibrated IMU (`calibrated.py` builds these from the flight's truth): `--acc-sub x,y,z` and `--gyro-sub x,y,z`
(m/s^2, rad/s, body FRD) come off every tick; `--pad-quat w,x,y,z` is the pad attitude to start from (final: its
magnetometer error included), `--pad-ab x,y,z` the accelerometer bias the filter starts with, `--pad-wb-sub x,y,z` what
comes off the gyro bias it had.

Experiments that change no firmware (each one a switch the harness throws on the filter's own parameters):

| option | what it does |
|---|---|
| `--gate-full-from-apogee`, `--gate-full-at-s X` | the heading-only gate until the log's apogee call (or flight time X), then the full gate: "H" |
| `--act-from-apogee`, `--act-at-s X` | observe until the apogee call (or X), then act's launch edge (prior, tilt floor, gate) |
| `--epoch-shift-ms X` | every epoch's MCU time moved by X (a receiver output delay tried) |
| `--diag-from-s X --diag-to-s Y` | each epoch in that flight-time window worked one satellite at a time and printed (`DIAG` on stdout, `DIAGDATA` on stderr for `diag_truth.py`) |
| `--guard-min-pr N` | (a1) in a boost epoch (IMU peak since the last epoch above 1 + `--guard-g` g, default 2 g) with fewer than N pseudoranges, the attitude is held out of the update |
| `--guard-doppler-rms X` | (a2) the same when the epoch's Dopplers spread more than X m/s about one velocity and clock rate fitted to them (5 or more needed) |
| `--guard-dv-max X`, `--guard-dv-sigma K` | (a3) the same when the epoch's Dopplers, fitted to one velocity and clock rate, point to a velocity correction over max(X, K x the filter's velocity 1-sigma); fewer than 4 Dopplers count as failing |
| `--guard-reject-epoch` | (a4) such an epoch is dropped whole instead |
| `--guard-majority` | (a6) a boost epoch is dropped whole when the innovation gate refused more of its range rates than it used |
| `--guard-majority-half` | (a6h) the same when it refused half or more of them |
| `--guard-step-deg X`, `--guard-step-sigma K` | (b) in a boost epoch, a satellite whose update would turn the heading past X deg (or K x its 1-sigma) in that epoch is refused |
| `--guard-gate-chi2 X --guard-tight-g G` | (c) the innovation gate set to chi2 X while the IMU peak is above 1 + G g |
| `--cut-from-s X --cut-to-s Y` | a synthesized dropout: every epoch whose GPS time is in [X, Y) of flight time is dropped |
| `--corrupt-at-s X --corrupt-seed N` | the epoch nearest X corrupted like c30's T+1: on 3 satellites the range rate off by 30-70 m/s and the pseudorange by 20-40 m |

A sidecar `<csv>.events.json` records the guards' counts, the apogee votes the run's own estimate gives, and when the
gate switched.

`extract.py --borrow OTHER.events` takes the pad context (covariance, ephemerides, ionosphere, IMU biases) from
another run of the same replay for a log whose OC image dropped records 0x9B-0x9F (r16, r18, r19).

The CSV has one row per filter step (100 Hz): MCU and flight time (s from the ignition edge), the quaternion (body
FRD to NED), latitude, longitude, height and NED from the pad, velocity, attitude and heading 1-sigma, position and
velocity 1-sigma, the IMU biases, the receiver clock, health (running, finite, GNSS-aided, barometer-aided), the
last epoch's satellites used and rejected and its age, the ephemerides held, the carrier rates built, screened in and
fused, levelling, zero-velocity updates, barometer samples fused and held, magnetometer updates, and the heading fit.

`scorelib.py` scores a harness CSV or the log's own on-board records (0x96) against the rig flight's truth with the
same metrics and phases; `onboard_vs_harness.py` puts the two side by side; `run_matrix.py` and `tables.py` run and
tabulate the configuration matrix.

## Studies: the scripts beside the harness

### Layout they expect

The scripts find everything relative to their own directory (`HERE`):

- the binary at `HERE/build/raw_replay`, so build with `cmake -S tools/raw_replay -B tools/raw_replay/build`
  (`one.py` takes another through `ONE_BINARY`, `dropout_sweep.py` through `DROP_BINARY`);
- each log's events at `HERE/in/RUN.txt`, from `python3 extract.py OC_LOG in/RUN.txt` (`--borrow` for r16, r18, r19);
- the truth: `$COCOM_SDR/scenarios/sim_rp5_0829_sensors.csv.gz` (`scorelib.TRUTH_CSV`; default
  `Code/gnss/cocom-rig/sdr`), and the repository's `Data_Analysis/` for the log parser (`scorelib.WT`: the repository
  the script sits in, else `$RAW_REPLAY_WT`);
- `calibrated.RECORDED`: the magnetometer error (`TR_BENCH_HEADING_ERROR_DEG`) each log's image was built with. Add an
  entry for every new log; `one.py`, `calibrated.py`, `guard_sweep.py` and `dropout_sweep.py` pass it as
  `--recorded-mag-error-deg`.

The configurations come from `run_matrix.py`: `BASE_A` (act, full gate, 0.6 deg tilt floor, 30 deg prior, no hand-off,
carrier observed), `BASE_C` (the same, heading-only gate), `BASE_U` (observe: "unaided"). H is `BASE_C` plus
`--gate-full-from-apogee`; H10 / H30 replace the prior.

### The hand-calibrated pad (`calibrated.py`, `one.py RUN cal ...`)

The rig replays the truth CSV's sensor errors: over the pad's last 60 s the accelerometer reads [1.49, 10.26, 0.57] mg
off and the gyro [-0.027, -0.097, -0.167] deg/s (body FRD). The 10 mg sideways bias levels the uncalibrated pad 0.8-1.3
deg off, which is what makes act cost coast attitude on the rig with a good magnetometer. A hand calibration takes
those off. To emulate it, for each log:

- `csv_bias()`: the truth CSV's mean accelerometer and gyro errors over the pad; `--acc-sub` and `--gyro-sub` take
  them off every tick (with a residual added back if wanted: `calibrated.py` uses none, or 2 mg in 4 random
  directions);
- `pad_means()`: the log's own pad (flight time -1.1 to -0.05 s), mean accelerometer and magnetometer;
- `triad()`: the pad attitude the levelling and the magnetometer agree on, with the flight computer's declination
  (PNAV) less the recorded bench error; with no residual it is the truth's to 0.03 deg. `qz(mag)` then turns it by the
  magnetometer error under test, and that goes in as `--pad-quat`;
- `--pad-ab`: the accelerometer bias the pad can see (the residual along gravity; 0 with none);
- `--pad-wb-sub`: what came off the gyro, taken off the gyro bias the filter had.

`calibrated.py OUT RUN...` runs A, C and unaided at 0, +10, +40 deg (`CALIB_STUDY=hybrid`: A, C, H at 30 and 10 deg,
magnetometer -40 to +40; `CALIB_STUDY=late`: act opened at the apogee call; `CALIB_PRIOR10=1`: A and C with the 10 deg
prior; `CALIB_UNCAL=1`: the same runs on the rig IMU as recorded). `calib_tables.py` and `hp_tables.py` tabulate them. For one run:
`one.py RUN cal MAG OUT_PREFIX [flags]` (or `uncal`), writing `OUT_PREFIX.csv`, `.out` (the `DIAG` lines) and `.err`.

### Boost dropouts and the safeguards (`dropout_sweep.py`, `dropout_tables.py`, `recommend.py`)

`dropout_sweep.py OUT IMU RUN...` (`IMU`: `cal` for the calibrated pad, `uncal` for the rig as recorded) runs, per log
and magnetometer error 0, +10, +40 deg, 16 scenarios:

- `real`: the log as flown (c30 as flown is a real boost dropout: its receiver gave one corrupted epoch at T+1.0 s and
  nothing after);
- `cutA-B`: every epoch cut from T+A (0.5, 1.5, 3 s) until T+B (8 s, 12 s, the end);
- `badN-cut1.5-B`: one epoch corrupted like c30's at T+1 s (draws 1 and 2), then everything cut from T+1.5 s until B;

against the configurations `H30_X` and `H10_X` for each safeguard set `X` in `SAFE` (`none`; `a4`, `a5`: the Doppler-fit
check, epoch dropped; `a6`, `a6h`: the majority check; `b2`: the 5 deg heading-step cap; `c`: the tight gate; and
combinations), plus `A30_none`, `C30_none` and `U_ref` (unaided). `DROP_CFGS=H10_a6h,U_ref,...` runs a subset;
`DROP_BINARY=...` another binary (for example one built against a patched tree). Each log writes
`OUT/IMU/RUN/dropout.json`: per run its log, scenario, configuration, magnetometer error, the guards' counts, the
pitch vote, and per phase (boost, coast, descent) the median / p90 / max of the nose, roll, heading and pitch errors,
plus `end` (the medians over the last 2 s before landing). The as-flown runs keep their CSVs; the synthesized ones
overwrite `synth_last.csv`. About 0.13 s a run: 16 scenarios x 3 errors x the configurations, per log.

The acceptance test ("never worse than unaided because of boost GNSS"): a run is worse than unaided when, in some
phase, its median nose or roll error is more than `--tol` deg (default 1) above unaided's on the same log, scenario and
magnetometer error, or its heading, roll or nose over the last 2 s before landing is.

- `dropout_tables.py SWEEP[,SWEEP...] OUT.md --imu cal [--tol 1] [--lost c30]`: per configuration, the runs worse
  than unaided by scenario class, the attitude as flown and after a dropout (median of the per-log medians [worst
  log]), and every flagged run. Several sweeps merge, a later one replacing an earlier one's runs.
- `recommend.py OUT.md --imu cal --tol T`: the candidates side by side: dropout runs worse than unaided per
  magnetometer error, the worst end heading less unaided's, and what each keeps as flown (coast nose and roll,
  descent roll at +10 and +40; boost roll at 0, +10, +40; the latest pitch vote). Its two source lists (firmware
  levelling, heading-free levelling) name the 2026-10-15 sweep directories: edit them for new sweeps.
- `controls_level.py OUT_DIR OUT.md`: the no-GNSS logs (r16, r19, r26), act against unaided, with `build/` (the
  firmware's levelling) and `build_exp/` (heading-free levelling, below).

The 2026-10-15 result (calibrated pad, 10 logs, 453 dropout runs per configuration): with heading-free levelling,
H10 + a6h had 0 runs more than 3 deg worse than unaided (7 more than 2 deg, 25 more than 1 deg, all at magnetometer 0,
worst +2.6 deg) and kept the coast and descent gains (as flown at +40: descent roll 0.4 vs unaided's 40.2).

### Heading-free levelling (`experiments/heading_free_levelling.patch`)

The experiment behind the levelling finding: before each in-flight levelling update the heading's correlations are
taken out (`setHeadingSigma` at its current size), so gravity cannot turn the heading. It is applied to an exported
copy of the sources, never to the firmware here:

```
mkdir -p /tmp/src_exp && git archive HEAD tinkerrocket-idf/components/TR_RawGnssShadow \
    tinkerrocket-idf/components/TR_RawGnssEKF tinkerrocket-idf/components/TR_GNSSRaw \
    tinkerrocket-idf/components/TR_GNSSReceiverLC86_Serial tinkerrocket-idf/components/TR_RocketComputerTypes \
    tinkerrocket-idf/projects/flight_computer/main tests_cpp/host_shim | tar -x -C /tmp/src_exp
patch -p1 -d /tmp/src_exp < tools/raw_replay/experiments/heading_free_levelling.patch
cmake -S tools/raw_replay -B tools/raw_replay/build_exp -DSRC_ROOT=/tmp/src_exp && cmake --build tools/raw_replay/build_exp
```

### When the firmware carries the recommended set

The harness compiles the firmware's own sources, so a firmware version of heading-free levelling, the majority check
and the 10 deg prior is replayed as it is once rebuilt. Then:

- give the new firmware switches their own harness arguments (as the bench flags above), and drop the harness-only
  ones they replace from the configurations (`--guard-majority-half`, and `--gate-full-from-apogee` if H moves into
  the firmware), or the two will act twice;
- the harness's majority check counts the gate's verdicts after working the epoch and restores the filter if half or
  more of its range rates were refused; a firmware pre-pass (every range rate gated against the state before the
  epoch) may decide slightly differently. Compare both on the dropout sweep;
- its "boost" is the IMU peak since the last epoch above 2 g (`--guard-g 1.0`, a dropped epoch restarting the peak).

### On the firmware that carries the safeguards (fc/gnss-heading-fuse 7ce83c27; on main since #1637)

The firmware now has heading-free levelling (every levelling update, `RawGnssEKFParams::level_heading_free`), the boost
screen (`RawGnssShadow::Config::boost_screen`: the Dopplers gated at the epoch's prior, then the epoch worked through
on a scratch copy of the filter; half or more refused drops it) and the launch prior tied to the magnetometer (10 deg
when a magnetometer heading passed its gates within 5 s of the launch edge, else 30). Their run-time switches:
`--level-heading-free 0|1`, `--boost-screen 0|1|2` (off, in act, always), `--boost-screen-g X`,
`--pad-sigma-deg X` (the prior with the magnetometer's heading), `--pad-sigma-fallback-deg X`, `--mag-recent-s X`.
H is the firmware's own: `--gate-full 2`, the gate opening on the flight computer's apogee call, which every tick from
the log's apogee flag now carries (`kShadowApogee`; `--no-apogee-flag` leaves it off) -- so `--gate-full-from-apogee`
and `--guard-majority-half` drop out of the configurations. Each magnetometer sample is flagged `--mag-flags 2`
(kShadowMagSim) by default: the rig's field is the sim's, and the firmware refuses an unflagged one (a6164eb0);
`--mag-flags 0` replays a pad whose magnetometer is refused (the fallback prior).

`--compare-sequential` works each due epoch first on a copy of the filter with the screen off and writes both
verdicts for every boost epoch to the events JSON (`screen_compare`), the firmware's own counts beside them (`fw`).
`--max-epoch-age-s` sets the firmware's stale-epoch limit (0.6 s). The guarded paths (`--guard-*`) work every due
epoch themselves and never drop one as stale: on f3c, f4b, r18 and r20-r25 they applied 1-4 epochs a flight that
reached the filter more than 0.6 s after their time (on r20 a second epoch of T+1's time of week, one satellite,
0.97 s late), which the firmware drops; with the limit raised to 1.5 s the firmware's H10 matches the guarded
sequential check to 0.00 deg in every phase of every as-flown run.

### The design as flown (main since #1637)

The flags that replay the Beetle's flight image, as the 2026-10-16 rig confirmation and sweeps ran it:

```
--heading-mode 2 --gate-full 1 --tilt-sigma-deg 0.6 --pad-sigma-deg 10 --handoff-deg -1 --carrier 1 \
    --boost-screen-g 2 --boost-screen-window-s 1.5 --vert-check-mps 20
```

and unaided (the magnetometer and the gyro, GNSS observed only) is `--heading-mode 1 --carrier 1`. The safeguards'
run-time switches, each defaulting to the firmware's value:

| option | firmware (`RawGnssShadow::Config`) | meaning |
|---|---|---|
| `--vert-check-mps X` | `vert_check_mps` (20; 0 off) | an epoch whose Dopplers' fitted vertical is more than X m/s off the filter's is dropped |
| `--boost-screen-window-s X` | `boost_screen_window_s` (1.5) | an epoch within X s of the IMU passing `--boost-screen-g` is screened |
| `--vert-release-mps X` | `vert_release_mps` (5; 0 off) | the latch: after a vertical drop, every epoch is dropped until one fits within X m/s |
| `--max-epoch-age-s X` | `max_epoch_age_s` (0.6) | an epoch older than this when worked is dropped |
| `--rr-dyn-sigma X` | the receiver profile's `rr_dyn_sigma` (0.5) | the Doppler's allowance, m/s per g past 1 since the last epoch |

`RAW_REPLAY_VLOG=1`, `2` or `3` prints each vertical check's verdict, then the fitted rows, then every closed
epoch's rows.

A flight log (no truth): extract it as above, run the flown flags with `--recorded-mag-error-deg 0`, and put the
harness beside the on-board records with `onboard_vs_harness.py`. A run with another magnetometer error or a
calibrated pad answers "what would this flight have done if".

Checked on main (75fd7dd7, 2026-10-17): the 24 rig logs at magnetometer errors -40, -10, 0, +10, +40 deg, with the
rig's IMU as recorded and hand-calibrated (0 or 2 mg left), flown and unaided, reproduce the 2026-10-16 sweeps on the
branch to 0.03 deg in every run (720 runs; uncalibrated, 16 of 120 flown runs more than 3 deg worse than unaided before
apogee, the worst 10.3 deg). The two experiment patches below reproduce theirs to 0.01 deg.

### Experiments the firmware does not carry (`experiments/*.patch`)

Each patch applies to this tree's firmware; build it against an exported copy as for heading-free levelling, with its
option on:

- `doppler_settling_fade.patch`, `-DRAW_REPLAY_DYNMEM=ON`, `--rr-dyn-tau-s X`: the boost's Doppler allowance kept
  after the thrust, faded with a time constant of X s (the receiver profile's `rr_dyn_tau_s`), for the low
  satellites whose loops held through the boost and still read 1-2 m/s off in the first coast epochs. Tried on the
  24 rig logs on 2026-10-16 and not adopted.
- `doppler_fit_gate.patch`, `-DRAW_REPLAY_FITGATE=ON`, `--rr-fit-gate X`: each Doppler judged by its epoch's own
  velocity-and-clock fit, past X of its sigmas refused. Dropped: in the cloud session's simulations it made the stale
  receiver worse, on 5-row epochs whose precise rows fix the fit exactly.

### Other scripts

`one.py` (one run, calibrated or not), `guard_sweep.py` and `guard_tables.py` (the first safeguard sweep: a1, a3, b1,
b2, c and act from apogee), `diag_truth.py` (an epoch's measurements against the truth, from `--diag-from-s`),
`aided_check.py` (does a log have GNSS in flight: epochs after T+8 s with 4 or more pseudoranges), `voters.py` (the
on-board apogee votes), `tonight.py` and `tonight_summary.py` (the 2026-10-15 rig night: on-board against the
harness for each log, its logs in `../replay/rf_oc/`), and the timing checks `tag_check.py`, `shift_scan.py`,
`clock_offset.py`, `msm_offset.py`, `csv_bias.py`, `pad_check.py`.

## What it reproduces

- The IMU ticks: 1 kHz logic windows at the log's own EKF tick rate (`NonSensor.ekf_ticks`), every second window
  handed to the filter (`EKF_DECIMATION` 2) as its integer-rounded mean (`imu_drain::roundedMean`), converted as
  `SensorConverter` does, with the high-g channel when any sample in the window was near the low-g rail. The window
  boundaries are a regular grid, not the loop's own.
- The flags on each tick: levelling (not in flight, or after the apogee flag), in flight (from the pad context's own
  launch tick), landed.
- The barometer: the frozen ground pressure (`Snapshot.ground_pressure_pa`), the pressure altitude in float as the
  loop computes it, and `BaroGatePolicy` itself (spike filter, 20 ms throttle).
- The magnetometer: each new sample, body FRD.
- The raw epochs: the logged RTCM3 frames (0x95) through `Lc86Parser`, each MSM7 epoch placed on the MCU clock by
  the driver's own clock line (0x9D) and built by a `RawGnssMeasStage` at the filter's published position, as the GNSS
  task does; ephemerides from the pad context first, then the flight's own; the satellites the filter finds jumped go
  back to the stage.
- Epoch timing: each epoch reaches the filter when the on-board filter closed it (its logged age, less 20 ms for the
  slices before the close). The Beetle took 30-220 ms per epoch; handing them over as they arrived made them ~110 ms
  younger than on board.
- The start: the log keeps about 1.2 s of the pad before ignition, so the filter starts at the log's first record with
  the on-board state (0x96 and 0x9B: position, velocity, attitude, IMU biases, clock, system offsets) and the pad
  context's covariance (0x9E, taken at the launch tick) with its velocity block scaled to the first record's sigmas;
  the pad context's ionosphere, declination, field size and barometer offset. The builder's lock history and the
  filter's last-update times are seeded for the first epochs' satellites (long locked, updated an epoch ago).
- A magnetometer error other than the recorded one turns that pad state about the vertical (attitude, horizontal
  velocity, and the NED blocks of the covariance) and north with it, which is what the magnetometer does on the pad.

## What it does not

- The epoch's work is done in one step, not a slice a step; the receiver's fix checks (FixCheck, report-only) are not
  fed; the pad survey's corrections are not restored (every rig run used the broadcast ionosphere).
- Logs without a clock line use the first logged line for the epochs before it, or, with none at all, the frame's
  arrival plus 25 ms (what the line sat at on r20-r25).

## How faithful it is

Built against the code each log ran (`aef947c5`), with the log's recorded flags, against the log's own on-board
records (r20-r26, each with its pad context): attitude difference median 0.1-0.9 deg, p90 0.2-2.2 deg; maxima under
3 deg except r20's coast (9 deg for one epoch at its hand-off); horizontal position 0.1-4.8 m median, vertical
0.1-1.3 m, velocity 0.02-0.4 m/s. Against the truth, the harness's and the on-board scores agree within a few tenths
of a degree in every phase, and the first time the estimated pitch goes below -5 deg (the apogee pitch voter's test)
within 10 ms of the flight loop's own. The full tables are in the session's `offline_replay/val/`.

Built against dc37cf07, against the 2026-10-15 rig logs (median, on board / harness): f2 (act, heading-only gate,
10 deg prior, +10) nose coast 2.00 / 2.00, roll coast 7.99 / 8.00, roll descent 0.28 / 0.35; f3c (A, +10) 2.29 / 2.30,
8.43 / 8.66, 1.35 / 1.23; f4a (A, 0) 3.29 / 2.77, 10.57 / 9.36, 0.55 / 0.20; f4b (A, +40) 1.03 / 1.23, 6.17 / 3.98,
3.19 / 2.40; c30 (C, +10, lost in boost) 10.36 / 10.91, 29.68 / 30.57, 33.18 / 32.59. The pitch < -5 deg crossing
within 1-7 ms of the flight loop's on all five. Two differences are left:

- at a 40 deg magnetometer error (f4b) the two make the same first boost turn and then converge along slightly
  different paths in coast (2-3 deg); the flight computer works an epoch a slice a step while the IMU propagates, the
  harness in one step, and that far from linear the difference shows;
- on c30 and f4a the on-board vertical velocity runs 0.3-0.4 m/s off the truth from T+0.3 s while the harness's stays
  within 0.05 (f2, f3c and f4b agree). It starts before the first in-flight epoch; the launch biases and the vertical
  sigmas match. The cause is not found; no attitude result depends on it.
