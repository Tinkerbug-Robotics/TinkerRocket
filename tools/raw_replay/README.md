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
| `--corrupt-rr-mps lo,hi`, `--corrupt-pr-m lo,hi` | the corruption's sizes instead (each drawn uniform in [lo, hi] with a random sign): under the gate, say, `0.05,0.12` and `5,10` on the mosaic |
| `--corrupt-pr-epoch` | the first epoch at or after X on a whole GPS second instead (on a 10 Hz receiver its pseudoranges are fused there, `pr_period_s` 1 s) |

The spinning body (fc/v9-windup; a `RAW_REPLAY_WINDUP` build, on by default): each switch the firmware's unless
given -- off, but for what `--receiver mosaic`'s flight safeguards set since fc/v9-highroll (`--att-exact 1
--dv-lever-mode 1 --dv-lever-radial-m 0 --windup-k 0.99`; the image as built before them: `--att-exact 0
--dv-lever-mode 0 --windup-k 0`):

| option | what it does |
|---|---|
| `--windup-k K` | each Doppler (and carrier rate) less the carrier wind-up an antenna on the nose carries, K cycles a turn, from the gyro's roll at the Doppler's own instant (`RawGnssShadow::Config::windup_k`; the Goblin's helix: 0.99) |
| `--windup-geom 1` | Wu's whole rate from each satellite's line of sight, not the roll alone |
| `--windup-monitor 1` | observe: the roll rate the Dopplers' wind-up implies between wavelengths, against the gyro's |
| `--att-exact 1` | the covariance's attitude transition exact, exp(-[w x] dt) (`RawGnssEKFParams::att_exact`) |
| `--dv-lever-mode 1`, `--dv-lever-radial-m X` | the change check's lever allowance on the nose's own turn and a radial bound, not the whole rate (the arm: `dv_lever_allow_m`, or with the offset known `knownLeverArm`'s) |
| `--clk-drift-psd X` | the clock drift's walk (m^2/s^3) in place of the profile's 100: a tighter clock, which the wind-up correction is a prerequisite for |

The epochs file then also carries each epoch's clock drift, screen and check numbers and the spinning body's
(`clk_d_mps` .. `att_sig_d_deg`), and the events JSON a `windup` block.

A sidecar `<csv>.events.json` records the guards' counts, the apogee votes the run's own estimate gives, and when the
gate switched. A mosaic or cold-start replay also writes `<csv>.epochs.csv`: one row per epoch the filter closed,
whatever `--every` (its time, the codes and Dopplers used and refused, the largest NIS, the longest any one satellite
has been refused running for each kind -- the EKF's own count, `rejects_`, which `reject_persist` lets back in at,
times that kind's period: 1 s for a code, 0.1 s for a Doppler on the mosaic -- and the corrupted satellites built
into it). `--pvt-out FILE` writes each
receiver fix's height above the ellipsoid, for `score_static.py --pvt`; it and the CSV stay out of the repo.

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

## The V8/V9 with the mosaic-G5 (`--receiver mosaic`)

On the mosaic-only build (the Goblin's V9 on 2026-10-24; [the configuration](../../docs/builds/v9-mosaic-rawnav.md))
the raw-GNSS filter is the one navigation filter, fed by the Septentrio mosaic-G5's SBF instead of the LC86G's RTCM.
The same harness replays it from a V8/V9 OC log, with the firmware's own decoders, and runs the apogee quorum the
flight computer would call from it.

### Build and run

The build makes two binaries from the same sources: `raw_replay`, whose ephemeris store holds the Beetle's 64, and
`raw_replay_mosaic`, built with `TR_EPH_CAPACITY=128` as the mosaic-only image is. Use `raw_replay_mosaic` for the
mosaic; each binary warns when it replays the other receiver. `RAW_REPLAY_MOSAIC` (on) compiles the mosaic path;
turn it off to build against a tree from before fc/v9-raw-primary.

```
python3 tools/raw_replay/extract.py --sbf OC_LOG run.events          # a V8/V9 log with the mosaic's SBF in 0x95
tools/raw_replay/build/raw_replay_mosaic --in run.events --out run.csv --receiver mosaic \
    [--rinex BRDC00WRD_S_20262890000_01D_MN.rnx] [--unaided] [flags]
```

`extract.py --sbf` writes no S, A or P lines (the 10/16 and 10/17 logs carry no raw-filter records), puts every SBF
block of 0x95 in R lines as it arrived (PVTGeodetic among them, so no V lines), and every navigation block (GPSNav,
GALNav, BDSNav, GPSIon) once more just before the first event, marked `preload` (META `nav_preloaded N gps G gal E
bds C ion I` counts them), unless `--nav-as-logged`: the receiver sends a satellite's ephemeris only when it changes,
and the logs begin long after it started. The harness files a preloaded issue no earlier than a receiver can first
have it, by the RINEX rule below (`rinex::availGpsS`), so an issue decoded later in the log is not handed over at
the start; the GPSIon sets go through the ionosphere rule at once. That is still an estimate: a satellite this
receiver decoded late (one rising: E02 came 1857 s after its toe on the cone-off log, against the rule's 665 s) can
be filed before this receiver had it, and an issue the receiver held before the log began and never sent again is
missing. On the 10/16 cone-off log the rule leaves 2 satellites with an ephemeris at the start (8 from T+367 s, 9
from T+967 s), where filing everything at the start gave 9 from the first second. Use `--nav-as-logged` for a log
the flight computer booted in (its esoc burst resends everything the receiver holds, so the log then holds exactly
what the driver got) and for G9's comparison with the on-board 0x96 from that boot on. T+0 is the log's in-flight
edge, or the log's own clock where it has none (META `t_base_s`, which a mosaic or cold-start run takes unless
`--t-base-s` is given). I, B, M, F and K lines are as for the Beetle. `--seed-quat-from-log` writes META `seed_quat`
from the log's own flight filter (NonSensor, the auto image's GpsInsEKF) for a cold start that should begin from
its attitude. Its heading is then that filter's, and on a static pad nothing turns it (below), so never score
heading against the log's quaternion from such a run; the harness warns when it takes META `seed_quat`.

A rig replay of the mosaic-only image (`tools/rig_replay/run_v9.sh`) takes `extract.py --sbf --rig OC_LOG run.events
--blob REPLAY.bin`. Its OC log begins at launch, less about 0.4 s of prelaunch ring, so a cold start would begin in
the boost. With `--rig`, the events file starts from the on-board filter's own records instead, as an LC86G rig log
does: S at its first running pad record, A, and the pad context (PNAV, PCOV, PSUR) when the log has it. T+0 comes
from the replay's ignition edge against the log's accelerometer (META `t_base_s`, `edge_s`), and the ticks are in
flight from the pad context's launch tick. `raw_replay_mosaic --receiver mosaic --quorum` then restores that state
(`tools/rig_replay/score_v9.py --harness` runs it, aided and `--unaided`).

For the Beetle's and other non-SBF logs, `extract.py` (without `--sbf`) now also writes a `V` line beside each `F`:
the receiver's fix as the flight loop's GPS voter gets it, so `--quorum` has its fourth voter. The 25 bitident files
predate it and carry none.

### What `--receiver mosaic` sets

What `startRawShadow()` builds under `TR_RAW_NAV_MOSAIC` (build_units D1a; checked against fc/v9raw-d 2502c322):

- `mosaicG5Profile()` (v0) with the image's flight profile over it, by the firmware's own call
  (`gnss_raw::applyMosaicFlightProfile`, `ReceiverProfileApply.h`; a `RAW_REPLAY_FLIGHT_PROFILE` build, on unless
  the tree is older than fc/v9-1024-candidate or one of `RAW_REPLAY_MOSAIC_SAFEGUARDS`, `BUILD_LITE`, `SHARELOS` and
  `DOPAVG` is off): the owner's 10/24 set, the cheaper epoch build, the epoch's geometry shared with the Doppler's
  prediction in float, the Dopplers in pairs, at most 16 satellites with the cap fixed. A build without it keeps the
  profile's own (each off, 16 satellites), and its configure says why. Then
  `--fuse-ms` (`TR_BENCH_RAW_FUSE_MS`), `--sat-cap` (`TR_BENCH_RAW_SAT_CAP`), `--build-lite [0|1]`
  (`TR_BENCH_RAW_BUILD_LITE`: the cheaper epoch build, `RawMeasBuilder.h`, on alone or with 1, off with 0; a
  `RAW_REPLAY_BUILD_LITE` build, on refused but for the mosaic) and `--share-los` (`TR_BENCH_RAW_SHARE_LOS`: 0,
  neither; 1, the epoch's geometry taken once and shared by its units; 2, and the Doppler's prediction in float; a
  `RAW_REPLAY_SHARELOS` build; the events JSON's `build_lite` and `share_los` say which) applied to it, with the
  Dopplers in pairs below (`--rr-pair-avg 0|1`), so each turns its own off for an experiment: `--build-lite 0
  --share-los 0 --rr-pair-avg 0` is the high-roll-only twin, the image as fc/v9-highroll built it. In a
  `RAW_REPLAY_CAPGOV` build `--cap-max` (`TR_BENCH_RAW_CAP_MAX`: the satellite cap
  follows the filter's slack up to it, `CapGovernor.h`; its counts in the events JSON's `cap_gov`) and `--slice-us`
  (`TR_BENCH_RAW_SLICE_US`: the profile's `epoch_slice_us`), then `applyReceiverProfile`. Without a clock the
  harness works every epoch at once, so the governor sees only the handover's delay in the ages: `--fc-unit-us`
  slices them; neither prices a satellite's build, so where the cap settles on the flight computer is the bench's
  (or a CPU model's) to say, not the harness's. The logs' ticks carry no PRELAUNCH (`kShadowPrelaunch`, which holds
  the cap at the floor on the flight computer's pad), so a replay climbs on the pad as in READY, and an epoch built
  above the floor whose job starts in the boost is cut to the floor's (`trims`, `trimmed` in `cap_gov`);
- the navigation filter's settings (`primary`, 100 steps a second, the profile's 2.5 ms slices, `external_build`);
- the restore sigmas (100 m, 20 m/s, 15 deg) and the snapshot's accelerometer bias, `level_on_fused_epochs`;
- the flight safeguards, by the firmware's own call (`gnss_raw::applyMosaicFlightSafeguards`,
  `ReceiverProfileApply.h`; a `RAW_REPLAY_MOSAIC_SAFEGUARDS` build): the Doppler change check acting in flight
  (`--dv-check 2 --dv-window-s 1 --dv-att-deg 10 --dv-epoch-k 4 --dv-latch-k 5`, and any heading about the
  vertical after a launch with no magnetometer heading: `--dv-hdg-unset-deg 180`), GNSS's heading gain by the
  horizontal specific force (`--hdg-obs-mode 1 --hdg-obs-fh 5 --hdg-obs-tau 1 --hdg-obs-scope 0`), the turning
  body's Doppler allowance (`--rot-lever-axial-m 0.3 --rot-lever-radial-m 0.03 --rot-lever-min-rate 2`) and the
  spinning body's settings (fc/v9-highroll: `--att-exact 1 --dv-lever-mode 1 --dv-lever-radial-m 0 --windup-k 0.99`).
  It runs right after the profile, so each `--dv-*`, `--hdg-obs-*`, `--rot-*`, `--att-exact` and `--windup-*` flag
  still changes its own value for an experiment. Today's design before them (and its `--unaided` twin) is
  `--dv-check 0 --hdg-obs-mode 0 --rot-lever-axial-m 0 --rot-lever-radial-m 0 --rot-lever-min-rate 0`; the image as
  built before the spinning body's, `--att-exact 0 --dv-lever-mode 0 --windup-k 0`. A build without them
  (`RAW_REPLAY_MOSAIC_SAFEGUARDS` off, which any of `RAW_REPLAY_DVCHECK`, `HDGOBS` and `ROT` off turns off, saying
  so at configure) warns on every `--receiver mosaic` run that it flies none of them;
- the heading at the firmware's defaults (`--fw-defaults`, which now also takes `--heading-mode` to the primary
  build's `TR_HEADING_FIT` 2 unless given): act, the whole attitude from launch (`RAW_HEADING_GATE` 1), the 10/30 deg
  prior, no hand-off, the 0.6 deg tilt floor;
- each epoch handed to the GNSS task's stage 30 ms after its block arrived (`--epoch-delay-ms`; there is no on-board
  age to match yet, and the P4's build time is not measured).

The SBF blocks go through `mosaic_feed.h`, which does what the driver (`rawMeasEpoch`, `rawPvt`) and the GNSS task
(`rawGnssMeasTask`) do with them:

- every MeasEpoch's clock jumps are followed (`MosaicJumpState`), with its logged arrival placing a jump the count
  shows only mod 256 (a sky come back early or late on the RF rig) and telling a reset from a big step; those on
  the fused grid (`onFuseGrid`, the profile's 100 ms) become RawEpochs (`rawEpochFromMeasEpoch`), and an empty one
  is not handed over;
- an epoch's GPS time is its receiver time less the clock bias of the last PVTGeodetic that anchors the jump count
  (`pvtAnchorsJumps`), carried by its drift and the jumps since (`dtrMs`), at most two minutes and while the count
  has not started over; without one the epoch is untimed and counted (`dtr_zero`), as the driver leaves it
  (`MosaicTimeTag.h`, since the 10/17 rig underrun: the bias 0 it took before was up to 0.5 ms off with a fix and
  anything at all after a receiver restart). The log's clock line (K, the PPS fit) then places it on the MCU clock
  only where the driver would (`rawMeasEpoch`, `mosaic_feed.h` `TimeTagGate`): the line valid, the receiver's last
  ReceiverTime saying FINETIME, a 1PPS edge paired within 120 s (each K line is a pairing), a bias to carry, the
  last edge on the line (each K line's edge against that line, within 100 us) and on the chain (moved from the
  last trusted line by as much as the PVTs' bias moved against the receiver's clock: a 1PPS still on the old sky
  after a step reads 0 in the PPS check), and the chain of PVT biases confirmed by the 1PPS: after a break (the
  receiver's clock lost, a restart of the jump count, an anchor too old, a PVT 0.1 ms off the bias carried to it, a
  failed check, a restart of the line, an edge on the line off the chain) the tags are held until two whole
  seconds' tags check out against edges on the chain with their own PVT's bias. Each K line is taken where the
  driver took it, at the first block that paired its edge (an xPPSOffset or a whole-second PVTGeodetic naming its
  second), not at the edge it is stamped with, so the epochs that came between go on the line before it, as they
  did in flight; one whose block never comes is taken at the next K line (`k_unpaired`). K lines extracted with the
  record's seq and restarts (extract.py since the tag fix) count the records the driver's queue or the log lost
  (`k_lost`, with a warning: the edges the driver judged are then not all here) and give the line's restarts; older
  ones count a restart where the line's references moved. Before the log's first ReceiverTime and first K line
  those first ones stand for the earlier, and so does its first chain (the log begins long after the receiver
  did); `--time-tag-from-boot` leaves epochs untimed until them, as after a boot. A log with no ReceiverTime at all
  is untimed, as the driver would leave it, with a warning (`--assume-finetime` times it). The events JSON's
  `time_tag` counts the untimed epochs by reason, the chain's breaks, the jumps placed and resets, the K lines
  unpaired and lost, and the PPS check; the console's `time tag chain` line says the same. Logs from before the
  mosaic image logged a K line at every edge carry one only where the line moved;
- navigation blocks are kept as the driver keeps them (`sbfNavKeep`: no Galileo F/NAV, no BeiDou GEO), decoded
  (`decodeSbfEphemeris`) and filed with the stage's `addEphemeris`;
- GPSIon goes through the GNSS task's one ionosphere rule (the firmware's `raw_iono_rule.h` where the tree has it,
  else the same rule in `mosaic_feed.h`) to the stage and the filter;
- PVTGeodetic is the flight loop's fix (`gnssDataFromPvt`, the converter's units), and its geoid separation turns
  the filter's heights for the loop (`RawNavAdapter::setGeoidSeparation`).

The cut, corrupt and relock faults (`--cut-from-s`, `--corrupt-at-s`, `--relock-min-lock-ms`) apply to the decoded
epochs as for the LC86G. On a mosaic or cold-start replay they are keyed on flight time (the epoch's MCU time, its
arrival where untimed, plus `t_base_s`: the CSV's `t_s` and the quorum's frame), and the corrupted epoch is the one
within half a fused period of `--corrupt-at-s` (with `--corrupt-pr-epoch`, the first on a whole second at or after
it). The corruption lands before the stage's satellite cap, so some of its satellites may never be built: the events
JSON's `corrupt` gives each one's errors and how many the stage built, and the epochs file says what the filter did
with that epoch. `--truth-t0-tow` keys them on the GPS time of week less T+0's
instead, as the Beetle's rig runs are (the rig scenario's by default). A cut that cuts no epoch or a corruption that
hits no satellite is a warning on every run and fails a mosaic or cold-start run with exit status 3 (the events
JSON's `cut` and `corrupted_sats`, and `faults_clock`, say what happened). The LC86G's 1 Hz lock-history seeding and
the A-line age matching apply only to a restored LC86G start.

### The cold start (`--cold-start`; automatic without an S line)

The filter starts on the pad as the mosaic-only image starts it, through `RawNavAdapter` as main.cpp does: at the
first IMU tick once the receiver's position is known (META `seed_lla` "lat_deg lon_deg h", the height above the
geoid, else the first PVTGeodetic with a solution) and 16 ticks are in the gyro ring, `init()` with that position and
the last 16 ticks' mean gyro (main.cpp's `initGyroRing`, which a boot fills long before the receiver's first fix: a
log that begins after that fix, as the 10/16-17 V9 logs do a few ms in, would otherwise seed the gyro bias from one
or two of the ISM6's quantized tick means, 0.13 deg/s off about the rocket's axis on the cone-on log, and nothing on a
still pad observes that bias without the magnetometer; `--seed-gyro-dps x,y,z`, body FRD, gives the seed instead,
for a log that begins while the payload turns: the cone-off spin-rig log),
`setQuaternion()` from the tick's accelerometer and the pad heading guess (`quatFromAccelHeading`,
`--seed-heading-deg`, `config::PAD_HEADING_DEG` 0; META `seed_quat` "w x y z" instead), `setDeclination()` and
`setMagReference()` from WMM2025 at the pad (`TR_GeoMag`, on META `decimal_year` or the first fix's date; META
`declination_rad` and `field_ned_ut` "n e d" instead), with `--mag-error-deg` added to the declination as
`TR_BENCH_HEADING_ERROR_DEG` does; then `startSeed()`, published once a second as the seed. The ground pressure is
META `ground_pressure_pa`, else the first second's mean. The NED origin of the CSV is `--pad`, else META `pad_lla`
"lat_deg lon_deg h", else the seed's position.

### The day's broadcast ephemerides (`--rinex FILE`, repeatable)

`rinex_nav.cpp` reads RINEX 3 navigation files into `gps_eph_t` by `SbfNav.cpp`'s conventions (full GPS week of toe;
toe and toc in the system's own time, BeiDou's BDT; GPS T_GD, Galileo BGD(E5b,E1) of the I/NAV clock, BeiDou TGD1;
health 0 usable, with SbfNav's Galileo no-status and no-SISA marks; IODE/IODC, IODnav, AODE/AODC). Galileo F/NAV
(data source bit 1, or no E5b,E1 clock), BeiDou GEO and every other system are left out. A record reaches the stage
no earlier than a receiver first has it, by the mosaic's own nav blocks (the 10/15 P1 capture and the 10/16 logs): a
GPS set 2 h before its toe, a Galileo I/NAV batch 665 s after its toe, a BeiDou set 18 s after (BDT). Before each
epoch is offered, the two latest such records of each satellite the epoch carries are filed (`addEphemeris`; a
merged file's repeats of one issue are filed once), and none more than 4 h from the epoch (counted `rinex_stale`).
The approved file is `gnss-attitude-study/v9-mosaic-filter-plan/reference/rinex-2026-289/`.

### The unaided twin (`--unaided`)

`RawGnssEKFParams::att_gnss_off`: the GNSS rows' gain on the attitude and the gyro bias is masked at the update, as
the acceptance compares (plan section 4); the levelling, the magnetometer, the barometer and ZUPT act as ever. The
carrier update is not masked, so `--carrier 2` is refused with it. (The GNSS rows still move the accelerometer bias,
which the levelling takes out: TR_RawGnssEKF.h says so.)

### The apogee quorum (`--quorum`; on with the mosaic)

`flight_votes.h` runs main.cpp's kinematics block a flight-loop pass at a time (`--loop-hz`, 1000) with the
firmware's code: `RawNavAdapter` (the filter's published state brought forward through the IMU ticks, its health,
heights above the geoid), the transonic lockout on its speed, the pad reference (`ref_alt_m`: the receiver's fixes,
then the filter's own heights from its first GNSS-aided state, frozen at launch), `GnssAscentGate` (admitted against
the barometer only on the mosaic: `ekf_valid` false), the raw vote guard (`raw_vote_guard.h`, stepped after the gate
with `baroFusable`, reset at launch; it withholds the filter's two votes once latched, `--vote-guard 0` only reports
it -- this is the wiring D1b is to add; no firmware tree steps the guard yet, so `--vote-guard 0` is the image as
built), `TR_KinematicChecks` (the four leaky voters, the 2-of-N quorum, the Layer-2 backstop) and
`tr::burnoutDetectStep`.
Its inputs: each IMU tick, each barometer sample (pressure altitude on the ground pressure), and the receiver's fix:
each PVTGeodetic in the log, or `V` events (a sim's, or `extract.py`'s from a non-SBF log's GNSS records).

With the quorum on, the filter's tick flags are the quorum's, as `rawShadowPushImu` builds them
(`FlightVotes::shadowFlags`): `kShadowApogee` is its own call; the levelling in flight (`kShadowLevel`, the
firmware's `use_ahrs_acc`) starts at that call, so each twin's descent levelling starts at its own call; and
`kShadowLanded` is its kinematics' landed flag or the LANDED state. The events file's level bit is ignored in flight
and its landed bit counts only once the ticks are no longer in flight. The launch and the landing themselves (the
in-flight flag, `enterInflight`'s resets) are the ticks' own edge by default, a log's state machine; with
`--flight-state quorum`, or by default for an events file that says META `state_machine none` (a sim, which has no
flight computer behind its flags), they are the quorum's: INFLIGHT in the pass its kinematics' launch flag latches,
LANDED after `landing_transition`'s dwell on its landed flag, as main.cpp's state machine does. The barometer
reaches the filter through the same transonic lockout and stuck-barometer gate the EKF tick applies.

A voter that never existed changes the quorum (2-of-3 instead of 2-of-4, another latency): with no receiver fix or
no barometer sample (no META `ground_pressure_pa` on a restored run) the harness warns, and a mosaic run exits with
status 3. With the quorum on, the events JSON leaves out the older `pitch_vote_t`/`vel_vote_t` (first votes from
`--burnout-s` with simplified thresholds): read `quorum.votes`.

The guard has a gap the coherent coast fault falls in (G4/G5's pattern: the Dopplers and the PVT biased together by a
ramp from 0 to 5 m/s low over the 3 s before apogee). It latches only on a filter more than 10 m/s below the
barometer's rate, so at 5 m/s it never does: in `test_raw_replay_votes` (`ACoherentCoastRampIsTheGuardsGap`) the
filter's velocity votes 0.43 s before the true apogee with the rocket still climbing at 4.2 m/s, and the GPS vote
joins it for a call 0.17 s early (true vz 1.7 m/s). G5's bar of no raw vote above +2 m/s fails on that pattern
whatever this guard does; it needs another safeguard or another bar (an open item with the user). The test's 15 m/s
coherent fault (`ACoherentReceiverFaultIsWhatTheGuardIsFor`) shows the guard working on a fault it is built for,
not G5's pattern.

The events JSON gains `quorum`: the launch, the kinematics' own launch flag, burnout, GNSS admission (and why), the
guard's latch (with the filter's vz and the barometer's rate then), the apogee call and the backstop, as flight times;
each voter's rises and falls (`votes.vel`, `.baro`, `.gps`, `.pitch`: `on` and `off` lists), so a table can count
votes cast while the true vertical velocity is above +2 m/s; and what stood at the call (`at_call`: which voters
passed and were available, vz, pitch, the barometer's rate, the receiver's vU, the height). With the mosaic it also
has `receiver`, `stage` (epochs, untimed, built, ephemerides, `eph_sats`, `tracked_no_eph`, `capped_out`), `sbf`
(the feed's counts: off the grid, empty, tagged without a bias, clock jumps, nav blocks kept and dropped, ionosphere
sets, CRC failures), `eph_from` (the log's own, preloaded and when filed, RINEX, N events), `time_tag`,
`faults_clock`, the pad's zero-velocity updates (`pad_zupt`, `pad_zupts`, `landed_zupts`), `doppler` (each closed
epoch's Dopplers fused, refused, and its largest NIS: median and p95) and `mag` (samples handed, fused, refused as
uncalibrated, off the model's size or its dip; the median |B| against the model's). The 25 bitident files carry no
receiver velocity, so their quorum has three voters unless re-extracted with `V` lines.

### New event lines

| Line | Fields | |
|---|---|---|
| `V t fix_mode sats alt_m vel_n vel_e vel_u` | a receiver fix (a sim's PVT, or a non-SBF log's GNSS record): the GPS voter's and GnssAscentGate's | alt above the geoid, the filter's datum |
| `R t hex preload` | a navigation block `extract.py --sbf` put at the start: filed when a receiver has it | |
| `META state_machine none` | the ticks' in-flight and landed flags are not a flight computer's (a sim): the quorum's own state | |
| `N t sys prn week iode iodc health ura toe toc sqrt_a e i0 omega0 omega m0 delta_n idot omega_dot cuc cus crc crs cic cis af0 af1 af2 tgd` | an ephemeris in `gps_eph_t`'s units, to the stage | |
| `META seed_lla`, `seed_quat`, `decimal_year`, `declination_rad`, `field_ned_ut`, `pad_lla` | the cold start's (above) | |
| `X t mcu_us tow_s week n` then n x `sys prn has_pr has_rr sx sy sz vx vy vz ax ay az pr rr cn0 rr_sig_add` | a sim's epoch, released to the GNSS task at t (`sim_events.py`, below): each satellite placed (ECEF of the transmit time), its corrected pseudorange and range rate, C/N0 and an extra Doppler sigma | through the stage (`sim_epoch.h`), then the filter |
| `META seed_quat_truth 1` | `seed_quat` is a sim's truth: turned by `--mag-error-deg` (as the declination) and `--seed-heading-error-deg` | |

'F' stays the LC86G's fix (week, time of week, mode, satellites), so the Beetle's files parse as before.

### Scoring a static log: what it can and cannot show

- **Velocity.** On the pad the filter takes a zero-velocity update every 0.1 s while still (`pad_zupt`) and levels
  against gravity, and between them they hold a static log's velocity to millimetres a second whatever the receiver
  gives: on the 10/16 outdoor log, the log's own nav (one satellite with an ephemeris) scores 0.006/0.002/0.006 m/s
  rms, better than RINEX's nineteen (0.018/0.015/0.020). `--gnss-only-from-s T` stops both from flight time T on the
  ground and takes act's launch edge on the attitude there (`gnss_only_arm.h`: the flight gate, the heading prior,
  the tilt floor, again after any restart): the IMU, GNSS and the barometer alone, as in flight. The ground's own
  gate is cos^4 pitch, about 1e-7 on a payload standing nose up, and it scales only the attitude's gain, not the
  gyro bias's: an arm that kept it (`--gnss-only-gate ground`, E4's first run) left the tilt to the gyro alone while
  GNSS still moved the bias. Then the two separate: 0.91/0.21/0.025 m/s with the one satellite, 0.018/0.015/0.021
  with RINEX (from 20 s into the log, scored from 2 s later). Score G3's static velocity from such a run, with the
  `doppler` counts beside it. (`--pad-zupt 0` turns the ZUPT off from the start but keeps the levelling, which on a
  static pad still holds the horizontal velocity: 0.001/0.005/0.025 with the one satellite.)
- **Heading.** With the antenna on the IMU (r = 0) a still receiver says nothing about heading, so on a static pad
  only the magnetometer can, and on the 10/16 logs it never does: they read |B| 70-73 uT against the model's 50.5
  (uncalibrated), so every sample fails the field's size gate and none is fused. The heading there is the seed's and
  the gyro's; `--mag-error-deg` changes nothing (byte-identical CSVs at 0 and 40), and the harness warns whenever a
  run fused no magnetometer sample. So G3's heading p95 and the magnetometer-error arms cannot be tested on these
  logs as recorded, and seeding from the log's quaternion (`--seed-quat-from-log`) would pass the heading check by
  construction. They become testable on a log whose magnetometer passes the filter's gates (after the stored
  calibration, G10), or on the spin rig's turns scored against the rig's own angle, not the log's GpsInsEKF.
- **The descent as flown on a turning payload.** `--gnss-only-keep-level` (with `--gnss-only-from-s T`; from
  exp/v9-gyrobias) keeps the levelling on and hands the filter no magnetometer from T: no pad ZUPT, act's launch edge
  on the attitude, the IMU, GNSS, the barometer and the levelling -- what the filter has under a slowly turning
  drogue (in flight it levels once the drag is steady and g's size, LevelGate.h, which a payload turning steadily on
  the spin rig passes throughout; the ground's size test stands for it). With `--gnss-only-from-s` alone the
  levelling is off too: GNSS alone on the attitude, not a configuration that flies.

### Scoring a static or spin-rig log (`score_static.py`)

```
tools/raw_replay/score_static.py RUN.csv --from-s T0 [--to-s T1] [--log OC_LOG] [--rig RIG.npz] [--events EVENTS] \
    [--pvt PVT.csv] [--ref NOFAULT.csv (--cut-from-s A --cut-to-s B | --corrupt-at-s T)] [--json OUT.json]
```

Gate G3's numbers from one run, distances and statistics only (no position or height is printed or written): the
filter's stops in [T0, T1] (the whole run's beside them: a stop after the window is not G3's), non-finite and
unhealthy rows; the velocity's mean and rms per axis in [T0, T1]; per closed epoch (from the epochs file where there
is one) the satellites tracked and with an ephemeris, the Dopplers and pseudoranges used and refused, the
measurement rows the update worked (the CPU counts) and the longest any one satellite was refused; with `--log`,
tilt (the angle between the two filters' gravity directions) and heading (the twist about NED down, as is and from
T0) against the log's own flight filter (NonSensor q, the auto image's GpsInsEKF; its NonSensor is cached beside the
run); with `--events`, tilt against the accelerometer's own 1 s means (its bias in them), for the run and the log's
filter -- on a still pad GpsInsEKF is no tilt truth either, its split of tilt and accelerometer bias being its own;
with `--rig`, board +Z's bearing against the spin rig's own (an npz of t_mcu_s and bearing_deg, deg true); with
`--pvt`, the raw filter's height against the receiver's mean; each whole-millisecond step of the receiver clock with
the velocity across it (the epoch before and after, and the 1 s means, beside the same at every other epoch) and
each satellite's refusals within 5 s; and with `--ref`, the velocity, position and tilt against the no-fault run 2 s
after GNSS returns (one sample, and the mean over the next second), paired step for step so a fault run written
`--every 10` pairs exactly, with the pad ZUPTs around the fault and, for a corrupted epoch, how many of its
satellites the stage built and the filter refused.

What the 10/16-17 V9 logs showed (E4, revised after the 10/17 review; the report and its runner are in
`gnss-attitude-study/v9-mosaic-filter-plan/evidence/real-logs/`, outside the repo):

- No stop and no non-finite state in any scored window of an as-flown or RINEX run, faults included. Static velocity
  holds 0.014-0.025 m/s rms per axis as flown, and 0.018-0.025 GNSS-only with the flight's gate (aided).
- Tilt against the log's GpsInsEKF: 0.36-0.40 deg p95 as flown on the two spin-rig logs. Outdoors the raw filter
  made a 0.7 deg excursion 30-40 s after its start (tilt and body-z accelerometer bias together, with or without
  GNSS), not slow convergence. On 10/17 GpsInsEKF itself sits 0.56 deg off the accelerometer's gravity direction
  from the first sample: on a still pad it is no tilt truth, so `--events` scores against the accelerometer too.
- On the turning spin rig GNSS moves the gyro's bias about the rocket's axis within two minutes and the heading
  integrates it: with the rig's true heading at the start, 137-143 deg p95 off over 25 min GNSS-only (15-80 as
  flown on the pad, where the ground's gate does not scale the bias's gain), while its heading sigma says 6-23 deg;
  the unaided twin stays within 4-8.
- The -1 ms clock jump steps the velocity 0.072 m/s epoch to epoch (the bar is 0.05; the spread at every other epoch
  is 0.103 p95) and 0.006 as 1 s means (0.011 p95 elsewhere); no satellite is refused.
- 3 s cuts: as flown on the rig, 1 of 21 over 0.1 m/s 2 s after GNSS returns; GNSS-only, cone on 0 of 10, cone off 6
  of 11 (at most 0.42). Gross corrupted Dopplers are refused at the gate (or the vertical check drops the epoch);
  gross codes are fused on pseudorange epochs (0.3 m of position); errors under the gate leave nothing measurable.

### What it does not reproduce

- The driver's 1PPS pairing itself: the log's clock line is used as it was logged, and its K lines stand for the
  pairings (a pairing whose clock record the driver's queue dropped is missed: `k_lost` counts them where the K
  lines carry the seq). Its permit for the line to step later is in the K lines already. The 3-deep epoch queue and
  the two-slot built-epoch handoff never fill here.
- The GNSS task's cadence and the P4's build time: one 30 ms delay for every epoch.
- MeasExtra, the carrier (off on the mosaic), the PPS tag check (`ppsResidualUs`; the driver's console counts it).
- The flight loop's own rate and phase: the passes are a regular grid; the burnout detector and the launch fallbacks
  see the IMU tick's mean, not the 3840 Hz samples.

### Checked (2026-10-17, on fa7b2e2b + this lane)

- With no new flag, the 25 Beetle events files give byte-identical CSVs, events JSON and digests against the
  harness at fa7b2e2b: 150 runs (as recorded, flown, unaided observe, `--fw-defaults` act and observe with the
  emulated clock, cut and corrupt with the relock rule). The console is identical too but for three cut runs (a10,
  r19, r26), where it now says the corruption at T+1 hit no satellite.
- The 10/16 outdoor log (60 s, 3788 Hz IMU): 599 epochs at 10 Hz, all timed (598 with a PVT's bias; ReceiverTime
  FINETIME, a K line a second), no non-finite row, no stop; the log's own nav gives one satellite an ephemeris
  (Galileo), RINEX all 19 tracked. With RINEX the static velocity is 0.015-0.020 m/s rms per axis after 20 s with
  the pad's ZUPT and levelling on, and 0.018/0.015/0.021 on GNSS alone (`--gnss-only-from-s`, above); 16 satellites
  fused an epoch (the cap). Its magnetometer is never fused (above).
- The 10/16 cone-off spin-rig log (26 min): Galileo-only from its own nav (no download), but with each preloaded
  issue filed when a receiver could have it, only 2 satellites have an ephemeris at the start and the filter is
  unaided for the first ~6 minutes; 8 from T+367 s, 9 from T+967 s (filing everything at the start had shown 9 and
  aided throughout). With RINEX 26. Its -1 ms clock jump (T+1306 s) steps the clock bias by c x 1 ms with no
  velocity step above the noise and nothing refused. About 7-10 s a run.
- RINEX against the SBF decoder: every nav block of both 10/16 captures (GPS 3, Galileo 29, BeiDou 12) has its
  issue in the day-289 file, and the two place the satellite alike within 0.1 mm and its clock within 1 ps over
  toe +-2 h, health read the same (`test_rinex_nav`, private run).
- The quorum on nine Beetle rig logs (`--quorum`, the Beetle's `ekf_valid` gate): the harness's call falls within
  15 ms of the flight computer's own on eight (velocity and pitch), and 147 ms later on f2 without the receiver's
  fixes. Re-extracted with `V` lines, f2 calls at T+12.423 on velocity and GPS (GNSS admitted at T+6.52), 1 ms
  before the flight computer's T+12.424: the GPS voter path reproduces a real call. c30, f3c, r16 and r21 with `V`
  lines call as before (within 14 ms). The mosaic's barometer-only admission (`ekf_valid` false) has not met a real
  flight yet.

### Grid B: simulated flights (`sim_events.py`, `sim_grid.py`, `sim_tables.py`)

The firmware's one navigation filter on E0's 16 simulated drogue flights (gnss-attitude-study
`v9-mosaic-filter-plan/grid/flights`: Rolly Polly III on a G80T and the 54 mm Rolly Polly on an F52C, still air and
4 m/s, four seeds each), each with a fault pattern, against its `--unaided` twin: plan section 4, gates G4 and G5.

- **`sim_events.py FLIGHT.pkl OUT --pattern P`** writes the events file a V8/V9 would be handed: the EKF ticks
  (every second 1 kHz row), the barometer at 500 Hz, the magnetometer at 100 Hz, the receiver's epochs at its own
  20 Hz -- each fused-grid one (every 100 ms of GPS time) as an `X` line released 30 ms later, and every one's fix as
  a `V` line 25 ms after it (the mosaic streams PVTGeodetic at msec50; only the raw epochs are cut to the fused
  grid) -- and the cold start's META (the seed is the sim's truth: `seed_quat_truth 1`). The receiver (`MosaicRx`,
  the gnss repo's `RawGNSSModel`'s sky, clock and lock logic) is three Walker shells mapped to GPS 1-24, Galileo
  1-24 and BeiDou 19-42 on one clock, lock kept to 200 m/s^2 along the line of sight, a dipole-like C/N0 along the
  nose (46 dB-Hz broadside, 6 dB on the axis), and noise at the mosaic profile v0's fits for each satellite's C/N0
  (the Doppler's without the profile's 1.5 margin; 0.05 m/s per g of stress against the profile's 0.5). The
  antenna sits `--lever-arm` m (0.3) along the nose from the IMU (`--lever-arm-b x,y,z`: anywhere in the body);
  the filter keeps r = 0 unless given it (`--antenna-m`). `--roll` lays a roll over the flight (sim_roll.py's
  profiles, with `--windup-k`, `--gyro-clip-dps` and `--gyro-scale-ppm`), and `--roll-axis-tilt-deg D` rolls it
  about an axis D deg off the nose toward body +y (a spin axis off the board's x: the nose cones about it). The fix is a
  least-squares solution from the same measurements -- its error added to the truth's velocity of 4-6 ms before
  the epoch (the PVT's lag) -- so a receiver fault reaches the GPS voter as it reaches the filter. After touchdown
  it adds a 0.05 s impact and 5 s at rest (the flights end falling at 15 m/s). Once a flight it also writes the
  truth: `truth/<flight>.npz` (what the scores are made against) and `truth/<flight>.nav` (the same at the IMU, a
  line a 1 kHz row, for `--truth-nav`).
- **Patterns:** `none`; `cut05`, `cut15`, `cut30` (every epoch from T+0.5/1.5/3 to T+8 cut, the fused epoch before
  corrupted as c30's: GNSS comes back at or after the true apogee, so the coast flies unaided and boost and coast
  pass by construction); `gap05`, `gap15`, `gap30` (the same cut 0.5/1.5/3 s long from burnout: GNSS comes back in
  coast); `stale_rx` (the receiver at the pad's velocity through the boost, closing with tau 2 s: the closing tail
  with GNSS after, a bound); `stale_sats` (two Galileo satellites so, tau 4 s -- the filter is not told: no firmware
  path widens one satellite's Doppler sigma); `settle15`, `settle40` (each Doppler off by A exp(-dt/1.5 s) from
  burnout; settle40 also 2 m/s rms common to every Doppler over the first 5 s of coast, the rig's rp5
  `--post-thrust 4,1.5,2`); `lol0`..`lol3` (every satellite's lock lost 0-3 s before the true apogee, re-acquired
  as the model does); `coherent` (the receiver's velocity biased down by a ramp 0 -> 5 m/s over the 3 s before the
  true apogee, then held: every Doppler and the fix); `coherent15` (a fast coherent step, 0 -> 15 m/s over 1 s from
  3 s before apogee: the Doppler gate refuses it, so it is not the guard's case); `slow12`, `slow15` (slow coherent
  ramps reaching 12/15 m/s at the true apogee over 6 s, never starting before burnout) and `slow15b` (0 -> 15 m/s
  over 6 s from burnout): the filter accepts them and reads more than 10 m/s low before apogee -- the raw vote
  guard's own case; `slow08` (the same ramp to 8 m/s: between the plan's 5 m/s and the guard's 10 m/s, its gap); `baro_lag05`, `baro_lag07`, `baro_lag10` (the static port's first-order lag, GNSS clean: a
  sensitivity sweep of the guard's false-latch case, nothing measured on the Goblin). The GNSS faults are
  tc_ekf_eval's (Grid A's) where it has them, applied before the PRN mapping; each file says what it did in META
  `sim_*`.
- **The X path** (`sim_epoch.h`, `test_raw_replay_sim`): the stage's own `see()`, lock ages, cap and ionosphere
  choice on a stand-in ephemeris for each satellite the sim names, and the builder's C/N0 floors, elevation mask and
  noise model mirrored from its config (no stress term: the filter adds it). On the same sky the host test builds,
  epoch by epoch, exactly what the stage's own `offer()` builds from ephemerides: the same picks, sigmas and lock
  ages. The X line's `rr_sig_add` is for experiments only; no pattern sets it.
- **`--truth-nav FILE`** (`truth_nav.h`): the same quorum flown beside the filter's on a perfect one -- the sim's
  truth published as the filter's state each pass, with the run's own barometer, IMU ticks and fixes -- reported as
  `truth_quorum` in the events JSON: the truth's own call, which the counters' latency alone gives. It withholds
  nothing, but says when the guard would have latched on the truth.
- **`sim_grid.py OUT --flights GRID/flights --events-dir SCRATCH`** makes the events files (16 flights x 24
  patterns) and replays each in six magnetometer arms -- 0, +10, -10, +40, -40 deg (`--mag-error-deg`, the seed's
  heading turned with it: a long pad wait) on the 10 deg prior, and none (`--mag-flags 0`, the seed's heading
  guess 30 deg off, alternating in sign by seed) on the 30 deg prior -- as three arms: the flight design
  (`--receiver mosaic`, the raw vote guard applied as `flight_votes.h` wires it: D1b's wiring, assumed, since no
  firmware tree has it yet), its `--unaided` twin, and `noguard` (`--vote-guard 0`: the quorum RC0's main.cpp
  has). Every run gets `--truth-nav`. Each run is scored as it finishes (`sim_tables.score_run`) into
  `OUT/scores/`. `--harness-args "..."` adds raw_replay arguments to every run (an experiment: use a new OUT).
  An events file already in `--events-dir` is reused only when its truth's antenna (`META lever_arm_m`,
  `lever_arm_b_m`) and spinning body (`sim_roll_*`, `sim_windup_k`) are the ones asked for (`--lever-arm`,
  `--lever-arm-b`; `--roll`, `--windup-k`, `--gyro-clip-dps`, `--gyro-scale-ppm`, `--roll-axis-tilt-deg`, as
  `sim_events.py` takes them); otherwise, or with `--force-events`, it is made again. A file is named by flight and
  pattern alone, so give each roll profile its own `--events-dir` (its truth turns with it).
- **`sim_tables.py OUT [--against OTHER_OUT]`** writes `OUT/tables.md` and `OUT/verdict.json`: first what the runs
  assume (the guard's wiring, the 20 Hz fixes, the P1/G6 placeholders -- the 30 ms hand-over, the 500 Hz EKF
  tick, the 1 kHz loop, no epoch slicing without `--fc-unit-us` -- the port lags as a sensitivity sweep, and the
  effective sample: 10 distinct trajectories over 2 airframes, neither the Goblin); then per pattern and phase the
  runs passing p90 <= max(twin + delta, floor) (boost 1/2, coast 2/3, descent 3/5 deg) with the magnetometer arms
  pooled as the plan words it, and again per arm (the +-40 and no-magnetometer twins sit at 30-40 deg, so only
  0 and +-10 can fail), with magnetometer 0 counted once a trajectory; the pitch within +-2 s of the true apogee
  (and before the drogue's deployment); the worst excess over the twin before apogee and in descent; p90/p99/max per
  arm; the votes and calls while the true vertical velocity was above +2 m/s, for the design and noguard arms, with
  the margin to that bar; the call's latency against the twin's and against the truth's own; the vertical velocity
  at the true apogee; the guard's latches (the filter's error then, the should-latch runs, false latches, latches
  on the truth); and E3's unit tests. `--against` sets the no-fault runs at magnetometer 0 beside another grid's
  (the antenna on the IMU against 0.3 m up the nose).
- **`--reject-persist-rr N`** (an experiment on the filter's own parameter, after the profile): the Doppler's refusal
  count before a satellite is let back in (the mosaic profile's 30 epochs, 3 s at 10 Hz).
- **`--arm-args NAME=ARGS`** (repeatable): an arm of an experiment's own beside today's design -- the design flown
  with ARGS (raw_replay arguments) added, or, with NAME ending `_unaided`, its `--unaided` twin; name it in
  `--arms`. **`--rig-state FILE`**: no events file or run starts while FILE reads `active` (checked every 30 s),
  for a machine shared with the bench rig.
- **The Doppler change check** (`RawGnssShadow::Config::dv_*`, off by default; a `RAW_REPLAY_DVCHECK` build):
  `--dv-check 0|1|2` (off, observe, act), `--dv-sat-k`, `--dv-epoch-k` (the satellite and the epoch tests'
  sigmas), `--dv-floor`, `--dv-per-g`, `--dv-att-deg`, `--dv-scale`, `--dv-bias`, `--dv-lever-m` (the change's
  floor and the epoch test's allowances), `--dv-hold-s`, `--dv-epoch-hold-s`, `--dv-window-s` (the leaky sum),
  `--dv-latch-k` (a failed epoch holds every Doppler until the Dopplers' level agrees with the filter's velocity
  within this many sigmas), `--dv-deweight`, `--dv-ground`, `--dv-noise-tau-s`, `--dv-hold-all-frac`,
  `--dv-hdg-unset-deg` (a `RAW_REPLAY_DVHDG` build: after a launch with no magnetometer heading -- the sims'
  `--mag-flags 0` -- the epoch test also allows the IMU's change turned this far about the vertical; 180 any
  heading). The events JSON gets `"dv"`: the counts (checks, satellite and epoch fails, Dopplers and epochs held,
  latches begun and let go, epochs latched), when each epoch failed, had satellites fail or had Dopplers held, and
  the heading allowance's setting and whether the flight took it, as its launch left it (`hdg_unset_deg`,
  `hdg_unset`). `RAW_REPLAY_DVLOG=1`
  prints each check (2: the window's sums and allowances too). `sim_tables.score_run` keeps the block.
- **`--hdg-obs-mode 0|1|2|3`** (exp/v9-observ, a `RAW_REPLAY_HDGOBS` build): GNSS's gain on the heading -- the
  attitude about the vertical and the gyro bias along it -- scaled by how well the moment observes it
  (`RawGnssEKFParams::hdg_obs_mode`, off by default): 1 by the filter's own horizontal specific force
  (`--hdg-obs-fh X`, m/s^2, averaged over `--hdg-obs-tau X` s), 2 by the heading's information from it against the
  heading's own sigma (`--hdg-obs-tau`, `--hdg-obs-sigma-v X`), 3 closed for `--hdg-obs-window X` s after the boost;
  open whenever the IMU read past `--hdg-obs-open-g X` (2 g) within `--hdg-obs-open-hold X` (0.2 s) of the epoch,
  at least `--hdg-obs-floor X`; `--hdg-obs-scope 0|1|2` everywhere (the pad too), in flight, or launch to the apogee
  call. The events JSON gets `hdg_obs` (the settings and each closed epoch's share), and `sim_tables.score_run` the
  mean and least share per phase.
- **`sim_grid.py --pause-while FILE`**: the same as `--rig-state FILE` (exp/v9-observ's name for it).
- **The turning body** (`RawGnssShadow::Config`, off by default; a `RAW_REPLAY_ROT` build, on unless the tree is
  older than exp/v9-robust): the antenna's offset from the IMU is never measured before a flight, and the sim puts
  it 0.3 m up the nose, which at the drogue's snatch (5 rad/s) moves 1.5 m/s against the IMU. `--rot-lever-axial-m A
  --rot-lever-radial-m R` give every Doppler, in quadrature, the most such an antenna can move:
  A x the rate across the nose + R x the whole rate, both the gyro less its bias at its largest since the epoch
  before (`RawGnssShadow::rotLeverSigma`; `R` alone is the isotropic bound |omega| r); `--rot-lever-min-rate M` only
  on an epoch whose rate across the nose reached M rad/s (the snatch, not the boost's weathercocking or the pitch-over
  at apogee). `--att-hold-rate W
  --att-hold-s S` hold GNSS off the attitude and the gyro bias (the unaided twin's mask) for every epoch whose rate
  across the nose reached W rad/s and for S s after one; a roll alone never holds it. `--rot-skip-rate W
  --rot-skip-s S` fuse nothing of those epochs instead (each closed as the vertical check closes one it drops): the
  heading error the turn builds into the velocity is then still there for the first epoch after it, where a hold
  would have let the velocity take it. `--rot-after-apogee` lets all three act only once the flight computer has
  called apogee (the drogue's snatch and the descent; the boost, the coast and the votes fly as without them). The
  events JSON's `rot` gives the settings, the largest allowance taken, the epochs held and the epochs skipped. The
  mosaic image sets them in `gnss_raw::applyMosaicFlightSafeguards` (`TR_RawGnssShadow/ReceiverProfileApply.h`),
  which `--receiver mosaic` applies too: `rot_lever_axial_m` 0.30, `rot_lever_radial_m` 0.03, `rot_lever_min_rps`
  2.0, the hold and the skip off (every other image: all off).
  Results (2026-10-17, gnss-attitude-study/v9-mosaic-filter-plan/evidence/wave3/robust/scorecard.md, outside the
  repo; 15 patterns x 6 magnetometer arms x 16 flights an arm, the antenna 0.3 m up the nose and on the IMU):
  - The allowance only on epochs that turned across the nose at 2 rad/s or more (`--rot-lever-axial-m 0.3
    --rot-lever-radial-m 0.03 --rot-lever-min-rate 2`) takes out 76 % of the snatch's no-fault error (its first 3 s,
    p90 median 7.8 -> 2.6 deg, worst 41.8 -> 10.4) and leaves every apogee item the design's (a windy boost that
    reaches the rate moves vz at apogee by mm/s). On the IMU, a heading already wrong at the snatch is corrected
    about a second later, which the descent p90 and the pitch near apogee count; under the drogue it costs
    nothing. A 2.5 rad/s floor never acts before apogee here but misses the still-air flights' gentler snatch.
  - Without a floor the allowance weakens the filter's vz near apogee: more early calls (102 -> 120). The hold lets
    the velocity take the snatch's epochs and loses the heading error they carry (the descent's worst excess over
    the twin 29 -> 92 deg). The skip keeps it, but after a long turn the velocity error it leaves can be too large
    to read back (one coherent-fault run diverged to 48 deg where the allowance recovered to 1.4). Acting only
    from the apogee call misses a snatch that comes before the call (a late call, an early ejection).
  - On the four real V9 logs the floored allowances never act (byte-identical to the design).
- **The antenna's offset, known** (`RawGnssShadow.h`, "The antenna's offset"; a `RAW_REPLAY_ANTENNA` build, on unless
  the tree is older than fc/v9-antenna-offset): `--antenna-m x,y,z` (body FRD, m: the IMU to the antenna's phase
  centre) gives the filter the offset the owner's setting (BLE cmd 81) gives the image, through
  `gnss_raw::applyAntennaOffset` after the safeguards, as `startRawShadow()` does: every Doppler and pseudorange is
  predicted at the antenna, and what the offset could be off by (`--antenna-sigma-m axial,radial`, the image's 0.05,
  0.02, from `--antenna-min-rate` rad/s across the nose, the image's 0: any) replaces the turning body's 0.3 m bound
  and the change check's lever allowance. `--antenna-att-mode 0|1|2`: the offset's attitude terms considered only,
  also observed (the image's: the change check's lever roll), or fused. The epochs file gains `lever_roll_epochs`,
  `lever_roll_deg`, `lever_roll_sigma_deg` (the lever roll so far and the last epoch's, RawGnssShadow's
  `lever_roll_*`) and `rot_sigma_mps` (the epoch's turning-body allowance); the events JSON an `antenna` block (the
  settings and the run's lever-roll sums). A flight's log carries the offset set on it and the one its navigation
  filter flew with (the pad context's `PCX_ANT`: `applied_body_m`, which a setting sent just before launch may not
  have replaced yet; its `used`: the filter flew with the setting). `extract.py` writes the one the filter flew with
  as `META antenna_m` (and `antenna_used`, 1 if that was the setting): the harness takes it where `--antenna-m`
  gives none, says so on stderr, and `--antenna-m none` replays the flight unknown. A setting the filter did not fly
  with (it was still on an earlier offset, or on none) is `META antenna_m_set` (and `antenna_used 0`): the harness
  only names it on stderr, flying the earlier offset (`antenna_m`) or unknown, as the flight did, and says that
  `--antenna-m x,y,z` applies it. With a known offset the boost guards' Doppler fit (`--guard-doppler-rms`,
  `--guard-dv-*`) and `--diag-from-s`'s innovations predict at the antenna too, as the filter does.
  `sim_events.py --lever-arm-b x,y,z` (and `sim_grid.py`'s) puts the truth's antenna anywhere in the body, not only
  up the nose. The evaluation (Grid B with the antenna 0.10 m and 0.3 m up the nose, the offset given right, 3 or
  5 cm wrong, and not given) is in gnss-attitude-study/v9-mosaic-filter-plan/evidence/antenna-offset/ (outside the
  repo).
- **The Dopplers in pairs** (`MosaicRawEpoch.h`, "The epochs between the grid's"; a `RAW_REPLAY_DOPAVG` build, on
  unless the tree is older than exp/v9-dop-avg): on with `--receiver mosaic` where the flight profile is
  (`RAW_REPLAY_FLIGHT_PROFILE`, above), as the image flies them; `--rr-pair-avg 0` is a bench image's
  `TR_BENCH_RAW_DOP_AVG=0`, and `--rr-pair-avg 1` turns them on in a build without the flight profile.
  The feed reads each MeasEpoch off the fused grid for its Dopplers (`noteMeasEpochDopplers`) and averages each grid
  epoch's with them by the driver's rule (`averageDopplerPairs`: the epoch 50 ms before, read whole, no clock jump
  between, valid and at the floor at both ends, the lock time not down, within `--rr-pair-max-diff`, 15 m/s). An
  epoch with at least half its Dopplers in pairs is handed over in pairs (`TimedRawEpoch::rr_paired`): their means,
  each one left alone dropped, and the filter takes its Dopplers the profile's 25 ms further back (`rr_pair_lag_s`);
  any other epoch (none before it, a clock jump, fewer pairs) as it came, at its own instant. It needs the 100 ms
  grid over the receiver's 50 ms (`--fuse-ms 100`): `--rr-pair-avg 1` on another grid is refused, and the flight
  profile's pairs on another grid go off with a line on stderr, as the image's start turns them off. A sim's
  20 Hz epochs (`sim_events.py` at rate 20) are cut to the fused grid first, as the driver cuts the receiver's
  (`--sim-fuse-grid` does the cut alone: on Grid B it is byte-identical to the rate-10 events, whose draws are the
  same), and paired by the same rule (a sim has no lock time or clock jump). `--rr-pair-sigma X` scales each averaged
  Doppler's sigma, offline only (no firmware path). The events JSON gains a `pairs` block: the counts (the epochs as
  they came and why, the Dopplers dropped and why), the two lags, and each grid epoch with a Doppler not averaged
  (flight time, usable, averaged). Two offline diagnostics: `--rr-lag-ms X` sets the filter's `rr_lag_s` after the
  profile (every epoch's own lag; an epoch in pairs adds `rr_pair_lag_s` to it), and `--sim-pair-exact` (a sim with
  `--truth-nav`) puts the truth's second-order term
  back into each averaged Doppler, so the pair is the range rate at its middle exactly.
- **Each satellite's bands in one Doppler** (`--rr-multiband 1`; a `RAW_REPLAY_MULTIBAND` build, on unless the tree
  has no `TR_GNSSReceiverMosaic_Serial/MosaicMultiband.h`; offline only, no firmware includes the header). The filter
  fuses one Doppler row a satellite from its L1-band signal (GPS L1 C/A, Galileo E1, BeiDou B1I); the mosaic also
  tracks GPS L2C and L5, Galileo E6, E5a and E5b, BeiDou B1C, B2a and B3I, and a Detailed log's MeasEpoch (the R
  lines, byte for byte) carries them all. With `--rr-multiband 1` the feed builds each epoch with
  `rawEpochFromMeasEpochMb` and each epoch between with `noteMeasEpochDopplersMb`: the same cells as the image, every
  anchor field the same but the range rate, which becomes the inverse-variance mean of the anchor and its other
  bands (each from its own C/N0), and has_rate, cleared where two agreeing bands vote the anchor out. The pairs then
  average the combined rates unchanged. The rules (the header's top comment has them in full): a band joins from its
  anchor's channel with a lock of at least 1 s and its C/N0 at least its floor + 2 dB (stays to the floor); each is
  first referred to its anchor's wavelength with the epoch's own wind-up estimate (the median of
  (r_b - r_a) / (lambda_b - lambda_a) over the epoch: no gyro), so the row carries exactly its anchor's wind-up and
  the filter's correction stays right; a band more than max(5 sigma, 0.10 m/s + 3 ms x the anchor's acceleration)
  off its anchor leaves; an epoch with fewer than 3 wind-up values, or more than half its bands off, goes anchor-only.
  Options:

  | Option | What | Default |
  |---|---|---|
  | `--rr-multiband 0\|1` | the combination | 0 (the image's L1 rows, digest-identical) |
  | `--rr-mb-bands LIST\|none` | SBF signal numbers that may join | 3,4,19,20,21,13,14,30 |
  | `--rr-mb-model measured\|equalhz\|FILE` | each band's Doppler noise, one 20 Hz epoch: sigma^2 = scale^2 (a^2 + b^2 10^(-(C/N0-35)/10)); FILE lines `sig a b [scale [cn0_min [lag_ms]]]` | measured (the noise measurement's fit, a 0, floors 25 dB-Hz, lags 0) |
  | `--rr-mb-sigma 0\|1` | 1: each combined row's white Doppler term at the effective C/N0 that gives its combined noise on the builder's own model (what a uint8 in RawObs's padding would carry) | 0: the anchor's sigma kept, as the pairs keep theirs |
  | `--rr-mb-windup ref\|lam\|none` | ref: refer each band to its anchor's wavelength; lam: leave the row at its weighted wavelength and correct the rest from the gyro (the filter's wind-up ring, `windup_k`) before the filter takes it; none: neither (the error left in) | ref |
  | `--rr-mb-min-lock-s`, `--rr-mb-hyst-db`, `--rr-mb-screen-k`, `--rr-mb-screen-floor`, `--rr-mb-anchor-vote 0\|1`, `--rr-mb-max-bands N`, `--rr-mb-w-min N` | the join rule, the screen, the anchor's vote, bands a row takes, wind-up values an epoch needs | 1 s, 2 dB, 5, 0.10 m/s, 1, 3, 3 |
  | `--rr-mb-trace FILE` | a CSV row per satellite and epoch looked at (grid `g` or between `o`): its anchor's C/N0, what the row became, the epoch's wind-up estimate, each band's signal, C/N0, weight, referred difference and whether it joined. No positions | off |

  Refused on a sim's epochs (X lines carry one signal a satellite) and with `--receiver lc86g`. The events JSON gains
  a `multiband` block and stderr a `multiband:` line: the combinations and why any went anchor-only, the rows
  combined and the mean sigma ratio, each band's candidates and joins, the screen's and the vote's counts,
  composition changes, the wind-up estimate's rms, and what `--rr-mb-sigma 1` and `--rr-mb-windup lam` changed. A log
  whose MeasEpochs carry no other band (the Reduced GNSS log keeps one a second: no 10 Hz path at all) joins none, and
  the harness says so. `extract.py --sbf` writes `META sbf_signals` (each SBF signal number's count over the
  MeasEpochs, the main antenna), which the harness keeps and does not read, so a file with it replays exactly as one
  without. With `--rr-multiband 0`, and with `--rr-multiband 1 --rr-mb-bands none`, every digest is the image's.
  Where a firmware version would differ: the header's capture rides in the adapter's own pass over the block (one
  read), but the effective C/N0 (`--rr-mb-sigma 1`) needs a uint8 in RawObs's padding and the builder's white term
  at it (both builds), and `--rr-mb-windup lam` a uint16 effective wavelength in RawObs's padding copied into
  RawMeas's (offset 4) for applyWindup and the wind-up monitor.
- **`sim_compare.py BASE NAME=EXP ...`** sets experiment grids (each a `sim_grid.py --arms design --harness-args
  "..."` run into its own OUT) beside the base grid's design and unaided twin, flight by flight: the phase bars per
  pattern (pooled over the informative magnetometer arms and over all of them), the worst excess, the deployment's
  first 3 s, the pitch near apogee and the quorum (early calls and raw votes, latency, vz at apogee, guard latches).
  Every arm is judged against today's twin.
- **`sim_grid.py --pause-file FILE`**: before each events file and each run the workers wait, checking every 30 s,
  while FILE says "active" (a bench rig's timing run needs the machine quiet).

A run takes 0.2-0.6 s (about 70 s of flight, 16 satellites fused an epoch at 10 Hz, ~9 capped out, and the truth's
quorum beside it); an events file 7-10 s; the whole grid (384 events files, 6912 runs in three arms) about 20
minutes on 8 workers of a loaded machine. The events are deterministic (per flight the receiver's draws, per flight
and pattern the fault's: the same bytes on a re-run), and so is a run (its CSV and events JSON byte-identical).

Checked (2026-10-17, gnss-attitude-study `v9-mosaic-filter-plan/evidence/grid-b/r2/main/tables.md`, the antenna
0.3 m up the nose; `r2/lever0` with it on the IMU): no stop or non-finite row in any of the 6912 runs. No fault:
pad attitude p90 0.15 deg at magnetometer 0; the quorum on the truth calls 0.18-0.30 s after the true apogee (the GPS
voter at 20 Hz), and the filter's call is within -8..+48 ms of it. G4 fails as worded and per magnetometer arm. G5
fails with the guard as assumed (an 8 m/s coherent ramp, under the guard's 10 m/s, calls 0.39-0.46 s early at a
true 3.8-4.4 m/s in every run; the 12 m/s ramp in the 6 runs of one F52C trajectory and the 15 m/s ramps in one run
too) and as built (every slow ramp calls 0.49-1.20 s early, at a true 4.8-11.7 m/s). The guard latches in every run
the truth says it should (190, the slow ramps), in no no-fault run, and on a correct filter -- the truth itself --
in every run with the static port lagging 0.7 s or more, the call then 1.8-2.4 s after the true apogee instead of
0.2-0.35 s. With the antenna on the IMU (`sim_tables.py r2/main --against r2/lever0`) the no-fault runs at
magnetometer 0 hold descent p90 0.70 deg median (1.12 worst) against 2.10 (6.56), the drogue's first 3 s 1.00
(1.57) against 7.84 (41.8), and the filter's velocity vote moves from 15 ms before the truth's to with it. The
Doppler's refusal count (`r2/exp_reject_persist_rr{3,10,255}`, `--harness-args "--reject-persist-rr N"`, settling,
stale-satellite and coherent patterns at magnetometer 0): 255 (never let back) flies as the profile's 30; 3 and 10
make settle40's worst coast excess over the twin 41-43 deg against 28.5.

## The Beetle's bit-identity check (`bitident.sh`, `../m1_identity.sh`)

Code the Beetle (M1, LC86G) shares with the mosaic-G5 work must leave the Beetle's filter bit-identical. Two scripts
check a tree against a base commit:

```
tools/raw_replay/bitident.sh BASE [HEAD]   # the filter replayed: a digest of every step, compared
tools/m1_identity.sh BASE [HEAD]           # the M1 image's main.cpp, main/ and collectors, preprocessed, compared
```

BASE and HEAD are commits (archived, so only what is committed counts) or directories holding a source tree (a
worktree's path, for changes not committed yet); HEAD defaults to `HEAD`. A commit is named as the worktree the
script is *run from* sees it: `HEAD` is the caller's HEAD, not that of the worktree holding the script, so a lane
can run another worktree's copy of the scripts from its own worktree. Run from outside every worktree of the
repository, names are the script's worktree's. The scripts print the commit each name came to; a branch name or a
worktree path leaves no doubt. Both exit 0 when identical, 1 on a difference and 2 when the check could not run, and
keep their scratch directory (printed; `BITIDENT_TMP` and `M1_IDENTITY_TMP` name one, which must be new or empty).

### `bitident.sh`

- The harness is the one frozen at 771e8374 (the commit that made the digest take the pad context; 4b647494 added
  `--digest`), built twice through `SRC_ROOT`: against BASE's firmware sources and against HEAD's, with
  `RAW_REPLAY_VCHECK` and `RAW_REPLAY_LATCH` on. Later harness work never has to compile against BASE.
- Each events file in `BITIDENT_IN` is replayed in act (`--heading-mode 2`) and observe (`1`) at magnetometer errors
  0, 10 and 40 deg (`BITIDENT_MAGS`), with `--fw-defaults` (the M1 image's configuration without bench flags) and
  `--digest`. The default inputs are `gnss-attitude-study/rig-replay/bitident-in/` beside the repository: the 25
  Beetle events files, kept out of the repository, and `recorded.txt` ("tag deg", the magnetometer error each log's
  image was built with; 0 if not listed).
- Each of those runs is made three times (`BITIDENT_UNITS`, default `0 100 300`): without a clock, so each epoch is
  worked in one step, and with the emulated FC clock at 100 and 300 us a reading (`--fc-unit-us`), so epochs are
  worked a slice a step (`epoch_slice_us`, `epoch_job_max_s`) as on the M1. The emulated clock counts readings: a
  tree that reads the clock more or less often on the M1's path slices its epochs differently and can differ in the
  clock runs alone. Read such a difference; it is not noise. The summary counts the runs that differ by clock.
- Every digest is compared byte for byte. A difference names the run (`TAG_CONFIG_mMAG_uUNIT`), the first step that
  differs and which of its hashes do. The 450 runs take about 30 s on a Mac.
- A tree the frozen harness cannot compile (a field it names renamed or gone) fails with status 2: a change to look
  at, not a pass. A tree whose filter needs a source file or include directory the frozen CMakeLists does not list:
  `BITIDENT_EXTRA_SRC` and `BITIDENT_EXTRA_INC` (';'-separated, relative to the tree, each used where it exists).
  Those the script knows of are in its `DEFAULT_EXTRA_SRC`, used where a tree has them, so the command above needs
  no variable: today the builder's cheaper build, `TR_GNSSRaw/RawMeasBuilderLite.cpp`.
- If a rebase rewrites the frozen commit, the script finds the newest commit carrying its subject; otherwise set
  `BITIDENT_HARNESS`.

`--digest FILE` (any run of a `RAW_REPLAY_VCHECK` + `RAW_REPLAY_LATCH` build) writes a line each filter step,
whatever `--every`: the step count, the IMU time (us), then FNV-1a hashes of the `ShadowNav` (every field), the 0x96
and 0x9B records the step filled, its `HEADING_FIT_MSG` records and its `PAD_CONTEXT_MSG` record (`-`: none that
step; the pad context is taken as main.cpp's raw task takes it, at most one record each 20 ms), a named list of the
`Stats` counters (not the CPU times), and the filter's state and covariance (`stateHash`, as in
`test_raw_gnss_shadow.cpp`). An `end` line closes it with the step count and the last hashes of the `ShadowNav`, the
counters and the state. Every field is named in `raw_replay.cpp`, so a tree that only adds fields hashes the same.

`--fw-defaults` applies `--pad-sigma-deg`, `--handoff-deg`, `--gate-full` and `--tilt-sigma-deg` only when given, and
leaves the rest at `RawGnssShadow::Config`'s defaults, so the replay flies what the Beetle flies; `--heading-mode`,
when not given, is then the primary build's `TR_HEADING_FIT` (2, act). Without it the harness sets all five from its
own defaults (observe, 30, 10, 0, 0: the earlier rig studies'), as before. (bitident.sh always gives the heading
mode, and builds its own frozen harness.)

What `bitident.sh` cannot see:

- Code the 25 flight segments barely run, and thresholds they never come near. Each events file starts from a
  restored pad context 1.4 to 5.6 s before the flight computer calls INFLIGHT and ends about 2 s after the landed
  flag, so levelling and its gate, resync, the pad's zero-velocity updates and the pad survey run for a few seconds
  at most. Changing one `RawGnssShadow::Config` default at a time in a copy of fd45b296 left all 450 digests
  identical for `still_rate_rps` 0.05 -> 0.04, `resync_period_s` 1.0 -> 1.1, `max_epoch_age_s` 0.6 -> 0.5 and
  `vert_check_mps` 20 -> 19. A changed default is therefore not caught as such: the `Config` pins in the tests guard
  the default values themselves (A4). Covering the pad and the landing needs an input that starts at boot, with no
  restore, and runs past landing; there is none yet.
- `GNSS_CHECK_MSG` (`takeCheckLog`): the events files carry the receiver's fix times, not its fixes, so the harness
  feeds the filter none and the fix check never runs.
- The M1's real timing: the emulated clock slices epochs by a count of readings, not by the time the work takes on
  the ESP32-S3, and the host compiler's floating point is not the Xtensa's. On-target checks (P4) cover those.

### `m1_identity.sh`

- Both trees are copied into the scratch directory and configured for the M1 (`idf.py -B ... -DTR_BOARD_M1=1
  reconfigure`; `idf.py` from the PATH, or `IDF_EXPORT`, default `~/esp/esp-idf-v6.0/export.sh`). No tree's
  `dependencies.lock` or `managed_components` is touched. The private `TR_GuidancePN` submodule is never copied, so
  both sides build with its stub.
- The compile commands of `main.cpp`, `TR_Sensor_Collector.cpp` and `TR_Sensor_Collector_Sim.cpp` are re-run with
  `-E` (line markers kept). Kept, file by file: the lines of the file itself and of every file under
  `projects/flight_computer/main/` it includes (`config.h`, `board/*.h`, `raw_nav_adapter.h`, the `*_policy.h` ...),
  those whose preceding line marker names that file, without blank lines; a file with no lines left is not kept. The
  tree roots, `FW_GIT_SHA`, `FW_GIT_DIRTY` and every integer `__LINE__` produced are normalized. `__LINE__`'s are found
  exactly: each compile is run again from a copy of the file, and for `main.cpp` of every source file in `main/`, each
  starting with `#line 1000001` under its own name, and an integer that moved by 1000000 came from `__LINE__`.
- Each kept file is diffed (one kept on one side only is a difference), and so are the `TR_*` and `CONFIG_*` macros
  `main.cpp` sees (`-dM`).
- Not compared: headers outside `main/` (shared headers gain defaulted fields and methods by design; the filter's
  sources are `bitident.sh`'s job, the rest the host tests'), compile flags other than those macros, the linker
  fragments (`main/*.lf`) and the CMake files.
- A new macro in `config.h`, even one that is 0 on the M1, shows in the `-dM` diff: define a mosaic-only switch from
  the mosaic build's CMake, or read the diff and judge it.
- About 30 s on a Mac, most of it the two configures.

### Checked (2026-10-17, base fd45b296)

- `bitident.sh fd45b296 fd45b296`: 450 runs identical. In copies of fd45b296's `RawGnssShadow::Config`:
  `epoch_job_max_s` 0.3 -> 0.2: 123 runs differ, 27 at 100 us a reading and 96 at 300 us, none without a clock
  (status 1); `pad_context_delay_s` 6 -> 5: all 450 differ, in the pad context only; `still_rate_rps` 0.05 -> 0.04:
  identical (above). At 4b647494: `heading_pad_sigma_deg` 10 -> 11 made the act runs differ from their launch step,
  and a renamed `Stats` field gave status 2.
- The revised harness (771e8374) against 4b647494 on r21, s10, f3c and a10: the CSVs and every digest column but the
  new pad context one are byte-identical; each run takes 17 to 21 pad context records.
- Run from another lane's worktree with `HEAD`, both scripts compared that worktree's HEAD (c178b415, fc/v9raw-core).
- `m1_identity.sh fd45b296 fd45b296`: identical (main.cpp and 15 headers of main/, the two collectors, 1074 macros). In
  copies of fd45b296: `FLIGHT_LOOP_UPDATE_RATE` 1000 -> 999 in `config.h` and `kMaxStaleUs` 200000 -> 150000 in
  `raw_nav_adapter.h`: those two lines are the whole diff, status 1. Blocks under `#if TR_RAW_NAV_MOSAIC` at the top
  of `config.h`, `raw_nav_adapter.h`, `board/board_m1.h` and `main.cpp`, and a new header included unguarded whose
  body is all guarded: identical. A `__LINE__` at the end of `config.h`, moved by a guarded block above it:
  identical. At ca4f8b89: one unguarded line added to `startRawShadow()` was the whole diff, status 1.
- Main's harness (cd6ebe36) against the one the 2026-10-16 firmware sweeps used (`fw_replay/build_l`, built at 17:58
  that day): on the 25 events files x {flown, unaided} x {0, 10, 40} deg, 120 of 150 CSVs are byte-identical, and all
  150 with the latch off (`--vert-release-mps 0`). The other 30 (c30, gv10, r21, v10 and zz, a copy of r21) differ
  where the latch engages: build_l predates the latch as committed (4755f128, 19:55).

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
