# Real-time raw-GNSS filter on the flight computer (Tinker-Beetle LC86G)

A tightly coupled GNSS/INS filter that uses the LC86G's raw measurements (RTCM MSM7: pseudorange and
Doppler per satellite, once a second) instead of the receiver's own fix, run on the flight computer
beside today's filter. Tracked in #1600.

## Why

On the COCOM rig (2026-10-12), the Beetle's LC86G flew four low-altitude flights through the
HackRF: Rolly Polly V L2 (real and its simulated stand-in), Eagle Claw and Rim66. Scored against
the injected truth, per phase (RMS):

| | Receiver's own fix | Today's flight filter on that fix | Raw-measurement filter (prototype) |
|---|---|---|---|
| Altitude, boost / coast / descent | 2.7-42 / 7.5-91 / 9-19 m | 1.1-12 / 17-30 / 14-64 m | 0.3-1.0 / 0.6-2.1 / 0.3-3.0 m |
| Vertical velocity, boost / coast | 22-53 / 9-19 m/s | 1.0-8.0 / 7.2-8.7 m/s | 0.01-0.4 / 0.05-0.3 m/s |

The receiver withholds its fix from about T-1 s to T+4-8 s, comes back 20-90 m off, and the flight
filter carries that for tens of seconds (no barometer in this comparison; the flight filter fuses
one). The raw measurements keep flowing through the withheld window, and an IMU-propagated filter
updated by them holds metres throughout. Related: #1508 (valid-flagged LC86G fixes kilometres off).
The prototype and the bench data are in the gnss submodule (`sim/tc_ekf_cocom.py`,
`sim/score_lc86g_bench.py`, `cocom-rig/sdr/results/lc86g_tc_bench/`).

## What the prototype established

- **Ephemeris from the receiver.** With `$PAIR436,1` the LC86G sends the broadcast ephemeris as RTCM
  1019 / 1046 / 1042, about one satellite a second. Decoded, it equals the day's broadcast file.
- **Atmosphere.** Saastamoinen troposphere. The Klobuchar ionosphere parameters are not in RTCM:
  without them absolute altitude reads 6-10 m high, but altitude above the pad and velocity are
  unchanged (it is a near-constant bias). They come from pad calibration (below).
- **The receiver steers its clock.** At its first fix the LC86G steps its code clock and its Doppler
  together, by an amount that is not whole milliseconds (-2629 km and -71 m/s on the rig); the
  filter takes any step every satellite agrees on.
- **Timing.** MSM7 epochs fall on whole GPS seconds and reach the flight computer roughly 0.1-0.25 s
  later; the Doppler is the range rate 55.5 ms before its tag.
- **Measurement noise.** A satellite missing for more than 2.5 epochs has relocked, and its
  pseudorange starts biased (~27 m); the Doppler of a loop under stress is unreliable (+0.5 m/s of
  sigma per g of specific force above 1 g, +3 m/s at a relock). With those, the horizontal error
  through Rim66's boost and coast fell from 4.7-10 m to 0.4-0.6 m.
- **States.** The flight filter's 15 (position, velocity, attitude, accelerometer and gyro biases)
  plus clock bias, clock drift, and the Galileo and BeiDou time offsets: 19. The PX1105R's Doppler
  rate offset and lag states and the carrier clones are not needed for the LC86G.

## Design

**Shadow mode first.** The new filter keeps its own state, reads the same IMU samples and the MSM7
epochs (unused today), and is logged; nothing it computes reaches `GpsInsEKF`, the state machine or
deployment until flights have validated it.

- **All of it on core 0, in a task of its own** (phase 4 changed this; core 1 was the first plan).
  The task runs at priority 1, below the sensors and the I2S sender. It does the ephemeris decode into
  a store; the satellite positions and clocks, troposphere and ionosphere (double precision); the
  per-satellite noise; and the 19-state filter in float, in the flight filter's error-state form.
  The flight loop on core 1 only copies each EKF tick's IMU input into a lock-free ring and, once a
  second, the flight filter's state into a seqlock; neither can make it wait. Each epoch is applied
  at its own time, from the IMU's velocity history.

  Why it moved: profiling (2026-10-12) found the Beetle's flight loop instruction-fetch bound. It
  runs ~340 passes a second against a 1 kHz design, and the flight task spins at top priority, so
  anything added to core 1 comes straight out of the loop rate. Core 0 is ~70 % idle.
- **Time without PPS.** The M1 wires the LC86G's 1PPS only to an LED: keep the receiver's GPS time,
  backdate each UART read by the bytes still buffered, track the GPS-to-`esp_timer` offset with a
  minimum-latency filter over the 10 Hz PVT and 1 Hz MSM7, and calibrate the receiver's fixed latency
  once on the rig. Beetle V2 routes 1PPS to a GPIO.
- **Budget (to be measured on the M1):** core 1 should see only the handoff, about a microsecond a
  tick. Core 0 takes the filter: the shadow task times itself and says what it costs. RAM: ~40 KB
  for the filter object (PSRAM where there is some), the 2 KB IMU ring, and 3.8 KB of GNSS driver
  queues.

## Pad calibration

The app's "Gyro & Accel Calibration" (iOS Settings) and "Sensor cal" (Android) become **Pad
calibration**:

- **The app** fetches the Klobuchar parameters from the day's broadcast navigation file (cached
  with its date, so it works offline at the field) and sends them with a new start command: the 8
  bytes GPS broadcasts, and the date. Command 21 stays for older firmware.
- **The flight computer** stores them in NVS, runs the existing gyro/accel window, and starts the
  GNSS pad survey: the pad position from an average of the receiver's own fix; per-satellite
  corrections against it (the backup ionosphere, kept fresh until liftoff); a check that every
  tracked satellite has its ephemeris. (Phase 5 gave the survey an average of its own rather than
  the flight filter's launch-site one: see below.)
- **A new status message with progress** replaces the 12 s spinner: the gyro/accel result,
  satellites with ephemeris and with pad corrections, the ionosphere source and age, and the pad
  position's uncertainty.

## Phases

1. **`TR_GNSSRaw`:** the GNSS receiver's satellite and atmosphere code, the RTCM ephemeris decoders and
   an ephemeris store, host-tested against the prototype on the Beetle's own frames.
2. **GPS time on the flight computer:** keep the receiver's time of week, backdated stamps, the
   offset estimator.
3. **The filter and its measurement builder on the host,** with a replay harness and Python bindings
   (as the flight filter has), matching the prototype on the bench flights.
4. **Integration in shadow mode,** with CPU and timing measured on the M1.
5. **Pad calibration** across both apps, the out computer and the flight computer.

Later: a rig test that replays a flight's IMU while the rig plays its signals (the firmware sim drops
raw GNSS today), then flight use.

## Phase 5: pad calibration

**Where the ionosphere comes from.** The app reads it from NOAA's CORS archive, which needs no
login:
- the file: `https://geodesy.noaa.gov/corsdata/rinex/YYYY/DDD/brdcDDD0.YYn.gz`, GPS RINEX 2;
- its header carries `ION ALPHA` / `ION BETA`;
- today's file is there from early in the UTC day and grows through it (33-70 KB).

The merged multi-GNSS files (BKG's `BRDC00WRD`, both its stream and receiver versions) leave the
ionosphere out of their headers, so they will not do. 2026-10-13's set was alpha 21, 0, -2, 0 and
beta 60, -1, -4, 3, in broadcast units.

**The command.** BLE 75 carries `[iono i8 x8][gps_day u16]`: the eight bytes as GPS broadcasts
them, and the file's date as days since 1980-01-06 (0 when the app has none).
- The out computer relays it as `PAD_CAL_PENDING` + `PadCalConfigData` (0x97 / 0x98).
- The flight computer stores the set in NVS (`"gnss"`: `iono`, `ionoday`) and reloads it at boot.
- It runs the gyro/accel window, as command 21 does; 21 stays for older apps.
- It restarts the pad average.

**Which ionosphere the filter uses.** The broadcast set, while it is no more than 30 days old by the
receiver's date. Otherwise, or without one, the pad survey's corrections. The choice is made on the
pad and kept through the flight.

**The pad average.** The receiver's fixes from the pad calibration until launch, ellipsoidal (the
receiver's altitude plus its geoid separation, `$PQTMPVT`'s `<Sep>`).

It is not the flight filter's launch-site average. That one freezes two minutes after the first fix,
wherever the rocket was then. Restarting it would move the flight filter's datum, and with it the
main-deploy backstop's count of converged fixes.

The shadow's seed altitude is ellipsoidal now too. The flight filter's is above the geoid, so in
phase 4 the shadow's first raw epoch moved its altitude by the geoid separation (-33 m in New
Jersey).

**The pad survey (`PadSurvey`).** On each raw epoch on the pad:
- each satellite's residual is its corrected pseudorange less its range from the pad (the filter's
  own Sagnac model);
- less each system's clock, a correction is that residual smoothed over 120 s;
- the clock is the median over the satellites whose corrections are in use, so a satellite coming
  or going does not move the others' corrections;
- a satellite counts only once it has been locked 50 s (a relocked pseudorange starts biased), and
  its correction is used once it has 30 s of residuals behind it;
- a residual more than 30 m from its correction is refused.

The corrections freeze at liftoff. As the backup ionosphere, each one comes out of its satellite's
pseudorange. A satellite with none keeps the ionosphere, so its correlated sigma grows by 5 m times
Klobuchar's obliquity factor; with no correction in use at all, nothing changes (as in phase 4).

**The pad, checked against the raw measurements.** On the bench (2026-10-13, indoors), the
LC86G's own fix said:
- 312 m below sea level, with 29 satellites, a 3-D fix and a claimed 1 m vertical accuracy;
- the barometer read 1013.6 hPa, so the board was near sea level.

Its raw measurements disagreed with its fix:
- they put the antenna near sea level, as the barometer did;
- fitted as a position offset with a clock per constellation, they put the pad 316 m above and 70 m
  west of the receiver's average;
- each constellation alone agreed, to 0.2-1.7 m RMS;
- the shadow filter, on raw measurements alone, went there too.

The survey's corrections against that pad would have dragged the filter onto the receiver's error
(they did, before this check), and so would the flight filter, which fuses that fix. An hour earlier,
with 9-10 satellites, fix and raw solution agreed within 15-30 m. This is #1508's failure, caught in
the act.

So each epoch the survey fits the corrections in use as a position offset plus a clock per system.
Further than 30 m from the receiver's average (the uncorrected ionosphere is metres of that), the
corrections are not used. The status says so (`PCS_PAD_MISMATCH`), and they come back when the two
agree.

**The status (`PAD_CAL_STATUS_MSG`, 0x99, 40 B).** Sent once a second on the ground and at each
change; the out computer relays it to the app as file_ops 0xD0. It carries:
- the gyro/accel window's state, progress and measured gravity;
- the satellites tracked, without an ephemeris, settling, and with a correction, and the
  corrections' spread;
- the ionosphere source, the broadcast set's date and age;
- the pad average (fixes, span, scatter and position);
- the filter's position sigma.

The gyro/accel result itself still comes as the 0xCB frame when the window closes.

## Physics checks on the GNSS (#1508)

The receiver's own fix can be wrong while it claims to be fine. #1508 caught the LC86G at it:
- frozen near the pad while the vehicle fell from 79 km;
- climbing 4 m/s through a 3 g boost;
- 312 m below sea level on the bench (above).

Checking the fix against another position estimate needs that estimate to be right. Checking it
against the IMU needs only physics. Two checks run in the shadow, both looking for motion that cannot
have happened.

**Each receiver fix (`FixCheck`, report-only).**
- The LC86G driver hands each `$PQTMPVT` to the shadow with its GPS instant on the MCU clock.
- Once the IMU has passed that instant, the IMU's own motion predicts the fix's position and velocity:
  the velocity change since the last fix that passed, and the displacement it adds, both from the
  filter's history ring (`RawGnssEKF::imuMotion`).
- A fix outside the tolerance is flagged, and the next ones are judged from the same good fix until
  one agrees. The tolerance covers:
  - the fixes' own noise;
  - their timing: a lateness common to all fixes cancels, except for the velocity and acceleration
    that changed in between, plus one fix's jitter against another's;
  - the IMU's error growing with the span: 0.5 m/s^2 plus 2 degrees' share of the specific force.
- Once the good fix is older than the history reaches (4 s), a fix cannot be judged and becomes the
  next one's reference.
- It sees a jump, a frozen fix on a moving vehicle, and a velocity the specific force does not allow.
  A fix wrong by a constant moves as the vehicle does, and passes.
- It is logged as `GNSS_CHECK_MSG` (0x9A, 24 B per fix) and shown on the console and in the pad status
  (`PCS_FIX_SUSPECT`).
- The flight filter still takes every fix. Keeping flagged fixes out of it, the GNSS main-deploy
  backstop and the recovery gate is the owner's call once flights show the check is right.

**Each satellite (`SatJumpCheck`, acting within the shadow).**
- At each raw epoch, every satellite's pseudorange change since the last epoch is set against its
  range change from the filter's IMU-propagated prior.
- The receiver clock's change is their median.
- More than 20 m off is a jump (a relock, a reflection taking over). The satellite is taken as
  relocked: the builder's relock sigma, decaying over 10 s.
- Being a time difference, the check is blind to an error that stays the same from epoch to epoch (the
  innovation gate's business) and sharp on a jump, however the filter's position has been led.

## The rig replay (hardware in the loop)

The COCOM rig plays a flight's GNSS signals to the receiver in a Faraday cage. On its own that tests
the receiver: the raw filter, the physics checks and the flight filter need the IMU, the barometer and
the magnetometer to fly the same flight, or every check fails by design (a sim's IMU against the
bench's sky is why the shadow stops for a sim). The rig replay supplies them.

- `tools/sim_replay_blob.py` writes a flight's sensors from the sensors CSV a `sim_flight_3d.py` run
  writes beside its scenario: 500 Hz pairs (the mean over each 2 ms keeps the integral), the rocket
  frame FLU, the attitude, the barometer with its static-port error, the scenario's GPS time and pad.
- A `TR_SIM_REPLAY` build (`idf.py -DTR_SIM_REPLAY_BLOB=FILE`, bench-only) carries the file (1.3 MB
  for the 107 s of rp5_0829 from t = -30 s) and plays it from boot through the sensor sim's path
  (`SimReplay.h`). It replaces each real IMU, barometer and magnetometer sample's values at that
  sample's own instant; the magnetometer reads the flight's attitude against the field the flight
  computer models (TR_GeoMag at the pad and date).
- The clock: each receiver fix carries its GPS time and that instant on the MCU clock (GnssTimeSync).
  The replay follows the fixes while on the pad and freezes 5 s before ignition; from there the
  samples run on the MCU crystal, as a real IMU's do. Before the first fix it loops the pad's first
  10 s. A first fix after that instant never starts the flight.
- The receiver's output passes through: its fixes, raw epochs, ephemerides and satellites are the
  rig's flight. The shadow runs on it, and the pad average takes its fixes. The receiver
  cold-starts at boot ($PAIR006), as on a pad power-up.
- It runs as a sim: the dry-fire gate holds and the log says sim. After landing it loops the last 5 s
  on the ground until a Stop or a reboot (one run per boot).
- The console prints the replay's time, phase and clock state and the flight filter's ENU state once a
  second (`[REPLAY]`), to set beside the flight's truth.

### What the first rig replay showed (rp5_0829, 2026-10-13)

The flight computer flew the flight on schedule: launch at +0.36 s, burnout at +2.72 s (truth 2.755 s),
apogee at about 11.9 s (truth about 11.7 s), LANDED at 68.7 s (touchdown 66.7 s). Against the flight's
truth, from the OC's log:

| Phase | Shadow (raw GNSS) | Flight filter | Receiver's own fixes |
|---|---|---|---|
| Pad | 0.4 m H, 0.8 m V | 0.1 m, 0.1 m | 0.5 m, 0.0 m |
| Boost | 6.0 m, 2.2 m, 3.2 m/s | 5.1 m, 4.2 m, 2.4 m/s | velocity 41 m/s off |
| Coast | 8.2 m, 8.2 m, 3.2 m/s | 32.5 m, 21.9 m, 7.5 m/s | velocity 71 m/s off, 23 m V |
| Drogue | 19.3 m, 14.5 m, 8.8 m/s | 6.4 m, 4.5 m | 1.2 m, 3.9 m |
| Main | 2.7 m, 3.3 m, 0.5 m/s | 17.0 m, 14.1 m, 7.3 m/s | 13.2 m, 28.5 m, 7.3 m/s |

- **The flight filter** went wherever the receiver's bad fixes took it: lagging through boost, 120-150 m
  off at reacquisition, and 28 m off vertically under the main. The physics checks flagged those runs
  (142 of 1,937 fixes; none on the pad).
- **The shadow** held under the main, where the receiver's own fixes did not.
- **Both filters' attitude turned at the drogue opening:** 31 degrees for the flight filter, 19 for the
  shadow.
  - The cause is the levelling after apogee (`use_ahrs_acc`). For 5 s the drag pulled 10-45 degrees off
    vertical while the rocket still moved 10-12 m/s sideways.
  - The flight filter's 10 Hz fixes hid the error in its position. The shadow's raw epochs come once a
    second, so it turned into 4 m/s of velocity a second.
  - So the shadow now levels in flight only once its velocity has held steady from epoch to epoch and
    the specific force is g's size (`LevelGate.h`). The flight filter's own levelling is the owner's
    call.
- **Launch latched on the accelerometer-only fallback**, with the #1102 "blocked static port" warning.
  The modeled port error (-k q) reads the first second of boost below the pad.
- **Receiver at touchdown:** it lost every satellite (the 3-D antenna pattern, lying on the ground), so
  the flight filter landed 11 m off.

## The Beetle's navigation filter

The rig showed the receiver's own fix 300-400 m off where the raw filter held 10 m. So on the Beetle the raw filter is
now the navigation filter, and GpsInsEKF does not run (`TR_RAW_GNSS_PRIMARY`, on wherever the raw filter runs). The
V9 keeps GpsInsEKF on the SAM-M10Q's fix.

**What the raw filter gained to stand alone**
- **The barometer:** the samples the flight filter's gate passes, as a height.
  - The offset to the filter's own (ellipsoidal) height is learned on the pad and frozen at launch, however short
    the pad (a sim's is 5 s).
  - The variance is #1579's q-scaled one.
  - It moves only the vertical position and velocity and the receiver clock. Through the propagation's
    cross-covariance it otherwise tilted the attitude, or pulled the body's accelerometer bias off the levelling's
    tie, and with no satellites gravity on a wrong tilt is kilometres of drift.
  - In r2's outage under the main, the shadow without a barometer drifted 70 m in 20 s.
- **Zero velocity once landed** (#1418).
- **The magnetometer as a heading on the ground** (#1578's measurement).
- **Levelling:**
  - On the ground only at g's size. Between ignition and the launch verdict the vehicle is still "on the pad" with
    its motor burning.
  - In flight once the raw epochs say steady (#1621), or with no epoch for 2 s once the force has been g's size for
    5 s. Under a chute without satellites a gyro bias would otherwise tilt it unchecked.
- **Its own start:** it keeps its own attitude on the pad and is never resynced; a new start generation starts it over.
  It can start in flight from a restored snapshot.
- **Holes and sims:** it steps over an IMU hole up to 5 s, and it flies a sensor sim (the sim drops the receiver's raw
  output).

**The firmware sim on the Beetle, four times** (no satellites in a sim, so no horizontal aiding):
- Launch, burnout, apogee (the velocity and GNSS votes, within 0.25 s of the sim's) and landing ran on the raw filter.
- The vertical held through the flight once the barometer was fused in flight.
- The horizontal drift went 430 m, 1.4 km, 1.6 km, then 495 m, as levelling on the motor, the barometer's coupling and
  the unlevelled descent were fixed in turn.
- The last left a constant 13 m/s east from a gyro bias the sim carries: its synthetic counts less the board's stored
  gyro calibration. The same flight on the host (`TheNavigationFilterFliesTheFirmwaresSim`) is 0.6 m horizontal, and
  39 m with a 0.4 dps gyro bias.

**How main.cpp reads it**
- `ekf` on the Beetle is `RawNavAdapter` (`raw_nav_adapter.h`), GpsInsEKF's methods answered from the raw filter's
  state.
- The raw filter publishes that state every step (`ShadowLink::nav`).
- The apogee and landing voters, the ascent gate, the transonic lockout, NonSensorData, the sensor health and the
  snapshot all read it unchanged.
- The calls that configured GpsInsEKF now carry the raw filter's start in the seed: position, the accelerometer's
  tilt, the gyro's mean, the declination and the field's size.
- Heights turn from the ellipsoid to the receiver's geoid with its separation. The snapshot keeps the raw filter's
  own (ellipsoidal) height, since a receiver cold-started by a reboot has no separation to give for a while.

**Rates**
- The raw filter steps at 100 Hz on core 0 (it was 25 as a shadow). Its raw epochs come once a second.
- The flight loop on core 1 no longer runs a filter (GpsInsEKF cost about 1 ms a tick at 290 ticks a second). It reads
  the state, at most a step and a task period old (about 15 ms).

**Logging:** the receiver's 10 Hz fix is still logged (GNSS_MSG), as are the raw filter's 10 Hz record (0x96) and the
checks (0x9A).

## Phase 3: the port against the prototype

`TR_RawGnssEKF` (the filter; float on the flight computer) and `TR_GNSSRaw`'s `RawMeasBuilder`, fed
by the LC86G driver's own parser through `Lc86RawEpoch.h`, were flown through the bench scorer on
the four bench flights. The scorer is the gnss repo's `sim/tc_ekf_cocom.py`, in modes `ins-cpp` and
`ins-cpp64`, via `tinkerrocket_sim._rawgnss`. The prototype ran with the states the port leaves out
turned off, which changes nothing on these flights:

| Compared | Largest position / velocity difference |
|---|---|
| C++ in double, the prototype's measurements | 1.7e-7 m / 7e-8 m/s; the same updates accepted and refused |
| C++ in float, against C++ in double | 4.5 mm / 4.5 mm/s |
| The whole C++ chain, ephemeris preloaded as the prototype does | 1.0 mm / 0.1 mm/s |
| The whole C++ chain, ephemeris in the order it arrived | 0.3-0.8 m |

The last row is the flight computer's view. Each satellite waits 16-35 s after it appears for the
receiver to send its ephemeris, where the prototype had every ephemeris from the start. Per phase,
the altitude and velocity RMS still agree with the prototype's to 0.1 m and 0.01 m/s.

**Applied late.** On the flight computer, epochs arrive 0.1-0.25 s after their time. Applied that
late, from the history of the IMU's velocity increments, they changed the flight's (ignition to
landing) RMS as follows:

- altitude: by at most 0.3 m;
- vertical velocity: by at most 0.01 m/s;
- horizontal position: by up to 0.6 m at 0.25 s.

A first-order model of the attitude and accelerometer-bias errors over the delay helped two flights
and hurt one, and was left out. Taking each epoch as it arrives, with its covariance restored and the
IMU replayed, would be exact; whether that fits the CPU budget is phase 4's question.

**Memory on the ESP32-S3:**

- the filter: 9.4 KB;
- the ephemeris store: 12.8 KB;
- the builder: 3.4 KB;
- a 48-satellite measurement package: 5.8 KB;
- about 18 KB of code.

That's above the 25 KB estimate, with room to trim (e.g. a smaller history ring and float satellite
velocities). CPU is measured in phase 4.

## Phase 4: shadow integration

**On the flight computer:**
- **`TR_RawGnssShadow`** runs `TR_RawGnssEKF`. `RawGnssShadow` is pure and host-tested; `ShadowLink`
  is the core-1-to-core-0 handoff.
- **The task:** `main.cpp`, section "Raw-GNSS shadow filter", core 0, priority 1. It is on wherever
  the LC86G driver is; `-DTR_RAW_GNSS_SHADOW=0` builds without it.
- **The LC86G driver** hands over each MSM7 epoch with its GPS instant on the MCU clock (its
  `GnssTimeSync`), and each GPS, Galileo and BeiDou ephemeris, through queues. While a sensor sim
  runs they are consumed and dropped, as the raw RTCM log is.

**Behaviour:**
- **Start and pad:** it starts on the pad from the flight filter's state. On the pad it keeps
  taking the flight filter's attitude and gyro bias once a second, because the heading there is the
  magnetometer's and this filter has none. It takes the position and velocity too, until its first
  raw epoch.
- **Raw epochs:** each is held until the IMU passes its time, then applied with its age; one later
  than 0.6 s is dropped.
- **Stops:** a sim, an IMU gap over 0.5 s, a non-finite state, or (on the pad) 500 m from the flight
  filter stops it. It restarts from the next pad seed.
- **Log:** `RAW_GNSS_EKF_MSG` (0x96), 90 B at 10 Hz. It carries:
  - position, velocity, attitude, clock and system offsets, and their sigmas;
  - what the last epoch did, and how late;
  - the ephemerides held;
  - the CPU each epoch and IMU step took.
- **Console:** a line every 5 s with the same, plus the task's share of core 0 and its distance
  from the flight filter.

**Measured on the Beetle's FC** (2026-10-13; IMU at 7,680 Hz; indoors, 9-15 satellites; FreeRTOS
run-time stats for each task's share of its core):

| Build | Flight loop | EKF tick | Core 0 idle | The shadow |
|---|---|---|---|---|
| IDF's DIO flash, 16 KB cache, shadow off | 313/s | 1,854 us | 19 % | -- |
| the same, shadow on (50 Hz steps) | 273/s | 2,216 us | 0 % | 18 %, fell behind |
| QIO, 32 KB cache (#1607), -O2, shadow off | 806/s | 785 us | 55 % | -- |
| the same, shadow on (25 Hz steps) | 775/s | 848 us | 47 % | 7.5 % |

**What the first two rows showed.** At IDF's defaults the shadow could not keep up, and it took 12 % off
the flight loop through the cache and flash bus the two cores share. With #1607's QIO flash and
32 KB cache, the shadow costs core 0 7.5 % and the flight loop 4 %, with no IMU or I2S drops.

**Cost per epoch** (QIO build, 9 satellites):
- build: ~20 ms of wall time;
- update: ~16 ms;
- a 25 Hz step: ~1.2 ms.

**What brought it there:**
- each satellite's orbit solved exactly every 5 s and carried between by its velocity and
  acceleration;
- the update's geometry computed once per satellite;
- steps at 25 Hz on the mean of the flight filter's ticks;
- the 40 KB object in PSRAM, which cost the same as internal RAM (775 vs 777 passes/s) and leaves
  94 KB of internal RAM rather than 53.

**Still open:** two things are not calibrated:
- the ionosphere (phase 5);
- the receiver's latency, which places every epoch late by the same amount until Beetle V2's 1PPS
  or a scope measures it.

Both are left in the raw log, so a flight can be replayed with them later.
