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

**The status (`PAD_CAL_STATUS_MSG`, 0x99, 40 B).** Sent once a second on the ground and at each
change; the out computer relays it to the app as file_ops 0xD0. It carries:
- the gyro/accel window's state, progress and measured gravity;
- the satellites tracked, without an ephemeris, settling, and with a correction, and the
  corrections' spread;
- the ionosphere source, the broadcast set's date and age;
- the pad average (fixes, span, scatter and position);
- the filter's position sigma.

The gyro/accel result itself still comes as the 0xCB frame when the window closes.

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
