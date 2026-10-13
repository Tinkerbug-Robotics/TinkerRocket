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

- **Core 0, a low-priority GNSS task, once per MSM7 epoch.** Ephemeris decode into a store,
  satellite positions and clocks (double precision), troposphere and ionosphere, per-satellite noise,
  then one measurement package to core 1.
- **Core 1, inside the flight loop.** The 19-state filter in float (the flight filter's error-state
  form), propagated by the same IMU input, its covariance every few ticks. Each epoch is applied at
  its own time from a ~0.5 s history of the filter's state, and its updates are spread over a few
  ticks so no tick grows.
- **Time without PPS.** The M1 wires the LC86G's 1PPS only to an LED: keep the receiver's GPS time,
  backdate each UART read by the bytes still buffered, track the GPS-to-`esp_timer` offset with a
  minimum-latency filter over the 10 Hz PVT and 1 Hz MSM7, and calibrate the receiver's fixed latency
  once on the rig. Beetle V2 routes 1PPS to a GPIO.
- **Budget (estimates, to be measured on the M1):** about 1-2 % of core 1 and 2-5 % of core 0;
  ~25 KB of RAM. Separately, every firmware build is at ESP-IDF's default `-Og`.

## Pad calibration

The app's "Gyro & Accel Calibration" (iOS Settings) and "Sensor cal" (Android) become **Pad
calibration**:

- **The app** fetches the Klobuchar parameters from the day's broadcast navigation file (cached
  with its date, so it works offline at the field) and sends them with a new start command: the 8
  bytes GPS broadcasts, and the date. Command 21 stays for older firmware.
- **The flight computer** stores them in NVS, runs the existing gyro/accel window, and starts the
  GNSS pad survey: the pad position from the existing launch-site average of the receiver's own fix;
  per-satellite corrections against it (the backup ionosphere, kept fresh until liftoff); a check
  that every tracked satellite has its ephemeris.
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
