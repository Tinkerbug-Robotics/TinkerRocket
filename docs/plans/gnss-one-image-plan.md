# One V9/V10 image for every J1 receiver

Owner's goal (2026-11-13): one flight-computer image that finds the receiver on J1 at boot and
flies the raw-GNSS filter on every receiver that gives measurements -- the mosaic-G5, the SkyTraq
PX1105R carrier and TinkerRocket's gnss-sdr-dev board -- and GpsInsEKF on the u-blox SAM-M10Q,
which gives none. Today each raw receiver flies the raw filter only in an image of its own
(`-DTR_GNSS_MOSAIC=1`, `-DTR_GNSS_PX1105R=1`, `-DTR_GNSS_SDR=1`); the auto image
(`-DTR_BOARD_V9/V10=1`) finds all but the PX1105R and flies GpsInsEKF on the receiver's own fix
(with the board, on none).

Not for the ~11-21 flight: that flies the 5463028c gnss-sdr-dev build. Branch `fc/gnss-one-image`.

## The build

`-DTR_GNSS_AUTO_RAW=1` (V9 or V10): `TR_GNSS_DRIVER_AUTO=1` and `TR_GNSS_AUTO_RAW=1`, the mosaic
overlay (UART ISR in IRAM) and the rawnav overlay (PSRAM), `TR_EPH_CAPACITY=128`. Every other
image keeps its own flags and code: the auto image's detection plans are unchanged (gnss_detect's
`px1105r` flag false), and each dedicated image's compiled functions are checked against a
baseline built from the commit this branch starts from.

## Which filter

The receiver is chosen once, at boot, before the filter starts, and holds for the boot:

- found by the probes: that receiver;
- nothing found: the NVS hint, if it names a raw receiver (the board silent until its first fix,
  a PX1105R or mosaic still booting): its driver listens and the raw filter starts degraded, as
  each dedicated image does; a u-blox or no hint: the auto image's path (GpsInsEKF, GNSS-absent);
- a restored flight: no probes, straight to the hinted driver (setBeginWait off), as the mosaic
  and PX1105R images do today;
- a PX1105R hint and nothing answering in 6 s of rounds: the gnss-sdr-dev board's driver, not the
  PX1105R's. A PX1105R answers within seconds (board 1 booting with the FC: 1.3 s), so one this
  silent is not on J1; the board, silent until its receiver starts and streaming from its first
  fix, is the one receiver that can be -- a stale hint with the board put on since would otherwise
  leave it unheard for the boot (the rig owner's review, 2026-11-13). The hint stays until a
  receiver is proven.

The ionosphere the last pad calibration stored is read in setup before the choice; it goes to the
raw filter right after the choice (the rig owner's review: it was dropped, RAW_ON still false).

Raw receiver -> the raw filter is the one navigation filter (RawNavAdapter), with that receiver's
profile, flight profile and safeguards, in the dedicated image's order. u-blox -> GpsInsEKF, the
auto image's path exactly.

## The layers

1. gnss_detect: the PX1105R probe (done).
2. TR_GNSSAuto holds the PX1105R driver and passes on every raw output of the active driver;
   SensorCollector and SensorCollectorSim offer the union of the three raw surfaces under
   `TR_GNSS_AUTO_RAW` (an `#if TR_GNSS_AUTO_RAW` block ahead of each `#if MOSAIC/#elif ...` chain,
   so the dedicated images' text is untouched).
3. config.h / CMakeLists.txt: the option, its overlays, and `TR_RAW_NAV_AUTO`: every
   `TR_RAW_NAV_*` compiled in, chosen at run time.
4. main.cpp: `ekf` holds GpsInsEKF and RawNavAdapter and hands each call to the one chosen;
   each `TR_RAW_NAV_*` / `TR_RAW_GNSS_PRIMARY` block gets a run-time gate in the one image
   (declarations compile everywhere; statements run only for their receiver; exclusive
   `#if/#elif/#else` chains become `if/else` there).
5. Checks (below), then stage the image. No flashing.

Layers 1-4 are in (2026-11-13). The gates are `RAW_NAV_WHEN(cond)` / `RAW_NAV_AND(cond)` in
raw_nav_auto.h: `if (cond)` / `(cond) &&` in the one image, nothing in every other; the boot's
choice is `rawNavChoice` (host-tested, test_raw_nav_choice).

## Must keep, from the gnss-sdr-dev image (its owner's list, 2026-11-13)

- The board's fix as GNSSData (gnss_log_only false; gnssTypeGivesFix follows the filter).
- The ionosphere rule (app's NVS set vs 4001 sub-type 4).
- The whole degraded start: raw_sdr_gnss_held, degradedStartDue, restartOnFix,
  sdr_degraded_armed, the boot's wait for a 4001 (kHeardMs) -> FCB_DEG_GNSS, never on a restore.
- The filter setup's order: applySdrFlightProfile, the rig-replay latency, bench overrides,
  setGNSSRawEpochConfig, bench carrier, applyReceiverProfile, applySdrDopplerMeans,
  applySdrFlightSafeguards LAST.
- The 4002 senders (phase + ignition cue; attitude/rests with no attitude while the filter is
  not running -- that gate follows the boot's choice; board frame; log copies; clock line).
- The crash rules (a crash restart leaves the receiver out; the crash-loop rule).
- The overlays, 128 ephemerides, the IMU cap of 3840.

## Checks

- Host tests (tests_cpp), test_gnss_detect, test_sdr_nav_start_policy, test_raw_nav_policy.
- Every dedicated image (gnss-sdr-dev, mosaic, PX1105R, auto, u-blox-only, M1) builds with the
  same compiled function sizes as the baseline.
- tools/raw_replay on the eleven #1703 rig recordings (hl1-hl4, tr1, hil2b-hil4, hot1-hot3):
  byte-identical to main.
- The one image builds for the V10 and V9, and its boot path is walked per receiver.

## Results (2026-11-13, d84b0e68)

- Host tests 3538/3538; test_gnss_detect 67/67; test_raw_nav_choice 5/5.
- raw_replay --receiver sdr --fw-defaults on the eleven recordings, healthy and with GNSS lost from
  T+1: 22/22 byte-identical to main 5463028c.
- The other images against the 4a9d1e5a baseline: u-blox-only, V9 mosaic, PX1105R identical;
  gnss-sdr-dev and M1 differ in startRawShadow and rawGnssMeasTask only, with the same calls in the
  same order and the same relocation targets; the auto image differs in detection.
- Builds: the one image for the V10 (d84b0e68-v10, 1.19 MB, 62 % of the app partition free) and V9.

## Left for the rig (its owner, not this branch's author)

- The PX1105R query and the other probes' bytes against the board's 4002 reader (`rx fc`).
- J1 pin 2 under the u-blox driver's swapped-wire search with the board on J1.
- A boot of the one image with each receiver; a first boot with no hint and the board's output
  off (~67 s to the u-blox driver's end).
- The rig replay (TR_SIM_REPLAY_BLOB) is not built for the one image yet.
