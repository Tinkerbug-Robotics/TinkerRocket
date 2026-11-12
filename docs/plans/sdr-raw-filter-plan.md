# The gnss-sdr-dev board's measurements in the V10's navigation filter

**Written 2026-11-02, branch `fc/sdr-raw-offline` (on `fc/sdr-rtcm`, PR #1688).** The offline half is done: the
board's logged RTCM3 runs through TinkerRocket's raw-GNSS filter in `tools/raw_replay` (`--receiver sdr`). This is
what that showed, what it could not, and the steps to fly it.

## What the offline run shows

On the V10's bench log of 2026-11-02 (52 s, the board on its bench antenna, still):

| | |
|---|---|
| Epochs | 5,245 at 100 Hz, every one made whole by its 4001; 525 handed over at 10 Hz, 107 untimed while the time sync filled (10 s), 418 applied |
| Measurements | 672 pseudoranges and 6,688 Dopplers fused, none refused; largest NIS 5.4 |
| The board's clock steps | 8 in the window (~330 m each, every ~7 s). Put back from the 4001's step sum: none refused. Left in (`--sdr-raw-steps`): 348 of the 672 pseudoranges refused, NIS to 41 |
| Time sync from arrivals | `GnssTimeSync` on every 4001: 17.4 ppm, envelope 0.54 ms rms; the FC's 25 ms UART reads leave 90 % of stamps within 21 ms above it |
| Doppler noise at 10 Hz | 0.015-0.042 m/s by C/N0 (GPS 0.034, Galileo 0.039, BeiDou B1C 0.010): half to a quarter of the mosaic's v0 model; `sdrDevProfile()` now has it |
| Position | 1.5-2 m from the mean of the board's own fixes: the stream carries no ionosphere, so the filter flew with none |

The plot: `bench-backups/v10-sdr-logs/` (`sdr_raw_filter_bench.png`).

## What it cannot show

- **Dynamics.** A still bench scores neither the Doppler weights (the clock drift's walk, the mosaic's 100
  m^2/s^3 kept for flight, dominates every innovation: NIS median 0.16) nor boost, coast or the attitude.
- **The board's output delay.** The time sync places each epoch late by the board's delay from its epoch to its
  4001's last byte (encode ~1 ms, the UART ~2 ms for the group). `--sdr-latency-ms` takes it off once measured.
- **Minutes.** 52 s, 42 s of it with GNSS. GNSS-absent drifted only 0.7 m over those 40 s here, because this filter
  learnt its accelerometer biases on the pad first; the V10's own filter (GpsInsEKF, GNSS-absent, no such phase)
  drifted 13.6 m in the same log.

## Next, in order

1. **An RF-rig replay with truth.** The board on the RF replay rig's flight scenarios (the HackRF host, not the
   open-sky Pi it sits on now; both belong to another session), the V10 logging, then `raw_replay --receiver sdr`
   against the scenario's truth: boost, coast, attitude, unaided twin. The bar: not meaningfully worse than unaided
   with GNSS lost in boost.
2. **The ionosphere.** RTCM has no message for it. The board decodes GPS's Klobuchar set: a 4001 sub-type for it
   (gnss repo, `rtcm_msm.c`), filed through the GNSS task's ionosphere rule as the mosaic's GPSIon is
   (`raw_iono_rule.h`). The app's upload path is the fallback.
3. **The output delay.** Measure it once: the V2 board's 1PPS on J401 pin 5 does it for good; on the V1, a GPIO
   pulse at the latch against the 4001's arrival on a scope, or the delay that best lines up the Dopplers with the
   IMU on the spin rig.
4. **The flight computer.** As the mosaic-only image did it, then folded into the auto image:
   - an image whose one navigation filter is the raw filter on the board (`RawNavAdapter` in place of GpsInsEKF, no
     second filter), with PSRAM (`sdkconfig.defaults.rawnav`) and 128 ephemerides; the auto image picks its filter
     at boot later (today it refuses `TR_RAW_GNSS_SHADOW`);
   - the driver's raw outputs (`TR_GNSSReceiverSdr_Serial::enableRawOutputs`): `SdrEpochAssembler` on each frame
     the framer already cuts (its two epoch buffers and one MSM7's cells as members, ~7 kB, not on the poll task's
     stack), a `GnssTimeSync` on every 4001, timed epochs into a 3-deep queue and each ephemeris into the stage's,
     as `MosaicRawEpoch.h` feeds them; the board's fix (4001 sub-type 2) as the seed's position and the GPS apogee
     voter's fix;
   - the profile's 10 Hz with 16 satellites first, then the CPU ladder on the P4 as the mosaic's (`v9-more-data`):
     each fused epoch could also average the nine Dopplers between it and the last, as the mosaic's pairs do;
   - the bench gates the mosaic image passed before its flight: pad, CPU, the OC log at 3840 Hz, a rig replay.
5. **Board side, alongside:** IMU aiding on the board (the bring-up session's plan) keeps the measurements coming
   through boost, which is where the flight computer gains most from them.

## Built: the V10's gnss-sdr-dev image (2026-11-12, branch `fc/sdr-raw-v10`)

Item 4's first half, for the 2026-11-14 flight (the rocket has backup charges). Not flashed, not on hardware yet.

- **Build:** `idf.py -B build_v10_sdr -DTR_BOARD_V10=1 -DTR_GNSS_SDR=1 build`; the rig replay adds
  `-DTR_SIM_REPLAY_BLOB=<blob>` (its clock from the filter's timed epochs, as the mosaic's). The board's driver alone
  (`TR_GNSS_DRIVER_SDR`), PSRAM, 128 ephemerides; one navigation filter (RawNavAdapter, no GpsInsEKF). The auto image
  is unchanged.
- **What it flies:** `sdrDevProfile()` + `applySdrFlightProfile()` (the mosaic's 10/24 set: 10 Hz, 16 satellites,
  the cheaper build, shared geometry) with each fused epoch's Dopplers the means of the board's ten 100 Hz epochs since
  the last (`SdrRawEpoch.h`), taken at the window's middle (45 ms); the mosaic's flight safeguards with no wind-up
  taken off; no ionosphere; the board's ephemerides into the stage as the Beetle's RTCM ones; the board's fix as
  GNSSData (the start's position, the GPS voter; heights above the ellipsoid); the attitude, rests and phase still
  sent to the board.
- **Offline, the RF rig's runs of 2026-11-09 through the image's lines** (`raw_replay --receiver sdr`, medians,
  aided / GNSS cut at T+0): hotshot coast velocity 0.07-0.10 vs 3.4-3.8 m/s, descent 0.03-0.04 vs 6.5 m/s; traveler
  coast 0.06 vs 11.4 m/s. Against the same lines without the means (`--rr-pair-avg 0`, = the offline result): coast
  velocity 0.08-0.12 -> 0.06-0.10, descent 0.05-0.06 -> 0.03-0.04, boost within +-0.04 m/s; the V10 bench log's static
  velocity 0.021 -> 0.012 m/s rms. GNSS lost over the boost (0-4 s) or to T+20 s: coast velocity 0.05-0.09 m/s after
  it returns.
- **Next:** the bench gates the mosaic image passed (pad, CPU on the P4 -- the driver prints its cost per MSM7 and per
  record every 30 s --, the OC log at 3840 Hz, a rig replay), then the ionosphere and the output delay (items 2-3).

