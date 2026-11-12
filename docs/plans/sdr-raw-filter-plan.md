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
