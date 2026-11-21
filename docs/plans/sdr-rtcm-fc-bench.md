# gnss-sdr-dev board on a V9/V10's J1: the flight computer logs its RTCM3

**Written 2026-11-02, branch `fc/sdr-rtcm`.** TinkerRocket's own receiver
board (gnss repo, `firmware/gnss-sdr-dev`) plugs into J1 of a Tinker-Mantis
V9 or V10 with its 5-pin J401. It sends the flight computer RTCM 3 at
2 Mbaud (J401 pin 2 -> J1 pin 2 "module TXD" -> FC GPIO3) and ignores what
it is sent. The FC now finds the board at boot and logs every frame beside
the IMU, so a flight test records the board's 100 Hz measurements. The FC's
own navigation filter does not use them yet: with the board on J1 the FC
flies GNSS-absent (#557), as with no receiver.

## What the board sends

About 400 frames and 40-55 kB a second (`TR_GNSSReceiverSdr_Serial/SdrRtcm.h`):

- MSM7 1077 (GPS), 1097 (Galileo E1-C) and 1127 (BeiDou B1C pilot, signal ID
  31), one set an epoch, 100 epochs a second in the board's measurement mode;
- TinkerRocket's message 4001 (not registered with RTCM): sub-type 1 after
  each epoch (32-78 B: since 2026-11-02 it ends with the ranges the board's
  checks left out and the satellites it marked, which the FC does not read),
  sub-type 2 after each fix;
- ephemerides 1019, 1046 and 1042, each new one and again every 30 s.

Nothing before its first fix: after a cold power-up the board is silent for
roughly 40 s (with its stored ephemerides) or longer.

## What the image does

`-DTR_BOARD_V10=1` now builds the auto image, as `-DTR_BOARD_V9=1` already
did (`-DTR_GNSS_AUTO=0` gives the old u-blox-only image).

1. **Detection** (`TR_Sensor_Collector/gnss_detect.h`). A probe at 2 Mbaud
   that sends nothing and listens 250 ms; one CRC-24Q-valid frame carrying
   message 4001 proves the board. Any probe listening at 2 Mbaud counts it,
   the mosaic's too. It is in every round of probes, before the mosaic's
   2 Mbaud CR and before the u-blox driver's own search (whose swapped-wires
   attempt drives J1 pin 2, the board's output, through the board's 1 kOhm).
   It is in the second look too.
2. **The hint.** The board found is remembered (NVS `gnssauto`/`rx` = 4). With
   the hint the board's probe goes first, and a board that is silent (every
   cold power-up) gets the board's driver anyway after ~2 s of probes: it only
   listens, and logs the stream from whenever it starts. Only another
   receiver's proof replaces the hint.
3. **The driver** (`TR_GNSSReceiverSdr_Serial`). UART at 2 Mbaud with its
   interrupt in IRAM (`sdkconfig.defaults.mosaic`), a 16 kB driver ring read
   every 25 ms by the GNSS poll task, frames cut by `Rtcm3Framer` (a lost
   byte costs only its frame), each into a 32 kB log ring with the MCU time
   of its last byte. The flight loop sends them as GNSS_RTCM_MSG (0x95), split
   into parts like any frame. Console line every 30 s: frames/s, epochs/s,
   epoch gaps, fixes, CRC fails, UART overflows, ring drops.
4. **The rest.** `gnssTypeGivesFix()` is false for it: GNSS-absent init,
   guidance off, the degraded GNSS boot bit. The IMU stops at 3840 Hz
   whatever the GNSS log setting (`imuRateMaxForGnss`), like the mosaic's
   detailed log. The FC names it "gnss-sdr-dev" in its identity push; an OC
   built before this reads the name as no receiver (its `irmax` then says
   the board's 7680 while the FC flies 3840). A crash reset leaves the board
   out of the next boot, as for the mosaic.

## Budget

`tests_cpp/test_i2s_link_budget.cpp`, the stream budgeted at 32 observables
(~71 kB/s with its record and I2S framing, the 4001 at its longest): the link at 3840 Hz well under
75 %; the OC's flash under 95 % of its ~310 kB/s drain. At 7680 Hz the log
would outrun the drain, hence the cap. The board's own recording of
2026-11-02 had 18-25 observables: ~38-46 kB/s of frames.

## Bench, 2026-11-02 (V10 on the rig Pi, FC 80:F1:B2:D0:94:AE)

FC image `62d8fb04-dirty-v10+20261102-2101` in ota_1 (the flight image
600569bb-v10 stays in ota_0). Board streaming, 100 Hz measurement mode.

- First boot of the image: found and remembered the board (the hint read back
  on the next boot: "Last boot: gnss-sdr-dev"); the image validated itself
  (#1123: flight loop + OC link).
- Second boot: `gnss-sdr-dev on J1 (an RTCM 4001 frame at 2000000 baud,
  10 ms, 1 probe)`; GNSS-absent degraded init; IMU 3854 Hz; loop gaps
  > 10 ms: 0.
- Driver, every 30 s for 4+ min: 401.6-401.8 frames/s, 100.0 epochs/s, epoch
  gaps 0, CRC fails 0, UART overflows 0, log ring drops 0, 20-25
  observables; 117 bytes in no frame (the partial frame at the UART's open).
- FC -> OC I2S: ~2,200 frames/s enqueued, 0 dropped, 0 TX failures.
- Board silent at boot (its recorder being read): 2.7 s of probes, then the
  listening driver; when its UART came on 7 min later the frames came in at
  once, 0 CRC fails, 0 bytes in no frame.
- OC log (BLE cmd 23 on/off, 52 s, downloaded over BLE, `flight_20261102_213043.bin`,
  10.8 MB): the OC took 280,437 frames, dropped 0. In it: 21,076 RTCM frames,
  0 bad CRC, 0 incomplete; 5,245 MSM7 epochs at a 10 ms step, 0 missing; 53
  board fixes, all valid; ephemerides for GPS, Galileo and BeiDou; 44 kB and
  402 frames a second; IMU 3,854 Hz, baro 491 Hz in the same log. convbin
  read all 5,245 epochs (21 satellites); RTKLIB single-point fixes from the
  log alone (10 Hz, no ionosphere model) averaged 0.8 m east and 2.7 m up of
  the board's own fixes. On the still bench the V10's own filter (GNSS-absent)
  drifted 13.6 m horizontally in the 52 s; the board's fix stayed within 1.1 m.

## Reading a log

```bash
python3 Data_Analysis/extract_gnss_rtcm.py flight.bin     # -> flight.rtcm3, and the epochs' summary
convbin -r rtcm3 -tr 2026/11/02 19:30:00 -v 3.04 -od -os -o flight.obs -n flight.nav flight.rtcm3
```

The summary gives the MSM7 epochs (count, step, missing) and the 4001
epochs and fixes. RTKLIB (v2.5.1) passes 4001 over and reads the rest.
convbin writes every 100 Hz epoch, but rnx2rtkp groups observations within
25 ms into one epoch (DTTOL), which merges 100 Hz epochs and spoils its
fixes: give it every 10th epoch (keep the MSM7 frames whose time of week is
a multiple of 100 ms from the first, and every other frame).
`tests/unit/test_sdr_rtcm_record.py` runs that path on the board's recorded
frames (set `TR_CONVBIN` to the binary where it is not in
`bench-backups/rtklib-builds/v2.5.1/`).
