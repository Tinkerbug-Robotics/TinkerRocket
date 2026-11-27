# V9 with the PX1105R carrier: bench plan

**Written 2026-10-21, branch `fc/v9-px1105r-raw`.** A V9 flight-computer image that flies the SkyTraq PX1105R
carrier (`hardware/gnss-px1105r-18mm-hv`, its J3 on the V9's J1) in place of the Septentrio mosaic-G5. The
navigation filter is the mosaic image's raw-GNSS filter, unchanged in design. It is the one navigation filter: no
GpsInsEKF beside it, with the same flight policy, the same 20 Hz epochs fused on the 10 Hz grid, the same Doppler
pairs, satellite cap and CPU choices, and the same safeguards. The PX1105R's own numbers are in
`gnss_raw::px1105rProfile()`.

Nothing here has run on hardware yet. Host tests on the receiver's own captures and the image build are all it has.

## What the image does

- **Bring-up** (`SkyTraqLink.h`). It probes 921600, then 115200 (the factory baud, NMEA), then the other bauds. It
  moves the receiver to 921600 and sets it up, all in its SRAM (a power cycle undoes it), with each command ACKed:
  - RTK base, kinematic function: the PX1105R sends raw measurements only as a base;
  - 0xE5 raw measurements, 0xDF state, 0xE7 channel status and the GPS/Galileo/BeiDou subframes, at 20 Hz;
  - airborne navigation mode (auto froze after apogee on the RF rig);
  - power normal (it ships in power save);
  - a 5° elevation mask (it ships at 15°).

  It then reads back what the receiver can report, and runs only once 0xE5 comes every 50 ms. A receiver that goes
  silent is found and set up again; one whose 0xE5 stops is set up again.
- **No 1PPS.** The carrier's J3 pin 5 is the receiver's RXD2 (an input), so the image never touches GPIO2. Epochs
  are placed on the MCU clock by when each 0xE5 arrives, as the Beetle's LC86G is (`GnssTimeSync`): by its
  first byte (#1771), since the frame's own wire time (7-15 ms for 20-45 signals at 921600 baud) moves with the
  signal count. The receiver's own delay from an epoch to the start of its 0xE5 is **not calibrated** (0). Every epoch is placed late by it, which in boost is
  acceleration × delay of velocity error (30 ms at 10 g ≈ 3 m/s). To measure it, put a scope on the PPS LED
  (D1/R1) and the receiver's TXD (J3 pin 2).
- **Measurements** (`Px1105rRawEpoch.h`):
  - one L1-band signal per system: GPS L1 C/A, Galileo E1, BeiDou B1I (MEO/IGSO);
  - the code is fused once a second and the Doppler every 100 ms, the mosaic's design;
  - the Doppler is whole Hz truncated toward zero, so the image moves it to the middle of its step and lags it
    25 ms (both measured on the RF rig's flights);
  - the carrier is off, as on the mosaic.
- **Ephemerides** are decoded on the FC from the subframes (`SkyTraqNav.h`), bit-identical to the rig's Python
  decoder on the 09-26 capture. Each satellite's first one comes about 25-40 s after it is tracked. The GPS
  ionosphere (subframe 4 page 18) comes every 12.5 min, or sooner from the app's pad calibration.
- **Log.** Every frame is logged verbatim as GNSS_RTCM_MSG (0x95), A0 A1 … 0D 0A, about 30 kB/s. The IMU stops at
  3840 Hz with this receiver. The offline tools (`extract.py`, `plot_flight_data_mini.py`, `raw_replay`) do not
  read SkyTraq frames yet.
- **GNSSData** (the app's fix) comes from 0xDF. **Its altitude is above the ellipsoid**: the receiver's binary
  output has no geoid, so the app's altitude differs from MSL by the geoid separation (tens of metres across North America). Its accuracies are 3 m × the DOPs.

## Before you start

- **Which V9.** Not the Goblin's "RC V9" (FC 80:F1:B2:D0:94:A7) unless it gets the mosaic flight image back before
  10/24.
- **Image.** `idf.py -B build_v9_px1105r -DTR_BOARD_V9=1 -DTR_GNSS_PX1105R=1 build`, from a fresh build dir, with
  the real `TR_GuidancePN` checked out:
  `git -c submodule.tinkerrocket-idf/components/TR_GuidancePN.update=checkout submodule update --init tinkerrocket-idf/components/TR_GuidancePN`.
- **OC.** No change is needed to fly. An OC names this FC's receiver "none": the PX1105R's name is compiled
  into the PX1105R FC image alone, so every other image (the OC's too) stays code-identical to main. Teaching the
  OC the name is a one-line follow-up.
- **Carrier.** It is powered from the V9's GNSS rail (GPS_ACT), about 125 mA through its LDO.

## Bench steps

1. **Flash the FC over USB.** Check the USB serial (= MAC) before any esptool call.
2. **First boot, indoors or out.** On the FC console, within about 10 s of power-up:
   - `PX1105R set up (#1): 921600 baud (found at 115200, NMEA), RTK kinematic base, airborne, 20 Hz 0xE5/0xDF,
     receiver <kernel> <ODM> <revision>; 0 refused, 0 readbacks off`.
   - `found at 921600, binary` after an FC-only reset (the receiver kept its setup).
   - **DEGRADED** or any refused command means the receiver did not take part of the setup; note the last
     failure it names.
3. **Every 30 s** the `PX1105R:` line should show ~20 0xE5/s and ~20 0xDF/s, raw period 50 ms, no checksum fails,
   no UART overflows, no log-ring drops once the flight loop runs.
4. **Outdoors, open sky.** First fix in roughly 30-60 s (cold start: no backup battery on the carrier).
   - On the `PX1105R raw:` line, epochs are handed over and go from untimed to timed once the arrival line is
     valid (10 s after the first fix).
   - Ephemerides count up for GPS, GAL and BDS, with 0 refused.
   - The clock line is valid, with an rms well under 1 ms.
   - The `[RAWGNSS]` line shows the filter navigating with GNSS: ephemeris store filling, satellites used, pad
     datum set.
5. **Static check.** On the pad the raw filter's velocity should sit within a few cm/s of zero, and its position
   within a few metres of the receiver's own fix.
6. **Power-cycle and reset.** A power cycle re-runs the whole setup from 115200. An FC-only reset (if the rail
   holds) finds it at 921600. Both should end `set up`.
7. **Log.** Download a short log and check the 0x95 records hold whole SkyTraq frames (A0 A1 … 0D 0A).

## Offline evidence (2026-10-21)

`tools/raw_replay --receiver px1105r` runs the image's chain (the same sources) on the PX1105R's captures. Results
are in the session scratchpad's `px_replay/final/`.
- **rp5 sky.** This is the RF rig's flight replay, with the sim's IMU on the same trajectory. Compared with the
  mosaic V9's own on-board run of that sky:
  - attitude is the same within ~0.1 deg in every phase;
  - boost and coast velocity are the same (p90 1.24 / 0.12 m/s against 1.37 / 0.14);
  - velocity at apogee ±2 s is 0.25 against 0.10 m/s (−0.18 m/s vz bias, the 20 ms tag stand-in), and in descent
    0.14 against 0.06.
- **Doppler pairs.** Keep them on. Off, coast velocity goes from 0.12 to 0.39 m/s.
- **A tag 50 ms late** puts coast velocity at 0.40 m/s, which is why the latency matters.
- **Tracking is the PX1105R's weakness.** At ignition it drops most GPS L1 satellites, holding BeiDou. On a weak sky
  (median C/N0 32 dB-Hz) it dropped all of them from T+0.6 to T+23.6 s, so the filter flew unaided until then.
- **20 minutes of real open sky** (GNSS alone, a still IMU):
  - velocity 45 mm/s rms;
  - position within 1.2 m of the receiver's own mean fix;
  - Dopplers 0.054 m/s rms paired (GPS, Galileo);
  - group delays, ionosphere and ephemerides all check;
  - no sign, lag or clock errors found.
- **Cold-start height.** On the rig the receiver's first fix was 12 m high (the rig's ionosphere against the
  receiver's), and the barometer then held the filter's absolute height there. AGL, velocity and attitude are
  unaffected. On the real sky the two agreed within 0.5 m vertically.

## Not verified yet

- The setup against the real receiver. The order and payloads follow the rig's tools, which used them through a
  bridge on the TinkerNav PX1105R, but never this sequence from the FC.
- The 0xE5 delay (latency), above.
- The filter with a real flight IMU and the PX1105R together (the replay's IMU is simulated).
- Flight. Its antenna wind-up is taken as 1 cycle a turn (an RHCP patch facing up the nose: unmeasured).
