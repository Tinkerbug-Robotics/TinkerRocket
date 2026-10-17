# V9 GNSS auto-detect: bench plan

**Written 2026-10-16, branch `fc/gnss-autodetect`.** One V9 flight-computer
image for either receiver on J1, the stock u-blox SAM-M10Q carrier or the
Septentrio mosaic-G5 harness (mosaic-go G5 P3), found at boot. Until now the
receiver was a build choice (`-DTR_GNSS_MOSAIC=1`), and an image built for the
other one searched for 30-60 s and flew GNSS-absent.

Not for the Goblin's 2026-10-24 flight: it flies the mosaic-only image from
`fc/mosaic-g5`. Nothing here has run on hardware yet. Host tests, the
simulated line and the builds are all it has.

## What the image does

At boot, before either driver starts (`TR_Sensor_Collector/gnss_detect.h`,
`gnss_auto_driver.h`):

1. It reads the receiver the last boot found (NVS `gnssauto`/`rx`).
2. It probes. Each probe sets the UART baud, sends a few bytes and listens:
   - For the mosaic: a CR at 2 Mbaud (set up) and at 115200 (factory). Proof
     is a CRC-valid SBF block, the `COM1>` prompt, or a `$R` reply.
   - For the u-blox: a UBX-MON-VER poll at 460800 (set up), at the baud its
     driver last found it at, and at 38400 and 9600 (factory). Proof is a
     checksum-valid UBX frame or NMEA sentence.

   Last boot's receiver is probed first, so a boot with the same receiver
   costs one probe.
3. It runs the proven receiver's driver exactly as that receiver's own image
   ran it. If nothing answered, it runs the u-blox driver, which is what every
   V9 ran before this. If the mosaic was there last boot, it flies
   GNSS-absent after 30 s instead.
4. If the u-blox driver's own 60 s search finds nothing, it takes a last look
   for a mosaic that was still booting.

Nothing touches J1 pin 5 (GPIO2) until the mosaic is found. On the PX1105R
carrier that pin is a receiver input.

The rest of the image follows the receiver:

- **IMU limit.** With the mosaic-G5 the IMU stops at 3840 Hz: its SBF is about
  the IMU's size at 4k, and 8k would put the FC->OC link at ~92 % and outrun
  the OC's flash. A setting above the limit flies as 4k Dynamic or 3840 Hz.
  The FC and the OC both keep the setting as sent, so a u-blox gets 8k back
  and the two copies never disagree.
- **The app.** The FC names the receiver in its identity push. The OC relays
  it as `fc_gnss` and lowers `irmax`, so the app shows the receiver under the
  GPS row and offers no 8k rates with the mosaic.
- **Crash restarts.** A boot after a crash leaves the mosaic out, as the
  mosaic image did. A u-blox board keeps its receiver through a crash, as it
  always has.

## Before you start

- **Images.** Build both from `fc/gnss-autodetect`:
  - FC: `idf.py -B build_v9 -DTR_BOARD_V9=1 build`. The image refuses a build
    dir whose `sdkconfig` predates it, because it needs the UART interrupt in
    IRAM (`sdkconfig.defaults.mosaic`). Use a fresh dir, or delete that
    `sdkconfig`.
  - OC: `idf.py -B build_v9 -DTR_BOARD_V9=1 build` in `out_computer`.

  An older OC works with the new FC: the app then shows no receiver and still
  offers the 8k rates, and the FC refuses or caps them on its own.
- **Apps.** iOS and Android from the same branch, for the receiver under the
  GPS row. Older apps ignore the new key.
- **Hardware.** A V9 on the bench with the FC console on USB. The FC and OC can
  share a port name, so check the USB serial number (the MAC) before flashing.
  You also need the SAM-M10Q carrier, the mosaic-go G5 harness with its
  antenna, and sky or a re-radiator for fixes.
- **Before each test**, note the IMU rate setting in the app. Test 4 checks
  what 8k Dynamic turns into.

## Tests

Every line quoted below is on the FC console under the `GNSS` tag unless it
says otherwise. Times are from the simulated line in
`tests_cpp/test_gnss_detect.cpp`. The bench numbers are what this plan is for.

### u-blox

**1. First boot of the image on a u-blox V9.** Flash over USB or OTA, then let
it boot.
- `GNSS auto: which receiver is on J1 (RX=3 TX=4)? Last boot: nothing known`
- `GNSS auto: u-blox on J1 (a UBX frame at 460800 baud, ...)`. After an OTA the
  receiver kept its power: about 10 ms, 1 probe. After a USB flash and a power
  cycle it is at 38400 or 9600: under 0.5 s.
- Then the u-blox driver's own lines, as on today's image (`Trying remembered
  GNSS link...`, `Connected at ...`), and `GNSS auto: remembered u-blox for the
  next boot`.
- `[GNSS] Receiver: u-blox; IMU rates up to 7680 Hz` (main).
- With the new OC and app: `u-blox` under GPS, and the 8k rates still offered.

**2. Power cycle, same receiver.**
- `Last boot: u-blox` and found in 1 probe (sim: 221 ms at 9600, 60 ms at
  38400).
- Compare the time from power-on to `GNSS found and initialized` against the
  u-blox-only image on the same board (`-DTR_GNSS_AUTO=0`). Expect it within
  about 0.3 s.

**3. Fix and log.** Outdoors or on the re-radiator: a 3D fix, and GNSS_SAT_MSG
in a short log as before. No SBF records (`0x95`) with the u-blox.

### mosaic-G5

**4. Swap J1 to the mosaic harness** (the receiver keeps the setup a flight
computer saved into it), set the IMU to 8k Dynamic in the app if you can,
power on.
- `Last boot: u-blox`, then the u-blox probes (~0.6 s of nothing), then
  `GNSS auto: mosaic-G5 on J1 (its prompt at 2000000 baud, ~700 ms, 4 probes)`.
  Or "an SBF block": either is proof.
- `GNSS auto: remembered mosaic-G5 for the next boot`, then the mosaic
  driver's lines: `Looking for the mosaic-G5 ...` and `mosaic-G5 set up: ...`.
- `[GNSS] Receiver: mosaic-G5; IMU rates up to 3840 Hz`. With 8k Dynamic set:
  `[GNSS] the mosaic-G5 caps the IMU at 3840 Hz: the logging rate setting 1
  flies as 0 (the setting is kept)`.
- After 30 s: `mosaic: 20.0 MeasEpoch/s ...` and `mosaic PPS: ... paired`.
- OC console: `FC GNSS receiver mosaic-G5: IMU rates up to 3840 Hz`. App:
  `mosaic-G5` under GPS, and the rate menu shows 4k Dynamic and no 8k
  choices.
- An app that connected while the FC was still booting still offers 8k.
  Pick 8k Dynamic there. The FC logs `[CFG] IMU logging rate setting 1 flies
  as 0: the mosaic-G5 caps the IMU at 3840 Hz`, and the app comes back to 4k
  Dynamic once the FC has named its receiver.

**5. Power cycle, same receiver.**
- `Last boot: mosaic-G5` and found in 1 probe (sim: 1 ms, by the next block or
  the prompt).

**6. Log with the mosaic.** Record a minute.
- `Data_Analysis/extract_gnss_sbf.py` gets a clean `.sbf` from it, as from the
  mosaic image.
- The IMU runs at 3840 Hz.
- The FC's I2S enqueue drops and the OC's `rx_ovf` stay at zero.

**7. Factory mosaic (optional; it changes the receiver).** Reset the receiver
to factory over its USB (`eccf, RxDefault, Boot`, then power-cycle), put it
back on J1, boot.
- `GNSS auto: mosaic-G5 on J1 (its prompt at 115200 baud, ...)`.
- The mosaic driver then switches it to 2 Mbaud and saves the setup again,
  so the receiver ends where it started.

### Back and forth, and nothing

**8. Back to the u-blox.** Swap J1 back.
- `Last boot: mosaic-G5`, then the mosaic probes, then `u-blox on J1 ...`,
  and `remembered u-blox`.
- `IMU rates up to 7680 Hz`, and 8k Dynamic flies again if it was set.

**9. Nothing on J1, u-blox last time.** Unplug J1, power on.
- About 1.1 s of probes, then `no receiver answered the probes ...; the
  u-blox driver searches as it always has`.
- Its 60 s search, then `the u-blox driver found nothing; a last look for a
  mosaic-G5`, and `no mosaic-G5 either`.
- GNSS-absent as today: the app's NO GNSS banner and the boot status's GNSS
  bit.
- Time from power-on to the loop: about 62 s more than a healthy boot, and no
  rescue mode (the setup watchdog is 120 s).

**10. Nothing on J1, mosaic last time.** Do test 5, unplug, power on.
- About 31 s of probes, then `the mosaic-G5 that was here last boot did not
  answer in 30 s; continuing without GNSS` and `remembered no receiver`.
- The next boot takes the plan for nothing known: 6.6 s of probes, then the
  u-blox driver's 60 s.

**11. A mosaic that boots late (optional).** Power the mosaic separately and
switch it on a few seconds after the FC.
- With the mosaic last time, it is found once it is up, within 30 s.
- With the u-blox last time, after the u-blox driver's 60 s: `a last look for
  a mosaic-G5` then `mosaic-G5 on J1`.

**Not covered here: a single crash restart.** With the mosaic last boot, a
boot that follows a panic or a watchdog should print `[GNSS] the last reset was
a crash (...): the mosaic-G5 stays out of this boot` and fly GNSS-absent; with
the u-blox it should keep its receiver. I found no way to cause one crash on
demand: the rescue-mode test build (`-DTR_TEST_SETUP_FAIL=2`) aborts on every
boot, before the GNSS bring-up, so it loops into rescue mode instead. The rule
is the mosaic image's, with the last boot's receiver added to its condition
(`setup_fc()`).

A board whose u-blox its driver found on swapped wires (its `gnsslink`
record) skips the probes and goes straight to the u-blox driver:
`GNSS auto: the u-blox driver last found its receiver on swapped wires:
straight to it, no probes`. No V9 carrier is wired that way that I know of.

## The u-blox on a V8

A V8 stands in for a SAM-M10Q V9 on the bench. It has the same receiver
wiring (the UART on GPIO3 and GPIO4, the same SAM-M10Q carrier) and the same
P4, so its u-blox path is the same code on the same pins. Two things it cannot
show:
- 8k with the u-blox: the V8 has no IMU FIFO capture, so it stops at 3840 Hz
  anyway.
- The mosaic tests: its J1 has not been checked for the harness, and on a V8
  the auto image gives the mosaic no PPS pin.

**Images**, both from this branch:
- Baseline, the V8's normal u-blox-only image:
  `idf.py -B build_v8 -DTR_BOARD_V8=1 build`.
- Test image, the auto image as a V8 bench build:
  `idf.py -B build_v8_auto -DTR_BOARD_V8=1 -DTR_GNSS_AUTO=1 build`.

**Flashing.** Over USB to the FC. Check the port's USB serial number (the MAC)
first, then `idf.py -B <dir> -p <port> flash`.

**Boot logs.** `tools/bench_capture_boot.py <port> 90 <log>` resets the FC
through the USB port and records 90 s. That reset is `rst:0x17
(CHIP_USB_UART_RESET)`, not a power-on, so the probes try 460800 first. The
FC switches the receiver's rail itself (GPS_ACT, GPIO15) and doesn't hold it
through a reset, so a reset can still cost the receiver its setup: it then
answers in NMEA at 9600. Both ways are fine, but they read differently in the
log.

**V8-1. Baseline, three boots on the u-blox-only image.** Note:
- the time from `Initializing GNSS` to `GNSS found and initialized`;
- the u-blox driver's lines: the remembered link, `Connected at ...`, the
  module, the OTP state, the rate set.

**V8-2. Flash the test image, first boot.**
- `GNSS auto: which receiver is on J1 (RX=3 TX=4)? Last boot: nothing known`.
- `GNSS auto: u-blox on J1 (a UBX frame at ... baud, ... ms, ... probes)`.
- Then the same u-blox driver lines as V8-1, and `remembered u-blox`.

**V8-3. Three more boots.**
- `Last boot: u-blox`, found in 1 probe when the receiver kept its setup
  through the reset. When it didn't, expect 3 probes (about 0.7 s, NMEA at
  9600).
- The u-blox driver's lines should match V8-1 line for line.
- The GNSS bring-up time should be within the probe time (0.1-0.6 s) of
  V8-1's.

**V8-4. Fix and a short log (optional).**
- GNSSData at 18 Hz, GNSS_SAT_MSG present, and no SBF records (0x95).
- The same as a log from the baseline image.

**V8-5. Nothing on J1.** Unplug the carrier and boot each image.
- The baseline goes GNSS-absent after its 60 s search.
- The test image: about 1.1 s of probes, the same 60 s search, `the u-blox
  driver found nothing; a last look for a mosaic-G5` (~0.8 s), then
  GNSS-absent.

**Afterwards.** Flash the baseline image back unless you want the V8 to keep
the test image.

### Results, 2026-10-17

The V8 "RC V8": FC MAC 30:ED:A0:E3:6A:5B, its OC on
393fed6c-dirty-v8+20261017-0940. Both FC images from this branch, flashed
over USB after the MAC check. The time is from `Initializing GNSS` to
`GNSS found and initialized`.

| Boot | Image | Receiver | Detection | GNSS up |
|---|---|---|---|---|
| first after the flash | u-blox only | 9600 | - | 14.3 s |
| warm | u-blox only | 460800, saved link 9600 tried first | - | 4.86 s |
| first after the flash | auto | 9600 | NMEA at 9600, 699 ms, 3 probes | 14.5 s |
| hinted | auto | 460800, saved link 9600 tried first | UBX at 460800, 62 ms, 1 probe | 4.96 s |
| hinted | auto | 460800, saved link answered at once | UBX at 460800, 52 ms, 1 probe | 0.31 s |
| hinted | auto | 9600, the reset dropped the rail | NMEA at 9600, 706 ms, 3 probes | 15.2 s |

- V8-1 to V8-3 pass. The u-blox driver's lines match the baseline's line for
  line in the matching cases: the saved link, the bootstrap, `Connected at`,
  SAM-M10Q SPG 5.10, the OTP check, the rate set. The auto image adds its own
  lines, and writes the hint only on the first boot.
- After each flash the receiver was back at 9600: the seconds in the
  bootloader drop its rail. One USB reset in four did the same.
- The OC's readback (BLE cmd 20) with the auto image: `fc_gnss` "u-blox",
  `irmax` 3840 (the V8's own limit), `irate` 0. Telemetry: PRELAUNCH, 8 sats,
  the FC->OC link carrying data.
- Not run: V8-4 (log), V8-5 (nothing on J1), and a real power-on of the auto
  image.
- Seen on both images, not from this work: every boot logs
  `[RECOVERY] read OK but no SNAPSHOT frame (first bytes AA 55 AA 55 A9)` five
  times, about 1.5 s of setup.
- Today's u-blox driver saves the baud it first connected at, before its
  switch to 460800. So a warm boot after a 9600 boot spends 4.6 s on 9600
  first. The detection already knows the baud within about 60 ms. Handing it
  to the driver would save that time, but it changes the u-blox path, so it
  isn't in this branch.

## The mosaic on a V8

From 2026-10-17 the V8's auto image gives the mosaic-G5 its 1PPS, as the V9 does. The V8's J1 is the
V9's: the same BM05B-SRSS-TB, module RXD / TXD / VSYS / VSS on pins 1-4, and GNSS_RXD2 = GPIO2 on pin 5. So
the Goblin's mosaic harness plugs in unchanged. The plan is one V8 image (`-DTR_BOARD_V8=1
-DTR_GNSS_AUTO=1`) for two flights with no reflash between them: the Goblin with the mosaic, then another
rocket with a SAM-M10Q. Each rocket still needs its own settings (pyros, servos, orientation) from the app.

**Before the Goblin flight**, with the mosaic harness on the V8's J1:
- The boot log shows `GNSS auto: mosaic-G5 on J1`. The readback shows `fc_gnss` "mosaic-G5" and `irmax` 3840.
- The mosaic stays up on the V8's VSYS. This is the first time a V8 powers it.
- With a fix and a minute of log: SBF at 20 Hz with none missing, `GnssClock` valid with PPS on every
  epoch, and the IMU at 3840 Hz with no gaps (as test 6 on the Goblin's V9).

**Back to the SAM-M10Q**: the first boot tries the mosaic first, because the last boot found it, then
finds the u-blox about a second later. That is the "Back and forth" test above.

## If something goes wrong

- **The u-blox-only image**, exactly what every V9 ran before this:
  `-DTR_GNSS_AUTO=0`.
- **The mosaic-only image**: `-DTR_GNSS_MOSAIC=1`, the Goblin's.
- **Wrong receiver chosen.** The `GNSS auto:` lines say what each probe proved
  and at which baud. Send me that boot log.

## Expected detection times (simulated)

| Receiver on J1, as found | Last boot | Proof | Time | Probes |
|---|---|---|---|---|
| u-blox, factory 9600 | u-blox (driver found it at 9600) | UBX frame at 9600 | 221 ms | 1 |
| u-blox, factory 38400 | u-blox (38400) | UBX frame at 38400 | 60 ms | 1 |
| u-blox, set up, after an OTA | u-blox | UBX frame at 460800 | 11 ms | 1 |
| u-blox, factory 9600 | nothing known | UBX frame at 9600 | 399 ms | 2 |
| u-blox, factory 9600 | mosaic-G5 | UBX frame at 9600 | 818 ms | 4 |
| mosaic, set up | mosaic-G5 | SBF block at 2 Mbaud | 1 ms | 1 |
| mosaic, set up | u-blox or nothing known | prompt at 2 Mbaud | 700 ms | 4 |
| mosaic, factory | nothing known | prompt at 115200 | 850 ms | 5 |
| nothing | u-blox | — | 1.1 s, then the u-blox driver's 60 s | 5 |
| nothing | nothing known | — | 6.6 s, then the u-blox driver's 60 s | 30 |
| nothing | mosaic-G5 | — | 30.7 s, GNSS-absent | 140 |
