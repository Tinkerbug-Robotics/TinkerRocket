# Mosaic-only raw-GNSS navigation image

A Tinker-Mantis V9 (or a legacy V8) flight computer with a Septentrio mosaic-G5 on J1, flying the raw-GNSS filter
as its only navigation filter. GpsInsEKF is not built into this image. This page names the configuration, says what
it carries and what it leaves alone, and points at the evidence behind it.

This is a build configuration, not a release. The image the Goblin flies on 2026-10-24 is picked from it later in
the week.

## Build it

From `tinkerrocket-idf/projects/<project>`, with ESP-IDF v6.0:

| Board | Flight computer (`flight_computer`) | Out computer (`out_computer`) |
|---|---|---|
| V9 | `idf.py -B build_v9_rawnav -DTR_BOARD_V9=1 -DTR_GNSS_MOSAIC=1 build` | `idf.py -B build_v9 -DTR_BOARD_V9=1 build` |
| V8 | `idf.py -B build_v8_rawnav -DTR_BOARD_V8=1 -DTR_GNSS_MOSAIC=1 build` | `idf.py -B build_v8 -DTR_BOARD_V8=1 build` |

- **Use a fresh build directory.** A build directory keeps every `-D` value and its `sdkconfig`, PSRAM included.
- **Check out the private `TR_GuidancePN` submodule first.** Until releases start, every flight computer build
  carries the real guidance (owner, 2026-10-18). Without the submodule the build falls back to
  `TR_GuidancePN_Stub`. The configure log says which: `TR_GuidancePN submodule found — guidance compiled in`.
  Guidance runs only where the unit's own guidance setting turns it on, as on every image.
- **The configure log names the configuration:** `raw-GNSS NAVIGATION filter; GpsInsEKF not instantiated
  (ephemeris store 128)`, after the mosaic-G5 driver and its two overlays.
- **The version** reads `<sha>-v9+<date>` (or `-v8`). Any bench switch adds `-bench` to it; never fly an image
  marked `-bench`.
- **CI builds both flight computers:** the `V9-mosaic` and `V8-mosaic` jobs in
  [`firmware-build.yml`](../../.github/workflows/firmware-build.yml).

`-DTR_RAW_GNSS_SHADOW=0` builds the mosaic image with GpsInsEKF alone and PSRAM off instead. That is not this
configuration.

## What it includes

### One navigation filter

The raw-GNSS filter (`TR_RawGnssEKF`, run by `TR_RawGnssShadow`) fuses the receiver's own pseudoranges and Dopplers
with the IMU, barometer and magnetometer. The flight logic reads it through `RawNavAdapter`
([`raw_nav_adapter.h`](../../tinkerrocket-idf/projects/flight_computer/main/raw_nav_adapter.h)), GpsInsEKF's face
on the raw filter's state, as on the Beetle. `config.h` refuses a mosaic build that would fly both filters.

### The receiver path

- **The link.** The mosaic-G5 on J1's UART (GPIO3/GPIO4) at 2 Mbaud, SBF at 20 Hz, its 1PPS on J1 pin 5 (GPIO2).
  The UART interrupt runs from IRAM (`sdkconfig.defaults.mosaic`).
- **The measurements.** Each MeasEpoch becomes a raw epoch: GPS L1 C/A, Galileo E1 and BeiDou B1I, BeiDou GEO left
  out ([`MosaicRawEpoch.h`](../../tinkerrocket-idf/components/TR_GNSSReceiverMosaic_Serial/MosaicRawEpoch.h)).
- **The time tag.** Each epoch is tagged from the 1PPS line, and is either tagged right or handed over untimed
  ([`MosaicTimeTag.h`](../../tinkerrocket-idf/components/TR_GNSSReceiverMosaic_Serial/MosaicTimeTag.h)). After a
  slip in the sky the filter flies a few seconds on the IMU and barometer; it never fuses a wrong tag.
- **The ephemerides.** GPSNav, GALNav and BDSNav, and the ionosphere from GPSIon
  ([`SbfNav.h`](../../tinkerrocket-idf/components/TR_GNSSReceiverMosaic_Serial/SbfNav.h)), into a 128-entry store.
  The receiver is asked to send its navigation blocks again on the pad, at launch and after landing.
- **The fused grid.** 100 ms of GPS time, at most 16 satellites an epoch (the mosaic-G5's receiver profile).
- **The log.** Every SBF block is logged raw (0x95), so the offline harness can replay the flight.

### Ascent safeguards

They are set in one place, `gnss_raw::applyMosaicFlightSafeguards`
([`ReceiverProfileApply.h`](../../tinkerrocket-idf/components/TR_RawGnssShadow/ReceiverProfileApply.h)), which the
offline harness calls too. The boot prints them on one `[RAWGNSS] flight safeguards:` line.

- **The Doppler change check, acting in flight.** Each satellite's Doppler change is checked against the IMU's
  velocity change along its line of sight. A stale or settling receiver's Dopplers are held out of the filter.
- **The heading share.** GNSS's gain on the heading is scaled by how well the moment observes the heading.
- **The turning body's Doppler allowance.** Each Doppler's sigma widens by what a turning body swings an antenna
  offset nobody measured (bounds of 0.30 m along the nose and 0.03 m across it), on an epoch whose rate across the
  nose reached 2 rad/s: the drogue's snatch or a tumble, not the pitch-over at apogee.
- **The spinning body's settings** (below): the exact attitude transition, the change check's lever allowance on
  the nose's own turn, and the carrier wind-up taken off each Doppler.
- **GNSS on the whole attitude from launch** (`config::RAW_HEADING_GATE` 1).

### The spinning body (high roll)

The Goblin's past flights rolled at 0.32-0.85 rev/s (a 1 s mean). Three settings keep the filter and the change
check working under a roll (fc/v9-highroll, from the windup study of 2026-10-18):

- **The exact attitude transition** (`att_exact`). Each 10 ms step turns the attitude's uncertainty with the body by
  exp(-[w x] dt), not by I - [w x] dt. The first-order step stretched the tilt's variance by 1 + (|w| dt)^2 a step:
  1.5 times a second at 1 rev/s, and to its cap within a second at 5 rev/s, where Doppler noise then turned the
  attitude.
- **The change check's lever allowance on the nose's own turn** (`dv_lever_mode` 1, no radial bound). The allowance
  for an antenna off the IMU is the arm (0.3 m, or with the offset known what its prediction can be off by) times
  how fast the nose turns, not the whole rate: a roll about the nose does not move an antenna on the nose's axis.
  With the roll in it, the check's 1 s window allowed 1.1 m/s at 0.3 rev/s and 1.9 m/s at 0.5, so by the model no
  slow receiver ramp (2-2.5 m/s^2) was caught at the Goblin's own roll rates, and the raw vote guard only logs. The
  nose's turn is tracked in float each step.
- **The carrier wind-up taken off each Doppler** (`windup_k` 0.99: the Goblin's helix, measured on the 10/16 spin
  rig). A circularly polarized antenna turning about its boresight adds about 0.19 m/s per rev/s to every L1/E1
  Doppler. The filter's clock drift takes that common part either way (no ascent median moved more than 0.03 deg in
  the study), but with it taken off, the drift the flight logs is the receiver's own. k belongs to the antenna, so it
  is set with the image's flight settings and the receiver profile keeps 0. In the firmware only the mosaic-only
  images take that profile, the V9's and the V8's (`config.h` stops a raw-GNSS build with the auto driver), and both
  call `applyMosaicFlightSafeguards`, so both fly 0.99: the V8 is taken to fly the Goblin's helix too, its boresight
  along the nose. The profile's other users keep 0: `raw_replay --receiver mosaic` built without
  `RAW_REPLAY_MOSAIC_SAFEGUARDS`, and the simulator's bindings (`mosaic_g5_profile`, `profile_for`). A wrong k is
  common to every Doppler of one band, and the clock drift takes it.

Where it fails toward unaided, and what it leaves out:

- **An antenna off the roll axis that nothing predicts** (the offset Unknown, or Measured with no magnetometer
  heading) changes the Dopplers between epochs by up to twice its own speed, which the allowance no longer covers.
  At a fast roll the check then holds the coast's Dopplers and the filter coasts unaided: for the Goblin's 12.6 mm, by
  the model, from about 3.5 to 9 rev/s and above about 11 rev/s; at 1.5 rev/s or less, nothing. A radial bound of
  about 4 mm would keep those Dopplers but lose the slow-ramp catch from about 7.5 rev/s; it is not taken.
- **A roll axis off the board's x** cones the nose at the roll rate, which the allowance takes as a radial bound of
  the arm times the tilt (5.2 mm a degree with the offset unknown): at 1 deg the slow ramps are caught to about
  5 rev/s. `sim_events.py --roll-axis-tilt-deg` puts it in the simulated flights.
- **The gyro's rail** (about 17.6 rev/s of pure roll on the V9) is not handled: every filter fails there alike.

The windup study's grid (48,000 runs with a roll laid over Grid B, the antenna on the roll axis) passed every adopted
bar from 0 to 15 rev/s with the exact transition and the nose lever on, where the image as built failed the coast
from 1.5 rev/s and let the slow ramps through from 1 rev/s
(`gnss-attitude-study/v9-mosaic-filter-plan/evidence/windup/`). The grid at the Goblin's own antenna offset, with
it known and unknown, has not been run yet. Nor has the V9's step time (the filter's `prop_us` and `step_us_max`,
these settings on against off): take it on the flight image's placement. With `-DTR_RAW_GNSS_IRAM=1` the filter
runs from internal RAM, but the exact transition's blocks (`attExactBlocks`) stay in flash with the carrier's code
(`main/raw_gnss_iram.lf`), so that bench pays a flash fetch every 10 ms step that neither the flight image nor an
internal-RAM placement would. The boot line `[RAWGNSS] flight safeguards:` names the three settings.

### The GNSS antenna offset (BLE cmd 81)

An owner setting in both apps (Settings, "GNSS Antenna Offset"): Unknown, the default, or Measured, with the IMU to
the antenna's phase center in millimeters along the board's own axes. The out computer stores and relays it; the
flight computer keeps it in NVS (`antenna`/`ofs`), turns it into the filter's frame with the IMU's mounting, and
takes it only on the ground: one relayed in flight is held and applied once the flight computer is not INFLIGHT.
The flight computer reports the offset it holds in its config report (`ConfigReportData` v4), and the apps' readback
shows that one (`antsrc` "fc") once the flight computer has taken the offset last sent and keeps one of its own (or
both are Unknown). Until then the out computer's copy stands in (`antsrc` "oc"): a flight computer with no offset of
its own (a fresh board, or its NVS erased) is sent the out computer's again, and its first report after power-on
comes before it takes one queued with its rail off. One that does not take it shows its own again once the command
has been served. The rule is in
[`antenna_readback_policy.h`](../../tinkerrocket-idf/projects/out_computer/main/antenna_readback_policy.h). The Beetle's out computer
leaves the setting out of its readback, so the apps lock it there. The apps send nothing until all three values
are entered, and take a readback into the rocket's profile only when the flight computer reported it.

- **Unknown** flies the image as described above: the turning body's 0.30 m and 0.03 m bounds stand in for the
  antenna nobody measured.
- **Measured:** every Doppler and pseudorange is predicted at the antenna, and a few centimeters of doubt on the
  value (`antenna_sigma_*`) replace those bounds and the change check's lever allowance. The change check also
  allows for the attitude turning the antenna's predicted motion: with the heading known, the attitude allowance's
  share of the offset; with no magnetometer heading, it takes the prediction back out and allows the whole offset,
  as Unknown does with its 0.3 m, and its latch lets go at Unknown's level the same way. With no magnetometer
  heading, each Doppler's sigma also allows twice the offset (any heading can turn the predicted motion that far)
  once the body turns at 2 rad/s or more across the nose, where Unknown's bounds start, so a swing's Dopplers let in
  after a latch cannot pull the velocity off. That costs some descent accuracy and next to none in the ascent. The
  offset's roll terms are observed and logged, never fused.
- **The log:** the pad context gains a record, `PCX_ANT`, beside the calibrations at launch + 5 s: the setting, the
  same in the filter's frame, the offset the navigation filter had on at the record (the one it flew with: an
  earlier one while a setting sent just before launch waits for the ground), and whether that was the setting.
  `tools/raw_replay/extract.py` passes the offset the filter flew with to the harness (`META antenna_m`), which
  replays the flight with it unless given `--antenna-m none`; a setting not flown with is `META antenna_m_set`,
  only named.

### The log-only vote guard

With one filter, the barometer's climb rate is the only independent check on the filter's apogee votes
([`raw_vote_guard.h`](../../tinkerrocket-idf/projects/flight_computer/main/raw_vote_guard.h)). After burnout the
guard latches when the filter's vertical velocity reads more than 10 m/s below the barometer's climb for 1 s. It
logs the latch (the console, the `[RAWGNSS]` line and the FcStatus bit `FCS_RAW_VOTES_WITHHELD`), but the filter's
votes are still cast (`config::RAW_VOTE_GUARD_ACTS` is false): on the simulated flights a lagging static port
tripped it on a good filter and pushed the apogee call past its bar.

### Restore and crash-loop rules

These are pure and host-tested
([`raw_nav_policy.h`](../../tinkerrocket-idf/projects/flight_computer/main/raw_nav_policy.h)).

- **A reboot in flight** asks the out computer for the flight before the receiver's bring-up, and does not wait for
  the receiver. The filter starts again from the crash-recovery snapshot (100 m, 20 m/s and 15 deg sigmas). Its
  height comes from the barometer when one can vouch for itself within 3 s. The snapshot's spare float carries the
  receiver's geoid separation for that start.
- **A filter that stops in flight** holds its last good state (unhealthy, so it casts no vote) and is restarted
  from it, at most 3 times a flight.
- **Two crash resets in a row:** the next boot skips the filter and flies the barometer and the receiver's own fix,
  so the flight computer still reaches its loop and can take an update.
- **The barometer's transonic lockout** follows the filter's speed only from launch to just after burnout, and only
  while the filter is healthy.
- **No filter:** if the filter cannot start (for example no PSRAM), the boot says `*** NO NAVIGATION FILTER ***`
  and reports a degraded sensor status the apps show.

### Rig replay support

`-DTR_SIM_REPLAY_BLOB=FILE` builds a rig replay image of this configuration: the flight's IMU, barometer and
magnetometer are played from the file on the receiver's own GPS time, while the RF rig plays the same flight's sky
into the mosaic-G5. It is a bench image (marked `-bench`, every pyro dry). See
[the V9 section of the rig replay README](../../tools/rig_replay/README.md#the-v9-with-the-mosaic-g5).

### PSRAM

[`sdkconfig.defaults.rawnav`](../../tinkerrocket-idf/projects/flight_computer/sdkconfig.defaults.rawnav) turns the
in-package PSRAM on for this image only: the filter does not fit in internal RAM. Only explicit PSRAM allocations
land there: the filter and its working storage, and the driver's 64-entry navigation-block queue. Default task
stacks, queues and `malloc()` stay internal.

## What it does not change

This is a statement about behavior, not bytes.

- **Every other image behaves as main's:** the V9 and V8 auto images (either receiver, found at boot, flying
  GpsInsEKF with PSRAM off), the V8 u-blox image, the V10, the Beetle (M1) and every out computer. What makes this
  image what it is (the raw filter as the one navigation filter, the mosaic-only driver path, PSRAM) compiles only
  where `TR_GNSS_MOSAIC` is set.
- **Shared code compiles into the other images too, and stays off there:**
  - GnssTimeSync's exact-step mode: `exact_step_s` defaults to 0, and only the mosaic-only driver sets it;
  - the mosaic driver's deferred bring-up: `begin_wait_` defaults to true, and only the mosaic-only build calls
    `setBeginWait`;
  - the rig replay clock's 1PPS check (`SimReplay`), which only a mosaic-only replay feeds;
  - on the Beetle, the raw filter's safeguard code, whose settings default off; only
    `applyMosaicFlightSafeguards` turns them on.
- **So the other images grow against main** (both built with the same real guidance and managed components):
  - the V9 and V8 auto images by 1,968 B (flash code and constants, and 184 B of `.bss`);
  - the Beetle by 23,792 B: 21,872 B of flash code, 1,132 B of IRAM code (1,280 B of internal RAM with its
    alignment) and 208 B of `.bss`, so about 1.5 KB less internal RAM.
  - The antenna offset adds, against the configuration without it (main at 9f813b15, the same real guidance): the
    Beetle 5,736 B of flash code, 184 B of flash data and 212 B of IRAM code; the V9 auto image 3,772 B of flash and
    16 B of `.bss`; this image 13,500 B of flash and 96 B of internal RAM. The filter object grows by about 10.5 KB,
    in PSRAM, and the lever's stack frames by under 0.1 KB on every board.
  - The spinning body (fc/v9-highroll) adds, against main at 6276a563 (`idf.py size`, both sides built from a git
    archive with the guidance stub): this image and the V8's 9,642 B of flash (9,338 B of code, 304 B of constants)
    and no internal RAM; the Beetle 8,220 B of flash code, 96 B of flash data and 372 B of IRAM code, no `.bss`; the
    V9 auto image nothing. The shadow object grows by about 1.8 KB on every board that builds it (the gyro's history
    for the wind-up, dormant on the Beetle), in PSRAM where the filter is.
- **The Beetle's filter is bit-identical to main's:** `tools/raw_replay/bitident.sh` reports 450 of 450 runs
  IDENTICAL. `tools/m1_identity.sh` reports its `main.cpp` and sensor collectors preprocess IDENTICAL. Neither
  covers GnssTimeSync; the host tests cover its default path.
- **The wire:** the antenna offset adds BLE command 81, the out computer's ANTENNA_CONFIG_PENDING/MSG pair
  (0x8C/0x8D), the config report's version 4 (`ConfigReportData`, 201 bytes: the offset the flight computer holds,
  and `F_ANTENNA_FROM_NVS`) and the pad context's `PCX_ANT` record (every board but the Beetle writes it). Firmware
  that predates them ignores the command, an out computer that predates version 4 refuses the newer report (the OTA
  relay updates the out computer first), and a log reader that predates them skips the record. FcStatus bit 1 and
  the snapshot's spare float are set only by this image; every other image leaves them 0.

## Bench and rig tools

- [`tools/raw_replay`](../../tools/raw_replay/README.md#the-v8v9-with-the-mosaic-g5---receiver-mosaic): the offline
  harness. `raw_replay_mosaic --receiver mosaic` flies the image's safeguards on a log's SBF (`extract.py --sbf`)
  or on simulated flights (`sim_events.py`, `sim_grid.py`, `sim_tables.py`); `score_static.py` scores still and
  spin-rig logs.
- [`tools/rig_replay`](../../tools/rig_replay/README.md): `run_v9.sh` (one RF-rig run on the V9, after the MAC
  check), `score_v9.py`, `tx_log.py`, `usb_ports.py` and `rinex_gps_iono.py`.
- `tools/raw_replay/bitident.sh` and `tools/m1_identity.sh`: the Beetle's identity gates.
- Bench switches (`flight_computer/CMakeLists.txt`, all marked `-bench`): `TR_BENCH_MOSAIC_CN0`,
  `TR_BENCH_MOSAIC_COLD_START` and `TR_BENCH_MOSAIC_TAG_TRACE` for the receiver; `TR_BENCH_RAW_FUSE_MS`,
  `TR_BENCH_RAW_SAT_CAP`, `TR_BENCH_BOOST_SCREEN_G` and `TR_BENCH_DV_CHECK` for the filter's CPU and checks; the
  four ways to use more GNSS data within the P4's budget (exp/v9-moredata-bench, each off unless set, any of them
  together): `TR_BENCH_RAW_BUILD_LITE` (the cheaper epoch build), `TR_BENCH_RAW_SHARE_LOS` (the epoch's geometry
  shared by its units, 2 with the Doppler's prediction in float), `TR_BENCH_RAW_DOP_AVG` (the Dopplers in pairs)
  and `TR_BENCH_RAW_CAP_MAX` (the satellite cap's governor, with `TR_BENCH_RAW_CAP_PAD` and
  `TR_BENCH_RAW_SLICE_US`); and the robustness hooks `TR_BENCH_RAW_START_FAIL`, `TR_BENCH_RESTORE_SIM`,
  `TR_BENCH_PANIC_AT_S`, `TR_BENCH_PANIC_FILTER_AT_S` and `TR_BENCH_RAW_STOP_AT_S`.

## Evidence

The evidence stays outside the repository, in `gnss-attitude-study/` beside the checkout. None of it holds a
position. The simulated flights and the real logs were run on earlier builds (RC1 and RC2), before the time-tag fix
(which changes how the receiver's epochs are tagged, not the filter). This configuration carries RC2's safeguards.

- **Simulated flights, RC1** (`v9-mosaic-filter-plan/evidence/wave3/final/`, `evidence/grid-b/`). Sixteen
  simulated drogue flights flown through the firmware's own code, 21,504 runs with GNSS faults, antenna offsets and
  magnetometer errors, on RC1 (fc/v9raw-dopp 357e7021, since rewritten as b0092f54). The adopted ascent bar held in
  every pattern: boost and coast attitude error p90 within the larger of the unaided filter's plus 3 deg and
  10 deg. Every apogee call came within 2 s of the true apogee and none above 20 m/s of climb (2,304 of 2,304 at
  the 0.3 m antenna offset). No run stopped or went non-finite.
- **RC2's change check, with no magnetometer heading** (`evidence/wave3/rc2/`). RC2 added `dv_hdg_unset_deg`,
  which `applyMosaicFlightSafeguards` sets to 180: after a launch with no magnetometer heading, the V9's case until
  its magnetometer is calibrated in flight configuration, the change check allows any heading. This is the change
  check this configuration flies.
  - The uniform-heading no-magnetometer arm (`hdg_uniform_faults.txt`): all 24 faults at ±90, ±150 and 180 deg,
    7,680 runs. The adopted ascent bar held in every pattern (1,920 of 1,920 boost, 1,919 of 1,920 coast). Every
    call came within 2 s, none above 8.9 m/s of true climb.
  - The sliced re-run (`sliced.txt`): 13 patterns, 16 flights and all 6 magnetometer arms at 0.3 m, with the
    flight computer's epoch slicing emulated, 4,992 runs. The adopted bars held everywhere (1,248 of 1,248 boost and
    coast), every call within 2 s and at most 4.22 m/s of true climb.
  - The magnetometer arms are RC1's run for run (3,840 of 3,840 identical), and so are the real logs (76 of 76
    runs CSV-identical, `real_logs.txt`).
- **Secondary bars that still fail** (the plan's section 4, on RC1 and again on RC2):
  - the pitch within ±2 s of apogee no worse than the unaided filter's plus 2 deg: 1,977 of 2,304. The misses are
    in the ±40 deg and no-magnetometer arms (which pass 842 of 1,152), where GNSS corrects 30-40 deg of heading as
    the drogue deploys; a calibrated magnetometer moves the flight into the arms that pass;
  - no apogee call or raw vote above 2 m/s of true climb: the slow 8 and coherent ramps fail it (calls at most
    4.22 m/s);
  - the vertical velocity at apogee on the ramps.
  - Unsliced, one run's descent p90 sat 10.9 deg over its twin (the 1.0 s static-port lag) and one run's
    pre-apogee p90 5.2 deg over. Sliced as the flight computer works its epochs, they are 3.3 and 4.9 deg, and pass.
- **Without a calibrated magnetometer** (RC2, `evidence/wave3/rc2/`): at 170-180 deg of pad heading error, 1-2 of
  16 no-fault flights still hold the coast's Dopplers, with the vertical velocity at apogee up to 4.8 m/s. The
  apogee calls are unaffected.
- **Real V9 + mosaic-G5 logs** (`evidence/real-logs/`, `evidence/wave3/final/tables_real.md`). On the 2026-10-16
  spin-rig logs the heading stayed at the unaided filter's (p95 8.0 against 7.9 deg cone off, 4.2 against 4.2 deg
  cone on), with no stops.
- **Hardware in the loop** (`evidence/plots/`, `INDEX.md` and `v9_rig_hil_20261017.png`). Three RF-rig replays on
  the V9 on 2026-10-17.
- **Known and accepted for 2026-10-24** (`evidence/antenna-offset/`). A heading spike at the drogue's snatch comes
  from the antenna's unmodelled offset. The optional offset that removes it lands after that flight.
