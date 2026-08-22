# SDR injection half of the COCOM bench rig (#491)

Generates the GPS signal that the receiver under test is fed, and turns the
resulting NMEA capture into a number. The receiver half of the rig — the USB
passthrough and the FIX / BLOCKED / NO_LOCK classifier — lives one directory up.

The measurement: **where does a hobby-grade GNSS receiver stop publishing
position, and why?** Five parts have been through it. The export limits, where a
receiver implements them, are consistent — velocity at 515 m/s and altitude at
80 km, each acting independently rather than as a conjunction. What differs
between parts is everything else, including two that stop far below the export
limit for unrelated reasons.

A rocket almost never trips both limits at once — boost is fast-but-low, a
space-shot apogee is high-but-slow — which is why the scenarios isolate one
variable at a time.

| Path | What it is |
|---|---|
| `make_trajectories.py` | Writes the five test trajectories as gps-sdr-sim `-x` motion files, plus a JSON ground-truth sidecar |
| `build_scenarios.sh` | Runs gps-sdr-sim to produce the `.C8` + `.TXT` pairs the PortaPack needs |
| `correlate.py` | Maps an NMEA capture onto the injected trajectory and brackets the threshold |
| `rinex3to2.py` | Converts RINEX 3 mixed nav to the RINEX 2 GPS nav gps-sdr-sim requires |
| `pick_start.py` | Picks a scenario start inside the ephemeris file's densest window |
| `patch_horizon.py` | Fixes gps-sdr-sim's visibility mask so it follows the vehicle's altitude |
| `best_geometry.py` | Scans latitude and hour for the most satellites above a given elevation |
| `plot_flight.py` | Altitude, speed, acceleration, lock state and satellite count on one time axis |
| `recovery.py` | Shut lag and re-open latency per blocked window, with satellites in the wait |
| `make_flights.py` | Realistic flight profiles integrated from thrust, drag and gravity |
| `run_fc.py` | Transmits a scenario and records the rocket computer's console, in one command |
| `align_start.py` | Recovers a capture's scenario start time by matching reported to injected altitude |
| `receiver_table.py` | Renders the receiver comparison from `results/receivers.json` |
| `oscillation.py` | Characterises the periodic C/N&#8320; swing in a capture |
| `ubx_config.py` | Identifies a u-blox and switches it to UBX output, RAM layer only |
| `gain_sweep.py` | Finds the working TX gain for a receiver, ranking satellites over C/N&#8320; |
| `serial_probe.py` | Diagnoses a silent UART: baud sweep plus an adapter loopback test |
| `air530_config.py` | Raises an Air530/AT6558R off its 9600 default via `$PCAS01` |
| `blanking.py` | Tests whether C/N&#8320; blanking tracks the gate or free-runs |
| `report_text.html` | **The report's prose — edit this**, then run `build_report.py` |
| `build_report.py` | Assembles `report.html` from the text, `receivers.json` and the figures |
| `make_block_diagram.py` | Draws the rig block diagram used in the report |
| `replot_all.py` | Regenerates every report figure, shading each at its own measured gate |

## Quick start

```bash
# 1. ephemeris -- BKG serves broadcast nav with no account (CDDIS needs an Earthdata login)
curl -sSLo brdc.rnx.gz \
  https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2026/231/BRDC00WRD_R_20262310000_01D_MN.rnx.gz
gunzip brdc.rnx.gz

# 2. trajectories
python3 make_trajectories.py

# 3. baseband -- handles the RINEX 3 conversion and start-time choice itself
./build_scenarios.sh -e brdc.rnx t0_baseline t1_velramp
```

Substitute today's year and day-of-year in the URL (`.../<YYYY>/<DOY>/BRDC00WRD_R_<YYYY><DOY>0000_01D_MN.rnx.gz`).
Verified end to end on 2026-08-19: raw RINEX 3 to a transmit-ready 312 MB `.C8`
with 11 satellites, in 8 seconds.

---

## 1. Legal and safety, first

GPS L1 is a live, protected band and a HackRF at full tilt will deny GPS to
everything within a useful radius, including aircraft.

- **Conducted only.** Coax from the PortaPack straight into `RF_IN` through
  attenuators. No antenna on the TX port at any point in this procedure.
- **DC block is mandatory**, in line, closest to the receiver. SkyTraq
  `RF_IN` pins source ~3.3 V of antenna bias; that must not reach the pad stack
  or the HackRF's PA.
- **Meter the receiver's SMA centre pin for DC before connecting anything.** The
  TinkerNav V25 leaves `VCC_RF` unconnected so its port is cold, but the C3
  bench board carrying the PX1125R has no schematic in any Tinkerbug repo — its
  port has to be measured, not assumed.
- **Mayhem's antenna DC bias must be OFF** (Settings → Radio). It is a global
  setting, not per-app, and it will happily push 3.3 V into your pad stack.

## 2. Hardware chain

```
PortaPack/HackRF TX ──SMA── [50–60 dB pad] ── [DC block] ── RF_IN (PX1125R)
   1575.42 MHz                                                    │
   2.6 MSa/s C8 from SD                       receiver UART ──────┘
                                                    │
                                              USB passthrough ── Mac
                                                                  └ gnss_nmea_monitor.py
```

Target level at `RF_IN` is roughly **−100 dBm**: real-sky GPS is about −130 dBm,
and these boards expect an active antenna, whose LNA would have contributed
another ~30 dB. Levels are set in §6.

## 3. Software you need

```bash
brew install hackrf              # already present on this Mac (hackrf_transfer)
git clone https://github.com/osqzss/gps-sdr-sim
cd gps-sdr-sim && gcc -O3 -o gps-sdr-sim gpssim.c -lm && sudo cp gps-sdr-sim /usr/local/bin/
```

`gps-sdr-sim` is **not** currently installed here; the rest of the chain is.
No pyproj — the trajectory generator writes lat/lon/height directly.

### Ephemeris

Use **[BKG](https://igs.bkg.bund.de/root_ftp/IGS/BRDC/)**, not CDDIS: it serves
broadcast nav over plain HTTPS with no account, where CDDIS wants an Earthdata
login. The catch is that BKG publishes **RINEX 3 mixed** (`BRDC..._MN.rnx`)
while gps-sdr-sim's reader is strictly RINEX 2 fixed-column with no version
handling — `build_scenarios.sh` detects this and converts via `rinex3to2.py`.

Prefer a **same-day** file. These merges are often partial days: the file
collected on 2026-08-19 carried 2 satellites at 06:00 UTC and 31 at 22:00, and
starting a scenario in the thin part costs you the fix with no error message.
`pick_start.py` reads the census and places the start half an hour past the
densest epoch; run it with `--report` to see the table yourself.

## 4. Generate the scenarios

```bash
python3 make_trajectories.py
```

```
scenario            dur   peak alt  peak spd exceeds         .C8 size
t0_baseline         60s      1.0km      50m/s NEITHER           0.31GB
t1_velramp          90s      5.0km     900m/s VELOCITY only     0.47GB
t2_altramp         240s     85.5km     354m/s ALTITUDE only     1.25GB
t3a_both_18km       70s     27.1km     778m/s BOTH              0.36GB
t3b_both_80km      110s     95.5km     671m/s BOTH              0.57GB
```

The `exceeds` column is the point of the script. **The test-2 trajectory as
filed in #491 does not isolate altitude**: climbing 10 → 85 km in 120 s is a
625 m/s vertical velocity, over the 515 m/s limit, so it would have tripped both
gates and settled nothing. COCOM acts on 3-D speed, and the vertical component
of a climb counts. Every scenario here is defined by velocity functions that are
integrated, and checked against both limits, so that mistake cannot come back
quietly.

`t2_altramp` climbs at 350 m/s — 354 m/s in 3-D, 69% of the velocity limit, with
enough margin that a noisy velocity estimate cannot nudge it over — and crosses
from 5 km to 85 km in one continuous run. Continuity matters: a receiver
that never acquires at 70 km looks identical to one that is gated there, and a
single ramp from a known-good 5 km removes that ambiguity.

Then build the baseband:

```bash
./build_scenarios.sh -e brdc.rnx
```

This converts the ephemeris if needed, picks a start time, writes `c8/NAME.C8`
and `c8/NAME.TXT` per scenario, and prints the scenario start time —
**record it**, `correlate.py` needs it. Check the satellite list it echoes for
each run: **below about 8 satellites, stop and fix the ephemeris** rather than
going to the bench and blaming RF levels.

The `.TXT` sidecar is the Mayhem metadata file:

```
center_frequency=1575420000
sample_rate=2600000
```

The app derives the sidecar path by replacing the extension with `.TXT`, so base
names must match and extensions must be upper case. Without it the app silently
falls back to 1575.42 MHz / 2.6 MSa/s — right for us by luck, but do not rely on it.

## 5. SD card

```
/GPS/t0_baseline.C8      /GPS/t0_baseline.TXT
/GPS/t1_velramp.C8       /GPS/t1_velramp.TXT
     …
/APPS/                   ← must contain the GPS Sim .ppma from your firmware release
```

**GPS Sim is an external app**, not part of the firmware image. If it is missing
from the app list, the `APPS` folder on the card does not match the installed
firmware — copy `APPS/` out of the same Mayhem release `.zip` you flashed.

Card requirements are set by throughput, and the margin is thinner than it
looks: 2.6 MSa/s of C8 is **5.2 MB/s sustained**, and the app buffers
`read_size = 16384 × buffer_count = 3` — **48 KB, about 9 ms**. A card that
stalls longer than that underruns the stream. Use a U3/A2 card, formatted exFAT,
with nothing else being read. An underrun shows up in `correlate.py` as a
drifting clock fit, which is why that number is printed.

## 6. Operate the PortaPack

1. Power up. Confirm **Settings → Radio → antenna DC bias is OFF.**
2. Connect the chain of §2. Start with **~70 dB of pad** — more attenuation than
   you think you need.
3. Open **GPS Sim** from the app list.
4. **Open** → the browser opens in `/GPS` and lists only `.C8` files. Pick
   `t0_baseline.C8`. Frequency and sample rate populate from the sidecar; check
   they read **1.575 GHz** and **2.6 MHz**, and that the duration matches the
   scenario (60 s for t0).
5. **Leave the Loop checkbox unchecked.** On loop the app restarts the file at
   EOF, which teleports the receiver back to the start of the trajectory
   mid-capture and destroys the run.
6. Set **TX gain to 0** and the **RF amp OFF**.
7. Press **play**, and watch the receiver, not the PortaPack.
8. If no fix within ~60 s, stop, raise TX gain by 3 dB, and repeat. Work
   **upward from cold** — too hot saturates the front end and looks exactly like
   too cold. When it first fixes, add ~5 dB and leave it. C/N0 should read
   40–45 dBHz; there is ±10–15 dB of AGC tolerance.

Once t0 fixes, the level is set for every other scenario — the pad and the gain
do not change between runs.

## 7. Receiver-side prerequisite

Set the receiver to a **navigation mode that permits the dynamics you are about
to inject**, or you risk measuring its internal dynamics filter rather than the
COCOM gate — and the two produce the same NMEA symptom.

SkyTraq AN0037 (Phoenix binary protocol), *Configure GNSS Navigation Mode*,
**ID 0x64 / sub-ID 0x17**, 4-byte payload:

```
A0 A1 00 04  64 17 <mode> <attr>  <XOR cs>  0D 0A

mode:  0 auto  1 pedestrian  2 car  3 marine  4 balloon
       5 airborne  7 quadcopter  9 SLR (speed lag reduced)
attr:  0 = SRAM only   1 = SRAM + FLASH
```

Airborne, SRAM only, is `A0 A1 00 04 64 17 05 00 76 0D 0A`. Query with 0x64/0x18;
the receiver answers 0x64/0x8B. `../skytraq_cmd.py` does not implement this yet —
it needs adding before the runs are worth trusting.

Record the mode with every capture. **Run the whole matrix in one mode**, and if
you have time, repeat `t1` in `auto` — a different answer between the two is
itself a finding, and tells you the dynamics filter is in play.

## 8. Run the matrix

Order matters: `t0` proves the chain, `t1` answers AND-vs-OR, `t2` reads the
altitude threshold, and **which of `t3a`/`t3b` you run depends on what `t2` says.**

```bash
# terminal 1 — start the capture BEFORE pressing play
python3 ../gnss_nmea_monitor.py --log captures/t1_velramp.log

# terminal 2 — after the run
python3 correlate.py -s scenarios/t1_velramp.json \
                     -t 2026/08/20,12:00:00 captures/t1_velramp.log
```

| Run | Reads | If AND | If independent / OR |
|---|---|---|---|
| `t0_baseline` | chain, level, time mapping | fix | fix |
| `t1_velramp` | velocity gate alone | **holds to 900 m/s** | **blanks at ~515 m/s** |
| `t2_altramp` | altitude gate and its threshold | holds to 85 km | blanks at 18 km or 80 km |
| `t3a` / `t3b` | both limits together | **blanks** | already blanked in t1 |

## 9. Reading the result

Nothing needs to be synchronised by hand. gps-sdr-sim stamps the scenario with
`-t`, and a locked receiver reports that same UTC back in GGA/RMC — so
trajectory time is `NMEA UTC − scenario start`, exact and indifferent to when
Play was pressed. Host timestamps are only a fallback for epochs with no UTC
field, and the offset between the two clocks is fitted across every epoch that
has both. `correlate.py` prints the spread of that fit; **more than ~2 s means a
playback underrun and the run should be repeated.**

```
  t_traj  src verdict     inj alt   inj spd    rx alt   rx spd  sats   C/N0
    51.0  gps FIX          5.00km       510      5.00      510     8   43.5
    52.0  gps BLOCKED      5.00km       520      --       --       8   43.5

VERDICT: position withheld while satellites stayed tracked -- the COCOM signature.
         threshold brackets: speed 510-520 m/s
```

The whole measurement hinges on **BLOCKED vs NO_LOCK**. A COCOM gate withholds
position while satellites stay tracked at healthy C/N0 — `sats` and `C/N0` hold
steady across the transition while lat/lon and fix quality drop. If C/N0
collapses and satellites fall out *first*, that is a loss of lock and says
nothing about the gate. Watch those two columns as closely as the verdict.

`build_scenarios.sh` passes gps-sdr-sim `-p`, holding channel power constant, so
C/N0 stays flat for the whole run and that call is unambiguous. It trades
realism for diagnostic power, deliberately.

## 10. Known traps

- **MAX2839 clones.** Several batches of Chinese H2+ units ship a MAX2839 in
  place of the MAX2837, and GPS Sim is
  [reported non-functional on them](https://github.com/portapack-mayhem/mayhem-firmware/issues/1305)
  while the same HackRF works fine driven from a PC. If t0 will not fix at any
  gain, check the RF chip before you chase levels — and fall back to §11.
- **Loop checkbox.** Unchecked. Every time.
- **Sample rate.** 2.6 MSa/s. Mayhem also accepts 2.5 and 1.25 MSa/s, and 1.25
  would halve both file size and underrun risk — but it clips the 2.046 MHz C/A
  main lobe and costs C/N0, which is the exact margin this test spends to tell
  BLOCKED from NO_LOCK. Not worth it.
- **gps-sdr-sim smashes its stack on long paths.** Every path argument is
  `strcpy`-ed into a 100-byte buffer with no bounds check (`gpssim.c:1824`), and
  a session scratchpad path alone is ~180 characters. It dies as a bare
  `Trace/BPT trap: 5` with no message, which reads exactly like a corrupt
  ephemeris. `build_scenarios.sh` runs from the output directory and passes short
  relative names, and refuses any argument over 90 characters.
- **Do not pass gps-sdr-sim `-T`.** It rewrites every ephemeris relative to the
  file's *earliest* epoch, so on a multi-set daily file it aligns the wrong set
  to your start time — the constellation collapsed from 10 satellites to 1 in
  testing here. `-T` exists to let a stale ephemeris stand in for a current one;
  with a same-day file it is unnecessary, and it is actively harmful. Place the
  start inside the file's own validity window instead, which is what
  `pick_start.py` does.
- **Scope.** This measures the firmware gate only. The injected signal is clean
  and vibration-free; it says nothing about whether boost-phase data is usable,
  which is the separate high-g/transonic problem (#174 / #249 / #262).

## 11. Fallback: drive the HackRF from the Mac

If the PortaPack path fails — MAX2839, SD underruns, anything — the same `.C8`
files play straight out of `hackrf_transfer`, which streams from the Mac and has
none of the SD-card constraints. Everything else in the procedure is unchanged.

```bash
hackrf_transfer -t c8/t1_velramp.C8 -f 1575420000 -s 2600000 -a 0 -x 0
```

`-a 0` amp off, `-x` TX gain 0–47 (the §6 sweep), `-R` repeat — **omit `-R`**,
for the same reason the Loop checkbox stays unchecked.

## References

- [Mayhem GPS Sim wiki](https://github.com/portapack-mayhem/mayhem-firmware/wiki/GPS-Sim) ·
  [app source](https://github.com/portapack-mayhem/mayhem-firmware/blob/next/firmware/application/external/gpssim/gps_sim_app.cpp)
- [gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim)
- [SkyTraq AN0037 — Phoenix binary messages](https://www.skytraq.com.tw/homesite/AN0037.pdf)
- [SkyTraq Commonly Asked Questions](https://www.skytraq.com.tw/Commonly%20Asked%20Questions.pdf) (the AND claim)
- [PX1125R datasheet](https://macrogroup.ru/upload/iblock/809/ugl3zqyphweatu0hjenfjwbxxtsiyt4b/PX1125R.pdf) (the 80 km / 515 m/s wording)


## Testing the rocket computer instead of the bench PX1125R

The rocket computer is a different receiver and nothing measured here transfers
to it. GNSS is on the **ESP32-P4** (GPIO3 rx / GPIO4 tx), and the part is a
**u-blox SAM-M10Q** with an integrated patch antenna and LNA -- the S3 has no
GNSS connection at all. u-blox documents its own limits, near 500 m/s and 80 km,
enforced independently; the "AND" story is specifically a u-blox myth. Re-measure
rather than assume. The scenarios and the whole analysis chain drop straight in.

Two practical notes:

- The FC calls `setUART1Output(COM_TYPE_UBX)`, so **NMEA is off on the wire** and
  a passive tap on the receiver's TXD sees UBX only, at 460800 baud (38400 is the
  module's factory default). `../ublox_binary.py` parses it: UBX-NAV-PVT and
  UBX-NAV-SAT map onto SkyTraq's 0xDF and 0xE7, and `gnss_nmea_monitor.py` feeds
  both into the same FIX / BLOCKED / NO_LOCK classifier. Captures are logged as
  `TS U <hex>` beside the SkyTraq `TS B <hex>`.
- The FC already sets `DYN_MODEL_AIRBORNE4g` and verifies the readback, which is
  the u-blox equivalent of the navigation-mode lever this rig sets on the SkyTraq.

Two traps in the UBX path, both of which land on the measurement and both of
which `ublox_binary.py` handles:

- NAV-PVT's `gSpeed` is **ground** speed. COCOM acts on 3-D speed and a rocket's
  velocity is almost all vertical, so `gSpeed` reads near zero through exactly
  the part of the flight the limit is about. Use velN/velE/velD.
- A fix requires `gnssFixOK`, not just `fixType`. u-blox reports a populated
  `fixType` with that flag clear while it is withholding, which is precisely the
  transition being measured.

### A standalone receiver on its own USB (ArduSimple ZED-F9P)

Unlike the rocket computer's SAM-M10Q, a bench receiver on its own USB is ours
to configure -- and it has to be, because **it ships emitting NMEA only and NMEA
cannot carry this measurement**. GGA has no flag separating "withholding a
solution" from "no solution", and its speed is *ground* speed, near zero through
exactly the part of a rocket flight the velocity limit is about.

    ./ubx_config.py --identify-only        # confirm the part and firmware first
    ./ubx_config.py                        # UBX on, NMEA off, airborne <4 g

**Everything is written to the RAM layer only**, so a power cycle restores
whatever the owner had configured -- this is somebody's surveying receiver, not
bench equipment. The cost is that the configuration must be re-applied after
every power cycle. `ubx_config.py` ACKs and then reads every key back; do not
trust a run whose read-back was not clean.

The ZED-F9P enumerates as its own USB CDC (`1546:01A9`, "u-blox GNSS receiver"),
so the baud rate is irrelevant. No new parser is needed: it is a u-blox, and
`ublox_binary.py` reads NAV-PVT and NAV-SAT from it unchanged.

Note the signal is **GPS L1 C/A only**. A multi-band RTK receiver will therefore
use one constellation on one band, well below what it is capable of -- which is
a property of the injection, not of the part, and applies equally to every
receiver on this rig.

### A plain NMEA receiver on a USB-UART (Air530 / AT6558R)

No configuration is required, and on this unit none is possible: it ignores
`$PCAS01` and stays at 9600. That is fine. **NMEA carries this measurement** --
GGA reporting no fix while GSV still lists satellites *is* the BLOCKED versus
NO_LOCK distinction, and ground truth comes from the injected trajectory rather
than the receiver's own reported speed, so the `gSpeed` trap that forced UBX on
the u-blox parts does not apply.

If nothing arrives at all, `serial_probe.py` sweeps the common bauds and, with
`--loopback`, tests the adapter with its own TX shorted to its RX. That matters
because **software cannot distinguish reversed TX/RX from an unpowered module**
-- both are exactly zero bytes. The loopback splits the path so you know which
side to look at.

Two things to know about this part before reading its numbers:

- Its RMC date is **1024 weeks behind** (2007 instead of 2026), a GPS
  week-number rollover. Time-of-day is correct, so pass `-t` explicitly.
- 9600 baud is *not* the bottleneck it looks like. A full flight capture came
  back with 862 GGA epochs over 862 s at exactly 1.00 Hz and no bad checksums.
  Check before blaming the wire.

### A u-blox M8 (NEO-M8T): different configuration, and a false gate

**`CFG-VALSET` does not exist before protocol 27.** An M8 answers it with a NAK
or with nothing, which reads exactly like a wiring fault. `ubx_config.py` now
reads PROTVER from MON-VER and switches to legacy `CFG-MSG` / `CFG-NAV5`
automatically; it prints which path it chose.

    ./ubx_config.py -p /dev/cu.usbserial-0001 -b 115200

**Beware a ceiling that is not COCOM.** The NEO-M8T stops publishing position at
50 km, which is *not* an export gate -- it is the u-blox dynamic model. Airborne
&lt;4 g is specified at 50,000 m, and measured at 49.80-50.15 km here. The test
is to change the model rather than the trajectory:

    ./ubx_config.py -b 115200 --dynmodel 0      # portable
    ./run_radiated.py -s t2_altramp -p auto -b 115200 -x 38

The ceiling moves to ~5 km. An export gate does not track the platform model.
No u-blox model exceeds 50 km and airborne &lt;4 g is already both the highest
ceiling and the highest velocity limit, so an M8's true COCOM altitude behaviour
cannot be measured at all -- the model stops it first. M10 and F9 parts held
fixes at 68.8 km on the same model, so this is M8-generation behaviour.

The general form of the trap: **a receiver that stops publishing is not
necessarily a receiver that has hit the export limit.** Dynamic models,
base-station modes and firmware ceilings all produce the same silence. Change
the suspected cause and watch whether the threshold moves.

### Editing the report

`report.html` is **generated — do not edit it**, it is overwritten on every
build. The words live in `report_text.html`:

    $EDITOR report_text.html
    ./build_report.py

That file is read as plain text and never string-formatted, so braces, quotes
and percent signs in the copy need no escaping. `{{PLACEHOLDERS}}` are filled
from `results/receivers.json` and `results/figures/` — leave them in place and
everything around them is free text. Per-receiver blurbs sit at the bottom
between `<!--#blurb id-->` markers.

`./build_report.py --check` verifies the inputs without writing, and reports a
placeholder that has been deleted, an unknown one, or a figure that is
referenced but missing.

Measured numbers are generated from `receivers.json`, so a figure typed into the
prose will not stay in step with the data — change the JSON instead.

### When a ramp cannot measure the receiver: dwell instead

Every original scenario here is a ramp, and a ramp measures a slow receiver's
**latency**, not its threshold. On a 3 g climb the vehicle spends about one
second within +/-15 m/s of the velocity limit; a receiver that reacts a few
seconds late smears the bracket by hundreds of m/s. The Air530 came back as
"538-1334 m/s" that way, which is not a threshold at all.

The dwell scenarios hold a value steady for far longer than any plausible lag:

    vel_stair       90 s dwells at 495-530 m/s, constant 5 km
    alt_stair       90 s dwells at 76-82 km
    alt_stair_low   90 s dwells at 12-22 km
    alt_stair_vlow  90 s dwells at 8-13 km
    blockdur        excursions of 5/30/150 s, each followed by 155 s clear

`blockdur` separates a slow gate from a receiver that drops navigation state
when gated: a fixed re-convergence cost does not care how long the block lasted,
a slow gate should.

**Watch for a ceiling that is not COCOM.** Two of five parts stop publishing
below the export limit for unrelated reasons -- the NEO-M8T at 50 km (u-blox
dynamic model) and the Air530 at 10-11 km. Both look exactly like a gate. Change
the suspected cause, or dwell below and above the suspected ceiling, before
recording a number as COCOM. `alt_stair_vlow` deliberately crosses **no** export
limit (13 km, 156 m/s), so anything that blocks during it provably is not one.

Note `align_start.py` cannot align a constant-altitude scenario -- there is no
altitude signature to match, and it will say so rather than return a confident
wrong answer. Use the `start_time` the build records in the scenario JSON.

### Finding the level for a new receiver

    ./gain_sweep.py -p /dev/cu.usbmodem101 -s t00_static

Never carry a gain over from another part. The PX1125R locked solidly at TX gain
22 and *never* fixed at 32 -- its front end overdrives -- and when a bad
connector later cost ~25 dB the working point moved to 47. **The tell for
overdrive is higher C/N&#8320; with fewer satellites**, so the sweep ranks by
satellites tracked and uses C/N&#8320; only to break ties, and says so explicitly
when the hottest setting is not the best one.

### Radiated test in a Faraday cage

`run_radiated.py` transmits a scenario and captures the receiver through a
passive tap. `--listen-only` checks the tap without transmitting; `--list` shows
candidate ports. It never writes to the receiver: the rocket computer owns that.

**No tap is needed if the FC is built with the diagnostic.** The FC's u-blox is
otherwise unreachable -- its firmware sets UART1 output to UBX only and the
rocket computer consumes it, putting nothing raw on USB, and the console reports
GNSS fix state only under `[SIM DIAG]`, which fires in simulation mode alone.
Worse, the driver polls NAV-PVT and takes the satellite count from `getSIV()`;
it never asks for NAV-SAT, so per-satellite C/N0 -- the input the whole
BLOCKED-vs-NO_LOCK distinction rests on -- does not exist in its data flow.

    idf.py -B build_cocom -DTR_BOARD_V8=1 -DTR_GNSS_COCOM_DIAG=1 build flash

adds ~1.5 kB and logs, once a second:

    I (25263) GNSS: [COCOM] P tow=24805 fix=0 ok=0 nsv=0 lat=0 lon=0 alt=0 vn=0 ve=0 vd=0
    I (25263) GNSS: [COCOM] S n=2 0:22:20:0 5:1:10:0

`cocom_fcdiag.py` converts a captured console log into the rig's `TS U <hex>`
format, so `correlate.py`, `recovery.py`, `plot_flight.py` and `oscillation.py`
all work unchanged against the same classifier the conducted rig used. The flag
defaults off and the flight path neither sets nor needs it.

`run_fc.py` drives that whole path in one command -- switch the PortaPack into
HackRF mode, transmit, record the console, convert it:

    ./run_fc.py -s gentle_alt -x 12          # -p defaults to /dev/cu.usbmodem101

It refuses to start unless `hackrf_transfer` is actually streaming, because a
run where the transmitter failed to open the device looks exactly like a run
where the receiver heard nothing.

**Recover the scenario start time from the capture, do not trust a note.**
`build_scenarios.sh` now records `start_time` into `scenarios/NAME.json`, but
captures predating that need `align_start.py`, which scans whole-minute
candidates for the one that best matches reported altitude to injected altitude:

    ./align_start.py captures/fc_spaceshot.log -s scenarios/spaceshot.json

The median residual is the check -- tens of metres when the alignment is right,
over a kilometre when the capture is not that scenario. This matters because a
wrong start does not fail loudly: `correlate.py` still prints a full table, just
aligned against the wrong part of the trajectory.

L1's quarter wave is **47.6 mm**, not the 83 mm of a 900 MHz whip. Free-space
loss at L1 is only 26 dB at 30 cm, so with the separation fixed by the box, TX
gain is the only live adjustment -- and with 100 dB of pad it runs out:

    separation   path loss   TX gain for -130 dBm at the patch
       0.20 m      22.4 dB     39      (8 dB of headroom)
       0.30 m      25.9 dB     43      (4 dB)
       0.40 m      28.4 dB     45      (2 dB)
       0.50 m      30.4 dB     47      (at the limit)
       1.00 m      36.4 dB     53      (unreachable)

So **keep the separation at or under about 0.4 m with 100 dB**, or drop to 90 dB
if the cage forces more. Two things push the other way and are worth knowing
before adding pad: the `.C8` files are amplitude-boosted by `boost_c8.py`, worth
about +5 dB, and the receiver tolerated a wide range of levels on the conducted
bench. Sweep gain and let C/N0 decide, exactly as in section 6.

Cage-specific traps, none of which the conducted setup has:

- **Cable leakage.** USB cables through the wall carry RF out; the cage is only
  as good as its feedthroughs. Bulkhead connectors, or clamp-on ferrites at the
  wall on both sides. Verify with a phone's GPS just outside the closed cage
  while transmitting -- if its position or C/N0 moves, the cage is not closing.
- **Multipath.** A metal box is a resonant cavity. Standing waves give deep
  nulls, and moving the receiver a few centimetres can swing C/N0 by 20 dB --
  worse than the ~15 dB bench oscillation documented in the report, and easily
  mistaken for it. Absorber around the receiver, and once a working position is
  found, mark it and leave it alone.
