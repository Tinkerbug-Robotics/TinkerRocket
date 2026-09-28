# Bench record, 2026-08-19

Raw receiver output from the runs the report cites, plus `results.json` (the
batch runner's per-run summary and correlator output). Gzipped because they are
kept as evidence, not working files.

To re-analyse a run:

```bash
gunzip -c results/t1_velramp_a1.log.gz > /tmp/t1.log
python3 correlate.py -s scenarios/t1_velramp.json -t 2026/08/19,22:30:00 /tmp/t1.log
```

`scenarios/` is gitignored; regenerate it first with `python3 make_trajectories.py`.
The scenario definitions are deterministic, so the ground truth these captures
were measured against is reproduced exactly.

Scenario start for every capture here: **2026/08/19,22:30:00**.
Ephemeris: BKG `BRDC00WRD_R_20262310000_01D_MN.rnx`, converted to RINEX 2.
TX: HackRF One r9 clone via PortaPack in HackRF mode, gain 44-47, ~70 dB pad,
DC block, conducted into RF_IN. Receiver: PX1125R on a dual-MCU TinkerNav via
the RP2040 USB port, SkyTraq binary output, warm-restarted and seeded per run.

| Capture | Only thing exceeded | Result |
|---|---|---|
| `t0_baseline_a3` | nothing | control -- never blocked |
| `t1_velramp_a1/a2/a3` | velocity | blocked 510 -> 520 m/s at 5.00 km |
| `t2_altramp_a1/a2/a3` | altitude | blocked 79.55 -> 79.90 km at 354 m/s |
| `t3a_both_18km_a1` | both | blocked 510 -> 517 m/s at 16.22 km |
| `t3b_both_80km_a1` | both | blocked 79.90 -> 80.20 km at 366 m/s |

Flight profiles from `make_flights.py`, flown 2026-08-20 to measure how fast the
gate re-opens. Same start time and RF setup. The scenario each was flown against
is archived beside it as `*_v2.scenario.json` -- a capture means nothing against
a regenerated scenario, and `plot_flight.py` now refuses the mismatch.

| Capture | Boost | Result |
|---|---|---|
| `gentle_alt_v2` | 3 g | altitude gate re-opened **0.7 s** after descending below 80 km, with 6 satellites |
| `spaceshot_v2` | 15 g | velocity gate re-opened **1.2 s** and **1.5 s** in two separate windows, with 6 and 4 satellites |
| `spaceshot_v3` | 15 g | flown after the SMA was re-seated; 7 satellites held at 48-50 dBHz straight through the burn, gate re-opened in 1.0 s and 1.5 s |
| `spaceshot_eq` | 15 g | equator / 08:30 / complete-day ephemeris, 14 SV transmitted. Never fell below 4 satellites; all three windows recovered in 1.0-1.5 s |
| `spaceshot_horizon` | 15 g | equator 08:30 with the horizon patch: 15 SV transmitted at altitude, median 8 tracked, recoveries 1.5 / 0.0 / 1.2 s |
| `spaceshot_slr_pwr` | 15 g | same profile with nav mode SLR (0x64/0x17 mode 9) and power mode Normal (0x0C) instead of airborne + default power save |

Every window where four or more satellites were tracked re-opened in 0.7-1.5 s,
on both gates. Windows that took 12-33 s all had two satellites: that is
re-acquisition, not the gate. `recovery.py` prints the satellite count beside
the latency so the two cannot be confused.

Superseded and deleted: the first flight captures (`*_a1`). Their trajectories
deployed the main parachute at 3 km while still doing 560 m/s, which produced
~7000 m/s^2, reversed the velocity in an explicit integrator, and lofted the
vehicle back to 28 km. Both the flights and the tracking losses in them were
artefacts. The profiles now fly a drogue from apogee and the main at 600 m AGL.


## The 15 g tracking claim, retracted

An earlier version of this record said a 15 g boost breaks the tracking loops,
on the grounds that 15 g is 787 Hz/s of Doppler rate and the first flights shed
five of seven satellites at ignition. `spaceshot_v2` and `spaceshot_v3` disprove it: the same
profile held six and then seven satellites at 47-50 dBHz straight through the
burn, 0 to 1339 m/s, losing none. The fix vanishing partway up is the COCOM gate at
515 m/s, not a lock failure.

What differs is margin. The flights that lost satellites had them at 34-35 dBHz
before ignition; the flight that did not had 49 dBHz. That is the bench RF path,
not the flight. Acceleration has not been shown to matter independently here,
and testing it properly needs a bench that holds a steady C/N0.


## Satellite count is the binding constraint, and it is a free parameter

Re-seating the SMA changed nothing: median C/N0 stayed at 44 dBHz and the run
afterwards spent *less* time above four satellites, 70% against 89%. Moving the
simulated launch site to the equator and the scenario to 08:30, against a
complete-day ephemeris, puts 14 satellites in the sky instead of 10:

    run                       SV tx  med sats  med C/N0  >=4 SV  recovery
    40N 22:30                    10         6        44     89%  1.5 / 19.0 / 1.2 s
    40N 22:30, re-seated         10         6        44     70%  1.5 / 1.0 / 146 s
    equator 08:30                14         7        45    100%  1.5 / 1.0 / 1.2 s

Signal level did not move, so this is redundancy rather than power. Use the
equator and a scanned hour for future runs: `best_geometry.py <nav> <day>
<lats> <hours> [lon] [elev_mask]`, and `make_flights.py --lat/--lon`.


## Satellites near apogee: there was no deficit

Checked because a vehicle at 80 km should see more sky, not less. It does: near
apogee the tracked count is flat or higher (equator run 6 -> 8 -> 7 across the
80 km band). The low counts sit where median C/N0 dips to ~31 dBHz, climbing and
descending fast, not at apogee.

The check did turn up a simulator bug. gps-sdr-sim tests visibility against the
local horizontal plane, but from 82.5 km the horizon is 9.2 degrees below it. And
allocateChannel accepts an elvMask parameter then ignores it, hardcoding 0.0 in
its checkSatVisibility call. `patch_horizon.py` fixes both; at 40N the sim then
transmits 14 satellites at 82.5 km against 11 on the ground.


## The bench carries a ~15 dB, ~80 s C/N0 oscillation

**Explained (2026-09-27): the HackRF's carrier runs 22 Hz off its own code, and the
PX1125R, as configured in August, drove its code with its carrier.** Each channel's
pseudorange walked off the true code at 4.0-4.7 m/s, reached 0.83-0.97 of a chip
(243-285 m), and snapped back as the channel re-acquired, every 65-78 s. A one-chip
correlation peak loses 15-18 dB at that misalignment, and 15-18 dB is what each
channel's C/N0 lost: it follows the channel's code error with a correlation of +0.90 to
+0.98 on all six captures (`saw_static_raw`, `saw_static_boosted`, `gentle_alt_eq`,
`gentle_alt_eq2`, `spaceshot_eq`, `spaceshot_horizon`). The elimination below missed it
because it compared the transmitter's clock with the host's, not the carrier with the
code. The satellite-count dips, and with them the PX1125R's 12-33 s and 146 s
recoveries, came from the rig. See [the next section](#the-hackrfs-carrier-runs-22-hz-off-its-own-code-2026-09-27).

The original analysis, kept as it was:

Asked what reduced the satellite count in the 3 g run when the 15 g run at the
same site and hour held 15 satellites throughout. The answer is neither flight.

Reported C/N0 oscillates by about 15 dB with a period near 80 s, and every
satellite-count dip in every flight sits in one of its troughs -- the epochs
below four satellites sat 14 dB and 38 dB under their own run's median. Cornered
by elimination:

  flight dynamics       static scenario, stationary        still present
  boost acceleration    3 g and 15 g prologues (identical) identical sawtooth
  amplitude clipping    12.6% vs 0.04% clipping            same 15 dB swing
  transmit clock        GPS time vs host over 200-800 s    agree to test resolution
  injection level       TX gain 22 / 38 / 47               present at all

`saw_static_raw` and `saw_static_boosted` are the clipping test pair. What
remains is the receiver's AGC or C/N0 estimator, or something subtler in the
chain: localised, not identified.

It does not touch the gate results. Every threshold and latency was measured
across a transition with satellites tracked either side, which is what the
classifier requires before calling anything.

## The HackRF's carrier runs 22 Hz off its own code (2026-09-27)

Every receiver on this rig sees a carrier that disagrees with its code.
Code-minus-carrier (pseudorange minus carrier phase, per satellite) grows at
+4.19 m/s, so the carrier sits 22.0 Hz above where its own code says it should be.
`code_carrier.py` reads it from any raw capture, with no truth or ephemeris needed:

| Receiver | Capture | Code-minus-carrier |
|---|---|---|
| PX1105R, power normal | `px1105r_gentle_alt_pad600_smooth_gain3_nav9_el3_pmnormal_run2` | +4.185 m/s (59 arcs) |
| PX1125R, September setup | `px1125r_spaceshot_pad600_smooth_gain2_nav9_el3_pmnormal_flight` | +4.189 m/s |
| NEO-M8T, 2026-08-20 | `neo_m8t_gentle_alt`, `neo_m8t_spaceshot` | +4.186, +4.188 m/s |
| LC86G, MSM7 | `lc86g_balloon_gentle_alt`, `lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7` | +4.176, +4.176 m/s |
| PX1105R, real sky | the two static captures kept outside the repo (window, outdoor) | -0.011, +0.002 m/s |

Against the truth on the pad it is the same split: the clock drift the Doppler
reports runs 3.9-4.5 m/s below the rate at which the pseudoranges' clock bias moves,
on 15 PX1105R pads, on the PX1125R with every IQ build (stock, float, fixed-point
ramp, smooth) and on the NEO-M8T. The PX1105R's own solution (0xDF) carries the same
two clocks, 3.7-4.4 m/s apart (median 4.2). The LC86G takes its own clock drift out
of its MSM7 Doppler, which is why its pseudorange bias was seen to "grow ~4 m/s"
(`msm7_clock.py`): its carrier phase shows the same -4.3 m/s against its code.

**Where it comes from: the HackRF, not gps-sdr-sim.** gps-sdr-sim derives each
channel's code rate from the same range difference as its carrier (`f_code =
CODE_FREQ + f_carr / 1540`), and a static file it writes is code/carrier-consistent
to 0.08 m/s on four satellites, measured from the IQ. Every runner transmits with
`hackrf_transfer -f 1575420000 -s 2600000` and no crystal correction. By the
firmware's arithmetic the HackRF's 2.6 MHz sample clock is an exact fraction of its
reference (no rounding in the divider), while its 1575.42 MHz LO is synthesized in
fractional-N steps; a reconstruction of that tuning arithmetic puts the carrier
+17 Hz high, the same sign and scale as the measured +22.0 Hz. The exact figure
depends on the firmware in the PortaPack's HackRF mode, which was not read. It was
the same from 2026-08-20 to 2026-09-27.

**What it does to receivers:**
- **A receiver that drives its code with its carrier walks off the code** at
  4.2 m/s: the August PX1125R, 0.83-0.97 chip and 15-18 dB of C/N0 (the section
  above); in September the PX1125R in both power modes and the PX1105R in power save,
  ramps of 40-130 m. The PX1105R in power normal, the NEO-M8T and the LC86G track the
  code and show none.
- **Carrier-smoothed pseudoranges restart off-level at every (re)lock.** Smoothing
  with time constant tau holds a settled channel r x tau below its raw code (r =
  4.19 m/s). A channel that has just locked starts at the raw code, so it reads high by
  up to r x tau and settles over tau. PX1105R in flight: +7 to +15 m median at lock,
  +17 to +30 m in the upper quartile, settling over 10-20 s. The PX1105R on the real
  sky shows no systematic offset (medians within 3 m, scatter 5-9 m); the NEO-M8T
  shows none on the rig.
- **The receivers' own altitude:** on the LC86G, yes -- its 2.5 km descent drift is
  gone on the corrected file (below, and open question 4).
- **Not the own velocity:** its bias is 0.00 m/s on the pad on all four receivers,
  so the speed gate and every velocity result are unaffected. **Not acquisition:**
  22 Hz against the 1.1-2.0 kHz clock offsets these receivers already search.
- **Not the ~15 s SkyTraq collapses,** as far as the data can say: averaged over every
  collapse, the channels' code shows no re-alignment at them, and the PX1125R's
  power-normal flight collapses 25 times while most of its channels track the code.

**The fix, built and checked offline:** `patch_carrier_offset.py` adds an opt-in
`-DCARR_OFFSET_HZ` to gps-sdr-sim that offsets the carrier alone. Built with -22.0
it writes every carrier 22.0 Hz low and leaves the code untouched (from the IQ: carrier
-22.01 to -22.03 Hz, code drift under 0.0001 chip/s); built without it, the patched
source reproduces `spaceshot_pad600_stock.C8` byte for byte. On air the HackRF's
+22.0 Hz should bring the carrier back onto the code, and `code_carrier.py` should
read about 0.

**Checked on air (2026-09-27): it does, and the LC86G's altitude drift goes with it.**
Same receivers, same settings, same trajectory; only the IQ file changed, to the
`*_cofs.C8` built with `-DCARR_OFFSET_HZ=-22.0`:

| Receiver | Run | Code-minus-carrier, original -> corrected | Own height against the truth, original -> corrected |
|---|---|---|---|
| NEO-M8T, conducted, GPS at 18 Hz | 330 s of the `gentle_alt_pad600_smooth` pad | +4.188 -> +0.002 m/s (14 arcs each) | -16 -> -17 m on the pad |
| LC86G, Balloon, radiated | the whole `gentle_alt_pad600_smooth` flight | +4.176 -> +0.001 m/s (37 and 34 arcs) | the table below |

The LC86G's own fix against the injection, from `own_fix.py` (median, and the 5-95 %
spread; fixes from the first 30 s after the first fix left out):

| Phase | Height, original | Height, corrected | East, original | East, corrected |
|---|---|---|---|---|
| Pad | -123 m (-136 to -108) | -11 m (-12 to -10) | -57 m | -1 m |
| Boost, 3 g | -196 m (-229 to -149) | -17 m (-47 to -12) | -67 m | 0 m |
| Coast, 69-80 km | -727 m (-861 to -618) | +2 m (+2 to +3) | -306 m | +1 m |
| Descent | -2.5 km (-2.7 km to -45 m) | -6 m (-14 to 0) | -1.8 km | -1 m |

- **The drift was the rig.** Corrected, the LC86G's altitude stays within 14 m of the
  injection on 96 % of its fixes, where the original ran 2.3-2.6 km low from 20 km down
  to 2 km. The westward drift goes with it: east within 3.4 m (5-95 %), against 1.8 km.
  The worst corrected fixes are transients: 48 m low 6 s into the 3 g boost, and 60 m
  low just after the model's main-deploy step (open question 3).
- **Its 80 km mute falls on the injection's 80 km.** Silent from 740.6 to 786.5 s,
  against injected 80 km crossings at 740.7 and 786.4 s; last fix 79.985 km own and
  79.983 injected. On the original file it went silent at 80.87 km injected (own
  79.998). The speed mutes are the same on both files to the 0.1 s epoch (630.2-711.4 s
  and 816.0-875.9 s). `receivers.json` now takes the LC86G's altitude bracket from this
  run, 79.98-80.01 km, so the report and the table above say 80 km where they said 81.
- **Level:** 43-45 dB-Hz against 36 in the original run, at the same TX gain. The files
  carry the same level (the NEO-M8T, on its cable, reads 48.0 dB-Hz on both), so that is
  where the board sat in the cage. Neither run had an underrun.
- **What remains follows altitude:** -11 m on the pad, -13 m at 2 km, -9 m at 5 km,
  -5 m at 10 km, -3 m at 20 km, 0 to +3 m above 69 km. The NEO-M8T's -16 to -17 m on
  the pad did not move with the correction either. That is the shape of a troposphere
  delay taken out that was never put in: gps-sdr-sim adds an ionospheric delay
  (`ionosphericDelay` in `gpssim.c`) and no tropospheric one, and a receiver's
  troposphere model shrinks with altitude the same way. Inferred, not tested.
- **Still to run:** the PX1125R's pad on `spaceshot_pad600_stock_cofs.C8`.

Captures: `results/lc86g_20260927_gentle_alt_pad600_smooth_cofs_balloon_msm7.log.gz`
(against `lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7`) and
`results/neo_m8t_20260927_gps_18hz_gain14_gentle_alt_pad600_smooth_cofs_pad.log.gz`
(against `neo_m8t_20260927_gps_18hz_gain14_gentle_alt_pad600_smooth`).

## u-blox SAM-M10Q, radiated (2026-08-20)

Flown against the same two trajectories to separate what is a rule from what
is one vendor's choice. Different part,
different protocol, different signal path.

Scenario start for these captures: **2026/08/18,08:30:00** -- but do not take
that on trust, recover it:

```bash
gunzip -c results/ublox_m10_gentle_alt.log.gz > /tmp/g.log
python3 align_start.py /tmp/g.log -s results/ublox_m10_gentle_alt.scenario.json
```

The scenario each was flown against is archived beside it, `start_time` included.
TX: HackRF One r9 via PortaPack in HackRF mode, **gain 12**, 100 dB pad, L1
quarter-wave, inside a Faraday cage. Receiver: SAM-M10Q, configured by its host firmware
(`DYN_MODEL_AIRBORNE4g`) and never written to by the rig. `.console.log.gz` is the raw host console -- the primary record;
`.log.gz` is `cocom_fcdiag.py`'s conversion of it.

| Capture | Boost | Result |
|---|---|---|
| `ublox_m10_spaceshot` | 15 g | 3 windows, all recovered: 0.6 / 0.0 / 0.4 s. 618/795 epochs with a fix; satellites never below 4 |
| `ublox_m10_gentle_alt` | 3 g | 3 windows, all recovered: 0.3 / 0.7 / 1.2 s. 646/824 with a fix; **the run that brackets the velocity gate**, min 9 satellites |

Taken together the two flights corner the velocity gate between **514 and
515 m/s** and the altitude gate at **~80.16 km** (514 and 516 m/s as first
published, against truth velocities that read 0.05 s late; the 515 is a re-open
edge, so it carries that part's latency). The 15 g flight alone could only say
506-623 m/s: at 1 Hz a 15 g boost covers 117 m/s between epochs, so
the slow ascent is what does the measuring.

## ZED-F9P, conducted (2026-08-20)

ArduSimple simpleRTK2B, standalone on its own USB CDC (`1546:01A9`), fed through
the same 70 dB pad the PX1125R used. TX gain **38**, found by `gain_sweep.py`.
Configured by `ubx_config.py` (UBX on, NMEA off, airborne <4 g).

**It arrived configured as a fixed-position RTK base** (`CFG-TMODE-MODE=2`) and
therefore did not navigate at all. Every symptom looked like an RF problem and
none of it was: GPS tracked at a median 41 dBHz with valid ephemeris, 335 of 420
epochs had four or more usable satellites, and `used_in_fix` stayed 0 while the
reported position sat on the unit's surveyed base coordinates in New Jersey.
Cold-starting did not help, because the position was configuration rather than
retained state. `ubx_config.py` now reads `CFG-TMODE-MODE` first and says so.

The config has since been written to **BBR+Flash**, so the unit comes up
navigating; its original base-station setup was overwritten, not shadowed.

| Capture | Boost | Result |
|---|---|---|
| `zed_f9p_spaceshot` | 15 g | 3 windows, recovered 0.5 / 1.0 / 0.2 s. 590/805 epochs with a fix |
| `zed_f9p_gentle_alt` | 3 g | 3 windows, recovered 0.3 / 0.7 / 1.0 s. 517/812 with a fix |

Velocity edges bracket **(514, 518] m/s** as first published. Against the retimed
truth (its velocities had read 0.05 s late) the 13.5 g closing edge held a fix at
518 m/s, where one epoch spans 117 m/s, and a re-open edge was withheld at 517, so
the bracket inverts to (518, 517]: latency and a coarse epoch, not a lower
threshold. The altitude bracket *inverts* too --
80.48 km held a fix on one flight and was blocked on the other -- because this
part lags **+2.3 s and +3.3 s on closing** the altitude gate, 400-600 m of
overshoot at climb speed. Its descending edges are crisp and agree at ~80.1 km.

## Air530 / AT6558R, conducted (2026-08-20, re-tested with dwell scenarios)

GPS + BeiDou, NMEA 0183 at 9600 on a CP2102, 70 dB pad, TX gain 32.

**The first write-up of this part was wrong in every particular and the dwell
scenarios corrected it.** It was recorded as having a latent COCOM velocity gate
smeared over 539-1339 m/s with recoveries of 31-134 s. It has no velocity gate at
all, and an altitude ceiling far below any export limit.

Ramps cannot measure a receiver that reacts slowly: on a 3 g climb the vehicle
spends ~1 s within +/-15 m/s of the limit, so a few seconds of lag smears the
answer by hundreds of m/s and what comes back is the latency. Dwelling fixes it.

| Capture | What it shows |
|---|---|
| `air530_vel_stair` | 90 s dwells at 495-530 m/s, 5 km: **100% fix at every level** |
| `air530_t1_velramp` | 0-900 m/s at 5 km: **held a fix throughout**, reported 899 m/s |
| `air530_blockdur` | **fix held at 560 m/s for 148 continuous seconds** |
| `air530_alt_stair_vlow` | 90 s dwells: 100% at 8 km, 97% at 9, 91% at 10, **0% at 11/12/13** |
| `air530_alt_stair_low` | 90 s dwells at 12-22 km: zero fixes anywhere |
| `air530_alt_stair` | 76-82 km: 726 epochs, zero fixes, median 10 satellites |
| `air530_t2_altramp` | 354 m/s constant: fix at **9.90 km**, blocked at **10.25 km** |
| `air530_t3a_both_18km` | 394 m/s: fix at **10.10 km**, blocked at **10.44 km** |

**No velocity gate. An altitude ceiling at 10-11 km, which is not COCOM** -- it
sits far below the export altitude, and `alt_stair_vlow` (13 km, 156 m/s)
crosses no export limit at all.

The flights misled because a rocket crosses 10 km fast, so the ceiling fires at
almost the same moment a velocity limit would. The tell was in the data:
on `spaceshot` the fix stops while speed is **falling**, 1339 -> 1302 m/s, as
altitude rises through 9.83 km. No velocity gate fires on decreasing speed.
The "recoveries" were the vehicle descending back through the ceiling, and the
two windows that "never recovered" clear at 68-80 km, far above it.

**The 18 s C/N0 blanking** is confined to intervals where the receiver is
withholding -- 19/677 epochs while withholding vs 0/170 while publishing on
`gentle_alt`; 26/413 vs 4/419 on `spaceshot`; and on `blockdur`, where it holds a
fix almost throughout, 2 events in 988 epochs. Note this cannot be measured with
`verdict()`: a blank epoch has every C/N0 at zero, so its satellite count is zero
and the verdict is forced to NO_LOCK. Use the fix flag (`blanking.py` does).

For flight use this is the worst of the five: it stops publishing at
10 km, below apogee for most high-power flights, for reasons unrelated to export
control and with no documented threshold to design around.

## u-blox NEO-M8T, conducted (2026-08-20)

M8 generation, **PROTVER 22.00**, FWVER `TIM 1.10`, UBX at **115200** on a CP2102.
TX gain **38**. Configured by `ubx_config.py`, which now detects the generation
from PROTVER and uses **legacy CFG-MSG / CFG-NAV5** -- `CFG-VALSET` only exists
from protocol 27 (F9/M9) and an M8 answers it with a NAK or with nothing, which
reads exactly like a wiring fault.

| Capture | Scenario | Result |
|---|---|---|
| `neo_m8t_gentle_alt` | 3 g flight | velocity gate 511 -> 525 m/s; w3 recovered 1.0 s; w1/w2 never, because they clear above 50 km |
| `neo_m8t_spaceshot` | 15 g flight | same pattern; w3 recovered 3.2 s at 28 km |
| `neo_m8t_t2_altramp` | 85 km at 354 m/s | **fix at 49.80 km, none at 50.15 km** |
| `neo_m8t_t2_altramp_portable` | same, portable model | **ceiling moves to 5.04 km** -- the control |

**The 50 km ceiling is the u-blox dynamic model, not COCOM.** Airborne <4 g is
specified at 50,000 m and measured here at 49.80-50.15 km with 14 satellites
either side. The control run proves it: changing only the platform model, from
airborne <4 g to portable, moved the same ceiling to 5.04 km. An export gate does
not track the platform model.

It is recorded in the table anyway, labelled `(dyn model)`, because a flight
computer does lose position above 50 km with this part fitted. But it is not a
fifth altitude threshold to set against four independent measurements of 80 km,
and its true COCOM altitude behaviour is **unmeasurable** -- no u-blox model
exceeds 50 km, and airborne <4 g is already the highest ceiling and the highest
velocity limit available. The SAM-M10Q and ZED-F9P held fixes at 68.8 km on that
same model 8, so this is M8-generation behaviour.

Its velocity gate sits far below the ceiling and is therefore measurable:
**511-525 m/s**, the same limit as every other part.

## Receivers compared

Generated from `results/receivers.json` by `receiver_table.py`. The whole of
`report.html` is likewise generated, by `build_report.py`, from that same JSON
plus the archived figures in `results/figures/`, and so is its companion on boost
dynamics, `boost_report.html`. Edit the data and regenerate --
neither table nor report should be hand-edited.

| Receiver | Path | Update rate | Velocity gate | Altitude gate | Limits combined | Re-open latency |
|---|---|---|---|---|---|---|
| SkyTraq PX1125R | conducted | 1 Hz | 510-517 m/s | 80 km | independent | 0.0-1.5 s |
| u-blox SAM-M10Q | radiated, Faraday cage | 18 Hz | 514-515 m/s | 80 km | independent | 0.0-1.2 s |
| u-blox ZED-F9P (ArduSimple) | conducted | 1 Hz | 517-518 m/s * | 80 km † | independent | 0.2-1.0 s |
| Air530 (AT6558R) | conducted | 1 Hz | none to 900 m/s | 10 km ‡ | n/a -- no velocity gate | n/a |
| u-blox NEO-M8T | conducted | 1 Hz | 511-525 m/s | 50 km § | independent | 1.0-3.2 s |
| Quescan M10 | radiated, Faraday cage | 1 Hz | 511-518 m/s * | 80 km † | independent | 0.2-5.0 s |
| Beitian BN-182 | radiated, Faraday cage | 1 Hz | 497-511 m/s ¶ | 80 km † | independent | 0.5-11.3 s |
| Quectel LC86G, Balloon mode | radiated, Faraday cage | 10 Hz | 499.9-500.9 m/s ‖ | 80 km ‖ | independent | 0.0-0.6 s |

† Slow to close: this part held a fix 2-3 s past the limit on both flights, about 400-600 m of overshoot above 80 km with position still being published. The threshold itself is normal.

* Inverted bracket: one edge held a fix at a higher speed than another edge withheld. One epoch at 13.5 g spans ~117 m/s, a part slow to close or re-open holds a fix past the limit or stays blocked below it, and a single edge can close an epoch early, so this is latency and epoch width, not a threshold below 515. The per-edge midpoints sit near 515.

‡ Not an export gate. This ceiling sits below the COCOM altitude, and the receiver stops publishing there for reasons unrelated to export control.

§ The u-blox dynamic model's own altitude ceiling, not an export gate. Airborne <4 g is specified at 50,000 m; no u-blox model goes higher, so this part's export behavior above it cannot be measured.

¶ Rests on a single closing edge, so it is bracketed only to the width of one navigation epoch. This part was slow enough to re-open that the gate had not cleared before the next window, leaving no fix to close again.

‖ Enforced by muting ALL output -- NMEA, acknowledgements and raw measurements -- rather than by withholding the position while satellites are still reported, so on the wire it looks like a dead receiver until it comes back. It acts on the receiver's own estimates, 500 m/s and 80.0 km, stopping within 0.1 s of passing either and returning within 0.1 s straight into a valid fix. On the carrier-corrected file its own altitude is right and the mute falls on the injection's 80.0 km as well; on the original files it read about 0.9 km low near 80 km, which put the limit near 81 km.

**SkyTraq PX1125R** (2026-08-19, ~70 dB pad + DC block into RF_IN, TX gain 44-47): Satellite starvation was the dominant confound: windows that took 12-33 s all had two satellites, which is re-acquisition rather than the gate. The starvation came from a ~15 dB, ~70-80 s C/N0 oscillation that was the rig, not the part (identified 2026-09-27): in its August configuration this receiver drove its code with its carrier, and the HackRF's carrier runs 22 Hz off its code, so every channel's code walked 0.83-0.97 chip off the correlation peak and re-acquired about every 70 s.

**u-blox SAM-M10Q** (2026-08-20, 100 dB pad, L1 quarter-wave in Faraday cage, TX gain 12): Never fell below 4 satellites in either flight, so every withheld epoch is the gate rather than a link failure. No periodic C/N0 oscillation appeared (r = 0.02 and 0.11).

**u-blox ZED-F9P (ArduSimple)** (2026-08-20, 70 dB pad, TX gain 38): Arrived configured as a fixed-position RTK base (CFG-TMODE-MODE=2) and therefore did not navigate at all: it tracked GPS at a median 41 dBHz with valid ephemeris, had four or more usable satellites in 335 of 420 epochs, and still reported used_in_fix=0 while holding its surveyed base coordinates. Disabling base mode fixed it immediately. It is slow to CLOSE the altitude gate -- +2.3 s and +3.3 s across the two flights, about 400-600 m of overshoot at climb speed -- which is why its altitude bracket inverts. It was the only one of the first three parts to do so; the Quescan M10, measured later, closes as slowly (+2.3 s). Its descending edges are crisp and agree at ~80.1 km.

**Air530 (AT6558R)** (2026-08-20, 70 dB pad, TX gain 32): EVERYTHING FIRST RECORDED FOR THIS PART WAS WRONG, and dwell tests corrected it. It has NO velocity gate: it held a fix to 900 m/s at 5 km on t1_velramp (reporting 899), and 100% of epochs at every 90 s dwell from 495 to 530 m/s on vel_stair. What it has is an ALTITUDE ceiling at 10-11 km -- 100% fix at 8 km, 91% at 10 km, 0% at 11/12/13 km on 90 s dwells, and 9.90->10.25 km on a 354 m/s ramp with 11 satellites either side. That ceiling is far below the COCOM altitude, so it is not an export gate at all. The flight profiles read as a latent velocity gate only because they cross 10 km at high speed: the spaceshot transition happens while speed is DECREASING (1339 -> 1302 m/s) as altitude rises through 9.83 -> 11.15 km, which no velocity gate can do. Re-open latency is not defined for this part because there is no COCOM gate to re-open: on blockdur it held a fix at 560 m/s for 148 continuous seconds, dropping only 1 s at the sharp 130 m/s^2 transition. The 31-134 s 'recoveries' seen on flights were simply the vehicle descending back through the 10-11 km ceiling. The 18 s C/N0 blanking accompanies withholding (19/677 epochs while withholding vs 0/170 while publishing on gentle_alt).

**u-blox NEO-M8T** (2026-08-20, 70 dB pad, TX gain 38): Position is gated at 50 km, but by the u-blox DYNAMIC MODEL rather than by COCOM: airborne <4g is specified at 50,000 m and measured here at 49.80-50.15 km on an altitude-only ramp at 354 m/s. Proved by moving the model -- switching to portable dropped the same ceiling to 5.04 km. No u-blox model goes above 50 km, and airborne <4g is already both the highest ceiling and the highest velocity limit, so this part cannot be made to navigate higher. The ceiling is real for flight use and is recorded as such, but it is NOT an export gate, and its true COCOM altitude behavior is unmeasurable because the model stops it first. Note the SAM-M10Q and ZED-F9P held fixes at 68.8 km on the same model 8, so this is an M8-generation behavior. It also explains what looked like two failed recoveries on gentle_alt: those gaps sit at 68-80 km, above the ceiling, while the window that cleared at 29 km recovered in 1.0 s.

**Quescan M10** (2026-08-28, L1 antenna in Faraday cage, TX gain 26): It answers u-blox's UBX interface down to SEC-UNIQID; its MON-VER reports ROM SPG 5.10, hardware 000A0000, PROTVER 34.10 and no MOD= string. Gate behavior is in family -- velocity around 515, altitude at 80 km, limits independent -- but it is the slowest part measured on the ALTITUDE gate: +2.3 s to close and 4.7-5.0 s to re-open, against 0.7-1.7 s elsewhere, which is why both its brackets inverted. Flown on the same ephemeris, start time and launch site as the SAM-M10Q, ZED-F9P and NEO-M8T, so its satellite geometry is directly comparable rather than merely similar. One velocity edge closed a single epoch early, blocking at 511 m/s, while every other edge on this part is consistent with 515; at the crossing's net 14 m/s^2 an epoch is 14 m/s wide, so on that edge the part stopped a few m/s below 515 by the injection, and one edge cannot say whether its threshold or its own speed estimate is the reason.

**Beitian BN-182** (2026-08-28, L1 antenna in Faraday cage, TX gain 20): It shares the Quescan's UBX interface -- its MON-VER answer is identical, its chip serial differs (dee2c50fbf vs c8bf908e28) -- but it is a different part, flown on the same ephemeris, start time and launch site. It behaves like its MIRROR IMAGE on recovery: fast on altitude (1.0 s) and slow on velocity (10.2 s), where the Quescan is slow on altitude (5.0 s) and fast on velocity (0.2 s). On three of four velocity windows it does not re-open when speed drops below 515 but waits until 327-410 m/s, with 9-13 satellites held throughout, so it is the gate rather than re-acquisition. Transmit level is NOT the cause: a control flight at gain 26, matching the Quescan, reproduced every latency to the tenth of a second (0.5 / 1.0 / 10.2 s) and every shut lag. Besides the part itself, what differs and was not controlled is configuration in the modules' own flash -- this one runs GPS+Galileo+BeiDou with GLONASS off, the Quescan has GLONASS enabled, and CFG-NAVSPG holds more than the dynamic model. The practical lesson is that a shared interface does not predict gate behavior: two modules that answer UBX identically differ by two orders of magnitude on velocity-gate recovery, and no datasheet says which you are buying.

**Quectel LC86G, Balloon mode** (2026-09-24, on-board patch antenna in Faraday cage, TX gain 0): The same module after $PAIR080,3. The boost is tracked (climb rate within 3 m/s of the injection through the 3 g ascent) and the limits are clean and independent: ALL output stops at 500 m/s on its own speed estimate (fired at 9.3 km) and at 80.0 km on its own altitude, and returns within one 0.1 s epoch straight into a valid fix. The altitude bracket comes from the carrier-corrected gentle flight of 2026-09-27, where its own altitude matched the injection to 2-5 m at both edges (last fix 79.98 km, first silent epoch 80.01). On the 2026-09-24 flights its own altitude read about 0.9 km low near 80 km, which put the limit near 81 km against the injection; that was the rig's 22 Hz carrier offset, not the receiver. 500 m/s matches neither COCOM's 515, MTCR's 600 nor the datasheet's 490. At 15 g it still loses every channel at ignition; its RTCM MSM7 shows the four with a Doppler rate at or below 128 Hz/s re-locking within 2 s and every one at or above 171 Hz/s staying lost through the burn, so it has no fix until the descent brings the vehicle back under 500 m/s. On those flights its altitude drifted low, 2.6 km by landing with velocity still right to 0.9 m/s; on the corrected file it stays within 14 m of the injection on the pad, through the coast and down the descent (5-95 %; 48 m low at worst early in the 3 g boost), so the drift was the bench.

Every part that gates velocity at the COCOM figure brackets it within a few m/s of 515 -- the tightest, the SAM-M10Q's, is
**(514, 515] m/s** -- and wherever an altitude gate is genuinely COCOM it sits at
**80 km**, with both limits always independent. The Quectel LC86G stops at
**500 m/s** -- neither COCOM's 515 nor MTCR's 600 -- and does it by muting all
output. The Air530 has no velocity gate to 900 m/s and a 10-11 km ceiling that is
not an export limit. What varies enormously is **re-open latency**: under 1.5 s
on most parts and 0.1 s on the LC86G in Balloon mode, but 5-11 s on the Quescan
and the Beitian. In Normal and Drone mode, which the table leaves out, the LC86G
never re-opened after its 500 m/s mute.

To add another part: fly
`spaceshot` and `gentle_alt`, archive the capture and its scenario here, add an
entry to `receivers.json`, and regenerate. If a part stops publishing at an
unexpected altitude, run `t2_altramp` and then change the dynamic model before
believing it is COCOM.

## Open questions, for whoever picks this up

Things the current data raises and does not settle. All of them are visible in
the archived captures; none needs new hardware.

**1. The Air530 recovers below its ceiling on one flight and not the other.**
On `spaceshot` it regains a fix at ~10.5 km on the way down and holds it solidly
to landing. On `gentle_alt` it never recovers: 251 descent epochs below 10 km,
13 satellites tracked, not one published position. The dwell tests are
unambiguous that it publishes below 10 km when it has *never* been above -- 100%
of epochs at 8 km on `alt_stair_vlow`, and a fix held at 560 m/s for 148 s at
5 km on `blockdur`. So there is a recovery behavior sitting on top of the
altitude ceiling that is not characterized. The test that would settle it: a
scenario that climbs above 10 km, dwells, descends below, and dwells again.
None of the current excursions do that.

**2. The ZED-F9P blocks briefly during steady descent.** 34 in-envelope epochs
across six groups on `gentle_alt`, every one with 10-14 satellites tracked, so
they are genuine withholding rather than signal loss. One group is explained:
twelve epochs at 1.7 km coincide with main-chute deploy, where the injected
trajectory has a -361 m/s^2 spike. The other four groups -- at 22, 12, 7 and
2.9 km -- sit in steady drogue descent at 0.1 to 0.5 m/s^2, with nothing
happening. Unexplained.

**3. That -361 m/s^2 deploy transient is not physical.** It is an artifact of
how `make_flights.py` models canopy inflation, and no real parachute does that.
Any receiver behavior at main deploy on these profiles may be a response to a
transient that could not occur in flight. Worth softening the model before
reading anything into it.

For contrast, the SAM-M10Q and NEO-M8T have **zero** in-envelope blocked epochs
on descent (0/418 and 0/392), and the PX1125R's 57 are all at 2-4 satellites --
the bench C/N0 oscillation, not the receiver.

**4. Every receiver's altitude drifts low on these flight files.** Found checking
the LC86G. On `gentle_alt` its Balloon-mode altitude is 0.05 km low on the pad,
0.9 km low near 80 km, and 2.6 km low at landing -- while its vertical velocity
stays within 0.9 m/s of the injection the whole way, and its east position drifts
west by 1.6 km although the injected track has no east motion at all. The Quescan
M10, a different vendor, drifts the same way on the same file: 0.57 km low near
70 km, 1.5 km low at landing. The trajectory CSV that was transmitted matches the
truth in the scenario JSON exactly (worst difference 0.000 m over 8,478 samples),
so the truth is right; the position solution is being pulled against the Doppler
somewhere between gps-sdr-sim and the receivers. gps-sdr-sim derives each
channel's carrier from the same range difference as its code phase, so the
obvious suspect is ruled out. Unexplained. It moves a gate quoted against the
injection by up to ~1 km near 80 km (a receiver acting on its own altitude, low by
that much, stops late against the truth); every velocity result is unaffected.

**Update (2026-09-27): the measurements are right and the receivers' own filters
drift; the rig's code/carrier split is the leading suspect.** A least-squares fix
from each receiver's own pseudoranges, solved epoch by epoch with its own clock,
stays on the truth while the receiver's fix walks away:

| Capture | Own fix, altitude (east) | Least squares from its own pseudoranges |
|---|---|---|
| `lc86g_balloon_gentle_alt` (stock IQ) | -59 m on the pad -> -2.5 km (-1.5 km) at landing | +2 to +7 m (-1 to -3 m) |
| `lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7` | -133 m on the pad -> -2.7 km (-2.2 km) on the descent, -65 m under the main | +0.3 to +6.5 m (within 2 m) |
| `neo_m8t_spaceshot` | -17 m on the pad, -40 to -370 m under the main | +0.7 m |

It is not every receiver: at 1-10 km on the descent the SAM-M10Q and the Beitian
stay within tens of metres, while the LC86G (2.3 km) and the Quescan (0.85-0.97 km)
drift most. gps-sdr-sim is still cleared; the HackRF was not, and its carrier runs
4.19 m/s off its code ("The HackRF's carrier runs 22 Hz off its own code", above). A
clock propagated with the carrier's drift falls behind the code's, and a receiver
that lets part of that common error into its position moves the fix down: on this
sky 1.85 m of altitude per metre of range the clock does not absorb. The SkyTraq
parts show exactly that on the pad, their own clock-bias state 9-12 m below the
pseudoranges' common bias and their own altitude 8-17 m low. The westward part of
the drift is not explained that way. The test is the A/B on the carrier-corrected
file (`patch_carrier_offset.py`).

**Answered for the LC86G (2026-09-27): it was the rig.** Flown again on the
carrier-corrected file, the same flight keeps its own altitude within 14 m of the
injection on 96 % of its fixes, and the westward drift goes too: east within 3.4 m
(5-95 %). What is left, -11 m on the pad shrinking to 0 to +3 m above 69 km, follows
altitude the way an unmodeled troposphere would. Numbers in "The HackRF's carrier runs
22 Hz off its own code". The Quescan M10 and the SkyTraq parts have not been re-flown.

**5. The LC86G in Normal mode never re-acquires after its mute.** 580 s of 3 g
descent without a valid fix, on the same signal the module tracked at 45 dBHz in
Balloon mode. The likely cause is its own navigation state -- it never believed
the climb, so its predicted Doppler was off by kilohertz -- but that is inferred
from the pattern. The Normal-mode flights had no RTCM MSM7 on; re-flying one with
it would show each channel's measured Doppler against where it should have been.
Drone mode fails the same way, and it did follow the climb. Its GSV lists a
median of 13 satellites at 42 dBHz after its mute, but its MSM7 carries almost
none of them and nothing after 464 s, so it reports signal it never turns into
measurements. Its GSV then runs a strict 12 s cycle: the same 7-9 satellites
leave the list together for one second at 644, 656, 668, 680 and 692 s while the
rest stay, which looks like a search restarting on channels it cannot use.
Question 6 is answered -- Drone mode runs without carrier lock after a cold start
-- but whether that is what keeps it from re-acquiring after the mute is open.

**6. Answered 2026-09-25: Drone mode gives up carrier-phase lock itself; the
3 dB was the bench.** An A/B/A on one static signal, a survey of every mode, and
two sessions on the real sky -- see "Tracking by navigation mode" in the LC86G
section.

## Quescan M10, radiated (2026-08-28)

It **answers u-blox's UBX interface in full**: `ROM SPG 5.10`, hardware
`000A0000`, `PROTVER=34.10`, down to `SEC-UNIQID`, `MON-RF`, `MON-HW`,
`MON-GNSS` and `MON-COMMS` at correct payload sizes, and it reports **no `MOD=`
string**. That establishes the interface, not the part inside it. Found at **38400 baud**, the M9/M10 UART default,
emitting NMEA only. TX gain **26**, off a broad plateau: 12-13 satellites and
44-47 dBHz from gain 14 to 47, no compression at the top.

| Capture | Result |
|---|---|
| `quescan_m10_spaceshot` | 3 windows, recovered 0.5 / 5.0 / 0.2 s |
| `quescan_m10_gentle_alt` | 3 windows, recovered 0.3 / 4.7 / 1.0 s |

Gate behavior is in family -- velocity around 515, altitude at 80 km, limits
independent -- but it is **the slowest part measured on the altitude gate**:
+2.3 s to close and 4.7-5.0 s to re-open against 0.7-1.7 s everywhere else,
which is why both its brackets invert.

**Flown on the same ephemeris, start time and launch site as the SAM-M10Q,
ZED-F9P and NEO-M8T**, so its satellite geometry is directly comparable rather
than merely similar.

### Three receivers, one sky, the same two satellites

| | GPS:11 @ 70 deg | GPS:24 @ 51 deg | r(sin elev, dC/N0) | >=45 deg | <30 deg |
|---|---|---|---|---|---|
| ZED-F9P | 36 -> 0 dBHz | 34 -> 0 dBHz | -0.67 | -35 dB | +10 dB |
| NEO-M8T | (same sky) | (same sky) | -0.45 | -28 dB | -3 dB |
| Quescan M10 | 42 -> 24 dBHz | 39 -> 11 dBHz | -0.44 | -23 dB | -3 dB |

Because the geometry was matched, all three lose **the same two physical
satellites** rather than merely showing three separate correlations. The
acceleration control holds here too: at 2.0 g, r = **+0.31** and >=45 deg at
**+7 dB** -- the effect vanishes, as on the other two.

One velocity edge closed a single epoch early, blocking at 511 m/s while every
other edge on this part is consistent with 515. At the net 14 m/s^2 of that crossing
an epoch is 14 m/s wide (corrected 2026-09-27: this said 29 m/s, the boost's thrust),
so this is not quantization, which can only make a gate look late: an exact 515 gate
would have blocked one epoch later. On that edge the part stopped a few m/s below
515 by the injection, and one edge cannot say whether its threshold or its own speed
estimate is the reason. It is why `receiver_table.py` now estimates the threshold
from the **median of every measured edge** instead of `max(fix)`/`min(blocked)`,
which one sample can drag a whole rounding step.

## Beitian BN-182, radiated (2026-08-28)

**It shares the Quescan's UBX interface but is a different part** -- its MON-VER
answer is identical, its chip serial differs (`dee2c50fbf` vs `c8bf908e28`) --
on another vendor's board, flown on the same ephemeris, start time and launch
site. Found at
**115200 baud**, NMEA only. TX gain **20**.

| Capture | Result |
|---|---|
| `beitian_bn182_spaceshot` | 3 windows, recovered 0.5 / 1.0 / **10.2** s |
| `beitian_bn182_gentle_alt` | 3 windows, recovered **11.3** / 0.7 / **10.0** s |
| `beitian_bn182_spaceshot_g26` | control at gain 26 -- see below |

### One interface, mirror-image recovery

| | w1 velocity | w2 altitude | w3 velocity |
|---|---|---|---|
| Beitian BN-182 | 0.5 s | **1.0 s** | **10.2 s** |
| Quescan M10 | 0.5 s | **5.0 s** | **0.2 s** |

The Beitian is fast on altitude and slow on velocity; the Quescan is the
reverse. On three of four velocity windows the Beitian does not re-open when
speed falls below 515 but waits until **327-410 m/s**, with 9-13 satellites held
throughout, so it is the gate rather than re-acquisition.

**Transmit level is not the cause.** A control flight at gain 26, matching the
Quescan and identical in every other respect, reproduced every latency to the
tenth of a second and every shut lag:

    gain 26    0.5 s   1.0 s   10.2 s
    gain 20    0.5 s   1.0 s   10.2 s

That is worth knowing beyond this part: **re-open latency is not measuring
signal level** on any row of the table.

Besides the part itself, what differs and was not controlled is configuration in
the modules' own flash.
This one runs GPS+Galileo+BeiDou with GLONASS off; the Quescan has GLONASS
enabled; and `CFG-NAVSPG` holds a good deal more than the dynamic model. Dumping
and diffing both modules' `CFG-NAVSPG` and `CFG-SIGNAL` blocks would settle it,
and needs no transmission at all.

**The practical lesson: a shared interface does not predict gate behavior.** Two
modules that answer UBX identically differ by two orders of magnitude on
velocity-gate recovery, and no datasheet says which you are buying.

### Two estimator bugs this exposed

The table first reported **410 m/s** for this part's velocity gate. That was the
estimator averaging *opening* edges -- but on a receiver that takes 10 s to
re-open, the vehicle has shed 180 m/s by then, so those edges measure latency,
not threshold. `receiver_table.py` now uses **closing edges only**, reduced to
per-edge midpoints before the median, which is also robust to the coarse 15 g
crossing where one epoch spans 117 m/s.

Fixing that dropped the Quescan to 510 while the estimator function still said
515 -- because a **second copy** of the rule had been created inside
`receiver_table.py` and only one was updated. That is the same drift already
fixed once between this file and `replot_all.py`. `vel_cell` now delegates to
`velocity_threshold`; one implementation, called from everywhere.

The Beitian's 505 carries a footnote: it rests on a **single closing edge**,
because on the other windows the gate had not re-opened and there was no fix
left to close. One epoch at that boost is 14 m/s wide (corrected 2026-09-27: this
said 29), and the edge, blocked at 511 m/s, sits a few m/s below 515 like the
Quescan's early one, so it is not independently resolved.

### A guard the runner now has

The first attempt at these flights was invalid and would not have looked it.
RAM-layer configuration does not survive a power cycle, and a receiver behind a
USB-UART bridge power-cycles whenever the bridge does -- so between its gain
sweep and its flight the Beitian silently reverted to NMEA with `DYNMODEL=0`,
which is *portable*, ceiling ~12 km. That would have been measured and written
up as a 12 km altitude gate, exactly the NEO-M8T trap, and nothing downstream
would have flagged it. `run_radiated.py` now refuses to transmit unless NAV-PVT
and NAV-SAT are on the wire and NMEA is quiet.

The same reversion also invalidated the gain sweeps that chose the level: they
had counted satellites from NMEA GSV rather than UBX NAV-SAT, which is why
"14 satellites at gain 8" became 4 in flight.

## Quectel LC86G on the Tinker-Beetle, radiated (2026-09-24)

The GNSS on the Tinker-Beetle (rocket-computer-mini, M1): U5, a Quectel LC86G
(LA), firmware `LC86GLANR12A03S` built 2025/04/11, on the flight computer's
own UART. Nothing raw reaches USB in the flight image, so the flight computer
was loaded with `../firmware/lc86_bridge`, a bench image that copies bytes
between that UART and USB-Serial-JTAG, parks the pyro ARM/FIRE outputs low
exactly as the flight image does, and prints `# lc86_bridge: LC86G silent N ms`
whenever the module stops talking. The flight image was backed up first and
written back byte-for-byte afterwards.

**Configured as the Beetle flies it.** `lc86_config.py` replays the flight
driver's `begin()` command for command -- GGA every fix, GSV every 10th,
`$PQTMPVT` and `$PQTMEPE` every fix, 10 Hz -- and reads every setting back.
Nothing is saved to the module's flash. When these flights were made the driver
never sent `$PAIR080`, so the Beetle flew in **Normal** mode, with **Balloon** as
the control; **Drone** mode was added on 2026-09-25 (below). PR #1500 makes `begin()` send `$PAIR080,3` straight after the fix
rate, and the tool now follows it: a plain `./lc86_config.py` leaves the module in
Balloon, and `--navmode 0` puts it back in Normal to repeat these flights.
`--rtcm msm7` adds RTCM3 raw measurements (per-satellite C/N0 to 1/16 dB, the
receiver's own Doppler, carrier-phase lock time, at 1 Hz), parsed by
`../rtcm3.py` and tabulated per channel by `msm_channels.py`.

Radiated in the Faraday cage from the board's own patch antenna, **TX gain 0**,
the lowest the HackRF has. Cage floor with nothing transmitted: zero satellites
in view. Gain sweep on `t00_static` (`results/lc86g_gain_sweep.txt`): 13
satellites at 44 dBHz with a fix at gain 0, fewer at 6 and 12 (each step
restarts the file, so part of that is the clock stepping back). Flown on the
**same `.C8` files as the Quescan and Beitian** (built 2026-08-28 16:32-16:34,
flown from 16:49 that day; scenario JSONs hash-identical to theirs and the
NEO-M8T's). Every flight began with a cold start, `$PAIR006`, which is how a
Beetle comes up on the pad: its V_BCKP rides the same switched rail.

| Capture | Mode | Result |
|---|---|---|
| `lc86g_normal_spaceshot` | Normal | lost every satellite at ignition; valid-flagged fixes 18-80 km wrong near apogee and on the descent; withheld with 13 satellites in GSV until the main |
| `lc86g_normal_gentle_alt` | Normal | climb rate near zero for 18 s of boost with a valid fix; **all output stopped at 500 m/s** (own estimate) for 56 s; no valid fix for the rest of the flight |
| `lc86g_balloon_gentle_alt` | Balloon | boost tracked to within 3 m/s; **silent above 500 m/s and above 80.0 km** (own estimates), each lifted within 0.1 s straight into a valid fix; limits independent |
| `lc86g_balloon_spaceshot` | Balloon | lost every channel at ignition; the four at or below 128 Hz/s re-locked within 2 s, the rest stayed lost; no fix until the descent brought it under 500 m/s, then a valid one within 0.6 s |
| `lc86g_drone_gentle_alt` | Drone | climb rate within 0.5 m/s after the first 2 s of boost, altitude ~1 s late (0.45 km low at 497 m/s); **all output stopped at 500 m/s** (own estimate) for 51 s; no valid fix for the rest of the flight -- GSV lists a median 13 satellites, MSM7 carries none after 464 s |
| `lc86g_drone_spaceshot` | Drone | lost every channel at ignition and none came back in MSM7 until 242 s, then only bursts of 2-7 and none from 452 s; 10 returned with the first fix at 571.4 s, 3.7 s after the descent passed 10 km; that fix and every one after it right |

Each capture's `.runner.txt.gz` holds the preflight read-back that proves the
module's configuration at transmit time.

### Normal vs Balloon

Quectel's protocol specification (LC26G/LC76G/LC86G V1.4, section 2.4.24,
Tables 7 and 8) gives every navigation mode except Balloon a 10 km altitude
limitation, calls 10-50 km "cannot be guaranteed", and stops all output above
50 km; Balloon mode is limited to 80 km. The datasheet adds 490 m/s and 4 g.

- **Normal mode cannot follow a boost.** On the 3 g flight the vertical solution
  lagged from ignition: 1.23 km and 1.4 m/s reported at t=190 s against 2.17 km
  and 191 m/s injected, 4.31 km against 9.18 km at 210 s, all with a valid 3-D
  fix from 13 satellites. The correlator's guard flags 256 such epochs.
- **Both modes mute at 500 m/s on their own speed estimate** -- last output at
  498.7-499.6 m/s own (499.6 injected), first silent epoch 501.0 injected. The
  datasheet says 490; COCOM says 515.
- **Balloon mode also mutes above 80.0 km on its own altitude** (last output
  80.009 km own, 80.94 injected), and returns at 80.016 own. Against the
  injection that is ~81 km; see open question 4 for why the receiver's own
  altitude reads low up there. On the carrier-corrected file (2026-09-27) the mute
  falls on the injection's 80.0 km (last output 79.985 km own, 79.983 injected), so
  the ~0.9 km was the rig.
- **Re-open:** Balloon mode comes back within one 0.1 s epoch straight into a
  valid fix (0.6 s after the 15 g flight's no-fix spell). Normal mode never
  re-opened on the 3 g flight (open question 5).

### Drone mode, and the missing Aviation mode (2026-09-25)

Asked for an aviation mode. Quectel's L26/L76/L86/L96, LC86L and LG77L modules
speak the PMTK protocol, and their specification (Lx6&LC86L&LG77L Series GNSS
Protocol Specification v2.4, 2025-11-07, section 2.3.41, `$PMTK886`
PMTK_FR_MODE) lists 0 Normal, 1 Fitness, **2 Aviation** (high dynamics, large
accelerations weighted in the solution), 3 Balloon and 4 Stationary -- every mode
but Balloon limited to 10 km, Aviation included. The LC86G's `$PAIR080` keeps 0,
1, 3 and 4, marks 2 and 6 **reserved**, and adds 5 Drone and 7 Swimming. The
firmware refuses both reserved values: `$PAIR080,2` and `$PAIR080,6` are answered
`$PAIR001,080,4` (parameter error) and `$PAIR081` still reads the previous mode.
So the third mode flown is **Drone** (5), which the specification describes for
"vertical acceleration at different flight phases" and still limits to 10 km.
`lc86_config.py --navmode` offers 0, 1, 3, 4, 5 and 7 and nothing else. Nor is
there a back door through the PMTK protocol (tried 2026-09-26): `$PMTK886,2`,
`$PMTK886,3` and even `$PMTK605`, the firmware query, are each answered
`$<command>,ERROR,3`, and `$PAIR081` still reads the previous mode.

Same rig, same `.C8` files, same bridge image (the flight computer's MAC checked
before flashing, its flight image backed up and written back byte-for-byte
afterwards), `run_radiated.py --lc86 5 --rtcm msm7 --cold-start`, TX gain 0 in
the sealed cage. The receiver reported **about 3 dB less C/N0** on the pad than
on 2026-09-24 -- 42 dBHz median on GPS GSV and MSM7 against 45-46 -- and **it
never held carrier phase, even on the pad**: MSM7 lock time never passed 10 s
before ignition (Balloon: 176 s by ignition, 754 s by landing on the 3 g
flight), and the
half-cycle flag was set on over 90% of pad cells (Balloon about 1%). Its Doppler
was no noisier -- second difference 0.33-0.34 Hz against 0.38-0.40 -- so this is
not a degraded signal. The tests in "Tracking by navigation mode" below settled
both halves: **the lock loss is the mode, and the 3 dB is the bench** -- no mode
changes the reported C/N0 (so the idea that a mode without phase lock reads 3 dB
low was wrong), and at a fixed setting the level crept up 2.4 dB over the first
hour of transmitting on 2026-09-25.

- **The boost is tracked, but the altitude is a second old.** The climb rate
  lagged by up to 36 m/s for the first 1.8 s after liftoff (Balloon: 13 m/s)
  and then held within 0.5 m/s to the mute. The altitude fell behind in step
  with the climb rate -- 0.18 km low at 191 m/s, 0.45 km at 497 m/s -- and
  shifted by 0.98 s it fits the injection to 11 m rms over 186-210 s (322 m
  unshifted). Balloon's best shift is 0.22 s, the climb rate's under 0.1 s in both.
- **Mutes at 500 m/s** like the other two (last output at 500.1 m/s injected,
  500.0 on its own estimate; first silent epoch at 501.5), at 9.3 km.
- **Never re-opens after the mute** (3 g): output returned at 261.2 s, 51 s
  later, and every `$PQTMPVT` from there to the end of the flight has FixMode 0
  and no satellites used. GSV lists a median of 13 at 42 dBHz all the way down,
  but MSM7 carries none of them after 263 s except three short bursts of 1-5
  cells, and none after 464 s: signal seen, nothing measured. Normal mode's GSV
  listed a median of 4 after its own mute. Open question 5.
- **At 15 g**: every channel lost at ignition; MSM7 carries none until G13 at
  242 s (50 s after burnout; GSV has single-satellite blips before that), where
  Balloon kept its four low ones. With no carrier lock even on the pad -- the
  mode's own weakness, see below -- this is not a clean comparison with Balloon. From 242 s MSM7 has only bursts of
  2-7 cells (GSV lists a median of 8), and none at all from 452 s until 572 s,
  when 10 return with the first fix: 571.4 s, 3.7 s after the descent passed
  10 km (567.7 s), 9.78 km reported against 9.77 injected, and right to landing.
- **No valid-flagged wrong fix** on either flight.

### The ignition loss, per channel

MSM7 on the Balloon flights, against the Doppler rates the NEO-M8T measured on
the same file (`doppler_ref_spaceshot.json`):

    held through the 13.5 g burn   G29 54, G12 98, G30 114 (phase slip), G19 128 Hz/s
    lost for the rest of the burn  G14 171, G15 199, G22 261, G05 286, G20 308,
                                   G06 312, G21 378, G24 501, G11 (steepest) Hz/s

Every channel drops in the first second of the 13.5 g ignition, low and high
alike -- a common-mode failure -- and then the rate decides which come back. The
knee is **128-171 Hz/s**, below the 206 Hz/s its rated 4 g presents at zenith.
At 2 g every channel held 46 dBHz; the three steepest (79, 66, 53 Hz/s) slipped
carrier phase at ignition. Section 06 of the report has the figure.

### Traps this part set

- **Its UTC is 18 s behind the injection.** It applies leap seconds the
  simulated signal does not broadcast (`<LeapS>` 18). `$PQTMPVT`'s `<TOW>` is GPS
  time and is what `correlate.py` now uses when present.
- **GGA and PQTMPVT disagree.** Through the 15 g coast GGA claimed a 3-satellite
  fix at 18 km (29 km injected) that PQTMPVT reported as FixMode 0. The Beetle's
  driver reads PQTMPVT, so the classifier now lets PQTMPVT decide.
- **The clock survives a cold start.** The first seconds of each capture carry
  the previous run's time and were drawn mid-flight until
  `correlate.clock_outliers()` learned to spot them. The same fault was in the
  archived Quescan captures: 9 pre-lock epochs sat at t=270-278 s in its
  published figure, in the clear interval between two gates. Corrected; no
  latency changed.
- **Silence is a state.** A receiver that stops talking is not NO_LOCK. The
  bridge's notes become a grey SILENT span in `plot_flight.py`; without them the
  last epoch's colour was stretched across the gap.
- **Configuration is RAM-only**, as in flight, and a rail cycle resets the module
  to factory defaults (1 Hz, full NMEA, Normal, no PQTM). `run_radiated.py --lc86
  MODE --rtcm msm7 --cold-start` reads everything back before transmitting and
  refuses a mismatch.
- **The rail needs the app.** The Beetle's flight-computer rail is switched by
  the out computer on BLE command 8, which cannot reach it inside a closed cage:
  power it before closing the lid.

### Tracking by navigation mode: bench, then sky (2026-09-25)

Why Drone mode never held carrier phase, settled in one evening on the bench and
one night outdoors. The evidence is the module's own RTCM MSM7: per satellite a
lock-time counter that restarts when the carrier loop loses phase (a **reset**),
and the half-cycle-ambiguity flag, which a loop holding phase clears within
seconds (**half-cycle %** = share of measurements with it set). Tables report the
**strong** satellites alone (median C/N0 >= 38 dBHz in the window) as well as all
of them, because on a weak sky every mode loses lock on its weak satellites.
Two readings to avoid: the lock counter reads 0 until the receiver has time, so
nothing before the first fix counts; and GSV lists satellites MSM7 shows are not
being measured, so only MSM7 says "tracked".

Tools, all in this directory:

    make_level_steps.py   stepped-level copy of a static .C8 (the level sweep)
    lc86_bench_run.py     transmit a static .C8 at gain 0, log, switch modes on a schedule
    lc86_sky_log.py       listen-only logger for the real sky; re-applies the mode after
                          a power cycle, takes commands from a file while it logs
    lc86_tracking.py      the tables below (aba, levels, modes, sky, overnight)
    plot_lc86_tracking.py lc86g_mode_survey.svg, lc86g_sky_coldstart.svg, lc86g_level_sweep.svg

The bench signal, `c8/pad_static.C8`, is the flights' own pad extended to 25
minutes -- its first 170 s are byte-identical to `spaceshot.C8`:

    gps-sdr-sim -e BRDC_2026230.rx2.n -l 0.0,-119.0,1200 -d 1500 -b 8 -s 2600000 \
        -t 2026/08/18,08:30:00 -p -o pad_static.C8

**A/B/A on one static signal** (`lc86g_aba_pad_static`, Balloon -> Drone -> Balloon
at 244 and 484 s, gain 0, sealed cage). `./lc86_tracking.py aba`:

| Window | Used | C/N0 | Resets / strong sat-min | Half-cycle | Drops/min |
|---|---|---|---|---|---|
| Balloon 66-244 s | 11 | 42.8 | 0.00 | 0.0% | 0.0 |
| Drone 249-484 s | 11 | 42.6 | 3.60 | 60.2% | 13.3 |
| Balloon 489-733 s | 10 | 43.1 | 0.02 | 4.7% | 6.4 |

The lock loss follows the command; the C/N0 does not. Drone left damage that
Balloon did not repair in four minutes: G30 lost for good, G15 lost later, and G06
held for five minutes at 15-19 dBHz, 24 dB under its real level -- the C/A
cross-correlation level, i.e. a false lock on another satellite's code.

**Every mode** (`lc86g_mode{3,0,4,1,7,5}_pad_static`, each cold-started on the same
first 184 s). `./lc86_tracking.py modes`:

| Mode | First fix | Used | Carrier locked | Resets/min | Drops/min |
|---|---|---|---|---|---|
| Balloon (3) | 36 s | 11 | 99% | 0 | 0 |
| Normal (0) | 36 s | 13 | 100% | 0 | 0 |
| Stationary (4) | 42 s | 13 | 100% | 0 | 0.5 |
| Fitness (1) | 36 s | 12 | 0% | 0 | 10.0 |
| Swimming (7) | 36 s | 11 | 0% | 0 | 0 |
| Drone (5) | 36 s | 10 | 9% | 75 | 11.8 |

Fitness and Swimming never gain lock at all (the counter stays at 0, so they show
no resets); Drone gains it and loses it. C/N0 43.1-44.9 dBHz in every mode, rising
run to run with the transmitter's warm-up. Reserved 2 and 6 refused (result 4).

**Level sweep** (`lc86g_levels_pad_static`, Balloon, the static signal scaled in the
file 0 -> -36 -> 0 dB in 3 dB steps of 45 s; schedule
`lc86g_levels_pad_static.schedule.json`). `./lc86_tracking.py levels`: C/N0 follows
the level 1:1 (43.1, 40.6, 37.7, 34.8, 31.4, 28.4 ... dBHz), so gain 0 is not
overdriving the receiver. Carrier lock holds to about 35 dBHz (-9 dB), is half
gone at 31 (-12) and gone at 28 (-15). The fix holds to 28 dBHz, comes and goes
from 26 to 21 (-18 to -24 dB), holds again on 6-7 satellites at 15-18 (-27, -30),
and is lost at 13 (-33); stepping up, it returns at 18 dBHz (-27). The flights ran
~9 dB above where carrier lock starts to go.

**Real sky** (`lc86g_sky_20260925`, board out of the cage, partial sky: 24
satellites over GPS, Galileo, BeiDou and GLONASS, median C/N0 30-35 dBHz;
listen-only). `./lc86_tracking.py sky`:

| Phase | TTFF | Used | C/N0 | Resets / strong sat-min | Strong half-cycle |
|---|---|---|---|---|---|
| Drone, cold (first record) | ~46 s | 9 | 34.8 | 3.75 | 65% |
| Balloon, switched in | | 16 | 34.1 | 0.26 | 4% |
| Drone, switched in | | 19 | 33.2 | 0.26 | 3% |
| Balloon, cold | 42 s | 21 | 33.1 | 0.00 | 0% |
| Drone, cold | 36 s | 16 | 32.9 | 3.70 | 50% |
| Balloon, cold | 42 s | 21 | 33.2 | 0.05 | 1% |
| Drone, cold | 42 s | 16 | 35.6 | 5.10 | 71% |

Drone switched in from a settled Balloon fix holds lock like Balloon; Drone from a
cold start never does, and uses fewer satellites (it picks up few Galileo in its
first minutes). Left in Drone after the last cold start, it never settled: every
30 minutes for 6.5 hours its strong satellites reset 1.6-7.6 times a sat-minute
with a fix from 19-30 satellites (`lc86g_sky_20260925_overnight.csv`; the raw
overnight log, 137 MB, was not archived -- the capture here is the first 50
minutes, all four tests).

**So:** the simulator aggravates Drone mode (switched in, it loses lock on the
bench and keeps it on the sky) but did not invent its weakness. A module with no
backup supply of its own comes up cold at every power-up, and Drone mode from a
cold start tracks GPS and BeiDou without carrier lock. Balloon, Normal and
Stationary hold lock from a cold start.

**Traps:** `pgrep -f hackrf_transfer` matches any shell whose command line merely
mentions the name -- a waiting loop tripped the runner's "is a transmitter still
running?" guard; `pgrep -x` matches the process. NMEA numbers GLONASS satellites
slot + 64, MSM7 by slot. The HackRF's delivered level drifts up while it warms, so
compare levels only within a session.

### Boost repeatability, and the stepped signal behind the scatter (2026-09-26)

The same module in Balloon mode through the spaceshot to apogee (13.5 g burn
180.0-192.1 s), sealed cage, a cold start before every run. Thirteen runs in
three sets:

| set | file | fix rate | HackRF gain | captures (`results/`) |
|---|---|---|---|---|
| rate | stepped | 10, 5, 1 Hz | 0 | `lc86g_20260926_rate10_run1`, `_rate5_run2`, `_rate1_run3` |
| level | stepped | 10 Hz | 0, +1, +3, +3, +1, 0 | `lc86g_20260926_gain{0,1,3,3,1,0}_run{1..6}` |
| smooth | smooth | 10 Hz | 0 | `lc86g_20260926_smooth_run{1..4}` |
| smooth rate | smooth | 1, 10, 5, 1, 10, 5 Hz | 0 | `lc86g_20260926_smooth_rate{R}_run{1..6}` |

Each is `_spaceshot.log.gz` with its `.runner.txt.gz`; the smooth runs add
`.hackrf.txt.gz`, the transmitter's own log with per-second underrun counts, and
the rate set `.config.txt.gz`. The rate set followed a 5 minute warm-up on
`pad_static.C8` (`lc86_boost_series.sh smoothrate c8/spaceshot_smooth.C8 1 10 5 1 10 5`).
"Held" below means measured in every MSM7 epoch 183-192.1 s.

**On the stepped file the outcome was a coin flip.** Held through the burn: 1, 0,
0, 0, 0, 4, 0 in the seven 10 Hz runs (in run order; the 2026-09-24 run held 4),
0 at 5 Hz, 5 at 1 Hz. All or nothing: a run kept the gentlest four (G29, G12, G30,
G19 at 54-128 Hz/s; G14 at 171 too at 1 Hz) or at most one. Every channel lost
carrier lock within the first seconds of the burn in every run; "held" meant the
gentle four came back within 1-2 s in frequency-only tracking. Level made no difference: both +3 dB runs
lost everything, across pad levels of 41.9-45.9 dBHz.

**The cause is the simulator.** Stock gps-sdr-sim holds each satellite's carrier
constant for a 0.1 s block (the block's average range rate), so the burn arrives
as a staircase: G24 jumps 54 Hz every 0.1 s (measured from the IQ file), G29
about 5 Hz. A real flight slides. `patch_smooth_carrier.py` sweeps the frequency
across each block instead, and `c8/spaceshot_smooth.C8` is the same flight built
that way (the build command is in its docstring).

**On the smooth file tracking repeats.** All four runs held the same five: G29 in
unbroken phase lock; G12, G30 and G19 frequency-only at pad C/N0, with phase back
by 188-195 s; G14 with a mean loss of 1.3-4.4 dB (2.8-8 dB peak), the only
satellite whose C/N0 differs run to run. Nothing at 199 Hz/s or above held in any
run. **Every earlier boost number in this README, and in both reports, came from
the stepped file.**

**The navigation solution does not repeat, and it goes wrong more often.** Wrong
fixes (valid-flagged, more than 5 km or 50 m/s off, counted from 181 s): the seven
stepped 10 Hz runs 689, 0, 60, 0, 0, 263, 1 in run order; the four smooth runs 1,
618, 23 and 966. Two smooth runs stayed wrong to the end of the capture: one 26 km
low from 6-7 satellites at PDOP ~3, velocity right to ~20 m/s and a `$PQTMEPE`
vertical estimate of ~65 m; the other 65-70 km low from 4 satellites at PDOP ~20.
Neither muted at 80 km. The satellites a vertical burn cannot shake are the low
ones (G29 at 5 deg, G12 at 9 deg), so the fix it keeps rests on horizon geometry.
The stepped file hid some of this: a run that lost everything published nothing.

**The navigation rate does not change what the burn costs in tracking.** Over the
ten smooth runs (1 Hz x2, 5 Hz x2, 10 Hz x6) G29, G12, G30 and G19 held every time
at every rate. G14 dipped 5-9 dB at every rate and dropped for 5 s once (10 Hz).
G15 (199 Hz/s) held twice, at 5 and 10 Hz -- both times it had locked only 17-22 s
before ignition, and never when locked 43 s or more (six runs 164-176 s, one 43 s,
one 3.4 s): a loop still wide after locking, not the rate, is the likelier reading,
on two cases. **The navigation solution may differ by rate:** wrong fixes in all six
10 Hz runs (1-105 s of them), one of the two 5 Hz runs (7 s), neither 1 Hz run --
but the 1 Hz runs had no fix instead: none at all after ignition in one, one right
fix at 292 s in the other. Two runs a rate does not settle it. The runs that came
back right did so at ~262 s, as the 500 m/s mute lifted, from 9-12 satellites.
The warm-up did not hold the level: pad C/N0 rose 43.2 -> 44.8 dBHz over the first
three runs, then held 44.6-45.0. Each rate had one early and one late run.

**Not the pad clock, the stream or the start.** `msm7_clock.py`: the LC86G takes
its own clock drift out of MSM7 Doppler (0.0 +- 0.2 ppb in every run), and its
pseudorange clock bias grows ~4 m/s after the first fix in held and lost runs
alike. The HackRF streamed with no underruns, and the cold start and TX launch
landed within 6 ms of the same point in every smooth run.

**For finer tests:** satellites inside the limit now come out identical, so they
say nothing about a setting; G14 does, with a spread of about +-1.3 dB in its mean
loss over four runs, so a setting has to move it by roughly 2 dB to show with 3-4
runs each. Smooth run 1 sat 1.3 dB low on every satellite after the HackRF had
been idle ~50 minutes, most likely its level still warming up (it drifts up as it
warms); back-to-back runs agree within 0.3 dB. Warm it on a pad file first.

Tools, all in this directory:

    lc86_boost_series.sh      the run series: blind cold start, configure, transmit, log
    lc86_tracking.py boost    held / lost / first fix / WRONG, per satellite and run
    lc86_tracking.py ignition second by second through ignition, fix collapse included
    plot_boost_runs.py        one strip per run (C/N0 heatmap over the fix state), + _held.png
    plot_boost_cn0.py         C/N0 run against run: pad levels, burn relative to each pad
                              (--group: runs sharing a label prefix, e.g. "5 Hz", share a colour)
    msm7_clock.py             the pad clock: MSM7 Doppler offset and pseudorange bias
    lc86_limits.py            through a flight: GPS satellites per MSM7 epoch, the own fix
                              against the injection, and the spans with no output at all
    patch_smooth_carrier.py   the SMOOTH_CARRIER gps-sdr-sim build
    m10_rate_series.sh        the same series for the SAM-M10Q on the V9 (staged, not yet run)

**Traps:** MSM7 comes once a second at every fix rate, so "held" and lock resets
have one-second resolution at 10 Hz too. Balloon's 80 km mute outlives the
transmitter: the next run's configuration gets no answer unless a blind `$PAIR006`
goes first. A single wrong epoch at ignition (the last pad fix still reading 0 m/s
0.4 s into the burn) is latency, which is why WRONG counts from 181 s.

### When the LC86G gives measurements: the gentle flight with MSM7 on (2026-09-27)

The 3 g gentle flight (82.5 km apogee, 997 m/s peak) through the LC86G in its flight
configuration (Balloon, 10 Hz) with RTCM MSM7 on. It is the same IQ file as the
PX1105R's gentle reference, `gentle_alt_pad600_smooth`, at gain 0 and ~36 dB-Hz: 13 GPS
satellites, with no underruns. File time, from `lc86_limits.py`:

| No output at all | Injected at the edges | What stops it |
|---|---|---|
| 630.2-711.4 s (81 s) | 499.6 -> 499.1 m/s | its own speed over 500 m/s, ascent |
| 745.1-780.4 s (35 s) | 80.9 -> 81.1 km | its own altitude over 80 km (reads ~0.9 km low) |
| 816.0-875.9 s (60 s) | 500.6 -> 498.3 m/s | its own speed over 500 m/s, descent |

- **Everywhere else there is raw data:** 12-13 GPS satellites in every once-a-second
  MSM7 epoch, the 3 g boost included, with carrier lock held through the burn.
- **Inside the three windows there is nothing:** no fix and no MSM7, only a once-a-minute
  `$PAIR010` (the module asking for aiding: GPS week and time only).
- **The module keeps tracking while muted.** The MSM7 lock-time indicator on either side of each
  window shows carrier lock kept on all 12 satellites through the 81 s ascent mute, on all 12
  through the 80 km mute (a 13th, G17, left the list), and on 10 of 12 through the 60 s descent
  mute (G11 and G24 re-locked). The indicator is quantized to 1/64-1/32 of its value, so the
  comparison allows one step.
- **Output returns within one 0.1 s epoch,** straight into a valid fix.

**Against the PX1105R on the same file:** the PX1105R withholds only its own fix, and its raw
0xE5 keeps flowing through all three windows (median 13, 9 and 9 measurements per epoch). The
LC86G gives nothing, raw data included. A filter on the LC86G coasts across 81, 35 and 60 s
gaps and picks up with carrier lock intact.

Also seen:
- As before, its own altitude runs low on the descent, ~2-3 km by landing. That, and
  the 80 km mute firing at 80.9 km injected, were the rig's carrier offset: on the
  carrier-corrected file both go (see "The HackRF's carrier runs 22 Hz off its own code").
- The fix drops for 2.8 s at 1182.7 s, where the injected speed steps from ~40 to ~6 m/s.

Capture: `results/lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7.log.gz`.
`lc86_config.py` now also sends `$PAIR732,0` (ALP off), matching the flight driver since
PR #1527. This module's firmware, LC86GLANR12A03S, refuses it: `$PAIR001,732,2` (result 2,
failed) within 0.05 s, on every try, in Normal and in Balloon mode. `lc86_config.py`
reports it as not applied and carries on; the flight driver sends it three times, logs a
WARN for each and one for the refusal, and carries on too. The module powers up in
Continuous mode, so nothing is lost but ~0.15 s of boot.

### u-blox raw measurements through the gentle flight: ZED-F9P and NEO-M8T (2026-09-27)

Both u-blox parts flew the same `gentle_alt_pad600_smooth` file as the PX1105R and the LC86G,
on the conducted chain (HackRF cabled into the receiver, no cage). They logged UBX RXM-RAWX
(pseudorange, carrier phase, Doppler, C/N0 per satellite) and RXM-SFRBX (subframes, for the
ephemeris) next to NAV-PVT.

**Level.** This chain has less loss than August's 70 dB pad. `gain_sweep.py` on the static
scene found:
- **ZED-F9P:** best at gain 20, with 13 satellites at 48 dB-Hz; 42.5 at gain 26.
- **NEO-M8T:** best at gain 14, with 15 satellites at 45 dB-Hz; 41 at gain 32.

Higher gains compress both front ends: the F9P flown at 38 and 44 read a median 34 and 29
dB-Hz. The sweeps are in `results/*_20260927_gain_sweep.txt`. At 20 Hz the F9P stops sending
NAV-SAT a minute or two in, so only the first steps of its sweep are valid.

**Rates.** What each setting actually delivered:

| Receiver, setting | NAV-PVT | RXM-RAWX | Notes |
|---|---|---|---|
| ZED-F9P (HPG 1.13, USB), 20 Hz, 4 GNSS | 20.0/s | ~8 % of epochs, in bursts | raw only while few satellites are tracked; NAV-SAT ~0.2/s |
| ZED-F9P, 20 Hz, GPS only | 20.0/s | ~5 % of epochs | not the link (USB, 2.2 kB/s average); it also drops two polls in three |
| ZED-F9P, 10 Hz, GPS only | 10.0/s | 10 Hz, continuous | its practical raw rate |
| NEO-M8T (TIM 1.10, UART), 10 Hz, 4 GNSS, 115200 baud | 10.0/s | 10.0/s | transmit buffer peaked at 6 % |
| NEO-M8T, 18 Hz, GPS only, 460800 baud | 13-16.5/s | 17.7-17.9/s | skips fixes, never raw epochs; 20 Hz is not accepted and it runs at 10 |

**When raw data flows**, against the three COCOM windows (file time):

| Receiver | Raw in the speed windows (631-710, 818-875 s) | Raw in the 80 km window | Own fix |
|---|---|---|---|
| PX1105R | yes | yes | withheld above ~515 m/s and 80 km |
| LC86G | no: all output stops | no: all output stops | its own 500 m/s and 80 km |
| ZED-F9P | no: RAWX stops within 0.1 s of the fix | yes, 423 epochs | its own 514.3-515.0 m/s and 80.04-80.08 km |
| NEO-M8T | no | yes | lost at 513.6 m/s; its 50 km airborne ceiling keeps it off until 875 s |

The NEO-M8T also flew at 18 Hz (GPS only, 460800 baud, gain 14). That is the highest-rate raw
capture of the four receivers: 20,362 RAWX epochs at 17.9 Hz with 14 GPS satellites, gapped
only in the speed windows (631.4-709.9 and 817.6-875.3 s). It kept the full rate through the 80
km window and through its 50 km no-fix span. The fix edges match the 10 Hz flight (lost at
514.6 m/s, back at 875.2 s), and the transmit buffer stayed at or below 4 %.

**Open:** the F9P's fix drops for a few seconds 20-27 times a flight, at every gain, and on the
static pad (every ~80-100 s at first). Its raw data flows straight through the dropouts. No
clock reset (one `clkReset`, at the first fix) and no pseudorange jump lines up with them. The
NEO-M8T on the same file never dropped, so it is the F9P, not the signal. The August F9P gentle
capture had a fix in only 517 of 812 epochs, fewer than its gate windows explain.

**Setup quirks:**
- The NEO-M8T had been left in a timing setup. At 115200 baud, seven extra NMEA sentences plus
  RXM-SVSI (~1.2 kB at every epoch), RXM-MEASX and 02-61 filled the line completely.
- The M8 path of `ubx_config.py` never set a rate, so an M8 stayed at 1 Hz.
- The F9P NAKs `CFG-SIGNAL-GPS_L2C_ENA = 0`, so its L2C stays on.

**Tools:**
- `ubx_config.py` gains `--raw` (RAWX + SFRBX; NAV-SAT drops to once a second), `--gps-only`
  (CFG-SIGNAL on F9, CFG-GNSS on M8), `--mon-comms` (MON-COMMS on F9, MON-TXBUF on M8) and
  `--set-baud` (M8 UART1, RAM only), with polls retried three times.
- Its M8 path now sets the rate, silences every NMEA sentence and any leftover UBX output, and
  reports the rates that actually arrive.
- `run_radiated.py` writes a `# host: tx` header and idles the radio at the end;
  `gain_sweep.py` idles it too.
- `ubx_limits.py` plots GPS satellites per RAWX epoch against the NAV-PVT fix and the
  injection, and summarizes MON-COMMS.

Captures: `results/zed_f9p_20260927_*` and `results/neo_m8t_20260927_*`.


## PX1105R (TinkerNav): where the withheld output goes, and what a filter recovers (2026-09-26)

The SkyTraq PX1105R (TinkerNav board, ESP32-S3 bridge running `gnss_passthrough`,
RTK **kinematic base** mode so it emits 0xE5 raw at 20 Hz; see the raw-measurement
work on PR #1523) through the smooth spaceshot and gentle flights, radiated in the
sealed cage. What this adds to the fix-level COCOM picture: the raw measurements
themselves, and what a filter does with them.

**The measurements are clean the whole flight, even where the receiver withholds
its own fix.** A direct single-point solve from the raw pseudoranges (`px_validity.py`,
and the `spp_fix` sweep) lands within 3-200 m at every point of both flights --
992 m/s boost, 82 km apogee, 761 m/s descent. Where a measurement exists it is good;
the export limit is enforced by *withholding output*, not by degrading it.

**The gate is on the receiver's own solution, not the flight.** With no fix (it
doesn't know it is fast) raw flows at 1300 m/s; once it has a fix above 500 m/s or
80 km it goes silent, and the mute latches on its own state (it can stay muted well
after the flight is back inside the limits). Its speed ceiling is ~514 m/s (1000 kn),
not the LC86G's 500.

**Navigation (dynamics) mode is the lever -- `nav_mode.py`, AN0037 0x64/0x17**
(query 0x64/0x18 -> 0x64/0x8B; all eight modes ACK and read back even in RTK base).
Earlier runs set no mode, so they ran the default low-dynamics filter -- much of the
"lockout" was that filter failing, not a clean gate (the companion note warns of
exactly this). On the 3 g gentle flight, pedestrian (1) loses its fix at ~480 s and
never recovers; **SLR "speed-lag-reduced" (9)** tracks the true speed to 500 m/s on
both ascent and descent and recovers repeatedly. A full mode sweep on the gentle
flight is archived (nav1/4/5/7/9; auto/car/marine owed).

**A GNSS-only filter follows the trajectory where measurements exist**
(`tc_ekf_slr.py --gnss-only` on PR #1523's `tc_ekf`: constant-velocity kinematic
model + `update_gnss_raw`). Gentle 3 g: tracks end to end, ~20-80 m horizontal,
~75-285 m vertical (weak GPS-L1-only geometry), sub-m/s to 12 m/s velocity. High-g
13.5 g: the ignition drops output from ~360 to ~470 s (burn + >500 m/s ascent, a
real gap the receiver cannot bridge), so GNSS-only coasts and errs to tens of km
there, then **snaps to truth on reacquisition** and tracks apogee and the whole
descent to 40-150 m. Adding a simulated IMU + baro bridges the gentle flight but the
INS vertical channel diverges through the high-speed descent once the BMP585 baro
(spec 300 hPa ~ 9.16 km) is out of range -- a vertically-stabilized mechanization is
needed, and that gap is exactly where the IMU earns its place.

Tools (this directory): `px1105r_run.py` (cold start, set nav mode, transmit, log,
idle the radio), `nav_mode.py` (0x64/0x17), `skytraq_raw.py` (kinematic-base + 0xE5,
vendored from PR #1523), `px_limits.py` (raw + own-fix vs the injected limits),
`px_validity.py` (per-satellite pseudorange/Doppler residuals against the truth).
The tightly-coupled/GNSS-only filter driver is `tinkerrocket-sim/scripts/tc_ekf_slr.py`
(committed 2026-09-27; the EKF it uses came in with PR #1523). Captures:
`results/px1105r_*` (gzip; the exploratory gain/level probes stayed local).


### Restarting after burnout, and the elevation mask (2026-09-26)

Can a hot start at burnout, seeded with the vehicle's state (the truth here, an IMU in
flight), bring measurements back sooner after the 13.5 g boost? `px1105r_run.py
--hot-start FILET` sends AN0037 `0x01` System Restart, built by
`seed_restart.restart_payload`; `--restart-mode cold` sends a cold start instead. The
seed's altitude is bounded to -1000..18,300 m (AN0037 p. 15) -- the 60,000 ft COCOM
altitude -- so on the spaceshot an in-spec seed must go out within 6.8 s of burnout.
Times below are seconds after ignition (burnout 12.1, 500 m/s crossing 82.1); "raw back"
is 4+ measurements in every 0xE5 epoch for 5 s.

With the receiver's default 15 degree elevation mask and a 360 s pad, restarts looked
like they helped, sometimes:

| Run (15 deg mask, 360 s pad) | Ephemeris at ignition | Raw back | First fix | Fix epochs |
|---|---|---|---|---|
| Baseline, no restart | 9 of 9 | 115.5 | 110.0 | 1245 |
| Hot start at burnout, run 1 | 6 of 9 | 86.0 | 81.0 | 1860 |
| Hot start at burnout, run 2 | 5 of 9 | 253.0 | 246.2 | 676 |
| Cold start at burnout | 9 of 9 | 262.5 | 253.7 | 426 |
| Hot start at 32 km (seed altitude out of spec) | 9 of 9 | 90.0 | 89.2 | 1710 |

The spread came from the setup, not the restart:

- **The ephemeris decoded by ignition varied from 5 to 9 of 9 on the same file.** A hot
  start restarts with only what it holds; the pad's "measurements per epoch" is exactly
  that count. A cold start throws it all away and was the worst.
- **The PX1105R ships with a 15 degree elevation mask** (`0x2F` -> `0xB0`: select 1,
  elevation 15, CNR 0). It kept the five lowest satellites (G12, G13, G19, G29, G30 at
  5-13 degrees) out of its channel table entirely, although the IQ carried all 14 at
  equal power. On a vertical flight those are the satellites that see the least boost
  Doppler and acceleration -- both scale with sin(elevation) -- so the mask discarded the
  most boost-robust part of the sky. (`sky_scan.py`: the scenario's origin and time are
  already the best sky that day, 14 GPS above 5 degrees.)

With the mask at 3 degrees (`--elev-mask 3`: AN0037 `0x2B`, p. 27, SRAM only -- a power
cycle restores 15) and a 600 s pad (`pad_scenario.py`), every satellite had ephemeris at
ignition, the baseline came back on its own at the first moment the speed limit allows,
and the hot start added nothing:

| Run (3 deg mask, 600 s pad) | Ephemeris at ignition | Raw back | First fix | Fix epochs |
|---|---|---|---|---|
| Baseline, no restart, run 1 | 14 of 14 | 79.7 | 80.7 | 1831 |
| Baseline, no restart, run 2 | 14 of 14 | 19.9 | 80.6 | 1833 |
| Hot start at burnout | 14 of 14 | 82.0 | 80.8 | 1844 |

Both now report a fix through every stretch where one is allowed -- between the 500 m/s
crossing and 80 km, below 80 km before the descent speeds up, and after the descent's
500 m/s crossing -- so what remains is the export limits themselves. Flight firmware that
leaves the SkyTraq default of 15 degrees drops exactly the satellites that ride through a
boost best.

The fix repeats to 0.2 s and 1% in fix count; raw output during the fast coast does not.
Run 2 released 5-12 raw measurements from ~15 s after ignition (at ~1100 m/s, no fix of
its own); run 1 released none until ~80 s, although 0xE7 shows 7-9 satellites locked
through its whole coast. Neither had a fast fix before losing lock at ignition (both
lost it within ~1 s, at 3 and 20 m/s of their own) and both pads were identical, so what
decides it is not visible in these messages. A restart stops 0xE5 output entirely until
the fix returns, so a filter built on raw measurements is better off without one.

Caveats: two baselines and one hot start at 3 degrees. Channel status (0xE7) can show satellites
locked at full strength while 0xE5 carries no measurements -- the receiver withholds raw
output with its fix -- so "raw back" is read from 0xE5, not from 0xE7.

Tools: `px_restart_compare.py` (runs side by side, seconds after ignition),
`px_limits.py` (now marks any `# host:` event the runner logged), `sky_scan.py`,
`pad_scenario.py`. Captures: `results/px1105r_spaceshot_pad360_*_{hot372_run1,
hot372_run2,hot392,cold372}` and `results/px1105r_spaceshot_pad600_*_el3_{run1,run2,hot612}`.


### PX1125R: power save, and the carrier build (2026-09-26)

The older SkyTraq, on the dual-MCU TinkerNav (RP2040 + ESP32-C3 + PX1125R). The receiver hangs
off the **RP2040**; the C3 is only the radio. The RP2040 needs main's `gnss_passthrough`, which
scores SkyTraq binary frames as a live link -- this branch's copy is the older NMEA-only one and
cannot follow the receiver to a new baud rate. Firmware kernel 3.0.1 / ODM 1.7.33, built
2022-08-22. Factory state: RTK base (survey), 1 Hz, 115200, auto dynamics, 15 degree mask,
**power save**. Gain sweep 0/2/4/6 dB -> 37/39/41/43 dB-Hz; +2 dB matches the PX1105R's
37-40 dB-Hz at +3. Configured like the PX1105R for the flights: kinematic base, 20 Hz, 921600,
SLR, 3 degree mask.

**On the smooth-carrier IQ, in power save, it never fixed** -- five runs up to 420 s, with 13-14
satellites tracked at 39-41 dB-Hz, frame sync reached and ~160 subframes decoded. It never got
three consecutive subframes (1-2-3) from any satellite, so it never assembled an ephemeris.
Underneath: every ~15 s, three or more channels lose frame sync at once. Both SkyTraq parts do
this on the bench (the PX1105R too, on its own board), in every IQ build and at every gain; the
LC86G on the same transmit chain shows no common-mode lock resets at all. Real-sky data never
shows it, so it is a bench effect that only SkyTraq reacts to -- open, and next. (On the
PX1105R they turned out to be power save; see the next section.)

First fix from a cold start on the same pad, cold / 20 Hz / kinematic / SLR / 3 degrees:

| IQ build (gps-sdr-sim) | Power save (factory) | Power normal |
|---|---|---|
| stock (fixed-point carrier) | ~136 s | -- |
| float-only (`FLOAT_CARR_PHASE`) | ~326 s | -- |
| fixed-point ramp (`patch_smooth_fixed.py`) | ~290 s | -- |
| smooth (`patch_smooth_carrier.py`: float + ramp) | never (5 runs) | **~53 s** |

The smooth file differs from stock mostly through gps-sdr-sim's floating-point carrier path, not
the ramp: today's source built fixed-point reproduces the stock file byte for byte, and on a
static pad smooth and float-only IQ are 68 % bit-identical (correlation 0.9995) against 10 %
(0.997) for stock and float-only. One satellite at a time, all builds track the same (Doppler,
amplitude, phase continuity, code). Power save -- which AN0037 says throttles the search engine
-- is what turned slow into never: with it off (`0x0C 00`, SRAM) the smooth file fixes in ~53 s
and the collapses no longer stop decoding. **Every PX1105R and PX1125R run before this one was
in power save.**

The 13.5 g spaceshot, seconds after ignition (500 m/s crossings 82.1 and 246.6):

| Receiver, IQ, power | Ephemeris at ignition | Raw back | First fix | Fix epochs |
|---|---|---|---|---|
| PX1105R, smooth, factory (run 1) | 14 of 14 | 79.7 | 80.7 | 1831 |
| PX1105R, smooth, factory (run 2) | 14 of 14 | 19.9 | 80.6 | 1833 |
| PX1125R, stock, save | 13 of 13 | 67.6 | 85.8 | 1520 |
| PX1125R, smooth, normal | 13 of 13 | 91.0 | 91.1 | 1634 |

The PX1125R comes back ~10 s after the PX1105R on the ascent, and at the same moment on the
descent once power save is off (the stock/power-save run was ~13 s late there). The PX1105R
rerun with power save off is in the next section.

**Flight software driving a SkyTraq receiver must, at every boot:** turn power save off
(`0x0C 00`), set the elevation mask to 3-5 degrees (`0x2B`), and set a dynamics mode
(`0x64/0x17`). All three are SRAM settings that revert on a power cycle.

Tools: `px1105r_run.py --power-mode normal|save`, `--pad-restart warm|hot|cold` (seeded from
the scenario start), `--rx NAME --mac SERIAL|--port DEV`; `patch_smooth_fixed.py`;
`px_restart_compare.py` rows now name the receiver, IQ build and power mode. Captures:
`results/px1125r_spaceshot_pad600_*` (sweep, the diagnostic static runs, both flights).

### PX1105R with power save off, and the gentle flight for filter work (2026-09-26)

The PX1105R's spaceshot again with power save off (`--power-mode normal`), everything else as in
runs 1 and 2. Seconds after ignition:

| Receiver, IQ, power | Ephemeris at ignition | Raw back | First fix | Fix epochs |
|---|---|---|---|---|
| PX1105R, smooth, factory (run 1) | 14 of 14 | 79.7 | 80.7 | 1831 |
| PX1105R, smooth, factory (run 2) | 14 of 14 | 19.9 | 80.6 | 1833 |
| PX1105R, smooth, normal | 14 of 14 | 61.5 | 80.8 | 1923 |

The first fix does not move: the gate sets it, at the 82.1 s crossing. Raw return at 61.5 s is
inside the power-save spread, so one run shows no effect there. The extra 5 % of fix epochs is
continuity: in power save the fix dropped for ~1 s every 11-13 s on the pad and a few times in
flight; in normal it never dropped.

**On the PX1105R the ~15 s collapses are power save.** Epochs where three or more GPS channels
lose frame sync together, over the pad (100-595 s host time), with `skytraq_collapses.py`:

| Receiver, power | Runs | Collapses | Median interval | Fix dropouts > 0.3 s |
|---|---|---|---|---|
| PX1105R, factory save | 3 | 30, 32, 36 | 11-12 s | 6, 10, 13 |
| PX1105R, normal | 2 | 1, 2 | -- | 0, 2 |
| PX1125R, save (stock IQ) | 1 | 29 | 15 s | 4 |
| PX1125R, normal (smooth IQ) | 2 | 25, 30 | 15 s | 13, 0 |

So the PX1105R's collapses are its own power-save search cycle, not the bench. The PX1125R's
persist in normal, ~15 s apart, and stay open. The LC86G shows none.

**The gentle flight, for filter work.** The 82.5 km, 3 g profile (peak 997 m/s) through the
PX1105R with every current setting: +3 dB, SLR, 3 degree mask, power normal, cold start on a
600 s pad, raw 0xE5 and nav 0xDF at 20 Hz, no underruns. Capture
`results/px1105r_gentle_alt_pad600_smooth_gain3_nav9_el3_pmnormal_run2.log.gz`; truth
`scenarios/gentle_alt_pad600.csv` (10 Hz, file time, ignition at 600 s). The receiver withholds
its own fix in three windows (file time), each edge within ~1 s of the injected crossing, and
raw measurements never stop:

| Fix withheld (s) | Why | Raw per epoch: min / median / max |
|---|---|---|
| 631.5-710.1 | speed over ~515 m/s | 11 / 13 / 13 |
| 740.4-786.6 | above 80 km | 6 / 9 / 13 |
| 818.5-875.5 | speed over ~515 m/s | 7 / 9 / 13 |

Outside those windows it holds a fix from ignition to the end of the file, apart from 0.3 s at
605.8 s just after ignition and 0.1 s at 881.1 s. On the descent, 900-1260 s, the median is
13 raw measurements per epoch.

Gentle run 1 (the same name without `_run2`) is void from file time 373 s. One 27 ms HackRF
underrun (140,224 bytes) time-shifted the whole simulated sky on the pad; every channel dropped,
and the receiver never regained frame sync or a fix in the remaining 900 s, though the IQ file
was intact. Underruns are rare -- 2 of 19 runs on the 600 s pad; the other was 3.6 ms on the
PX1125R stock run -- and they do not explain the collapses: the runs with 30 collapses had none.
`skytraq_collapses.py` prints each capture's underrun count; check it before trusting a run.

Tools: `skytraq_collapses.py T0 T1 CAP...`; `px_restart_compare.py` now titles each row with only
the settings that differ between rows (shared ones go in the figure title) and labels captures
made before `--power-mode` existed as factory power save. Captures: `results/px1105r_*_pmnormal*`.

### Data for filter work (2026-09-27)

Everything a tracking-filter study needs from this rig is committed here or regenerates exactly.

**Captures** (`results/*.log.gz`): one line per message, `<host s> B <hex>` for SkyTraq
binary (message id first) or `<host s> U <hex>` for UBX, plus `#` comments. The first line
is the runner's `# host:` header with the settings.

| Receiver | What it logs | Runs to start from |
|---|---|---|
| PX1105R | 0xE5 raw at 20 Hz, 0xE0 subframes, 0xDF own fix, 0xE7 channels | gentle: `px1105r_gentle_alt_pad600_smooth_gain3_nav9_el3_pmnormal_run2` (the reference); spaceshot: `px1105r_spaceshot_pad600_smooth_gain3_nav9_el3_{run1,run2,pmnormal,hot612}`; nav-mode sweep: `px1105r_gentle_alt_pad360_smooth_gain3_nav{1,4,5,7,9}` |
| PX1125R | the same set; raw passes both gates | `px1125r_spaceshot_pad600_smooth_gain2_nav9_el3_pmnormal_flight`, `px1125r_spaceshot_pad600_stock_gain2_nav9_el3` |
| NEO-M8T | UBX RXM-RAWX (~1 Hz) + RXM-SFRBX; RAWX flowed while NAV-PVT was withheld | `neo_m8t_gentle_alt`, `neo_m8t_spaceshot`, `neo_m8t_t2_altramp` |
| LC86G | fix level (NMEA, PQTM) | `lc86g_{normal,balloon,drone}_{gentle_alt,spaceshot}`, the `lc86g_20260926_*` boost series, `lc86g_sky_20260925` (real sky) |

**Truth.** `scenarios/` is gitignored, but the flights regenerate byte for byte:
`./make_flights.py --lat 0 --lon -119 --only spaceshot --only gentle_alt`, then
`./pad_scenario.py NAME PAD [END]` for the padded CSVs. The JSONs (10 Hz truth, `blocked_windows`,
`start_time`) are already committed as `results/<receiver>_spaceshot.scenario.json` and
`results/<receiver>_gentle_alt.scenario.json`: every copy is identical, e.g. `neo_m8t_*`. The
`_v2`, `_eq` and `_horizon` ones are older flights.

**Clocks.** Every IQ file here starts at 2026/08/18 08:30:00 GPS, TOW 203400 (checked on five
captures). So file time = the receiver's TOW - 203400, and ignition is at 600 s (pad600), 360 s
(pad360) or 180 s (no pad). Scenario time is file time - (pad - 180). Host timestamps lag file
time by the header's "TX launched" delay plus ~1.4 s of HackRF start latency; `px_limits.py` and
`px_restart_compare.py` align by the median host - TOW offset instead. `tc_ekf_capture.py` takes
`--tow0 203400 + (pad - 180)`; `tc_ekf_slr.py` takes `--tow0 203400 --pad-shift <pad - 180>`.

**Filter code** (tinkerrocket-sim, from PR #1523 on main):
- `estimation/tc_ekf.py`: `TcEkf`, with `update_gnss_raw`, `update_carrier` and `propagate_kinematic`.
- `estimation/gnss_raw.py`: `read_capture` builds the ephemeris from the capture's own
  subframes, so no RINEX file is needed.
- `scripts/tc_ekf_capture.py`: GNSS-only, scored against a scenario. It now tracks straight
  through `blocked_windows` (it used to skip every epoch inside them; `--skip-blocked` still does).
- `scripts/tc_ekf_slr.py`: tightly coupled with a synthesized IMU and baro, or `--gnss-only`. Its
  scoring phases are hard-coded for gentle_alt on the 360 s pad.
- `scripts/tc_ekf_cocom.py` (2026-09-27): both, on any pad, scored per phase against the truth;
  the results and the models it needs are in the next section.

**Caveats:**
- Gentle run 1 (`..._pmnormal`, no `_run2`) is void after 373 s: the HackRF underrun.
- `px1105r_gentle_alt_pad360_smooth_gain3_nav4` had a 73 ms underrun (378,656 bytes) at file
  time ~240 s. Every channel was lost for ~40 s, and it re-fixed on the time-shifted sky before
  ignition, so its pad is not comparable with the other nav-mode runs.
- Every capture carries the rig's 22 Hz carrier-vs-code offset: code-minus-carrier grows at
  4.19 m/s, and the clock drift the Doppler reports sits 4.2 m/s off the pseudoranges' clock-bias
  rate, so a filter needs a clock-rate offset state. Carrier-smoothed pseudoranges also restart
  +7 to +15 m high at every (re)lock; that is the rig, not the receiver (see "The HackRF's
  carrier runs 22 Hz off its own code").
- Truth velocities are the signal's at each row's own time. `make_flights.py` writes them that
  way, and the archived JSONs were retimed on 2026-09-27. Files made before that carry the 0.1 s
  block ending at each row, 0.05 s late: 1 m/s in the 3 g boost, 6.6 m/s at 13.5 g.
  `make_flights.block_timed()` tells the two apart, and `make_flights.py --retime` converts an
  old one. Altitude is exact at the rows either way.
- PX1105R captures without `pmnormal` ran factory power save, with ~1 s fix dropouts every
  11-13 s on the pad; raw keeps flowing.
- Captures without `_el3` ran the factory 15 degree mask, 9 of 14 satellites.
- Measurements are 0xE5; 0xE7 channel lock is not a measurement.
- The simulated sky is GPS L1 C/A only, noiseless, ~37-40 dB-Hz at +3 dB. GNSS-only vertical
  errors were 75-285 m in the study above; that was mostly the PX1105R's late Doppler under a
  constant-velocity model, not the geometry (next section).
- On the 13.5 g spaceshot the PX1105R's raw output stops at ignition and returns 20-80 s later,
  varying run to run. The gentle flight never loses raw.
- No capture has an IMU. `tc_ekf_slr.py` synthesizes one from truth.

**Ephemeris.** The broadcast ephemeris the IQ files were built from is committed:
- `results/BRDC_2026230.rx2.n.gz` is GPS-only RINEX 2, converted by `rinex3to2.py`. It covers
  2026-08-18, the day of every scenario here.
- `results/BRDC_2026230_MN.rnx.gz` is the original RINEX 3 download from CDDIS.
- `BRDC_2026239.*` is 2026-08-27. No committed scenario uses it.

Unpack into `c8/` before running `build_scenarios.sh` or gps-sdr-sim:
`gzip -dc results/BRDC_2026230.rx2.n.gz > c8/BRDC_2026230.rx2.n`.

**Left local on purpose:** the IQ files (`c8/`, rebuilt from `build_scenarios.sh` with the
patch scripts here), the exploratory PX1105R probes, and the 137 MB overnight LC86G sky log.

### Filtering the PX1105R's raw data: GNSS only, then with an IMU (2026-09-27)

What a filter gets from the PX1105R's raw pseudorange and Doppler on the two power-normal
flights -- gentle (`px1105r_gentle_alt_pad600_smooth_gain3_nav9_el3_pmnormal_run2`) and spaceshot
(`px1105r_spaceshot_pad600_smooth_gain3_nav9_el3_pmnormal`) -- first GNSS only, then tightly
coupled with an IMU synthesized from the truth. Driver: `tinkerrocket-sim/scripts/tc_ekf_cocom.py`.
IMU runs used three noise seeds; the tables give the median. The seeds agree within 10 % wherever
raw measurements have been flowing for a while; right after the spaceshot's gap they spread
(W1: 12-17 m altitude, 8-21 m horizontal).

**Truth, first.** gps-sdr-sim takes one motion row per 0.1 s, and `make_flights.py` integrates
h_k = h_(k-1) + v_k dt, so v_k is the mean velocity over the block *before* t_k: the smooth-carrier
build puts it at t_k - 0.05 s. Read at t_k, as the scenario JSONs gave it, the truth velocity was
half a block late -- 1 m/s in the gentle boost, 6.6 m/s at 13.5 g. `make_flights.py` now writes the
signal's velocity at each row and the archived JSONs are retimed (PR #1534); `RigTruth` tells the
two kinds apart from the rows, so the numbers below hold with either.

**The measurements**, against the truth, each epoch's common part removed
(`figures/px1105r_raw_errors.svg`):

- **The Doppler is 0.22 s late; the pseudorange is on time.** Each satellite's range-rate error is
  lag x acceleration x sin(elevation). Fitted phase by phase over a grid of lags, the lag is
  0.22-0.23 s on the gentle flight (0.15-0.25 s on the spaceshot's coarser grid), and the carrier
  phase is ~0.20 s late too. Taken as the range rate 0.22 s earlier, the in-flight residual falls
  from 0.6-1.6 m/s to about 0.3 m/s (0.13 on the pad). gps-sdr-sim injects code and carrier with no
  lag between them, so this is the receiver. In factory power save the lag is 0.05-0.2 s and moves
  with the dynamics.
- **The pseudoranges carry the receiver's smoothing.** 5-7 m RMS on the pad and in the boost,
  10-13 m in the coast and descent, where the two highest satellites drift to -20 to -30 m. About
  40 % of it follows the low-passed range acceleration, the signature of smoothing the code with a
  late carrier. Every (re)lock starts high -- +7 to +15 m at the median, 30-45 m in a fifth to a
  third of them -- and decays over 10-20 s. That part is the rig, not the receiver: a
  carrier-smoothed pseudorange restarts at the raw code while settled channels sit the rig's
  4.19 m/s carrier-vs-code split times the smoothing time below it, and the real sky shows no such
  offset (PR #1534). On the spaceshot the two highest satellites drop and re-lock again and again
  through the coast and descent. In power save every channel re-syncs every ~12 s and the pad
  alone is 33 m RMS.
- **The rig's carrier runs 4.2 m/s off its code clock**: the clock rate the Doppler reports minus
  the rate at which the pseudoranges' clock bias moves. One oscillator drives both in a receiver
  (0.22 m/s on the real sky); here the HackRF shifts the carrier alone, and a filter state takes it.

**GNSS only.** Each cell is the RMS altitude error in metres / the RMS vertical velocity error in
m/s over that phase. Phases, in seconds after ignition -- gentle: pad -60-0, boost 0-60, W1 31-110,
coast 110-141, W2 141-186, descent 186-218, W3 218-275, to main 275-582; spaceshot: boost 0-12,
W1 4-81, coast 81-112, W2 112-157, descent 157-188, W3 188-246 (W1 starts inside the boost).
Gentle flight -- raw never stops:

| RMS error: altitude (m) / vertical velocity (m/s) | pad | boost | W1 >515 m/s up | coast | W2 >80 km | descent | W3 >515 m/s down | to main |
|---|---|---|---|---|---|---|---|---|
| Receiver's own fix | 11.5 / 0.05 | 49 / 11.1 | -- | 47 / 3.4 | -- | 35 / 3.8 | -- | 14 / 1.8 |
| Epoch least squares | 9.3 / 0.15 | 26 / 3.9 | 23 / 2.9 | 34 / 2.4 | 38 / 2.1 | 29 / 2.2 | 24 / 2.4 | 11 / 1.0 |
| EKF, constant velocity | 2.4 / 0.11 | 117 / 3.9 | 134 / 2.9 | 20 / 2.1 | 79 / 2.1 | 146 / 2.1 | 192 / 2.3 | 24 / 1.0 |
| EKF, constant acceleration, late Doppler modelled | 2.1 / 0.11 | 10 / 0.46 | 14 / 0.37 | 6.2 / 0.52 | 5.5 / 0.38 | 2.8 / 0.50 | 15 / 0.73 | 12 / 0.24 |

The constant-velocity EKF is `tc_ekf_capture.py`'s model and the source of the 75-285 m above. It
cannot say where the vehicle was 0.22 s ago, so every Doppler reads off by lag x acceleration and
the error integrates into altitude. Carrying the acceleration as a state lets the filter predict
each range rate at t - 0.22 s; the constant-acceleration model without the lag is no better than
constant velocity (27-199 m by phase). The gates reject 0.01 % of the measurements.

Spaceshot, power normal: raw stops at ignition for 61 s (bar a 0.6 s burst of five satellites at
13 s), so GNSS alone never sees the burn -- 1.5 km of altitude error in the boost, 2.8 km in W1.
When raw returns, the filter restarts on a checked least-squares fix if its prediction disagrees
with one: within 100 m 2 s later, within 30 m after 12 s. Then: coast 11 / 0.47, W2 28 / 1.3,
descent 40 / 0.63, W3 55 / 1.0.

**With an IMU.** A body-frame IMU synthesized from the truth: nose up, not rotating, with the
sim's `IMUModel` noise and bias (its ISM6HG256X numbers), in a real-Earth world -- WGS84 gravity,
the Earth's rate on the gyros, and the Coriolis force a vertical ground track needs.
(`make_flights.py` integrated the trajectory with a spherical 9.80665 m/s^2 gravity, so in this
world the coast reads ~0.03 m/s^2 of non-gravitational force, as a whisper of drag would.) The pad
is seeded as the flight computer does it (attitude from gravity and heading, gyro bias from the
stationary mean, levelling until ignition).
The tightly coupled filter fuses the same raw pseudorange and Doppler, predicting each late
Doppler from the IMU's own velocity history. No barometer: with raw measurements flowing, the
vertical channel needs none, and nothing diverges in the fast descent.

Gentle flight:

| RMS error: altitude (m) / vertical velocity (m/s) | boost | W1 >515 m/s up | coast | W2 >80 km | descent | W3 >515 m/s down | to main |
|---|---|---|---|---|---|---|---|
| IMU + receiver's own fix (the flight filter today) | 178 / 8.1 | 404 / 7.6 | 101 / 3.4 | 134 / 2.7 | 62 / 3.5 | 230 / 4.1 | 34 / 1.8 |
| IMU + raw, flight filter's mechanization | 8.8 / 0.29 | 14 / 0.20 | 10 / 0.24 | 13 / 0.27 | 15 / 0.23 | 27 / 0.60 | 19 / 0.19 |
| IMU + raw, WGS84 gravity + Earth rotation, no pitch gate | 8.7 / 0.28 | 13 / 0.18 | 7.6 / 0.17 | 9.2 / 0.11 | 9.3 / 0.17 | 21 / 0.51 | 15 / 0.19 |
| IMU alone after ignition (same mechanization) | 4.5 / 0.10 | 10 / 0.21 | 25 / 0.44 | 45 / 0.69 | 75 / 0.92 | 118 / 0.97 | 321 / 4.4 |

Spaceshot, power normal:

| RMS error: altitude (m) / vertical velocity (m/s) | boost | W1 >515 m/s up | coast | W2 >80 km | descent | W3 >515 m/s down |
|---|---|---|---|---|---|---|
| IMU + receiver's own fix (the flight filter today) | 41 / 4.5 | 315 / 8.2 | 77 / 2.9 | 63 / 3.6 | 56 / 3.5 | 135 / 1.8 |
| IMU + raw, flight filter's mechanization | 3.0 / 0.02 | 26 / 2.2 | 13 / 0.30 | 24 / 1.5 | 37 / 0.38 | 53 / 0.92 |
| IMU + raw, WGS84 gravity + Earth rotation, no pitch gate | 3.1 / 0.04 | 16 / 0.48 | 11 / 0.16 | 28 / 0.84 | 40 / 0.23 | 54 / 0.70 |
| IMU alone after ignition (same mechanization) | 3.0 / 0.03 | 7.2 / 0.18 | 19 / 0.44 | 40 / 0.65 | 69 / 0.95 | 118 / 1.2 |

Trajectories with each filter's estimate, the signed errors, and the satellites reported:
`figures/px1105r_track_gentle.svg` and `figures/px1105r_track_spaceshot.svg`
(`tc_ekf_cocom.py --plot`; off-scale excursions are named with their peak).

What the IMU adds:

- **It carries the 13.5 g ignition gap**: 16 m of altitude and 0.5 m/s through W1, horizontal
  10 m, where GNSS alone is kilometres off.
- **Velocity**: 0.1-0.8 m/s vertically, against 0.4-1.3 m/s from GNSS alone.
- **Not altitude, once raw flows.** The 10-55 m floor (a bias of -30 to -50 m through the spaceshot's
  descent) is the smoothed pseudoranges' own error, and both filters sit on it. For the first
  80-110 s after ignition the IMU alone holds altitude better (3-10 m) than either.

The flight filter's mechanization, which TcEkf mirrors, meets three things on this flight:

- **Constant gravity (#1530) and no Earth rotation (#1529).** Gravity at 80 km is 2.5 % weaker
  than on the pad, and Coriolis at 1 km/s is 0.15 m/s^2. Each fix has its own channel -- through
  the spaceshot's 61 s gap (W1, RMS, median of 3 seeds):

  | W1 through the 61 s gap | altitude (m) | vertical velocity (m/s) | horizontal (m) | horizontal velocity (m/s) |
  |---|---|---|---|---|
  | as flown: constant G, no Earth rotation | 25.9 | 2.25 | 67.7 | 3.52 |
  | WGS84 gravity (`--gravity wgs84`) | 16.5 | 0.49 | 67.7 | 3.52 |
  | Earth rotation (`--earth-rate`) | 25.6 | 2.23 | 17.6 | 1.11 |
  | both | 16.7 | 0.49 | 17.8 | 1.12 |
  | both, no pitch gate (`--no-att-gate`) | 16.3 | 0.48 | 9.8 | 0.92 |

  Both need a matching change elsewhere. The pad levelling must use the same gravity: levelled
  against a constant 9.807 while propagating WGS84, the filter parks the difference (0.03 m/s^2 at
  the equator) in its accelerometer bias, and it acts for the whole flight -- a 2 m/s, 68 m ramp
  through this gap before the fix. And the pad's stationary-mean gyro seed must leave the Earth's
  rate out once the mechanization models it (`TcEkf.seed_gyro_bias`); levelling hides the
  horizontal part of a double count, not the part about the vertical.
- **The cos^4(pitch) gate on GNSS attitude corrections** (#1531) is zero when the vehicle is vertical,
  so no GNSS update ever corrects its tilt. On the gentle flight the tilt drifted to 4-6 degrees, the
  filter parked it in its accelerometer-bias states (0.05-0.26 m/s^2 against a true 0.01), and with
  13 satellites in view the horizontal error reached 12-19 m and 0.5 m/s late in the flight. With
  the gate off (`--no-att-gate`), 3-4 m and under 0.1 m/s, and the spaceshot's W1 horizontal goes
  from 18 to 10 m. The synthetic vehicle stays nose-up all the way down, which a real one does not,
  so the late numbers overstate it; the drift starts in the coast. GNSS never measures attitude:
  a tilt error points the thrust or drag slightly sideways, the horizontal velocity drifts by
  tilt x specific force (0.5 m/s per second for 1 degree at 3 g), the Doppler sees the drift, and
  the covariance built up by the propagation maps the velocity correction back onto the tilt. Only
  tilt across the specific force is observable, and only while there is some: heading about a
  vertical thrust axis never is, and nothing is in the zero-g coast. The gate removes the
  unobservable heading and the observable tilt alike. Removing only the rotation about the specific
  force (`--att-gate heading`, the fix proposed in #1531) keeps nearly all of the benefit: 3.6 and
  4.3 m late in the gentle flight, 8.5 m through the spaceshot's gap, against 2.7, 4.2 and 9.8 m
  with no gate at all.

The receiver's own fix, fused as the flight filter does today, is the worst of the lot: 60-400 m
in the windows, where there is nothing to fuse, and 180 m and 8 m/s through the gentle boost,
because the fix itself lags there (49 m and 11 m/s on its own). Its pad height is 11.6 m low.

**Power save** (spaceshot runs 1 and 2, `--rr-lag 0.08`): 15-74 m of altitude on the pad, and a
vertical velocity of 0.5-0.8 m/s in the coast and descent but 1.2-3.9 m/s in W3, where the lag
moves -- power save costs the raw data as well as the fix.

Next, for the altitude floor: a per-satellite pseudorange-bias state (the smoothing error and the
re-lock transients are per satellite and slow), or the carrier delta-range TcEkf already has, once
its ~0.2 s lag is modelled like the Doppler's. The lag itself is to be re-measured on the first
flight that logs the PX1105R's raw data (#1528).

Caveats: one run per configuration, on a GPS-L1-only, noiseless simulated sky. The IMU is
synthetic -- no vibration, spin or misalignment; a worse one (5 mg turn-on bias, 0.1 deg/s gyro
bias, 0.3 % scale factor) moves the spaceshot's W1 from 16 to 20 m. The lag and the pseudorange
smoothing are this receiver's, measured on this rig; the 4.2 m/s rate offset, and the re-lock
transients it causes, are the rig's. A
Doppler-lag state (`--p0-rr-lag`) exists but was left off: GNSS alone cannot separate it from the
pseudorange errors.

    cd tinkerrocket-sim
    PYTHONPATH=src python3 scripts/tc_ekf_cocom.py \
        ../tools/gnss-cocom/sdr/results/px1105r_gentle_alt_pad600_smooth_gain3_nav9_el3_pmnormal_run2.log.gz \
        ../tools/gnss-cocom/sdr/results/neo_m8t_gentle_alt.scenario.json \
        --mode ins --mode ca --mode own --gravity wgs84 --earth-rate --no-att-gate \
        --plot-lims 100,5,25 --plot ../tools/gnss-cocom/sdr/results/figures/px1105r_track_gentle.svg

(`--mode ins-lc` adds the "today" row; it always runs the flight filter's own mechanization. The
spaceshot figure is the same with its capture and scenario and `--plot-lims 100,5,30`; the tables
are medians over `--seed 1..3`. The raw-measurement figure and the lag fits:
`scripts/raw_residuals_cocom.py CAPTURE SCENARIO --plot figures/px1105r_raw_errors.svg`.)

In `TcEkf` (defaults unchanged, so `tc_ekf_eval.py` and the other scripts behave as before):
`rr_lag_s` (and an optional lag state), a Doppler clock-rate offset state (`p0_rate_ofs`), the
constant-acceleration GNSS-only model (`enable_kinematic_acceleration`, `kin_gravity`,
`kin_acc_tau`), `gravity_model="wgs84"` (`gravity_wgs84`, also used by the levelling update) and
`earth_rate=True` for the IMU mechanization, and `seed_gyro_bias`. The gravity-gradient Jacobian
term now has the C++'s sign (+2g/R, #1154).

## Experiments still owed on the first four receivers

Only the Air530 was ever run against the dwell scenarios. Every other part's
thresholds are **ramp-derived**, and the central finding of this work is that a
ramp measures a latent receiver's lag rather than its threshold. That lesson was
applied to the Air530 and not carried back.

    PX1125R   19 captures   dwell scenarios: NONE
    SAM-M10Q   4 captures   dwell scenarios: NONE
    ZED-F9P    2 captures   dwell scenarios: NONE
    NEO-M8T    4 captures   dwell scenarios: NONE
    Air530    11 captures   all five

**How much it matters.** Measured shut lags on `gentle_alt`, converted into the
units of the window they occur in (w1 crosses velocity at a net 14.1 m/s^2, w3 at
9.4 m/s^2, w2 crosses altitude at 218 m/s):

| | w1 vel | w2 alt | w3 vel | velocity smear, w1 / w3 | altitude smear |
|---|---|---|---|---|---|
| PX1125R | 0.7 s | 0.3 s | 0.5 s | 10 / 5 m/s | 65 m |
| SAM-M10Q | 0.4 s | 0.7 s | 0.9 s | 6 / 8 m/s | 150 m |
| ZED-F9P | 0.7 s | **3.3 s** | 0.5 s | 10 / 5 m/s | **720 m** |
| NEO-M8T | 0.7 s | 0.3 s | 0.5 s | 10 / 5 m/s | 65 m |

Corrected 2026-09-27: this table first converted with 29.4 m/s^2, the boost's
thrust rather than its net acceleration, and gave 18-24 m/s and 60-660 m; the
velocity lags are also 0.1 s longer against the retimed truth, whose velocities had
read 0.05 s late.

The reported velocity brackets are 8-14 m/s wide, so 5-10 m/s of lag is up to most of a
bracket -- and it biases **both** edges the same way, because the receiver holds a
fix past the true threshold and then blocks late. The numbers may sit a few m/s
high, not merely uncertain. The combined `(514, 515]` bracket is real
arithmetic across the edges but is tighter than the method supports for the four
ramp-measured parts.

In priority order, all scenarios already built in `c8/` (21 GB, no regeneration):

1. **`vel_stair` on the four ramp-measured parts.** 90 s dwells at 495-530 m/s
   remove the lag entirely. ~16 min each. Either confirms 514-516 or shows the
   true threshold is up to ~10 m/s lower. Run the staircase descending as well as
   ascending and it also gives hysteresis, which nothing has tested.
2. **`alt_stair` on the PX1125R, SAM-M10Q and ZED-F9P.** Their 80 km figures come
   from flight profiles only. The F9P matters most: its 3.3 s altitude lag is
   720 m of smear, which is exactly why its bracket inverts and carries a footnote.
3. **Re-run the PX1125R entirely.** Its 19 captures all predate the link being
   fixed -- 5th-percentile 4 satellites and median 7, against 10-14 for
   everything measured since. It is the least trustworthy row in the table. Most
   of that starvation was its C/N0 walking 15-18 dB down and back every ~70 s
   with the rig's carrier-vs-code offset (see the C/N0 oscillation section), so
   re-run it on a carrier-corrected file (`patch_carrier_offset.py`).

Best done as each part goes back on the bench alongside a new one, since only
one receiver connects at a time.

## What this rig does not test: boost dynamics

> **Caveat (2026-09-26):** every number in this section came from stock gps-sdr-sim,
> which steps each carrier every 0.1 s, so the burn is a frequency staircase no real
> flight produces. On the smoothly swept file the LC86G's losses repeat run to run and
> sit higher up the Doppler-rate scale -- see its 2026-09-26 subsection. Re-running
> these on `c8/spaceshot_smooth.C8` is owed.

**None of the five UBX parts loses its POSITION during the burn on this bench;
the Quectel LC86G does, in all three navigation modes flown -- see its section above.
Individual satellites are a different story, and the ones a receiver loses are
exactly the ones carrying the most Doppler.** Measured on five receivers with
`boost_sats.py`, against Doppler measured from RXM-RAWX by `doppler_ref.py`;
written up in `boost_report.html` (GNSS Under Boost).

The elevation correlation below came first and is kept because it is what the
per-satellite data supports on its own. It is a proxy: for a near-vertical boost
the Doppler rate goes as `a * sin(elev)`, so elevation stands in for the thing we
think is the cause. `doppler_ref.py` measures the cause instead, and the two
agree -- measured rate matches `a * sin(elev) * 5.25` to a factor of 0.82-0.98
over twelve satellites, the residual being peak vs mean acceleration across the
burn. What the Doppler axis adds that elevation cannot:

| receiver | r(Doppler rate) | r(peak shift) | r(sin elev) |
|---|---|---|---|
| ZED-F9P | **-0.60** | -0.07 | -0.71 |
| NEO-M8T | **-0.91** | -0.74 | -0.84 |
| Quescan M10 | **-0.49** | -0.32 | -0.50 |
| Beitian BN-182 | **-0.66** | -0.49 | -0.59 |
| SAM-M10Q | **-0.63** | -0.44 | n/a: no elevation logged |

**Rate beats shift on all five**, which names the mechanism: the loop failing to
slew, not the carrier sitting far off nominal. Peak shift is a real control, not
a straw man -- it runs 1.2-6.9 kHz and is near-independent of elevation because
it is set mostly by satellite motion, so GPS:5 at 28 deg carries the largest
shift in the sky (6915 Hz) and barely suffers.

Pooling both flights on the rate axis gives a dose-response: n=125, r=-0.61,
median -13.5 dB above 300 Hz/s against 0 dB below 100 Hz/s. Only the NEO-M8T
logged RXM, but every flight replayed a byte-identical scenario file
(hash-checked by `doppler_ref.py --verify`), so the Doppler is a property of the
injected signal and applies to all five parts. Without the SAM-M10Q the archive
gives n=100, r=-0.62 and -17 dB above 300 Hz/s; the -14 dB first written here
for those four does not reproduce.

GPS:11 is the sharpest single observation. At 70 deg it presents the steepest
rate in the set; its raw measurement is steady at -98 Hz on the pad, reads
+483 Hz one second after ignition with carrier down 48->30 dBHz, vanishes from
RXM entirely for ten seconds, and returns at +6302 Hz with carrier back to
47 dBHz as the burn ends. Its rate is therefore unmeasurable and it is excluded
from the rate axis: the instrument that would measure the stimulus is the one the
stimulus disabled.

The dynamics themselves are real and correctly injected. `spaceshot` peaks at
132 m/s^2 (13.5 g), which is **695 Hz/s** of L1 Doppler rate against a peak shift
of 7 kHz, and gps-sdr-sim derives that from the trajectory the same way physics
would. Through the 12 s burn:

| | on the pad | through the burn | NO_LOCK |
|---|---|---|---|
| SAM-M10Q | 13 sats, 41 dBHz | 11 sats, 38 dBHz | 0/11 |
| ZED-F9P | 14 sats, 36 dBHz | 11 sats, 42 dBHz | 0/12 |
| NEO-M8T | 14 sats, 51 dBHz | 12 sats, 43 dBHz | 0/12 |

The satellite counts above hide what is actually happening. Doppler rate goes as
`a * sin(elevation)` -- 693 Hz/s toward zenith against 60 Hz/s near the horizon,
an 11x spread across the sky at the same instant -- and **the drop is entirely
in the high-elevation satellites**. ZED-F9P through the 13.5 g burn, every
tracked satellite:

    GPS:11   70 deg   36 -> 0 dBHz   LOST
    GPS:24   51 deg   34 -> 0 dBHz   LOST
    GPS:21   38 deg   38 -> 48
    GPS:6    31 deg   48 -> 0 dBHz   LOST
    GPS:5    28 deg   37 -> 47
    ... all nine satellites below 30 deg kept, several gaining 10-15 dB

Acceleration is the control. Same geometry, same scenario, 4.5x less
acceleration:

`boost_sats.py` recomputes this from the raw NAV-SAT in any capture, so it now
covers every receiver that reports per-satellite elevation -- four of the seven.
The other three cannot answer the question: the Air530 speaks NMEA, the PX1125R
SkyTraq binary, and the SAM-M10Q archive comes through the console diagnostic
that synthesized elevation as zero (see the planned trial below).

    python3 boost_sats.py results/zed_f9p_spaceshot.log.gz \
                          results/zed_f9p_spaceshot.scenario.json

| | peak | r(sin elev, dC/N0) | >=45 deg | <30 deg | lost |
|---|---|---|---|---|---|
| ZED-F9P spaceshot | 13.5 g | **-0.71** | **-38 dB** | +2 dB | 3 |
| ZED-F9P gentle_alt | 2.0 g | +0.35 | +3 dB | +0 dB | 0 |
| NEO-M8T spaceshot | 13.5 g | **-0.84** | **-20 dB** | -4 dB | 0 |
| NEO-M8T gentle_alt | 2.0 g | -- | +0 dB | +0 dB | 0 |
| Quescan M10 spaceshot | 13.5 g | **-0.50** | **-24 dB** | -6 dB | 0 |
| Quescan M10 gentle_alt | 2.0 g | +0.28 | +6 dB | +0 dB | 0 |
| Beitian BN-182 spaceshot | 13.5 g | **-0.59** | **-19 dB** | +4 dB | 0 |
| Beitian BN-182 gentle_alt | 2.0 g | +0.41 | +4 dB | +0 dB | 0 |

**Four independent receivers show it at 13.5 g and none shows it at 2.0 g**, where
the correlation does not merely weaken but comes back positive on every part.
Every channel is transmitted at equal power (`-p`), so the elevation dependence
has to be receiver-side. This is Doppler-rate stress on the tracking loops, and
the rig does reproduce it.

The ranking is consistent even where the outcome is not. GPS:11 (70 deg) and
GPS:24 (51 deg) are the two worst-hit satellites on all five parts -- -39/-38 on
the ZED-F9P, -16/-25 on the NEO-M8T, -15/-32 on the Quescan, -12/-26 on the
Beitian, -20/-35 on the SAM-M10Q -- but only the ZED-F9P actually drops any of
them.

Caveats: n=2 in the >=45 deg band, because that geometry simply does not put
many satellites overhead; the four parts with elevation are two u-blox modules
(M8, F9) and two that speak its UBX interface, with the SAM-M10Q, an M10 module,
added on the rate axis only; and the result is sensitive to how the burn window
and baseline are chosen -- a first pass with a slightly different window showed
no effect at all. `boost_sats.py` fixes the windows (baseline is the 60 s of pad
time ending 5 s before ignition, burn starts 1 s after) so the choice is at least
the same for every part. The earlier hand-computed pass gave -0.67 and -0.45 for
the two receivers it covered, against -0.71 and -0.84 here; the signs and the
reversal are robust, the second decimal is not.

The NEO-M8T deserves its own asterisk: it reports a flat 51 dBHz for all 13
satellites on the pad -- one distinct value, against 10 on the ZED-F9P -- so its
reported carrier is clamped at the top of its range at this injection level. Its
baseline therefore carries no per-satellite information and its delta is just the
burn value minus a constant, which is why its r moved furthest between passes
(-0.45 to -0.84). The sign still means what it says, but read that row as a
ranking of burn C/N0 by elevation rather than a change from a measured baseline.
Backing the injection level off until its pad C/N0 spreads would settle it. What makes it credible is four
receivers agreeing and the effect reversing with acceleration.

**The SAM-M10Q could not be checked this way, and now can.** Its archived
captures come through the `[COCOM] S` console diagnostic, which logged
`gnssId:svId:cno:used` and nothing else, so `cocom_fcdiag.py` synthesized
elevation as zero. The log line now carries `gnss:sv:cno:used:elev`, and the
four-field form is still accepted so the existing archive stays readable. The
trial that uses it is written up below.

So dynamics *are* measurable here, on individual channels. What the rig still
cannot reproduce is everything else that co-occurs with boost in a real flight,
and it is the combination that costs a position fix rather than a few channels:

* **Vibration.** A solid motor is broadband violence and it modulates carrier
  phase directly. The injected trajectory is smooth 10 Hz motion, interpolated.
* **Plume attenuation.** Exhaust can attenuate and scatter L-band, worst for an
  aft-facing antenna.
* **Antenna pattern and body shadowing.** A real vehicle rolls and pitches, and
  satellites sweep through pattern nulls. The injection is isotropic -- the
  horizon patch fixes the elevation mask but there is no antenna gain pattern.
* **Signal level.** The bench injects a clean 36-51 dBHz. Real flight is lower,
  and loop stress multiplies with low C/N0: a loop that holds 695 Hz/s at 45 dBHz
  can fail at 32.
* **Airframe attenuation and multipath** from a nosecone or fairing.

Read the results accordingly: this bench characterizes **the export gate**, not
the tracking loop. A part that sails through boost here may still drop lock on a
real motor, and nothing measured here contradicts that.

## Planned: where is the knee? Doppler rate vs C/N0 on a u-blox M10

**Status: designed, needs an M10 on the bench and two new scenario files.** The
LC86G has since given a knee for one part, 128-171 Hz/s, from its own RTCM MSM7
on the existing `spaceshot` file -- a clean split rather than a gradual slope, so
the hypothesis below now has one supporting case, from another vendor.

u-blox publishes a 4 g dynamics limit for the M10. The tracking loop does not
see g, it sees Hz/s, and the conversion is worst-case at zenith:

    4 g = 39.23 m/s^2      x 5.255 Hz per m/s      = 206 Hz/s at zenith
                                                   = 146 Hz/s at 45 deg
                                                   =  71 Hz/s at 20 deg

That single fact is the whole experiment. **One acceleration presents a whole
spread of Doppler rates at once**, because each satellite is projected onto the
velocity vector by its own elevation. If the receiver were acceleration-limited,
every satellite would degrade together the moment the vehicle passed 4 g. If it
is rate-limited, only the satellites above the knee degrade and the ones near
the horizon are untouched at the same instant.

The existing `spaceshot` captures already point one way. That flight holds
13.5 g -- 3.4x the published limit -- and splitting its satellites at the
4 g-equivalent rate gives, on the same vehicle at the same instant:

| | n | median dC/N0 | elevations |
|---|---|---|---|
| below 206 Hz/s | 30 | **-3.5 dB** | 5-20 deg |
| at or above 206 Hz/s | 30 | **-7.5 dB** | 26-51 deg |

(All five rate-axis parts; the four without the SAM-M10Q gave -4 and -10 dB.
The elevations are from the four that log them.)

The only thing separating those groups is where the satellite sits in the sky,
so acceleration by itself is not the variable. Binned further, the shape looks
like a knee rather than a slope:

| rate, Hz/s | n | median dC/N0 |
|---|---|---|
| 0-100 | 75 | +0.0 |
| 100-206 | 20 | -3.5 |
| 260-320 | 20 | -0.2 |
| 320-400 | 5 | -14.0 |
| 400-520 | 5 | -32.0 |

The SAM-M10Q is flat through 312 Hz/s, which is what pulls the 260-320 bin from
-5.5 dB (the four parts without it) to -0.2: the parts differ in where they
start to fade, not only in how much.

That is consistent with no meaningful loss below the 4 g-equivalent and
progressive degradation above it, but it does not establish it: 206-260 Hz/s is
empty, the top two bins are n=4, and every point above 206 Hz/s comes from the
same handful of high satellites, so rate and identity are confounded.

### The design

Fly an acceleration ladder and use the elevation spread within each flight. The
point is not more points, it is **overlap**: with this sky every rate band up to
460 Hz/s is reachable at three or more different accelerations.

| accel | zenith-equivalent | rates this sky presents |
|---|---|---|
| 2 g | 103 Hz/s | 9-97 |
| 4 g | 206 Hz/s | 18-194 |
| 6 g | 309 Hz/s | 27-291 |
| 9 g | 464 Hz/s | 40-436 |
| 13.5 g | 696 Hz/s | 61-654 |
| 15 g | 773 Hz/s | 67-726 |

**The decisive comparison is vertical, not horizontal.** 250 Hz/s is reachable
at 6 g (near zenith), at 9 g (~33 deg) and at 13.5 g (~21 deg). If loss depends
only on rate, those three coincide. If acceleration matters on its own, they
separate by acceleration -- and the direction of that separation says whether
the extra term helps or hurts.

Predictions worth writing down before running it, since they are distinguishable:

* **Rate-limited with a knee** (the hypothesis): all accelerations fall on one
  curve, flat below roughly 206 Hz/s and dropping above.
* **Rate-limited, no knee:** one curve, but degradation is gradual from zero
  with no threshold. The 100-206 band already shows -3.3 dB, which mildly
  favours this over a hard knee.
* **Acceleration-limited:** the curves separate by g at matched rate, and the
  4 g flight is clean everywhere while the 13.5 g flight is degraded everywhere.

### Trajectory

A boost is a poor stimulus for this: it holds peak acceleration only briefly and
gives about 12 samples at 1 Hz. Use a **velocity sawtooth** instead -- ramp at
+a, then -a, repeatedly -- which holds |acceleration| and therefore |Doppler
rate| constant for as long as wanted while velocity stays bounded. The rig
already has sawtooth scenarios (`saw_static_raw`, `saw_static_boosted`), so this
is a parameter change rather than new machinery.

Keep the peak velocity modest, around 1000 m/s (5.3 kHz of vehicle Doppler,
about 9 kHz once satellite motion is included). Letting velocity run to several
km/s would push total Doppler toward the edge of the acquisition search window
and confound a rate result with a range result. At 13.5 g, 1000 m/s is a 7.6 s
half-period, so a 60 s dwell gives four full cycles and roughly 60 epochs per
acceleration.

Two files: `accel_stair` stepping 2/4/6/9/13.5 g with 60 s at each and a coast
between, and `accel_stair_hi` adding 15 g if the 13.5 g result warrants it.

### Controls, and the traps this rig has already hit

* **Check for C/N0 clamping first.** The NEO-M8T reports a flat 51 dBHz for all
  13 satellites at the level used so far, which destroys the baseline and makes
  its delta a ranking rather than a change. Run `gain_sweep.py` and pick the
  lowest level that still tracks the full constellation *and* shows spread in
  reported C/N0. Verify the spread before flying, not after.
* **Doppler reference.** Try `CFG-MSGOUT-UBX_RXM_RAWX` on the M10 first; if it
  refuses (raw measurements are a timing/high-precision feature and the M8T is
  the part that has it here), fly each scenario on the NEO-M8T as well. The
  scenario file is byte-identical across runs, so the M8T's measured Doppler is
  the reference for the M10's C/N0 -- the pattern `doppler_ref.py` already uses
  and `--verify` already hash-checks.
* **Apply `patch_horizon.py`** so satellites below the horizon are not
  transmitted; otherwise the elevation axis includes signals no real antenna
  would see.
* **Equal channel power** (`-p`) so no elevation-dependent level creeps in.
* **`preflight()` before every run.** The Beitian silently reverted to NMEA
  with DYNMODEL=0 between its gain sweep and its flight. The same revert here
  would substitute a different dynamic model mid-experiment.
* **Expect the position to blank** each time the sawtooth crosses 515 m/s. That
  is fine: NAV-SAT keeps reporting C/N0 and elevation while the gate withholds
  position, which is what the whole per-satellite analysis relies on.

## Answered: the SAM-M10Q loses its high-rate satellites too

**Status: answered 2026-09-25 from the archived captures; no bench run was
needed.** The plan below assumed the archive could not answer, because the
console diagnostic logged no elevation. The measured-Doppler reference does not
need elevation: it is keyed by satellite, and the SAM-M10Q flew the same
byte-identical files as the NEO-M8T (`doppler_ref.py --verify` now checks five).
Its archive is stamped with the flight computer's uptime, which the altitude fit
in `boost_sats.py` cannot reach, so the clock comes from the receiver's own
iTOW instead (`align_by_tow`, -1635.5 s on `spaceshot`). It ran at 18 Hz; the
console printed once a second.

13.5 g, change from the pad to the burn, by measured Doppler rate:

| satellite | rate, Hz/s | dC/N0 |
|---|---|---|
| GPS:29 ... GPS:6 (ten satellites) | 54-312 | -5 to +8 dB (median 0) |
| GPS:21 | 378 | -13 dB |
| GPS:24 | 501 | **-35 dB** (40 -> 5 dBHz) |
| GPS:11 | not measurable | -20 dB |

None of the 13 was lost outright; at 2.0 g none moved (median -1 dB). **The
prediction held**: the satellites above 45 deg lost 20-35 dB (GPS:11 a little
under the band), everything below 30 deg stayed within a few dB, and the effect
vanished at 2.0 g. It is also the one part where the fade starts late: flat to
312 Hz/s, half again the 206 Hz/s its 4 g rating implies overhead.

The original plan follows, kept for a part whose archive has no per-satellite
data.

The elevation-dependent loss through the burn was measured on the ZED-F9P and
NEO-M8T, which could be tapped directly for UBX. The SAM-M10Q is the part most
worth knowing about and is the one part where it has never been tested, because
its console diagnostic did not log elevation. It does now.

**Prediction.** If the SAM-M10Q behaves like the other two, satellites above
roughly 45 deg should lose 25-35 dB through the 13.5 g burn while everything
below 30 deg holds, and the effect should vanish on `gentle_alt` at 2.0 g. If it
does *not* show the effect, that is more interesting than if it does: the three
parts would then differ in tracking-loop behavior under acceleration, which no
datasheet reports and which matters directly for a boost phase.

**Procedure.**

1. Rebuild and flash with the diagnostic on. It is compile-gated and off by
   default:

       idf.py -B build_cocom -DTR_BOARD_V8=1 -DTR_GNSS_COCOM_DIAG=1 build flash

2. Confirm the new field is present before spending a flight on it -- the log
   line should read `0:22:20:0:41`, five fields, not four:

       grep -m2 'COCOM. S' <console capture>

   Worth the check: **no CI job compiles this block.** It is `#if`-gated off by
   default, so the firmware build that runs on every push never sees it, and a
   mistake inside it surfaces only at the bench. It was built locally with
   `-DTR_GNSS_COCOM_DIAG=1` when the field was added, but nothing keeps it that
   way.

3. Fly both standard profiles, so the acceleration control is available:

       ./run_fc.py -s spaceshot  -x 12
       ./run_fc.py -s gentle_alt -x 12

   Gain 12 was the working point for this part on the radiated cage link.

4. Analyse exactly as the other two were: median C/N0 over the 40 s pad hold
   before ignition against the burn window, per satellite, correlated against
   `sin(elevation)`. Compare the 13.5 g flight to the 2.0 g one; the reversal is
   what carries the result, not either number alone.

**Watch for two things that bit this analysis the first time.** The result is
sensitive to how the burn window and baseline are chosen -- a first pass with a
slightly different window showed no effect at all -- so fix the windows before
looking at the answer rather than after. And the >=45 deg band held only two
satellites on the other parts, which is a geometry limit rather than a sampling
choice: `best_geometry.py` can pick a site and hour that put more satellites
overhead, and doing that first would make the result far stronger.
