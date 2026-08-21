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
| `spaceshot_v2` | 15 g | velocity gate re-opened **1.1 s** and **1.5 s** in two separate windows, with 6 and 4 satellites |
| `spaceshot_v3` | 15 g | flown after the SMA was re-seated; 7 satellites held at 48-50 dBHz straight through the burn, gate re-opened in 1.0 s and 1.5 s |
| `spaceshot_eq` | 15 g | equator / 08:30 / complete-day ephemeris, 14 SV transmitted. Never fell below 4 satellites; all three windows recovered in 1.0-1.5 s |
| `spaceshot_horizon` | 15 g | equator 08:30 with the horizon patch: 15 SV transmitted at altitude, median 8 tracked, recoveries 1.5 / 0.0 / 1.1 s |
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
burn, 0 to 1334 m/s, losing none. The fix vanishing partway up is the COCOM gate at
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
    40N 22:30                    10         6        44     89%  1.5 / 19.0 / 1.1 s
    40N 22:30, re-seated         10         6        44     70%  1.5 / 1.0 / 146 s
    equator 08:30                14         7        45    100%  1.5 / 1.0 / 1.1 s

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

## u-blox SAM-M10Q, radiated (2026-08-20)

The rocket computer's own receiver, flown against the same two trajectories to
separate what is a rule from what is one vendor's choice. Different part,
different protocol, different signal path.

Scenario start for these captures: **2026/08/18,08:30:00** -- but do not take
that on trust, recover it:

```bash
gunzip -c results/ublox_m10_gentle_alt.log.gz > /tmp/g.log
python3 align_start.py /tmp/g.log -s results/ublox_m10_gentle_alt.scenario.json
```

The scenario each was flown against is archived beside it, `start_time` included.
TX: HackRF One r9 via PortaPack in HackRF mode, **gain 12**, 100 dB pad, L1
quarter-wave, inside a Faraday cage. Receiver: SAM-M10Q on the flight computer,
configured by flight firmware (`DYN_MODEL_AIRBORNE4g`) and never written to by
the rig. `.console.log.gz` is the raw FC console -- the primary record;
`.log.gz` is `cocom_fcdiag.py`'s conversion of it.

| Capture | Boost | Result |
|---|---|---|
| `ublox_m10_spaceshot` | 15 g | 3 windows, all recovered: 0.6 / 0.0 / 0.3 s. 618/795 epochs with a fix; satellites never below 4 |
| `ublox_m10_gentle_alt` | 3 g | 3 windows, all recovered: 0.3 / 0.7 / 1.1 s. 646/824 with a fix; **the run that brackets the velocity gate**, min 9 satellites |

Taken together the two flights corner the velocity gate between **514 and
516 m/s** and the altitude gate at **~80.16 km**. The 15 g flight alone could
only say 500-618 m/s: at 1 Hz a 15 g boost covers 118 m/s between epochs, so
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
| `zed_f9p_spaceshot` | 15 g | 3 windows, recovered 0.5 / 1.0 / 0.1 s. 590/805 epochs with a fix |
| `zed_f9p_gentle_alt` | 3 g | 3 windows, recovered 0.3 / 0.7 / 0.9 s. 517/812 with a fix |

Velocity edges bracket **(514, 518] m/s**. The altitude bracket *inverts* --
80.48 km held a fix on one flight and was blocked on the other -- because this
part lags **+2.3 s and +3.3 s on closing** the altitude gate, 400-600 m of
overshoot at climb speed. Its descending edges are crisp and agree at ~80.1 km.

## Air530 / AT6558R, conducted (2026-08-20, re-tested with dwell scenarios)

GPS + BeiDou, NMEA 0183 at 9600 on a CP2102, 70 dB pad, TX gain 32.

**The first write-up of this part was wrong in every particular and the dwell
scenarios corrected it.** It was recorded as having a latent COCOM velocity gate
smeared over 538-1334 m/s with recoveries of 31-134 s. It has no velocity gate at
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
sits below both candidate export altitudes, and `alt_stair_vlow` (13 km,
156 m/s) crosses no export limit at all.

The flights misled because a rocket crosses 10 km fast, so the ceiling fires at
almost the same moment a velocity limit would. The tell was in the data:
on `spaceshot` the fix stops while speed is **falling**, 1334 -> 1304 m/s, as
altitude rises through 9.83 km. No velocity gate fires on decreasing speed.
The "recoveries" were the vehicle descending back through the ceiling, and the
two windows that "never recovered" clear at 68-80 km, far above it.

**The 18 s C/N0 blanking** is confined to intervals where the receiver is
withholding -- 19/677 epochs while withholding vs 0/170 while publishing on
`gentle_alt`; 26/413 vs 4/419 on `spaceshot`; and on `blockdur`, where it holds a
fix almost throughout, 2 events in 988 epochs. Note this cannot be measured with
`verdict()`: a blank epoch has every C/N0 at zero, so its satellite count is zero
and the verdict is forced to NO_LOCK. Use the fix flag (`blanking.py` does).

For a flight computer this is the worst of the five: it stops publishing at
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
| `neo_m8t_gentle_alt` | 3 g flight | velocity gate 510 -> 524 m/s; w3 recovered 0.9 s; w1/w2 never, because they clear above 50 km |
| `neo_m8t_spaceshot` | 15 g flight | same pattern; w3 recovered 3.1 s at 28 km |
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
**510-524 m/s**, the same limit as every other part.

## Receivers compared

Generated from `results/receivers.json` by `receiver_table.py` -- edit the JSON
and re-run it rather than hand-editing this table or the one in `report.html`.

| Receiver | Bands | Path | Runs | Velocity gate | Altitude gate | 18 km gate | Limits combined | Re-open latency | Sats min / median |
|---|---|---|---|---|---|---|---|---|---|
| SkyTraq PX1125R | L1 + L5 | conducted | 9 | 510-517 m/s | 79.90-80.20 km | none | independent | 0.0-1.5 s | 2 / 7 |
| u-blox SAM-M10Q | L1 (GPS/GAL/BDS/GLO) | radiated, Faraday cage | 2 | 514-516 m/s | ~80.16 km | none | independent | 0.0-1.1 s | 4 / 13 |
| u-blox ZED-F9P (ArduSimple) | L1 + L2 (L1 used here) | conducted | 2 | 514-518 m/s | 80.22-80.48 km † | none | independent | 0.1-1.0 s | 6 / 13 |
| Air530 (AT6558R) | L1 (GPS + BeiDou) | conducted | 9 | none to 900 m/s | 10.00-11.00 km (not COCOM) | none | n/a -- no velocity gate | n/a | 6 / 12 |
| u-blox NEO-M8T | L1 (GPS/GLO/GAL/BDS) | conducted | 3 | 510-524 m/s | 49.80-50.15 km (dyn model) | none | independent | 0.9-3.1 s | 8 / 14 |

**SkyTraq PX1125R** (2026-08-19, ~70 dB pad + DC block into RF_IN, TX gain 44-47): Satellite starvation was the dominant confound: windows that took 12-33 s all had two satellites, which is re-acquisition rather than the gate. Also carried a ~15 dB, ~82 s C/N0 oscillation that was never identified.

**u-blox SAM-M10Q** (2026-08-20, 100 dB pad, L1 quarter-wave, TX gain 12): Never fell below 4 satellites in either flight, so every withheld epoch is the gate rather than a link failure. No periodic C/N0 oscillation appeared (r = 0.02 and 0.11).

**u-blox ZED-F9P (ArduSimple)** (2026-08-20, 70 dB pad, TX gain 38): Arrived configured as a fixed-position RTK base (CFG-TMODE-MODE=2) and therefore did not navigate at all: it tracked GPS at a median 41 dBHz with valid ephemeris, had four or more usable satellites in 335 of 420 epochs, and still reported used_in_fix=0 while holding its surveyed base coordinates. Disabling base mode fixed it immediately. Uniquely among the three parts it is slow to CLOSE the altitude gate -- +2.3 s and +3.3 s across the two flights, about 400-600 m of overshoot at climb speed -- which is why its altitude bracket inverts. Its descending edges are crisp and agree at ~80.1 km.

**Air530 (AT6558R)** (2026-08-20, 70 dB pad, TX gain 32): EVERYTHING FIRST RECORDED FOR THIS PART WAS WRONG, and dwell tests corrected it. It has NO velocity gate: it held a fix to 900 m/s at 5 km on t1_velramp (reporting 899), and 100% of epochs at every 90 s dwell from 495 to 530 m/s on vel_stair. What it has is an ALTITUDE ceiling at 10-11 km -- 100% fix at 8 km, 91% at 10 km, 0% at 11/12/13 km on 90 s dwells, and 9.90->10.25 km on a 354 m/s ramp with 11 satellites either side. That ceiling is BELOW every candidate COCOM altitude (18 km and 80 km), so it is not an export gate at all. The flight profiles read as a latent velocity gate only because they cross 10 km at high speed: the spaceshot transition happens while speed is DECREASING (1334 -> 1304 m/s) as altitude rises through 9.83 -> 11.15 km, which no velocity gate can do. Re-open latency is not defined for this part because there is no COCOM gate to re-open: on blockdur it held a fix at 560 m/s for 148 continuous seconds, dropping only 1 s at the sharp 130 m/s^2 transition. The 31-134 s 'recoveries' seen on flights were simply the vehicle descending back through the 10-11 km ceiling. The 18 s C/N0 blanking accompanies withholding (19/677 epochs while withholding vs 0/170 while publishing on gentle_alt).

**u-blox NEO-M8T** (2026-08-20, 70 dB pad, TX gain 38): Position is gated at 50 km, but by the u-blox DYNAMIC MODEL rather than by COCOM: airborne <4g is specified at 50,000 m and measured here at 49.80-50.15 km on an altitude-only ramp at 354 m/s. Proved by moving the model -- switching to portable dropped the same ceiling to 5.04 km. No u-blox model goes above 50 km, and airborne <4g is already both the highest ceiling and the highest velocity limit, so this part cannot be made to navigate higher. The ceiling is real for a flight computer and is recorded as such, but it is NOT an export gate, and its true COCOM altitude behaviour is unmeasurable because the model stops it first. Note the SAM-M10Q and ZED-F9P held fixes at 68.8 km on the same model 8, so this is an M8-generation behaviour. It also explains what looked like two failed recoveries on gentle_alt: those gaps sit at 68-80 km, above the ceiling, while the window that cleared at 29 km recovered in 0.9 s.

&dagger; marks an inverted bracket: a value that still held a fix sitting above
one that was withheld. On the F9P that is a 2-3 s closing lag; on the Air530 the
latency is so large the bracket carries no threshold information at all.

`(dyn model)` marks an altitude ceiling that is **not** an export gate --
`altitude_gate_cause` in `receivers.json` carries this.

Across the four parts that implement a velocity gate at all the limit brackets
to **(514, 516] m/s**, and wherever an altitude gate is genuinely COCOM it sits
at **80 km**. The Air530 implements neither: no velocity gate to 900 m/s, and a
10-11 km ceiling that is not an export limit, with no 18 km gate
anywhere and both limits always independent. What varies enormously is **re-open
latency**: under 1.5 s on most parts, 31-134 s on the Air530.

More parts are planned against the same trajectories. To add one: fly
`spaceshot` and `gentle_alt`, archive the capture and its scenario here, add an
entry to `receivers.json`, and regenerate. If a part stops publishing at an
unexpected altitude, run `t2_altramp` and then change the dynamic model before
believing it is COCOM.
