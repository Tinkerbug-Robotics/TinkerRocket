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
sits far below the export altitude, and `alt_stair_vlow` (13 km, 156 m/s)
crosses no export limit at all.

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

Generated from `results/receivers.json` by `receiver_table.py`. The whole of
`report.html` is likewise generated, by `build_report.py`, from that same JSON
plus the archived figures in `results/figures/`, and so is its companion on boost
dynamics, `boost_report.html`. Edit the data and regenerate --
neither table nor report should be hand-edited.

| Receiver | Path | Update rate | Velocity gate | Altitude gate | Limits combined | Re-open latency |
|---|---|---|---|---|---|---|
| SkyTraq PX1125R | conducted | 1 Hz | 515 m/s | 80 km | independent | 0.0-1.5 s |
| u-blox SAM-M10Q | radiated, Faraday cage | 18 Hz | 515 m/s | 80 km | independent | 0.0-1.1 s |
| u-blox ZED-F9P (ArduSimple) | conducted | 1 Hz | 515 m/s | 80 km † | independent | 0.1-1.0 s |
| Air530 (AT6558R) | conducted | 1 Hz | none to 900 m/s | 10 km ‡ | n/a -- no velocity gate | n/a |
| u-blox NEO-M8T | conducted | 1 Hz | 515 m/s | 50 km § | independent | 0.9-3.1 s |
| Quescan M10 | radiated, Faraday cage | 1 Hz | 515 m/s | 80 km † | independent | 0.1-5.0 s |
| Beitian BN-182 | radiated, Faraday cage | 1 Hz | 505 m/s ¶ | 80 km † | independent | 0.5-11.3 s |
| Quectel LC86G, Balloon mode | radiated, Faraday cage | 10 Hz | 500 m/s ‖ | 81 km ‖ | independent | 0.0-0.6 s |

† Slow to close: this part held a fix 2-3 s past the limit on both flights, about 400-600 m of overshoot above 80 km with position still being published. The threshold itself is normal.

‡ Not an export gate. This ceiling sits below the COCOM altitude, and the receiver stops publishing there for reasons unrelated to export control.

§ The u-blox dynamic model's own altitude ceiling, not an export gate. Airborne <4 g is specified at 50,000 m; no u-blox model goes higher, so this part's export behavior above it cannot be measured.

¶ Rests on a single closing edge, so it is bracketed only to the width of one navigation epoch. This part was slow enough to re-open that the gate had not cleared before the next window, leaving no fix to close again.

‖ Enforced by muting ALL output -- NMEA, acknowledgements and raw measurements -- rather than by withholding the position while satellites are still reported, so on the wire it looks like a dead receiver until it comes back. It acts on the receiver's own estimates, 500 m/s and 80.0 km, stopping within 0.1 s of passing either and returning within 0.1 s straight into a valid fix. Its own altitude read about 0.9 km low near 80 km on this bench, which puts the limit near 81 km against the injection.

**SkyTraq PX1125R** (2026-08-19, ~70 dB pad + DC block into RF_IN, TX gain 44-47): Satellite starvation was the dominant confound: windows that took 12-33 s all had two satellites, which is re-acquisition rather than the gate. Also carried a ~15 dB, ~82 s C/N0 oscillation that was never identified.

**u-blox SAM-M10Q** (2026-08-20, 100 dB pad, L1 quarter-wave in Faraday cage, TX gain 12): Never fell below 4 satellites in either flight, so every withheld epoch is the gate rather than a link failure. No periodic C/N0 oscillation appeared (r = 0.02 and 0.11).

**u-blox ZED-F9P (ArduSimple)** (2026-08-20, 70 dB pad, TX gain 38): Arrived configured as a fixed-position RTK base (CFG-TMODE-MODE=2) and therefore did not navigate at all: it tracked GPS at a median 41 dBHz with valid ephemeris, had four or more usable satellites in 335 of 420 epochs, and still reported used_in_fix=0 while holding its surveyed base coordinates. Disabling base mode fixed it immediately. It is slow to CLOSE the altitude gate -- +2.3 s and +3.3 s across the two flights, about 400-600 m of overshoot at climb speed -- which is why its altitude bracket inverts. It was the only one of the first three parts to do so; the Quescan M10, measured later, closes as slowly (+2.3 s). Its descending edges are crisp and agree at ~80.1 km.

**Air530 (AT6558R)** (2026-08-20, 70 dB pad, TX gain 32): EVERYTHING FIRST RECORDED FOR THIS PART WAS WRONG, and dwell tests corrected it. It has NO velocity gate: it held a fix to 900 m/s at 5 km on t1_velramp (reporting 899), and 100% of epochs at every 90 s dwell from 495 to 530 m/s on vel_stair. What it has is an ALTITUDE ceiling at 10-11 km -- 100% fix at 8 km, 91% at 10 km, 0% at 11/12/13 km on 90 s dwells, and 9.90->10.25 km on a 354 m/s ramp with 11 satellites either side. That ceiling is far below the COCOM altitude, so it is not an export gate at all. The flight profiles read as a latent velocity gate only because they cross 10 km at high speed: the spaceshot transition happens while speed is DECREASING (1334 -> 1304 m/s) as altitude rises through 9.83 -> 11.15 km, which no velocity gate can do. Re-open latency is not defined for this part because there is no COCOM gate to re-open: on blockdur it held a fix at 560 m/s for 148 continuous seconds, dropping only 1 s at the sharp 130 m/s^2 transition. The 31-134 s 'recoveries' seen on flights were simply the vehicle descending back through the 10-11 km ceiling. The 18 s C/N0 blanking accompanies withholding (19/677 epochs while withholding vs 0/170 while publishing on gentle_alt).

**u-blox NEO-M8T** (2026-08-20, 70 dB pad, TX gain 38): Position is gated at 50 km, but by the u-blox DYNAMIC MODEL rather than by COCOM: airborne <4g is specified at 50,000 m and measured here at 49.80-50.15 km on an altitude-only ramp at 354 m/s. Proved by moving the model -- switching to portable dropped the same ceiling to 5.04 km. No u-blox model goes above 50 km, and airborne <4g is already both the highest ceiling and the highest velocity limit, so this part cannot be made to navigate higher. The ceiling is real for flight use and is recorded as such, but it is NOT an export gate, and its true COCOM altitude behavior is unmeasurable because the model stops it first. Note the SAM-M10Q and ZED-F9P held fixes at 68.8 km on the same model 8, so this is an M8-generation behavior. It also explains what looked like two failed recoveries on gentle_alt: those gaps sit at 68-80 km, above the ceiling, while the window that cleared at 29 km recovered in 0.9 s.

**Quescan M10** (2026-08-28, L1 antenna in Faraday cage, TX gain 26): It answers u-blox's UBX interface down to SEC-UNIQID; its MON-VER reports ROM SPG 5.10, hardware 000A0000, PROTVER 34.10 and no MOD= string. Gate behavior is in family -- velocity around 515, altitude at 80 km, limits independent -- but it is the slowest part measured on the ALTITUDE gate: +2.3 s to close and 4.7-5.0 s to re-open, against 0.7-1.7 s elsewhere, which is why both its brackets inverted. Flown on the same ephemeris, start time and launch site as the SAM-M10Q, ZED-F9P and NEO-M8T, so its satellite geometry is directly comparable rather than merely similar. One velocity edge closed a single epoch early, blocking at 510 m/s, while every other edge on this part is consistent with 515; at 29 m/s^2 an epoch is 29 m/s wide, so that is quantization rather than a lower threshold.

**Beitian BN-182** (2026-08-28, L1 antenna in Faraday cage, TX gain 20): It shares the Quescan's UBX interface -- its MON-VER answer is identical, its chip serial differs (dee2c50fbf vs c8bf908e28) -- but it is a different part, flown on the same ephemeris, start time and launch site. It behaves like its MIRROR IMAGE on recovery: fast on altitude (1.0 s) and slow on velocity (10.1 s), where the Quescan is slow on altitude (5.0 s) and fast on velocity (0.1 s). On three of four velocity windows it does not re-open when speed drops below 515 but waits until 328-410 m/s, with 9-13 satellites held throughout, so it is the gate rather than re-acquisition. Transmit level is NOT the cause: a control flight at gain 26, matching the Quescan, reproduced every latency to the tenth of a second (0.5 / 1.0 / 10.1 s) and every shut lag. Besides the part itself, what differs and was not controlled is configuration in the modules' own flash -- this one runs GPS+Galileo+BeiDou with GLONASS off, the Quescan has GLONASS enabled, and CFG-NAVSPG holds more than the dynamic model. The practical lesson is that a shared interface does not predict gate behavior: two modules that answer UBX identically differ by two orders of magnitude on velocity-gate recovery, and no datasheet says which you are buying.

**Quectel LC86G, Balloon mode** (2026-09-24, on-board patch antenna in Faraday cage, TX gain 0): The same module after $PAIR080,3. The boost is tracked (climb rate within 1-2 m/s of the injection through the 3 g ascent) and the limits are clean and independent: ALL output stops at 500 m/s on its own speed estimate (fired at 9.3 km) and at 80.0 km on its own altitude (fired at 173 m/s), and returns within one 0.1 s epoch straight into a valid fix. Its own altitude reads about 0.9 km low near 80 km, so against the injection the altitude limit sits near 81 km. 500 m/s matches neither COCOM's 515, MTCR's 600 nor the datasheet's 490. At 15 g it still loses every channel at ignition; its RTCM MSM7 shows the four with a Doppler rate at or below 128 Hz/s re-locking within 2 s and every one at or above 171 Hz/s staying lost through the burn, so it has no fix until the descent brings the vehicle back under 500 m/s. Its altitude drifts low through the flight, 2.6 km by landing with velocity still right to 0.5 m/s; the Quescan M10 drifts 1.5 km on the same file, so most of that is the bench.

Every part that gates velocity at the COCOM figure brackets it to
**(514, 516] m/s**, and wherever an altitude gate is genuinely COCOM it sits at
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
stays within 0.5 m/s of the injection the whole way, and its east position drifts
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

**5. The LC86G in Normal mode never re-acquires after its mute.** 580 s of 3 g
descent without a valid fix, on the same signal the module tracked at 45 dBHz in
Balloon mode. The likely cause is its own navigation state -- it never believed
the climb, so its predicted Doppler was off by kilohertz -- but that is inferred
from the pattern. The Normal-mode flights had no RTCM MSM7 on; re-flying one with
it would show each channel's measured Doppler against where it should have been.
Drone mode fails the same way and that explanation does not cover it: it followed
the climb, and after its mute it kept a median of 13 channels at 42 dBHz, yet
reported no fix and no satellites used for the ten minutes to landing. Its
flights had MSM7 on, so each channel's Doppler after the mute can be set against
the NEO-M8T's RXM-RAWX on the same file, which `doppler_ref.py` already reads --
no new hardware needed.

## Quescan M10, radiated (2026-08-28)

It **answers u-blox's UBX interface in full**: `ROM SPG 5.10`, hardware
`000A0000`, `PROTVER=34.10`, down to `SEC-UNIQID`, `MON-RF`, `MON-HW`,
`MON-GNSS` and `MON-COMMS` at correct payload sizes, and it reports **no `MOD=`
string**. That establishes the interface, not the part inside it. Found at **38400 baud**, the M9/M10 UART default,
emitting NMEA only. TX gain **26**, off a broad plateau: 12-13 satellites and
44-47 dBHz from gain 14 to 47, no compression at the top.

| Capture | Result |
|---|---|
| `quescan_m10_spaceshot` | 3 windows, recovered 0.5 / 5.0 / 0.1 s |
| `quescan_m10_gentle_alt` | 3 windows, recovered 0.3 / 4.7 / 0.9 s |

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

One velocity edge closed a single epoch early, blocking at 510 m/s while every
other edge on this part is consistent with 515. At 29 m/s^2 an epoch is 29 m/s
wide, so that is quantization rather than a lower threshold -- and it is why
`receiver_table.py` now estimates the threshold from the **median of every
measured edge** instead of `max(fix)`/`min(blocked)`, which one sample can drag
a whole rounding step.

## Beitian BN-182, radiated (2026-08-28)

**It shares the Quescan's UBX interface but is a different part** -- its MON-VER
answer is identical, its chip serial differs (`dee2c50fbf` vs `c8bf908e28`) --
on another vendor's board, flown on the same ephemeris, start time and launch
site. Found at
**115200 baud**, NMEA only. TX gain **20**.

| Capture | Result |
|---|---|
| `beitian_bn182_spaceshot` | 3 windows, recovered 0.5 / 1.0 / **10.1** s |
| `beitian_bn182_gentle_alt` | 3 windows, recovered **11.3** / 0.7 / **9.9** s |
| `beitian_bn182_spaceshot_g26` | control at gain 26 -- see below |

### One interface, mirror-image recovery

| | w1 velocity | w2 altitude | w3 velocity |
|---|---|---|---|
| Beitian BN-182 | 0.5 s | **1.0 s** | **10.1 s** |
| Quescan M10 | 0.5 s | **5.0 s** | **0.1 s** |

The Beitian is fast on altitude and slow on velocity; the Quescan is the
reverse. On three of four velocity windows the Beitian does not re-open when
speed falls below 515 but waits until **328-410 m/s**, with 9-13 satellites held
throughout, so it is the gate rather than re-acquisition.

**Transmit level is not the cause.** A control flight at gain 26, matching the
Quescan and identical in every other respect, reproduced every latency to the
tenth of a second and every shut lag:

    gain 26    0.5 s   1.0 s   10.1 s
    gain 20    0.5 s   1.0 s   10.1 s

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
crossing where one epoch spans 118 m/s.

Fixing that dropped the Quescan to 510 while the estimator function still said
515 -- because a **second copy** of the rule had been created inside
`receiver_table.py` and only one was updated. That is the same drift already
fixed once between this file and `replot_all.py`. `vel_cell` now delegates to
`velocity_threshold`; one implementation, called from everywhere.

The Beitian's 505 carries a footnote: it rests on a **single closing edge**,
because on the other windows the gate had not re-opened and there was no fix
left to close. One epoch at that boost is 29 m/s wide, so it is consistent with
515 but not independently resolved.

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
| `lc86g_normal_spaceshot` | Normal | lost every satellite at ignition; valid-flagged fixes 18-80 km wrong near apogee and on the descent; withheld with 13 satellites until the main |
| `lc86g_normal_gentle_alt` | Normal | climb rate near zero for 18 s of boost with a valid fix; **all output stopped at 500 m/s** (own estimate) for 56 s; no valid fix for the rest of the flight |
| `lc86g_balloon_gentle_alt` | Balloon | boost tracked to 1-2 m/s; **silent above 500 m/s and above 80.0 km** (own estimates), each lifted within 0.1 s straight into a valid fix; limits independent |
| `lc86g_balloon_spaceshot` | Balloon | lost every channel at ignition; the four at or below 128 Hz/s re-locked within 2 s, the rest stayed lost; no fix until the descent brought it under 500 m/s, then a valid one within 0.6 s |
| `lc86g_drone_gentle_alt` | Drone | climb rate within 1.3 m/s after the first 2 s of boost, altitude ~1 s late (0.45 km low at 496 m/s); **all output stopped at 500 m/s** (own estimate) for 51 s; no valid fix for the rest of the flight, with a median 13 satellites tracked |
| `lc86g_drone_spaceshot` | Drone | lost every channel at ignition and none came back in MSM7 until 242 s; withheld with a median 8 satellites tracked until 571.4 s, 3.7 s after the descent passed 10 km; that fix and every one after it right |

Each capture's `.runner.txt.gz` holds the preflight read-back that proves the
module's configuration at transmit time.

### Normal vs Balloon

Quectel's protocol specification (LC26G/LC76G/LC86G V1.4, section 2.4.24,
Tables 7 and 8) gives every navigation mode except Balloon a 10 km altitude
limitation, calls 10-50 km "cannot be guaranteed", and stops all output above
50 km; Balloon mode is limited to 80 km. The datasheet adds 490 m/s and 4 g.

- **Normal mode cannot follow a boost.** On the 3 g flight the vertical solution
  lagged from ignition: 1.23 km and 1.4 m/s reported at t=190 s against 2.17 km
  and 190 m/s injected, 4.31 km against 9.18 km at 210 s, all with a valid 3-D
  fix from 13 satellites. The correlator's guard flags 256 such epochs.
- **Both modes mute at 500 m/s on their own speed estimate** -- last output at
  498.7-499.6 m/s own (498.9 injected), first silent epoch 500.3 injected. The
  datasheet says 490; COCOM says 515.
- **Balloon mode also mutes above 80.0 km on its own altitude** (last output
  80.009 km own, 80.94 injected), and returns at 80.016 own. Against the
  injection that is ~81 km; see open question 4 for why the receiver's own
  altitude reads low up there.
- **Re-open:** Balloon mode comes back within one 0.1 s epoch straight into a
  valid fix (0.6 s after the 15 g flight's no-fix spell). Normal mode never
  re-opened on the 3 g flight (open question 5).

### Drone mode, and the missing Aviation mode (2026-09-25)

Asked for an aviation mode. Older Quectel parts offered one as mode 2 of the same
numbering; this specification marks mode 2 **reserved**, and the firmware refuses
it: `$PAIR080,2` is answered `$PAIR001,080,4` (parameter error) and `$PAIR081`
still reads the previous mode. So the third mode flown is **Drone** (5), which
the specification describes for "vertical acceleration at different flight
phases" and still limits to 10 km. `lc86_config.py --navmode` offers 0, 1, 3, 4,
5 and 7 and nothing else.

Same rig, same `.C8` files, same bridge image (the flight computer's MAC checked
before flashing, its flight image backed up and written back byte-for-byte
afterwards), `run_radiated.py --lc86 5 --rtcm msm7 --cold-start`, TX gain 0 in
the sealed cage. The pad signal came in **about 3 dB weaker** than on
2026-09-24 -- 42 dBHz median on GPS GSV and MSM7 against 45-46 -- at the same
gain. The cage was opened and re-sealed between the sessions, and the board's
placement in it is the likeliest difference.

- **The boost is tracked, but the altitude is a second old.** The climb rate
  lagged by up to 35 m/s for the first 1.8 s after liftoff (Balloon: 12 m/s)
  and then held within 1.3 m/s to the mute. The altitude fell behind in step
  with the climb rate -- 0.18 km low at 190 m/s, 0.45 km at 496 m/s -- and
  shifted by 0.98 s it fits the injection to 11 m rms over 186-210 s (322 m
  unshifted). Balloon's best shift is 0.22 s, the climb rate's 0.00 s in both.
- **Mutes at 500 m/s** like the other two (last output at 499.4 m/s injected,
  first silent epoch at 500.8), at 9.3 km.
- **Never re-opens after the mute** (3 g): output returned at 261.2 s, 51 s
  later, and every `$PQTMPVT` from there to the end of the flight has FixMode 0
  and no satellites used, while GSV shows a median of 13 tracked at
  42 dBHz. Normal mode held a median of 4 after its own mute. Open question 5.
- **At 15 g**: every channel lost at ignition; MSM7 carries none until G13 at
  242 s (50 s after burnout; GSV has single-satellite blips before that), where
  Balloon kept its four low ones. With the 3 dB deficit this does not separate
  mode from signal level. Withheld with a median of 8 tracked from 260 s until
  571.4 s, 3.7 s after the descent passed 10 km (567.7 s), then 9.78 km
  reported against 9.77 injected, and right to landing.
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
units of the window they occur in (w1/w3 cross velocity at ~29 m/s^2, w2 crosses
altitude at ~200 m/s):

| | w1 vel | w2 alt | w3 vel | velocity smear | altitude smear |
|---|---|---|---|---|---|
| PX1125R | 0.6 s | 0.3 s | 0.4 s | 18 m/s | 60 m |
| SAM-M10Q | 0.3 s | 0.7 s | 0.8 s | 24 m/s | 140 m |
| ZED-F9P | 0.6 s | **3.3 s** | 0.4 s | 18 m/s | **660 m** |
| NEO-M8T | 0.6 s | 0.3 s | 0.4 s | 18 m/s | 60 m |

The reported velocity brackets are 8-14 m/s wide, so 18-24 m/s of lag is larger
than the bracket itself -- and it biases **both** edges the same way, because the
receiver holds a fix past the true threshold and then blocks late. The numbers
are shifted up, not merely uncertain. The combined `(514, 516]` bracket is real
arithmetic across the edges but is tighter than the method supports for the four
ramp-measured parts.

In priority order, all scenarios already built in `c8/` (21 GB, no regeneration):

1. **`vel_stair` on the four ramp-measured parts.** 90 s dwells at 495-530 m/s
   remove the lag entirely. ~16 min each. Either confirms 514-516 or shows the
   true threshold is ~20 m/s lower. Run the staircase descending as well as
   ascending and it also gives hysteresis, which nothing has tested.
2. **`alt_stair` on the PX1125R, SAM-M10Q and ZED-F9P.** Their 80 km figures come
   from flight profiles only. The F9P matters most: its 3.3 s altitude lag is
   660 m of smear, which is exactly why its bracket inverts and carries a footnote.
3. **Re-run the PX1125R entirely.** Its 19 captures all predate the link being
   fixed -- 5th-percentile 4 satellites and median 7, against 10-14 for
   everything measured since. It is the least trustworthy row in the table.

Best done as each part goes back on the bench alongside a new one, since only
one receiver connects at a time.

## What this rig does not test: boost dynamics

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
