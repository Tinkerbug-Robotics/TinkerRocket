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

## Receivers compared

Generated from `results/receivers.json` by `receiver_table.py` -- edit the JSON
and re-run it rather than hand-editing this table or the one in `report.html`.

| Receiver | Bands | Path | Runs | Velocity gate | Altitude gate | 18 km gate | Limits combined | Re-open latency | Sats min / median |
|---|---|---|---|---|---|---|---|---|---|
| SkyTraq PX1125R | L1 + L5 | conducted | 9 | 510-517 m/s | 79.90-80.20 km | none | independent | 0.0-1.5 s | 2 / 7 |
| u-blox SAM-M10Q | L1 (GPS/GAL/BDS/GLO) | radiated, Faraday cage | 2 | 514-516 m/s | ~80.16 km | none | independent | 0.0-1.1 s | 4 / 13 |
| u-blox ZED-F9P (ArduSimple) | L1 + L2 (L1 used here) | conducted | 2 | 514-518 m/s | 80.22-80.48 km † | none | independent | 0.1-1.0 s | 6 / 13 |

**SkyTraq PX1125R** (2026-08-19, ~70 dB pad + DC block into RF_IN, TX gain 44-47): Satellite starvation was the dominant confound: windows that took 12-33 s all had two satellites, which is re-acquisition rather than the gate. Also carried a ~15 dB, ~82 s C/N0 oscillation that was never identified.

**u-blox SAM-M10Q** (2026-08-20, 100 dB pad, L1 quarter-wave, TX gain 12): Never fell below 4 satellites in either flight, so every withheld epoch is the gate rather than a link failure. No periodic C/N0 oscillation appeared (r = 0.02 and 0.11).

**u-blox ZED-F9P (ArduSimple)** (2026-08-20, 70 dB pad, TX gain 38): Arrived configured as a fixed-position RTK base (CFG-TMODE-MODE=2) and therefore did not navigate at all: it tracked GPS at a median 41 dBHz with valid ephemeris, had four or more usable satellites in 335 of 420 epochs, and still reported used_in_fix=0 while holding its surveyed base coordinates. Disabling base mode fixed it immediately. Uniquely among the three parts it is slow to CLOSE the altitude gate -- +2.3 s and +3.3 s across the two flights, about 400-600 m of overshoot at climb speed -- which is why its altitude bracket inverts. Its descending edges are crisp and agree at ~80.1 km.

&dagger; marks an inverted bracket: a value that still held a fix sitting above
one that was withheld, which happens when a receiver is slow to close the gate.
It is a latency, not a threshold difference.

Across all three parts the velocity limit brackets to **(514, 516] m/s** and the
altitude limit to **80 km**, with no 18 km gate anywhere and both limits always
independent. More parts are planned against the same trajectories. To add one:
fly `spaceshot` and `gentle_alt`, archive the capture and its scenario here, add
an entry to `receivers.json`, and regenerate.
