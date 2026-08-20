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
