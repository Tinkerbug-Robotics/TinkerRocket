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

Every window where four or more satellites were tracked re-opened in 0.7-1.5 s,
on both gates. Windows that took 12-33 s all had two satellites: that is
re-acquisition, not the gate. `recovery.py` prints the satellite count beside
the latency so the two cannot be confused.

Superseded and deleted: the first flight captures (`*_a1`). Their trajectories
deployed the main parachute at 3 km while still doing 560 m/s, which produced
~7000 m/s^2, reversed the velocity in an explicit integrator, and lofted the
vehicle back to 28 km. Both the flights and the tracking losses in them were
artefacts. The profiles now fly a drogue from apogee and the main at 600 m AGL.
