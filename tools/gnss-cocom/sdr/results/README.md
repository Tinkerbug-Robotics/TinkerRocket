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

## Air530 / AT6558R, conducted (2026-08-20)

GPS + BeiDou, **NMEA 0183 at 9600**, on a CP2102 USB-UART through the same 70 dB
pad. TX gain **32**. No configuration needed or possible: it ignores `$PCAS01`,
so it cannot be moved off 9600 by that command. NMEA is sufficient here -- GGA
reporting no fix while GSV still lists satellites *is* the BLOCKED/NO_LOCK
distinction, and ground truth comes from the injected trajectory, not the
receiver's own speed.

| Capture | Boost | Result |
|---|---|---|
| `air530_spaceshot` | 15 g | w1/w2 never re-opened; w3 took **134.1 s**. Held a fix at **1334 m/s** before closing |
| `air530_gentle_alt` | 3 g | w1/w2 never re-opened; w3 took **62.9 s**. Blocked for 76% of the flight |
| `air530_gentle_alt_coldrun` | 3 g | first attempt, kept for the record: acquired only at t~200 s, so its recovery numbers are contaminated |

**This part is the outlier.** It enforces both limits, and the signature is the
usual one -- position withheld while a median of 12-13 satellites stay tracked at
~50 dBHz. But both gate edges are latent. It closes **1.6-8.9 s late**, which on
a 15 g boost means publishing a valid fix at two and a half times the limit, so
its velocity bracket measures its latency rather than a threshold. And it
re-opens **31-134 s** late, twice not at all before the next exceedance, against
0.0-1.5 s everywhere else.

**An 18 s periodic dip, and why it is not the bench.** The Air530's satellite
count dips hard and regularly while the gate is shut -- median spacing 18 s on
both flights (stdev 6 s and 11 s). Three things locate it inside the receiver:

1. The **ZED-F9P is a control**: same scenario files, same 70 dB conducted path,
   **zero** periodic dips. Its only blank epochs are three at t=850-862 s on
   `gentle_alt`, after the 848 s file ends -- the transmitter stopping.
2. **All twelve satellites blank in the same epoch** and return together within
   three. Attenuation is graded; this is not. The receiver emits GSV sentences
   with an *empty* C/N0 field, which is a reporting decision, not a signal level.
3. It occurs **only while position is withheld** -- t=197-437 s on `spaceshot`,
   nothing before launch and nothing after the fix returns at 560 s.

18 s is three GPS subframes (the ephemeris span, subframes 1-3), so the part may
re-validate ephemeris on that cadence and blank C/N0 while it does. That is a
hypothesis; the three observations stand without it. Figure:
`results/figures/air530_dip_periodicity.svg`.

This mattered: it is exactly what made `recovery.py` misreport this receiver.

Two things this run fixed in the tooling, both of which had been quietly wrong:

* `recovery.py` judged "was the receiver starved?" on the **minimum** satellite
  count across the wait. The Air530 drops a single GSV set about every 70 s, and
  that one transient epoch pulled the minimum to 1 and made the tool dismiss a
  real slow gate as "re-acquisition time, not the gate". It now judges on the
  median and reports what fraction of the wait was actually starved. Both u-blox
  parts reproduce their previous latencies exactly.
* `align_start.py` only parsed UBX. It now reads NMEA GGA too, working in
  seconds-of-day so one routine serves every receiver. Note the Air530's RMC date
  reads **2007** rather than 2026 -- exactly 1024 weeks, a GPS week-number
  rollover -- so pass `-t` explicitly rather than trusting the date it reports.

## Receivers compared

Generated from `results/receivers.json` by `receiver_table.py` -- edit the JSON
and re-run it rather than hand-editing this table or the one in `report.html`.

| Receiver | Bands | Path | Runs | Velocity gate | Altitude gate | 18 km gate | Limits combined | Re-open latency | Sats min / median |
|---|---|---|---|---|---|---|---|---|---|
| SkyTraq PX1125R | L1 + L5 | conducted | 9 | 510-517 m/s | 79.90-80.20 km | none | independent | 0.0-1.5 s | 2 / 7 |
| u-blox SAM-M10Q | L1 (GPS/GAL/BDS/GLO) | radiated, Faraday cage | 2 | 514-516 m/s | ~80.16 km | none | independent | 0.0-1.1 s | 4 / 13 |
| u-blox ZED-F9P (ArduSimple) | L1 + L2 (L1 used here) | conducted | 2 | 514-518 m/s | 80.22-80.48 km † | none | independent | 0.1-1.0 s | 6 / 13 |
| Air530 (AT6558R) | L1 (GPS + BeiDou) | conducted | 2 | 538-1334 m/s † | -- | none | independent | 31.0-134.1 s | 0 / 12 |

**SkyTraq PX1125R** (2026-08-19, ~70 dB pad + DC block into RF_IN, TX gain 44-47): Satellite starvation was the dominant confound: windows that took 12-33 s all had two satellites, which is re-acquisition rather than the gate. Also carried a ~15 dB, ~82 s C/N0 oscillation that was never identified.

**u-blox SAM-M10Q** (2026-08-20, 100 dB pad, L1 quarter-wave, TX gain 12): Never fell below 4 satellites in either flight, so every withheld epoch is the gate rather than a link failure. No periodic C/N0 oscillation appeared (r = 0.02 and 0.11).

**u-blox ZED-F9P (ArduSimple)** (2026-08-20, 70 dB pad, TX gain 38): Arrived configured as a fixed-position RTK base (CFG-TMODE-MODE=2) and therefore did not navigate at all: it tracked GPS at a median 41 dBHz with valid ephemeris, had four or more usable satellites in 335 of 420 epochs, and still reported used_in_fix=0 while holding its surveyed base coordinates. Disabling base mode fixed it immediately. Uniquely among the three parts it is slow to CLOSE the altitude gate -- +2.3 s and +3.3 s across the two flights, about 400-600 m of overshoot at climb speed -- which is why its altitude bracket inverts. Its descending edges are crisp and agree at ~80.1 km.

**Air530 (AT6558R)** (2026-08-20, 70 dB pad, TX gain 32): The outlier, and the reason the slow flight matters. It enforces both limits -- position is withheld while 12-13 satellites stay tracked at ~50 dBHz -- but both edges are badly latent. It is 1.6-8.9 s LATE to close, so on the 15 g boost it published a valid fix at 1334 m/s, two and a half times the limit, which is why its velocity bracket inverts rather than measuring anything. Re-opening is worse: 31 s, 63 s and 134 s across the two flights, and two windows never re-opened at all before the next limit was exceeded, against 0.0-1.5 s for the other three parts. Its RMC date also reads 2007 rather than 2026 -- exactly 1024 weeks, a GPS week-number rollover -- though time-of-day is correct so the measurement is unaffected. Ignores $PCAS01, so it cannot be moved off 9600 baud by that command. Its satellite count also dips on a tight 18 s cycle while the gate is shut -- every satellite blanking its C/N0 field in one epoch and returning within three. Pinned to the receiver, not the bench: the ZED-F9P recorded zero periodic dips on the same files through the same conducted path.

&dagger; marks an inverted bracket: a value that still held a fix sitting above
one that was withheld. On the F9P that is a 2-3 s closing lag; on the Air530 the
latency is so large the bracket carries no threshold information at all.

Across the four parts the velocity limit brackets to **(514, 516] m/s** and the
altitude limit to **80 km**, with no 18 km gate anywhere and both limits always
independent. What varies enormously is **re-open latency**: under 1.5 s on three
parts, 31-134 s on the Air530. More parts are planned against the same
trajectories. To add one: fly `spaceshot` and `gentle_alt`, archive the capture
and its scenario here, add an entry to `receivers.json`, and regenerate.
