# #534 sim A/B: station-keep vs PN on the canonical guided scenario

**Verdict: the #534 acceptance criterion passes.** Station-keep ≤ PN apogee drift on
every discriminating seed/condition pair (35/35; 52/52 including the null wind8/latched pairs), with zero singularity behavior in
70 station-keep runs, and it holds that edge under the three off-nominal stress legs
(heading bias, GNSS denial, gust latch lottery). Gains **kp=0.8 / kd=1.5 confirmed**.
Recommended: keep `GUIDANCE_LAW_DEFAULT = PN` until a first guided flight validates
either law in the air, then flip config.h + app RocketProfile in lockstep.

- Runner (regenerates all 144 runs deterministically):
  `tinkerrocket-sim/scripts/ab534_pn_vs_station_keep.py`
- Regression pin: `tinkerrocket-sim/tests/test_scenarios.py::test_scenario_a2_station_keep_beats_pn_no_cpa`
- Plant: scenario (a) — RollyPolly III / G80T, 85° launch, apogee ~360 m,
  guidance active ~2.6 s → apogee ~8.5 s, PN target_alt 600 m (above apogee).
- All numbers were independently re-derived by a second pass over the raw JSONL;
  the analyst/verifier discrepancies were presentation errors only.

## Metric definitions (two traps, hit and corrected)

- **Effective apogee drift** = `final_horiz_offset_m` (offset at the last *guided*
  row) when guidance ran to near apogee; `max_horiz_offset_m` when tilt-latched
  early or unguided. The raw "final" for a latched run is the offset at *latch
  time* (~0.4 m at t≈2.7 s), not apogee — treating it as drift silently flatters
  latched runs.
- **`max_tilt_coast_deg`** covers the whole post-burnout window *including* after
  the benign speed-gate quit near apogee, so values above the 20° coast latch can
  coexist with `cause=speed_gate`. The latch itself samples EKF tilt only while
  guidance is active.

## Core A/B — effective apogee drift, median [min–max] over 5 seeds

| cond | PN (m) | SK (m) | unguided (m) | SK/PN | SK≤PN per seed |
|---|---|---|---|---|---|
| calm  | 13.8 [12.4–15.9] | **3.0** [0.3–3.5] | 64.9 | **0.21** | 5/5 |
| wind4 | 34.8 [34.2–35.4] | **21.4** [20.4–21.8] | 105.0 | **0.61** | 5/5 |
| wind8 | 189.7 | 188.9 | 189.8 | 1.00 | 5/5 (null — see below) |
| gust  | 32.1 [21.4–141.8] | **17.6** [2.9–140.0] | 100.4 | **0.55** | 5/5 |

- **wind8 is a null condition.** 8 m/s weathercocks the vehicle past the 20° coast
  tilt latch essentially at guidance activation (`guided_frac` 0.009, both laws,
  every seed, every gain cell). All wind8 rows fly the same ballistic arc; the
  column carries **zero law or gain information**. It is an envelope statement:
  at 8 m/s this airframe's guidance is decided by the tilt latch, not the law.
- Gust is bimodal: 3/5 seeds fly guided to apogee (SK wins big), 2/5 latch both
  laws right after burnout (wash, ≈ unguided).
- **Apogee cost: none.** Both laws land within ~1 m of each other and 0.1–11 m
  *above* the unguided same-seed flight (straighter flight). No seed anywhere has
  either law worse than unguided.
- SK actively re-centers (calm: out to ~7.8 m, back to 0.3–3.5 m at apogee);
  PN only retards outward drift (max ≈ final).

## Singularity probe — the reason station-keep exists

PN with its target set *below* apogee (reachable — exactly what a Drift-Cast/#435
offset target would create for PN, and why the aim point is station-keep-only):

| case | cause | quit time | guided window lost | late accel (m/s²) | max LOS |
|---|---|---|---|---|---|
| PN, target 200 m | **CPA** | 3.0 s / 8.5 s | **94 %** | **20.0 (clamp-pinned)** | 90° |
| PN, target 300 m | **CPA** | 5.0 s / 8.5 s | **59 %** | **20.0 (clamp-pinned)** | 91° |
| SK, same configs | speed gate | — | none | 2.7 | 1.1° |

The full signature: LOS sweeps 90° (overhead pass), commands pin the accel clamp
(19–32× the calm PN baseline of 0.6–1.1 m/s²), ~90 % fin saturation, then the
sticky `v_cl ≤ 0` quit throws away the rest of the coast. The two SK rows are
**bit-identical** to the calm SK baseline — station-keep provably ignores
`target_alt` for its law, and across all 70 SK runs in the study
`cause=cpa` occurred **zero** times (the library hard-codes `cpa_reached_=false`).
PN with the standard 600 m (unreachable) target never CPA'd either — keep it that
way; PN must never be pointed at a reachable target.

## Gain grid — kp × kd, calm, seed 7 (wind8 arm void per above)

| kp\kd | 0.75 | 1.5 | 3.0 |
|---|---|---|---|
| 0.4 | 5.0 m, tilt 13.6° | 4.2 m, 9.9° | 5.0 m, 6.4° |
| 0.8 | 1.3 m, **21.7°** | **0.32 m, 15.0°** ← | 2.3 m, 10.0° |
| 1.6 | 5.0 m, **25.9°** | 3.6 m, 18.2° | 1.1 m, 11.8° |

Best cell per kp row is the one nearest critical damping (ζ = kd/2√kp);
underdamped cells (ζ ≤ 0.42) drive coast tilt past the 20° latch threshold.
**kp=0.8 / kd=1.5 (ζ=0.84) confirmed**: drift-minimal, on the efficient frontier,
~5° of tilt margin, late accel 2.7 m/s² (no blow-up — no 1/range term in the law).
Corroborated across the 5-seed calm A/B at the same gains (0.3–3.5 m). Grid was
calm/seed-7 only in its discriminating leg — if gains are revisited, rerun under
wind4/gust, never wind8.

## Stress legs (run after the completeness critique; all closed in SK's favor)

**Heading bias** (`inject_heading_bias_deg`; the real vehicle's dominant estimator
defect — heading is uncorrected all flight, and the IIS2MDC needs hard-iron cal):

| bias | PN calm | SK calm | PN wind4 | SK wind4 |
|---|---|---|---|---|
| 10° | 15.1 | **0.6** | — | — |
| 20° | 15.2 | **1.1** | 35.4 | **20.5** |
| 45° | 15.8 | **2.5** | 35.5 (tilt-latched) | **21.1** |
| 90° | 17.2 | **5.6** | — | — |

SK's position feedback degrades gracefully: even a 90° rotation of the commanded
accel still spirals the error down. SK never lost its win at any bias.

**GNSS denial** (`enable_gnss_updates=False`, EKF coasts on IMU): SK 3.9–4.3 m vs
PN 12.7–15.8 m vs unguided 64.9 m (2 seeds). SK does *not* chase a diverging
estimate off the pad on this flight duration.

**Gust latch lottery** (20 fresh seeds × both laws): latches PN 13 / SK 10;
**SK-only latches: 0**, PN-only: 3, both: 10. On the 7 both-guided seeds SK ≤ PN
7/7 (medians 7.2 vs 24.2 m). The "one SK-only latch inverts a flight" concern does
not materialize — PN is the more latch-prone law (it tilts more because it corrects
less).

## Costs and caveats — what station-keep pays

- **Control effort is the price.** Calm: 3× PN's fin saturation (32 % vs 11 %) and
  coast tilt 14–19° vs 3–4°. Wind4: fins pinned 97–99 % of the guided window with
  late accel 16–17 m/s² (authority-limited proportional command, not divergence —
  LOS stays ≤ 10°, no quit, no oscillation, no integrator in the law).
- **Tilt margin is thin in calm+gust**: SK works 1.3–5° from the 20° coast latch
  (one gust seed crossed 20° *after* the speed gate). The lottery leg shows this
  never actually latches SK where PN survives, but the margin should be watched in
  flight logs (`GuidanceTelemData` + the tilt-latch log line).
- **"Drift" here is apogee offset.** Descent wind drift (~100 m scale per
  #191/#552) dominates the landing point; SK buys 3–20 m at apogee. State the
  benefit in those terms — this is about controlled, singularity-free guidance
  (and the #435 aim point), not about moving the landing site.
- Sim-only evidence; single airframe/motor (the fleet *is* this airframe today).
  Real-flight unknowns (servo behavior at altitude, real GNSS in boost per #249,
  real heading error statistics) motivate the flight gate below.

## Recommendation

1. **Acceptance criterion: met.** SK ≤ PN on 35/35 discriminating pairs (52/52 overall); no
   singularity behavior in any of 70 SK runs; gains 0.8/1.5 confirmed.
2. **Do not flip `GUIDANCE_LAW_DEFAULT` yet.** Keep PN as the boot default until a
   first guided flight (either law) validates the guidance stack in the air —
   consistent with project precedent (#262 mach lockout, #552). The law is
   BLE/NVS-selectable per flight, so the default costs nothing operationally.
3. Fly the first guided flight with station-keep selected via the app (it is the
   safer law by construction and now by simulation), watch coast tilt vs the 20°
   latch and fin saturation in the log.
4. Wind ≥ 8 m/s: don't fly guided — the tilt latch decides the flight, not the law.
5. #435 (Drift-Cast aim point) remains station-keep-only. Never point PN at a
   reachable target — the probe above is what happens.
