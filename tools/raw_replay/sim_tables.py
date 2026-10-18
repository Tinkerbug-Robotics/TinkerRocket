#!/usr/bin/env python3
"""Grid B's scores and tables (G4 and G5, plan section 4): each sim run of raw_replay_mosaic against the truth, the
flight design against its --unaided twin.

score_run() scores one run (sim_grid.py calls it as each run finishes and keeps only the scores):
  - the total attitude error e = 2 acos|q_est . q_true| (deg) at every filter step, per phase: boost (T+0 to the
    burnout label), the tail-off ([burnout, thrust end), reported on its own and inside coast), coast (burnout to the
    true apogee), descent (the true apogee to the touchdown), and before apogee (T+0 to the apogee): p90, p99, max;
  - the pitch-only error within +-2 s of the true apogee (max and p90 of |pitch error|);
  - the filter's vertical velocity at the true apogee (its error: the truth's is 0 there);
  - the quorum (the events JSON): the call and its latency after the true apogee, each voter's rises, the votes and
    calls cast while the true vertical velocity was above +2 m/s (and the true vertical velocity at the call: its
    margin to that bar), the guard's latch, and whether the guard should have latched (the filter's vertical
    velocity more than 10 m/s below the truth for 1 s after burnout and before the true apogee or the call, if
    earlier: the guard's own condition is against the barometer, this one against the truth);
  - the same quorum on the truth (raw_replay --truth-nav: a perfect filter with the run's own barometer and fixes):
    its call and votes -- the truth's own call, which the counters' latency alone gives;
  - the filter's health: stops, non-finite and unhealthy rows; the magnetometer and levelling counts.

    python3 tools/raw_replay/sim_tables.py GRID_OUT [--report OUT.md] [--against OTHER_GRID_OUT]

reads GRID_OUT/scores/*.json and writes the tables (markdown) and GRID_OUT/verdict.json. --against sets the no-fault
runs at magnetometer 0 beside another grid's (the antenna on the IMU against 0.3 m up the nose, say).

The arms: "design" (the mosaic build's filter with the raw vote guard applied as flight_votes.h wires it -- D1b's
wiring, assumed: no firmware tree has it yet), "unaided" (its twin, GNSS masked from the attitude) and "noguard"
(the flight design with the guard observed but not applied: the quorum RC0's main.cpp has).
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from scorelib import att_errors, pitch_deg  # noqa: E402

# plan section 4: (delta, floor) per phase
BARS = {"boost": (1.0, 2.0), "coast": (2.0, 3.0), "descent": (3.0, 5.0)}
PITCH_NEAR_APOGEE_DEG = 2.0
PRE_APOGEE_TWIN_MAX = 5.0
DESCENT_TWIN_MAX = 10.0
PASS_FRACTION = 0.95
LAT_MEDIAN_S, LAT_P95_S = 0.2, 0.5
VZ_P95_MPS, VZ_TWIN_MPS = 1.0, 0.5

PATTERN_NAMES = collections.OrderedDict([
    ("none", "No fault"),
    ("cut05", "Cut T+0.5 to T+8 (coast unaided), corrupted epoch before"),
    ("cut15", "Cut T+1.5 to T+8 (coast unaided), corrupted epoch before"),
    ("cut30", "Cut T+3 to T+8 (coast unaided), corrupted epoch before"),
    ("gap05", "Cut 0.5 s from burnout, corrupted epoch before"),
    ("gap15", "Cut 1.5 s from burnout, corrupted epoch before"),
    ("gap30", "Cut 3 s from burnout, corrupted epoch before"),
    ("stale_rx", "Stale receiver, closing tail, GNSS after"),
    ("stale_sats", "Two stale satellites, the filter not told"),
    ("settle15", "Settling after the thrust, 1.5 m/s"),
    ("settle40", "Settling after the thrust, 4 m/s, 2 m/s common"),
    ("lol0", "Loss of lock at the true apogee"),
    ("lol1", "Loss of lock 1 s before apogee"),
    ("lol2", "Loss of lock 2 s before apogee"),
    ("lol3", "Loss of lock 3 s before apogee"),
    ("coherent", "Coherent coast fault, 0 to 5 m/s down over 3 s"),
    ("coherent15", "Fast coherent step, 0 to 15 m/s down over 1 s (the Doppler gate refuses it)"),
    ("slow08", "Slow coherent ramp to 8 m/s down at apogee, 1.3 m/s a second (the guard's gap)"),
    ("slow12", "Slow coherent ramp to 12 m/s down at apogee, 2 m/s a second"),
    ("slow15", "Slow coherent ramp to 15 m/s down at apogee, 2.5 m/s a second"),
    ("slow15b", "Slow coherent ramp, 0 to 15 m/s down over 6 s from burnout"),
    ("baro_lag05", "Static port lag 0.5 s, GNSS clean (sensitivity)"),
    ("baro_lag07", "Static port lag 0.7 s, GNSS clean (sensitivity)"),
    ("baro_lag10", "Static port lag 1.0 s, GNSS clean (sensitivity)"),
])
GNSS_PATTERNS = [p for p in PATTERN_NAMES if not p.startswith("baro_lag")]
GUARD_PATTERNS = ["slow12", "slow15", "slow15b"]   # built to drive the filter more than 10 m/s low before apogee
MAG_ARMS = ["0", "+10", "-10", "+40", "-40", "none30"]
INFORMATIVE_ARMS = ["0", "+10", "-10"]   # the twin's error is small enough that the bar can fail
NOFAULT_TRUTH_DEG = 0.5                  # E3's unit test: no-fault runs within 0.5 deg of the truth
GUARD_FALSE_MPS = 3.0                    # a latch on a filter never further than this below the truth is false


def pct(x, p):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if x.size else float("nan")


# ------------------------------------------------------------------ one run

def load_truth(path):
    tr = np.load(path)
    return dict(t=tr["t"], q=tr["q"], v=tr["v"], pd=tr["pd"], phases=json.loads(str(tr["phases"])))


def score_run(csv_path, json_path, truth, meta):
    """One run's scores (a dict, JSON-ready). meta: flight, pattern, arm, mag, rc."""
    import pandas as pd
    out = dict(meta)
    ph = truth["phases"]
    t_burn, t_tail, t_apo = ph["burnout_s"], ph["thrust_end_s"], ph["apogee_s"]
    t_legal, t_touch = ph["vz_below_2mps_s"], ph["touchdown_s"]
    # when the true pitch first falls under -5 deg after burnout (for reference; the truth's own call is the quorum
    # run on it, below)
    tp = pitch_deg(truth["q"])
    kp = np.flatnonzero((truth["t"] > t_burn) & (tp < -5.0))
    t_pitch = float(truth["t"][kp[0]]) if kp.size else float("nan")
    out["truth"] = dict(burnout_s=t_burn, thrust_end_s=t_tail, apogee_s=t_apo, legal_s=t_legal, touchdown_s=t_touch,
                        pitch_m5_s=t_pitch, deploy_s=ph["deploy_s"])
    try:
        d = pd.read_csv(csv_path)
    except Exception as e:                              # noqa: BLE001 -- a run that wrote nothing scores nothing
        out["error"] = f"csv: {e}"
        return out
    ev = json.load(open(json_path)) if os.path.exists(json_path) else {}
    tt = d["t_s"].to_numpy()
    qe = d[["qw", "qx", "qy", "qz"]].to_numpy()
    T = truth["t"]
    i = np.clip(np.searchsorted(T, tt), 1, len(T) - 1)
    # the truth at the step's time (nearest of the 1 kHz rows: the step's own tick is one of them)
    j = np.where(np.abs(T[i - 1] - tt) < np.abs(T[i] - tt), i - 1, i)
    total, nose, roll, hdg, pitch = att_errors(qe, truth["q"][j])
    out["rows"] = int(len(tt))
    out["nonfinite_rows"] = int((d["finite"] == 0).sum())
    out["unhealthy_rows"] = int((d["healthy"] == 0).sum())
    out["running_rows"] = int((d["running"] == 1).sum())
    t_dep = ph["deploy_s"]
    phases = {"boost": (0.0, t_burn), "tail": (t_burn, t_tail), "coast": (t_burn, t_apo),
              "descent": (t_apo, t_touch), "pre_apogee": (0.0, t_apo), "pad": (-20.0, -1.0),
              # the descent's parts: before the drogue's deployment, its 3 s, and under it after
              "pre_deploy": (t_apo, t_dep), "deploy": (t_dep, t_dep + 3.0), "under_drogue": (t_dep + 3.0, t_touch)}
    out["phase"] = {}
    for name, (a, b) in phases.items():
        k = (tt >= a) & (tt < b)
        if not k.any():
            out["phase"][name] = None
            continue
        e = total[k]
        out["phase"][name] = dict(n=int(k.sum()), p90=pct(e, 90), p99=pct(e, 99), max=float(np.nanmax(e)),
                                  nose_p90=pct(nose[k], 90), roll_p90=pct(np.abs(roll[k]), 90),
                                  hdg_p90=pct(np.abs(hdg[k]), 90))
    k = (tt >= t_apo - 2.0) & (tt <= t_apo + 2.0)
    out["pitch_apogee"] = dict(max=float(np.nanmax(np.abs(pitch[k]))) if k.any() else float("nan"),
                               p90=pct(np.abs(pitch[k]), 90))
    # ... and in that window before the drogue's deployment (the sim's: apogee + 1 s), which turns the body over
    k = (tt >= t_apo - 2.0) & (tt <= min(t_apo + 2.0, ph["deploy_s"]))
    out["pitch_apogee_predeploy"] = dict(max=float(np.nanmax(np.abs(pitch[k]))) if k.any() else float("nan"))
    # the vertical velocity at the true apogee (up-positive error; the truth's is zero there)
    vz_f = -d["vd"].to_numpy()
    out["vz_at_apogee"] = float(np.interp(t_apo, tt, vz_f))
    vz_true = -np.interp(tt, T, truth["v"][:, 2])
    # the guard's case against the truth: the filter more than 10 m/s below it for 1 s, after burnout and before the
    # true apogee (G5's wording) -- or the call, if earlier: the guard judges only until the call
    q = ev.get("quorum", {})
    t_call = q.get("apogee_t")
    t_end = min(t_call, t_apo) if t_call is not None else t_apo
    below = (vz_f - vz_true < -10.0) & (tt >= t_burn) & (tt < t_end)
    run, best, t_first = 0.0, 0.0, None
    dt = np.diff(tt, prepend=tt[0])
    for kk in range(len(tt)):
        run = run + dt[kk] if below[kk] else 0.0
        if run >= 1.0 and t_first is None:
            t_first = float(tt[kk])
        best = max(best, run)
    out["vz_low_10_for_1s_t"] = t_first
    out["vz_low_10_longest_s"] = float(best)
    out["vz_err_max_coast"] = float(np.nanmax(np.abs((vz_f - vz_true)[(tt >= t_burn) & (tt < t_apo)]))) \
        if ((tt >= t_burn) & (tt < t_apo)).any() else float("nan")
    # where the guard latched: the filter's vertical velocity error then (filter less truth), and its lowest from
    # burnout to then -- a latch on a filter within a few m/s of the truth is a false one
    t_g = q.get("guard_latch_t")
    if t_g is not None:
        err = vz_f - vz_true
        k = (tt >= t_burn) & (tt <= t_g)
        out["guard_vz_err"] = float(np.interp(t_g, tt, err))
        out["guard_vz_err_lowest"] = float(np.min(err[k])) if k.any() else float("nan")
    # the quorum
    vz_at = lambda x: float(-np.interp(x, T, truth["v"][:, 2]))   # noqa: E731
    votes = q.get("votes", {})
    out["quorum"] = dict(call_t=t_call, latency_s=(t_call - t_apo) if t_call is not None else None,
                         launch_t=q.get("launch_t"), burnout_t=q.get("burnout_t"),
                         admitted_t=q.get("gnss_admitted_t"), guard_t=q.get("guard_latch_t"),
                         guard_vz=q.get("guard_vz"), guard_baro_rate=q.get("guard_baro_rate"),
                         backstop_t=q.get("backstop_t"), at_call=q.get("at_call"), landed_t=q.get("landed_t"),
                         fixes=q.get("fixes"))
    launch = q.get("launch_t") if q.get("launch_t") is not None else 0.0
    early = {}
    for vname in ("vel", "pitch", "gps", "baro"):
        ons = [x for x in votes.get(vname, {}).get("on", []) if x is not None and x >= launch]
        early[vname] = [x for x in ons if vz_at(x) > 2.0]
        out["quorum"][f"{vname}_on"] = ons
    out["quorum"]["early_votes"] = early
    vel_on = out["quorum"]["vel_on"]
    out["quorum"]["vel_vote_vz_true"] = vz_at(vel_on[0]) if vel_on else None
    out["quorum"]["call_vz_true"] = vz_at(t_call) if t_call is not None else None
    out["quorum"]["early_call"] = bool(t_call is not None and vz_at(t_call) > 2.0)
    out["quorum"]["call_before_apogee"] = bool(t_call is not None and t_call < t_apo)
    # the same quorum on the truth (--truth-nav): its call, and when each voter first rose after its launch
    tq = ev.get("truth_quorum")
    if tq is not None:
        tl = tq.get("launch_t") if tq.get("launch_t") is not None else 0.0
        out["truth_quorum"] = dict(call_t=tq.get("apogee_t"), guard_t=tq.get("guard_latch_t"),
                                   admitted_t=tq.get("gnss_admitted_t"), at_call=tq.get("at_call"),
                                   **{f"{vname}_on": [x for x in tq.get("votes", {}).get(vname, {}).get("on", [])
                                                      if x is not None and x >= tl]
                                      for vname in ("vel", "pitch", "gps", "baro")})
    # what else the run says
    fw = ev.get("fw", {})
    out["fw"] = {k: fw.get(k) for k in ("prior_deg", "prior_mag", "mag_fused", "levels", "levels_held", "screened",
                                        "dropped", "act_launches")}
    out["vert"] = dict(dropped=ev.get("vert", {}).get("dropped"), latched=ev.get("vert_latched"))
    out["dv"] = ev.get("dv")
    out["doppler"] = ev.get("doppler")
    out["sim"] = ev.get("sim")
    out["stage"] = {k: ev.get("stage", {}).get(k) for k in ("built", "capped_out", "eph_sats")}
    return out


def parse_stderr_stops(text):
    """'stops N' from the run's summary line."""
    import re
    m = re.search(r"stops (\d+)", text or "")
    return int(m.group(1)) if m else None


# ------------------------------------------------------------------ the tables

def load_scores(grid):
    runs = []
    for p in sorted(glob.glob(os.path.join(grid, "scores", "*.json"))):
        runs.append(json.load(open(p)))
    return runs


def twins(runs):
    """{(flight, pattern, mag): {arm: run}}"""
    tw = collections.defaultdict(dict)
    for r in runs:
        tw[(r["flight"], r["pattern"], r["mag"])][r["arm"]] = r
    return tw


def phase_pass(rd, ru, phase):
    a, b = rd["phase"].get(phase), ru["phase"].get(phase)
    if a is None or b is None:
        return None
    delta, floor = BARS[phase]
    return a["p90"] <= max(b["p90"] + delta, floor)


def pitch_pass(rd, ru, key="pitch_apogee"):
    return rd[key]["max"] <= ru[key]["max"] + PITCH_NEAR_APOGEE_DEG


def fmt(x, nd=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "-"
    return f"{x:.{nd}f}"


def frac(a, b, mark=True):
    if not b:
        return "-"
    f = a / b
    return f"{a}/{b} ({100 * f:.0f} %)" + (" **FAIL**" if mark and f < PASS_FRACTION else "")


def airframe(flight):
    return "_".join(flight.split("_")[:2])


def trajectory(run):
    """The run's trajectory: an airframe's still-air seeds fly one (the same truth), differing only in the receiver's
    noise, so they are not independent samples."""
    tr = run["truth"]
    return airframe(run["flight"]), round(tr["apogee_s"], 3), round(tr["touchdown_s"], 1)   # (the landing's ms
    # differs in the last digit between still-air seeds)


def by_trajectory(items):
    """items: (run, passed). The share passing with each trajectory counted once (its runs' mean), and how many
    trajectories."""
    g = collections.defaultdict(list)
    for r, ok in items:
        g[trajectory(r)].append(1.0 if ok else 0.0)
    return (float(np.mean([np.mean(v) for v in g.values()])) if g else float("nan")), len(g)


def g4_items(pp):
    """For a list of (key, {arm: run}) pairs: per item, the list of (design run, passed)."""
    out = {}
    for phase in ("boost", "coast", "descent"):
        out[phase] = [(v["design"], phase_pass(v["design"], v["unaided"], phase)) for _, v in pp]
        out[phase] = [(r, ok) for r, ok in out[phase] if ok is not None]
    out["pitch"] = [(v["design"], pitch_pass(v["design"], v["unaided"])) for _, v in pp]
    out["pitch_predeploy"] = [(v["design"], pitch_pass(v["design"], v["unaided"], "pitch_apogee_predeploy"))
                              for _, v in pp]
    return out


def withheld(r):
    """When the guard took the filter's two votes away in this run (None: never, or not applied -- the noguard arm
    reports the latch but withholds nothing)."""
    if r["arm"] == "noguard" or "--vote-guard 0" in r.get("args", ""):
        return None
    return r["quorum"]["guard_t"]


def cast_early(r, vname):
    """The run's raw votes of the filter (vel, pitch) while the true vertical velocity was above +2 m/s that reached
    the quorum: a vote that rose once the guard had latched is withheld (TR_KinematicChecks leaves it out of
    'available'), so it is not cast."""
    ev = r["quorum"]["early_votes"][vname]
    t_g = withheld(r)
    return ev if t_g is None else [x for x in ev if x < t_g]


def against_table(grid, other, L, h):
    """The no-fault flight design at magnetometer 0 in this grid beside another's (e.g. the antenna on the IMU)."""
    def rows(g):
        tw = twins([r for r in load_scores(g) if "error" not in r])
        return [v for k, v in tw.items() if k[1] == "none" and k[2] == "0" and "design" in v and "unaided" in v]

    sets = [(os.path.basename(os.path.normpath(grid)), rows(grid)), (os.path.basename(os.path.normpath(other)),
                                                                     rows(other))]
    h(f"### Against {sets[1][0]} ({other}): no fault, magnetometer 0, the flight design (deg unless noted; median "
      "[worst])")
    L.append("| Measure | " + " | ".join(n for n, _ in sets) + " |")
    L.append("|---|" + "---|" * len(sets))

    def ph(vs, phase, stat="p90"):
        x = [v["design"]["phase"][phase][stat] for v in vs if v["design"]["phase"].get(phase)]
        return f"{fmt(np.median(x))} [{fmt(max(x))}]" if x else "-"

    meas = [
        ("Runs", lambda vs: str(len(vs))),
        ("Coast p90", lambda vs: ph(vs, "coast")),
        ("Descent p90", lambda vs: ph(vs, "descent")),
        ("The drogue's deployment, its first 3 s, p90", lambda vs: ph(vs, "deploy")),
        ("Pitch error within +-2 s of apogee, max", lambda vs: (
            f"{fmt(np.median([v['design']['pitch_apogee']['max'] for v in vs]))} "
            f"[{fmt(max(v['design']['pitch_apogee']['max'] for v in vs))}]") if vs else "-"),
        ("Absolute vz error at the true apogee, p95 (m/s)", lambda vs: fmt(pct([abs(v["design"]["vz_at_apogee"])
                                                                          for v in vs], 95))),
        ("Runs passing the pitch +-2 s bar", lambda vs: f"{sum(pitch_pass(v['design'], v['unaided']) for v in vs)}"
                                                      f" of {len(vs)}"),
        ("Runs failing descent", lambda vs: "%d of %d" % (
            sum(phase_pass(v["design"], v["unaided"], "descent") is False for v in vs), len(vs))),
        ("Raw velocity vote minus the truth quorum's (ms), median [earliest]", lambda vs: _vote_lead(vs)),
    ]
    for name, f in meas:
        L.append(f"| {name} | " + " | ".join(f(vs) for _, vs in sets) + " |")


def _vote_lead(vs):
    d = []
    for v in vs:
        r = v["design"]
        tq = r.get("truth_quorum")
        if tq and tq["vel_on"] and r["quorum"]["vel_on"]:
            d.append(1000.0 * (r["quorum"]["vel_on"][0] - tq["vel_on"][0]))
    return f"{fmt(np.median(d), 0)} [{fmt(min(d), 0)}]" if d else "-"


def tables(grid, out_md, against=None):
    allruns = load_scores(grid)
    errs = [r for r in allruns if "error" in r]
    runs = [r for r in allruns if "error" not in r]
    tw = twins(runs)
    pairs = [(k, v) for k, v in tw.items() if "design" in v and "unaided" in v]
    pats = [p for p in PATTERN_NAMES if any(k[1] == p for k, _ in pairs)]
    arms = [a for a in ("design", "unaided", "noguard") if any(r["arm"] == a for r in runs)]
    L = []
    verdict = {}
    if not pairs:
        raise SystemExit(f"{grid}: no scored pairs (a flight design run and its unaided twin)")

    def h(s):
        L.append("")
        L.append(s)
        L.append("")

    flights = sorted({r["flight"] for r in runs})
    trajs = {trajectory(r) for r in runs}
    frames = sorted({airframe(f) for f in flights})
    n_arm = collections.Counter(r["arm"] for r in runs)
    h(f"Runs scored: {', '.join(f'{n_arm[a]} {a}' for a in arms)} ({len(pairs)} design/twin pairs), {len(pats)} "
      f"patterns, magnetometer arms {', '.join(MAG_ARMS)}; {len(errs)} runs wrote nothing.")
    L.append("What these runs assume (read every verdict with it):")
    L.append("")
    L.append("- **The raw vote guard is applied as flight_votes.h wires it** (D1b's wiring: stepped after "
             "GnssAscentGate with the barometer as baroFusable lets it vouch; once latched, the filter's two votes "
             "are withheld). No firmware tree has it yet: main.cpp on fc/v9raw-d, fc/v9-raw-primary and fc/v9raw-e3 "
             "never steps RawVoteGuard. So the image as built (RC0) is the **noguard** arm, and every "
             "guard-dependent result of the design arm is for the assumed wiring. Re-run once D1b lands, and check "
             "its barometer input (baroFusable against D1b's baro_healthy) is what flight_votes.h passes.")
    L.append("- The receiver's fixes reach the GPS voter at its own 20 Hz (PVTGeodetic at msec50); the filter fuses "
             "the 10 Hz grid.")
    L.append("- Placeholders until P1/G6 measure them: each epoch handed over 30 ms after its GPS instant, EKF ticks "
             "at 500 Hz, the flight loop at 1 kHz, and **no epoch slicing** -- with no clock (fc_unit_us 0) each "
             "epoch of up to 16 satellites is fused in one step, where startRawShadow's epoch_slice_us 2500 spreads "
             "it over steps on the target. Re-run with sim_grid --harness-args \"--fc-unit-us X\" once the P4's cost "
             "a unit is measured.")
    L.append("- The static-port lags (baro_lag) are a sensitivity sweep: nothing on the Goblin's port is measured.")
    L.append(f"- Effective sample: {len(flights)} flights are {len(trajs)} distinct trajectories over "
             f"{len(frames)} airframes ({', '.join(frames)}), neither of them the Goblin. An airframe's still-air "
             "seeds fly one trajectory and differ only in the receiver's noise; the magnetometer arms are "
             "deterministic variants of one run. One or two trajectories can decide a 95 % bar: the "
             "'by trajectory' figures count each trajectory once.")
    L.append("- The truth's own call is the same quorum flown on a perfect filter beside each run (raw_replay "
             "--truth-nav): the run's own barometer, IMU and receiver fixes -- so under a receiver fault its GPS "
             "voter is the faulted one.")

    # ---- G4 as worded: the magnetometer arms pooled
    h("### G4 as worded: runs passing each phase against their unaided twin, the magnetometer arms pooled "
      "(p90 <= max(twin p90 + delta, floor))")
    L.append("The +-40 and no-magnetometer arms' twins sit at 30-40 deg in every phase, so a run there passes a phase "
             "bar unless it is worse than that: pooling them hides phase failures in the arms with a real reference "
             "(next tables). The pitch bar is not saturated so: there every arm counts.")
    L.append("")
    L.append("| Pattern | Runs | Boost | Coast | Descent | Pitch +-2 s of apogee | (before the deployment) | "
             "Worst pre-apogee p90 above twin | Worst descent p90 above twin |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    g4_ok = True
    g4_fail = []
    per_pattern = {}
    for p in pats:
        pp = [(k, v) for k, v in pairs if k[1] == p]
        it = g4_items(pp)
        res = {k: (sum(ok for _, ok in v), len(v)) for k, v in it.items()}
        pre = [v["design"]["phase"]["pre_apogee"]["p90"] - v["unaided"]["phase"]["pre_apogee"]["p90"] for _, v in pp]
        des = [v["design"]["phase"]["descent"]["p90"] - v["unaided"]["phase"]["descent"]["p90"] for _, v in pp]
        res["pre_worst"] = max(pre) if pre else float("nan")
        res["des_worst"] = max(des) if des else float("nan")
        per_pattern[p] = res
        cells = []
        for key in ("boost", "coast", "descent", "pitch"):
            a, b = res[key]
            cells.append(frac(a, b))
            if b and a / b < PASS_FRACTION:
                g4_ok = False
                g4_fail.append(f"{p} {key} {a}/{b}")
        cells.append(frac(*res["pitch_predeploy"], mark=False))
        pre_mark = " **FAIL**" if res["pre_worst"] > PRE_APOGEE_TWIN_MAX else ""
        des_mark = " **FAIL**" if res["des_worst"] > DESCENT_TWIN_MAX else ""
        if pre_mark:
            g4_ok = False
            g4_fail.append(f"{p} pre-apogee twin +{res['pre_worst']:.1f}")
        if des_mark:
            g4_ok = False
            g4_fail.append(f"{p} descent twin +{res['des_worst']:.1f}")
        L.append(f"| {PATTERN_NAMES[p]} | {len(pp)} | " + " | ".join(cells) +
                 f" | {fmt(res['pre_worst'], 1)}{pre_mark} | {fmt(res['des_worst'], 1)}{des_mark} |")
    nf = [r for r in runs if r["pattern"] == "none"]
    stops = sum((r.get("stops") or 0) for r in nf)
    nonfin = sum((r.get("nonfinite_rows") or 0) for r in nf)
    unhealthy = sum((r.get("unhealthy_rows") or 0) for r in nf)
    allstops = sum((r.get("stops") or 0) for r in runs)
    allnonfin = sum((r.get("nonfinite_rows") or 0) for r in runs)
    h(f"No-fault runs (every arm): {len(nf)}, filter stops {stops}, non-finite rows {nonfin}, unhealthy rows "
      f"{unhealthy} ({'PASS' if stops == 0 and nonfin == 0 else 'FAIL'}). Every run: stops {allstops}, non-finite "
      f"rows {allnonfin}.")
    if stops or nonfin:
        g4_ok = False
        g4_fail.append("no-fault stops/NaN")

    # ---- G4 per magnetometer arm
    g4_arm_ok = True
    g4_arm_fail = []
    titles = {"boost": "Boost", "coast": "Coast", "descent": "Descent", "pitch": "Pitch within +-2 s of apogee"}
    for key in ("boost", "coast", "descent", "pitch"):
        h(f"### G4 per magnetometer arm: {titles[key]} (runs passing; the bar applied to each arm; the "
          + (f"informative arms are {', '.join(INFORMATIVE_ARMS)})" if key != "pitch" else
             "every arm is informative: the twin's pitch is good whatever its heading)"))
        L.append("| Pattern | " + " | ".join(MAG_ARMS) + " | Magnetometer 0 by trajectory |")
        L.append("|---|" + "---|" * (len(MAG_ARMS) + 1))
        for p in pats:
            cells = []
            for mag in MAG_ARMS:
                pp = [(k, v) for k, v in pairs if k[1] == p and k[2] == mag]
                it = g4_items(pp)[key]
                a, b = sum(ok for _, ok in it), len(it)
                fail = b and a / b < PASS_FRACTION
                cells.append(("**" if fail else "") + f"{a}/{b}" + ("**" if fail else "") if b else "-")
                if fail and (mag in INFORMATIVE_ARMS or key == "pitch"):
                    g4_arm_ok = False
                    g4_arm_fail.append(f"{p} {key} mag {mag} {a}/{b}")
            pp0 = [(k, v) for k, v in pairs if k[1] == p and k[2] == "0"]
            wf, nt = by_trajectory(g4_items(pp0)[key])
            cells.append(f"{100 * wf:.0f} % of {nt}" if nt else "-")
            L.append(f"| {PATTERN_NAMES[p]} | " + " | ".join(cells) + " |")
    verdict["G4"] = dict(passed=g4_ok, failing=g4_fail, per_pattern=per_pattern, nofault_stops=stops,
                         nofault_nonfinite=nonfin, passed_per_informative_arm=g4_arm_ok,
                         failing_per_informative_arm=g4_arm_fail)

    # ---- per phase and magnetometer arm: p90 medians and worst, design against unaided
    h("### Attitude error per phase and magnetometer arm, all patterns pooled (deg; median of the runs' p90 [worst "
      "run's p90]; p99 and max: the worst run's)")
    L.append("| Magnetometer | Arm | Boost p90 | Boost p99 / max | Tail-off p90 | Coast p90 | Coast p99 / max | "
             "Descent p90 | Descent p99 / max |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for mag in MAG_ARMS:
        for arm in ("design", "unaided"):
            rs = [r for r in runs if r["mag"] == mag and r["arm"] == arm]
            if not rs:
                continue
            cells = []
            for phase in ("boost", "tail", "coast", "descent"):
                v = [r["phase"][phase]["p90"] for r in rs if r["phase"].get(phase)]
                cells.append(f"{fmt(np.median(v))} [{fmt(max(v))}]" if v else "-")
                if phase != "tail":
                    p99 = [r["phase"][phase]["p99"] for r in rs if r["phase"].get(phase)]
                    mx = [r["phase"][phase]["max"] for r in rs if r["phase"].get(phase)]
                    cells.append(f"{fmt(max(p99))} / {fmt(max(mx))}" if p99 else "-")
            L.append(f"| {mag} | {arm} | " + " | ".join(cells) + " |")

    # ---- per pattern, in the informative arms
    for label, mags in (("magnetometer 0", ["0"]), ("magnetometer +-10", ["+10", "-10"])):
        h(f"### Per pattern at {label}: the flight design's p90 against its twin's (deg; median design / median "
          "twin, worst design-minus-twin)")
        L.append("| Pattern | Boost | Coast | Descent | Pitch +-2 s of apogee (max) |")
        L.append("|---|---|---|---|---|")
        for p in pats:
            pp = [v for k, v in pairs if k[1] == p and k[2] in mags]
            if not pp:
                continue
            cells = []
            for phase in ("boost", "coast", "descent"):
                dsg = [v["design"]["phase"][phase]["p90"] for v in pp if v["design"]["phase"].get(phase)]
                una = [v["unaided"]["phase"][phase]["p90"] for v in pp if v["unaided"]["phase"].get(phase)]
                exc = [a - b for a, b in zip(dsg, una)]
                cells.append(f"{fmt(np.median(dsg))} / {fmt(np.median(una))}, {fmt(max(exc), 1)}" if exc else "-")
            dsg = [v["design"]["pitch_apogee"]["max"] for v in pp]
            una = [v["unaided"]["pitch_apogee"]["max"] for v in pp]
            exc = [a - b for a, b in zip(dsg, una)]
            cells.append(f"{fmt(np.median(dsg))} / {fmt(np.median(una))}, {fmt(max(exc), 1)}")
            L.append(f"| {PATTERN_NAMES[p]} | " + " | ".join(cells) + " |")

    # ---- G4 failures listed
    fails = []
    for k, v in pairs:
        for phase in ("boost", "coast", "descent"):
            if phase_pass(v["design"], v["unaided"], phase) is False:
                fails.append((k, phase, v["design"]["phase"][phase]["p90"], v["unaided"]["phase"][phase]["p90"]))
    h(f"### Runs failing a phase, by pattern and magnetometer arm ({len(fails)} run-phases; b boost, c coast, "
      "d descent; of 16 flights each)")
    cnt = collections.Counter((k[1], k[2], ph_[0]) for k, ph_, _, _ in fails)
    L.append("| Pattern | " + " | ".join(MAG_ARMS) + " |")
    L.append("|---|" + "---|" * len(MAG_ARMS))
    for p in pats:
        cells = []
        for mag in MAG_ARMS:
            c = " ".join(f"{ph_}{cnt[(p, mag, ph_)]}" for ph_ in "bcd" if cnt[(p, mag, ph_)])
            cells.append(c or "-")
        L.append(f"| {PATTERN_NAMES[p]} | " + " | ".join(cells) + " |")
    worst = sorted(fails, key=lambda x: -(x[2] - max(x[3] + BARS[x[1]][0], BARS[x[1]][1])))[:40]
    h(f"The {len(worst)} worst (design p90 furthest over its bar):")
    if worst:
        L.append("| Flight | Pattern | Magnetometer | Phase | Design p90 | Twin p90 | Bar |")
        L.append("|---|---|---|---|---|---|---|")
        for (fl, p, mag), phase, a, b in worst:
            delta, floor = BARS[phase]
            L.append(f"| {fl} | {p} | {mag} | {phase} | {fmt(a)} | {fmt(b)} | {fmt(max(b + delta, floor))} |")
    else:
        L.append("None.")

    # ---- G5: votes and calls while the rocket still climbs
    h("### G5: votes and calls while the true vertical velocity was above +2 m/s (every run)")
    L.append("design: the guard applied as flight_votes.h wires it (assumed); noguard: the image as built. A raw vote "
             "that rose after the guard latched is withheld from the quorum, so it is not cast: the count is the "
             "votes cast (and, in brackets, those withheld). The last column is the highest true vertical velocity "
             "at any call, and its margin to the +2 m/s bar.")
    L.append("")
    L.append("| Pattern | Arm | Runs | Early calls | Early raw velocity votes cast (withheld) | Early raw pitch votes "
             "cast (withheld) | Early GPS votes | Early barometer votes | Calls before the true apogee | Highest true "
             "vz at a call (margin) |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    hard = {}
    early_list = []
    for arm in [a for a in ("design", "noguard") if a in arms]:
        hard[arm] = dict(ok=True, failing=[], worst_call_vz=-1e9)
    for p in pats:
        for arm in hard:
            rs = [r for r in runs if r["pattern"] == p and r["arm"] == arm]
            ec = sum(1 for r in rs if r["quorum"]["early_call"])
            ev_ = sum(len(cast_early(r, "vel")) for r in rs)
            ep = sum(len(cast_early(r, "pitch")) for r in rs)
            wv = sum(len(r["quorum"]["early_votes"]["vel"]) for r in rs) - ev_
            wp = sum(len(r["quorum"]["early_votes"]["pitch"]) for r in rs) - ep
            eg = sum(len(r["quorum"]["early_votes"]["gps"]) for r in rs)
            eb = sum(len(r["quorum"]["early_votes"]["baro"]) for r in rs)
            cb = sum(1 for r in rs if r["quorum"]["call_before_apogee"])
            cv = [r["quorum"]["call_vz_true"] for r in rs if r["quorum"]["call_vz_true"] is not None]
            top = max(cv) if cv else float("nan")
            if cv:
                hard[arm]["worst_call_vz"] = max(hard[arm]["worst_call_vz"], top)
            if ec + ev_ + ep:
                hard[arm]["ok"] = False
                hard[arm]["failing"].append(f"{p}: {ec} early calls, {ev_} raw velocity and {ep} raw pitch votes")
            early_list += [r for r in rs if r["quorum"]["early_call"] or r["quorum"]["early_votes"]["vel"]
                           or r["quorum"]["early_votes"]["pitch"]]
            mk = lambda x: f"{x} **FAIL**" if x else "0"   # noqa: E731
            L.append(f"| {PATTERN_NAMES[p]} | {arm} | {len(rs)} | {mk(ec)} | {mk(ev_)} ({wv}) | {mk(ep)} ({wp}) | "
                     f"{eg} | {eb} | {cb} | {fmt(top)} ({fmt(2.0 - top)}) |")
    if early_list:
        h("Runs with an early call or raw vote, by flight (the magnetometer arms that had it; the first arm's times, "
          "T+ s, with the true vertical velocity, m/s):")
        L.append("| Flight | Pattern | Arm | Magnetometer arms | Call (true vz) | First raw velocity vote (true vz) | "
                 "Raw pitch votes | Guard latched |")
        L.append("|---|---|---|---|---|---|---|---|")
        groups = collections.OrderedDict()
        for r in sorted(early_list, key=lambda r: (r["pattern"], r["arm"], r["flight"], MAG_ARMS.index(r["mag"]))):
            groups.setdefault((r["flight"], r["pattern"], r["arm"]), []).append(r)
        for (fl, p, arm), rs in groups.items():
            q = rs[0]["quorum"]
            call = f"{fmt(q['call_t'])} ({fmt(q['call_vz_true'], 1)})" if q["call_t"] is not None else "-"
            vel = (f"{fmt(q['vel_on'][0])} ({fmt(q['vel_vote_vz_true'], 1)})" if q["vel_on"] else "-")
            L.append(f"| {fl} | {p} | {arm} | {', '.join(r['mag'] for r in rs)} | {call} | {vel} | "
                     f"{', '.join(fmt(x) for x in q['early_votes']['pitch']) or '-'} | "
                     f"{fmt(q['guard_t']) if q['guard_t'] is not None else '-'} |")

    # ---- G5: latency and the vertical velocity at apogee
    h("### G5: the call's latency after the true apogee and the vertical velocity there")
    L.append("Latency bars as worded: the design against its unaided twin (both with the guard). Beside them, each "
             "arm's call minus the truth's own (the quorum on a perfect filter, same barometer and fixes).")
    L.append("")
    L.append("| Pattern | Calls (design / twin / noguard) | Latency median, s (design / twin / noguard) | "
             "Latency p95, s (design / twin / noguard) | Latency bars | Call minus the truth's, median / p95, s "
             "(design; noguard) | Absolute vz error at apogee p95, m/s (design / twin / noguard) | vz bars |")
    L.append("|---|---|---|---|---|---|---|---|")
    g5_lat_ok = True
    g5_vz_ok = True
    lat_fail, vz_fail = [], []
    for p in pats:
        sel = {arm: [r for r in runs if r["pattern"] == p and r["arm"] == arm] for arm in arms}
        lat = {arm: [r["quorum"]["latency_s"] for r in rs if r["quorum"]["latency_s"] is not None]
               for arm, rs in sel.items()}
        med = {arm: (float(np.median(x)) if x else float("nan")) for arm, x in lat.items()}
        p95 = {arm: pct(x, 95) for arm, x in lat.items()}
        d, u = sel.get("design", []), sel.get("unaided", [])
        lat_ok = (med["design"] <= med["unaided"] + LAT_MEDIAN_S) and (p95["design"] <= p95["unaided"] + LAT_P95_S) \
            and len(lat["design"]) == len(d)
        vzp = {arm: pct([abs(r["vz_at_apogee"]) for r in rs], 95) for arm, rs in sel.items()}
        vz_ok = vzp["design"] <= VZ_P95_MPS and vzp["design"] <= vzp["unaided"] + VZ_TWIN_MPS
        if not lat_ok:
            g5_lat_ok = False
            lat_fail.append(p)
        if not vz_ok:
            g5_vz_ok = False
            vz_fail.append(p)
        vs_truth = []
        for arm in ("design", "noguard"):
            dd = [r["quorum"]["call_t"] - r["truth_quorum"]["call_t"] for r in sel.get(arm, [])
                  if r["quorum"]["call_t"] is not None and r.get("truth_quorum")
                  and r["truth_quorum"]["call_t"] is not None]
            vs_truth.append(f"{fmt(np.median(dd), 3)} / {fmt(pct(dd, 95), 3)}" if dd else "-")
        three = lambda m: " / ".join(fmt(m.get(a), 3) for a in ("design", "unaided", "noguard"))   # noqa: E731
        L.append(f"| {PATTERN_NAMES[p]} | {' / '.join(f'{len(lat.get(a, []))}/{len(sel.get(a, []))}' for a in arms)} | "
                 f"{three(med)} | {three(p95)} | {'PASS' if lat_ok else '**FAIL**'} | {'; '.join(vs_truth)} | "
                 f"{' / '.join(fmt(vzp.get(a)) for a in ('design', 'unaided', 'noguard'))} | "
                 f"{'PASS' if vz_ok else '**FAIL**'} |")

    # ---- G5: the raw vote guard
    h("### G5: the raw vote guard (the design arm; the noguard arm's latch is the same until its call)")
    L.append("Should latch: the filter's vertical velocity more than 10 m/s below the truth for 1 s, after burnout and "
             "before the true apogee (or the call, if earlier). The guard judges against the barometer's rate, not "
             "the truth, so it can latch on a filter that is wrong by less: the error at the latch says how wrong. "
             f"False: latched on a filter within {GUARD_FALSE_MPS:.0f} m/s of the truth from burnout to the latch. On "
             "the truth: the guard judged on the quorum's perfect filter. The noguard call is what the image as built "
             "does on the same run.")
    L.append("")
    L.append("| Pattern | Runs | Latched | Latched before the call | Filter error at the latch, m/s (range) | "
             "Should latch | Of those, latched | False latches | Latched on the truth | noguard calls before apogee "
             "(highest true vz) |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    guard_nofault = guard_false = guard_missed = guard_truth = 0
    guard_should_in_cases = guard_latched_in_cases = 0
    for p in pats:
        rs = [r for r in runs if r["pattern"] == p and r["arm"] == "design"]
        ng = [r for r in runs if r["pattern"] == p and r["arm"] == "noguard"]
        lat = [r for r in rs if r["quorum"]["guard_t"] is not None]
        before = [r for r in lat if r["quorum"]["call_t"] is None or r["quorum"]["guard_t"] <= r["quorum"]["call_t"]]
        should = [r for r in rs if r["vz_low_10_for_1s_t"] is not None]
        should_lat = [r for r in should if r["quorum"]["guard_t"] is not None]
        false_ = [r for r in lat if r.get("guard_vz_err_lowest", float("nan")) > -GUARD_FALSE_MPS]
        errs_ = [r["guard_vz_err"] for r in lat if r.get("guard_vz_err") is not None]
        on_truth = [r for r in rs if r.get("truth_quorum") and r["truth_quorum"]["guard_t"] is not None]
        if p == "none":
            guard_nofault += len(lat)
        guard_false += len(false_)
        guard_missed += len(should) - len(should_lat)
        guard_truth += len(on_truth)
        if p in GUARD_PATTERNS:
            guard_should_in_cases += len(should)
            guard_latched_in_cases += len(should_lat)
        ngb = [r for r in ng if r["quorum"]["call_before_apogee"]]
        ngv = [r["quorum"]["call_vz_true"] for r in ngb if r["quorum"]["call_vz_true"] is not None]
        er = f"{fmt(min(errs_), 1)} to {fmt(max(errs_), 1)}" if errs_ else "-"
        L.append(f"| {PATTERN_NAMES[p]} | {len(rs)} | {len(lat)} | {len(before)} | {er} | {len(should)} | "
                 f"{len(should_lat)} | {len(false_)} | {len(on_truth)} | {len(ngb)} of {len(ng)} "
                 f"({fmt(max(ngv)) if ngv else '-'}) |")
    tested = guard_should_in_cases > 0
    h(f"Guard: {guard_nofault} latches in no-fault runs; {guard_should_in_cases} runs of the slow ramps "
      f"({', '.join(GUARD_PATTERNS)}) meet the should-latch condition and {guard_latched_in_cases} of them latched "
      f"({'TESTED' if tested else 'NOT TESTED: no run met the condition'}); over every pattern {guard_missed} runs "
      f"should have latched and did not; {guard_false} false latches (on a filter within {GUARD_FALSE_MPS:.0f} m/s "
      f"of the truth); {guard_truth} runs latch on the truth itself.")
    guard_item = tested and guard_missed == 0
    verdict["G5"] = dict(hard_passed=hard.get("design", {}).get("ok"),
                         hard_failing=hard.get("design", {}).get("failing"),
                         hard_passed_noguard=hard.get("noguard", {}).get("ok"),
                         hard_failing_noguard=hard.get("noguard", {}).get("failing"),
                         worst_call_vz_true=hard.get("design", {}).get("worst_call_vz"),
                         worst_call_vz_true_noguard=hard.get("noguard", {}).get("worst_call_vz"),
                         latency_passed=g5_lat_ok, latency_failing=lat_fail, vz_passed=g5_vz_ok, vz_failing=vz_fail,
                         guard_nofault_latches=guard_nofault, guard_false_latches=guard_false,
                         guard_missed=guard_missed, guard_latches_on_truth=guard_truth,
                         guard_cases=guard_should_in_cases, guard_cases_latched=guard_latched_in_cases,
                         guard_latch_item_passed=guard_item)

    # ---- E3's unit tests
    h("### E3's unit tests")
    nfz = [v for k, v in pairs if k[1] == "none"]
    nf0 = [v["design"] for k, v in pairs if k[1] == "none" and k[2] == "0"]
    within = {phase: sum(1 for r in nf0 if r["phase"].get(phase) and r["phase"][phase]["p90"] <= NOFAULT_TRUTH_DEG)
              for phase in ("boost", "coast", "descent")}
    ok_truth = all(within[ph_] == len(nf0) for ph_ in within) and nf0
    L.append(f"- No-fault runs within {NOFAULT_TRUTH_DEG} deg of the truth (magnetometer 0, the flight design, p90 "
             f"total error): boost {within['boost']}, coast {within['coast']}, descent {within['descent']} of "
             f"{len(nf0)} -- **{'PASS' if ok_truth else 'FAIL'}**; descent p90 median "
             f"{fmt(np.median([r['phase']['descent']['p90'] for r in nf0])) if nf0 else '-'}, worst "
             f"{fmt(max(r['phase']['descent']['p90'] for r in nf0)) if nf0 else '-'}, max error worst "
             f"{fmt(max(r['phase']['descent']['max'] for r in nf0)) if nf0 else '-'} deg.")
    mag_ok = all((v["unaided"]["fw"]["mag_fused"] or 0) > 0 for k, v in pairs if k[2] != "none30")
    nomag_ok = all((v["unaided"]["fw"]["mag_fused"] or 0) == 0 for k, v in pairs if k[2] == "none30")
    lv_ok = all((v["unaided"]["fw"]["levels"] or 0) > 0 for k, v in pairs)
    L.append(f"- The unaided twins fused the magnetometer in every magnetometer arm: {'yes' if mag_ok else 'NO'}; "
             f"none in the no-magnetometer arm: {'yes' if nomag_ok else 'NO'}; levelled in every run: "
             f"{'yes' if lv_ok else 'NO'}.")
    tq = [v["design"]["truth_quorum"] for v in nfz if v["design"].get("truth_quorum")]
    if tq:
        apo = {id(v["design"]["truth_quorum"]): v["design"]["truth"]["apogee_s"] for v in nfz
               if v["design"].get("truth_quorum")}
        calls = [t["call_t"] - apo[id(t)] for t in tq if t["call_t"] is not None]
        firsts = {vn: [t[f"{vn}_on"][0] - apo[id(t)] for t in tq if t[f"{vn}_on"]] for vn in ("vel", "pitch", "gps",
                                                                                               "baro")}
        never_early = all(x >= 0.0 for xs in firsts.values() for x in xs) and all(x > 0.0 for x in calls)
        L.append(f"- Voters on the truth (the quorum on a perfect filter, no-fault runs, every arm): the call "
                 f"{fmt(min(calls), 3)} to {fmt(max(calls), 3)} s after the true apogee (median "
                 f"{fmt(float(np.median(calls)), 3)}), "
                 + ", ".join(f"{vn} first up {fmt(min(x), 3)} to {fmt(max(x), 3)} s" for vn, x in firsts.items() if x)
                 + f"; none before the true apogee: **{'PASS' if never_early else 'FAIL'}**.")
        for arm in ("design", "unaided", "noguard"):
            dd = [v[arm]["quorum"]["call_t"] - v[arm]["truth_quorum"]["call_t"] for v in tw.values()
                  if arm in v and v[arm]["pattern"] == "none" and v[arm].get("truth_quorum")
                  and v[arm]["quorum"]["call_t"] is not None and v[arm]["truth_quorum"]["call_t"] is not None]
            if dd:
                L.append(f"  - no-fault {arm} call minus the truth's: {fmt(min(dd), 3)} to {fmt(max(dd), 3)} s (median "
                         f"{fmt(float(np.median(dd)), 3)}).")
    else:
        L.append("- Voters on the truth: no truth quorum in these scores (runs without --truth-nav).")
    secs = [r.get("seconds") for r in runs if r.get("seconds")]
    if secs:
        L.append(f"- Time a run: median {fmt(float(np.median(secs)), 3)} s, p95 {fmt(pct(secs, 95), 3)} s, max "
                 f"{fmt(max(secs), 3)} s (wall clock, the grid's jobs sharing the machine).")
    pri = collections.Counter((k[2], v["design"]["fw"]["prior_deg"]) for k, v in pairs)
    L.append("- The launch prior the flight design took, per arm: " +
             ", ".join(f"{m} {fmt(pd_, 0)} deg x{n}" for (m, pd_), n in sorted(pri.items())))
    pad = [v["design"]["phase"]["pad"]["p90"] for k, v in pairs
           if k[1] == "none" and k[2] == "0" and v["design"]["phase"].get("pad")]
    if pad:
        L.append(f"- No-fault pad attitude at magnetometer 0 (T-20 to T-1 s, total error p90): median "
                 f"{fmt(np.median(pad))}, worst {fmt(max(pad))} deg.")
    cuts = [r for r in runs if r["pattern"].startswith(("cut", "gap"))]
    sm = lambda r, k: r.get("sim_meta", {}).get(k, 0)   # noqa: E731
    L.append("- Fault counts written by sim_events.py: every cut run cut epochs and corrupted satellites: "
             f"{'yes' if all(sm(r, 'sim_epochs_cut') > 0 and sm(r, 'sim_sats_corrupted') > 0 for r in cuts) else 'NO'}"
             "; no-fault runs none: "
             f"{'yes' if all(sm(r, 'sim_epochs_cut') == 0 and sm(r, 'sim_sats_corrupted') == 0 for r in nf) else 'NO'}"
             "; no-fault fixes at the receiver's 20 Hz, twice the fused epochs (within one): "
             f"{'yes' if all(abs(sm(r, 'sim_rx_epochs') - 2 * sm(r, 'sim_epochs')) <= 1 and sm(r, 'sim_fixes') == sm(r, 'sim_rx_epochs') for r in nf) else 'NO'}.")
    rc = collections.Counter(r.get("rc") for r in allruns)
    L.append(f"- Harness exit codes: {dict(rc)}.")

    if against:
        against_table(grid, against, L, h)

    verdict["G4"]["text"] = "PASS" if g4_ok else "FAIL"
    verdict["G4"]["text_per_informative_arm"] = "PASS" if g4_arm_ok else "FAIL"
    g5_common = g5_lat_ok and g5_vz_ok and guard_nofault == 0 and guard_item
    verdict["G5"]["text"] = "PASS" if (hard.get("design", {}).get("ok") and g5_common) else "FAIL"
    verdict["G5"]["text_noguard"] = "PASS" if (hard.get("noguard", {}).get("ok") and g5_lat_ok and g5_vz_ok) \
        else "FAIL"
    L.insert(0, f"Verdicts: G4 as worded {verdict['G4']['text']}, per informative arm "
                f"{verdict['G4']['text_per_informative_arm']}; G5 with the guard as assumed {verdict['G5']['text']}, "
                f"as built (noguard: the hard bar, latency and vz) {verdict['G5']['text_noguard']}.")
    with open(out_md, "w") as fh:
        fh.write("\n".join(L) + "\n")
    with open(os.path.join(grid, "verdict.json"), "w") as fh:
        json.dump(verdict, fh, indent=1)
    return verdict


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grid")
    ap.add_argument("--report", help="the tables' markdown (default GRID/tables.md)")
    ap.add_argument("--against", help="another grid's OUT: its no-fault magnetometer-0 runs set beside these")
    a = ap.parse_args()
    v = tables(a.grid, a.report or os.path.join(a.grid, "tables.md"), a.against)
    print(json.dumps({k: {kk: vv for kk, vv in v[k].items() if kk.startswith("text")} for k in v}))


if __name__ == "__main__":
    sys.exit(main())
