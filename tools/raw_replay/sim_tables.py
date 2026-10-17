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
    calls cast while the true vertical velocity was above +2 m/s, the guard's latch, and whether the guard should
    have latched (the filter's vertical velocity more than 10 m/s below the truth for 1 s after burnout and before
    the call, the barometer's rate then 1-60 m/s: the guard's own condition is against the barometer, this one
    against the truth);
  - the filter's health: stops, non-finite and unhealthy rows; the magnetometer and levelling counts.

    python3 tools/raw_replay/sim_tables.py GRID_OUT [--report OUT.md]

reads GRID_OUT/scores/*.json and writes the tables (markdown) and GRID_OUT/verdict.json.
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
    ("cut05", "Cut T+0.5 to T+8, corrupted epoch before"),
    ("cut15", "Cut T+1.5 to T+8, corrupted epoch before"),
    ("cut30", "Cut T+3 to T+8, corrupted epoch before"),
    ("stale_rx", "Stale receiver, closing tail, GNSS after"),
    ("stale_sats", "Two stale satellites, relock-class sigma"),
    ("settle15", "Settling after the thrust, 1.5 m/s"),
    ("settle40", "Settling after the thrust, 4 m/s"),
    ("lol0", "Loss of lock at the true apogee"),
    ("lol1", "Loss of lock 1 s before apogee"),
    ("lol2", "Loss of lock 2 s before apogee"),
    ("lol3", "Loss of lock 3 s before apogee"),
    ("coherent", "Coherent coast fault, 0 to 5 m/s down over 3 s"),
    ("coherent15", "Coherent fault, 0 to 15 m/s down over 1 s from 3 s before apogee (the guard's case)"),
    ("baro_lag05", "Barometer port lag 0.5 s (GNSS clean)"),
    ("baro_lag07", "Barometer port lag 0.7 s (GNSS clean)"),
    ("baro_lag10", "Barometer port lag 1.0 s (GNSS clean)"),
])
GNSS_PATTERNS = [p for p in PATTERN_NAMES if not p.startswith("baro_lag")]
MAG_ARMS = ["0", "+10", "-10", "+40", "-40", "none30"]


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
    # the quorum the truth itself would call (TR_KinematicChecks' velocity and pitch voters on the true vertical
    # velocity and pitch, each up after APOGEE_COUNT_HI = 6 passes of 1 ms): the second of them
    tp = pitch_deg(truth["q"])
    kp = np.flatnonzero((truth["t"] > t_burn) & (tp < -5.0))
    t_pitch = float(truth["t"][kp[0]]) if kp.size else float("nan")
    out["truth"] = dict(burnout_s=t_burn, thrust_end_s=t_tail, apogee_s=t_apo, legal_s=t_legal, touchdown_s=t_touch,
                        pitch_m5_s=t_pitch, call_s=max(t_apo, t_pitch) + 0.006)
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
    # the guard's case against the truth: the filter more than 10 m/s below it for 1 s, after burnout, before the call
    q = ev.get("quorum", {})
    t_call = q.get("apogee_t")
    t_end = t_call if t_call is not None else t_apo
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
    out["quorum"]["call_vz_true"] = vz_at(t_call) if t_call is not None else None
    out["quorum"]["early_call"] = bool(t_call is not None and vz_at(t_call) > 2.0)
    out["quorum"]["call_before_apogee"] = bool(t_call is not None and t_call < t_apo)
    # what else the run says
    fw = ev.get("fw", {})
    out["fw"] = {k: fw.get(k) for k in ("prior_deg", "prior_mag", "mag_fused", "levels", "levels_held", "screened",
                                        "dropped", "act_launches")}
    out["vert"] = dict(dropped=ev.get("vert", {}).get("dropped"), latched=ev.get("vert_latched"))
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
    """{(flight, pattern, mag): {"design": run, "unaided": run}}"""
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


def fmt(x, nd=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "-"
    return f"{x:.{nd}f}"


def tables(grid, out_md):
    runs = load_scores(grid)
    tw = twins(runs)
    pairs = [(k, v) for k, v in tw.items() if "design" in v and "unaided" in v and "error" not in v["design"]
             and "error" not in v["unaided"]]
    pats = [p for p in PATTERN_NAMES if any(k[1] == p for k, _ in pairs)]
    L = []
    verdict = {}
    if not pairs:
        raise SystemExit(f"{grid}: no scored pairs (a flight design run and its unaided twin)")

    def h(s):
        L.append("")
        L.append(s)
        L.append("")

    n_design = sum(1 for r in runs if r["arm"] == "design")
    n_unaided = sum(1 for r in runs if r["arm"] == "unaided")
    h(f"Runs scored: {n_design} flight-design runs and {n_unaided} unaided twins ({len(pairs)} pairs), "
      f"{len(pats)} patterns, magnetometer arms {', '.join(MAG_ARMS)}.")

    # ---- G4: per phase and pattern, the fraction of runs that pass
    h("### G4: runs passing each phase against their unaided twin (p90 <= max(twin p90 + delta, floor))")
    L.append("| Pattern | Runs | Boost | Coast | Descent | Pitch +-2 s of apogee | (before the deployment) | "
             "Worst pre-apogee p90 above twin | Worst descent p90 above twin |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    g4_ok = True
    g4_fail_items = []
    per_pattern = {}
    for p in pats:
        pp = [(k, v) for k, v in pairs if k[1] == p]
        n = len(pp)
        res = {}
        for phase in ("boost", "coast", "descent"):
            ok = [phase_pass(v["design"], v["unaided"], phase) for _, v in pp]
            ok = [x for x in ok if x is not None]
            res[phase] = (sum(ok), len(ok))
        pa = [v["design"]["pitch_apogee"]["max"] <= v["unaided"]["pitch_apogee"]["max"] + PITCH_NEAR_APOGEE_DEG
              for _, v in pp]
        res["pitch"] = (sum(pa), len(pa))
        pb = [v["design"]["pitch_apogee_predeploy"]["max"] <=
              v["unaided"]["pitch_apogee_predeploy"]["max"] + PITCH_NEAR_APOGEE_DEG for _, v in pp]
        res["pitch_predeploy"] = (sum(pb), len(pb))
        pre = [v["design"]["phase"]["pre_apogee"]["p90"] - v["unaided"]["phase"]["pre_apogee"]["p90"] for _, v in pp]
        des = [v["design"]["phase"]["descent"]["p90"] - v["unaided"]["phase"]["descent"]["p90"] for _, v in pp]
        res["pre_worst"] = max(pre) if pre else float("nan")
        res["des_worst"] = max(des) if des else float("nan")
        per_pattern[p] = res
        cells = []
        for key in ("boost", "coast", "descent", "pitch"):
            a, b = res[key]
            frac = a / b if b else float("nan")
            mark = "" if frac >= PASS_FRACTION else " **FAIL**"
            if frac < PASS_FRACTION:
                g4_ok = False
                g4_fail_items.append(f"{p} {key} {a}/{b}")
            cells.append(f"{a}/{b} ({100 * frac:.0f} %){mark}")
        a, b = res["pitch_predeploy"]
        cells.append(f"{a}/{b} ({100 * a / b:.0f} %)" if b else "-")
        pre_mark = " **FAIL**" if res["pre_worst"] > PRE_APOGEE_TWIN_MAX else ""
        des_mark = " **FAIL**" if res["des_worst"] > DESCENT_TWIN_MAX else ""
        if pre_mark:
            g4_ok = False
            g4_fail_items.append(f"{p} pre-apogee twin +{res['pre_worst']:.1f}")
        if des_mark:
            g4_ok = False
            g4_fail_items.append(f"{p} descent twin +{res['des_worst']:.1f}")
        L.append(f"| {PATTERN_NAMES[p]} | {n} | " + " | ".join(cells) +
                 f" | {fmt(res['pre_worst'], 1)}{pre_mark} | {fmt(res['des_worst'], 1)}{des_mark} |")
    # no-fault health
    nf = [r for r in runs if r["pattern"] == "none"]
    stops = sum((r.get("stops") or 0) for r in nf)
    nonfin = sum((r.get("nonfinite_rows") or 0) for r in nf)
    unhealthy = sum((r.get("unhealthy_rows") or 0) for r in nf)
    allstops = sum((r.get("stops") or 0) for r in runs)
    allnonfin = sum((r.get("nonfinite_rows") or 0) for r in runs)
    h(f"No-fault runs: {len(nf)}, filter stops {stops}, non-finite rows {nonfin}, unhealthy rows {unhealthy}"
      f" ({'PASS' if stops == 0 and nonfin == 0 else 'FAIL'}). Every run: stops {allstops}, non-finite rows "
      f"{allnonfin}.")
    if stops or nonfin:
        g4_ok = False
        g4_fail_items.append("no-fault stops/NaN")
    verdict["G4"] = dict(passed=g4_ok, failing=g4_fail_items, per_pattern=per_pattern,
                         nofault_stops=stops, nofault_nonfinite=nonfin)

    # ---- per phase and magnetometer arm: p90 medians and worst, design against unaided
    h("### Attitude error per phase and magnetometer arm, all patterns pooled (deg; median of the runs' p90 [worst "
      "run's p90]; p99 and max: the worst run's)")
    L.append("| Magnetometer | Arm | Boost p90 | Boost p99 / max | Tail-off p90 | Coast p90 | Coast p99 / max | "
             "Descent p90 | Descent p99 / max |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for mag in MAG_ARMS:
        for arm in ("design", "unaided"):
            rs = [r for r in runs if r["mag"] == mag and r["arm"] == arm and "error" not in r]
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

    # ---- per pattern: design p90 against unaided (median, worst excess)
    h("### Per pattern: the flight design's p90 against its twin's (deg; median design / median unaided, "
      "worst design-minus-twin)")
    L.append("| Pattern | Boost | Coast | Descent | Pitch +-2 s of apogee (max) |")
    L.append("|---|---|---|---|---|")
    for p in pats:
        pp = [v for k, v in pairs if k[1] == p]
        cells = []
        for phase in ("boost", "coast", "descent"):
            dsg = [v["design"]["phase"][phase]["p90"] for v in pp]
            una = [v["unaided"]["phase"][phase]["p90"] for v in pp]
            exc = [a - b for a, b in zip(dsg, una)]
            cells.append(f"{fmt(np.median(dsg))} / {fmt(np.median(una))}, {fmt(max(exc), 1)}")
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

    # ---- G5: the apogee
    h("### G5: votes and calls while the true vertical velocity was above +2 m/s (flight design; every run)")
    L.append("| Pattern | Runs | Early calls | Early raw velocity votes | Early raw pitch votes | Early GPS votes | "
             "Early barometer votes | Calls before the true apogee |")
    L.append("|---|---|---|---|---|---|---|---|")
    g5_hard_ok = True
    g5_fail = []
    early_list = []
    for p in pats:
        rs = [r for r in runs if r["pattern"] == p and r["arm"] == "design" and "error" not in r]
        ec = sum(1 for r in rs if r["quorum"]["early_call"])
        ev_ = sum(len(r["quorum"]["early_votes"]["vel"]) for r in rs)
        ep = sum(len(r["quorum"]["early_votes"]["pitch"]) for r in rs)
        eg = sum(len(r["quorum"]["early_votes"]["gps"]) for r in rs)
        eb = sum(len(r["quorum"]["early_votes"]["baro"]) for r in rs)
        cb = sum(1 for r in rs if r["quorum"]["call_before_apogee"])
        bad = ec + ev_ + ep
        if bad:
            g5_hard_ok = False
            g5_fail.append(f"{p}: {ec} early calls, {ev_} raw velocity and {ep} raw pitch votes")
            early_list += [r for r in rs if r["quorum"]["early_call"] or r["quorum"]["early_votes"]["vel"]
                           or r["quorum"]["early_votes"]["pitch"]]
        mk = lambda x: f"{x} **FAIL**" if x else "0"   # noqa: E731
        L.append(f"| {PATTERN_NAMES[p]} | {len(rs)} | {mk(ec)} | {mk(ev_)} | {mk(ep)} | {eg} | {eb} | {cb} |")
    if early_list:
        h("Runs with an early call or raw vote (true vertical velocity at each, m/s):")
        L.append("| Flight | Pattern | Magnetometer | Call (T+, true vz) | Raw velocity votes | Raw pitch votes | "
                 "Guard |")
        L.append("|---|---|---|---|---|---|---|")
        for r in sorted(early_list, key=lambda r: (r["pattern"], r["flight"], r["mag"]))[:80]:
            q = r["quorum"]
            vz = lambda x: x  # noqa: E731
            call = f"{fmt(q['call_t'])} ({fmt(q['call_vz_true'], 1)})" if q["call_t"] is not None else "-"
            L.append(f"| {r['flight']} | {r['pattern']} | {r['mag']} | {call} | "
                     f"{', '.join(fmt(x) for x in q['early_votes']['vel']) or '-'} | "
                     f"{', '.join(fmt(x) for x in q['early_votes']['pitch']) or '-'} | "
                     f"{fmt(q['guard_t']) if q['guard_t'] is not None else '-'} |")
        if len(early_list) > 80:
            L.append(f"(and {len(early_list) - 80} more)")

    h("### G5: the call's latency after the true apogee and the vertical velocity there (flight design against "
      "unaided)")
    L.append("| Pattern | Calls (design / twin) | Latency median, s (design / twin) | Latency p95, s (design / twin) | "
             "Latency bars | |vz error| at apogee p95, m/s (design / twin) | vz bars |")
    L.append("|---|---|---|---|---|---|---|")
    g5_lat_ok = True
    g5_vz_ok = True
    lat_fail, vz_fail = [], []
    for p in pats:
        d = [r for r in runs if r["pattern"] == p and r["arm"] == "design" and "error" not in r]
        u = [r for r in runs if r["pattern"] == p and r["arm"] == "unaided" and "error" not in r]
        ld = [r["quorum"]["latency_s"] for r in d if r["quorum"]["latency_s"] is not None]
        lu = [r["quorum"]["latency_s"] for r in u if r["quorum"]["latency_s"] is not None]
        md, mu = (float(np.median(ld)) if ld else float("nan")), (float(np.median(lu)) if lu else float("nan"))
        pd_, pu = pct(ld, 95), pct(lu, 95)
        lat_ok = (md <= mu + LAT_MEDIAN_S) and (pd_ <= pu + LAT_P95_S) and len(ld) == len(d)
        vd = [abs(r["vz_at_apogee"]) for r in d]
        vu = [abs(r["vz_at_apogee"]) for r in u]
        vpd, vpu = pct(vd, 95), pct(vu, 95)
        vz_ok = vpd <= VZ_P95_MPS and vpd <= vpu + VZ_TWIN_MPS
        if not lat_ok:
            g5_lat_ok = False
            lat_fail.append(p)
        if not vz_ok:
            g5_vz_ok = False
            vz_fail.append(p)
        L.append(f"| {PATTERN_NAMES[p]} | {len(ld)}/{len(d)} / {len(lu)}/{len(u)} | {fmt(md, 3)} / {fmt(mu, 3)} | "
                 f"{fmt(pd_, 3)} / {fmt(pu, 3)} | {'PASS' if lat_ok else '**FAIL**'} | {fmt(vpd)} / {fmt(vpu)} | "
                 f"{'PASS' if vz_ok else '**FAIL**'} |")

    h("### G5: the raw vote guard (flight design)")
    L.append("| Pattern | Runs | Latched | Latched before the call | Should latch (vz > 10 m/s below the truth for 1 s, "
             "after burnout, before the call) | Of those, latched | Latched on a filter within 10 m/s of the truth |")
    L.append("|---|---|---|---|---|---|---|")
    guard_nofault = 0
    guard_false = 0
    guard_missed = 0
    for p in pats:
        rs = [r for r in runs if r["pattern"] == p and r["arm"] == "design" and "error" not in r]
        lat = [r for r in rs if r["quorum"]["guard_t"] is not None]
        before = [r for r in lat if r["quorum"]["call_t"] is None or r["quorum"]["guard_t"] <= r["quorum"]["call_t"]]
        should = [r for r in rs if r["vz_low_10_for_1s_t"] is not None]
        should_lat = [r for r in should if r["quorum"]["guard_t"] is not None]
        false_ = [r for r in lat if r["vz_low_10_for_1s_t"] is None]
        if p == "none":
            guard_nofault += len(lat)
        guard_false += len(false_)
        guard_missed += len(should) - len(should_lat)
        L.append(f"| {PATTERN_NAMES[p]} | {len(rs)} | {len(lat)} | {len(before)} | {len(should)} | {len(should_lat)} | "
                 f"{len(false_)} |")
    h(f"Guard: {guard_nofault} latches in no-fault runs; {guard_false} latches where the filter was within 10 m/s "
      f"of the truth (false latches, every pattern); {guard_missed} runs the truth says it should have latched and "
      "it did not.")
    verdict["G5"] = dict(hard_passed=g5_hard_ok, hard_failing=g5_fail, latency_passed=g5_lat_ok,
                         latency_failing=lat_fail, vz_passed=g5_vz_ok, vz_failing=vz_fail,
                         guard_nofault_latches=guard_nofault, guard_false_latches=guard_false,
                         guard_missed=guard_missed)

    # ---- sanity: the unaided arm takes no GNSS attitude, while levelling and the magnetometer act
    h("### Checks on the setup")
    nfz = [v for k, v in pairs if k[1] == "none"]
    mag_ok = all((v["unaided"]["fw"]["mag_fused"] or 0) > 0 for k, v in pairs if k[2] != "none30")
    nomag_ok = all((v["unaided"]["fw"]["mag_fused"] or 0) == 0 for k, v in pairs if k[2] == "none30")
    lv_ok = all((v["unaided"]["fw"]["levels"] or 0) > 0 for k, v in pairs)
    pri = collections.Counter((k[2], v["design"]["fw"]["prior_deg"]) for k, v in pairs)
    pad = [v["design"]["phase"]["pad"]["p90"] for k, v in pairs
           if k[1] == "none" and k[2] == "0" and v["design"]["phase"].get("pad")]
    L.append(f"- The unaided twins fused the magnetometer in every magnetometer arm: {'yes' if mag_ok else 'NO'}; "
             f"none in the no-magnetometer arm: {'yes' if nomag_ok else 'NO'}; levelled in every run: "
             f"{'yes' if lv_ok else 'NO'}.")
    L.append("- The launch prior the flight design took, per arm: " +
             ", ".join(f"{m} {fmt(pd_, 0)} deg x{n}" for (m, pd_), n in sorted(pri.items())))
    if pad:
        L.append(f"- No-fault pad attitude at magnetometer 0 (T-20 to T-1 s, total error p90): median "
                 f"{fmt(np.median(pad))}, worst {fmt(max(pad))} deg.")
    nf_d = [v["design"] for v in nfz]
    if nf_d:
        lat = [r["quorum"]["latency_s"] for r in nf_d if r["quorum"]["latency_s"] is not None]
        L.append(f"- No-fault calls: {len(lat)} of {len(nf_d)}, latency {fmt(min(lat), 3)} to {fmt(max(lat), 3)} s "
                 f"after the true apogee, none before it: "
                 f"{'yes' if all(x > 0 for x in lat) else 'NO'}.")
        dtc = [r["quorum"]["call_t"] - r["truth"]["call_s"] for r in nf_d if r["quorum"]["call_t"] is not None]
        dtu = [v["unaided"]["quorum"]["call_t"] - v["unaided"]["truth"]["call_s"] for v in nfz
               if v["unaided"]["quorum"]["call_t"] is not None]
        L.append(f"- The truth's own call (its velocity and pitch voters on the true vertical velocity and pitch, 6 "
                 f"passes each): {fmt(min(r['truth']['call_s'] - r['truth']['apogee_s'] for r in nf_d), 3)} to "
                 f"{fmt(max(r['truth']['call_s'] - r['truth']['apogee_s'] for r in nf_d), 3)} s after the true apogee "
                 f"(the true pitch reaches -5 deg then); the no-fault design's call minus it: "
                 f"{fmt(min(dtc), 3)} to {fmt(max(dtc), 3)} s (median {fmt(float(np.median(dtc)), 3)}), the twin's "
                 f"{fmt(min(dtu), 3)} to {fmt(max(dtu), 3)} s.")
    cuts = [r for r in runs if r["pattern"].startswith("cut")]
    L.append("- Fault counts written by sim_events.py: every cut run cut epochs and corrupted satellites: "
             f"{'yes' if all(r.get('sim_meta', {}).get('sim_epochs_cut', 0) > 0 and r.get('sim_meta', {}).get('sim_sats_corrupted', 0) > 0 for r in cuts) else 'NO'}; "
             "no-fault runs none: "
             f"{'yes' if all(r.get('sim_meta', {}).get('sim_epochs_cut', 0) == 0 and r.get('sim_meta', {}).get('sim_sats_corrupted', 0) == 0 for r in nf) else 'NO'}.")
    rc = collections.Counter(r.get("rc") for r in runs)
    L.append(f"- Harness exit codes: {dict(rc)}.")

    verdict["G4"]["text"] = "PASS" if g4_ok else "FAIL"
    verdict["G5"]["text"] = "PASS" if (g5_hard_ok and g5_lat_ok and g5_vz_ok and guard_nofault == 0) else "FAIL"
    with open(out_md, "w") as fh:
        fh.write("\n".join(L) + "\n")
    with open(os.path.join(grid, "verdict.json"), "w") as fh:
        json.dump(verdict, fh, indent=1)
    return verdict


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grid")
    ap.add_argument("--report", help="the tables' markdown (default GRID/tables.md)")
    a = ap.parse_args()
    v = tables(a.grid, a.report or os.path.join(a.grid, "tables.md"))
    print(json.dumps({k: v[k]["text"] for k in v}))


if __name__ == "__main__":
    sys.exit(main())
