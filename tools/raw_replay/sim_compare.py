#!/usr/bin/env python3
"""Grid B: experiment arms against the flight design and its unaided twin, pattern by pattern and magnetometer arm by
arm (plan section 4's bars, the deployment, the pitch near apogee and the apogee quorum).

sim_grid.py writes each arm's scores: the base grid (OUT/scores) holds the flight design ("design", today's defaults)
and its twin ("unaided"); an experiment is another grid run with --harness-args and --arms design, its "design" the
experiment. This sets each experiment's runs beside the same flight's design and twin -- the twin the acceptance
compares against is today's (the experiment's switches act on GNSS, which the twin keeps off the attitude anyway):

    python3 tools/raw_replay/sim_compare.py BASE_OUT NAME=EXP_OUT [NAME=EXP_OUT ...] [--title T] [--out REPORT.md]
        [--json SUMMARY.json]

For each pattern and magnetometer arm (16 flights each), per arm (the design, then each experiment):
  - the runs passing each phase bar against the twin: p90 <= max(twin p90 + delta, floor), boost 1/2, coast 2/3,
    descent 3/5 deg -- informative only where the twin's own error is small (arms 0, +-10; the +-40 and
    no-magnetometer twins sit at 30-40 deg, so there a run passes unless far worse than that);
  - the worst p90 above the twin before the apogee and in descent (deg; bars 5 and 10);
  - the deployment's first 3 s: p90 median [worst] (the drogue's snatch: where an antenna off the IMU swings);
  - the pitch error within +-2 s of the true apogee: the runs within twin + 2 deg, and the worst max;
  - the apogee quorum: calls and raw velocity or pitch votes while the true vertical velocity was above +2 m/s, the
    call's latency after the true apogee (median, p95), |vz| at the true apogee (p95), the guard's latches.
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
from sim_tables import BARS, PATTERN_NAMES, MAG_ARMS, PITCH_NEAR_APOGEE_DEG, PRE_APOGEE_TWIN_MAX, \
    DESCENT_TWIN_MAX  # noqa: E402

INFORMATIVE = ("0", "+10", "-10")


def load(grid, arm_filter=None):
    """{(flight, pattern, mag): run} for each arm in the grid's scores: {arm: {...}}"""
    out = collections.defaultdict(dict)
    for p in glob.glob(os.path.join(grid, "scores", "*.json")):
        r = json.load(open(p))
        if arm_filter and r["arm"] not in arm_filter:
            continue
        out[r["arm"]][(r["flight"], r["pattern"], r["mag"])] = r
    return out


def ph(r, name, key="p90"):
    x = (r.get("phase") or {}).get(name)
    return float("nan") if x is None else x[key]


def pct(x, p):
    x = np.asarray([v for v in x if v is not None and math.isfinite(v)], float)
    return float(np.percentile(x, p)) if x.size else float("nan")


def switches(args):
    """The experiment's own flags in a run's recorded arguments (the turning body's)."""
    tok = args.split()
    out = []
    for i, t in enumerate(tok):
        if t.startswith("--rot") or t.startswith("--att"):
            out += [t] + ([tok[i + 1]] if i + 1 < len(tok) and not tok[i + 1].startswith("--") else [])
    return " ".join(out)


def f1(x, nd=1):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def cell_stats(runs, twins):
    """One arm's numbers over a cell's runs (each paired with its twin by key)."""
    s = dict(n=0, boost=0, coast=0, descent=0, pitch=0, pre_excess=-1e9, desc_excess=-1e9, dep=[], desc=[], pre=[],
             pitch_max=[], early_calls=0, early_raw=0, lat=[], vz=[], guard=0, stops=0, nonfinite=0)
    for k, r in runs.items():
        u = twins.get(k)
        if u is None or r.get("error") or u.get("error"):
            continue
        s["n"] += 1
        for phase, (delta, floor) in BARS.items():
            if ph(r, phase) <= max(ph(u, phase) + delta, floor):
                s[phase] += 1
        if r["pitch_apogee"]["max"] <= u["pitch_apogee"]["max"] + PITCH_NEAR_APOGEE_DEG:
            s["pitch"] += 1
        s["pre_excess"] = max(s["pre_excess"], ph(r, "pre_apogee") - ph(u, "pre_apogee"))
        s["desc_excess"] = max(s["desc_excess"], ph(r, "descent") - ph(u, "descent"))
        s["dep"].append(ph(r, "deploy"))
        s["desc"].append(ph(r, "descent"))
        s["pre"].append(ph(r, "pre_apogee"))
        s["pitch_max"].append(r["pitch_apogee"]["max"])
        q = r.get("quorum") or {}
        s["early_calls"] += 1 if q.get("early_call") else 0
        ev = q.get("early_votes") or {}
        s["early_raw"] += len(ev.get("vel", [])) + len(ev.get("pitch", []))
        if q.get("latency_s") is not None:
            s["lat"].append(q["latency_s"])
        s["vz"].append(abs(r["vz_at_apogee"]))
        s["guard"] += 1 if q.get("guard_t") is not None else 0
        s["stops"] += r.get("stops") or 0
        s["nonfinite"] += r.get("nonfinite_rows") or 0
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("base")
    ap.add_argument("exps", nargs="*", help="NAME=GRID_OUT: an experiment's grid (its 'design' arm)")
    ap.add_argument("--title", default="Grid B: experiments against the flight design and its unaided twin")
    ap.add_argument("--out")
    ap.add_argument("--json")
    a = ap.parse_args()
    base = load(a.base, {"design", "unaided"})
    twins = base["unaided"]
    arms = collections.OrderedDict([("design", base["design"])])
    exp_args = collections.OrderedDict()
    for e in a.exps:
        name, _, path = e.partition("=")
        g = load(path, {"design"})
        arms[name] = g["design"]
        args = {r.get("args", "") for r in g["design"].values()}
        exp_args[name] = sorted(args)[0] if args else ""
    pats = [p for p in PATTERN_NAMES if any(k[1] == p for k in twins)]
    lines = [f"# {a.title}", ""]
    lines.append("Arms: **design** = today's defaults; " + "; ".join(
        f"**{n}** = design + `{switches(exp_args[n])}`" for n in exp_args)
        + ". Every arm is scored against the same unaided twin (today's `--unaided`).")
    lines.append("")
    lines.append("Cells: runs passing / 16 flights. Bars (plan section 4): boost p90 <= max(twin + 1, 2), coast "
                 "max(twin + 2, 3), descent max(twin + 3, 5) deg; pitch within +-2 s of apogee <= twin + 2 deg; "
                 "worst pre-apogee p90 above the twin <= 5, worst descent <= 10. Only arms 0 and +-10 can fail the "
                 "phase bars (the +-40 and no-magnetometer twins sit at 30-40 deg).")
    summary = {}
    for scope_name, mags in (("informative arms 0, +10, -10", INFORMATIVE), ("every arm", tuple(MAG_ARMS))):
        lines += ["", f"## Phase bars and worst excess, pooled over {scope_name}", ""]
        hdr = "| Pattern | Arm | Runs | Boost | Coast | Descent | Pitch +-2 s | Worst pre-apogee above twin | " \
              "Worst descent above twin | Deploy 3 s p90 median [worst] | Descent p90 median |"
        lines += [hdr, "|" + "---|" * 11]
        for p in pats:
            tw_cell = {k: v for k, v in twins.items() if k[1] == p and k[2] in mags}
            for name, runs in arms.items():
                rc = {k: v for k, v in runs.items() if k[1] == p and k[2] in mags}
                if not rc:
                    continue
                s = cell_stats(rc, tw_cell)
                n = s["n"]
                flag = lambda c: f"**{c}/{n}**" if c < 0.95 * n else f"{c}/{n}"  # noqa: E731
                lines.append(f"| {PATTERN_NAMES[p]} | {name} | {n} | {flag(s['boost'])} | {flag(s['coast'])} | "
                             f"{flag(s['descent'])} | {flag(s['pitch'])} | "
                             f"{f1(s['pre_excess'])}{' **F**' if s['pre_excess'] > PRE_APOGEE_TWIN_MAX else ''} | "
                             f"{f1(s['desc_excess'])}{' **F**' if s['desc_excess'] > DESCENT_TWIN_MAX else ''} | "
                             f"{f1(pct(s['dep'], 50), 2)} [{f1(max(s['dep']) if s['dep'] else float('nan'))}] | "
                             f"{f1(pct(s['desc'], 50), 2)} |")
                summary.setdefault(scope_name, {}).setdefault(p, {})[name] = {
                    k: (v if not isinstance(v, list) else dict(p50=pct(v, 50), p95=pct(v, 95),
                                                               max=max(v) if v else None))
                    for k, v in s.items()}
            tw = cell_stats(tw_cell, tw_cell)
            lines.append(f"| {PATTERN_NAMES[p]} | (twin) | {tw['n']} | | | | | | | "
                         f"{f1(pct(tw['dep'], 50), 2)} [{f1(max(tw['dep']) if tw['dep'] else float('nan'))}] | "
                         f"{f1(pct(tw['desc'], 50), 2)} |")
    # per magnetometer arm, the no-fault and the GNSS-fault patterns' key numbers
    lines += ["", "## Per magnetometer arm: descent p90 median and deployment p90 median [worst] (deg)", ""]
    lines += ["| Pattern | Mag | " + " | ".join(f"{n} descent / deploy" for n in arms) + " | twin descent / deploy |",
              "|" + "---|" * (3 + len(arms))]
    for p in pats:
        for m in MAG_ARMS:
            row = [PATTERN_NAMES[p], m]
            for name, runs in arms.items():
                rc = [r for k, r in runs.items() if k[1] == p and k[2] == m]
                d = [ph(r, "descent") for r in rc]
                dep = [ph(r, "deploy") for r in rc]
                row.append(f"{f1(pct(d, 50), 2)} / {f1(pct(dep, 50), 2)} [{f1(max(dep) if dep else float('nan'))}]")
            rc = [r for k, r in twins.items() if k[1] == p and k[2] == m]
            d = [ph(r, "descent") for r in rc]
            dep = [ph(r, "deploy") for r in rc]
            row.append(f"{f1(pct(d, 50), 2)} / {f1(pct(dep, 50), 2)} [{f1(max(dep) if dep else float('nan'))}]")
            lines.append("| " + " | ".join(row) + " |")
    # pitch near apogee and the quorum
    lines += ["", "## Near apogee and the quorum (every magnetometer arm)", ""]
    lines += ["| Pattern | Arm | Pitch +-2 s max: median [worst] | Calls with true vz > 2 | Raw votes with true vz > 2 | "
              "Call latency median / p95 s | abs(vz) at apogee p95 | Guard latches | Stops / non-finite rows |",
              "|" + "---|" * 9]
    for p in pats:
        tw_cell = {k: v for k, v in twins.items() if k[1] == p}
        for name, runs in list(arms.items()) + [("(twin)", twins)]:
            rc = {k: v for k, v in runs.items() if k[1] == p}
            s = cell_stats(rc, tw_cell)
            lines.append(f"| {PATTERN_NAMES[p]} | {name} | {f1(pct(s['pitch_max'], 50), 2)} "
                         f"[{f1(max(s['pitch_max']) if s['pitch_max'] else float('nan'), 2)}] | {s['early_calls']} | "
                         f"{s['early_raw']} | {f1(pct(s['lat'], 50), 3)} / {f1(pct(s['lat'], 95), 3)} | "
                         f"{f1(pct(s['vz'], 95), 2)} | {s['guard']} | {s['stops']} / {s['nonfinite']} |")
    text = "\n".join(lines) + "\n"
    if a.out:
        with open(a.out, "w") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(dict(exp_args=exp_args, summary=summary), fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
