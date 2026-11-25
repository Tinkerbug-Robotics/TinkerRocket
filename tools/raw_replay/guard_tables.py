#!/usr/bin/env python3
"""boost_safeguards.md from guard_sweep.py: per pad IMU and magnetometer error, every configuration pooled over the
GNSS-aided logs, the GNSS-lost-in-boost log (c30) on its own, and every run whose heading ends worse than unaided's.

    guard_tables.py STUDY_DIR OUT.md --aided r18,r20,... --lost c30 [--controls r16,r19,r26]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

GUARD_NAME = {"none": "no safeguard", "a1": "a1: >= 6 pseudoranges", "a3": "a3: Doppler velocity <= 5 m/s",
              "b1": "b1: heading step <= 2 sigma", "b2": "b2: heading step <= 5 deg", "c": "c: chi2 3.84 above 3 g",
              "d": "d: act from the apogee call", "ref": "unaided (observe)"}
COLS = (("nose", "boost"), ("nose", "coast"), ("nose", "descent"), ("roll", "boost"), ("roll", "coast"),
        ("roll", "descent"), ("pitch", "coast"))


def label(cfg):
    g, p = cfg.split("_")
    return g, p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("study")
    ap.add_argument("out")
    ap.add_argument("--aided", required=True)
    ap.add_argument("--lost", default="c30")
    ap.add_argument("--controls", default="")
    ap.add_argument("--tol", type=float, default=1.0, help="deg: a heading this much worse than unaided's is flagged")
    a = ap.parse_args()
    aided, lost = a.aided.split(","), [x for x in a.lost.split(",") if x]
    controls = [x for x in a.controls.split(",") if x]
    tr = S.Truth()
    res = []
    for run in aided + lost + controls:
        f = os.path.join(a.study, run, "guard.json")
        if os.path.exists(f):
            res += json.load(open(f))["results"]
    bases = {}
    E = {}
    for r in res:
        b = bases.setdefault(r["run"], float(S.read_meta(os.path.join(HERE, "in", r["run"] + ".txt"))["t_base_s"]))
        E[(r["run"], r["cfg"], r["mag_err"], r["imu"])] = (S.sample_errors(S.harness_series(r["csv"], b), tr), r["score"])
    cfgs = sorted({r["cfg"] for r in res}, key=lambda c: (c[0] if c[0] != "U" else "Z", c))

    def c3(keys, metric, phase):
        xs = [E[k][0][metric][E[k][0]["phase"] == phase] for k in keys if k in E and (E[k][0]["phase"] == phase).any()]
        if not xs:
            return "-"
        s3 = S.stats3(np.concatenate(xs))
        return f"{s3[0]:.1f} / {s3[1]:.1f} / {s3[2]:.1f}"

    L = ["# Boost safeguards (offline replay)\n",
         "Harness-only switches, no firmware change. A: act, full gate. C: heading-only gate. H: C until the apogee "
         "call, then A. 10 / 30: the launch heading prior. Every one with the 0.6 deg launch tilt floor, no hand-off, "
         "carrier observed. A boost epoch is one whose IMU peak since the epoch before is above 2 g. Safeguards:",
         "", "- a1: the attitude held out of a boost epoch with fewer than 6 pseudoranges (it still corrects the "
         "velocity, position and clock);",
         "- a3: the same for a boost epoch whose Dopplers, fitted together, point to a velocity correction over 5 m/s "
         "(a receiver that has lost track: c30's T+1 epoch pointed to 66 m/s);",
         "- b1 / b2: in a boost epoch, a satellite whose update would turn the heading past 2 sigma of its 1-sigma "
         "(b1) or past 5 deg (b2) in that epoch is refused;",
         "- c: the innovation gate tightened from chi2 10.83 (3.3 sigma) to 3.84 (2 sigma) while the IMU peak is "
         "above 3 g;",
         "- d (comparison): observe until the apogee call, then act's launch edge (LA full gate, LC heading-only).", "",
         f"GNSS-aided logs pooled: {', '.join(aided)}. The GNSS-lost-in-boost log ({', '.join(lost)}) on its own: "
         "its receiver gave one corrupted epoch at T+1.0 s and then nothing. Cells: median / p90 / max, deg, every "
         "0.1 s of the phase; pitch vote: median T+ s of the first estimated pitch below -5 deg (truth "
         f"T+{tr.pitch_vote:.3f}).\n"]
    for imu, iname in (("cal", "calibrated pad (0 mg)"), ("uncal", "uncalibrated pad (the rig as recorded)")):
        for m in sorted({k[2] for k in E}):
            for group, runs in (("GNSS-aided logs", aided), ("GNSS lost in boost", lost)):
                if not runs:
                    continue
                L.append(f"\n## {iname}, magnetometer {m:+.0f} deg: {group} ({', '.join(runs)})\n")
                L.append("| config | prior | safeguard | " + " | ".join(f"{k}, {p}" for k, p in COLS) +
                         " | pitch vote | heading at the end, worse than unaided |")
                L.append("|---|---|---|" + "---|" * (len(COLS) + 2))
                for c in cfgs:
                    keys = [(r, c, m, imu) for r in runs]
                    if not any(k in E for k in keys):
                        continue
                    g, gd = label(c)
                    cname = {"U": "unaided"}.get(g, g[:-2] if g[-2:] in ("10", "30") else g)
                    pr = g[-2:] if g[-2:] in ("10", "30") else "-"
                    ts = [E[k][1].get("pitch_vote_t") for k in keys if k in E and E[k][1].get("pitch_vote_t")]
                    worse = []
                    for k in keys:
                        u = (k[0], "U_ref", m, imu)
                        if k in E and u in E and E[k][1].get("end_heading") is not None:
                            if E[k][1]["end_heading"] > E[u][1]["end_heading"] + a.tol:
                                worse.append(f"{k[0]} {E[k][1]['end_heading']:.0f} vs {E[u][1]['end_heading']:.0f}")
                    L.append(f"| {cname} | {pr} | {GUARD_NAME.get(gd, gd)} | " +
                             " | ".join(c3(keys, mt, ph) for mt, ph in COLS) +
                             f" | {np.median(ts):.3f} | {', '.join(worse) if worse else '-'} |")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
