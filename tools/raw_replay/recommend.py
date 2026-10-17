#!/usr/bin/env python3
"""The safeguard candidates side by side against the acceptance criterion and what they keep:

    recommend.py OUT.md [--imu cal] [--tol 1.0]

Sources (later ones replace earlier ones for the same log, scenario, configuration and magnetometer error):
  firmware levelling: study_drop, then study_drop2 (the a4 fix, the a5 family, f4a), study_drop_a6fw2 (a6)
  heading-free in-flight levelling (the experiment): study_drop_exp, study_drop_a6exp2 (a6)
Unaided: always the firmware's (study_drop / study_drop2 U_ref), the reference the user flies without the features.

(1) A run counts as worse than unaided when, after a boost dropout (c30 as flown, a synthesized cut, or a corrupted
epoch then a cut), its median nose or roll error in some phase is more than tol deg above unaided's on the same log,
scenario and magnetometer error, or its heading, roll or nose over the last 2 s before landing is.
(2) What it keeps with boost GNSS good: the aided logs as flown, median over the logs of the per-log medians.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dropout_tables as DT  # noqa: E402

LOST = {"c30"}


def load(dirs, imu):
    byk = {}
    for sw in dirs:
        for f in sorted(glob.glob(os.path.join(HERE, sw, imu, "*", "dropout.json"))):
            for r in json.load(open(f))["results"]:
                byk[(r["run"], r["scn"], r["cfg"], r["mag_err"])] = r
    return byk


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--imu", default="cal")
    ap.add_argument("--tol", type=float, default=1.0)
    a = ap.parse_args()
    fw = load(["study_drop", "study_drop2", "study_drop_a6fw2"], a.imu)
    ex = load(["study_drop_exp", "study_drop_a6exp2", "study_drop_a6hexp3"], a.imu)
    U = {(k[0], k[1], k[3]): r for k, r in fw.items() if k[2] == "U_ref"}
    rows = [("firmware levelling", fw, c) for c in ("H30_none", "H30_a4", "H30_a5", "H30_a6", "H30_b2", "H30_a4b2",
                                                    "H30_a6b2", "H30_b2c", "H10_none", "H10_a4", "H10_a6", "H10_b2",
                                                    "H10_a6b2", "A30_none", "C30_none")]
    rows += [("heading-free levelling", ex, c) for c in ("H30_none", "H30_a4", "H30_a5", "H30_a6", "H30_a6h",
                                                         "H30_b2", "H30_a4b2", "H30_a6b2", "H30_a6hb2", "H10_none",
                                                         "H10_a4", "H10_a6", "H10_a6h", "H10_b2", "H10_a6b2",
                                                         "H10_a6hb2", "A30_none", "C30_none", "U_ref")]
    logs = sorted({k[0] for k in fw})

    def klass(r):
        if r["scn"] == "real":
            return "lost" if r["run"] in LOST else "real"
        return "drop"

    L = [f"# Safeguard candidates ({'calibrated pad, 0 mg' if a.imu == 'cal' else 'uncalibrated pad'})\n",
         f"Logs: {', '.join(logs)}. Worse than unaided: some phase's median nose or roll, or the last 2 s heading, "
         f"roll or nose, more than {a.tol:g} deg above the firmware's unaided on the same log, scenario and "
         "magnetometer error. Dropouts: c30 as flown, every epoch cut from T+0.5/1.5/3 s to T+8/12 s or the end, and a "
         "c30-like corrupted epoch at T+1 s (two draws) then a cut from T+1.5 s. Kept with boost GNSS good: the aided "
         "logs as flown, the median of the per-log medians (deg).\n",
         "| levelling | configuration | dropouts worse than unaided: mag 0 | +10 | +40 | all | worst end heading "
         "after a dropout, less unaided's | as flown, aided: worse than unaided | coast nose +10 / +40 | coast roll "
         "+10 / +40 | descent roll +10 / +40 | boost roll 0 / +10 / +40 | pitch vote, latest |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for lev, src, c in rows:
        rs = [r for k, r in src.items() if k[2] == c]
        if not rs:
            continue
        cnt = {m: [0, 0] for m in (0.0, 10.0, 40.0)}
        aided_bad = [0, 0]
        worst = -1e9
        for r in rs:
            u = U.get((r["run"], r["scn"], r["mag_err"]))
            if u is None:
                continue
            w = DT.worse(r, u, a.tol)
            if klass(r) == "real":
                aided_bad[1] += 1
                aided_bad[0] += bool(w)
                continue
            cnt[r["mag_err"]][1] += 1
            cnt[r["mag_err"]][0] += bool(w)
            if "end" in r["sum"] and "end" in u["sum"]:
                worst = max(worst, r["sum"]["end"]["heading"] - u["sum"]["end"]["heading"])
        tot = [sum(v[0] for v in cnt.values()), sum(v[1] for v in cnt.values())]

        def keep(ph, k, m):
            v = [r["sum"][ph][k][0] for r in rs if klass(r) == "real" and r["mag_err"] == m and ph in r["sum"]]
            return f"{np.median(v):.1f}" if v else "-"
        votes = [r["pitch_vote_t"] for r in rs if r.get("pitch_vote_t")]
        L.append(f"| {lev} | {c.replace('_none', '').replace('_', ' + ').replace('U + ref', 'unaided')} | "
                 + " | ".join(f"{cnt[m][0]} / {cnt[m][1]}" for m in (0.0, 10.0, 40.0))
                 + f" | {tot[0]} / {tot[1]} | {worst:+.1f} | {aided_bad[0]} / {aided_bad[1]} | "
                 f"{keep('coast', 'nose', 10.0)} / {keep('coast', 'nose', 40.0)} | "
                 f"{keep('coast', 'roll', 10.0)} / {keep('coast', 'roll', 40.0)} | "
                 f"{keep('descent', 'roll', 10.0)} / {keep('descent', 'roll', 40.0)} | "
                 f"{keep('boost', 'roll', 0.0)} / {keep('boost', 'roll', 10.0)} / {keep('boost', 'roll', 40.0)} | "
                 f"{max(votes):.3f} |")
    rs = [r for k, r in fw.items() if k[2] == "U_ref" and klass(r) == "real"]

    def keepu(ph, k, m):
        v = [r["sum"][ph][k][0] for r in rs if r["mag_err"] == m and ph in r["sum"]]
        return f"{np.median(v):.1f}" if v else "-"
    votes = [r["pitch_vote_t"] for k, r in fw.items() if k[2] == "U_ref" and r.get("pitch_vote_t")]
    L.append(f"| firmware | unaided (the reference) | - | - | - | - | - | - | {keepu('coast', 'nose', 10.0)} / "
             f"{keepu('coast', 'nose', 40.0)} | {keepu('coast', 'roll', 10.0)} / {keepu('coast', 'roll', 40.0)} | "
             f"{keepu('descent', 'roll', 10.0)} / {keepu('descent', 'roll', 40.0)} | {keepu('boost', 'roll', 0.0)} / "
             f"{keepu('boost', 'roll', 10.0)} / {keepu('boost', 'roll', 40.0)} | {max(votes):.3f} |")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
