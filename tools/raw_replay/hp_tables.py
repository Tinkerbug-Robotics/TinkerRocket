#!/usr/bin/env python3
"""hybrid_prior.md: A, C and the hybrid H (heading-only gate to the apogee call, then the full gate) at launch heading
priors of 30 and 10 deg, magnetometer -40..+40, on the calibrated pad (calibrated.py, CALIB_STUDY=hybrid).

    hp_tables.py STUDY_DIRS OUT.md --logs ...
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

CFGS = ("A30", "C30", "H30", "A10", "C10", "H10", "LA30", "LA10", "LC30", "unaided")
NAME = {"A30": "A, 30 deg prior", "C30": "C, 30 deg prior", "H30": "H (C then A), 30 deg",
        "A10": "A, 10 deg prior", "C10": "C, 10 deg prior", "H10": "H (C then A), 10 deg",
        "LA30": "act from apogee, full gate, 30 deg", "LA10": "act from apogee, full gate, 10 deg",
        "LC30": "act from apogee, heading-only gate, 30 deg", "unaided": "unaided (observe)"}
IMU = (("r0", "0 mg"), ("r2", "2 mg (4 dirs)"))
COLS = (("nose", "boost"), ("nose", "coast"), ("nose", "descent"), ("roll", "boost"), ("roll", "coast"),
        ("roll", "descent"), ("att_total", "boost"), ("att_total", "coast"), ("att_total", "descent"),
        ("pitch", "coast"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("study")
    ap.add_argument("out")
    ap.add_argument("--logs", required=True)
    a = ap.parse_args()
    logs = a.logs.split(",")
    tr = S.Truth()
    res = []
    for d in a.study.split(","):
        res += json.load(open(os.path.join(d, "calib.json")))["results"]
    bases = {r: float(S.read_meta(os.path.join(HERE, "in", r + ".txt"))["t_base_s"]) for r in logs}
    E = {}
    for r in res:
        if r["run"] not in logs:
            continue
        imu = "r0" if r["residual"] == "r0" else "r2"
        ser = S.harness_series(r["csv"], bases[r["run"]])
        E.setdefault((imu, r["cfg"], r["mag_err"]), []).append((r["run"], S.sample_errors(ser, tr), r["score"]))
    mags = sorted({k[2] for k in E})

    def c3(ent, metric, phase):
        xs = [e[metric][e["phase"] == phase] for _r, e, _s in ent if (e["phase"] == phase).any()]
        if not xs:
            return "-"
        s3 = S.stats3(np.concatenate(xs))
        return f"{s3[0]:.1f} / {s3[1]:.1f} / {s3[2]:.1f}"

    L = ["# Hybrid gate and launch heading prior, calibrated pad (offline replay)\n",
         "A: act, full gate from launch. C: act, heading-only gate. H: the heading-only gate through the ascent, "
         "then the full gate from the apogee call on (the log's own call, T+12.57; a harness-only switch where the "
         "shadow sets the gate). All with the 0.6 deg launch tilt floor, no hand-off, carrier observed; launch heading "
         "prior 30 or 10 deg. Calibrated pad as in calibrated_pad.md: residual 0, or 2 mg in 4 random directions. "
         f"Logs: {', '.join(logs)}. Cells: median / p90 / max, degrees, pooled over every 0.1 s sample of the phase "
         "and the logs; pitch vote: median T+ s of the first estimated pitch below -5 deg (truth "
         f"T+{tr.pitch_vote:.3f}, true apogee T+{tr.apogee:.3f}).\n",
         "| mag error | config | pad IMU | " + " | ".join(f"{m.replace('att_total', 'total')}, {p}" for m, p in COLS)
         + " | pitch vote |",
         "|---|---|---|" + "---|" * (len(COLS) + 1)]
    for m in mags:
        for c in CFGS:
            for imu, iname in IMU:
                ent = E.get((imu, c, m))
                if not ent:
                    continue
                ts = [sc["pitch_vote_t"] for _r, _e, sc in ent if sc.get("pitch_vote_t") is not None]
                L.append(f"| {m:+.0f} | {NAME[c]} | {iname} | " + " | ".join(c3(ent, k, p) for k, p in COLS) +
                         f" | {np.median(ts):.3f} |")
    bad = sum(1 for v in E.values() for _r, _e, sc in v if sc["unhealthy_samples"] > 0)
    L.append(f"\nHealth: {sum(len(v) for v in E.values())} runs, {bad} with an unhealthy or non-finite sample.\n")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
