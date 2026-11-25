#!/usr/bin/env python3
"""calibrated_pad.md: A, C and unaided with the hand-calibrated pad (calibrated.py) beside the uncalibrated rig runs
(run_matrix.py), pooled over the same logs, per phase, with the spread of the per-log medians.

    calib_tables.py CALIB_DIR MATRIX_DIRS OUT.md --logs r18,r20,...
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

PHASES = ("boost", "coast", "descent")
CFGS = ("A", "C", "unaided")
CFG_NAME = {"A": "A (full gate)", "C": "C (heading-only gate)", "unaided": "unaided (observe)"}
IMU = (("uncal", "uncalibrated (the rig)"), ("r0", "calibrated, 0 mg"), ("r2", "calibrated, 2 mg (4 directions)"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("calib", help="calibrated.py folder(s), comma-separated")
    ap.add_argument("matrix", help="run_matrix.py folder(s), comma-separated")
    ap.add_argument("out")
    ap.add_argument("--logs", required=True)
    a = ap.parse_args()
    logs = a.logs.split(",")
    tr = S.Truth()
    cal = {"results": [], "info": None}
    for cdir in a.calib.split(","):            # several calibrated.py folders: their runs together
        cj = json.load(open(os.path.join(cdir, "calib.json")))
        cal["results"] += cj["results"]
        if cal["info"] is None:
            cal["info"] = cj["info"]
        else:
            cal["info"]["pad"].update(cj["info"]["pad"])
    unc = []
    for m in a.matrix.split(","):
        unc += json.load(open(os.path.join(m, "matrix.json")))["results"]
    bases = {r: float(S.read_meta(os.path.join(HERE, "in", r + ".txt"))["t_base_s"]) for r in logs}
    E = {}   # (imu, cfg, mag) -> [(run, sample errors, score)]
    for r in unc:
        if r["run"] in logs and r["cfg"] in CFGS:
            ser = S.harness_series(r["csv"], bases[r["run"]])
            E.setdefault(("uncal", r["cfg"], r["mag_err"]), []).append((r["run"], S.sample_errors(ser, tr), r["score"]))
    for r in cal["results"]:
        if r["run"] in logs:
            imu = "r0" if r["residual"] == "r0" else "r2"
            ser = S.harness_series(r["csv"], bases[r["run"]])
            E.setdefault((imu, r["cfg"], r["mag_err"]), []).append((r["run"], S.sample_errors(ser, tr), r["score"]))
    mags = sorted({k[2] for k in E})
    info = cal["info"]
    L = []
    w = L.append
    w("# The hand-calibrated pad (offline replay)\n")
    w("The rig's replayed IMU carries the flight CSV's sensor errors. Over the pad's last 60 s the accelerometer reads "
      f"{np.round(np.array(info['acc_bias_mps2']) / 9.80665 * 1000, 2).tolist()} mg off and the gyro "
      f"{np.round(np.degrees(info['gyro_bias_rps']), 3).tolist()} deg/s off (body x forward, y right, z down). "
      "The lateral 10 mg is why the uncalibrated pad levels to a tilt 0.8-1.3 deg off. A hand calibration "
      "(each axis up and down) takes those off and leaves a residual. Here every tick gets the bias taken off and "
      f"a residual added: none, or {info['residual_mg']:.0f} mg in a random direction (4 directions). The run starts "
      "from the pad state the filter would have had with that IMU: the attitude the levelling and the magnetometer "
      "agree on (with no residual, the truth's to "
      f"{max(v['triad_vs_truth_deg'] for v in info['pad'].values()):.2f} deg), turned about the vertical by the "
      "magnetometer error. The accelerometer bias is set to the part of the residual along gravity (the part the pad "
      "can see), and the gyro bias the filter had is reduced by what was taken off. The uncalibrated rows are the "
      "same logs as recorded on the rig, the stress case.\n")
    w(f"Logs: {', '.join(logs)} (GNSS-aided; r18 started from a borrowed pad context). Each cell: median / p90 / max "
      "over every 0.1 s sample of the phase, pooled over the logs (and the 4 directions); in brackets, the lowest "
      "and highest per-log median. Degrees. Truth: burnout "
      f"T+{tr.burnout:.2f}, apogee T+{tr.apogee:.2f}.\n")

    def cell(entries, metric, phase):
        xs, meds = [], {}
        for run, e, _sc in entries:
            m = e["phase"] == phase
            if m.any():
                xs.append(e[metric][m])
                meds.setdefault(run, []).append(e[metric][m])
        if not xs:
            return "-"
        s3 = S.stats3(np.concatenate(xs))
        pm = [float(np.median(np.abs(np.concatenate(v)))) for v in meds.values()]
        return f"{s3[0]:.1f} / {s3[1]:.1f} / {s3[2]:.1f} ({min(pm):.1f}-{max(pm):.1f})"

    # the summary: the cells that decide it
    def c3(entries, metric, phase):
        xs = [e[metric][e["phase"] == phase] for _r, e, _s in entries if (e["phase"] == phase).any()]
        if not xs:
            return "-"
        s3 = S.stats3(np.concatenate(xs))
        return f"{s3[0]:.1f} / {s3[1]:.1f} / {s3[2]:.1f}"
    w("\n## Summary (median / p90 / max, deg; pooled over the logs)\n")
    w("| mag error | config | pad IMU | nose, coast | nose, descent | roll, coast | roll, descent | total, coast | "
      "pitch, coast | pitch < -5 deg, median T+ s |")
    w("|---|---|---|---|---|---|---|---|---|---|")
    for m in mags:
        for c in CFGS:
            for imu, iname in IMU:
                ent = E.get((imu, c, m))
                if not ent:
                    continue
                ts = [sc["pitch_vote_t"] for _r, _e, sc in ent if sc.get("pitch_vote_t") is not None]
                w(f"| {m:+.0f} | {CFG_NAME[c]} | {iname} | {c3(ent, 'nose', 'coast')} | {c3(ent, 'nose', 'descent')} | "
                  f"{c3(ent, 'roll', 'coast')} | {c3(ent, 'roll', 'descent')} | {c3(ent, 'att_total', 'coast')} | "
                  f"{c3(ent, 'pitch', 'coast')} | {np.median(ts):.3f} |")

    for metric, title in (("nose", "Nose-pointing error"), ("roll", "Roll error about the nose"),
                          ("att_total", "Total attitude error"), ("pitch", "Pitch (nose elevation) error")):
        w(f"\n## {title}\n")
        w("| mag error | config | pad IMU | " + " | ".join(PHASES) + " |")
        w("|---|---|---|" + "---|" * len(PHASES))
        for m in mags:
            for c in CFGS:
                for imu, iname in IMU:
                    ent = E.get((imu, c, m))
                    if not ent:
                        continue
                    w(f"| {m:+.0f} | {CFG_NAME[c]} | {iname} | " + " | ".join(cell(ent, metric, p) for p in PHASES)
                      + " |")

    w("\n## Apogee pitch voter: first estimated pitch below -5 deg after burnout (T+ s)\n")
    w(f"The truth's pitch crosses at T+{tr.pitch_vote:.3f} s ({tr.pitch_vote - tr.apogee:.2f} s after the true "
      f"apogee, T+{tr.apogee:.3f}). On board, the velocity vote comes first and this one makes the call.\n")
    w("| mag error | config | pad IMU | median | earliest .. latest |")
    w("|---|---|---|---|---|")
    for m in mags:
        for c in CFGS:
            for imu, iname in IMU:
                ent = E.get((imu, c, m))
                if not ent:
                    continue
                ts = [sc["pitch_vote_t"] for _r, _e, sc in ent if sc.get("pitch_vote_t") is not None]
                if ts:
                    w(f"| {m:+.0f} | {CFG_NAME[c]} | {iname} | {np.median(ts):.3f} | {min(ts):.3f} .. {max(ts):.3f} |")

    # health
    bad = [(k, r) for k, v in E.items() for r, _e, sc in v if sc["unhealthy_samples"] > 0]
    w(f"\nHealth: {sum(len(v) for v in E.values())} runs, {len(bad)} with an unhealthy or non-finite sample.\n")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
