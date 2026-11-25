#!/usr/bin/env python3
"""The comparison tables from run_matrix.py's runs: every 0.1 s sample of every GNSS-aided log pooled per phase.

    tables.py MATRIX_DIR OUT.md [--pool r20,r21,r22,r23,r25] [--controls r16,r19,r26]
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

CFG_ORDER = ["A", "C", "unaided", "A+cf", "C+cf"]
CFG_NAME = {"A": "A (full gate)", "C": "C (heading-only gate)", "unaided": "unaided (observe)",
            "A+cf": "A + carrier fuse*", "C+cf": "C + carrier fuse*"}
PHASES = ["boost", "coast", "descent"]


def f3(x, nd=1):
    if x is None or not np.all(np.isfinite(x)):
        return "-"
    return " / ".join(f"{v:.{nd}f}" for v in x)


def pooled(errs, metric, phase):
    xs = [e[metric][e["phase"] == phase] for e in errs]
    x = np.concatenate(xs) if xs else np.zeros(0)
    return S.stats3(x) if x.size else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("matrix", help="the run_matrix.py folder(s), comma-separated")
    ap.add_argument("out")
    ap.add_argument("--pool", default="r20,r21,r22,r23,r25")
    ap.add_argument("--controls", default="r16,r19,r26")
    ap.add_argument("--approx", default="r18")
    a = ap.parse_args()
    d = {"results": []}
    for mdir in a.matrix.split(","):            # several matrix folders: their runs together
        d["results"] += json.load(open(os.path.join(mdir, "matrix.json")))["results"]
    tr = S.Truth()
    pool = a.pool.split(",")
    controls = a.controls.split(",")
    approx = a.approx.split(",") if a.approx else []
    res = d["results"]
    errs = {}      # (run, cfg, mag) -> sample errors
    bases = {}
    for r in res:
        run = r["run"]
        if run not in bases:
            bases[run] = float(S.read_meta(os.path.join(HERE, "in", run + ".txt"))["t_base_s"])
        ser = S.harness_series(r["csv"], bases[run])
        errs[(run, r["cfg"], r["mag_err"])] = S.sample_errors(ser, tr)
    mags = sorted({r["mag_err"] for r in res})
    L = []
    w = L.append
    w("# Raw-GNSS attitude configurations on the RF-rig logs (offline replay)\n")
    w(f"Truth: burnout T+{tr.burnout:.2f} s, apogee T+{tr.apogee:.2f} s, landing T+{tr.landed:.2f} s; the truth's "
      f"own pitch first goes below -5 deg at T+{tr.pitch_vote:.2f} s.\n")
    w(f"Pooled over the GNSS-aided logs {', '.join(pool)} (every 0.1 s sample of each phase; descent runs from "
      "apogee to the end of each log, at most the landing). Each cell is median / p90 / max, degrees unless "
      "marked. Code: fc/gnss-heading-fuse dc37cf07, replayed by tools/raw_replay.\n")
    w("Configurations: **A** = act, full gate, launch tilt floor 0.6 deg, 30 deg heading prior, no hand-off, "
      "carrier observed. **C** = the same with the heading-only gate. **unaided** = observe (TR_HEADING_FIT=1). "
      "*Carrier-fuse variants: the carrier update has no attitude columns yet, so these only show what the "
      "velocity-only carrier update does.\n")

    def table(title, metric, nd=1, cfgs=None):
        w(f"\n### {title}\n")
        hdr = "| mag error | config | " + " | ".join(PHASES) + " |"
        w(hdr)
        w("|" + "---|" * (2 + len(PHASES)))
        for m in mags:
            for c in (cfgs or CFG_ORDER):
                if not any((run, c, m) in errs for run in pool):
                    continue
                e = [errs[(run, c, m)] for run in pool if (run, c, m) in errs]
                cells = [f3(pooled(e, metric, p), nd) for p in PHASES]
                w(f"| {m:+.0f} | {CFG_NAME[c]} | " + " | ".join(cells) + " |")

    w("\n## Attitude\n")
    table("Nose-pointing error (angle between the estimated and true nose)", "nose")
    table("Roll error about the nose", "roll")
    table("Total attitude error (the whole error rotation)", "att_total")
    table("Heading error (twist about vertical; a diagnostic only, ill-defined near vertical)", "heading")
    w("\n## Pyro-relevant\n")
    table("Pitch (nose elevation) error", "pitch", 2)
    table("Vertical velocity error (m/s)", "vz", 2)

    # the apogee pitch voter
    w("\n### Apogee pitch voter: first time after burnout the estimated pitch is below -5 deg (T+ s)\n")
    w(f"The voter fires on pitch < -5 deg held over leaky counters (TR_KinematicChecks, ~6 loop passes); the "
      f"truth's pitch crosses at T+{tr.pitch_vote:.3f} s, {tr.pitch_vote - tr.apogee:.2f} s after the true "
      f"apogee (T+{tr.apogee:.3f}). Margin before true apogee = apogee - crossing (negative = after apogee).\n")
    runs_all = [r for r in pool + approx + controls if any(x["run"] == r for x in res)]
    w("| mag error | config | " + " | ".join(runs_all) + " | margin range (s) |")
    w("|" + "---|" * (3 + len(runs_all)))
    for m in mags:
        for c in CFG_ORDER:
            row, margins = [], []
            for run in runs_all:
                rr = [r for r in res if r["run"] == run and r["cfg"] == c and r["mag_err"] == m]
                t = rr[0]["score"]["pitch_vote_t"] if rr else None
                row.append("-" if t is None else f"{t:.3f}")
                if t is not None:
                    margins.append(tr.apogee - t)
            if all(x == "-" for x in row):
                continue
            mr = f"{min(margins):+.3f} .. {max(margins):+.3f}" if margins else "-"
            w(f"| {m:+.0f} | {CFG_NAME[c]} | " + " | ".join(row) + f" | {mr} |")

    # the apogee call: the second voter (TR_KinematicChecks, 2 of 3-4), with each log's own barometer and GNSS votes
    vj = os.path.join(HERE, "voters.json")
    if os.path.exists(vj):
        vot = json.load(open(vj))
        w("\n### The apogee call these give: the second of the votes (T+ s)\n")
        w("Velocity and pitch from the run; the barometer's and GNSS's own votes from each log (they do not use the "
          "navigation filter). On board, the velocity vote came first (T+11.75-11.82) and the pitch vote second, "
          "before the barometer's (T+12.70), in every rig run: the pitch voter made the call.\n")
        w("| mag error | config | " + " | ".join(runs_all) + " | call less the on-board call (s) |")
        w("|" + "---|" * (3 + len(runs_all)))
        for m in mags:
            for c in CFG_ORDER:
                row, dif = [], []
                for run in runs_all:
                    rr = [r for r in res if r["run"] == run and r["cfg"] == c and r["mag_err"] == m]
                    v = vot.get(run)
                    if not rr or v is None:
                        row.append("-")
                        continue
                    sc = rr[0]["score"]
                    votes = sorted(x for x in (sc.get("vel_vote_t"), sc.get("pitch_vote_t"), v.get("baro"),
                                               v.get("gps")) if x is not None)
                    call = votes[1] if len(votes) >= 2 else None
                    row.append("-" if call is None else f"{call:.3f}")
                    if call is not None and v.get("apogee") is not None:
                        dif.append(call - v["apogee"])
                if all(x == "-" for x in row):
                    continue
                w(f"| {m:+.0f} | {CFG_NAME[c]} | " + " | ".join(row) + " | " +
                  (f"{min(dif):+.3f} .. {max(dif):+.3f}" if dif else "-") + " |")

    # health
    w("\n### Health: samples from T+0 where the filter was not running or not finite\n")
    bad = [(r["run"], r["cfg"], r["mag_err"], r["score"]["unhealthy_samples"], r["score"]["samples"],
            r["score"]["max_gap_s"]) for r in res]
    nb = [b for b in bad if b[3] > 0]
    w(f"{len(bad)} runs; {len(nb)} with any unhealthy or non-finite sample"
      + ("" if not nb else ": " + ", ".join(f"{b[0]} {b[1]} {b[2]:+.0f} ({b[3]}/{b[4]})" for b in nb)) +
      f". Longest gap between rows: {max(b[5] for b in bad):.3f} s.\n")

    w("\n## Position and velocity (so a regression shows)\n")
    table("Horizontal position error (m)", "pos_h", 1)
    table("Vertical position error (m)", "pos_v", 1)
    table("Velocity error, 3-D (m/s)", "vel", 2)

    # controls
    w("\n## Controls: no GNSS in flight (" + ", ".join(controls) + ")\n")
    w("The receiver had lost the sky before launch, so only the IMU, barometer and (on the ground) the "
      "magnetometer reach the filter. Pooled the same way.\n")
    for metric, title in (("nose", "Nose-pointing error"), ("roll", "Roll error about the nose"),
                          ("pos_h", "Horizontal position error (m)")):
        w(f"\n### {title}\n")
        w("| mag error | config | " + " | ".join(PHASES) + " |")
        w("|" + "---|" * (2 + len(PHASES)))
        for m in mags:
            for c in ("A", "C", "unaided"):
                e = [errs[(run, c, m)] for run in controls if (run, c, m) in errs]
                if not e:
                    continue
                w(f"| {m:+.0f} | {CFG_NAME[c]} | " + " | ".join(f3(pooled(e, metric, p), 1) for p in PHASES)
                  + " |")

    # per run
    w("\n## Per log: nose-pointing and roll error, coast and descent (median / max)\n")
    w("| log | mag error | config | coast nose | coast roll | descent nose | descent roll | descent heading |")
    w("|---|---|---|---|---|---|---|---|")
    for run in pool + approx + controls:
        for m in mags:
            for c in CFG_ORDER:
                e = errs.get((run, c, m))
                if e is None:
                    continue
                cells = []
                for p, met in (("coast", "nose"), ("coast", "roll"), ("descent", "nose"), ("descent", "roll"),
                               ("descent", "heading")):
                    s3 = S.stats3(e[met][e["phase"] == p]) if (e["phase"] == p).any() else None
                    cells.append("-" if s3 is None else f"{s3[0]:.1f} / {s3[2]:.1f}")
                w(f"| {run}{' (approx.)' if run in approx else ''} | {m:+.0f} | {CFG_NAME[c]} | " +
                  " | ".join(cells) + " |")
    open(a.out, "w").write("\n".join(L) + "\n")

    # every number, one row per run x config x phase
    with open(os.path.splitext(a.out)[0] + "_per_run.csv", "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["run", "config", "mag_err_deg", "phase", "n"] +
                    [f"{m}_{s}" for m in S.METRICS for s in ("med", "p90", "max")] + ["pitch_vote_t_s"])
        for r in res:
            e = errs[(r["run"], r["cfg"], r["mag_err"])]
            for p in PHASES:
                msk = e["phase"] == p
                if not msk.any():
                    continue
                vals = []
                for met in S.METRICS:
                    vals += [f"{v:.4f}" for v in S.stats3(e[met][msk])]
                cw.writerow([r["run"], r["cfg"], r["mag_err"], p, int(msk.sum())] + vals +
                            [r["score"]["pitch_vote_t"]])
    print("wrote", a.out)


if __name__ == "__main__":
    main()
