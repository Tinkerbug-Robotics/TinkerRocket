#!/usr/bin/env python3
"""Boost GNSS dropouts, real and synthesized, against the safeguards, H as the base:

    dropout_sweep.py OUT_DIR IMU RUN [RUN ...]        (IMU: cal = the calibrated pad, 0 mg; uncal = the rig as recorded)

Scenarios on each log: as recorded ("real"); every epoch cut from T+0.5, 1.5 or 3 s until T+8 s, T+12 s or the end;
one epoch corrupted like c30's T+1 (3 satellites' range rates off by 30..70 m/s, pseudoranges by 20..40 m; two
draws) then everything cut from T+1.5 s until T+8 s, T+12 s or the end.
Configurations: H (C until the apogee call, then A) at a 30 and a 10 deg launch heading prior with
  none, a4 (a boost epoch whose Dopplers point to a velocity correction over 5 m/s dropped whole), b2 (no satellite
  turns the heading more than 5 deg in a boost epoch), a4+b2, b2+c (c: chi2 3.84 above 3 g), a4+b2+c;
and references A30, C30 and unaided (observe). Magnetometer 0, +10, +40 deg.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import calibrated as CB  # noqa: E402
import run_matrix as RM  # noqa: E402
import scorelib as S  # noqa: E402

MAGS = (0.0, 10.0, 40.0)
A4 = ["--guard-dv-max", "5", "--guard-reject-epoch"]
A5 = ["--guard-dv-max", "5", "--guard-dv-sigma", "3", "--guard-reject-epoch"]
B2 = ["--guard-step-deg", "5"]
C = ["--guard-gate-chi2", "3.84", "--guard-tight-g", "2.0"]
A6 = ["--guard-majority"]
SAFE = {"none": [], "a4": A4, "b2": B2, "a4b2": A4 + B2, "b2c": B2 + C, "a4b2c": A4 + B2 + C,
        "a5": A5, "a5b2": A5 + B2, "a4c": A4 + C, "a6": A6, "a6b2": A6 + B2,
        "a6h": ["--guard-majority-half"], "a6hb2": ["--guard-majority-half"] + B2}
# DROP_CFGS: a comma-separated subset of the configurations (e.g. after a harness fix that touched only some)
ONLY = [x for x in os.environ.get("DROP_CFGS", "").split(",") if x]
BINARY = os.environ.get("DROP_BINARY", "")


def prior(base, p):
    return [x if x != "30" else str(p) for x in base]


def configs():
    out = {}
    for p in (30, 10):
        for g, f in SAFE.items():
            out[f"H{p}_{g}"] = prior(RM.BASE_C, p) + ["--gate-full-from-apogee"] + f
    out["A30_none"] = list(RM.BASE_A)
    out["C30_none"] = list(RM.BASE_C)
    out["U_ref"] = list(RM.BASE_U)
    return {k: v for k, v in out.items() if not ONLY or k in ONLY}


def scenarios():
    out = {"real": []}
    for a in (0.5, 1.5, 3.0):
        for b, bn in ((8.0, "8"), (12.0, "12"), (1000.0, "end")):
            out[f"cut{a:g}-{bn}"] = ["--cut-from-s", str(a), "--cut-to-s", str(b)]
    for sd in (1, 2):
        for b, bn in ((8.0, "8"), (12.0, "12"), (1000.0, "end")):
            out[f"bad{sd}-cut1.5-{bn}"] = ["--corrupt-at-s", "1.0", "--corrupt-seed", str(sd), "--cut-from-s", "1.5",
                                           "--cut-to-s", str(b)]
    return out


def summarize(e, tr, t_last):
    out = {}
    for ph in ("boost", "coast", "descent"):
        m = e["phase"] == ph
        if m.any():
            out[ph] = {k: [float(np.median(np.abs(e[k][m]))), float(np.percentile(np.abs(e[k][m]), 90)),
                           float(np.max(np.abs(e[k][m])))] for k in ("nose", "roll", "heading", "pitch")}
    t_end = min(t_last, tr.landed)
    m = (e["t"] > t_end - 2.0) & (e["t"] <= t_end)
    if m.any():
        out["end"] = {k: float(np.median(np.abs(e[k][m]))) for k in ("nose", "roll", "heading")}
    return out


def main():
    out_dir, imu = sys.argv[1], sys.argv[2]
    runs = sys.argv[3:]
    tr = S.Truth()
    ba, bw = CB.csv_bias()
    binary = BINARY or os.path.join(HERE, "build", "raw_replay")
    cfgs, scns = configs(), scenarios()
    for run in runs:
        d = os.path.join(out_dir, imu, run)
        os.makedirs(d, exist_ok=True)
        ev = os.path.join(HERE, "in", run + ".txt")
        base = float(S.read_meta(ev)["t_base_s"])
        f_pad, _w, m_pad, pnav = CB.pad_means(ev, base)
        rec = CB.RECORDED[run]
        q_pad0 = CB.triad(f_pad - ba, m_pad, pnav[22] - math.radians(rec))
        results = []
        for scn, sflags in scns.items():
            for cfg, cflags in cfgs.items():
                for mag in MAGS:
                    tag = f"{scn}_{cfg}_m{int(mag)}"
                    # the recorded scenario's rows kept; the synthesized ones' summarized, their rows overwritten
                    csv = os.path.join(d, tag + ".csv") if scn == "real" else os.path.join(d, "synth_last.csv")
                    cmd = [binary, "--in", ev, "--out", csv, "--t-base-s", repr(base), "--every", "10"] + cflags + \
                        sflags + ["--mag-error-deg", str(mag), "--recorded-mag-error-deg", str(rec)]
                    if imu == "cal":
                        q_pad = CB.qmul(CB.qz(mag), q_pad0)
                        cmd += ["--acc-sub", ",".join(repr(float(x)) for x in ba),
                                "--gyro-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-wb-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-quat", ",".join(repr(float(x)) for x in q_pad), "--pad-ab", "0,0,0"]
                    p = subprocess.run(cmd, capture_output=True, text=True)
                    if p.returncode != 0:
                        raise SystemExit(p.stderr)
                    ser = S.harness_series(csv, base)
                    e = S.sample_errors(ser, tr)
                    sc = S.score(ser, tr)
                    results.append({"run": run, "scn": scn, "cfg": cfg, "mag_err": mag, "imu": imu,
                                    "sum": summarize(e, tr, float(ser["t"][-1])),
                                    "pitch_vote_t": sc.get("pitch_vote_t"), "events": ser.get("events"),
                                    "unhealthy": sc["unhealthy_samples"]})
            print(f"{run} {imu} {scn}: done", flush=True)
        json.dump({"results": results}, open(os.path.join(d, "dropout.json"), "w"), default=float)


if __name__ == "__main__":
    main()
