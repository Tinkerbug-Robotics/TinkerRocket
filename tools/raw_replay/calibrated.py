#!/usr/bin/env python3
"""The hand-calibrated pad, emulated on the rig logs: A, C and unaided at a magnetometer error of 0, +10 and +40 deg,
the replayed IMU's bias taken off (a residual of 0, or 2 mg in a random direction), against the uncalibrated runs.

    calibrated.py OUT_DIR RUN [RUN ...]       (runs whose events are in in/RUN.txt)

With CALIB_STUDY=hybrid: A, C and the hybrid H (the heading-only gate through the ascent, the full gate from the
apogee call on: --gate-full-from-apogee) at launch heading priors of 30 and 10 deg, magnetometer -40, -10, 0, +10,
+40; rows every 0.1 s (the apogee votes from the harness's own every-step events).

The rig's replayed IMU carries the truth CSV's sensor errors: on the pad (its last 60 s) the accelerometer reads
mean(f - f_true) and the gyro mean(w - w_true) off, body FRD. A hand calibration takes those off, leaving a residual.
The harness then gets every tick less (bias - residual), and starts from the pad state the filter would have had
with that IMU: the attitude the levelling and the magnetometer agree on (TRIAD: gravity from the calibrated
accelerometer's pad mean, north from the magnetometer's pad mean and the flight computer's declination; with no
residual it is the truth's), turned about the vertical by the magnetometer error; the accelerometer bias it can see
(the residual along gravity); the gyro bias it had, less what came off.
"""
import gzip
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402
import run_matrix as RM  # noqa: E402

G = 9.80665
RESIDUAL_MG = 2.0
SEEDS = (1, 2, 3, 4)
MAGS = (0.0, 10.0, 40.0)
CFGS = {"A": RM.BASE_A, "C": RM.BASE_C, "unaided": RM.BASE_U}
EVERY = []
if os.environ.get("CALIB_STUDY") == "hybrid":
    def _prior(base, p):
        return [x if x != "30" else str(p) for x in base]
    CFGS = {}
    for _p in (30, 10):
        CFGS[f"A{_p}"] = _prior(RM.BASE_A, _p)
        CFGS[f"C{_p}"] = _prior(RM.BASE_C, _p)
        CFGS[f"H{_p}"] = _prior(RM.BASE_C, _p) + ["--gate-full-from-apogee"]
    MAGS = (-40.0, -10.0, 0.0, 10.0, 40.0)
    EVERY = ["--every", "10"]
if os.environ.get("CALIB_STUDY") == "late":
    # act opened at the apogee call instead of launch (observe until then; a harness-only switch)
    _late = ["--heading-mode", "1", "--carrier", "1", "--tilt-sigma-deg", "0.6", "--handoff-deg", "-1",
             "--act-from-apogee"]
    CFGS = {"LA30": _late + ["--gate-full", "1", "--pad-sigma-deg", "30"],
            "LA10": _late + ["--gate-full", "1", "--pad-sigma-deg", "10"],
            "LC30": _late + ["--gate-full", "0", "--pad-sigma-deg", "30"]}
    MAGS = (-40.0, -10.0, 0.0, 10.0, 40.0)
    EVERY = ["--every", "10"]
if os.environ.get("CALIB_PRIOR10"):   # a variant: A and C with a 10 deg heading prior instead of 30
    CFGS = {"A10": [x if x != "30" else "10" for x in RM.BASE_A], "C10": [x if x != "30" else "10" for x in RM.BASE_C]}
if os.environ.get("CALIB_UNCAL"):     # the same configurations on the uncalibrated IMU (the rig as recorded)
    UNCAL = True
else:
    UNCAL = False
RECORDED = {"r18": 40.0, "r20": 40.0, "r21": 0.0, "r22": 10.0, "r23": 10.0, "r25": 5.0, "f3c": 10.0, "c30": 10.0,
            "f4b": 40.0, "f4a": 0.0, "f2": 10.0, "r16": 0.0, "r19": 0.0, "r26": 5.0, "c30b": 10.0}


def csv_bias():
    """The truth CSV's accelerometer and gyro errors over the pad's last 60 s (body FRD; m/s^2, rad/s)."""
    lines = [ln for ln in gzip.open(S.TRUTH_CSV, "rt") if not ln.startswith("#")]
    names = lines[0].strip().split(",")
    d = np.loadtxt(lines[1:], delimiter=",")
    c = {n: d[:, i] for i, n in enumerate(names)}
    m = (c["phase"] == 0) & (c["t_s"] > -60.0) & (c["t_s"] < 0.0)
    ba = np.array([(c[f"f{a}_mps2"] - c[f"f{a}_true_mps2"])[m].mean() for a in "xyz"])
    bw = np.array([(c[f"w{a}_rps"] - c[f"w{a}_true_rps"])[m].mean() for a in "xyz"])
    return ba, bw


def quat_from_dcm(R):
    """Body -> NED rotation matrix to a scalar-first quaternion (v_ned = q v_b q*)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def triad(f_b, m_b, decl_rad):
    """The attitude (body -> NED) with gravity exactly along -f_b and the field's horizontal part at decl_rad."""
    def unit(v):
        return v / np.linalg.norm(v)
    t1b = unit(-f_b)                                   # down, in the body
    t2b = unit(np.cross(t1b, m_b))
    t3b = np.cross(t1b, t2b)
    t1n = np.array([0.0, 0.0, 1.0])
    mn = np.array([math.cos(decl_rad), math.sin(decl_rad), 0.0])
    t2n = unit(np.cross(t1n, mn))
    t3n = np.cross(t1n, t2n)
    R = np.stack([t1n, t2n, t3n], 1) @ np.stack([t1b, t2b, t3b], 1).T
    return quat_from_dcm(R)


def qz(deg):
    a = math.radians(deg)
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


def qmul(a, b):
    return S.qmul(a[None, :], b[None, :])[0]


def pad_means(events, base):
    """The log's pad (flight time -1.1 .. -0.05 s): the mean IMU tick and magnetometer sample, body FRD."""
    I, M, pnav = [], [], None
    for ln in open(events):
        if ln.startswith("I "):
            I.append([float(x) for x in ln.split()[1:8]])
        elif ln.startswith("M "):
            M.append([float(x) for x in ln.split()[1:5]])
        elif ln.startswith("PNAV "):
            pnav = [float(x) for x in ln.split()[1:]]
    I, M = np.array(I), np.array(M)
    ti, tm = I[:, 0] * 1e-6 + base, M[:, 0] * 1e-6 + base
    mi, mm = (ti > -1.1) & (ti < -0.05), (tm > -1.1) & (tm < -0.05)
    return I[mi, 1:4].mean(0), I[mi, 4:7].mean(0), M[mm, 1:4].mean(0), pnav


def main():
    out = sys.argv[1]
    runs = sys.argv[2:]
    os.makedirs(out, exist_ok=True)
    tr = S.Truth()
    ba, bw = csv_bias()
    binary = os.path.join(HERE, "build", "raw_replay")
    rng_dirs = {}
    for sd in SEEDS:
        v = np.random.default_rng(sd).normal(size=3)
        rng_dirs[sd] = v / np.linalg.norm(v)
    results = []
    info = {"acc_bias_mps2": ba.tolist(), "gyro_bias_rps": bw.tolist(), "residual_mg": RESIDUAL_MG,
            "seed_dirs": {k: v.tolist() for k, v in rng_dirs.items()}, "pad": {}}
    for run in runs:
        ev = os.path.join(HERE, "in", run + ".txt")
        base = float(S.read_meta(ev)["t_base_s"])
        f_pad, w_pad, m_pad, pnav = pad_means(ev, base)
        rec = RECORDED[run]
        decl = pnav[22] - math.radians(rec)                 # the flight computer's own declination, bench error off
        _, _, qt = tr.at(np.array([-0.5]))
        for res_tag, r in [("r0", np.zeros(3))] + [(f"r2s{sd}", RESIDUAL_MG * 1e-3 * G * rng_dirs[sd]) for sd in SEEDS]:
            acc_sub = ba - r
            f_cal = f_pad - acc_sub
            q_pad0 = triad(f_cal, m_pad, decl)
            u = f_cal / np.linalg.norm(f_cal)
            ab = float(np.dot(r, u)) * u                     # the residual along gravity: what the pad can see
            if res_tag == "r0":
                err = S.att_errors(q_pad0[None, :], qt)
                info["pad"][run] = {"triad_vs_truth_deg": float(err[0][0]), "heading_deg": float(err[3][0])}
            for cfg, flags in CFGS.items():
                for mag in MAGS:
                    q_pad = qmul(qz(mag), q_pad0)
                    tag = f"{run}_{cfg}_m{int(mag)}_{res_tag}"
                    csv = os.path.join(out, tag + ".csv")
                    cmd = [binary, "--in", ev, "--out", csv, "--t-base-s", repr(base)] + EVERY + flags + [
                        "--mag-error-deg", str(mag), "--recorded-mag-error-deg", str(rec)]
                    if not UNCAL:
                        cmd += ["--acc-sub", ",".join(repr(float(x)) for x in acc_sub),
                                "--gyro-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-wb-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-quat", ",".join(repr(float(x)) for x in q_pad),
                                "--pad-ab", ",".join(repr(float(x)) for x in ab)]
                    elif res_tag != "r0":
                        continue
                    p = subprocess.run(cmd, capture_output=True, text=True)
                    if p.returncode != 0:
                        raise SystemExit(p.stderr)
                    ser = S.harness_series(csv, base)
                    sc = S.score(ser, tr)
                    results.append({"run": run, "cfg": cfg, "mag_err": mag, "residual": res_tag, "csv": csv,
                                    "score": sc})
            print(f"{run} {res_tag}: done", flush=True)
    json.dump({"info": info, "results": results}, open(os.path.join(out, "calib.json"), "w"), indent=1,
              default=float)
    print(json.dumps(info["pad"], indent=1))


if __name__ == "__main__":
    main()
