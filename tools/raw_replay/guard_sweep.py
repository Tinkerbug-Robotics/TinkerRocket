#!/usr/bin/env python3
"""Boost safeguards that keep GNSS active in boost, every combination on a set of logs (harness-only switches):

    guard_sweep.py OUT_DIR RUN [RUN ...]

A (full gate), C (heading-only gate), H (C until the apogee call, then A), each at a launch heading prior of 10 and
30 deg, magnetometer 0, +10, +40, with each safeguard on its own:
  none   no safeguard
  a1     the attitude held out of a boost epoch with fewer than 6 pseudoranges
  a3     ... or whose Dopplers point to a velocity correction over 5 m/s (an epoch the receiver lost track in)
  b1     a satellite refused that would turn the heading past 2 sigma of its 1-sigma in a boost epoch
  b2     ... past 5 deg
  c      the innovation gate tightened to chi2 3.84 (2 sigma) while the IMU peak is above 3 g
and the comparison d: act opened at the apogee call (observe until then; LA full gate, LC heading-only gate), and
unaided (observe). "Boost": an epoch whose IMU peak since the last one is above 2 g. Each on the uncalibrated rig IMU
and on the calibrated pad (0 mg residual).
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
GUARDS = {"none": [], "a1": ["--guard-min-pr", "6"], "a3": ["--guard-dv-max", "5"],
          "b1": ["--guard-step-sigma", "2"], "b2": ["--guard-step-deg", "5"],
          "c": ["--guard-gate-chi2", "3.84", "--guard-tight-g", "2.0"]}


def prior(base, p):
    return [x if x != "30" else str(p) for x in base]


def configs():
    out = {}
    for p in (10, 30):
        for g, gf in GUARDS.items():
            out[f"A{p}_{g}"] = prior(RM.BASE_A, p) + gf
            out[f"C{p}_{g}"] = prior(RM.BASE_C, p) + gf
            out[f"H{p}_{g}"] = prior(RM.BASE_C, p) + ["--gate-full-from-apogee"] + gf
        late = ["--heading-mode", "1", "--carrier", "1", "--tilt-sigma-deg", "0.6", "--handoff-deg", "-1",
                "--act-from-apogee", "--pad-sigma-deg", str(p)]
        out[f"LA{p}_d"] = late + ["--gate-full", "1"]
        out[f"LC{p}_d"] = late + ["--gate-full", "0"]
    out["U_ref"] = list(RM.BASE_U)
    return out


def main():
    out = sys.argv[1]
    runs = sys.argv[2:]
    tr = S.Truth()
    ba, bw = CB.csv_bias()
    binary = os.path.join(HERE, "build", "raw_replay")
    cfgs = configs()
    for run in runs:
        d = os.path.join(out, run)
        os.makedirs(d, exist_ok=True)
        ev = os.path.join(HERE, "in", run + ".txt")
        base = float(S.read_meta(ev)["t_base_s"])
        f_pad, w_pad, m_pad, pnav = CB.pad_means(ev, base)
        rec = CB.RECORDED[run]
        decl = pnav[22] - math.radians(rec)
        q_pad0 = CB.triad(f_pad - ba, m_pad, decl)
        results = []
        for imu in ("uncal", "cal"):
            for cfg, flags in cfgs.items():
                for mag in MAGS:
                    tag = f"{cfg}_m{int(mag)}_{imu}"
                    csv = os.path.join(d, tag + ".csv")
                    cmd = [binary, "--in", ev, "--out", csv, "--t-base-s", repr(base), "--every", "10"] + flags + [
                        "--mag-error-deg", str(mag), "--recorded-mag-error-deg", str(rec)]
                    if imu == "cal":
                        q_pad = CB.qmul(CB.qz(mag), q_pad0)
                        cmd += ["--acc-sub", ",".join(repr(float(x)) for x in ba),
                                "--gyro-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-wb-sub", ",".join(repr(float(x)) for x in bw),
                                "--pad-quat", ",".join(repr(float(x)) for x in q_pad),
                                "--pad-ab", "0,0,0"]
                    p = subprocess.run(cmd, capture_output=True, text=True)
                    if p.returncode != 0:
                        raise SystemExit(p.stderr)
                    ser = S.harness_series(csv, base)
                    sc = S.score(ser, tr)
                    e = S.sample_errors(ser, tr)
                    # the heading at the end: the last 2 s of the log's descent (or of the log)
                    t_end = min(float(ser["t"][-1]), tr.landed)
                    m = (e["t"] > t_end - 2.0) & (e["t"] <= t_end)
                    sc["end_heading"] = float(np.median(np.abs(e["heading"][m]))) if m.any() else None
                    sc["end_roll"] = float(np.median(np.abs(e["roll"][m]))) if m.any() else None
                    sc["guard"] = ser.get("events", {}).get("guard")
                    results.append({"run": run, "cfg": cfg, "mag_err": mag, "imu": imu, "csv": csv, "score": sc})
            print(f"{run} {imu}: done", flush=True)
        json.dump({"results": results}, open(os.path.join(d, "guard.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
