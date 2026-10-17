#!/usr/bin/env python3
"""Every configuration on every rig log: the harness run, each run scored against the truth, the tables written.

    run_matrix.py OUT_DIR [--binary build/raw_replay] [--logs r20,r21,...]

Configurations (dc37cf07's bench flags, set at run time):
  A       act, all three axes from launch (full gate), launch tilt floor 0.6 deg, 30 deg heading prior, no hand-off,
          carrier observed
  C       the same with the heading-only gate
  unaided observe (TR_HEADING_FIT=1): GNSS reaches the attitude only through the default cos^4(pitch) gate
  A+cf, C+cf  A and C with the carrier range rates fused (the carrier update has no attitude columns yet)
Magnetometer heading error (added to the declination): 0, +10, +40 deg for A, C and unaided; +10 for the carrier
variants. The log's own bench error is taken out first (the pad state turned by the difference).
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

SP = os.path.dirname(HERE)
LOGS = {   # run: (log, recorded heading mode, recorded bench heading error, GNSS in flight)
    "r18": ("replay/hdg_oc/flight_20261014_172119.bin", "act", 40.0, True),
    "r20": ("replay/hdg_oc2/flight_20261014_184223.bin", "act", 40.0, True),
    "r21": ("replay/hdg_oc2/flight_20261014_184958.bin", "observe", 0.0, True),
    "r22": ("replay/mag_oc/flight_20261014_192745.bin", "observe", 10.0, True),
    "r23": ("replay/mag_oc/flight_20261014_193445.bin", "act", 10.0, True),
    "r25": ("replay/mag_oc/flight_20261014_194843.bin", "act", 5.0, True),
    "r16": ("replay/hdg_oc/flight_20261014_145955.bin", "off", 0.0, False),
    "r19": ("replay/hdg_oc/flight_20261014_173016.bin", "observe", 0.0, False),
    "r26": ("replay/mag_oc/flight_20261014_195548.bin", "observe", 5.0, False),
}
BASE_A = ["--heading-mode", "2", "--gate-full", "1", "--tilt-sigma-deg", "0.6", "--pad-sigma-deg", "30",
          "--handoff-deg", "-1", "--carrier", "1"]
BASE_C = ["--heading-mode", "2", "--gate-full", "0", "--tilt-sigma-deg", "0.6", "--pad-sigma-deg", "30",
          "--handoff-deg", "-1", "--carrier", "1"]
BASE_U = ["--heading-mode", "1", "--carrier", "1"]
CONFIGS = {"A": BASE_A, "C": BASE_C, "unaided": BASE_U,
           "A+cf": BASE_A[:-1] + ["2"], "C+cf": BASE_C[:-1] + ["2"]}
MAG = {"A": (0.0, 10.0, 40.0), "C": (0.0, 10.0, 40.0), "unaided": (0.0, 10.0, 40.0), "A+cf": (10.0,),
       "C+cf": (10.0,)}


def run_one(binary, events, out_csv, args, base):
    cmd = [binary, "--in", events, "--out", out_csv, "--t-base-s", repr(base)] + args
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise SystemExit(p.stderr)
    return p.stderr.strip().splitlines()[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--binary", default=os.path.join(HERE, "build", "raw_replay"))
    ap.add_argument("--logs", default=",".join(LOGS))
    ap.add_argument("--events", default=os.path.join(HERE, "in"))
    ap.add_argument("--extra", help="JSON {run: [log, recorded mode, recorded bench heading error, GNSS in flight]}")
    a = ap.parse_args()
    if a.extra:
        LOGS.update({k: tuple(v) for k, v in json.load(open(a.extra)).items()})
    os.makedirs(a.out, exist_ok=True)
    tr = S.Truth()
    results = []
    for run in a.logs.split(","):
        log, mode, rec_err, gnss = LOGS[run]
        ev = os.path.join(a.events, run + ".txt")
        base = float(S.read_meta(ev)["t_base_s"])
        for cfg, args in CONFIGS.items():
            for err in MAG[cfg]:
                tag = f"{run}_{cfg}_m{int(err)}"
                csv = os.path.join(a.out, tag + ".csv")
                summ = run_one(a.binary, ev, csv, args + ["--mag-error-deg", str(err), "--recorded-mag-error-deg",
                                                          str(rec_err)], base)
                ser = S.harness_series(csv, base)
                sc = S.score(ser, tr)
                ex = ser["extra"]
                k = ser["t"] >= 0.0
                sc["handoffs"] = int(ex["hdg_handoffs"][-1]) if len(ex["hdg_handoffs"]) else 0
                sc["cr_used"] = int(ex["cr_used"][-1]) if len(ex["cr_used"]) else 0
                sc["epochs_flight"] = int(ex["epochs"][-1] - ex["epochs"][k][0]) if k.any() else 0
                results.append({"run": run, "cfg": cfg, "mag_err": err, "gnss": gnss, "csv": csv, "summary": summ,
                                "score": sc})
                print(f"{tag:22s} {summ.split(': ', 1)[1][:150]}")
    json.dump({"truth": {"burnout": tr.burnout, "apogee": tr.apogee, "landed": tr.landed,
                         "pitch_vote": tr.pitch_vote}, "results": results},
              open(os.path.join(a.out, "matrix.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
