#!/usr/bin/env python3
"""The no-GNSS controls (r16, r19, r26: the receiver had lost the sky before launch) with the firmware's levelling
and with the experiment's heading-free in-flight levelling (build_exp): does act end worse than unaided without any
GNSS, and does the levelling change fix it?

    controls_level.py OUT_DIR OUT.md
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dropout_sweep as DS  # noqa: E402
import one  # noqa: E402
import scorelib as S  # noqa: E402

LOGS = ("r16", "r19", "r26")
CFGS = ("H30_none", "H10_none", "A30_none", "C30_none", "U_ref")
BIN = {"firmware levelling": os.path.join(HERE, "build", "raw_replay"),
       "heading-free levelling (experiment)": os.path.join(HERE, "build_exp", "raw_replay")}


def main():
    out_dir, out_md = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    tr = S.Truth()
    cf = DS.configs()
    res = []
    for bname, binary in BIN.items():
        os.environ["ONE_BINARY"] = binary
        for imu in ("cal", "uncal"):
            for run in LOGS:
                for c in CFGS:
                    for m in DS.MAGS:
                        csv = os.path.join(out_dir, f"{run}_{c}_m{int(m)}_{imu}_{'exp' if 'exp' in bname else 'fw'}.csv")
                        cmd, base = one.command(run, imu, m, csv, cf[c])
                        cmd[0] = binary
                        p = subprocess.run(cmd, capture_output=True, text=True)
                        if p.returncode != 0:
                            raise SystemExit(p.stderr[-2000:])
                        ser = S.harness_series(csv, base)
                        e = S.sample_errors(ser, tr)
                        s = DS.summarize(e, tr, float(ser["t"][-1]))
                        res.append({"bin": bname, "imu": imu, "run": run, "cfg": c, "mag": m, "sum": s})
    json.dump(res, open(os.path.join(out_dir, "controls.json"), "w"))
    L = ["# No GNSS in flight: act against unaided, with the firmware's levelling and with heading-free levelling\n",
         "Logs r16, r19, r26 (the receiver had lost the sky before launch; r16 and r19 start from a borrowed pad "
         "context). Each cell: the median over the three logs of the per-log median, [the worst log], deg. "
         "End: the last 2 s before landing.\n",
         "| levelling | pad IMU | mag error | configuration | roll, coast | roll, descent | nose, descent | heading at the end |",
         "|---|---|---|---|---|---|---|---|"]
    for bname in BIN:
        for imu in ("cal", "uncal"):
            for m in DS.MAGS:
                for c in CFGS:
                    rs = [r for r in res if r["bin"] == bname and r["imu"] == imu and r["mag"] == m and r["cfg"] == c]

                    def agg(ph, k):
                        v = [r["sum"][ph][k][0] if ph != "end" else r["sum"][ph][k] for r in rs if ph in r["sum"]]
                        return f"{np.median(v):.1f} [{max(v):.1f}]" if v else "-"
                    L.append(f"| {bname} | {'calibrated, 0 mg' if imu == 'cal' else 'uncalibrated'} | {m:+.0f} | "
                             f"{'unaided' if c == 'U_ref' else c.split('_')[0]} | {agg('coast', 'roll')} | "
                             f"{agg('descent', 'roll')} | {agg('descent', 'nose')} | {agg('end', 'heading')} |")
    open(out_md, "w").write("\n".join(L) + "\n")
    print("wrote", out_md)


if __name__ == "__main__":
    main()
