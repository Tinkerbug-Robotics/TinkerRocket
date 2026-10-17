#!/usr/bin/env python3
"""One harness run on a log, the calibrated pad or the rig IMU as recorded, any extra harness flags:

    one.py RUN IMU MAG OUT_PREFIX [harness flags ...]      (IMU: cal | uncal)

Writes OUT_PREFIX.csv, OUT_PREFIX.out (stdout, the DIAG lines) and OUT_PREFIX.err (stderr).
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import calibrated as CB  # noqa: E402
import scorelib as S  # noqa: E402


def command(run, imu, mag, csv, flags):
    ev = os.path.join(HERE, "in", run + ".txt")
    base = float(S.read_meta(ev)["t_base_s"])
    rec = CB.RECORDED[run]
    cmd = [os.environ.get("ONE_BINARY") or os.path.join(HERE, "build", "raw_replay"), "--in", ev, "--out", csv, "--t-base-s", repr(base),
           "--every", "10"] + list(flags) + ["--mag-error-deg", str(mag), "--recorded-mag-error-deg", str(rec)]
    if imu == "cal":
        ba, bw = CB.csv_bias()
        f_pad, _w, m_pad, pnav = CB.pad_means(ev, base)
        q_pad = CB.qmul(CB.qz(mag), CB.triad(f_pad - ba, m_pad, pnav[22] - math.radians(rec)))
        cmd += ["--acc-sub", ",".join(repr(float(x)) for x in ba), "--gyro-sub", ",".join(repr(float(x)) for x in bw),
                "--pad-wb-sub", ",".join(repr(float(x)) for x in bw),
                "--pad-quat", ",".join(repr(float(x)) for x in q_pad), "--pad-ab", "0,0,0"]
    return cmd, base


if __name__ == "__main__":
    run, imu, mag, pre = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
    cmd, base = command(run, imu, mag, pre + ".csv", sys.argv[5:])
    p = subprocess.run(cmd, capture_output=True, text=True)
    open(pre + ".out", "w").write(p.stdout)
    open(pre + ".err", "w").write(p.stderr)
    if p.returncode != 0:
        raise SystemExit(p.stderr[-2000:])
    print("t_base_s", base)
