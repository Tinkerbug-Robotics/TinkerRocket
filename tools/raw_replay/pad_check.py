"""On the log's pad (before ignition): the replayed IMU ticks against the truth (accel and gyro bias, body FRD), and
the magnetometer's heading with the TRUE attitude against the declination the flight computer used."""
import math
import sys
import numpy as np
sys.path.insert(0, ".")
import scorelib as S
tr = S.Truth()
import gzip
lines = [ln for ln in gzip.open(S.TRUTH_CSV, "rt") if not ln.startswith("#")]
names = lines[0].strip().split(",")
d = np.loadtxt(lines[1:], delimiter=",")
c = {n: d[:, i] for i, n in enumerate(names)}
for run, rec_err in (("r21", 0.0), ("r20", 40.0), ("f3c", 10.0)):
    ev = f"in/{run}.txt"
    meta = S.read_meta(ev)
    base = float(meta["t_base_s"])
    I, M, pnav = [], [], None
    for ln in open(ev):
        if ln.startswith("I "):
            v = ln.split(); I.append([float(x) for x in v[1:8]])
        elif ln.startswith("M "):
            v = ln.split(); M.append([float(x) for x in v[1:5]])
        elif ln.startswith("PNAV "):
            pnav = [float(x) for x in ln.split()[1:]]
    I = np.array(I); M = np.array(M)
    ti = I[:, 0] * 1e-6 + base
    m = (ti > -1.1) & (ti < -0.05)
    ft = np.stack([np.interp(ti[m], c["t_s"], c[k]) for k in ("fx_true_mps2", "fy_true_mps2", "fz_true_mps2")], 1)
    wt = np.stack([np.interp(ti[m], c["t_s"], c[k]) for k in ("wx_true_rps", "wy_true_rps", "wz_true_rps")], 1)
    fb = (I[m, 1:4] - ft).mean(0); wb = (I[m, 4:7] - wt).mean(0)
    # magnetometer with the true attitude
    tm = M[:, 0] * 1e-6 + base
    mm = (tm > -1.1) & (tm < -0.05)
    _, _, qt = tr.at(tm[mm])
    w_, x_, y_, z_ = qt.T
    R = np.stack([np.stack([1-2*(y_*y_+z_*z_), 2*(x_*y_-w_*z_), 2*(x_*z_+w_*y_)], -1),
                  np.stack([2*(x_*y_+w_*z_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-w_*x_)], -1),
                  np.stack([2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)], -1)], 1)   # body -> NED
    mn = np.einsum("nij,nj->ni", R, M[mm, 1:4]).mean(0)
    mag_az = math.degrees(math.atan2(mn[1], mn[0]))     # magnetic north's azimuth east of true north... field points
    decl_fc = math.degrees(pnav[22]) - rec_err
    print(f"{run}: pad ticks n {m.sum()} accel bias {np.round(fb / 9.80665 * 1000, 2)} mg, gyro bias "
          f"{np.round(np.degrees(wb), 4)} deg/s | field NED {mn.round(2)} uT: horizontal azimuth {mag_az:+.2f} deg, "
          f"FC declination (bench removed) {decl_fc:+.2f} deg -> magnetometer heading error with the true tilt "
          f"{decl_fc - mag_az:+.2f} deg")
