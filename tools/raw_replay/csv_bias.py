"""The replayed IMU's errors in the truth CSV: mean(f - f_true) and mean(w - w_true) per axis (body FRD), on the pad
and through the flight, and whether the high-g channel was in use (acc_hg)."""
import gzip
import numpy as np
import scorelib as S  # noqa: E402
P = S.TRUTH_CSV
lines = [ln for ln in gzip.open(P, "rt") if not ln.startswith("#")]
names = lines[0].strip().split(",")
d = np.loadtxt(lines[1:], delimiter=",")
c = {n: d[:, i] for i, n in enumerate(names)}
t, ph = c["t_s"], c["phase"].astype(int)
f = np.stack([c["fx_mps2"], c["fy_mps2"], c["fz_mps2"]], 1)
ft = np.stack([c["fx_true_mps2"], c["fy_true_mps2"], c["fz_true_mps2"]], 1)
w = np.stack([c["wx_rps"], c["wy_rps"], c["wz_rps"]], 1)
wt = np.stack([c["wx_true_rps"], c["wy_true_rps"], c["wz_true_rps"]], 1)
print("t range", t[0], t[-1], "rate", 1 / np.median(np.diff(t)))
for name, m in (("pad (t<0, phase 0)", (ph == 0) & (t < 0)), ("pad last 60 s", (ph == 0) & (t > -60) & (t < 0)),
                ("pad last 5 s", (ph == 0) & (t > -5) & (t < 0)), ("boost", ph == 2), ("coast", ph == 3),
                ("drogue", ph == 4), ("main", ph == 5)):
    if not m.any():
        continue
    df = (f - ft)[m].mean(0)
    dw = (w - wt)[m].mean(0)
    print(f"{name:20s} n {m.sum():6d}  accel bias {df.round(4)} m/s^2 = {np.round(df / 9.80665 * 1000, 2)} mg  |  "
          f"gyro bias {np.round(np.degrees(dw), 4)} deg/s  | acc_hg used {c['acc_hg'][m].mean():.2f}")
# the bias drift: per 10 s on the pad
m = (ph == 0) & (t < 0)
for t0 in np.arange(np.floor(t[m][0] / 30) * 30, 0, 30):
    mm = m & (t >= t0) & (t < t0 + 30)
    if mm.sum() > 100:
        print(f"  pad {t0:6.0f}..{t0+30:6.0f}: accel {np.round((f - ft)[mm].mean(0) / 9.80665 * 1000, 2)} mg, gyro "
              f"{np.round(np.degrees((w - wt)[mm].mean(0)), 4)} deg/s")
