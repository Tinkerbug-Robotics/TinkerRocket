"""Are the epoch tags consistent with the replayed IMU? Each MSM7 epoch's MCU time from the driver's clock line, on
the flight's time (the ignition edge in the IMU), against its own GPS time less the scenario's T+0 (573786.0 s)."""
import sys
import numpy as np
sys.path.insert(0, ".")
import scorelib as S

IGN_TOW = 573786.0
for run in sys.argv[1:]:
    ev = f"in/{run}.txt"
    base = float(S.read_meta(ev)["t_base_s"])
    lines, frames = [], []
    for ln in open(ev):
        if ln.startswith("K "):
            v = ln.split()
            lines.append((int(v[1]), float(v[2]), int(v[3]), float(v[4]), float(v[5]), float(v[6]), float(v[7])))
        elif ln.startswith("R "):
            v = ln.split()
            fr = bytes.fromhex(v[2])
            p = fr[3:]
            bits = int.from_bytes(p[:8], "big")
            mt = bits >> 52
            if mt in (1077, 1097):
                tow = ((bits >> 10) & ((1 << 30) - 1)) * 1e-3
                frames.append((int(v[1]), tow))
    if not lines:
        print(run, "no clock lines")
        continue
    lt = np.array([l[0] for l in lines])
    week_s = 2433 * 604800.0
    out = []
    for t_arr, tow in frames:
        k = max(np.searchsorted(lt, t_arr, side="right") - 1, 0)
        _, gps0, mcu0, a_s, t_ref, drift, lat = lines[k]
        tt = week_s + tow - gps0
        mcu = mcu0 + round((tt + a_s + drift * (tt - t_ref) - lat) * 1e6)
        out.append((tow - IGN_TOW, mcu * 1e-6 + base, t_arr * 1e-6 + base))
    o = np.array(out)
    d = (o[:, 1] - o[:, 0]) * 1000.0
    a = (o[:, 2] - o[:, 0]) * 1000.0
    for lo, hi in ((-2, 0), (0, 3), (3, 12), (12, 40), (40, 80)):
        m = (o[:, 0] >= lo) & (o[:, 0] < hi)
        if m.any():
            print(f"{run}: GPS T{lo:+d}..{hi:+d} s: tag less GPS time {np.median(d[m]):+7.1f} ms "
                  f"(range {d[m].min():+.1f}..{d[m].max():+.1f}); the MSM frame arrives {np.median(a[m]):+7.1f} ms "
                  f"after its GPS time ({m.sum()} frames)")
