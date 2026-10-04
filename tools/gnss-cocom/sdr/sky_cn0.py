#!/usr/bin/env python3
"""What C/N0 a receiver reads on the real sky, per satellite and against elevation: where real
flights sit on the boost-vs-signal ladder (cn0_boost_report.html).

SkyTraq captures (px1105r_sky_log.py): C/N0 from 0xE5 raw; GPS L1 C/A elevations from the
receiver's own broadcast ephemeris (0xE0) and its median fixed position (0xDF), used in memory
only and never printed or written. u-blox captures (run_radiated.py --listen-only after
ubx_config.py --raw): C/N0 and elevation per satellite from UBX-NAV-SAT, which needs no position.

    ./sky_cn0.py CAPTURE [--json OUT.json] [--png OUT.png]

The JSON holds per-satellite 2-minute slices (system, band, satellite, elevation, median C/N0):
no position, no time of day.
"""
import json
import math
import statistics as st
import struct
import sys
from pathlib import Path

import numpy as np

SDR = Path(__file__).resolve().parent
ROOT = SDR.parents[2]
STQ_GNSS = {0: "GPS", 1: "SBAS", 2: "GLONASS", 3: "Galileo", 4: "QZSS", 5: "BeiDou", 6: "NavIC"}
STQ_BAND = {0: "L1", 1: "L1", 2: "L2", 3: "L2", 4: "L5", 5: "L5", 6: "other", 7: "other"}
UBX_GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS", 6: "GLONASS"}
SLICE = 120.0

args = sys.argv[1:]
cap = args[0]
opt = {k: args[args.index(k) + 1] for k in ("--json", "--png") if k in args}


def lines(path):
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) == 3 and p[1] in ("B", "U"):
            try:
                yield p[1], bytes.fromhex(p[2].strip())
            except ValueError:
                pass


# (system, band, sv) -> [(t, cn0)] and, for u-blox, [(t, elevation)]
meas, elev_obs, pos = {}, {}, []
kind = None
for tag, x in lines(cap):
    if tag == "B" and x[0] == 0xE5 and len(x) >= 14:
        kind = "skytraq"
        tow = struct.unpack(">I", x[5:9])[0] / 1000.0
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and r[3]:
                meas.setdefault((STQ_GNSS.get(r[0] & 0x0F, "?"), STQ_BAND.get(r[0] >> 4, "?"), r[1]),
                                []).append((tow, r[3]))
    elif tag == "B" and x[0] == 0xDF and len(x) >= 37 and x[2] >= 2:
        pos.append(struct.unpack(">ddd", x[13:37]))
    elif tag == "U" and x[:2] == b"\x01\x35" and len(x) >= 10:           # UBX-NAV-SAT
        kind = "ublox"
        p = x[2:]
        tow = struct.unpack_from("<I", p, 0)[0] / 1000.0
        for j in range(p[5]):
            b = p[8 + 12 * j: 8 + 12 * (j + 1)]
            if len(b) < 12:
                continue
            gnss, sv, cno, el = b[0], b[1], b[2], struct.unpack_from("<b", b, 3)[0]
            if cno and -90 <= el <= 90:
                key = (UBX_GNSS.get(gnss, "?"), "L1", sv)
                meas.setdefault(key, []).append((tow, cno))
                elev_obs.setdefault(key, []).append((tow, el))

if not meas:
    sys.exit("no C/N0 measurements in the capture")

elevation = None
if kind == "ublox":
    def elevation(key, t):
        e = [v for tt, v in elev_obs.get(key, []) if abs(tt - t) <= SLICE / 2]
        return float(st.median(e)) if e else None
elif pos:
    sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "src"))
    sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "scripts"))
    from tc_ekf_cocom import load_capture                               # noqa: E402
    from tinkerrocket_sim.estimation.gnss_raw import sat_pos             # noqa: E402
    rx = np.median(np.array(pos), axis=0)                                # in memory only
    up = rx / np.linalg.norm(rx)
    eph = load_capture(cap)[0]

    def elevation(key, t):
        if key[0] != "GPS":
            return None
        e = eph.pick("G", key[2], t)
        if not e:
            return None
        rs, _ = sat_pos(e, t - 0.075)
        u = (rs - rx) / np.linalg.norm(rs - rx)
        return math.degrees(math.asin(float(u @ up)))

rows = []
for key, v in sorted(meas.items()):
    v.sort()
    t0 = v[0][0]
    while t0 <= v[-1][0]:
        sl = [c for t, c in v if t0 <= t < t0 + SLICE]
        if len(sl) >= 10:
            el = elevation(key, t0 + SLICE / 2) if elevation else None
            rows.append(dict(system=key[0], band=key[1], sv=key[2], el=el, cn0=float(st.median(sl))))
        t0 += SLICE
t_all = [t for v in meas.values() for t, _ in v]
print(f"{Path(cap).name} ({kind}): {(max(t_all) - min(t_all)) / 60:.1f} min; elevations "
      + ("from NAV-SAT" if kind == "ublox" else "for GPS from its ephemeris (position not shown)"))

for system in ("GPS",):
    rr = [r for r in rows if r["system"] == system and r["band"] == "L1"]
    print(f"\n{system} L1, per satellite (median over the capture):")
    for sv in sorted({r["sv"] for r in rr}):
        s = [r for r in rr if r["sv"] == sv]
        els = [r["el"] for r in s if r["el"] is not None]
        print(f"  {system[0]}{sv:02d}  el {st.median(els):5.1f}  C/N0 {st.median([r['cn0'] for r in s]):4.1f}"
              if els else f"  {system[0]}{sv:02d}  el    ?  C/N0 {st.median([r['cn0'] for r in s]):4.1f}")
    for lo, hi in ((0, 15), (15, 30), (30, 60), (60, 91)):
        band = [r["cn0"] for r in rr if r["el"] is not None and lo <= r["el"] < hi]
        if band:
            q = np.percentile(band, [10, 50, 90])
            print(f"  elevation {lo:2d}-{min(hi, 90):2d}: median {q[1]:.1f}, 10-90% {q[0]:.1f}-{q[2]:.1f} "
                  f"({len(band)} satellite-slices)")

print("\nevery system and band (each satellite's median, then across satellites):")
groups = {}
for (g, b, sv), v in meas.items():
    groups.setdefault((g, b), []).append(st.median([c for _, c in v]))
for (g, b), meds in sorted(groups.items()):
    q = np.percentile(meds, [10, 50, 90])
    print(f"  {g:8s} {b:5s} {len(meds):2d} sats: median {q[1]:.1f}, 10-90% {q[0]:.1f}-{q[2]:.1f}")

if "--json" in opt:
    Path(opt["--json"]).write_text(json.dumps(dict(capture=Path(cap).name, kind=kind, slice_s=SLICE,
                                                   rows=rows), indent=0))
    print("wrote", opt["--json"])
if "--png" in opt:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    pts = [(r["el"], r["cn0"]) for r in rows if r["system"] == "GPS" and r["band"] == "L1" and r["el"] is not None]
    ax.scatter([a for a, _ in pts], [c for _, c in pts], s=18, color="#1f6feb", alpha=0.7,
               label="GPS L1 C/A, one satellite over 2 min")
    ax.set_xlabel("elevation, deg")
    ax.set_ylabel("C/N0 the receiver reports, dB-Hz")
    ax.set_xlim(0, 90)
    ax.grid(color="#d8dee4", lw=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title(f"{Path(cap).stem}: C/N0 against elevation on the real sky", loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(opt["--png"], dpi=130)
    print("wrote", opt["--png"])
