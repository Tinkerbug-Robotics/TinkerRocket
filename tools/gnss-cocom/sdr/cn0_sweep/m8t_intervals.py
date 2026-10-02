#!/usr/bin/env python3
"""(1) Sky C/N0 medians per system from sky_m8t.json / sky_px.json rows (el >= 10 and el > 60; C/N0 only, no
positions); (2) per NEO-M8T capture, after T-10: the stretches with no valid raw pseudorange (gaps > 1 s) and the fix
runs (NAV-PVT gnssFixOK, fixType >= 2); (3) the scenarios' purpose/limits text.   m8t_intervals.py"""
import json
import os
import statistics as st
import struct
from pathlib import Path

S = Path(__file__).resolve().parent / "data"
SDR = Path(__file__).resolve().parents[1]
IGN_TOW = 204000.0
for fn in ("sky_m8t.json", "sky_px.json"):         # roof-log summaries: not committed (antenna geometry)
    if not (S / fn).exists():
        continue
    d = json.load(open(S / fn))
    rows = d["rows"]
    print(f"##### {fn} ({d['capture']}, {d['slice_s']} s slices, {len(rows)} rows)")
    for sysn in sorted({r["system"] for r in rows}):
        rs = [r for r in rows if r["system"] == sysn and r["cn0"]]
        bands = sorted({r["band"] for r in rs})
        for b in bands:
            rb = [r for r in rs if r["band"] == b]
            e10 = [r["cn0"] for r in rb if r["el"] is not None and r["el"] >= 10]
            e60 = [r["cn0"] for r in rb if r["el"] is not None and r["el"] > 60]
            print(f"  {sysn:8s} {b:3s}: n {len(rb):3d}  el>=10 median {st.median(e10) if e10 else float('nan'):.0f} "
                  f"(n {len(e10)})  el>60 median {st.median(e60) if e60 else float('nan'):.0f} (n {len(e60)})"
                  f"  svs {len({r['sv'] for r in rb})}")
for scen in ("traveler_soft25", "hotshot"):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    pro = sc.get("prologue_s", 180.0)
    print(f"##### {scen}: purpose {str(sc.get('purpose'))[:300]}")
    print(f"  scenario {str(sc.get('scenario'))[:200]}")
    print(f"  limits {json.dumps(sc.get('limits'))[:300]}")
    print(f"  blocked {[[round(a - pro, 1), round(b - pro, 1)] for a, b in sc.get('blocked_windows', [])]}"
          f"  80km {[[round(a - pro, 1), round(b - pro, 1)] for a, b in sc.get('altitude_80km_windows', [])]}")
C = Path(os.environ.get("CAPTURES", str(SDR / "captures")))
for tag, stem in [("mhs12", "signalsim_hotshot_all_2026_57_w_p180"), ("mhsn6", "signalsim_hotshot_all_2026_45_w_p180"),
                  ("mtr12", "signalsim_traveler_all_2026_57_w_p180"), ("mtr6", "signalsim_traveler_all_2026_51_w_p180"),
                  ("mtr0", "signalsim_traveler_all_2026_45_w_p180_cofs"),
                  ("mtrn6b", "signalsim_traveler_all_2026_45_w_p180_cofs")]:
    raw, pvt = [], []
    for line in open(C / f"neo_m8t_wide{tag}_{stem}.log", errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "U":
            continue
        h = p[2].strip()
        try:
            b = bytes.fromhex(h[4:])
        except ValueError:
            continue
        if h.startswith("0215") and len(b) >= 16:
            tow, n = struct.unpack("<d", b[0:8])[0], b[11]
            nv = sum(1 for j in range(n) if len(b) >= 48 + 32 * j and b[16 + 32 * j + 30] & 1)
            raw.append((tow - IGN_TOW, nv))
        elif h.startswith("0107") and len(b) >= 64:
            pvt.append((struct.unpack("<I", b[0:4])[0] / 1000.0 - IGN_TOW, b[20] >= 2 and b[21] & 1))
    ts = [t for t, nv in raw if nv > 0 and t >= -10]
    gaps = [(a, b) for a, b in zip(ts, ts[1:]) if b - a > 1.0]
    runs, cur = [], None
    for t, ok in pvt:
        if t < -10:
            continue
        if ok and cur is None:
            cur = [t, t]
        elif ok:
            cur[1] = t
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur:
        runs.append(cur)
    print(f"##### {tag}: raw T{ts[0]:+.1f}..T{ts[-1]:+.1f}; no-raw gaps > 1 s: "
          + ", ".join(f"T{a:+.1f}..T{b:+.1f}" for a, b in gaps)
          + "; fix runs: " + ", ".join(f"T{a:+.1f}..T{b:+.1f}" for a, b in runs if b - a >= 0.5))
