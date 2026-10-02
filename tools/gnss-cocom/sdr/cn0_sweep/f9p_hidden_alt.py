#!/usr/bin/env python3
"""While the ZED-F9P's fix is off (fixType 0), does its NAV-PVT keep a moving solution? Samples NAV-PVT every 10 s
from T+0 to T+300: fixType, gnssFixOK, numSV, hMSL and speed, against the truth's altitude, per capture.
    f9p_hidden_alt.py CAPTURE [CAPTURE ...]"""
import json
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
sc = json.loads((SDR / "scenarios" / "traveler_soft25.json").read_text())
PRO = sc["prologue_s"]
TR = {round(s["t"] - PRO, 1): s["alt_m"] for s in sc["truth"]}
for cap in sys.argv[1:]:
    print("##", Path(cap).name.split("_signalsim")[0])
    seen = set()
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "U" or not p[2].startswith("0107"):
            continue
        try:
            b = bytes.fromhex(p[2].strip()[4:])
        except ValueError:
            continue
        if len(b) < 92:
            continue
        t = round(struct.unpack_from("<I", b, 0)[0] / 1000.0 - 204000.0, 1)
        k = int(t // 10)
        if t < 0 or t > 300 or k in seen or abs(t - 10 * k) > 0.05:
            continue
        seen.add(k)
        vn, ve, vd = struct.unpack_from("<iii", b, 48)
        print(f"  T+{t:5.1f}  fixType {b[20]} ok {b[21] & 1}  numSV {b[23]:2d}  hMSL "
              f"{struct.unpack_from('<i', b, 36)[0] / 1e6:7.2f} km  speed {(vn * vn + ve * ve + vd * vd) ** 0.5 / 1000:5.0f} m/s"
              f"   truth {TR.get(t, float('nan')) / 1000:7.2f} km")
