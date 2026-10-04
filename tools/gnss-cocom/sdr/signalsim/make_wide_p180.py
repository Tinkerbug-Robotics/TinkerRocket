#!/usr/bin/env python3
"""The wide all-three traveler, short enough to sit in the page cache (the 1256 s original is 46 GB and underran
from disk): a 180 s pad and the flight to T+360 s, 540 s = 20.0 GB at 18.48 Msps. Ignition stays at 2026-08-18
08:40:00 GPST (TOW 204000), so the file starts at 08:37:00 GPST (08:36:42 UTC) and the sky at ignition is the
earlier runs' sky; the motion is make_2026_configs.py's (the same pad600 CSV, the pad cut by 420 s). Everything
else as traveler_all_2026_45_w: GPS L1CA + GAL E1 + BDS B1I, wide 18.48 Msps at 1568.286 MHz, 45 dB-Hz uniform.

Writes traveler_all_2026_45_w_p180.json (output c8/signalsim_traveler_all_2026_45_w_p180.C8).
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
SDR = HERE.parent                    # tools/gnss-cocom/sdr
NAME = "traveler_all_2026_45_w_p180"
PAD_CUT = 420.0                      # s taken off the 600 s pad
END = 960.0                          # pad600 file time the flight is cut at (T+360)
UTC = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=36, second=42)
WIDE = dict(sampleFreq=18.48, centerFreq=1568.286)
SIGNAL = {"GPS": "L1CA", "Galileo": "E1", "BDS": "B1I"}

rows = [[float(v) for v in line.split(",")] for line in (SDR / "scenarios" / "traveler_soft25_pad600.csv").read_text().split()]
t = [r[0] for r in rows]
h = [r[3] for r in rows]
assert all(abs(r[1] - rows[0][1]) < 1e-9 and abs(r[2] - rows[0][2]) < 1e-9 for r in rows), "not vertical"
dt = round(t[1] - t[0], 6)
v = [0.0] + [(h[k] - h[k - 1]) / dt for k in range(1, len(h))]
lift = next(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)
last = max(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)
u = [0.0] * len(v)
for k in range(lift, last):
    u[k] = 0.5 * (v[k] + v[k + 1])
u[last] = v[last] if last == len(v) - 1 else 0.0
segs = [{"type": "Const", "time": round(t[lift - 1] - PAD_CUT, 6)}]
alt, err = h[lift - 1], 0.0
for k in range(lift, last + 1):
    if t[k] > END + 1e-6:
        break
    segs.append({"type": "VerticalAcc", "time": dt, "speed": round(u[k], 5)})
    alt += 0.5 * (u[k - 1] + u[k]) * dt
    err = max(err, abs(alt - h[k]))
dur = segs[0]["time"] + dt * (len(segs) - 1)
cfg = {
    "version": 1.0,
    "description": "TinkerRocket bench: the report's traveler (soft25), pad cut to 180 s and flight to T+360 so the "
                   "file caches; ignition 2026-08-18 08:40:00 GPST, GPS L1CA + GAL E1 + BDS B1I, wide, 45 dB-Hz",
    "time": UTC,
    "trajectory": {
        "name": NAME,
        "initPosition": {"type": "LLA", "format": "d", "longitude": -119.0, "latitude": 0.0, "altitude": 1200},
        "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
        "trajectoryList": segs,
    },
    "ephemeris": {"type": "RINEX", "name": "EphData/BRDC_2026230_MN.rnx"},
    "output": {
        "type": "IFdata", "format": "IQ8", **WIDE, "name": f"c8/signalsim_{NAME}.C8",
        "config": {"elevationMask": 5},
        "systemSelect": [{"system": s, "signal": SIGNAL[s], "enable": True} for s in ("GPS", "Galileo", "BDS")],
    },
    "power": {"noiseFloor": -172, "initPower": {"unit": "dBHz", "value": 45}, "elevationAdjust": False},
}
text = json.dumps(cfg, indent="\t") + "\n"
assert max(len(line) for line in text.splitlines()) < 250
(CONFIGS / f"{NAME}.json").write_text(text)
print(f"{NAME}.json: {len(segs)} segments, pad {segs[0]['time']} s, duration {dur:.1f} s "
      f"({dur * 18.48e6 * 2 / 1e9:.2f} GB), lift-off at file time {segs[0]['time']} (= pad600 {t[lift - 1]}), "
      f"max alt err {err:.3f} m, ends at pad600 {t[lift - 1] + dt * (len(segs) - 1):.1f} (T+{t[lift - 1] + dt * (len(segs) - 1) - 600:.1f})")
