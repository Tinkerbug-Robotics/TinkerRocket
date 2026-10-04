#!/usr/bin/env python3
"""The report's hotshot (the short boost: 10 -> 40 g over 4 s, 0.25 s soft start) in the wide all-three format of the
traveler sweep: a 180 s pad and the flight to T+120 s, 300 s = 11.1 GB at 18.48 Msps. Ignition at 2026-08-18
08:40:00 GPST (TOW 204000) like every other bench file, so the file starts at 08:37:00 GPST (08:36:42 UTC); the
motion is make_wide_p180.py's conversion of scenarios/hotshot_pad600.csv (pad Const + 0.1 s VerticalAcc segments).
GPS L1CA + GAL E1 + BDS B1I, wide 18.48 Msps at 1568.286 MHz, uniform power; carrier NOT corrected (fly with
hackrf_tx_ram -C -23 / TX_CARRIER_HZ=-23).

    make_wide_hotshot.py 45 51 57      -> hotshot_all_2026_<L>_w_p180.json (output c8/signalsim_<same>.C8)
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
SDR = HERE.parent                    # tools/gnss-cocom/sdr
PAD_CUT = 420.0                      # s taken off the 600 s pad
END = 720.0                          # pad600 file time the flight is cut at (T+120)
UTC = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=36, second=42)
WIDE = dict(sampleFreq=18.48, centerFreq=1568.286)
SIGNAL = {"GPS": "L1CA", "Galileo": "E1", "BDS": "B1I"}

rows = [[float(v) for v in line.split(",")] for line in (SDR / "scenarios" / "hotshot_pad600.csv").read_text().split()]
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
print(f"hotshot: {len(rows)} rows dt {dt}, lift-off at pad600 {t[lift - 1]} (file {segs[0]['time']} s), "
      f"{len(segs)} segments, duration {dur:.1f} s ({dur * 18.48e6 * 2 / 1e9:.2f} GB), max alt err {err:.3f} m, "
      f"ends T+{t[lift - 1] + dt * (len(segs) - 1) - 600:.1f}, top speed in file {max(u[:len(segs) + lift]):.0f} m/s")
for lv in sys.argv[1:] or ["45"]:
    name = f"hotshot_all_2026_{lv}_w_p180"
    cfg = {
        "version": 1.0,
        "description": f"TinkerRocket bench: the report's hotshot (short boost), 180 s pad, flight to T+120; ignition "
                       f"2026-08-18 08:40:00 GPST, GPS L1CA + GAL E1 + BDS B1I, wide, {lv} dB-Hz",
        "time": UTC,
        "trajectory": {
            "name": name,
            "initPosition": {"type": "LLA", "format": "d", "longitude": -119.0, "latitude": 0.0, "altitude": 1200},
            "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
            "trajectoryList": segs,
        },
        "ephemeris": {"type": "RINEX", "name": "EphData/BRDC_2026230_MN.rnx"},
        "output": {
            "type": "IFdata", "format": "IQ8", **WIDE, "name": f"c8/signalsim_{name}.C8",
            "config": {"elevationMask": 5},
            "systemSelect": [{"system": s, "signal": SIGNAL[s], "enable": True} for s in ("GPS", "Galileo", "BDS")],
        },
        "power": {"noiseFloor": -172, "initPower": {"unit": "dBHz", "value": int(lv)}, "elevationAdjust": False},
    }
    text = json.dumps(cfg, indent="\t") + "\n"
    assert max(len(line) for line in text.splitlines()) < 250
    (CONFIGS / f"{name}.json").write_text(text)
    print(f"{name}.json -> {cfg['output']['name']}")
