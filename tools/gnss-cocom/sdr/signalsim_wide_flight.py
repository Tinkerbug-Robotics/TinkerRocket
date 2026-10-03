#!/usr/bin/env python3
"""SignalSim configs for a rig flight in the C/N0 sweep's wide all-three format: a 180 s pad
and the flight to T+END, ignition at 2026-08-18 08:40:00 GPST (TOW 204000) like every bench file, so the file starts
at 08:37:00 GPST (08:36:42 UTC) and truth t = receiver TOW - 203820. The motion is the pad600 CSV as a pad Const
segment plus 0.1 s VerticalAcc segments. GPS L1CA + GAL E1 + BDS B1I, 18.48 Msps at 1568.286 MHz, uniform power,
carrier NOT corrected (fly with TX_CARRIER_HZ=-23).

Run it in the SignalSim working folder (IFdataGen, EphData/BRDC_2026230_MN.rnx and c8/ beside it: SignalSim
truncates long paths, so the config's are short and relative); the configs land there. SignalSim itself has no
licence and stays out of this repository. mosaic_82km.sh flies the result.

    signalsim_wide_flight.py NAME END_T LEVEL [LEVEL ...]   e.g. gentle_alt 300 51  -> NAME_all_2026_51_w_p180.json
    (from scenarios/NAME_pad600.csv: ./make_flights.py --lat 0 --lon -119 --only NAME, then ./pad_scenario.py NAME 600)
"""
import json
import sys
from pathlib import Path

SCEN = Path(__file__).resolve().parent / "scenarios"
OUT = Path.cwd()                     # the SignalSim working folder
PAD_CUT = 420.0                      # s taken off the 600 s pad
UTC = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=36, second=42)
WIDE = dict(sampleFreq=18.48, centerFreq=1568.286)
SIGNAL = {"GPS": "L1CA", "Galileo": "E1", "BDS": "B1I"}

flight, end_t, levels = sys.argv[1], float(sys.argv[2]), sys.argv[3:] or ["45"]
END = 600.0 + end_t                  # pad600 file time the flight is cut at
rows = [[float(v) for v in line.split(",")] for line in (SCEN / f"{flight}_pad600.csv").read_text().split()]
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
print(f"{flight}: {len(rows)} rows dt {dt}, lift-off at pad600 {t[lift - 1]} (file {segs[0]['time']} s), "
      f"{len(segs)} segments, duration {dur:.1f} s ({dur * 18.48e6 * 2 / 1e9:.2f} GB), max alt err {err:.3f} m, "
      f"ends T+{t[lift - 1] + dt * (len(segs) - 1) - 600:.1f}, top speed in file {max(u[:len(segs) + lift]):.0f} m/s, "
      f"top altitude {max(h[:len(segs) + lift]) / 1000:.2f} km")
for lv in levels:
    name = f"{flight}_all_2026_{lv}_w_p180"
    cfg = {
        "version": 1.0,
        "description": f"TinkerRocket bench: {flight}, 180 s pad, flight to T+{end_t:.0f}; ignition "
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
    (OUT / f"{name}.json").write_text(text)
    print(f"{name}.json -> {cfg['output']['name']}")
