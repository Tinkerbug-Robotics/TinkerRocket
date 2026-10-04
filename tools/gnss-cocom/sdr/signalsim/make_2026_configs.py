#!/usr/bin/env python3
"""SignalSim configs on the COCOM rig's own date and place: 2026-08-18 08:30:00 GPST (= 08:29:42 UTC, the
gps-sdr-sim scenarios' -t and one minute after the PX1105R cold start's seeded time), origin 0 N 119 W 1200 m,
ephemeris BRDC_2026230_MN (the rig's day-230 multi-GNSS broadcast file). Written:

  static_gpsgal_2026_45_n.json   static 240 s, GPS L1CA + GAL E1, narrow 8.184 Msps at 1575.42 MHz
  static_all_2026_45_w.json      static 240 s, + BDS B1I, wide 18.48 Msps at 1568.286 MHz
  traveler_gpsgal_2026_45_n.json the report's traveler (scenarios/traveler_soft25_pad600.csv, 10 Hz LLA,
                                 lift-off at 600.3 s, 102 km): the 600 s pad as one Const segment, then one
                                 0.1 s VerticalAcc segment per truth row ending at the average of the two
                                 neighbouring block-mean velocities -> continuous velocity (smooth Doppler),
                                 altitude within ~1 m of the truth. Vertical flights only (checked).

Indented JSON: SignalSim reads it line by line into a 255-byte buffer."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
SDR = HERE.parent                    # tools/gnss-cocom/sdr
UTC = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=29, second=42)
NARROW = dict(sampleFreq=8.184, centerFreq=1575.42)
WIDE = dict(sampleFreq=18.48, centerFreq=1568.286)
SYSTEMS = {"gpsgal": ("GPS", "Galileo"), "all": ("GPS", "Galileo", "BDS")}
SIGNAL = {"GPS": "L1CA", "Galileo": "E1", "BDS": "B1I"}


def config(name, desc, segments, band, systems, level=45, elev=False, mask=5):
    return {
        "version": 1.0,
        "description": desc,
        "time": UTC,
        "trajectory": {
            "name": name,
            "initPosition": {"type": "LLA", "format": "d", "longitude": -119.0, "latitude": 0.0, "altitude": 1200},
            "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
            "trajectoryList": segments,
        },
        "ephemeris": {"type": "RINEX", "name": "EphData/BRDC_2026230_MN.rnx"},
        "output": {
            "type": "IFdata", "format": "IQ8", **band, "name": f"c8/signalsim_{name}.C8",
            "config": {"elevationMask": mask},
            "systemSelect": [{"system": s, "signal": SIGNAL[s], "enable": s in systems}
                             for s in ("GPS", "Galileo", "BDS")],
        },
        "power": {"noiseFloor": -172, "initPower": {"unit": "dBHz", "value": level}, "elevationAdjust": elev},
    }


def traveler_segments(csv):
    rows = [[float(v) for v in line.split(",")] for line in csv.read_text().split()]
    t = [r[0] for r in rows]
    h = [r[3] for r in rows]
    assert all(abs(r[1] - rows[0][1]) < 1e-9 and abs(r[2] - rows[0][2]) < 1e-9 for r in rows), "not vertical"
    dt = round(t[1] - t[0], 6)
    v = [0.0] + [(h[k] - h[k - 1]) / dt for k in range(1, len(h))]          # v[k]: mean over (t[k-1], t[k]]
    lift = next(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)             # first moving block ends at t[lift]
    last = max(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)              # last moving block
    segs = [{"type": "Const", "time": round(t[lift - 1], 6)}]
    u = [0.0] * len(v)                                                     # u[k]: vertical speed at t[k]
    for k in range(lift, last):
        u[k] = 0.5 * (v[k] + v[k + 1])
    u[last] = v[last] if last == len(v) - 1 else 0.0                     # still descending at the file's end
    alt, err = h[lift - 1], 0.0
    for k in range(lift, last + 1):
        segs.append({"type": "VerticalAcc", "time": dt, "speed": round(u[k], 5)})
        alt += 0.5 * (u[k - 1] + u[k]) * dt
        err = max(err, abs(alt - h[k]))
    tail = round(t[-1] - t[last], 6)
    if tail > 0:
        segs.append({"type": "Const", "time": tail})
    return segs, dict(lift_off=t[lift - 1], landed=t[last], end=t[-1], max_alt_err_m=err, n=len(segs))


out = {
    "static_gpsgal_2026_45_n": config("static_gpsgal_2026_45_n",
                                      "TinkerRocket bench: static rig origin, 2026-08-18 08:30 GPST, GPS L1CA + GAL E1, "
                                      "narrow, 240 s, 45 dB-Hz", [{"type": "Const", "time": 240.0}], NARROW,
                                      SYSTEMS["gpsgal"]),
    "static_all_2026_45_w": config("static_all_2026_45_w",
                                   "TinkerRocket bench: static rig origin, 2026-08-18 08:30 GPST, GPS L1CA + GAL E1 + "
                                   "BDS B1I, wide, 240 s, 45 dB-Hz", [{"type": "Const", "time": 240.0}], WIDE,
                                   SYSTEMS["all"]),
}
# SignalSim's elevation fade is CN0 -= (1 - sqrt(sin(el))) * 25 dB (-7 at 30 deg, -18 at 5 deg): 50 dB-Hz at the
# zenith puts the PX1105R's readings near its real-sky spread (42 overhead, ~29 low; sky_cn0.py, 2026-09-29).
# Mask 0 deg: every satellite the receiver (mask 3) expects is in the file, as on the sky.
out["static_gpsgal_2026_50e_n"] = config("static_gpsgal_2026_50e_n",
                                         "TinkerRocket bench: static rig origin, 2026-08-18 08:30 GPST, GPS L1CA + GAL "
                                         "E1, narrow, 600 s, 50 dB-Hz at zenith, power scaled by elevation, mask 0",
                                         [{"type": "Const", "time": 600.0}], NARROW, SYSTEMS["gpsgal"], level=50,
                                         elev=True, mask=0)
segs, info = traveler_segments(SDR / "scenarios" / "traveler_soft25_pad600.csv")
out["traveler_gpsgal_2026_45_n"] = config("traveler_gpsgal_2026_45_n",
                                          "TinkerRocket bench: the report's traveler (soft25, pad 600 s), 2026-08-18 "
                                          "08:30 GPST, GPS L1CA + GAL E1, narrow, 45 dB-Hz", segs, NARROW,
                                          SYSTEMS["gpsgal"])
out["traveler_gpsgal_2026_50e_n"] = config("traveler_gpsgal_2026_50e_n",
                                           "TinkerRocket bench: the report's traveler (soft25, pad 600 s), 2026-08-18 "
                                           "08:30 GPST, GPS L1CA + GAL E1, narrow, 50 dB-Hz at zenith scaled by "
                                           "elevation, mask 0", segs, NARROW, SYSTEMS["gpsgal"], level=50, elev=True,
                                           mask=0)
out["traveler_all_2026_45_w"] = config("traveler_all_2026_45_w",
                                       "TinkerRocket bench: the report's traveler (soft25, pad 600 s), 2026-08-18 "
                                       "08:30 GPST, GPS L1CA + GAL E1 + BDS B1I, wide, 45 dB-Hz", segs, WIDE,
                                       SYSTEMS["all"])
for name, c in out.items():
    text = json.dumps(c, indent="\t") + "\n"
    assert max(len(line) for line in text.splitlines()) < 250, name
    (CONFIGS / f"{name}.json").write_text(text)
    print(f"{name}.json: {len(c['trajectory']['trajectoryList'])} segments")
print("traveler:", {k: (round(val, 3) if isinstance(val, float) else val) for k, val in info.items()})
