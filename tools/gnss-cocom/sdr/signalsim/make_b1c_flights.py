#!/usr/bin/env python3
"""The hotshot and the traveler with every L1 signal the stage-0 software receiver tracks: GPS L1CA + GAL E1 + BDS
B1C, narrow 8.184 Msps at 1575.42 MHz, 45 dB-Hz uniform (2026-10-04). The wide sweep files carry BeiDou on B1I,
outside the receiver's band; these put B1C in its place. Everything else is the wide 180 s-pad files': the same
trajectory segments (make_wide_hotshot.py, make_wide_p180.py), ignition at 2026-08-18 08:40:00 GPST, the file
starting at 08:37:00 GPST. Raw SignalSim output, no HackRF carrier correction: the receiver reads the files
directly.

    make_b1c_flights.py   -> configs/{hotshot,traveler}_gpsgalb1c_2026_45_n_p180.json
                             (output c8/signalsim_<same>.C8: 4.9 and 8.8 GB)
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
NARROW = dict(sampleFreq=8.184, centerFreq=1575.42)
SIGNAL = {"GPS": "L1CA", "Galileo": "E1", "BDS": "B1C"}
FLIGHTS = {"hotshot": ("hotshot_all_2026_45_w_p180", "the report's hotshot (short boost), 180 s pad, flight to T+120"),
           "traveler": ("traveler_all_2026_45_w_p180", "the report's traveler (soft25), 180 s pad, flight to T+360")}

for flight, (wide, what) in FLIGHTS.items():
    with open(CONFIGS / f"{wide}.json") as f:
        c = json.load(f)
    name = f"{flight}_gpsgalb1c_2026_45_n_p180"
    c["description"] = (f"TinkerRocket bench: {what}; ignition 2026-08-18 08:40:00 GPST, GPS L1CA + GAL E1 + BDS B1C, "
                        "narrow, 45 dB-Hz (for the stage-0 software receiver)")
    c["trajectory"]["name"] = name
    out = c["output"]
    out.update(NARROW)
    out["name"] = f"c8/signalsim_{name}.C8"
    out["systemSelect"] = [{"system": s, "signal": SIGNAL[s], "enable": True} for s in ("GPS", "Galileo", "BDS")]
    segs = c["trajectory"]["trajectoryList"]
    dur = sum(s["time"] for s in segs)
    text = json.dumps(c, indent="\t") + "\n"
    assert max(len(line) for line in text.splitlines()) < 250
    (CONFIGS / f"{name}.json").write_text(text)
    print(f"{name}.json -> {out['name']}: {len(segs)} segments, {dur:.1f} s ({dur * 8.184e6 * 2 / 1e9:.2f} GB)")
