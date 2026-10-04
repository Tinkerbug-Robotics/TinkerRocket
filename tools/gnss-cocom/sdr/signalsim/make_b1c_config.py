#!/usr/bin/env python3
"""static_gpsgal_2026_45_n with BeiDou B1C added and 300 s long, for the stage-0 software receiver (2026-10-01):
GPS L1CA + GAL E1 + BDS B1C, narrow 8.184 Msps at 1575.42 MHz, 45 dB-Hz. Raw SignalSim output, no HackRF carrier
correction: the receiver reads the file directly instead of over the air.

  static_gpsgalb1c_2026_45_n.json -> c8/signalsim_static_gpsgalb1c_2026_45_n.C8

Written as the file was first made (json.dump, tab indent, no final newline), so the config stays byte-identical."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
with open(CONFIGS / "static_gpsgal_2026_45_n.json") as f:
    c = json.load(f)
c["description"] = ("TinkerRocket bench: static rig origin, 2026-08-18 08:30 GPST, GPS L1CA + GAL E1 + BDS B1C, "
                    "narrow, 300 s, 45 dB-Hz (for the stage-0 software receiver)")
c["trajectory"]["name"] = "static_gpsgalb1c_2026_45_n"
c["trajectory"]["trajectoryList"] = [{"type": "Const", "time": 300.0}]
c["output"]["name"] = "c8/signalsim_static_gpsgalb1c_2026_45_n.C8"
for s in c["output"]["systemSelect"]:
    if s["system"] == "BDS":
        s["signal"], s["enable"] = "B1C", True
with open(CONFIGS / "static_gpsgalb1c_2026_45_n.json", "w") as f:
    json.dump(c, f, indent="\t")
print("static_gpsgalb1c_2026_45_n.json written")
