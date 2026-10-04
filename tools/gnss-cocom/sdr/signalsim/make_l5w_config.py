#!/usr/bin/env python3
"""Owner 10-03 "yes, run the warm start test": the L5 half of a warm-start flight. The receiver first gets a fix on
the first 150 s of the hotshot wide L1 file's static pad (hotshot_all_2026_45_w_p180: 2026-08-18 08:36:42 UTC =
08:37:00 GPST, rig origin lon -119 / lat 0 / 1200 m, 45 dB-Hz), the transmitter is then switched to this file, which
starts 160 s after the L1 file (150 s of L1 + a 10 s handover) at the same place, so the receiver's clock, position
and orbits all carry over. Otherwise identical to static_l5_2026_45 (GPS L5 + GAL E5a + BDS B2a, 1176.45 MHz,
18.48 Msps, 45 dB-Hz, 240 s).
  static_l5w_2026_45.json -> c8/signalsim_static_l5w_2026_45.C8"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
c = json.loads((CONFIGS / "static_l5_2026_45.json").read_text())
hot = json.loads((CONFIGS / "hotshot_all_2026_45_w_p180.json").read_text())
assert hot["trajectory"]["initPosition"] == c["trajectory"]["initPosition"], "not the same site"
assert hot["ephemeris"] == c["ephemeris"] and hot["power"] == c["power"], "not the same ephemeris / power"
assert hot["time"] == dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=36, second=42)
name = "static_l5w_2026_45"
c["time"] = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=39, second=22)   # L1 start + 160 s
c["trajectory"]["name"] = name
c["output"]["name"] = f"c8/signalsim_{name}.C8"
c["description"] = ("TinkerRocket bench: warm-start L5 half, rig origin, starts 2026-08-18 08:39:40 GPST = 160 s after "
                    "the hotshot L1 file; GPS L5 + GAL E5a + BDS B2a at 1176.45 MHz, 18.48 Msps, 240 s, 45 dB-Hz")
text = json.dumps(c, indent="\t") + "\n"
assert max(len(line) for line in text.splitlines()) < 250
(CONFIGS / f"{name}.json").write_text(text)
print(f"{name}.json written: start {c['time']}, {c['trajectory']['trajectoryList']}")
