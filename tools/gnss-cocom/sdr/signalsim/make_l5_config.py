#!/usr/bin/env python3
"""Owner 10-03: "construct a signalsim file for the l5 portion of the px1105r's band and run that". The PX1105R's
second band: GPS L5 + Galileo E5a + BeiDou B2a, all on 1176.45 MHz (its SAW diplexer passes L1 and L5). First file:
the static test that tells whether the PX1105R tracks L5 with no L1 present (on the sky its L5 channels looked slaved
to their L1 channels, and one HackRF sends one band at a time). Same date, place and ephemeris as the 2026 L1 files
(make_2026_configs.py); 18.48 Msps like the wide L1 files, which keeps the transmitter at the rate it streams now
and passes +-9.24 MHz of L5's +-10.23 MHz main lobe.

  static_l5_2026_45.json   static 240 s, GPS L5 + GAL E5a + BDS B2a at 1176.45 MHz, 18.48 Msps, 45 dB-Hz

Indented JSON: SignalSim reads it line by line into a 255-byte buffer."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
UTC = dict(type="UTC", year=2026, month=8, day=18, hour=8, minute=29, second=42)
L5 = dict(sampleFreq=18.48, centerFreq=1176.45)
SIGNAL = {"GPS": "L5", "Galileo": "E5a", "BDS": "B2a"}


def config(name, desc, segments, band, level=45, elev=False, mask=5):
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
            "systemSelect": [{"system": s, "signal": SIGNAL[s], "enable": True} for s in ("GPS", "Galileo", "BDS")],
        },
        "power": {"noiseFloor": -172, "initPower": {"unit": "dBHz", "value": level}, "elevationAdjust": elev},
    }


name = "static_l5_2026_45"
c = config(name, "TinkerRocket bench: static rig origin, 2026-08-18 08:30 GPST, GPS L5 + GAL E5a + BDS B2a at "
                 "1176.45 MHz, 18.48 Msps, 240 s, 45 dB-Hz", [{"type": "Const", "time": 240.0}], L5)
text = json.dumps(c, indent="\t") + "\n"
assert max(len(line) for line in text.splitlines()) < 250
(CONFIGS / f"{name}.json").write_text(text)
print(f"{name}.json written")
