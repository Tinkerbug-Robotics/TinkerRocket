#!/usr/bin/env python3
"""The wide 180 s-pad traveler (traveler_all_2026_45_w_p180.json) at other signal levels: the same config with only
initPower, the output name and the description changed.  make_wide_p180_levels.py 51 57 ...
Writes traveler_all_2026_<L>_w_p180.json (output c8/signalsim_traveler_all_2026_<L>_w_p180.C8, carrier NOT corrected:
fly with hackrf_tx_ram -C -23 / TX_CARRIER_HZ=-23)."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIGS = HERE / "configs"
base = json.loads((CONFIGS / "traveler_all_2026_45_w_p180.json").read_text())
for lv in sys.argv[1:]:
    cfg = json.loads(json.dumps(base))
    name = f"traveler_all_2026_{lv}_w_p180"
    cfg["power"]["initPower"]["value"] = int(lv)
    cfg["output"]["name"] = f"c8/signalsim_{name}.C8"
    cfg["trajectory"]["name"] = name
    cfg["description"] = cfg["description"].replace("45 dB-Hz", f"{lv} dB-Hz")
    text = json.dumps(cfg, indent="\t") + "\n"
    assert max(len(line) for line in text.splitlines()) < 250
    (CONFIGS / f"{name}.json").write_text(text)
    print(f"{name}.json: initPower {lv} dB-Hz -> {cfg['output']['name']}")
