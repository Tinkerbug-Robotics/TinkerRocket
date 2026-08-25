#!/usr/bin/env python3
"""Package the flight_report source tree + sample flight for the web tool.

Produces:
  payload/flight_report_src.zip  - source bundle unpacked into Pyodide's MEMFS
  payload/sample/                - sample flight files + manifest.json

Re-run after changing anything under Data_Analysis/. The zip must preserve the
on-disk layout (top-level sibling modules beside the flight_report/ package)
because the package resolves them via __file__-relative sys.path insertion.
"""

from __future__ import annotations

import json
import zipfile
from pathlib import Path

WEBTOOL = Path(__file__).resolve().parent
DATA_ANALYSIS = WEBTOOL.parent
REPO = DATA_ANALYSIS.parent

# Top-level modules imported by flight_report/ via sys.path manipulation.
TOP_LEVEL_MODULES = [
    "plot_flight_data_mini.py",
    # #850: flight_report/modules/lora_link.py imports this to read a
    # base-station log in either format. Without it in the bundle the import
    # fails inside Pyodide, the per-file except swallows it, and the Radio Link
    # section silently reports no data on the hosted tool while working locally.
    "bs_log.py",
    "analyze_gaps.py",
    "replay_kinematic_checks.py",
    "sim_kinematic_checks.py",
]

# The sample flight. Preferred source is the full field capture — .bin + .json +
# lora_*.csv — which exercises the LoRa module too; the in-repo example is the
# fallback so a fresh clone still builds. The multi-megabyte .csv sidecar is
# skipped deliberately: nothing in flight_report reads it.
SAMPLE_STEM = "flight_20260705_174532"
SAMPLE_ROCKET_NAME = "Rolly Polly 54mm"
SAMPLE_SOURCES = [
    Path.home() / "Documents/Hobbies/ModelRockets/TestFlights/2026_07_05 CENJARS/Rolly Polly 54mm",
    REPO / "examples" / "flights",
]
SAMPLE_PATTERNS = [SAMPLE_STEM + ".bin", SAMPLE_STEM + ".json",
                   "lora_*.bin", "lora_*.csv"]


def main() -> None:
    payload = WEBTOOL / "payload"
    payload.mkdir(exist_ok=True)

    zip_path = payload / "flight_report_src.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for name in TOP_LEVEL_MODULES:
            zf.write(DATA_ANALYSIS / name, name)
        pkg = DATA_ANALYSIS / "flight_report"
        for f in sorted(pkg.rglob("*")):
            if f.is_file() and "__pycache__" not in f.parts:
                zf.write(f, f.relative_to(DATA_ANALYSIS))
    print(f"{zip_path.relative_to(WEBTOOL)}: {zip_path.stat().st_size:,} bytes")

    src = next((d for d in SAMPLE_SOURCES if (d / f"{SAMPLE_STEM}.bin").exists()), None)
    if src is None:
        tried = "\n  ".join(str(d) for d in SAMPLE_SOURCES)
        raise SystemExit(f"no sample flight {SAMPLE_STEM}.bin found in:\n  {tried}")
    print(f"sample source: {src}")

    sample_dir = payload / "sample"
    sample_dir.mkdir(exist_ok=True)
    for stale in sample_dir.glob("*"):
        stale.unlink()

    files = sorted({f for pat in SAMPLE_PATTERNS for f in src.glob(pat)})
    manifest = {"rocket_name": SAMPLE_ROCKET_NAME, "files": []}
    for f in files:
        (sample_dir / f.name).write_bytes(f.read_bytes())
        manifest["files"].append(f.name)
        print(f"payload/sample/{f.name}: {f.stat().st_size:,} bytes")
    (sample_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
