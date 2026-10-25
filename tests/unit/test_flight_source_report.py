"""#1658: the report says which rocket and computer a flight was downloaded from.

The apps write a `source` block into the summary sidecar at download time. The
log cannot say which computer it came from, so this is the one row the System
Configuration section takes from the sidecar, printed beside the log's Board.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

from flight_report.flight import Flight  # noqa: E402
from flight_report.modules import settings  # noqa: E402

BIN = REPO_ROOT / "tests" / "test_data" / "flight_20260615_170318.bin"

SOURCE = {
    "unit_id": "9e8a1b2c",
    "unit_name": "Beetle",
    "rocket_profile": "RIM66",
    "rocket_id": 2,
    "fc_board": "M1",
    "oc_board": "M1",
    "fc_fw": "41b2ab92+20261022-1126",
    "oc_fw": "41b2ab92+20261022-1126",
    "fc_gnss": "LC86G",
    "downloaded_at": "2026-10-24T18:03:00Z",
}


def flight_with(tmp_path: Path, sidecar: dict | None) -> Flight:
    bin_path = tmp_path / BIN.name
    shutil.copy(BIN, bin_path)
    if sidecar is not None:
        bin_path.with_suffix(".json").write_text(json.dumps(sidecar))
    return Flight.from_bin(bin_path)


def group(result, name):
    return next((g for g in result.groups if g["name"] == name), None)


def test_source_prints_beside_the_board(tmp_path):
    result = settings.analyze_system(flight_with(tmp_path, {"max_altitude_m": 100.0, "source": SOURCE}))
    g = group(result, "Downloaded from")
    assert g is not None
    rows = dict(g["rows"])
    assert rows["Rocket profile"] == "RIM66"
    assert rows["Computer"] == "Beetle (unit 9e8a1b2c)"
    assert rows["Board (as provisioned)"] == "M1"
    assert rows["GNSS receiver"] == "LC86G"
    assert rows["Downloaded"] == "2026-10-24T18:03:00Z"
    # right after the log's own Firmware group, where Board is
    names = [g["name"] for g in result.groups]
    if "Firmware" in names:
        assert names.index("Downloaded from") == names.index("Firmware") + 1


def test_boards_that_disagree_are_both_shown(tmp_path):
    src = dict(SOURCE, oc_board="V9")
    rows = dict(group(settings.analyze_system(flight_with(tmp_path, {"source": src})),
                      "Downloaded from")["rows"])
    assert rows["Board (as provisioned)"] == "FC M1, OC V9"


def test_unknown_fields_drop_out(tmp_path):
    rows = dict(group(settings.analyze_system(flight_with(tmp_path, {"source": {"unit_id": "9e8a1b2c"}})),
                      "Downloaded from")["rows"])
    assert rows == {"Computer": "unnamed (unit 9e8a1b2c)"}


def test_no_source_no_group(tmp_path):
    assert group(settings.analyze_system(flight_with(tmp_path, {"max_altitude_m": 100.0})),
                 "Downloaded from") is None
    assert group(settings.analyze_system(flight_with(tmp_path, {"source": "Beetle"})),
                 "Downloaded from") is None
