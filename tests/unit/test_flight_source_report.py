"""#1658: the report says which rocket and computer a flight came from.

Two sources, merged into one Source group beside the log's Board: the out
computer's own record in the log (OC_IDENTITY_MSG, step 3), which wins for
what it knows, and the `source` block the apps write into the summary sidecar
at download, which adds the rocket profile, the flight computer as the app saw
it, and when it was downloaded.
"""

from __future__ import annotations

import json
import shutil
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402
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


def oc_identity_frame(name: bytes = b"Beetle", unit: bytes = b"9e8a1b2c",
                      board_code: int = 0x21, fw: bytes = b"41b2ab92-m1+20261022-1126") -> bytes:
    """An OC_IDENTITY_MSG frame as the out computer writes it."""
    payload = struct.pack(pfd.FMT_OC_IDENTITY, 1_100_000, 1, 7, 2, board_code, unit, name, fw)
    body = bytes([pfd.MSG_OC_IDENTITY, len(payload)]) + payload
    crc = pfd.crc16(body)
    return b"\xAA\x55\xAA\x55" + body + bytes([crc >> 8, crc & 0xFF])


def flight_with(tmp_path: Path, sidecar: dict | None, log_identity: bytes | None = None) -> Flight:
    bin_path = tmp_path / BIN.name
    shutil.copy(BIN, bin_path)
    if log_identity is not None:
        with open(bin_path, "ab") as f:
            f.write(log_identity)
    if sidecar is not None:
        bin_path.with_suffix(".json").write_text(json.dumps(sidecar))
    return Flight.from_bin(bin_path)


def group(result, name):
    return next((g for g in result.groups if g["name"] == name), None)


def test_source_prints_beside_the_board(tmp_path):
    result = settings.analyze_system(flight_with(tmp_path, {"max_altitude_m": 100.0, "source": SOURCE}))
    g = group(result, "Source")
    assert g is not None
    rows = dict(g["rows"])
    assert rows["Rocket profile"] == "RIM66"
    assert rows["Computer"] == "Beetle (unit 9e8a1b2c)"
    assert rows["Out computer board"] == "M1"
    assert rows["Flight computer board (as provisioned)"] == "M1"
    assert rows["GNSS receiver"] == "LC86G"
    assert rows["Downloaded"] == "2026-10-24T18:03:00Z"
    assert rows["Recorded by"] == "the app at download"
    # right after the log's own Firmware group, where Board is
    names = [g["name"] for g in result.groups]
    if "Firmware" in names:
        assert names.index("Source") == names.index("Firmware") + 1


def test_the_logs_own_record_wins_for_what_it_knows(tmp_path):
    # The unit was renamed after the flight: the log keeps the name it flew with.
    src = dict(SOURCE, unit_name="Beetle (renamed)", oc_fw="ffffffff+later")
    rows = dict(group(settings.analyze_system(
        flight_with(tmp_path, {"source": src}, oc_identity_frame())), "Source")["rows"])
    assert rows["Computer"] == "Beetle (unit 9e8a1b2c)"
    assert rows["Out computer firmware"] == "41b2ab92-m1+20261022-1126"
    assert rows["Network ID"] == 7
    assert rows["Rocket profile"] == "RIM66"          # only the app knows it
    assert rows["Recorded by"] == "the log, and the app at download"


def test_a_log_with_no_sidecar_still_names_its_computer(tmp_path):
    full_name = b"Twenty-char unitname"                 # all 20 bytes, no NUL
    rows = dict(group(settings.analyze_system(
        flight_with(tmp_path, None, oc_identity_frame(name=full_name, board_code=0x99))),
        "Source")["rows"])
    assert rows["Computer"] == "Twenty-char unitname (unit 9e8a1b2c)"
    assert rows["Out computer board"] == "V9 — asserted by the image, not provisioned"
    assert rows["Recorded by"] == "the log"
    assert "Rocket profile" not in rows


def test_unknown_fields_drop_out(tmp_path):
    rows = dict(group(settings.analyze_system(flight_with(tmp_path, {"source": {"unit_id": "9e8a1b2c"}})),
                      "Source")["rows"])
    assert rows == {"Computer": "unnamed (unit 9e8a1b2c)", "Recorded by": "the app at download"}


def test_no_source_no_group(tmp_path):
    assert group(settings.analyze_system(flight_with(tmp_path, {"max_altitude_m": 100.0})),
                 "Source") is None
    assert group(settings.analyze_system(flight_with(tmp_path, {"source": "Beetle"})),
                 "Source") is None


def test_a_phone_cached_name_keeps_its_date(tmp_path):
    # The apps cache flights as "<board name>__<unit id>"; a file shared from
    # the phone carries it, and the report still reads the date (and pairs the
    # sidecar by the same stem).
    bin_path = tmp_path / "flight_20261024_161431__a1cae150.bin"
    shutil.copy(BIN, bin_path)
    bin_path.with_suffix(".json").write_text(json.dumps({"source": SOURCE}))
    flight = Flight.from_bin(bin_path)
    assert flight.date is not None and flight.date.strftime("%Y%m%d_%H%M%S") == "20261024_161431"
    assert group(settings.analyze_system(flight), "Source") is not None
