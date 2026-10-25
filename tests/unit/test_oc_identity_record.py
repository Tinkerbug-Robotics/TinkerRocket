"""The out computer's identity record (OC_IDENTITY_MSG, 0x87, #1658).

The OC writes it into the log ~1 Hz while logging, so a log names the computer
that wrote it. The 68-byte layout is pinned by static_asserts and offsets on
the firmware side and generated into tests_cpp/fixtures/wire/logframes/; this
pins the Python reading of that same file and that the parser keeps the last
record in config.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402

WIRE = REPO_ROOT / "tests_cpp" / "fixtures" / "wire"


def test_registered_with_the_firmware_size():
    assert pfd.MSG_OC_IDENTITY == 0x87
    assert pfd.MSG_NAMES[0x87] == "OcIdentity"
    assert pfd.MSG_EXPECTED_LEN[0x87] == 68
    assert pfd.SIZE_OF_OC_IDENTITY == 68


def test_decodes_the_shared_fixture():
    payload = (WIRE / "logframes" / "ocidentity_68.bin").read_bytes()
    expected = json.loads((WIRE / "logframes" / "ocidentity_68.expected.json").read_text())
    assert pfd.decode_oc_identity(payload) == expected
    # the unit name fills all 20 bytes with no NUL, and still reads whole
    assert expected["unit_name"] == "Twenty-char unitname"


def test_wrong_size_is_not_decoded():
    assert pfd.decode_oc_identity(b"\x00" * 67) is None
    assert pfd.decode_oc_identity(b"\x00" * 69) is None


def test_parser_keeps_it_in_config():
    _records, _stats, config = pfd.parse_binary_file(str(WIRE / "csv" / "tiny_flight.bin"))
    assert config["oc_identity"]["unit_id"] == "9e8a1b2c"
    assert config["oc_identity"]["rocket_id"] == 2


def test_an_older_log_has_none(tmp_path):
    p = tmp_path / "flight_old.bin"
    p.write_bytes(b"\x00" * 64)
    _records, _stats, config = pfd.parse_binary_file(str(p))
    assert config["oc_identity"] is None
