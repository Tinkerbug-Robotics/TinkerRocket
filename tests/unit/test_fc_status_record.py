"""The flight computer's status record (FC_STATUS_MSG, 0x92), v1 and v2.

v2 (#1409) appends scap_mv: the hold-up supercap voltage the Tinker-Mantis
V10/V11 flight computer reads on its GPIO17, valid with FCS_SCAP_SENSE (0x04),
0xFFFF when its ADC did not answer. No log in the repo carries the record, so
the frames here are synthesised with the framing the out computer writes:
[AA 55 AA 55][type][len][payload][CRC16 BE]. The 7-byte layout and the 5-byte
v1 prefix are pinned on the firmware side by static_asserts in
RocketComputerTypes.h (and test_rocket_computer_types); this pins the Python
reading of both lengths.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
CAMERA, VOTES, SENSE = 0x01, 0x02, 0x04


def frame(payload: bytes) -> bytes:
    body = bytes([0x92, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def v1(time_us: int, flags: int) -> bytes:
    return struct.pack("<IB", time_us, flags)


def v2(time_us: int, flags: int, scap_mv: int) -> bytes:
    return struct.pack("<IBH", time_us, flags, scap_mv)


def test_registered_with_both_firmware_sizes():
    assert pfd.MSG_FC_STATUS == 0x92
    assert pfd.MSG_EXPECTED_LEN[0x92] == (5, 7)
    assert pfd.FCS_SCAP_SENSE == SENSE
    assert pfd.FCS_SCAP_NO_READING == 0xFFFF


def test_v1_and_v2_in_one_log(tmp_path):
    p = tmp_path / "flight_fcs.bin"
    p.write_bytes(
        frame(v1(1_000_000, CAMERA))              # an FC image from before #1409
        + frame(v2(1_200_000, SENSE, 2487))       # a V10 with its cap charged
        + frame(v2(1_400_000, SENSE | VOTES, 0))  # an empty cap: a reading of 0 V
        + frame(v2(1_600_000, SENSE, 0xFFFF))     # a V10 whose ADC did not answer
        + frame(v2(1_800_000, CAMERA, 0xFFFF))    # any other board: no sense line
        + frame(v2(2_000_000, 0, 2500))           # a stray field without the flag
        + frame(b"\x00" * 6)                      # neither length: skipped
        + b"\x00" * 16)
    records, _stats, _config = pfd.parse_binary_file(str(p))
    rows = records["FcStatus"]
    assert [r["time_us"] for r in rows] == [1_000_000, 1_200_000, 1_400_000,
                                            1_600_000, 1_800_000, 2_000_000]

    old = rows[0]
    assert old["fc_camera_engaged"] is True and old["fc_raw_votes_withheld"] is False
    assert old["scap_v"] is None          # v1 never carried it

    assert rows[1]["scap_v"] == 2.487
    assert rows[1]["fc_camera_engaged"] is False
    assert rows[2]["scap_v"] == 0.0 and rows[2]["fc_raw_votes_withheld"] is True
    assert rows[3]["scap_v"] is None      # the sense, but no reading
    assert rows[4]["scap_v"] is None and rows[4]["fc_camera_engaged"] is True
    assert rows[5]["scap_v"] is None      # the flag gates the field
