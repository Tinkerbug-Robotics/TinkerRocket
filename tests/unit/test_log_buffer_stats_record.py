"""The OC's log-ring snapshot (LOG_BUFFER_STATS_MSG, 0xE2), v1 and v2.

v2 (#1659) appends the FC->OC link's receive health and the longest flash
stall the RX ISR saw. Every log in the repo is v1, so the v2 frames here are
synthesised with the framing the out computer writes:
[AA 55 AA 55][type][len][payload][CRC16 BE]. The 52-byte layout and the v1
prefix are pinned on the firmware side by static_asserts in
RocketComputerTypes.h; this pins the Python reading of both lengths.
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
V1 = "<7I"
V2 = "<13I"


def frame(payload: bytes) -> bytes:
    body = bytes([0xE2, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def v1(time_us: int) -> bytes:
    return struct.pack(V1, time_us, 524288, 1200, 393580, 7, 9000, 0)


def v2(time_us: int, flash_stall_us: int) -> bytes:
    return v1(time_us) + struct.pack("<6I", 15872, 0, 2, 412, 1830, flash_stall_us)


def test_registered_with_both_firmware_sizes():
    assert pfd.MSG_LOG_BUFFER_STATS == 0xE2
    assert pfd.MSG_EXPECTED_LEN[0xE2] == (28, 52)
    assert struct.calcsize(pfd.FMT_LOG_BUFFER_STATS) == 28
    assert struct.calcsize(pfd.FMT_LOG_BUFFER_STATS_V2) == 52


def test_v1_and_v2_in_one_log(tmp_path):
    p = tmp_path / "flight_lbs.bin"
    p.write_bytes(frame(v1(1_000_000)) + frame(v2(2_000_000, 43_500))
                  + frame(b"\x00" * 40)          # neither length: skipped
                  + b"\x00" * 16)
    records, _stats, _config = pfd.parse_binary_file(str(p))
    old, new = records["LogBufferStats"]

    assert old["ring_highwater"] == 393580 and old["ring_overruns"] == 7
    # older firmware never measured the link: None, not a clean 0
    for name in pfd.LOG_BUFFER_STATS_V2_FIELDS:
        assert old[name] is None

    assert new["ring_highwater"] == 393580
    assert (new["rx_peak"], new["rx_overflow_bytes"], new["rx_bad_crc"]) == (15872, 0, 2)
    assert (new["rx_resync_bytes"], new["parser_max_us"]) == (412, 1830)
    assert new["flash_stall_max_us"] == 43_500
