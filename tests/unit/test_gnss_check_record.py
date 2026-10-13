"""The physics check of each receiver fix against the IMU (GNSS_CHECK_MSG, 0x9A, #1508).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 24-byte wire layout is pinned on the firmware side by a static_assert and
the host gtest's offsets (tests_cpp/test_rocket_computer_types.cpp); this pins
the Python reading of it and that the parser files it as a stream.
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
FMT = "<IBBHffff"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=91_234_567, verdict=2) -> bytes:
    return struct.pack(FMT, time_us, verdict, 3, 1_250, 312.5, 9.75, 4.5, 1.25)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_GNSS_CHECK == 0x9A
    assert pfd.MSG_NAMES[0x9A] == "GnssCheck"
    assert pfd.MSG_EXPECTED_LEN[0x9A] == 24
    assert pfd.SIZE_OF_GNSS_CHECK == 24
    assert struct.unpack(pfd.FMT_GNSS_CHECK, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_gnss_check(record())
    assert rec["time_us"] == 91_234_567
    assert rec["verdict"] == "flagged"
    assert rec["sats_jumped"] == 3
    assert abs(rec["span_s"] - 1.25) < 1e-12
    assert (rec["pos_err_m"], rec["pos_tol_m"]) == (312.5, 9.75)
    assert (rec["vel_err_mps"], rec["vel_tol_mps"]) == (4.5, 1.25)
    assert pfd.decode_gnss_check(record(verdict=1))["verdict"] == "pass"
    assert pfd.decode_gnss_check(record(verdict=0))["verdict"] == "unjudged"


def test_wrong_size_is_not_decoded():
    assert pfd.decode_gnss_check(b"\x00" * 23) is None
    assert pfd.decode_gnss_check(b"\x00" * 25) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9A, record(time_us=1_000_000 + 100_000 * k)) for k in range(6)]
    frames.append(frame(0x9A, b"\x00" * 12))     # a wrong-length one is skipped
    p = tmp_path / "flight_gnss_check.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["GnssCheck"]
    assert [r["time_us"] for r in recs] == [1_000_000 + 100_000 * k for k in range(6)]
    assert stats["type_counts"]["GnssCheck"] >= 6
