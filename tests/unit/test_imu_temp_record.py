"""The IMU's die temperature (IMU_TEMP_MSG, 0x9C).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 6-byte wire layout is pinned on the firmware side by a static_assert and
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
FMT = "<Ih"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=91_234_567, raw=2_560) -> bytes:
    return struct.pack(FMT, time_us, raw)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_IMU_TEMP == 0x9C
    assert pfd.MSG_NAMES[0x9C] == "ImuTemp"
    assert pfd.MSG_EXPECTED_LEN[0x9C] == 6
    assert pfd.SIZE_OF_IMU_TEMP == 6
    assert struct.unpack(pfd.FMT_IMU_TEMP, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_imu_temp(record())
    assert rec["time_us"] == 91_234_567
    assert rec["temp_raw"] == 2_560
    assert rec["temp_c"] == 35.0                          # 25 + 2560 / 256
    assert pfd.decode_imu_temp(record(raw=-1_280))["temp_c"] == 20.0
    assert pfd.decode_imu_temp(record(raw=0))["temp_c"] == 25.0


def test_wrong_size_is_not_decoded():
    assert pfd.decode_imu_temp(b"\x00" * 5) is None
    assert pfd.decode_imu_temp(b"\x00" * 7) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9C, record(time_us=1_000_000 * (k + 1), raw=256 * k)) for k in range(6)]
    frames.append(frame(0x9C, b"\x00" * 4))      # a wrong-length one is skipped
    p = tmp_path / "flight_imu_temp.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["ImuTemp"]
    assert [r["time_us"] for r in recs] == [1_000_000 * (k + 1) for k in range(6)]
    assert [r["temp_c"] for r in recs] == [25.0 + k for k in range(6)]
    assert stats["type_counts"]["ImuTemp"] >= 6
