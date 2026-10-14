"""The raw-GNSS filter's attitude 1-sigma and IMU biases (RAW_GNSS_ATT_MSG, 0x9B).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 46-byte wire layout is pinned on the firmware side by a static_assert and
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
FMT = "<I3H3f3f3H3H"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=91_234_567) -> bytes:
    return struct.pack(FMT, time_us, 35, 41, 1_250,        # sigma N, E, D: 0.35, 0.41, 12.5 deg
                       0.125, -0.25, 0.5,                    # accelerometer bias, m/s^2
                       0.0625, -0.03125, 0.25,               # gyro bias, deg/s
                       80, 90, 100,                          # their sigma, mm/s^2
                       15, 20, 25)                           #   and milli-deg/s


def test_registered_with_the_firmware_size():
    assert pfd.MSG_RAW_GNSS_ATT == 0x9B
    assert pfd.MSG_NAMES[0x9B] == "RawGnssAtt"
    assert pfd.MSG_EXPECTED_LEN[0x9B] == 46
    assert pfd.SIZE_OF_RAW_GNSS_ATT == 46
    assert struct.unpack(pfd.FMT_RAW_GNSS_ATT, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_raw_gnss_att(record())
    assert rec["time_us"] == 91_234_567
    assert abs(rec["att_sigma_n_deg"] - 0.35) < 1e-12
    assert abs(rec["att_sigma_e_deg"] - 0.41) < 1e-12
    assert abs(rec["heading_sigma_deg"] - 12.5) < 1e-12
    assert (rec["acc_bias_x"], rec["acc_bias_y"], rec["acc_bias_z"]) == (0.125, -0.25, 0.5)
    assert (rec["gyro_bias_x_dps"], rec["gyro_bias_y_dps"], rec["gyro_bias_z_dps"]) == (0.0625, -0.03125, 0.25)
    assert abs(rec["acc_bias_sigma_z"] - 0.1) < 1e-12
    assert abs(rec["gyro_bias_sigma_z_dps"] - 0.025) < 1e-12


def test_wrong_size_is_not_decoded():
    assert pfd.decode_raw_gnss_att(b"\x00" * 45) is None
    assert pfd.decode_raw_gnss_att(b"\x00" * 47) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9B, record(time_us=1_000_000 + 100_000 * k)) for k in range(6)]
    frames.append(frame(0x9B, b"\x00" * 12))     # a wrong-length one is skipped
    p = tmp_path / "flight_raw_gnss_att.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["RawGnssAtt"]
    assert [r["time_us"] for r in recs] == [1_000_000 + 100_000 * k for k in range(6)]
    assert stats["type_counts"]["RawGnssAtt"] >= 6
