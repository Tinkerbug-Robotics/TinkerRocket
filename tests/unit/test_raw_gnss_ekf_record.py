"""The raw-GNSS shadow filter's record (RAW_GNSS_EKF_MSG, 0x96, #1600 phase 4).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 90-byte wire layout is pinned on the firmware side by a static_assert and
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
FMT = "<IiiifffffffffffHHHHHHIhBBBBBBHHH"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=812_345_678, flags=0x01 | 0x02 | 0x04 | 0x40) -> bytes:
    return struct.pack(
        FMT, time_us,
        399_716_265, -749_363_464, 34_210,          # lat, lon (deg x 1e7), alt (mm)
        1.5, -0.25, -120.0,                         # vel N, E, D (m/s)
        0.70710678, 0.0, 0.70710678, 0.0,           # attitude
        12.5, -0.2, 8.25, -21.75,                   # clock bias, drift, Galileo, BeiDou offsets
        150, 160, 420,                              # position sigma, cm
        30, 31, 45,                                 # velocity sigma, mm/s
        573_795_000, 184,                           # epoch time of week (ms), age (ms)
        12, 11, 0, 1,                               # used / refused
        15, flags,                                  # ephemerides, flags
        21_000, 3_400, 950)                         # build, update, IMU step (us)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_RAW_GNSS_EKF == 0x96
    assert pfd.MSG_NAMES[0x96] == "RawGnssEkf"
    assert pfd.MSG_EXPECTED_LEN[0x96] == 90
    assert pfd.SIZE_OF_RAW_GNSS_EKF == 90
    # the parser's format reads the firmware layout field for field
    assert struct.unpack(pfd.FMT_RAW_GNSS_EKF, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_raw_gnss_ekf(record())
    assert rec["time_us"] == 812_345_678
    assert abs(rec["lat_deg"] - 39.9716265) < 1e-9
    assert abs(rec["lon_deg"] + 74.9363464) < 1e-9
    assert abs(rec["alt_m"] - 34.21) < 1e-9
    assert (rec["vel_n"], rec["vel_e"], rec["vel_d"]) == (1.5, -0.25, -120.0)
    assert abs(rec["q0"] - 0.70710678) < 1e-6 and rec["q1"] == 0.0
    assert rec["clk_bias_m"] == 12.5
    assert abs(rec["clk_drift_mps"] + 0.2) < 1e-6
    assert (rec["isb_gal_m"], rec["isb_bds_m"]) == (8.25, -21.75)
    assert (rec["pos_sigma_n"], rec["pos_sigma_e"], rec["pos_sigma_d"]) == (1.5, 1.6, 4.2)
    assert abs(rec["vel_sigma_d"] - 0.045) < 1e-12
    assert rec["epoch_tow_s"] == 573_795.0
    assert abs(rec["epoch_age_s"] - 0.184) < 1e-12
    assert (rec["used_pr"], rec["used_rr"], rec["rejected_pr"], rec["rejected_rr"]) == (12, 11, 0, 1)
    assert rec["eph_sats"] == 15
    assert rec["running"] and rec["clock_set"] and rec["timed"] and rec["in_flight"]
    assert not (rec["clock_step"] or rec["stale_epoch"] or rec["levelling"] or rec["pad_resync"])
    assert (rec["build_us"], rec["update_us"], rec["prop_us"]) == (21_000, 3_400, 950)


def test_wrong_size_is_not_decoded():
    assert pfd.decode_raw_gnss_ekf(b"\x00" * 89) is None
    assert pfd.decode_raw_gnss_ekf(b"\x00" * 91) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x96, record(time_us=1_000_000 + 100_000 * k)) for k in range(5)]
    frames.append(frame(0x96, b"\x00" * 60))     # a wrong-length one is skipped
    p = tmp_path / "flight_raw_gnss_ekf.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["RawGnssEkf"]
    assert [r["time_us"] for r in recs] == [1_000_000 + 100_000 * k for k in range(5)]
    assert stats["type_counts"]["RawGnssEkf"] >= 5
