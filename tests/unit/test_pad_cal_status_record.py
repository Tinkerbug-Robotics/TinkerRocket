"""The pad calibration's progress record (PAD_CAL_STATUS_MSG, 0x99, #1600 phase 5).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 40-byte wire layout is pinned on the firmware side by a static_assert and
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
FMT = "<IBBHBBBBHBBHhHHHiiiH"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=61_234_567, flags=0x01 | 0x02 | 0x04 | 0x08) -> bytes:
    return struct.pack(
        FMT, time_us,
        1, 42, 1003,                                # running, 42 %, 1.003 g
        14, 2, 3, 9,                                # tracked, no ephemeris, settling, corrected
        87,                                         # corrections' spread (cm)
        1, flags,                                   # ionosphere: broadcast
        17082, 2,                                   # its file's GPS day, two days old
        1500, 240, 180,                             # pad: fixes, seconds, scatter (cm)
        399_716_265, -749_363_464, 1_234,           # pad lat, lon (deg x 1e7), alt (mm, ellipsoidal)
        215)                                        # filter position sigma (cm)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_PAD_CAL_STATUS == 0x99
    assert pfd.MSG_NAMES[0x99] == "PadCalStatus"
    assert pfd.MSG_EXPECTED_LEN[0x99] == 40
    assert pfd.SIZE_OF_PAD_CAL_STATUS == 40
    # the parser's format reads the firmware layout field for field
    assert struct.unpack(pfd.FMT_PAD_CAL_STATUS, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_pad_cal_status(record())
    assert rec["time_us"] == 61_234_567
    assert rec["cal_state"] == "running" and rec["cal_progress"] == 42
    assert abs(rec["cal_gravity_g"] - 1.003) < 1e-12
    assert (rec["sats_tracked"], rec["sats_no_eph"], rec["sats_settling"], rec["sats_corrected"]) == (14, 2, 3, 9)
    assert abs(rec["corr_rms_m"] - 0.87) < 1e-12
    assert rec["iono_source"] == "broadcast"
    assert rec["iono_gps_day"] == 17082 and rec["iono_age_days"] == 2
    assert (rec["pad_fixes"], rec["pad_secs"]) == (1500, 240)
    assert abs(rec["pad_scatter_m"] - 1.8) < 1e-12
    assert abs(rec["pad_lat_deg"] - 39.9716265) < 1e-9
    assert abs(rec["pad_lon_deg"] + 74.9363464) < 1e-9
    assert abs(rec["pad_alt_m"] - 1.234) < 1e-12
    assert abs(rec["pos_sigma_m"] - 2.15) < 1e-12
    assert rec["raw_gnss"] and rec["filter_running"] and rec["iono_age_known"] and rec["pad_valid"]
    assert not rec["pad_frozen"]


def test_an_unknown_age_reads_as_none():
    rec = pfd.decode_pad_cal_status(record(flags=0x01))
    assert rec["iono_age_days"] is None and not rec["pad_valid"]


def test_wrong_size_is_not_decoded():
    assert pfd.decode_pad_cal_status(b"\x00" * 39) is None
    assert pfd.decode_pad_cal_status(b"\x00" * 41) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x99, record(time_us=1_000_000 + 1_000_000 * k)) for k in range(4)]
    frames.append(frame(0x99, b"\x00" * 20))     # a wrong-length one is skipped
    p = tmp_path / "flight_pad_cal.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["PadCalStatus"]
    assert [r["time_us"] for r in recs] == [1_000_000 + 1_000_000 * k for k in range(4)]
    assert stats["type_counts"]["PadCalStatus"] >= 4
