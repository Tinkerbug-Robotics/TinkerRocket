"""The GPS-to-MCU clock line (GNSS_CLOCK_MSG, 0x9D).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 68-byte wire layout is pinned on the firmware side by a static_assert and
the host gtest's offsets (tests_cpp/test_rocket_computer_types.cpp), and the
firmware's own mcuAt against a logged line by tests_cpp/test_gnss_time_sync.cpp;
this pins the Python reading of it, the replay's placement of an epoch on the
line, and that the parser files it as a stream.
"""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
FMT = "<IIIqddddffIHBB"
GPS0 = 2440 * 604800 + 345600          # a whole GPS second (week 2440, Wednesday)
MCU0 = 5_123_456_789                   # esp_timer, us: past 32 bits, as a flight a while after boot is


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=91_234_567, seq=42, flags=0x03, latency_s=0.0) -> bytes:
    return struct.pack(FMT, time_us, seq, GPS0, MCU0,
                       0.0831,         # a_s: the envelope's MCU - GPS, s
                       40.0,           # t_ref_s
                       30e-6,          # drift: 30 ppm
                       latency_s,
                       0.25, 4.5,      # env_rms_ms, late90_ms
                       600, 1, 41, flags)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_GNSS_CLOCK == 0x9D
    assert pfd.MSG_NAMES[0x9D] == "GnssClock"
    assert pfd.MSG_EXPECTED_LEN[0x9D] == 68
    assert pfd.SIZE_OF_GNSS_CLOCK == 68
    assert struct.unpack(pfd.FMT_GNSS_CLOCK, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    rec = pfd.decode_gnss_clock(record())
    assert rec["time_us"] == 91_234_567
    assert rec["seq"] == 42
    assert (rec["gps0_s"], rec["mcu0_us"]) == (GPS0, MCU0)
    assert (rec["a_s"], rec["t_ref_s"], rec["latency_s"]) == (0.0831, 40.0, 0.0)
    assert abs(rec["drift_ppm"] - 30.0) < 1e-9
    assert (rec["samples"], rec["restarts"], rec["bins"]) == (600, 1, 41)
    assert rec["valid"] and rec["started"]
    assert not pfd.decode_gnss_clock(record(flags=0x02))["valid"]


def test_the_line_places_an_epoch_as_mcuat_does():
    rec = pfd.decode_gnss_clock(record(latency_s=0.083))
    for t in (0.0, 1.0, 40.0, 59.9, 61.5):
        line = 0.0831 + 30e-6 * (t - 40.0)
        x = (t + line - 0.083) * 1e6
        want = MCU0 + int(math.copysign(math.floor(abs(x) + 0.5), x))
        assert pfd.gnss_clock_mcu_us(rec, GPS0 + t) == want
    # half a microsecond rounds away from zero, as C's llround (not Python's round)
    half = pfd.decode_gnss_clock(struct.pack(FMT, 1, 1, GPS0, MCU0, 0.5e-6, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 1, 0, 1, 0x03))
    assert pfd.gnss_clock_mcu_us(half, float(GPS0)) == MCU0 + 1
    assert pfd.gnss_clock_mcu_us(pfd.decode_gnss_clock(record(flags=0x02)), GPS0 + 1.0) is None


def test_wrong_size_is_not_decoded():
    assert pfd.decode_gnss_clock(b"\x00" * 67) is None
    assert pfd.decode_gnss_clock(b"\x00" * 69) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9D, record(time_us=1_000_000 * (k + 1), seq=k)) for k in range(6)]
    frames.append(frame(0x9D, b"\x00" * 12))     # a wrong-length one is skipped
    p = tmp_path / "flight_gnss_clock.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["GnssClock"]
    assert [r["seq"] for r in recs] == list(range(6))
    assert stats["type_counts"]["GnssClock"] >= 6
