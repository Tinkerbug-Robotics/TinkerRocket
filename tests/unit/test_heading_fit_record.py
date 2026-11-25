"""The heading from thrust, each fit (HEADING_FIT_MSG, 0x9F).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The 32-byte wire layout is pinned on the firmware side by a static_assert and
the host gtest's offsets (tests_cpp/test_rocket_computer_types.cpp), and what
the raw-GNSS filter writes into it by tests_cpp/test_raw_gnss_shadow.cpp (a
boosted flight, observed and acted on); this pins the Python reading of it and
that the parser files it as a stream.
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
FMT = "<IhHhhhhHHHBBBBHHH"


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def record(time_us=7_944_000, psi=-3963, sigma=625, filt=2813, turn=-2813, flags=0x07):
    return struct.pack(FMT, time_us, psi, sigma, filt, turn, 62, -8, 1270, 698, 30, 3, 0, 15, flags,
                       850, 410, 110)


def test_registered_with_the_firmware_size():
    assert pfd.MSG_HEADING_FIT == 0x9F
    assert pfd.MSG_NAMES[0x9F] == "HeadingFit"
    assert pfd.MSG_EXPECTED_LEN[0x9F] == 32
    assert pfd.SIZE_OF_HEADING_FIT == 32
    assert struct.unpack(pfd.FMT_HEADING_FIT, record()) == struct.unpack(FMT, record())


def test_decode_synthetic_payload():
    r = pfd.decode_heading_fit(record())
    assert r["time_us"] == 7_944_000
    assert (r["psi_deg"], r["sigma_deg"], r["filt_deg"], r["turn_deg"]) == (-39.63, 6.25, 28.13, -28.13)
    assert r["bias_mps2"] == (0.062, -0.008)
    assert (r["speed_mps"], r["vel_sigma_mps"], r["rms"]) == (12.7, 0.698, 0.3)
    assert (r["n"], r["n_cut"], r["sats"]) == (3, 0, 15)
    assert (r["vel_us"], r["solve_us"], r["seed_ms"]) == (850, 410, 110)
    assert r["act"] and r["solved"] and r["handed_now"] and not r["handed"] and not r["rival"]


def test_no_sigma_yet_is_none():
    r = pfd.decode_heading_fit(record(sigma=0xFFFF, flags=0x00))
    assert r["sigma_deg"] is None
    assert not (r["act"] or r["solved"] or r["handed_now"] or r["handed"] or r["rival"])


def test_a_rival_holds_the_hand_off():
    r = pfd.decode_heading_fit(record(flags=0x13))
    assert r["rival"] and r["act"] and r["solved"] and not r["handed_now"]


def test_wrong_size_is_not_decoded():
    assert pfd.decode_heading_fit(b"\x00" * 31) is None
    assert pfd.decode_heading_fit(b"\x00" * 33) is None


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9F, record(time_us=1_000_000 * (k + 7), flags=0x02 if k < 2 else 0x0A)) for k in range(5)]
    frames.append(frame(0x9F, b"\x00" * 12))     # a wrong-length one is skipped
    p = tmp_path / "flight_heading_fit.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["HeadingFit"]
    assert [r["time_us"] for r in recs] == [1_000_000 * (k + 7) for k in range(5)]
    assert [r["handed"] for r in recs] == [False, False, True, True, True]
    assert stats["type_counts"]["HeadingFit"] >= 5
