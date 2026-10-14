"""The pad, written once after launch (PAD_CONTEXT_MSG, 0x9E).

No flight log carries it yet, so the .bin under test is synthesised with the
framing the out computer writes: [AA 55 AA 55][type][len][payload][CRC16 BE].
The wire layouts are pinned on the firmware side by static_asserts and the
host gtest's offsets (tests_cpp/test_rocket_computer_types.cpp), and what the
raw-GNSS filter writes, record by record, by tests_cpp/test_raw_gnss_shadow.cpp;
this pins the Python reading of each kind, putting a launch's records back
together, and that the parser files them as a stream.
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


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def header(kind: int, part: int = 0, parts: int = 1, time_us: int = 7_000_000) -> bytes:
    return struct.pack("<IBBB", time_us, kind, part, parts)


def cal(time_us=6_000_000) -> bytes:
    return header(1, time_us=time_us) + struct.pack(
        "<hhh fff hhh B f iii", 12, -7, 3, 0.31, -0.12, 0.05, -210, 45, 1302, 1, 9.79, 0, -150000, 70000)


def nav(time_us=7_000_000, flags=0x2F) -> bytes:
    return header(2, time_us=time_us) + struct.pack(
        "<I ddd fff ffff fff fff d ffffff 8b H BB",
        5_123_456,                                   # filter_time_us
        0.5759, -1.8670, 1402.5,                     # lat, lon (rad), h (m)
        0.01, -0.02, 0.03,                           # vel_ned
        0.7, 0.1, -0.7, 0.1,                         # quat
        0.02, -0.01, 0.05,                           # acc bias
        1e-4, -2e-4, 3e-4,                           # gyro bias
        12345.678,                                   # clk_bias_m
        -0.4, 2.5, -1.25, 3.75, 0.153, 48.2,         # drift, ISBs, baro offset, declination, field
        14, 3, -1, -2, 51, 6, -2, -5,                # Klobuchar bytes
        17037, 1, flags)


def cov(part: int, first: int, n: int, parts: int = 4) -> bytes:
    return header(3, part, parts) + struct.pack(f"<HBB{n}f", first, n, 0, *[float(first + i) for i in range(n)])


def survey(entries, part=0, parts=1) -> bytes:
    body = struct.pack("<BBBB ff B", 12, 2, len(entries), 1, 0.85, 3.5, len(entries))
    for sys_id, prn, corr in entries:
        body += struct.pack("<BBf", sys_id, prn, corr)
    return header(4, part, parts) + body


def rtcm_frame(payload: bytes) -> bytes:
    f = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    return f + b"\x01\x02\x03"                       # the decoder leaves the CRC to the replay's parser


def eph(i: int, parts: int) -> bytes:
    return header(5, i, parts) + rtcm_frame(bytes([0x3F, 0xB0 + i]) + b"\x5a" * 59)


def all_cov():
    return [cov(p, p * 48, 48 if p < 3 else 46) for p in range(4)]


def launch(time_us=7_000_000):
    return [cal(time_us - 1_000_000), nav(time_us)] + all_cov() + \
        [survey([(0, 5, 1.5), (1, 12, -0.75)])] + [eph(i, 3) for i in range(3)]


def test_registered_as_variable_length():
    assert pfd.MSG_PAD_CONTEXT == 0x9E
    assert pfd.MSG_NAMES[0x9E] == "PadContext"
    assert pfd.MSG_EXPECTED_LEN[0x9E] is None
    assert (pfd.PAD_CONTEXT_HDR_LEN, pfd.SIZE_OF_PCX_CAL, pfd.SIZE_OF_PCX_NAV) == (7, 41, 124)
    assert pfd.PCX_COV_VALUES == 190


def test_decode_each_kind():
    c = pfd.decode_pad_context(cal())
    assert (c["kind"], c["part"], c["parts"], c["time_us"]) == ("cal", 0, 1, 6_000_000)
    assert c["gyro_offset_lsb"] == (12, -7, 3)
    assert c["mag_offset_lsb"] == (-210, 45, 1302) and c["mag_offset_set"]
    assert abs(c["cal_gravity_mps2"] - 9.79) < 1e-6
    assert c["mmc_offset_counts"] == (0, -150000, 70000)
    n = pfd.decode_pad_context(nav())
    assert n["kind"] == "nav" and n["filter_time_us"] == 5_123_456
    assert (n["lat_rad"], n["lon_rad"], n["height_m"]) == (0.5759, -1.8670, 1402.5)
    assert n["clk_bias_m"] == 12345.678
    assert abs(n["field_ut"] - 48.2) < 1e-5
    assert n["iono"] == (14, 3, -1, -2, 51, 6, -2, -5) and n["iono_gps_day"] == 17037
    assert n["iono_source"] == "broadcast"
    assert n["running"] and n["clock_set"] and n["has_baro"] and n["has_iono"] and n["primary"]
    assert not n["has_declination"]
    v = pfd.decode_pad_context(cov(3, 144, 46))
    assert (v["part"], v["parts"], v["first"], len(v["values"])) == (3, 4, 144, 46)
    s = pfd.decode_pad_context(survey([(0, 5, 1.5), (2, 33, -2.0)]))
    assert s["corrections"] == [(0, 5, 1.5), (2, 33, -2.0)]
    assert s["pad_ok"] and s["tracked"] == 12
    assert pfd.decode_pad_context(survey([]))["corrections"] == []
    e = pfd.decode_pad_context(eph(2, 15))
    assert (e["part"], e["parts"], len(e["frame"])) == (2, 15, 67)
    assert e["frame"][0] == 0xD3


def test_malformed_is_not_decoded():
    assert pfd.decode_pad_context(b"\x00" * 6) is None
    assert pfd.decode_pad_context(header(9)) is None                  # no such kind
    assert pfd.decode_pad_context(header(2, 1, 1) + nav()[7:]) is None  # part past parts
    assert pfd.decode_pad_context(header(2, 0, 0) + nav()[7:]) is None
    assert pfd.decode_pad_context(cal()[:-1]) is None
    assert pfd.decode_pad_context(nav() + b"\x00") is None
    assert pfd.decode_pad_context(cov(0, 0, 48)[:-4]) is None          # n says one more
    assert pfd.decode_pad_context(cov(0, 160, 48)) is None             # past the triangle
    assert pfd.decode_pad_context(survey([(0, 1, 0.5)])[:-1]) is None
    bad = bytearray(eph(0, 1))
    bad[7] = 0xD2
    assert pfd.decode_pad_context(bytes(bad)) is None                  # not an RTCM3 frame
    assert pfd.decode_pad_context(eph(0, 1)[:-1]) is None              # its length says otherwise


def test_join_puts_a_launch_back_together():
    ctx = pfd.join_pad_context([pfd.decode_pad_context(p) for p in launch()])
    assert ctx["missing"] == []
    assert ctx["cal"]["gyro_offset_lsb"] == (12, -7, 3)
    assert ctx["nav"]["quat"][0] == 0.699999988079071
    P = ctx["cov"]
    assert len(P) == 19 and all(len(row) == 19 for row in P)
    k = 0
    for i in range(19):                               # the triangle row by row, mirrored
        for j in range(i, 19):
            assert P[i][j] == P[j][i] == float(k)
            k += 1
    assert ctx["survey"]["corrections"] == [(0, 5, 1.5), (1, 12, -0.75)]
    assert len(ctx["eph_frames"]) == 3 and all(f[0] == 0xD3 for f in ctx["eph_frames"])


def test_join_says_what_was_lost():
    recs = [pfd.decode_pad_context(p) for p in launch()]
    lost = [r for r in recs if not (r["kind"] == "cov" and r["part"] == 2) and
            not (r["kind"] == "eph" and r["part"] == 1)]
    ctx = pfd.join_pad_context(lost)
    assert ctx["cov"] is None                         # a part of it is not it
    assert ctx["survey"] is not None
    assert len(ctx["eph_frames"]) == 2
    assert ctx["missing"] == ["cov", "eph"]
    # a u-blox board: the flight loop's calibrations only
    only_cal = pfd.join_pad_context([pfd.decode_pad_context(cal())])
    assert only_cal["cal"] is not None
    assert only_cal["missing"] == ["nav", "cov", "survey"]


def test_join_takes_each_launch_in_turn():
    recs = [pfd.decode_pad_context(p) for p in launch(7_000_000) + launch(90_000_000)]
    first, second = pfd.join_pad_context(recs, 0), pfd.join_pad_context(recs, 1)
    assert first["nav"]["time_us"] == 7_000_000 and second["nav"]["time_us"] == 90_000_000
    assert first["missing"] == [] and second["missing"] == []
    assert pfd.join_pad_context(recs, 2) is None
    assert pfd.join_pad_context([]) is None
    # the second launch's calibrations lost: its state still starts its own
    no_cal = [r for r in recs if not (r["kind"] == "cal" and r["time_us"] > 10_000_000)]
    assert pfd.join_pad_context(no_cal, 1)["nav"]["time_us"] == 90_000_000
    assert pfd.join_pad_context(no_cal, 1)["missing"] == ["cal"]


def test_parser_files_it_as_a_stream(tmp_path):
    frames = [frame(0x9E, p) for p in launch()]
    frames.append(frame(0x9E, header(9)))             # a malformed one is skipped
    p = tmp_path / "flight_pad_context.bin"
    p.write_bytes(b"".join(frames) + b"\x00" * 16)
    records, stats, _config = pfd.parse_binary_file(str(p))
    recs = records["PadContext"]
    assert [r["kind"] for r in recs] == ["cal", "nav"] + ["cov"] * 4 + ["survey"] + ["eph"] * 3
    assert pfd.join_pad_context(recs)["missing"] == []
    assert stats["type_counts"]["PadContext"] >= 10
