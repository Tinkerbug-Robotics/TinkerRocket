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


def acc(applied=1, offset=(0.0412, -0.0273, 0.0156), time_us=6_000_000) -> bytes:
    return header(6, time_us=time_us) + struct.pack("<B3f", applied, *offset)


def test_decode_the_hand_calibrated_accelerometer_offset():
    # #1629: PCX_ACC, beside PCX_CAL -- the low-g's offset, board frame
    r = pfd.decode_pad_context(acc())
    assert r["kind"] == "acc"
    assert r["applied"] is True
    assert r["offset_mps2"] == pytest_approx((0.0412, -0.0273, 0.0156))
    assert pfd.decode_pad_context(acc()[:-1]) is None            # a short body is not decoded
    ctx = pfd.join_pad_context([pfd.decode_pad_context(p) for p in [cal(), acc()] + launch()[1:]])
    assert ctx["acc"]["applied"] is True
    assert ctx["missing"] == []                                  # an older log without it misses nothing


def pytest_approx(t):
    import pytest
    return pytest.approx(t, abs=1e-6)


def test_the_offset_comes_off_the_low_g_and_high_g_as_the_converter_takes_it(tmp_path):
    # One ISM6 record, decoded with and without the flight's PCX_ACC: the low-g
    # and the high-g both come out the offset lower (board frame, the mounting
    # identity), the gyro untouched; a record whose offset was not applied (sim)
    # changes nothing.
    ism6 = struct.pack("<I hhh hhh hhh", 1_000_000, 1200, -900, 20000, 40, -30, 1250, 5, -6, 7)

    def decode(extra):
        p = tmp_path / f"flight_{len(extra)}.bin"
        p.write_bytes(b"".join([frame(0xA2, ism6)] + [frame(0x9E, e) for e in extra]) + b"\x00" * 16)
        records, _stats, _config = pfd.parse_binary_file(str(p))
        return records["ISM6HG256"][0]

    base, cal_on, cal_sim = decode([]), decode([acc()]), decode([acc(applied=0)])
    for axis, off in zip("xyz", (0.0412, -0.0273, 0.0156)):
        assert base[f"low_acc_{axis}"] - cal_on[f"low_acc_{axis}"] == pytest_approx(off)
        assert base[f"high_acc_{axis}"] - cal_on[f"high_acc_{axis}"] == pytest_approx(off)
        assert base[f"gyro_{axis}"] == cal_on[f"gyro_{axis}"]
        assert base[f"low_acc_{axis}"] == cal_sim[f"low_acc_{axis}"]


HG_M = (1.0081, 0.0042, -0.0031, -0.0027, 0.9934, 0.0056, 0.0018, -0.0049, 1.0122)


def hg(applied=1, matrix=HG_M, time_us=6_000_000) -> bytes:
    return header(7, time_us=time_us) + struct.pack("<B9f", applied, *matrix)


def test_decode_the_high_g_alignment():
    # PCX_HG, beside PCX_CAL and PCX_ACC -- the high-g's six-face alignment to the low-g
    r = pfd.decode_pad_context(hg())
    assert r["kind"] == "hg"
    assert r["applied"] is True
    assert r["matrix"] == pytest_approx(HG_M)
    assert pfd.decode_pad_context(hg()[:-1]) is None
    ctx = pfd.join_pad_context([pfd.decode_pad_context(p) for p in [cal(), acc(), hg()] + launch()[1:]])
    assert ctx["hg"]["matrix"] == pytest_approx(HG_M)
    assert ctx["missing"] == []                                  # an older log without it misses nothing


ANT_BODY_M = (0.12, 0.015, -0.04)    # the setting, body FRD
ANT_EARLIER_M = (0.30, 0.0, -0.02)   # an earlier setting the filter was still on (one sent just before launch waits)
ANT_NONE = dict(known=0, board_mm=(0, 0, 0), body_m=(0.0, 0.0, 0.0), used=0)   # the setting unknown


def ant(known=1, board_mm=(120, -15, 40), body_m=ANT_BODY_M, used=1, applied=None, time_us=6_000_000) -> bytes:
    """applied: the offset the navigation filter had on (body FRD); None is the setting if used, else none on."""
    if applied is None and used:
        applied = body_m
    a_known, a_m = (1, applied) if applied is not None else (0, (0.0, 0.0, 0.0))
    return header(8, time_us=time_us) + struct.pack("<B3h3fBB3f", known, *board_mm, *body_m, used, a_known, *a_m)


def test_decode_the_antenna_offset():
    # PCX_ANT, beside PCX_CAL -- the GNSS antenna's offset set on the flight, the setting and the filter's, whether
    # the navigation filter flew with it, and the offset that filter did fly with
    r = pfd.decode_pad_context(ant())
    assert r["kind"] == "ant"
    assert r["known"] is True and r["used"] is True
    assert r["board_mm"] == (120, -15, 40)
    assert r["body_m"] == pytest_approx(ANT_BODY_M)
    assert r["applied_known"] is True and r["applied_body_m"] == pytest_approx(ANT_BODY_M)
    assert pfd.decode_pad_context(ant()[:-1]) is None
    assert pfd.decode_pad_context(ant()[:-13]) is None          # the record is only ever its full size
    u = pfd.decode_pad_context(ant(**ANT_NONE))
    assert u["known"] is False and u["used"] is False and u["applied_known"] is False
    assert u["applied_body_m"] == (0.0, 0.0, 0.0)
    s = pfd.decode_pad_context(ant(used=0, applied=ANT_EARLIER_M))   # set, the filter still on an earlier one
    assert s["known"] is True and s["used"] is False
    assert s["board_mm"] == (120, -15, 40)
    assert s["body_m"] == pytest_approx(ANT_BODY_M)
    assert s["applied_known"] is True and s["applied_body_m"] == pytest_approx(ANT_EARLIER_M)
    e = pfd.decode_pad_context(ant(**ANT_NONE, applied=ANT_EARLIER_M))   # set Unknown, the filter on an earlier one
    assert e["known"] is False and e["used"] is False
    assert e["applied_known"] is True and e["applied_body_m"] == pytest_approx(ANT_EARLIER_M)
    n = pfd.decode_pad_context(ant(used=0))                      # set, but the filter never started
    assert n["known"] is True and n["used"] is False and n["applied_known"] is False
    assert n["applied_body_m"] == (0.0, 0.0, 0.0)
    ctx = pfd.join_pad_context([pfd.decode_pad_context(p) for p in [cal(), acc(), hg(), ant()] + launch()[1:]])
    assert ctx["ant"]["board_mm"] == (120, -15, 40)
    assert ctx["missing"] == []                                  # an older log without it misses nothing
    old = pfd.join_pad_context([pfd.decode_pad_context(p) for p in [cal()] + launch()[1:]])
    assert old["ant"] is None


def test_the_replay_extractor_passes_the_antenna_on():
    # tools/raw_replay/extract.py: the offset the navigation filter flew with becomes META antenna_m (body FRD) for
    # raw_replay, which flies it; a setting the filter did not fly with, META antenna_m_set beside it, which the
    # replay only names; META antenna_used says whether the filter flew with the setting; neither known, or an
    # older log without the record, none
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "raw_replay_extract", Path(__file__).resolve().parents[2] / "tools" / "raw_replay" / "extract.py")
    ex = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ex)

    def lines_of(*recs):
        return ex.antenna_lines(pfd.join_pad_context([pfd.decode_pad_context(p) for p in [cal(), *recs]]))

    def xyz(line):
        return [float(x) for x in line.split()[2:]]

    # flown with the setting
    lines = lines_of(ant(), *launch()[1:])
    assert [ln.split()[1] for ln in lines] == ["antenna_m", "antenna_used"]
    assert xyz(lines[0]) == pytest_approx(ANT_BODY_M)
    assert lines[1] == "META antenna_used 1"
    # a setting the filter did not take yet, flying an earlier known one: the replay flies the earlier one
    lines = lines_of(ant(used=0, applied=ANT_EARLIER_M), *launch()[1:])
    assert [ln.split()[1] for ln in lines] == ["antenna_m", "antenna_m_set", "antenna_used"]
    assert xyz(lines[0]) == pytest_approx(ANT_EARLIER_M)
    assert xyz(lines[1]) == pytest_approx(ANT_BODY_M)
    assert lines[2] == "META antenna_used 0"
    # the setting Unknown, the filter still on an earlier known one: that one flies, nothing set to name
    lines = lines_of(ant(**ANT_NONE, applied=ANT_EARLIER_M), *launch()[1:])
    assert [ln.split()[1] for ln in lines] == ["antenna_m", "antenna_used"]
    assert xyz(lines[0]) == pytest_approx(ANT_EARLIER_M)
    assert lines[1] == "META antenna_used 0"
    # set, but the filter never started: named only, the replay flies unknown as the flight did
    lines = lines_of(ant(used=0))
    assert [ln.split()[1] for ln in lines] == ["antenna_m_set", "antenna_used"]
    assert xyz(lines[0]) == pytest_approx(ANT_BODY_M)
    assert lines[1] == "META antenna_used 0"
    # unknown, and nothing applied
    assert lines_of(ant(**ANT_NONE)) == []
    assert lines_of() == []
    assert ex.antenna_lines(None) == []


def test_the_alignment_comes_before_the_high_g_bias_and_offset(tmp_path):
    # The high-g of one ISM6 record with the flight's PCX_HG: M times the board-frame reading (after the ISM6's
    # Z rotation; the mounting identity), then the bias and the offset as before; the low-g and gyro untouched;
    # an alignment not applied (sim) changes nothing.
    ism6 = struct.pack("<I hhh hhh hhh", 1_000_000, 1200, -900, 20000, 40, -30, 1250, 5, -6, 7)

    def decode(extra):
        p = tmp_path / f"flight_hg_{len(extra)}_{extra[-1][7] if extra else 0}.bin"
        p.write_bytes(b"".join([frame(0xA2, ism6)] + [frame(0x9E, e) for e in extra]) + b"\x00" * 16)
        records, _stats, config = pfd.parse_binary_file(str(p))
        return records["ISM6HG256"][0], config

    (base, cfg), (on, _), (sim, _) = decode([acc()]), decode([acc(), hg()]), decode([acc(), hg(applied=0)])
    _, s_hi, _ = pfd.ism6_scales(cfg["low_g_fs_g"], cfg["high_g_fs_g"], cfg["gyro_fs_dps"])
    import math
    r = math.radians(cfg["ism6_rot_z_deg"])
    h = [(40 * math.cos(r) + 30 * math.sin(r)) * s_hi, (40 * math.sin(r) - 30 * math.cos(r)) * s_hi, 1250 * s_hi]
    hb, off = cfg["hg_bias"], (0.0412, -0.0273, 0.0156)
    for i, axis in enumerate("xyz"):
        want = sum(HG_M[3 * i + j] * h[j] for j in range(3)) - hb[i] - off[i]
        assert on[f"high_acc_{axis}"] == pytest_approx(want)
        assert on[f"low_acc_{axis}"] == base[f"low_acc_{axis}"]
        assert sim[f"high_acc_{axis}"] == base[f"high_acc_{axis}"]
