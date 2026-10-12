"""Raw RTCM3 frame record (GNSS_RTCM_MSG, 0x95): the reader and the extractor.

No flight log carries the record yet, so the .bin is synthesised with the
out computer's framing, [AA 55 AA 55][type][len][payload][CRC16 BE], and the
FC's split rule: GnssRtcmHeader (time_us, part = index << 4 | count), then up
to 219 frame bytes. The frames are real: an MSM7 epoch the Beetle's LC86G sent
on 2026-09-25, plus a synthetic 400-byte frame that has to go in two parts.
Expected values are the independent decode pinned in tests_cpp/test_lc86_parser.cpp.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import extract_gnss_rtcm as ext              # noqa: E402
import plot_flight_data_mini as pfd          # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
CHUNK = 219                                   # GNSS_RTCM_MAX_CHUNK

GPS = bytes.fromhex(
    "d30099435d187d2eff42002061824a0200000000200000007fd1d1515212131493514000"
    "00000075e22b8a1ace5618efb91cfa7c059043420df923e12ff93e2dfdabde3a0cfa4ca2"
    "ef3db3246bbb8556cc02a3fa6c088c6bff775a433cf1425a043f4ee701a7d34153f04008"
    "21feae56020de8000001374a9e10000072de2c6398a7c40fe484e4b372cd84419459d81b"
    "dec3bd37896a0d8a1bd00a28df3011")
QZSS = bytes.fromhex("d3001645dd187d2eff40002000000000000000000000000000e8aea0")


def rtcm_frame(payload: bytes) -> bytes:
    head = bytes([0xD3, len(payload) >> 8, len(payload) & 0xFF]) + payload
    return head + ext.crc24q(head).to_bytes(3, "big")


# A 1019-numbered payload, 394 bytes: too long for one record.
LONG = rtcm_frame(bytes([0x3F, 0xB0]) + bytes(range(256)) + bytes(136))


def i2s_frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def records_for(frame: bytes, time_us: int) -> list[bytes]:
    """The FC's split: one 0x95 record per part."""
    parts = (len(frame) + CHUNK - 1) // CHUNK
    return [i2s_frame(0x95, struct.pack("<IB", time_us, (k << 4) | parts)
                      + frame[k * CHUNK:(k + 1) * CHUNK])
            for k in range(parts)]


def build_log(tmp_path: Path, extra: list[bytes] = ()) -> Path:
    out = b"".join(records_for(GPS, 1_000_000)
                   + records_for(LONG, 1_000_400)
                   + records_for(QZSS, 1_000_900)
                   + list(extra))
    p = tmp_path / "flight.bin"
    p.write_bytes(out)
    return p


def test_the_long_frame_really_splits():
    assert len(GPS) <= CHUNK and len(QZSS) <= CHUNK
    assert len(LONG) > CHUNK and len(records_for(LONG, 0)) == 2
    assert ext.frame_ok(GPS) and ext.frame_ok(QZSS) and ext.frame_ok(LONG)


def test_reader_decodes_and_joins_parts(tmp_path):
    records, stats, _ = pfd.parse_binary_file(str(build_log(tmp_path)))
    recs = records["GNSS_RTCM"]
    assert [(r["part"], r["parts"]) for r in recs] == [(0, 1), (0, 2), (1, 2), (0, 1)]
    assert stats["type_counts"]["GNSS_RTCM"] == 4
    frames, dropped = pfd.join_gnss_rtcm(recs)
    assert dropped == 0
    assert frames == [(1_000_000, GPS), (1_000_400, LONG), (1_000_900, QZSS)]


def test_a_frame_missing_a_part_is_dropped_whole():
    # Second half of LONG lost on the link: the first half must not be glued
    # onto the next frame, and the frames around it survive.
    a, b = [pfd.decode_gnss_rtcm(r[6:-2]) for r in records_for(LONG, 5)]
    g = pfd.decode_gnss_rtcm(records_for(GPS, 9)[0][6:-2])
    frames, dropped = pfd.join_gnss_rtcm([a, g])
    assert frames == [(9, GPS)] and dropped == 1
    frames, dropped = pfd.join_gnss_rtcm([g, b])          # an orphan second half
    assert frames == [(9, GPS)] and dropped == 1


def test_malformed_headers_are_refused():
    assert pfd.decode_gnss_rtcm(struct.pack("<IB", 1, 0x00) + b"\xd3") is None   # count 0
    assert pfd.decode_gnss_rtcm(struct.pack("<IB", 1, 0x22) + b"\xd3") is None   # index >= count
    assert pfd.decode_gnss_rtcm(struct.pack("<IB", 1, 0x01)) is None             # no frame bytes


def test_extractor_writes_the_stream_and_the_csv(tmp_path, capsys):
    log = build_log(tmp_path)
    assert ext.main([str(log), "--csv"]) == 0
    assert (tmp_path / "flight.rtcm3").read_bytes() == GPS + LONG + QZSS
    rows = (tmp_path / "flight.msm7.csv").read_text().splitlines()
    assert len(rows) == 1 + 9                       # header + GPS's nine satellites
    g30 = next(r.split(",") for r in rows[1:] if r.split(",")[4] == "30")
    assert g30[1] == "525058000" and g30[2] == "GPS"
    assert abs(float(g30[6]) - 20768905.4604) < 1e-3   # pseudorange
    assert abs(float(g30[7]) - 20768900.3460) < 1e-3   # carrier phase
    assert abs(float(g30[8]) - -149.9350) < 1e-4       # Doppler, m/s
    assert float(g30[9]) == 44.125 and g30[10] == "557056"
    assert "1019 x1" in capsys.readouterr().out


def test_extractor_leaves_out_a_frame_whose_crc_fails(tmp_path):
    bad = bytearray(GPS)
    bad[40] ^= 0x10
    log = build_log(tmp_path, extra=records_for(bytes(bad), 2_000_000))
    frames, st = ext.extract(log)
    assert st["bad_crc"] == 1 and len(frames) == 3


def eph_1019(sv: int, iode: int, toe_s: int, sqrt_a: float) -> bytes:
    """A GPS ephemeris frame with the identifying fields set, the rest zero."""
    bits = 488                                   # 1019 is 488 bits (61 bytes)
    v = 0
    def put(off, width, val):
        nonlocal v
        v |= (val & ((1 << width) - 1)) << (bits - off - width)
    put(0, 12, 1019)
    put(12, 6, sv)
    put(48, 8, iode)
    put(256, 32, round(sqrt_a * 2 ** 19))
    put(288, 16, toe_s // 16)
    return rtcm_frame(v.to_bytes(61, "big"))


def test_ephemeris_is_named_by_satellite():
    e = ext.decode_eph(eph_1019(sv=7, iode=42, toe_s=158400, sqrt_a=5153.6543))
    assert e == {"constellation": "GPS", "sv": 7, "iod": 42, "toe_s": 158400.0,
                 "sqrt_a": round(5153.6543 * 2 ** 19) * 2.0 ** -19}
    assert ext.decode_eph(GPS) is None             # an MSM7 is not ephemeris


def test_extractor_summarises_ephemeris(tmp_path, capsys):
    log = build_log(tmp_path, extra=records_for(eph_1019(7, 42, 158400, 5153.65), 3_000_000)
                                    + records_for(eph_1019(7, 42, 158400, 5153.65), 23_000_000))
    assert ext.main([str(log)]) == 0
    out = capsys.readouterr().out
    assert "GPS       7:   2,  20.0 s, iod 42, toe 158400 s, sqrtA 5153.650" in out
