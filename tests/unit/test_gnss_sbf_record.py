"""SBF blocks in the raw-frame record (GNSS_RTCM_MSG, 0x95): the reader and the extractors.

The V9's mosaic-G5 sends SBF (Septentrio Binary Format), and the flight
computer logs every block whose CRC checks, byte for byte, in the record the
Beetle's LC86G fills with RTCM3. No flight log carries one yet, so the .bin is
synthesised with the out computer's framing, [AA 55 AA 55][type][len][payload]
[CRC16 BE], and the FC's split rule: GnssRtcmHeader (time_us, part = index << 4
| count), then up to 219 block bytes. The blocks are built from their fields
(the SBF reference guide's layouts) with their Length and a CRC computed here
bit by bit, independently of the reader's; the position in them is made up.
"""

from __future__ import annotations

import csv
import re
import struct
import sys
from datetime import datetime, timedelta
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import extract_gnss_rtcm as ext_rtcm         # noqa: E402
import extract_gnss_sbf as ext               # noqa: E402
import plot_flight_data_mini as pfd          # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
CHUNK = 219                                   # GNSS_RTCM_MAX_CHUNK
WNC = 2440                                    # GPS week 2440
TOW0 = 345_600_000                            # its fifth day, 00:00 GPS time, ms


def crc_ccitt(data: bytes) -> int:
    """CRC-CCITT as the SBF guide gives it: x^16 + x^12 + x^5 + 1, seed 0,
    no reflection, no final XOR."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def sbf_block(number: int, tow_ms: int, body: bytes = b"", revision: int = 0,
              wnc: int = WNC) -> bytes:
    """A block as the receiver sends it: the time stamp, then `body`, padded
    to a multiple of 4, behind "$@", its CRC, ID and Length."""
    rest = struct.pack("<IH", tow_ms, wnc) + body
    rest += bytes(-(8 + len(rest)) % 4)
    rest = struct.pack("<HH", number | revision << 13, 8 + len(rest)) + rest
    return b"$@" + struct.pack("<H", crc_ccitt(rest)) + rest


def pvt_geodetic(tow_ms: int) -> bytes:
    """PVTGeodetic rev 2, 96 bytes: a standalone fix at a made-up position."""
    body = struct.pack("<BB ddd f 4f d f 4B HH I BB H 3H B",
                       1, 0,                         # Mode: standalone; Error: none
                       0.5, -1.2, 1234.5,            # latitude, longitude (rad), height (m)
                       -20.25,                       # Undulation, m
                       0.5, -0.25, 30.0, 4.0,        # Vn, Ve, Vu (m/s), COG (deg)
                       0.123456, 0.75,               # RxClkBias (ms), RxClkDrift (ppm)
                       0, 0, 21, 0,                  # TimeSystem, Datum, NrSV, WACorrInfo
                       65535, 65535,                 # ReferenceID, MeanCorrAge: none
                       0x3, 0, 0, 0,                 # SignalInfo, AlertFlag, NrBases, PPPInfo
                       12, 150, 220,                 # Latency (0.1 ms), HAccuracy, VAccuracy (cm)
                       0)                            # Misc
    return sbf_block(4007, tow_ms, body, revision=2)


def receiver_time(tow_ms: int) -> bytes:
    """ReceiverTime, 24 bytes: the UTC of that instant, 18 leap seconds, fully synced."""
    utc = datetime(1980, 1, 6) + timedelta(weeks=WNC, milliseconds=tow_ms - 18_000)
    return sbf_block(5914, tow_ms, struct.pack("<6b bB", utc.year - 2000, utc.month, utc.day,
                                               utc.hour, utc.minute, utc.second, 18, 7))


def xpps_offset(tow_ms: int) -> bytes:
    """xPPSOffset, 20 bytes: SyncAge (s), Timescale, the 1PPS edge's offset (ns)."""
    return sbf_block(5911, tow_ms, struct.pack("<BBf", 1, 1, 12.5))


def meas_epoch(tow_ms: int, n1: int = 34, n2: int = 2) -> bytes:
    """MeasEpoch rev 1 in its shape: N1 Type1 sub-blocks of 20 bytes, each
    followed by N2 Type2 of 12. 34 x 2 is 1,516 bytes, seven records; the
    measurements themselves are filler."""
    body = struct.pack("<6B", n1, 20, 12, 0, 0, 0)    # N1, SB1Length, SB2Length, CommonFlags, CumClkJumps, Reserved
    for ch in range(n1):
        body += struct.pack("<4B I i H b B H 2B",
                            ch, 0, ch + 1, 0,                         # RxChannel, Type, SVID, Misc
                            20_000_000 + 1013 * ch, -150_000 * ch,    # CodeLSB, Doppler
                            7 * ch, 0, 180 + ch, 4000, 0, n2)          # CarrierLSB/MSB, CN0, LockTime, ObsInfo, N2
        for k in range(n2):
            body += struct.pack("<4B b B 3H", 3 + k, 40, 150, 0, 0, 0, 100 + k, 200, 300)
    return sbf_block(4027, tow_ms, body, revision=1)


def block_of(length: int) -> bytes:
    """An RxMessage-numbered block exactly `length` bytes long (a multiple of 4)."""
    return sbf_block(4103, TOW0, bytes(i & 0xFF for i in range(length - 14)))


def unpadded(length: int) -> bytes:
    """"$@", a Length that is its own and a CRC that checks, but a length that
    is no multiple of 4: framed like SBF, and not a block."""
    rest = struct.pack("<HHIH", 4103, length, TOW0, WNC) + bytes(length - 14)
    return b"$@" + struct.pack("<H", crc_ccitt(rest)) + rest


def rtcm_frame(payload: bytes) -> bytes:
    head = bytes([0xD3, len(payload) >> 8, len(payload) & 0xFF]) + payload
    return head + ext_rtcm.crc24q(head).to_bytes(3, "big")


PVT = pvt_geodetic(TOW0)
TIME = receiver_time(TOW0)
MEAS = meas_epoch(TOW0)
PPS = xpps_offset(TOW0)
BLOCKS = [(1_000_000, MEAS), (1_004_000, PVT), (1_020_000, TIME), (1_021_000, PPS)]


def i2s_frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def records_for(frame: bytes, time_us: int) -> list[bytes]:
    """The FC's split: one 0x95 record per part."""
    parts = (len(frame) + CHUNK - 1) // CHUNK
    assert parts <= 15                        # the count's nibble
    return [i2s_frame(0x95, struct.pack("<IB", time_us, (k << 4) | parts)
                      + frame[k * CHUNK:(k + 1) * CHUNK])
            for k in range(parts)]


def decoded(frames) -> list[dict]:
    """[(time_us, frame)] -> the 0x95 records the reader decodes from them."""
    return [pfd.decode_gnss_rtcm(r[6:-2]) for t, f in frames for r in records_for(f, t)]


def build_log(tmp_path: Path, frames=BLOCKS) -> Path:
    p = tmp_path / "flight.bin"
    p.write_bytes(b"".join(r for t, f in frames for r in records_for(f, t)))
    return p


def test_blocks_are_built_as_the_receiver_sends_them():
    assert crc_ccitt(b"123456789") == 0x31C3          # the CRC's catalogued check value
    assert [len(b) for b in (PVT, TIME, MEAS, PPS)] == [96, 24, 1516, 20]
    assert [len(records_for(b, 0)) for b in (PVT, TIME, MEAS, PPS)] == [1, 1, 7, 1]
    assert all(pfd.sbf_block_ok(b) for b in (PVT, TIME, MEAS, PPS))
    assert [pfd.sbf_block_name(b) for b in (PVT, TIME, MEAS, PPS)] == \
        ["PVTGeodetic", "ReceiverTime", "MeasEpoch", "xPPSOffset"]
    assert (pfd.sbf_block_number(PVT), pfd.sbf_block_revision(PVT)) == (4007, 2)
    assert (pfd.sbf_block_number(MEAS), pfd.sbf_block_revision(MEAS)) == (4027, 1)
    assert pfd.sbf_tow_wnc(PVT) == (TOW0, WNC)
    assert pfd.sbf_gps_ms(PVT) == WNC * 604_800_000 + TOW0
    assert struct.unpack_from("<dd", PVT, 16) == (0.5, -1.2)
    assert pfd.sbf_block_name(sbf_block(4999, TOW0)) == "block 4999"


def test_do_not_use_time_stamps_read_as_none():
    assert pfd.sbf_tow_wnc(sbf_block(5914, 4294967295, bytes(8), wnc=65535)) == (None, None)
    assert pfd.sbf_tow_wnc(sbf_block(5914, 4294967295, bytes(8))) == (None, WNC)
    assert pfd.sbf_gps_ms(sbf_block(5914, 4294967295, bytes(8))) is None
    assert pfd.sbf_tow_wnc(b"$@" + bytes(10)) == (None, None)       # too short to carry one


def test_reader_joins_and_splits_the_blocks(tmp_path):
    records, stats, _ = pfd.parse_binary_file(str(build_log(tmp_path)))
    recs = records["GNSS_RTCM"]
    assert stats["type_counts"]["GNSS_RTCM"] == len(recs) == 7 + 1 + 1 + 1
    assert [(r["part"], r["parts"]) for r in recs[:8]] == [(k, 7) for k in range(7)] + [(0, 1)]
    frames, dropped = pfd.join_gnss_rtcm(recs)
    assert dropped == 0 and frames == BLOCKS
    assert pfd.split_gnss_raw_frames(frames) == {"rtcm3": [], "sbf": BLOCKS, "bad": 0}


def test_the_record_boundary_is_219_bytes():
    # 219 bytes go in one record and 220 in two. An SBF block is a multiple of
    # 4 long, so 216 is the longest that goes whole and 220 the shortest split;
    # a 219-byte frame is RTCM3, or framed like SBF and refused for its length.
    one, two = block_of(216), block_of(220)
    r219 = rtcm_frame(bytes([0x3F, 0xB0]) + bytes(211))
    odd = unpadded(219)
    seq = [(10, one), (20, two), (30, r219), (40, odd)]
    assert [len(f) for _, f in seq] == [216, 220, 219, 219]
    recs = decoded(seq)
    assert [(r["part"], r["parts"], len(r["data"])) for r in recs] == \
        [(0, 1, 216), (0, 2, 219), (1, 2, 1), (0, 1, 219), (0, 1, 219)]
    frames, dropped = pfd.join_gnss_rtcm(recs)
    assert dropped == 0 and frames == seq
    raw = pfd.split_gnss_raw_frames(frames)
    assert raw == {"rtcm3": [(30, r219)], "sbf": [(10, one), (20, two)], "bad": 1}


def test_the_longest_block_goes_in_fifteen_parts():
    # Fifteen parts (the count's nibble) carry 15 x 219 = 3,285 bytes; the
    # longest SBF block that fits is 3,284.
    full = unpadded(3285)
    recs = decoded([(7, full)])
    assert len(recs) == 15 and (recs[-1]["part"], recs[-1]["parts"], len(recs[-1]["data"])) == (14, 15, 219)
    assert pfd.join_gnss_rtcm(recs) == ([(7, full)], 0)
    big = block_of(3284)
    frames, dropped = pfd.join_gnss_rtcm(decoded([(8, big)]))
    assert dropped == 0 and pfd.split_gnss_raw_frames(frames)["sbf"] == [(8, big)]


def test_a_missing_part_drops_only_its_own_block():
    recs = decoded(BLOCKS)
    for lost in (0, 3, 6):                    # MeasEpoch's first, a middle and its last record
        frames, dropped = pfd.join_gnss_rtcm(recs[:lost] + recs[lost + 1:])
        assert frames == BLOCKS[1:] and dropped == 1
    # two MeasEpochs, the first's last record and the second's first lost:
    # two blocks gone, each counted once, and the PVTGeodetic after them kept
    seq = [(1_000_000, MEAS), (1_050_000, meas_epoch(TOW0 + 50)), (1_054_000, PVT)]
    recs = decoded(seq)
    assert pfd.join_gnss_rtcm(recs[:6] + recs[8:]) == ([seq[2]], 2)


def test_a_block_that_fails_its_crc_is_bad(tmp_path):
    bad = bytearray(PVT)
    bad[20] ^= 0x01                           # one bit of the latitude
    assert not pfd.sbf_block_ok(bytes(bad))
    lying = bytearray(TIME)                   # its CRC right, its Length not its own
    lying[6:8] = struct.pack("<H", 28)
    lying[2:4] = struct.pack("<H", crc_ccitt(bytes(lying[4:])))
    assert not pfd.sbf_block_ok(bytes(lying))
    log = build_log(tmp_path, BLOCKS + [(2_000_000, bytes(bad)), (2_100_000, bytes(lying))])
    blocks, st = ext.extract(log)
    assert blocks == BLOCKS and st["bad"] == 2 and st["incomplete"] == 0


def test_rtcm3_and_sbf_frames_split_apart(tmp_path, capsys):
    r1 = rtcm_frame(bytes([0x3F, 0xB0]) + bytes(59))      # a 1019-numbered frame
    r2 = rtcm_frame(bytes([0x43, 0x50]) + bytes(394))     # a 1077-numbered one, two records
    mixed = [(1_000_000, MEAS), (1_001_000, r1), (1_004_000, PVT), (1_010_000, r2),
             (1_020_000, TIME)]
    log = build_log(tmp_path, mixed)
    records, _, _ = pfd.parse_binary_file(str(log))
    frames, dropped = pfd.join_gnss_rtcm(records["GNSS_RTCM"])
    assert dropped == 0 and frames == mixed
    raw = pfd.split_gnss_raw_frames(frames)
    assert raw["rtcm3"] == [mixed[1], mixed[3]] and raw["bad"] == 0
    assert raw["sbf"] == [mixed[0], mixed[2], mixed[4]]
    # each extractor writes its own kind and points to the other for the rest
    assert ext.main([str(log)]) == 0
    assert (tmp_path / "flight.sbf").read_bytes() == MEAS + PVT + TIME
    assert "2 RTCM3 frames (LC86G), not SBF: see extract_gnss_rtcm.py" in capsys.readouterr().out
    good, st = ext_rtcm.extract(log)
    assert good == raw["rtcm3"] and st["bad_crc"] == 0 and st["sbf_blocks"] == 3


def test_the_rtcm3_extractor_counts_sbf_blocks_not_bad_crcs(tmp_path, capsys):
    log = build_log(tmp_path)
    good, st = ext_rtcm.extract(log)
    assert good == [] and st["bad_crc"] == 0 and st["sbf_blocks"] == 4
    assert ext_rtcm.main([str(log)]) == 0
    out = capsys.readouterr().out
    assert "10 records -> 4 frames (0 incomplete, 0 bad CRC)" in out
    assert "4 SBF blocks (mosaic-G5), not RTCM3: see extract_gnss_sbf.py" in out


def test_an_lc86g_log_reads_as_before(tmp_path, capsys):
    frames = [(1_000_000, rtcm_frame(bytes([0x3F, 0xB0]) + bytes(59))),
              (1_001_000, rtcm_frame(bytes([0x43, 0x50]) + bytes(394)))]
    log = build_log(tmp_path, frames)
    good, st = ext_rtcm.extract(log)
    assert good == frames
    assert st == {"records": 3, "frames": 2, "dropped_parts": 0, "bad_crc": 0, "sbf_blocks": 0}
    assert ext_rtcm.main([str(log)]) == 0
    assert "SBF" not in capsys.readouterr().out
    records, _, _ = pfd.parse_binary_file(str(log))
    pfd.print_gnss_sbf_summary(records["GNSS_RTCM"])
    assert capsys.readouterr().out == ""


def test_extractor_writes_a_byte_exact_sbf_file_and_the_times(tmp_path, capsys):
    blocks = [(1_000_000 + 50_000 * k, meas_epoch(TOW0 + 50 * k)) for k in range(5)]
    blocks += [(1_260_000, PVT), (1_270_000, TIME), (1_280_000, PPS)]
    log = build_log(tmp_path, blocks)
    assert ext.main([str(log), "--times"]) == 0
    assert (tmp_path / "flight.sbf").read_bytes() == b"".join(b for _, b in blocks)
    with (tmp_path / "flight.sbf_times.csv").open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert [r["block"] for r in rows] == ["MeasEpoch"] * 5 + ["PVTGeodetic", "ReceiverTime", "xPPSOffset"]
    assert rows[1]["tow_ms"] == str(TOW0 + 50) and rows[1]["length"] == "1516"
    assert rows[5] == {"fc_time_us": "1260000", "block": "PVTGeodetic", "number": "4007",
                       "revision": "2", "tow_ms": str(TOW0), "wnc": str(WNC), "length": "96"}
    out = capsys.readouterr().out
    assert "38 records -> 8 frames (0 incomplete, 0 bad)" in out
    assert "blocks: MeasEpoch x5, PVTGeodetic x1, ReceiverTime x1, xPPSOffset x1" in out
    assert "receiver time week 2440 TOW 345600.000 s to week 2440 TOW 345600.200 s (0.20 s)" in out
    assert "MeasEpoch: 5 epochs over 0.20 s -> 20.00 Hz" in out
    assert ext.main([str(log), "-o", str(tmp_path / "copy.sbf")]) == 0
    assert (tmp_path / "copy.sbf").read_bytes() == (tmp_path / "flight.sbf").read_bytes()


def test_the_log_summary_reports_sbf_blocks(tmp_path, capsys):
    blocks = [(1_000_000 + 50_000 * k, meas_epoch(TOW0 + 50 * k)) for k in range(5)]
    blocks.append((1_300_000, PVT))
    records, _, _ = pfd.parse_binary_file(str(build_log(tmp_path, blocks)))
    recs = records["GNSS_RTCM"]
    pfd.print_gnss_sbf_summary(recs[:3] + recs[4:])     # a record of the first MeasEpoch lost
    out = capsys.readouterr().out
    assert "35 records -> 5 blocks" in out
    assert "Dropped: 1 incomplete (a record lost), 0 bad" in out
    assert re.search(r"MeasEpoch +4\n", out) and re.search(r"PVTGeodetic +1\n", out)
    assert "MeasEpoch: 4 epochs over 0.15 s of receiver time -> 20.00 Hz" in out
