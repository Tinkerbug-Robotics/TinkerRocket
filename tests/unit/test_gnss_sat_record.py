"""GNSS per-satellite record (GNSS_SAT_MSG, 0x90): decoder + rig converter.

No golden flight carries the record yet (it is new), so the .bin under test
is synthesised here with the same framing the out computer writes:
[AA 55 AA 55][type][len][payload][CRC16 BE].  The wire layout is pinned on the
firmware side by static_asserts and the host gtest; this pins the Python
reading of it, the tracked-first truncation contract, and that the COCOM
converter produces frames the rig's own UBX parser reads back unchanged.
"""

from __future__ import annotations

import datetime as dt
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
for p in (REPO_ROOT / "Data_Analysis",
          REPO_ROOT / "tools" / "gnss-cocom",
          REPO_ROOT / "tools" / "gnss-cocom" / "sdr"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import plot_flight_data_mini as pfd          # noqa: E402
import cocom_flightlog as cfl                # noqa: E402
import ublox_binary as ubx                   # noqa: E402

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return PREAMBLE + body + bytes([crc >> 8, crc & 0xFF])


def sat_block(gnss_id, sv_id, cno, elev, azim, used, quality, health=1, eph=True, alm=False):
    flags = ((1 if used else 0)
             | ((quality & 7) << 1)
             | ((health & 3) << 4)
             | (0x40 if eph else 0)
             | (0x80 if alm else 0))
    return struct.pack("<BBBbBB", gnss_id, sv_id, cno, elev, azim // 2, flags)


def sat_payload(time_us, itow_ms, num_svs, blocks: list[bytes]) -> bytes:
    return struct.pack("<IIBB", time_us, itow_ms, num_svs, len(blocks)) + b"".join(blocks)


def gnss_payload(time_us, utc: dt.datetime, ms, fix, nsat, lat, lon, alt_m, ve, vn, vu) -> bytes:
    return struct.pack("<IHBBBBBHBBBiiiiiiBB",
                       time_us, utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second, ms,
                       fix, nsat, 15,
                       int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000),
                       int(ve * 1000), int(vn * 1000), int(vu * 1000), 3, 5)


# A Wednesday, 12:00:00 UTC -> GPS tow = (3*86400 + 43200 + 18) s
UTC0 = dt.datetime(2026, 9, 9, 12, 0, 0)
ITOW0 = ((3 * 86400 + 12 * 3600 + 18) * 1000)


def build_log(tmp_path: Path) -> Path:
    frames = []
    # Epoch 0: PVT then SAT (receiver order), 3 satellites, two tracked.
    frames.append(frame(0xA1, gnss_payload(1_000_000, UTC0, 0, 3, 2, 45.0, -122.0, 100.0, 1.0, 2.0, 3.0)))
    frames.append(frame(0x90, sat_payload(1_004_000, ITOW0, 3, [
        sat_block(0, 5, 42, 61, 180, True, 7),
        sat_block(2, 12, 0, 8, 90, False, 1, health=0, eph=False),
        sat_block(6, 3, 30, -4, 358, True, 4),
    ])))
    # Epoch 1 (55.6 ms later): a truncated table, num_svs > num_blocks.
    frames.append(frame(0xA1, gnss_payload(1_055_556, UTC0, 56, 3, 1, 45.0, -122.0, 100.5, 0.0, 0.0, -1.0)))
    frames.append(frame(0x90, sat_payload(1_059_000, ITOW0 + 56, 40, [
        sat_block(0, 7, 35, 30, 10, True, 6),
    ])))
    # Epoch 2: a PVT with no time solution (year 1980) and no SAT partner.
    frames.append(frame(0xA1, gnss_payload(1_111_111, dt.datetime(1980, 1, 6), 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0)))
    # Garbage: a 0x90 whose length is not 10 + 6n (must be dropped, not crash).
    frames.append(frame(0x90, b"\x00" * 13))
    p = tmp_path / "synthetic.bin"
    p.write_bytes(b"".join(frames))
    return p


def test_decoder_reads_the_record(tmp_path):
    records, stats, _ = pfd.parse_binary_file(str(build_log(tmp_path)))
    sats = records["GNSS_SAT"]
    assert len(sats) == 2, "the malformed 13-byte frame must be dropped, the two real ones kept"
    e0 = sats[0]
    assert (e0["time_us"], e0["itow_ms"], e0["num_svs"], e0["num_blocks"]) == (1_004_000, ITOW0, 3, 3)
    assert e0["n_tracked"] == 2 and e0["n_used"] == 2
    assert e0["cno_max"] == 42 and e0["cno_mean_used"] == 36.0
    s = e0["sats"]
    assert [x["gnss"] for x in s] == ["GPS", "Galileo", "GLONASS"]
    assert s[0] == {"gnss_id": 0, "gnss": "GPS", "sv_id": 5, "cno_dbhz": 42, "elev_deg": 61,
                    "azim_deg": 180, "used": True, "quality": 7, "health": 1, "eph": True, "alm": False}
    assert s[1]["cno_dbhz"] == 0 and s[1]["quality"] == 1 and s[1]["health"] == 0 and not s[1]["eph"]
    assert s[2]["elev_deg"] == -4 and s[2]["azim_deg"] == 358 and s[2]["quality"] == 4
    # Truncated epoch: the receiver's count survives beside the logged count.
    e1 = sats[1]
    assert (e1["num_svs"], e1["num_blocks"], len(e1["sats"])) == (40, 1, 1)
    assert stats["type_counts"]["GNSS_SAT"] == 3       # counted at the frame level, incl. the bad one
    assert stats["type_counts"]["GNSS"] == 3


def test_converter_pairs_by_itow_and_roundtrips_through_the_rig_parser(tmp_path):
    records, _, _ = pfd.parse_binary_file(str(build_log(tmp_path)))
    lines, st = cfl.convert(records)
    assert st == {"pvt": 3, "sat": 2, "fixes": 2, "paired_by_itow": 2,
                  "paired_by_time": 0, "unpaired_pvt": 1, "orphan_sat": 0}

    # Every line is `t U <hex>`.  Epochs are in time order and each NAV-SAT
    # sits immediately BEFORE the NAV-PVT that closes its epoch, even though
    # the receiver emitted it a few ms later (so its own timestamp is later):
    # correlate.py treats NAV-PVT as the epoch marker, same as cocom_fcdiag.
    parsed = []
    for ln in lines:
        t, u, hexs = ln.split(" ")
        assert u == "U"
        raw = bytes.fromhex(hexs)
        assert raw[0] == ubx.CLS_NAV
        parsed.append((float(t), raw[1], raw[2:]))
    kinds = [x[1] for x in parsed]
    assert kinds == [ubx.MSG_NAV_SAT, ubx.MSG_NAV_PVT, ubx.MSG_NAV_SAT, ubx.MSG_NAV_PVT, ubx.MSG_NAV_PVT]
    # Stamps are printed to the millisecond, like the console converter's.
    pvt_times = [x[0] for x in parsed if x[1] == ubx.MSG_NAV_PVT]
    assert pvt_times == sorted(pvt_times) == [1.0, 1.056, 1.111]
    assert parsed[0][0] == 1.004 and parsed[2][0] == 1.059   # SAT keeps its own stamp

    sat0 = ubx.parse_nav_sat(parsed[0][2])
    assert [(s["gnss"], s["svid"], s["cn0"], s["elev"], s["azim"], s["used_in_fix"], s["quality"])
            for s in sat0] == [(0, 5, 42, 61, 180, True, 7),
                                (2, 12, 0, 8, 90, False, 1),
                                (6, 3, 30, -4, 358, True, 4)]
    # The rig names constellations its own way (GAL, GLO); the id is the contract.
    assert [s["gnss_name"] for s in sat0] == [ubx.GNSS_ID[0], ubx.GNSS_ID[2], ubx.GNSS_ID[6]]
    assert sat0[0]["ephemeris"] and not sat0[0]["almanac"] and sat0[0]["health"] == 1

    pvt0 = ubx.parse_nav_pvt(parsed[1][2])
    assert pvt0["itow_ms"] == ITOW0 == struct.unpack_from("<I", parsed[0][2], 0)[0]
    assert pvt0["has_fix"] and pvt0["fix_type"] == 3 and pvt0["num_sv"] == 2
    assert abs(pvt0["lat"] - 45.0) < 1e-6 and abs(pvt0["alt_m"] - 100.0) < 1e-3
    # ENU (1, 2, 3) m/s in the log -> NED (2, 1, -3) in NAV-PVT.
    assert pvt0["vel"] == (2.0, 1.0, -3.0)

    # The fix with no time solution converts too (iTOW 0, no fix), so the
    # rig sees the outage rather than a gap.
    pvt2 = ubx.parse_nav_pvt(parsed[4][2])
    assert pvt2["itow_ms"] == 0 and not pvt2["has_fix"]


def test_converter_falls_back_to_the_fc_clock_when_utc_is_unresolved(tmp_path):
    # Same epoch, but the PVT's UTC is pre-2017 (no time solution yet) while
    # the SAT record still carries a real iTOW: pair by proximity instead.
    frames = [
        frame(0xA1, gnss_payload(5_000_000, dt.datetime(2000, 1, 1), 0, 0, 0, 0, 0, 0, 0, 0, 0)),
        frame(0x90, sat_payload(5_006_000, 123_456, 1, [sat_block(0, 1, 20, 10, 0, False, 2)])),
    ]
    p = tmp_path / "unresolved.bin"
    p.write_bytes(b"".join(frames))
    records, _, _ = pfd.parse_binary_file(str(p))
    lines, st = cfl.convert(records)
    assert st["paired_by_time"] == 1 and st["paired_by_itow"] == 0 and st["orphan_sat"] == 0
    pvt = ubx.parse_nav_pvt(bytes.fromhex(lines[1].split(" ")[2])[2:])
    assert pvt["itow_ms"] == 123_456          # borrowed from the paired SAT record


def test_itow_from_utc_matches_gps_week_arithmetic():
    assert cfl.itow_from_utc({"year": 2026, "month": 9, "day": 9, "hour": 12,
                              "minute": 0, "second": 0, "milli_sec": 250}) == ITOW0 + 250
    # Sunday 00:00:00 UTC is 18 s into the GPS week.
    assert cfl.itow_from_utc({"year": 2026, "month": 9, "day": 6, "hour": 0,
                              "minute": 0, "second": 0, "milli_sec": 0}) == 18_000
    # Saturday 23:59:50 UTC wraps into the next week's first seconds.
    assert cfl.itow_from_utc({"year": 2026, "month": 9, "day": 12, "hour": 23,
                              "minute": 59, "second": 50, "milli_sec": 0}) == 8_000
    assert cfl.itow_from_utc({"year": 1980, "month": 1, "day": 6, "hour": 0,
                              "minute": 0, "second": 0}) is None
