#!/usr/bin/env python3
"""Pull the GNSS receiver's raw RTCM3 frames (GNSS_RTCM_MSG, 0x95) out of a mini log.

The flight computer logs every RTCM3 frame the receiver sends, verbatim (on the
mini: the LC86G's MSM7 raw measurements, one epoch a second). Joined in log
order they are a standard RTCM3 stream, so this writes exactly that:

    ./extract_gnss_rtcm.py flight_20261012_143000.bin
        -> flight_20261012_143000.rtcm3   (any RTCM3 tool reads it; RTKLIB's
                                           convbin turns it into RINEX)

    ./extract_gnss_rtcm.py flight.bin --csv
        -> also flight.msm7.csv, one row per satellite-signal per MSM7 epoch:
           the frame's FC time, GPS time of week, constellation, satellite,
           signal, pseudorange, carrier phase, Doppler (phase-range rate),
           C/N0, lock time, half-cycle flag, GLONASS channel

The summary also names each broadcast-ephemeris message ($PAIR436,1: 1019 GPS,
1020 GLONASS, 1042 BeiDou, 1044 QZSS, 1046 Galileo) by satellite, issue of data
and reference time, with sqrt(A) as a sanity check on the decode.

Every frame written passes its own CRC-24Q; a frame missing a part, or failing
its CRC, is left out and counted. On the V9 the same record carries the
mosaic-G5's SBF blocks instead: those are not RTCM3, so they are left out and
counted on a line of their own, not as bad CRCs (extract_gnss_sbf.py writes
them). Field references are RTCM 10403.3 (DFnnn).
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_flight_data_mini as pfd          # noqa: E402

LIGHT_MS_M = 299792458.0 / 1000.0            # MSM ranges are in light-milliseconds

# MSM7 message type -> (UBX gnssId as GNSSSatBlock numbers them, name)
MSM7_TYPES = {1077: (0, "GPS"), 1087: (6, "GLONASS"), 1097: (2, "Galileo"),
              1117: (5, "QZSS"), 1127: (3, "BeiDou")}


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def frame_ok(frame: bytes) -> bool:
    """A whole RTCM3 frame whose length and CRC-24Q check."""
    if len(frame) < 6 or frame[0] != 0xD3 or frame[1] & 0xFC:
        return False
    if len(frame) != ((frame[1] & 0x03) << 8 | frame[2]) + 6:
        return False
    return crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big")


def message_type(frame: bytes) -> int:
    return (frame[3] << 4) | (frame[4] >> 4)


class _Bits:
    def __init__(self, data: bytes):
        self.v = int.from_bytes(data, "big")
        self.n = len(data) * 8
        self.p = 0

    def u(self, k: int) -> int:
        self.p += k
        return (self.v >> (self.n - self.p)) & ((1 << k) - 1)

    def s(self, k: int) -> int:
        x = self.u(k)
        return x - (1 << k) if x >> (k - 1) else x


def lock_time_ms(i: int) -> int:
    """DF407 lock-time indicator -> minimum lock time, ms."""
    if i < 64:
        return i
    if i >= 704:
        return 67108864
    b = (i - 64) // 32
    return 2 ** (b + 1) * (i - (64 + 32 * b)) + 2 ** (6 + b)


def decode_msm7(frame: bytes):
    """(constellation, GPS TOW ms or None, [cell dicts]) for an MSM7 frame, else None.

    Invalid fields come back as None. GLONASS's epoch is UTC(SU) and needs the
    leap seconds, so its GPS TOW is None; BeiDou's is shifted by its 14 s.
    """
    mt = message_type(frame)
    if mt not in MSM7_TYPES:
        return None
    gnss_id, name = MSM7_TYPES[mt]
    b = _Bits(frame[3:-3])
    b.u(12); b.u(12)
    epoch = b.u(30)
    b.u(1); b.u(3); b.u(7); b.u(2); b.u(2); b.u(1); b.u(3)
    sat_mask, sig_mask = b.u(64), b.u(32)
    sats = [i + 1 for i in range(64) if sat_mask >> (63 - i) & 1]
    sigs = [i + 1 for i in range(32) if sig_mask >> (31 - i) & 1]
    cells = [(s, g) for s in sats for g in sigs if b.u(1)]
    ns, nc = len(sats), len(cells)
    rough_ms = [b.u(8) for _ in range(ns)]
    ext = [b.u(4) for _ in range(ns)]
    rough_mod = [b.u(10) for _ in range(ns)]
    rough_rate = [b.s(14) for _ in range(ns)]
    fine_pr = [b.s(20) for _ in range(nc)]
    fine_cp = [b.s(24) for _ in range(nc)]
    lock = [b.u(10) for _ in range(nc)]
    half = [b.u(1) for _ in range(nc)]
    cnr = [b.u(10) for _ in range(nc)]
    fine_rate = [b.s(15) for _ in range(nc)]

    if gnss_id == 6:
        tow = None
    elif gnss_id == 3:
        tow = (epoch + 14000) % 604800000
    else:
        tow = epoch
    k = {s: i for i, s in enumerate(sats)}
    out = []
    for c, (s, g) in enumerate(cells):
        i = k[s]
        rough = rough_ms[i] + rough_mod[i] / 1024.0 if rough_ms[i] != 255 else None
        out.append({
            "gnss_id": gnss_id, "sv_id": s, "sig_id": g,
            "pseudorange_m": None if rough is None or fine_pr[c] == -524288
                             else (rough + fine_pr[c] * 2.0 ** -29) * LIGHT_MS_M,
            "phase_range_m": None if rough is None or fine_cp[c] == -8388608
                             else (rough + fine_cp[c] * 2.0 ** -31) * LIGHT_MS_M,
            "rate_mps": None if rough_rate[i] == -8192 or fine_rate[c] == -16384
                        else rough_rate[i] + fine_rate[c] * 1e-4,
            "cn0_dbhz": cnr[c] / 16.0,
            "lock_ms": lock_time_ms(lock[c]),
            "half_cycle": half[c],
            "ext_info": ext[i],
        })
    return name, tow, out


# Broadcast ephemeris ($PAIR436,1). Where each message keeps the fields that say
# WHICH ephemeris it is, as (bit offset, width) after the 12-bit message number:
# satellite, issue of data, reference time (and its unit, s), sqrt(A) (2^-19).
EPH_LAYOUT = {
    #        name       sv        iod        toe (bits, scale s)  sqrtA
    1019: ("GPS",     (12, 6), (48, 8),  (288, 16, 16.0),  (256, 32)),   # DF009, DF071 IODE, DF093, DF092
    1044: ("QZSS",    (12, 4), (78, 8),  (246, 16, 16.0),  (214, 32)),   # DF429, DF434 IODE, DF442, DF441
    1046: ("Galileo", (12, 6), (30, 10), (294, 14, 60.0),  (262, 32)),   # DF252, DF290 IODnav, DF304, DF303
    1042: ("BeiDou",  (12, 6), (49, 5),  (299, 17, 8.0),   (267, 32)),   # DF488, DF492 AODE, DF505, DF504
    1020: ("GLONASS", (12, 6), None,     (41, 7, 900.0),   None),        # DF038 slot, DF110 tb (15 min)
}


def decode_eph(frame: bytes):
    """{constellation, sv, iod, toe_s, sqrt_a} for an ephemeris frame, else None.

    toe_s is the ephemeris reference time in the constellation's own time of
    week (GLONASS: tb, time of day, Moscow time); sqrt_a is None for GLONASS,
    whose message carries positions, not Keplerian elements.
    """
    mt = message_type(frame)
    if mt not in EPH_LAYOUT:
        return None
    name, sv, iod, toe, sqa = EPH_LAYOUT[mt]
    v = int.from_bytes(frame[3:-3], "big")
    n = (len(frame) - 6) * 8

    def field(off, width):
        return (v >> (n - off - width)) & ((1 << width) - 1)

    return {"constellation": name, "sv": field(*sv),
            "iod": field(*iod) if iod else None,
            "toe_s": field(toe[0], toe[1]) * toe[2],
            "sqrt_a": field(*sqa) * 2.0 ** -19 if sqa else None}


def extract(bin_path: Path):
    """([(time_us, frame)], stats) for every good frame in a log, in log order."""
    records, _stats, _config = pfd.parse_binary_file(str(bin_path))
    joined, dropped = pfd.join_gnss_rtcm(records["GNSS_RTCM"])
    good = [(t, f) for t, f in joined if frame_ok(f)]
    sbf = sum(1 for _, f in joined if f[:2] == pfd.SBF_SYNC)   # a mosaic-G5's, not bad RTCM3
    return good, {"records": len(records["GNSS_RTCM"]), "frames": len(joined),
                  "dropped_parts": dropped, "bad_crc": len(joined) - len(good) - sbf,
                  "sbf_blocks": sbf}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="mini .bin flight log")
    ap.add_argument("-o", "--out", type=Path, help="output .rtcm3 (default: beside the log)")
    ap.add_argument("--csv", action="store_true", help="also write the decoded MSM7 as .msm7.csv")
    args = ap.parse_args(argv)

    frames, st = extract(args.log)
    out = args.out or args.log.with_suffix(".rtcm3")
    out.write_bytes(b"".join(f for _, f in frames))

    types = Counter(message_type(f) for _, f in frames)
    print(f"{args.log.name}: {st['records']} records -> {st['frames']} frames "
          f"({st['dropped_parts']} incomplete, {st['bad_crc']} bad CRC)")
    if st["sbf_blocks"]:
        print(f"  {st['sbf_blocks']} SBF blocks (mosaic-G5), not RTCM3: see extract_gnss_sbf.py")
    print(f"  wrote {out} ({out.stat().st_size} bytes)")
    print("  message types: " + (", ".join(f"{t} x{n}" for t, n in sorted(types.items())) or "none"))
    if frames:
        span = (frames[-1][0] - frames[0][0]) / 1e6
        print(f"  FC time span {span:.1f} s")
    seen = {}
    for t, f in frames:
        e = decode_eph(f)
        if e is not None:
            seen.setdefault((e["constellation"], e["sv"]), []).append((t, e))
    if seen:
        print("  ephemeris (satellite: messages, median repeat, issue of data, toe, sqrt(A)):")
        for (name, sv), msgs in sorted(seen.items()):
            ts = [t for t, _ in msgs]
            gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
            rep = f"{gaps[len(gaps) // 2] / 1e6:5.1f} s" if gaps else "    -  "
            last = msgs[-1][1]
            sqa = f"{last['sqrt_a']:.3f}" if last["sqrt_a"] is not None else "-"
            print(f"    {name:<8s}{sv:3d}: {len(msgs):3d}, {rep}, iod {last['iod']}, "
                  f"toe {last['toe_s']:.0f} s, sqrtA {sqa}")

    if args.csv:
        path = out.with_suffix(".msm7.csv")
        n = 0
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["time_us", "gps_tow_ms", "constellation", "gnss_id", "sv_id", "sig_id",
                        "pseudorange_m", "phase_range_m", "rate_mps", "cn0_dbhz",
                        "lock_ms", "half_cycle", "ext_info"])
            for t, f in frames:
                d = decode_msm7(f)
                if d is None:
                    continue
                name, tow, cells = d
                for c in cells:
                    w.writerow([t, "" if tow is None else tow, name, c["gnss_id"], c["sv_id"],
                                c["sig_id"],
                                *("" if c[k] is None else f"{c[k]:.4f}"
                                  for k in ("pseudorange_m", "phase_range_m", "rate_mps")),
                                c["cn0_dbhz"], c["lock_ms"], c["half_cycle"], c["ext_info"]])
                    n += 1
        print(f"  wrote {path} ({n} satellite-signal rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
