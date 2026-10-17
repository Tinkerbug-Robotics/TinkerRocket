#!/usr/bin/env python3
"""Pull the GNSS receiver's SBF blocks (GNSS_RTCM_MSG, 0x95) out of a V9 log.

The flight computer logs every SBF block the mosaic-G5 sends whose CRC checks,
byte for byte (MeasEpoch, MeasExtra and PVTGeodetic at 20 Hz, the receiver's
status once a second, the navigation messages as they come). With the GNSS log
setting at reduced (BLE cmd 79, which lets the IMU log at 8k) it keeps
MeasEpoch and MeasExtra once a second, so the summary's MeasEpoch rate reads
1 Hz; the receiver's own microSD card has all of them. Joined in log order
the blocks are a standard SBF file, so this writes exactly that:

    ./extract_gnss_sbf.py flight_20261020_143000.bin
        -> flight_20261020_143000.sbf   (Septentrio's RxTools read it; sbf2rin
                                         turns it into RINEX)

    ./extract_gnss_sbf.py flight.bin --times
        -> also flight.sbf_times.csv, one row per block: the FC time its last
           byte arrived, its name, block number and revision, the receiver's
           time stamp (TOW ms, WNc; empty where it says do-not-use) and its
           length -- for how long after its epoch the receiver sends each block

The summary counts the blocks by name and gives the span of the receiver's
time stamps, and MeasEpoch's epochs and rate over them.

Every block written passes its own CRC; a block missing a part, or failing its
CRC, is left out and counted. On the Beetle the same record carries the LC86G's
RTCM3 frames: extract_gnss_rtcm.py reads those. Block and field names are the
mosaic-G5's SBF reference guide's.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_flight_data_mini as pfd          # noqa: E402


def extract(bin_path: Path):
    """([(time_us, block)], stats) for every good SBF block in a log, in log order."""
    records, _stats, _config = pfd.parse_binary_file(str(bin_path))
    joined, dropped = pfd.join_gnss_rtcm(records["GNSS_RTCM"])
    raw = pfd.split_gnss_raw_frames(joined)
    return raw["sbf"], {"records": len(records["GNSS_RTCM"]), "frames": len(joined),
                        "incomplete": dropped, "bad": raw["bad"],
                        "rtcm3": len(raw["rtcm3"])}


def week_tow(gps_ms: int) -> str:
    """ms since the GPS epoch -> "week W TOW s.sss s"."""
    return f"week {gps_ms // pfd.SBF_WEEK_MS} TOW {gps_ms % pfd.SBF_WEEK_MS / 1000.0:.3f} s"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="V9 .bin flight log")
    ap.add_argument("-o", "--out", type=Path, help="output .sbf (default: beside the log)")
    ap.add_argument("--times", action="store_true",
                    help="also write each block's FC arrival and time stamp as .sbf_times.csv")
    args = ap.parse_args(argv)

    blocks, st = extract(args.log)
    out = args.out or args.log.with_suffix(".sbf")
    out.write_bytes(b"".join(b for _, b in blocks))

    names = Counter(pfd.sbf_block_name(b) for _, b in blocks)
    print(f"{args.log.name}: {st['records']} records -> {st['frames']} frames "
          f"({st['incomplete']} incomplete, {st['bad']} bad)")
    if st["rtcm3"]:
        print(f"  {st['rtcm3']} RTCM3 frames (LC86G), not SBF: see extract_gnss_rtcm.py")
    print(f"  wrote {out} ({out.stat().st_size} bytes, {len(blocks)} blocks)")
    print("  blocks: " + (", ".join(f"{n} x{c}" for n, c in
                                     sorted(names.items(), key=lambda x: (-x[1], x[0])))
                          or "none"))
    stamps = [g for g in (pfd.sbf_gps_ms(b) for _, b in blocks) if g is not None]
    if stamps:
        lo, hi = min(stamps), max(stamps)
        print(f"  receiver time {week_tow(lo)} to {week_tow(hi)} ({(hi - lo) / 1000.0:.2f} s)")
    epochs, span_s, rate_hz = pfd.sbf_epoch_rate(blocks)
    if rate_hz is None:
        print(f"  MeasEpoch: {epochs} epochs, too few for a rate")
    else:
        print(f"  MeasEpoch: {epochs} epochs over {span_s:.2f} s -> {rate_hz:.2f} Hz")
    if blocks:
        span = (blocks[-1][0] - blocks[0][0]) / 1e6
        print(f"  FC time span {span:.1f} s")

    if args.times:
        path = out.with_suffix(".sbf_times.csv")
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["fc_time_us", "block", "number", "revision", "tow_ms", "wnc", "length"])
            for t, b in blocks:
                tow, wnc = pfd.sbf_tow_wnc(b)
                w.writerow([t, pfd.sbf_block_name(b), pfd.sbf_block_number(b),
                            pfd.sbf_block_revision(b), "" if tow is None else tow,
                            "" if wnc is None else wnc, len(b)])
        print(f"  wrote {path} ({len(blocks)} block rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
