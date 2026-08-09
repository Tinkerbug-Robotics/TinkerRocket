#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_lora_link.py

Measures the real downlink packet loss of a flight by joining the two halves
of the record:

  * the rocket's flight log (.bin) — LORA_MSG (0xF1) records, one per frame the
    OC actually radiated, each carrying the `seq` that went on the air;
  * the base station's lora_*.csv — one row per frame it decoded, with `seq`,
    `rssi` and `snr`.

Loss = the seq values the rocket logged that never show up on the ground.  That
is the only honest number: the base station alone cannot distinguish "the
rocket skipped a slot" from "the packet did not make it", and the rocket alone
cannot tell you whether anything was heard.

Either side may be given on its own.  With only the base station, gaps in the
received seq run are *assumed* to be losses (the rocket is presumed to have
transmitted continuously) and the report says so — that assumption is exactly
what the 0xF1 records exist to remove.

Usage:
    analyze_lora_link.py --bin flight_YYYYmmdd_HHMMSS.bin \\
                         --bs  lora_YYYYmmdd_HHMMSS.csv
    analyze_lora_link.py --bs lora_*.csv          # ground side only
    analyze_lora_link.py --bin flight_*.bin       # rocket side only
"""

import argparse
import csv
import struct
import sys
from collections import Counter

# ── Wire format (see RocketComputerTypes.h) ─────────────────────────────────

PREAMBLE = b'\xAA\x55\xAA\x55'
MSG_LORA = 0xF1
MSG_ISM6HG256 = 0xA2          # carries FC time_us; dates neighbouring records
SIZE_OF_LORA_DATA = 65

# LoRaData prefix — the routing header plus the two fields worth reporting.
# Offsets are pinned by static_asserts in RocketComputerTypes.h.
FMT_LORA_HEAD = '<BBBHBB'     # network_id, rocket_id, next_channel_idx, seq,
                              # num_sats, pdop_u8
LORA_LOGGING_BIT = 0x80       # packed into the MSB of num_sats


def iter_frames(path):
    """Yield (type, payload) for every CRC-length-plausible frame in a .bin."""
    blob = open(path, 'rb').read()
    i = 0
    while True:
        p = blob.find(PREAMBLE, i)
        if p < 0:
            return
        j = p + len(PREAMBLE)
        if j + 3 >= len(blob):
            return
        mtype, length = blob[j], blob[j + 1]
        j += 2
        if j + length + 2 > len(blob):
            return
        yield mtype, blob[j:j + length]
        i = j + length + 2


def read_rocket_bin(path):
    """Extract the transmitted-seq record from a rocket flight log.

    Returns (rows, imu_frames) where rows is a list of dicts in log order.
    `t_s` is the FC timestamp of the most recent IMU frame — LoRaData has no
    time field of its own, and its position in the stream dates it to the IMU
    interval (~256 us), which beats a second clock domain.
    """
    rows, imu_frames, t_last = [], 0, None
    for mtype, payload in iter_frames(path):
        if mtype == MSG_ISM6HG256 and len(payload) >= 4:
            t_last = struct.unpack_from('<I', payload, 0)[0] / 1e6
            imu_frames += 1
        elif mtype == MSG_LORA:
            if len(payload) != SIZE_OF_LORA_DATA:
                print(f"  ! 0xF1 record with {len(payload)} B payload, expected "
                      f"{SIZE_OF_LORA_DATA} — LoRaData changed size and this "
                      f"parser was not swept (#227)", file=sys.stderr)
                continue
            nid, rid, nch, seq, sats, pdop = struct.unpack_from(FMT_LORA_HEAD, payload, 0)
            rows.append(dict(seq=seq, t_s=t_last, network_id=nid, rocket_id=rid,
                             next_ch=nch, num_sats=sats & ~LORA_LOGGING_BIT,
                             logging=bool(sats & LORA_LOGGING_BIT),
                             pdop=pdop / 10.0))
    return rows, imu_frames


def read_bs_csv(path):
    """Read a base-station lora_*.csv. Tolerates a truncated final row."""
    rows = []
    with open(path, newline='') as fh:
        for rec in csv.DictReader(fh):
            try:
                rows.append(dict(seq=int(rec['seq']),
                                 t_s=float(rec['time_ms']) / 1000.0,
                                 rssi=float(rec['rssi']),
                                 snr=float(rec['snr']),
                                 rocket_id=rec.get('rocket_id'),
                                 next_ch=rec.get('next_ch')))
            except (TypeError, ValueError, KeyError):
                continue          # truncated tail row, or a partial flush
    return rows


def pct(n, d):
    return f"{100.0 * n / d:.1f}%" if d else "n/a"


def describe(name, vals):
    if not vals:
        return f"  {name}: none"
    vals = sorted(vals)
    n = len(vals)
    q = lambda f: vals[min(n - 1, int(f * n))]
    return (f"  {name}: median {q(.5):.1f}  p10 {q(.1):.1f}  p90 {q(.9):.1f}  "
            f"min {vals[0]:.1f}  max {vals[-1]:.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bin', dest='binpath', help='rocket flight log (.bin)')
    ap.add_argument('--bs', dest='bspath', help='base-station lora_*.csv')
    args = ap.parse_args()
    if not args.binpath and not args.bspath:
        ap.error('give --bin, --bs, or both')

    tx, rx = [], []

    if args.binpath:
        tx, imu = read_rocket_bin(args.binpath)
        print(f"rocket log  {args.binpath}")
        if not tx:
            print(f"  no 0xF1 LORA_MSG records ({imu} IMU frames present).")
            print("  Firmware predating the 0xF1 restore does not log what it "
                  "transmitted — loss below is ground-side inference only.")
        else:
            spread = tx[-1]['seq'] - tx[0]['seq'] + 1
            print(f"  {len(tx)} transmitted frames, seq {tx[0]['seq']}..{tx[-1]['seq']}")
            if spread != len(tx):
                print(f"  ! {spread - len(tx)} seq values missing from the rocket's "
                      f"own log — ring overrun, not RF loss")
            ids = Counter((r['network_id'], r['rocket_id']) for r in tx)
            print(f"  network/rocket id: {dict(ids)}")
            hop = Counter(r['next_ch'] for r in tx)
            print(f"  next_channel_idx: {dict(hop)}"
                  + ("   (0xFF=255 => not hopping, fixed channel)" if 255 in hop else ""))
            if tx[0]['t_s'] is not None and tx[-1]['t_s'] is not None:
                print(f"  spans FC t = {tx[0]['t_s']:.3f}..{tx[-1]['t_s']:.3f} s")

    if args.bspath:
        rx = read_bs_csv(args.bspath)
        print(f"\nbase station  {args.bspath}")
        if not rx:
            print("  no decodable rows")
        else:
            print(f"  {len(rx)} received frames, seq {rx[0]['seq']}..{rx[-1]['seq']}, "
                  f"{(rx[-1]['t_s'] - rx[0]['t_s']) / 60:.1f} min")
            print(describe('rssi (dBm)', [r['rssi'] for r in rx]))
            print(describe('snr  (dB) ', [r['snr'] for r in rx]))

    print("\n" + "=" * 62)
    if tx and rx:
        sent = {r['seq'] for r in tx}
        heard = {r['seq'] for r in rx}
        lost = sent - heard
        extra = heard - sent          # BS heard a seq the rocket never logged
        print(f"TRUE downlink loss (rocket-logged seq vs base-station seq)")
        print(f"  transmitted {len(sent)}   received {len(sent & heard)}   "
              f"lost {len(lost)}  ({pct(len(lost), len(sent))})")
        if extra:
            print(f"  ! {len(extra)} received seq never logged as transmitted — "
                  f"overlapping sessions, a second rocket, or a dropped log record")
        if lost:
            runs, cur = [], None
            for s in sorted(lost):
                if cur and s == cur[1] + 1:
                    cur[1] = s
                else:
                    cur = [s, s]
                    runs.append(cur)
            worst = max(runs, key=lambda r: r[1] - r[0])
            print(f"  {len(runs)} loss bursts, longest {worst[1] - worst[0] + 1} "
                  f"consecutive frames (seq {worst[0]}..{worst[1]})")
    elif rx:
        heard = sorted(r['seq'] for r in rx)
        span = heard[-1] - heard[0] + 1
        print("Ground-side inference only (no 0xF1 records to compare against)")
        print(f"  seq run {heard[0]}..{heard[-1]} = {span} slots, "
              f"{len(heard)} decoded, {span - len(heard)} missing "
              f"({pct(span - len(heard), span)})")
        print("  Assumes the rocket transmitted every slot. Fly firmware that "
              "logs 0xF1 to replace this with a measurement.")
    elif tx:
        print("Rocket side only — what was transmitted is known, what was heard "
              "is not. Pair with the base station's lora_*.csv.")


if __name__ == '__main__':
    main()
