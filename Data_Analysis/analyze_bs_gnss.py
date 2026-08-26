#!/usr/bin/env python3
"""
Quantify 'bad' GNSS packets in a base-station CSV log.

The base station logs one row per *received* LoRa packet:
  time_ms,state,num_sats,pdop,lat,lon,alt_m,h_acc,...,rssi,snr,next_ch,rx_freq_mhz,seq,gap,event

Position is blanked to NaN by the BS when num_sats==0 or ECEF is all-zero
(base_station/main.cpp ~3242, #95). This script measures how often that
happens and tries to explain it (pre-fix? corrupt link? anomaly?).

Usage: python3 analyze_bs_gnss.py <base_station_log.csv>
"""
import sys, csv
from collections import Counter

def is_blank(v):
    # #850: rows come from bs_log now, which yields TYPED values (floats, ints)
    # rather than the strings a csv.DictReader produced. A float NaN is the
    # binary format's way of saying "no fix", so it has to be recognised here
    # as well as the CSV spellings.
    if v is None: return True
    if isinstance(v, float):
        return v != v or v in (float("inf"), float("-inf"))
    if not isinstance(v, str):
        return False
    s = v.strip().lower()
    return s in ("", "nan", "-nan", "inf", "-inf")

def fnum(v):
    try: return float(v)
    except (TypeError, ValueError): return None

def main(path):
    # #850: base-station logs are BINARY now. Read through bs_log, which detects
    # the format by content and still opens the legacy CSVs — a DictReader
    # pointed at a .bin yields garbage rows rather than an error, so this had to
    # move rather than merely tolerate the new extension.
    import pathlib
    import sys as _sys
    _root = str(pathlib.Path(__file__).resolve().parent)
    if _root not in _sys.path:
        _sys.path.insert(0, _root)
    import bs_log

    parsed, _events, _fmt = bs_log.read_bs_log(path)
    rows = []
    for row in parsed:
        # Skip EVENT rows (no telemetry payload) and malformed rows. Events come
        # back on their own channel from bs_log, so only the blank guard is left.
        if is_blank(row.get("num_sats")):
            continue
        rows.append(row)

    n = len(rows)
    if n == 0:
        print("No telemetry rows found."); return

    sats = [int(float(x["num_sats"])) for x in rows]
    lat_blank = [is_blank(x.get("lat")) for x in rows]
    rssi = [fnum(x.get("rssi")) for x in rows]

    zero_sat   = sum(1 for s in sats if s == 0)
    pos_blank  = sum(1 for b in lat_blank if b)
    # The anomaly to watch for: position blanked while sats look healthy.
    anomaly = [(i, sats[i]) for i in range(n) if lat_blank[i] and sats[i] > 0]
    # Expected: position blanked because there genuinely were 0 sats.
    blank_zerosat = sum(1 for i in range(n) if lat_blank[i] and sats[i] == 0)

    print(f"file: {path}")
    print(f"total telemetry packets: {n}")
    print()
    print("=== num_sats ===")
    print(f"  packets with num_sats == 0 : {zero_sat:5d}  ({100*zero_sat/n:.2f}%)")
    print(f"  min / median / max sats    : {min(sats)} / {sorted(sats)[n//2]} / {max(sats)}")
    hist = Counter(sats)
    print("  histogram (sats: count):")
    for s in sorted(hist):
        bar = "#" * min(60, hist[s])
        print(f"    {s:3d}: {hist[s]:5d} {bar}")
    print()
    print("=== blanked position (lat == nan) ===")
    print(f"  total blanked              : {pos_blank:5d}  ({100*pos_blank/n:.2f}%)")
    print(f"  blanked WITH 0 sats (exp.) : {blank_zerosat:5d}")
    print(f"  blanked WITH >0 sats (!!)  : {len(anomaly):5d}   <-- anomaly: sats look fine but no position")
    if anomaly:
        print(f"     example (row#, sats): {anomaly[:10]}")
    print()

    # Where do the bad packets sit? Front-loaded (pre-fix warmup) or scattered?
    bad_idx = [i for i in range(n) if lat_blank[i]]
    if bad_idx:
        first_good = next((i for i in range(n) if not lat_blank[i]), None)
        after_first_good = sum(1 for i in bad_idx if first_good is not None and i > first_good)
        print("=== distribution ===")
        print(f"  first valid-position packet at row: {first_good}")
        print(f"  blanked packets AFTER first good fix: {after_first_good}  "
              f"(pre-fix warmup explains {pos_blank - after_first_good})")
        good_rssi = [rssi[i] for i in range(n) if not lat_blank[i] and rssi[i] is not None]
        badf_rssi = [rssi[i] for i in bad_idx if first_good is not None and i > first_good and rssi[i] is not None]
        if good_rssi and badf_rssi:
            print(f"  mean RSSI  good={sum(good_rssi)/len(good_rssi):.1f}  "
                  f"bad-after-fix={sum(badf_rssi)/len(badf_rssi):.1f}  "
                  f"(much weaker bad => LoRa corruption)")
    print()

    # Dropped LoRa packets via sequence gaps (separate from blanked position).
    seqs = [int(float(x["seq"])) for x in rows if not is_blank(x.get("seq"))]
    if len(seqs) > 1:
        drops = sum(max(0, seqs[i+1]-seqs[i]-1) for i in range(len(seqs)-1)
                    if seqs[i+1] >= seqs[i])
        print("=== LoRa link ===")
        print(f"  seq range {min(seqs)}..{max(seqs)}, received {len(seqs)}, "
              f"missing (seq gaps) ~{drops}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1])
