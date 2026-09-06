#!/usr/bin/env python3
"""Convert a rocket-computer flight log (.bin) into the rig's UBX capture format.

Every flight log written since the GNSS_SAT_MSG record (type 0x90) carries a
per-satellite report at every GNSS epoch -- gnssId, svId, C/N0, elevation,
azimuth, the used flag and the receiver's tracking-quality indicator -- beside
the fix (GNSS, type 0xA1) it belongs to.  This turns both back into synthetic
UBX-NAV-PVT and UBX-NAV-SAT frames written as `TS U <hex>`, exactly what
gnss_nmea_monitor captures from a real tap and what cocom_fcdiag.py makes from
a console log.  correlate.py, recovery.py, plot_flight.py and oscillation.py
then analyse a real boost with the same code that analysed the injected one.

Pairing.  A NAV-SAT must precede the NAV-PVT of its epoch, because correlate.py
treats NAV-PVT as the epoch marker and closes the epoch on it.  The two records
are paired by GPS time of week: the satellite record carries iTOW verbatim,
and the fix record's iTOW is rebuilt from its UTC fields (GPS runs 18 s ahead
of UTC, and has since 2017-01-01).  A fix whose UTC is not yet valid (year
before 2017, i.e. no time solution) falls back to the nearest satellite record
by flight-computer clock, within half an epoch.

Timestamps are the flight computer's microsecond clock, as seconds.  Like the
console converter's ESP-log prefix, that places epochs relative to each other;
the analysis anchors on iTOW inside the PVT payload.

    ./cocom_flightlog.py flight_20260905_120000.bin -o results/flight_20260905.txt
    ./plot_flight.py results/flight_20260905.txt
"""

from __future__ import annotations

import argparse
import datetime as dt
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))               # ublox_binary
sys.path.insert(0, str(HERE))                      # cocom_fcdiag
sys.path.insert(0, str(REPO / "Data_Analysis"))    # plot_flight_data_mini

from ublox_binary import MSG_NAV_PVT, MSG_NAV_SAT          # noqa: E402
from cocom_fcdiag import nav_pvt, blob                     # noqa: E402
from plot_flight_data_mini import parse_binary_file        # noqa: E402

GPS_UTC_LEAP_S = 18          # since 2017-01-01; no leap second has been added since
WEEK_MS = 7 * 86400 * 1000
NAV_RATE_HZ = 18             # config::GNSS_UPDATE_RATE on the rocket computer


def itow_from_utc(rec: dict) -> int | None:
    """GPS time of week (ms) from a GNSS record's UTC fields, or None."""
    try:
        if rec["year"] < 2017:
            return None
        t = dt.datetime(rec["year"], rec["month"], rec["day"],
                        rec["hour"], rec["minute"], rec["second"])
    except (KeyError, ValueError):
        return None
    # GPS week starts Sunday 00:00.  Python: Monday=0 .. Sunday=6.
    dow = (t.weekday() + 1) % 7
    sec = dow * 86400 + t.hour * 3600 + t.minute * 60 + t.second + GPS_UTC_LEAP_S
    return (sec * 1000 + int(rec.get("milli_sec", 0))) % WEEK_MS


def nav_sat(itow_ms: int, sats: list[dict]) -> bytes:
    """A NAV-SAT payload carrying every field the rig's parser reads."""
    p = struct.pack("<IBBH", itow_ms & 0xFFFFFFFF, 1, len(sats), 0)
    for s in sats:
        flags = ((s["quality"] & 0x07)
                 | (0x08 if s["used"] else 0)
                 | ((s["health"] & 0x03) << 4)
                 | (0x0800 if s["eph"] else 0)
                 | (0x1000 if s["alm"] else 0))
        p += struct.pack("<BBBb", s["gnss_id"], s["sv_id"] & 0xFF,
                         max(0, min(255, s["cno_dbhz"])),
                         max(-128, min(127, s["elev_deg"])))
        p += struct.pack("<hh", s["azim_deg"], 0)      # prRes is not logged
        p += struct.pack("<I", flags)
    return p


def pvt_from_gnss(rec: dict, itow_ms: int) -> bytes:
    """A NAV-PVT payload from the flight log's fix summary.

    fix_mode is already zeroed by the firmware when the receiver clears
    gnssFixOK, so `ok` follows fix_mode >= 2 rather than a separate flag.
    Velocities: the log stores ENU in m/s; NAV-PVT wants NED in mm/s.
    """
    fix = int(rec["fix_mode"])
    ok = 1 if fix >= 2 else 0
    return nav_pvt(itow_ms, fix, ok, int(rec["num_sats"]),
                   int(round(rec["lat"] * 1e7)), int(round(rec["lon"] * 1e7)),
                   int(round(rec["alt_m"] * 1000)),
                   int(round(rec["vel_n"] * 1000)), int(round(rec["vel_e"] * 1000)),
                   int(round(-rec["vel_u"] * 1000)))


def convert(records: dict) -> tuple[list[str], dict]:
    gnss = sorted(records.get("GNSS", []), key=lambda r: r["time_us"])
    sat = sorted(records.get("GNSS_SAT", []), key=lambda r: r["time_us"])

    by_itow: dict[int, dict] = {}
    for s in sat:
        by_itow.setdefault(s["itow_ms"], s)   # first wins; duplicates are DMA replays

    half_epoch_us = int(0.5e6 / NAV_RATE_HZ)
    # (epoch time, order within the epoch, line).  A paired NAV-SAT is keyed
    # on ITS PVT's time with order 0, the PVT order 1, so the pair stays
    # adjacent and SAT-first no matter which the receiver emitted first --
    # NAV-SAT normally lands a few ms AFTER NAV-PVT on the wire, and a plain
    # sort by timestamp would put it after the epoch marker that closes it.
    events: list[tuple[int, int, str]] = []
    used_sat_ids: set[int] = set()
    n_paired_itow = n_paired_time = n_unpaired_pvt = 0
    fixes = 0
    j = 0   # cursor into sat for the time-based fallback

    for g in gnss:
        itow = itow_from_utc(g)
        pair = by_itow.get(itow) if itow is not None else None
        if pair is not None:
            n_paired_itow += 1
        else:
            # Nearest satellite record by FC clock, within half an epoch.
            while j + 1 < len(sat) and sat[j + 1]["time_us"] <= g["time_us"]:
                j += 1
            best = None
            for k in (j, j + 1):
                if 0 <= k < len(sat) and id(sat[k]) not in used_sat_ids:
                    d = abs(sat[k]["time_us"] - g["time_us"])
                    if d <= half_epoch_us and (best is None or d < best[0]):
                        best = (d, sat[k])
            if best is not None:
                pair = best[1]
                n_paired_time += 1
                if itow is None:
                    itow = pair["itow_ms"]
        if itow is None:
            itow = 0
        if pair is None:
            n_unpaired_pvt += 1

        if pair is not None and id(pair) not in used_sat_ids:
            used_sat_ids.add(id(pair))
            events.append((g["time_us"], 0,
                           f"{pair['time_us'] / 1e6:.3f} U "
                           f"{blob(MSG_NAV_SAT, nav_sat(pair['itow_ms'], pair['sats']))}"))
        events.append((g["time_us"], 1,
                       f"{g['time_us'] / 1e6:.3f} U {blob(MSG_NAV_PVT, pvt_from_gnss(g, itow))}"))
        if g["fix_mode"] >= 2:
            fixes += 1

    # Satellite records with no fix record of their own still describe the
    # sky; emit them in time order so the rig sees the tracking table, and let
    # the analysis fold them into the next closed epoch.
    orphans = [s for s in sat if id(s) not in used_sat_ids]
    for s in orphans:
        events.append((s["time_us"], 0,
                       f"{s['time_us'] / 1e6:.3f} U "
                       f"{blob(MSG_NAV_SAT, nav_sat(s['itow_ms'], s['sats']))}"))
    events.sort(key=lambda e: (e[0], e[1]))
    out = [e[2] for e in events]

    stats = {
        "pvt": len(gnss), "sat": len(sat), "fixes": fixes,
        "paired_by_itow": n_paired_itow, "paired_by_time": n_paired_time,
        "unpaired_pvt": n_unpaired_pvt, "orphan_sat": len(orphans),
    }
    return out, stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("binfile", type=Path, help="rocket computer flight log (.bin)")
    ap.add_argument("-o", "--out", required=True, type=Path)
    args = ap.parse_args()

    records, _stats, _config = parse_binary_file(str(args.binfile))
    out, st = convert(records)

    if not records.get("GNSS_SAT"):
        raise SystemExit(
            f"no GNSS_SAT (0x90) records in {args.binfile}.\nThe flight computer "
            "must run firmware with the per-satellite record; older logs carry "
            "only the fix summary, which cannot answer a per-satellite question.")

    args.out.write_text("\n".join(out) + "\n")
    print(f"{args.binfile.name} -> {args.out}")
    print(f"  {st['pvt']} PVT, {st['sat']} SAT records, {st['fixes']} with a valid fix")
    print(f"  paired by iTOW {st['paired_by_itow']}, by FC clock {st['paired_by_time']}, "
          f"PVT without SAT {st['unpaired_pvt']}, SAT without PVT {st['orphan_sat']}")
    if st["paired_by_itow"] == 0 and st["paired_by_time"] == 0:
        print("  note: no epoch paired at all -- check the log actually has both "
              "record types over the same interval")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
