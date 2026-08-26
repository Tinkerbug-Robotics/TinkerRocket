#!/usr/bin/env python3
"""Convert a rocket-computer console log into the rig's UBX capture format.

The FC's u-blox is not reachable directly: its firmware sets UART1 output to UBX
only and the rocket computer consumes it, putting nothing raw on USB. Rather
than clip a serial tap onto the receiver's TXD, the FC is built with
TR_GNSS_COCOM_DIAG=1, which logs fix state and per-satellite C/N0 to the console
that is already connected:

    I (25263) GNSS: [COCOM] P tow=24805 fix=0 ok=0 nsv=0 lat=0 lon=0 alt=0 vn=0 ve=0 vd=0
    I (25263) GNSS: [COCOM] S n=2 0:22:20:0:41 5:1:10:0:12

This turns those back into synthetic UBX-NAV-PVT and UBX-NAV-SAT frames written
as `TS U <hex>`, which is exactly what gnss_nmea_monitor captures from a real
tap. correlate.py, recovery.py, plot_flight.py and oscillation.py then work
unchanged, against the same classifier the conducted rig used.

Two ordering details matter and are handled here:

* NAV-SAT is emitted BEFORE its NAV-PVT, because correlate.py treats NAV-PVT as
  the epoch marker and closes the epoch on it -- the satellites for that epoch
  must already be in. The console logs them the other way round.
* The satellite field is `gnss:sv:cno:used:elev`, and the four-field form
  without elevation is still accepted: every capture archived before elevation
  was added is in that older form, and those get elevation 0.
* Timestamps come from the ESP log prefix, which is milliseconds since boot.
  That is a host clock, not a receiver clock, so it places epochs relative to
  each other but cannot anchor them to a scenario. Fix epochs carry GPS time in
  the PVT payload, which is what the analysis actually anchors on.
"""

from __future__ import annotations

import argparse
import math
import re
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ublox_binary import CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT   # noqa: E402

LINE = re.compile(r"\((\d+)\)\s+GNSS:\s+\[COCOM\]\s+([PS])\s+(.*)$")
KV = re.compile(r"(\w+)=(-?\d+)")


def nav_pvt(tow_ms, fix, ok, nsv, lat, lon, alt_mm, vn, ve, vd) -> bytes:
    """A 92-byte NAV-PVT payload carrying the fields the classifier reads."""
    return struct.pack(
        "<IHBBBBBBIiBBBBiiiiIIiiiiiIIH",
        tow_ms & 0xFFFFFFFF,
        1980, 1, 6, 0, 0, 0,      # UTC unresolved: valid flags are cleared below
        0x00,                     # valid = 0, so the analysis falls back to iTOW
        50, 0,
        fix, 0x01 if ok else 0x00, 0, nsv,
        lon, lat, alt_mm, alt_mm,
        1000, 1500,
        vn, ve, vd,
        int(math.hypot(vn, ve)), 0,
        300, 0, 180) + b"\x00" * 14


def nav_sat(sats) -> bytes:
    p = struct.pack("<IBBH", 0, 1, len(sats), 0)
    for gnss_id, svid, cno, used, elev in sats:
        # Azimuth is still not logged and nothing downstream reads it.
        p += struct.pack("<BBBb", gnss_id, svid, cno, elev)
        p += struct.pack("<hh", 0, 0)
        p += struct.pack("<I", (0x08 if used else 0) | 0x0800)
    return p


def blob(mid: int, payload: bytes) -> str:
    return (bytes([CLS_NAV, mid]) + payload).hex()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("console", type=Path, help="captured FC console log")
    ap.add_argument("-o", "--out", required=True, type=Path)
    args = ap.parse_args()

    out = []
    pending_p = None
    n_p = n_s = 0
    fixes = 0

    for raw in args.console.read_text(errors="replace").splitlines():
        m = LINE.search(raw)
        if not m:
            continue
        t = int(m.group(1)) / 1000.0
        kind, rest = m.group(2), m.group(3)

        if kind == "P":
            kv = {k: int(v) for k, v in KV.findall(rest)}
            pending_p = (t, kv)
            n_p += 1
            if kv.get("ok") and kv.get("fix", 0) >= 2:
                fixes += 1
            continue

        # S line: emit satellites first, then the PVT they belong to.
        # Two field counts are accepted on purpose. Firmware built before
        # elevation was added logs gnss:sv:cno:used, and every SAM-M10Q capture
        # already in results/ is in that older form -- refusing it would make
        # the archive unreadable to answer a question the archive cannot answer
        # anyway. Older captures get elevation 0, which is what they carried.
        sats = []
        for tok in rest.split():
            parts = tok.split(":")
            if len(parts) not in (4, 5):
                continue
            if not all(x.lstrip("-").isdigit() for x in parts):
                continue
            vals = [int(x) for x in parts]
            g, sv, cno, used = vals[0], vals[1], vals[2], vals[3]
            elev = vals[4] if len(parts) == 5 else 0
            sats.append((g, sv & 0xFF, max(0, min(255, cno)), used,
                         max(-128, min(127, elev))))
        n_s += 1
        out.append(f"{t:.3f} U {blob(MSG_NAV_SAT, nav_sat(sats))}")

        if pending_p is not None:
            pt, kv = pending_p
            out.append(f"{pt:.3f} U " + blob(MSG_NAV_PVT, nav_pvt(
                kv.get("tow", 0), kv.get("fix", 0), kv.get("ok", 0),
                kv.get("nsv", 0), kv.get("lat", 0), kv.get("lon", 0),
                kv.get("alt", 0), kv.get("vn", 0), kv.get("ve", 0),
                kv.get("vd", 0))))
            pending_p = None

    if not out:
        raise SystemExit(
            f"no [COCOM] lines in {args.console}.\nThe firmware must be built "
            "with -DTR_GNSS_COCOM_DIAG=1; a stock build logs nothing here.")

    args.out.write_text("\n".join(out) + "\n")
    print(f"{args.console.name} -> {args.out}")
    print(f"  {n_p} PVT, {n_s} SAT lines, {fixes} with a valid fix")
    if fixes == 0:
        print("  note: no epoch reported a usable fix, so any analysis of this\n"
              "        capture can only describe why not")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
