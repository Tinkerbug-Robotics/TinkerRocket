#!/usr/bin/env python3
"""Convert a RINEX 3 mixed navigation file to the RINEX 2 GPS nav gps-sdr-sim wants.

Needed because the ephemeris sources that serve without an account -- BKG, IGN --
publish RINEX 3.0x MIXED (`BRDC00WRD_R_*_MN.rnx`), while gps-sdr-sim's reader is
strictly RINEX 2: it slices fixed columns and has no version handling at all.
NASA CDDIS still offers `brdc<DOY>0.<YY>n` in v2, but only behind an Earthdata
login, which is a poor dependency for a bench procedure.

Two things this has to get right:

* **Column positions.** gps-sdr-sim reads by offset, not by whitespace. The epoch
  line puts PRN at 0, the two-digit year at 3, and the three clock terms at 22,
  41 and 60; continuation lines carry four values at 3, 22, 41 and 60. A field
  one column off is silently parsed as a different number.
* **Record order.** gps-sdr-sim walks records in file order and starts a new
  ephemeris set whenever the epoch jumps more than an hour ahead of the previous
  one. RINEX 3 files are grouped by satellite, so feeding them through unsorted
  manufactures a new "set" at every satellite boundary and scrambles the
  constellation. Records are therefore re-sorted by epoch, then PRN.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ORBIT_LINES = 7  # a GPS nav record is one epoch line plus seven orbit lines


def d19(text: str) -> str:
    """One RINEX 2 nav field: 19 columns, D exponent designator."""
    try:
        v = float(text.replace("D", "E").replace("d", "e"))
    except ValueError:
        v = 0.0
    return f"{v: .12E}".replace("E", "D")


def parse(path: Path):
    """Yield (epoch_key, prn, [epoch_fields], [[orbit fields] * 7])."""
    lines = path.read_text(errors="replace").splitlines()

    try:
        end = next(i for i, l in enumerate(lines) if l[60:73].strip() == "END OF HEADER")
    except StopIteration:
        raise SystemExit(f"{path}: no END OF HEADER -- is this a RINEX file?")

    header, body, i = lines[:end + 1], lines[end + 1:], 0

    while i < len(body):
        line = body[i]
        if not line.startswith("G"):
            i += 1
            continue
        if i + ORBIT_LINES >= len(body):
            break

        prn = int(line[1:3])
        # RINEX 3 epoch line: Gnn yyyy mm dd hh mm ss then three 19-column values.
        head = line[4:23].split()
        if len(head) < 6:
            i += 1
            continue
        y, m, d, hh, mm, ss = (int(float(x)) for x in head[:6])
        clock = [line[23:42], line[42:61], line[61:80]]
        orbits = [[body[i + k][4:23], body[i + k][23:42],
                   body[i + k][42:61], body[i + k][61:80]]
                  for k in range(1, ORBIT_LINES + 1)]

        yield (y, m, d, hh, mm, ss), prn, (y, m, d, hh, mm, ss, clock), orbits
        i += ORBIT_LINES + 1

    if not header:
        raise SystemExit(f"{path}: empty header")


def leap_seconds(path: Path):
    for line in path.read_text(errors="replace").splitlines():
        if line[60:72].strip() == "LEAP SECONDS":
            try:
                return int(line[:6])
            except ValueError:
                return None
        if line[60:73].strip() == "END OF HEADER":
            break
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="RINEX 3 navigation file (.rnx)")
    ap.add_argument("-o", "--output", type=Path, required=True,
                    help="RINEX 2 output, conventionally brdc<DOY>0.<YY>n")
    args = ap.parse_args()

    records = sorted(parse(args.input), key=lambda r: (r[0], r[1]))
    if not records:
        raise SystemExit(f"{args.input}: no GPS (G) navigation records found. "
                         "A mixed file with only GLONASS/Galileo/BeiDou is no "
                         "use here -- gps-sdr-sim simulates GPS L1 only.")

    leap = leap_seconds(args.input)
    out = ["     2.11           N: GPS NAV DATA                         "
           "RINEX VERSION / TYPE",
           "rinex3to2.py        gnss-cocom          "
           "                    PGM / RUN BY / DATE"]
    if leap is not None:
        out.append(f"{leap:6d}{'':54}LEAP SECONDS")
    out.append(f"{'':60}END OF HEADER")

    for _key, prn, (y, m, d, hh, mm, ss, clock), orbits in records:
        out.append(f"{prn:2d} {y % 100:02d} {m:2d} {d:2d} {hh:2d} {mm:2d}"
                   f"{ss:5.1f}" + "".join(d19(c) for c in clock))
        for fields in orbits:
            out.append("   " + "".join(d19(f) for f in fields))

    args.output.write_text("\n".join(out) + "\n")

    prns = sorted({r[1] for r in records})
    epochs = sorted({r[0] for r in records})
    print(f"{args.input.name} -> {args.output}")
    print(f"  {len(records)} GPS records, {len(prns)} satellites, "
          f"{len(epochs)} epochs")
    print(f"  first {epochs[0][0]}-{epochs[0][1]:02d}-{epochs[0][2]:02d} "
          f"{epochs[0][3]:02d}:{epochs[0][4]:02d}  "
          f"last {epochs[-1][3]:02d}:{epochs[-1][4]:02d}")
    if leap is None:
        print("  note: no LEAP SECONDS in the source header")
    return 0


if __name__ == "__main__":
    sys.exit(main())
