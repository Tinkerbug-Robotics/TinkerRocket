#!/usr/bin/env python3
"""Render the receiver comparison from results/receivers.json.

The comparison exists to answer one question across parts: does this receiver
enforce the COCOM limits the same way, and where are its thresholds? Keeping the
measured numbers in JSON rather than in prose means adding a receiver is a data
edit, and means the report and results/README.md cannot drift apart.

Brackets are reported as (highest value that still held a fix, lowest value that
was withheld]. That form is deliberate: it tightens automatically as runs
accumulate, and it never claims more resolution than the 1 Hz navigation rate
supports -- a 15 g boost covers 118 m/s between epochs, so a single fast run can
only ever give a coarse answer no matter how carefully it is flown.

    ./receiver_table.py            # markdown, for results/README.md
    ./receiver_table.py --html     # the block report.html embeds
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = HERE / "results" / "receivers.json"


def bracket(lo, hi, unit, fmt="{:.0f}"):
    """(lo, hi] as a string, or a point estimate when the two coincide."""
    if lo is None or hi is None:
        return "--"
    if abs(hi - lo) < 1e-9:
        return f"~{fmt.format(hi)} {unit}"
    return f"{fmt.format(lo)}-{fmt.format(hi)} {unit}"


def rows(d):
    out = []
    for r in d["receivers"]:
        out.append({
            "part": r["part"],
            "path": r["path"],
            "runs": r["runs"],
            "vel": bracket(r["velocity_fix_max_mps"], r["velocity_blocked_min_mps"], "m/s"),
            "alt": bracket(r["altitude_fix_max_km"], r["altitude_blocked_min_km"], "km", "{:.2f}"),
            "g18": "none" if not r["gate_18km"] else "yes",
            "comb": r["combination"],
            "rec": f"{r['recovery_s'][0]:.1f}-{r['recovery_s'][1]:.1f} s",
            "sats": f"{r['sats_min']} / {r['sats_median']}",
            "notes": r.get("notes", ""),
            "bands": r["bands"],
            "protocol": r["protocol"],
        })
    return out


HEADS = [("part", "Receiver"), ("bands", "Bands"), ("path", "Path"),
         ("runs", "Runs"), ("vel", "Velocity gate"), ("alt", "Altitude gate"),
         ("g18", "18 km gate"), ("comb", "Limits combined"),
         ("rec", "Re-open latency"), ("sats", "Sats min / median")]


def markdown(d) -> str:
    rs = rows(d)
    out = ["| " + " | ".join(h for _, h in HEADS) + " |",
           "|" + "|".join("---" for _ in HEADS) + "|"]
    for r in rs:
        out.append("| " + " | ".join(str(r[k]) for k, _ in HEADS) + " |")
    out.append("")
    for r, src in zip(rs, d["receivers"]):
        if r["notes"]:
            out.append(f"**{r['part']}** ({src['date']}, {src['rf']}): {r['notes']}")
            out.append("")
    return "\n".join(out)


def html(d) -> str:
    rs = rows(d)
    o = ['<table class="cmp">', "  <thead><tr>"]
    for _, h in HEADS:
        o.append(f"    <th>{h}</th>")
    o.append("  </tr></thead>")
    o.append("  <tbody>")
    for r in rs:
        o.append("    <tr>")
        for k, _ in HEADS:
            cls = ' class="part"' if k == "part" else ""
            o.append(f"      <td{cls}>{r[k]}</td>")
        o.append("    </tr>")
    o.append("  </tbody>")
    o.append("</table>")
    return "\n".join(o)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--html", action="store_true")
    args = ap.parse_args()
    d = json.loads(DATA.read_text())
    print(html(d) if args.html else markdown(d))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
