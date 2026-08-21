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
    """(lo, hi] as a string, or a point estimate when the two coincide.

    The bracket can *invert* -- a value that still held a fix sitting above one
    that was withheld -- when the receiver is slow to close the gate. The F9P
    lags 2-3 s on the altitude limit, which at 200 m/s of climb overshoots by
    400-600 m, so on one flight it held a fix at 80.48 km and on another it was
    already blocked there. That is a latency, not a different threshold, and it
    is marked rather than silently printed backwards.
    """
    if lo is None or hi is None:
        return "--"
    if abs(hi - lo) < 1e-9:
        return f"~{fmt.format(hi)} {unit}"
    if lo > hi:
        return f"{fmt.format(hi)}-{fmt.format(lo)} {unit} \u2020"
    return f"{fmt.format(lo)}-{fmt.format(hi)} {unit}"


def alt_cell(r):
    """Altitude gate, with its cause when that is not the export limit.

    A receiver can stop publishing altitude for reasons that have nothing to do
    with COCOM -- the NEO-M8T's u-blox dynamic model caps at 50 km, and no model
    u-blox offers goes higher, so the part stops navigating well below the
    export threshold. That is still a real ceiling a flight computer will hit,
    so it is recorded rather than omitted; but it is labelled, because reading
    it as an export gate would be wrong.
    """
    txt = bracket(r.get("altitude_fix_max_km"), r.get("altitude_blocked_min_km"),
                  "km", "{:.2f}")
    cause = r.get("altitude_gate_cause")
    if cause and cause != "cocom" and txt != "--":
        txt += f" ({cause})"
    elif txt == "--":
        txt = r.get("altitude_note", "--")
    return txt


def rows(d):
    out = []
    for r in d["receivers"]:
        out.append({
            "part": r["part"],
            "path": r["path"],
            "runs": r["runs"],
            "vel": bracket(r["velocity_fix_max_mps"], r["velocity_blocked_min_mps"], "m/s"),
            "alt": alt_cell(r),
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
