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
        return f"{fmt.format(hi)}-{fmt.format(lo)} {unit}"
    return f"{fmt.format(lo)}-{fmt.format(hi)} {unit}"


# The summary table rounds. Every part that gates velocity brackets it within a
# few m/s of 515, and every altitude gate lands within a few hundred meters of a
# round figure, so quoting 510-517 / 514-516 / 514-518 / 510-524 in a comparison
# invites the reader to look for a difference between parts that the measurement
# does not support. The precise brackets stay in receivers.json and in each
# receiver's own section; this is the at-a-glance view.
def _round_to(x, step):
    return round(x / step) * step


def vel_cell(r):
    """Velocity gate, rounded, or a bound when the part has none.

    A receiver that never withholds is not a missing measurement, it is a
    result -- the Air530 held a fix to 900 m/s, 1.75x the export limit -- and
    printing "--" would read as untested. `velocity_gate_present: false` makes
    it say so.
    """
    if r.get("velocity_gate_present") is False:
        v = r.get("velocity_fix_max_mps")
        return f"none to {v:.0f} m/s" if v else "none observed"
    lo, hi = r.get("velocity_fix_max_mps"), r.get("velocity_blocked_min_mps")
    if lo is None or hi is None:
        return "--"
    return f"{_round_to((lo + hi) / 2.0, 5):.0f} m/s"


# Footnote markers, in the order they are first used. Kept as markers rather
# than parentheticals in the cell because the qualifications matter -- a ceiling
# that is not an export gate is a different kind of fact from one that is -- and
# a cell wide enough to say so inline pushes the table past a readable width.
FOOTNOTES = {
    "inverted": ("\u2020",
                 "Slow to close: this part held a fix 2-3 s past the limit on "
                 "both flights, about 400-600 m of overshoot above 80 km with "
                 "position still being published. The threshold itself is normal."),
    "not cocom": ("\u2021",
                  "Not an export gate. This ceiling sits below the COCOM "
                  "altitude, and the receiver stops publishing there for reasons "
                  "unrelated to export control."),
    "dyn model": ("\u00a7",
                  "The u-blox dynamic model's own altitude ceiling, not an export "
                  "gate. Airborne <4 g is specified at 50,000 m; no u-blox model "
                  "goes higher, so this part's export behavior above it cannot "
                  "be measured."),
}


def alt_cell(r):
    """Altitude gate, rounded, with a footnote marker when it needs qualifying."""
    lo, hi = r.get("altitude_fix_max_km"), r.get("altitude_blocked_min_km")
    if lo is None or hi is None:
        return r.get("altitude_note", "--"), None
    inverted = lo > hi
    txt = f"{_round_to((lo + hi) / 2.0, 1):.0f} km"
    # Normalized: the JSON is hand-edited and has carried both "not COCOM" and
    # "not cocom" for the same thing.
    cause = (r.get("altitude_gate_cause") or "cocom").strip().lower()
    key = cause if cause in FOOTNOTES else ("inverted" if inverted else None)
    if key:
        txt += " " + FOOTNOTES[key][0]
    return txt, key


def rows(d):
    out = []
    for r in d["receivers"]:
        out.append({
            "part": r["part"],
            "path": r["path"],
            "runs": r["runs"],
            "vel": vel_cell(r),
            "alt": alt_cell(r)[0],
            "comb": r["combination"],
            "rec": (f"{r['recovery_s'][0]:.1f}-{r['recovery_s'][1]:.1f} s"
                    if r.get("recovery_s") else "n/a"),
            "notes": r.get("notes", ""),
            "bands": r["bands"],
            "protocol": r["protocol"],
        })
    return out


# Bands and run counts were receiver spec, not measurement, and the "18 km gate"
# column read "none" for every part ever tested -- a whole column restating that
# something does not exist.
HEADS = [("part", "Receiver"), ("path", "Path"),
         ("vel", "Velocity gate"), ("alt", "Altitude gate"),
         ("comb", "Limits combined"), ("rec", "Re-open latency")]


def used_footnotes(d):
    """(marker, text) for the footnotes this table actually needs, in order."""
    seen, out = set(), []
    for r in d["receivers"]:
        key = alt_cell(r)[1]
        if key and key not in seen:
            seen.add(key)
            out.append(FOOTNOTES[key])
    return out


def markdown(d) -> str:
    rs = rows(d)
    out = ["| " + " | ".join(h for _, h in HEADS) + " |",
           "|" + "|".join("---" for _ in HEADS) + "|"]
    for r in rs:
        out.append("| " + " | ".join(str(r[k]) for k, _ in HEADS) + " |")
    out.append("")
    for mark, text in used_footnotes(d):
        out.append(f"{mark} {text}")
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
