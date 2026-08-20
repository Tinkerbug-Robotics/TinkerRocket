#!/usr/bin/env python3
"""Plot a flight against what the receiver did: SVG, for the report.

Altitude and speed against trajectory time, with the COCOM-exceeded spans shaded
and the receiver's lock state drawn as a strip underneath. The point of putting
all three on one time axis is that the interesting question -- did the fix go
away because the gate shut, or because the signal did -- is answered by whether
the lock strip changes colour at a shaded edge or somewhere in the middle.

Colours come from CSS custom properties so the figure follows the report's
light/dark themes; each has a literal fallback so the file also stands alone.

Usage:
  ./plot_flight.py -s scenarios/gentle_alt.json -t 2026/08/19,22:30:00 \
      captures/gentle_alt_a1.log -o gentle_alt.svg
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from correlate import Truth, collect, parse_start, trajectory_time  # noqa: E402

W, H = 900, 420
PAD_L, PAD_R, PAD_T, PAD_B = 62, 18, 26, 54
STRIP_H = 16
GAP = 26

VERDICT_FILL = {"FIX": "var(--fix, #0E7C66)",
                "BLOCKED": "var(--blocked, #A2660A)",
                "NO_LOCK": "var(--nolock, #9B3535)"}


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def build_svg(meta, rows, truth):
    dur = meta["duration_s"]
    alt_max = max(s["alt_m"] for s in truth) / 1000.0
    spd_max = max(s["speed_mps"] for s in truth)
    alt_top = max(10.0, alt_max * 1.12)
    spd_top = max(100.0, spd_max * 1.12)

    plot_w = W - PAD_L - PAD_R
    panel_h = (H - PAD_T - PAD_B - STRIP_H - 2 * GAP) / 2
    y0_alt = PAD_T
    y0_spd = PAD_T + panel_h + GAP
    y0_strip = y0_spd + panel_h + GAP

    def x(t):
        return PAD_L + plot_w * (t / dur)

    def y_alt(a_km):
        return y0_alt + panel_h * (1 - a_km / alt_top)

    def y_spd(v):
        return y0_spd + panel_h * (1 - v / spd_top)

    out = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" '
           f'role="img" aria-label="Altitude and speed against time for the '
           f'{esc(meta["scenario"])} flight, with COCOM-exceeded spans shaded '
           f'and the receiver lock state below.">',
           '<style>'
           '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
           '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
           '.lbl{font-family:var(--f-mono,monospace);font-size:9px;'
           'fill:var(--ink-3,#79808F)}'
           '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;'
           'font-weight:600;fill:var(--ink-2,#4A5261)}'
           '.trace{fill:none;stroke-width:1.8;stroke-linejoin:round}'
           '</style>']

    # limit-exceeded shading, drawn behind everything
    for wins, fill, lab in ((meta.get("velocity_windows") or [],
                             "var(--nolock, #9B3535)", "&gt;515 m/s"),
                            (meta.get("altitude_80km_windows") or [],
                             "var(--accent, #29457E)", "&gt;80 km")):
        for a, b in wins:
            out.append(f'<rect x="{x(a):.1f}" y="{y0_alt:.1f}" '
                       f'width="{max(1.0, x(b)-x(a)):.1f}" '
                       f'height="{(y0_spd+panel_h-y0_alt):.1f}" '
                       f'fill="{fill}" opacity="0.13"/>')

    # axes
    for y0 in (y0_alt, y0_spd):
        out.append(f'<line class="ax" x1="{PAD_L}" y1="{y0+panel_h:.1f}" '
                   f'x2="{W-PAD_R}" y2="{y0+panel_h:.1f}"/>')
        out.append(f'<line class="ax" x1="{PAD_L}" y1="{y0:.1f}" '
                   f'x2="{PAD_L}" y2="{y0+panel_h:.1f}"/>')

    # the 80 km gate and the 515 m/s gate
    if alt_top > 80:
        out.append(f'<line class="gl" x1="{PAD_L}" y1="{y_alt(80):.1f}" '
                   f'x2="{W-PAD_R}" y2="{y_alt(80):.1f}"/>')
        out.append(f'<text class="lbl" x="{W-PAD_R-2}" y="{y_alt(80)-3:.1f}" '
                   f'text-anchor="end">80 km gate</text>')
    if spd_top > 515:
        out.append(f'<line class="gl" x1="{PAD_L}" y1="{y_spd(515):.1f}" '
                   f'x2="{W-PAD_R}" y2="{y_spd(515):.1f}"/>')
        out.append(f'<text class="lbl" x="{W-PAD_R-2}" y="{y_spd(515)-3:.1f}" '
                   f'text-anchor="end">515 m/s gate</text>')

    # traces, subsampled -- 10 Hz truth is far more than the plot can show
    step = max(1, len(truth) // 900)
    pa = " ".join(f"{x(s['t']):.1f},{y_alt(s['alt_m']/1000):.1f}"
                  for s in truth[::step])
    ps = " ".join(f"{x(s['t']):.1f},{y_spd(s['speed_mps']):.1f}"
                  for s in truth[::step])
    out.append(f'<polyline class="trace" points="{pa}" '
               f'stroke="var(--accent, #29457E)"/>')
    out.append(f'<polyline class="trace" points="{ps}" '
               f'stroke="var(--ink-2, #4A5261)"/>')

    # lock strip: one bar per epoch, width set by the gap to the next
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_strip-5:.1f}">'
               f'receiver lock state</text>')
    # Merge runs of the same verdict into one rect. One rect per epoch is ~40 kB
    # of markup for a 470 s flight, and it renders as hairline seams where
    # adjacent bars round to different pixels.
    spans = []
    for i, (t, verdict) in enumerate(rows):
        nxt = rows[i + 1][0] if i + 1 < len(rows) else min(t + 1.0, dur)
        if spans and spans[-1][2] == verdict and abs(spans[-1][1] - t) < 3.0:
            spans[-1][1] = nxt
        else:
            spans.append([t, nxt, verdict])
    for a, b, verdict in spans:
        out.append(f'<rect x="{x(a):.1f}" y="{y0_strip:.1f}" '
                   f'width="{max(1.0, x(b)-x(a)):.1f}" '
                   f'height="{STRIP_H}" fill="{VERDICT_FILL[verdict]}"/>')

    # axis ticks
    for frac in (0, .25, .5, .75, 1.0):
        t = dur * frac
        out.append(f'<text class="lbl" x="{x(t):.1f}" y="{H-PAD_B+STRIP_H+14:.1f}" '
                   f'text-anchor="middle">{t:.0f}s</text>')
    for frac in (0, .5, 1.0):
        out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y_alt(alt_top*frac)+3:.1f}" '
                   f'text-anchor="end">{alt_top*frac:.0f}</text>')
        out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y_spd(spd_top*frac)+3:.1f}" '
                   f'text-anchor="end">{spd_top*frac:.0f}</text>')
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_alt-8:.1f}">altitude (km)</text>')
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_spd-8:.1f}">speed (m/s)</text>')
    out.append('</svg>')
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("-s", "--scenario", required=True, type=Path)
    ap.add_argument("-t", "--start", required=True)
    ap.add_argument("-o", "--out", required=True, type=Path)
    args = ap.parse_args()

    meta = json.loads(args.scenario.read_text())
    truth = Truth(meta)
    start = parse_start(args.start)

    rows = []
    for s in collect(args.capture):
        t, src = trajectory_time(s, start, None)
        if src == "host" or t < 0 or t > meta["duration_s"]:
            continue
        rows.append((t, s[4]))
    rows.sort()
    if not rows:
        raise SystemExit(f"no placeable epochs in {args.capture}")

    args.out.write_text(build_svg(meta, rows, truth.samples))
    n = {v: sum(1 for _, x in rows if x == v) for v in VERDICT_FILL}
    print(f"{args.out}  ({len(rows)} epochs: "
          + ", ".join(f"{k} {v}" for k, v in n.items() if v) + ")")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
