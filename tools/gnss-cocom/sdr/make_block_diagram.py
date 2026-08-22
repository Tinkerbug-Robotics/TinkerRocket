#!/usr/bin/env python3
"""Draw the rig as a block diagram for the report's methodology section.

Hand-authoring SVG path data is miserable to maintain and easy to get subtly
wrong, so the boxes and connectors are laid out here from a small description
and the geometry is computed. Colours come from the report's CSS variables with
literal fallbacks, so the figure themes with the page and still renders if it is
opened on its own.

    ./make_block_diagram.py     ->  results/figures/rig_block_diagram.svg
"""

from __future__ import annotations

from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "rig_block_diagram.svg"

W, H = 900, 330
BOX_H = 46
R = 5

# (x, y, w, label, sublabel, kind)
BOXES = [
    (14,  30, 128, "Broadcast ephemeris", "BKG, RINEX 3 → 2", "src"),
    (14,  92, 128, "Trajectory", "10 Hz lat/lon/alt", "src"),
    (176, 61, 118, "gps-sdr-sim", "GPS L1 C/A", "proc"),
    (328, 61, 104, ".C8 baseband", "2.6 MSa/s", "data"),
    (466, 61, 118, "HackRF One", "1575.42 MHz", "hw"),
    (618, 61, 118, "Attenuators", "70 or 100 dB", "hw"),

    (466, 196, 118, "Receiver", "one of five", "hw"),
    (618, 196, 118, "Capture", "UART or USB", "proc"),
    (762, 196, 124, "Classifier", "FIX / BLOCKED / NO_LOCK", "proc"),
    (302, 196, 128, "Compare", "vs injected truth", "proc"),
    (110, 196, 152, "Gate thresholds", "velocity and altitude", "out"),
]

FILL = {"src": "var(--surface-2, #EEF1F5)", "proc": "var(--surface, #FFFFFF)",
        "data": "var(--surface-2, #EEF1F5)", "hw": "var(--surface, #FFFFFF)",
        "out": "var(--accent-soft, #E7EDF6)"}
STROKE = {"src": "var(--rule, #DDE2E9)", "proc": "var(--rule-strong, #C3CAD5)",
          "data": "var(--rule, #DDE2E9)", "hw": "var(--rule-strong, #C3CAD5)",
          "out": "var(--accent, #2C5CA8)"}


def box(x, y, w, label, sub, kind):
    o = [f'<rect x="{x}" y="{y}" width="{w}" height="{BOX_H}" rx="{R}" '
         f'fill="{FILL[kind]}" stroke="{STROKE[kind]}" stroke-width="1"/>']
    o.append(f'<text class="bl" x="{x + w/2:.0f}" y="{y + 19}" '
             f'text-anchor="middle">{label}</text>')
    if sub:
        o.append(f'<text class="sb" x="{x + w/2:.0f}" y="{y + 34}" '
                 f'text-anchor="middle">{sub}</text>')
    return o


def arrow(x1, y1, x2, y2, dashed=False, label=None, lx=None, ly=None):
    d = ' stroke-dasharray="4 3"' if dashed else ''
    o = [f'<line class="ar" x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}"{d} '
         f'marker-end="url(#ah)"/>']
    if label:
        o.append(f'<text class="lb" x="{lx}" y="{ly}" text-anchor="middle">{label}</text>')
    return o


def elbow(x1, y1, x2, y2, label=None):
    """Right-angled connector: across, then down/up, then into the target."""
    mid = (y1 + y2) / 2
    o = [f'<path class="ar" d="M {x1} {y1} V {mid} H {x2} V {y2}" fill="none" '
         f'marker-end="url(#ah)"/>']
    if label:
        o.append(f'<text class="lb" x="{(x1+x2)/2:.0f}" y="{mid-6:.0f}" '
                 f'text-anchor="middle">{label}</text>')
    return o


def main() -> int:
    p = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Block diagram of the GNSS receiver test rig: ephemeris and '
         f'trajectory into gps-sdr-sim, baseband to a HackRF through attenuators '
         f'into the receiver, then capture, classification and comparison against '
         f'the injected trajectory">']
    p.append('''<defs>
  <marker id="ah" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7"
          markerHeight="7" orient="auto-start-reverse">
    <path d="M 0 1 L 7 4 L 0 7 z" fill="var(--ink-3, #79808F)"/>
  </marker>
</defs>
<style>
  .bl{font-family:var(--f-display,sans-serif);font-size:12.5px;font-weight:600;fill:var(--ink,#1B2230)}
  .sb{font-family:var(--f-mono,monospace);font-size:9.5px;fill:var(--ink-3,#79808F)}
  .lb{font-family:var(--f-mono,monospace);font-size:9.5px;fill:var(--ink-3,#79808F)}
  .hd{font-family:var(--f-display,sans-serif);font-size:10px;font-weight:600;
      letter-spacing:.09em;text-transform:uppercase;fill:var(--ink-3,#79808F)}
  .ar{stroke:var(--ink-3,#79808F);stroke-width:1.3}
</style>''')

    p.append('<text class="hd" x="14" y="18">Signal generation</text>')
    p.append('<text class="hd" x="14" y="184">Measurement</text>')

    for b in BOXES:
        p += box(*b)

    # generation row
    p += elbow(142, 53, 176, 84)          # ephemeris -> sim
    p += elbow(142, 115, 176, 84)         # trajectory -> sim
    p += arrow(294, 84, 328, 84)
    p += arrow(432, 84, 466, 84)
    p += arrow(584, 84, 618, 84)

    # attenuators -> receiver, the two paths
    p.append('<path class="ar" d="M 736 84 H 772 V 150 H 525 V 196" fill="none" '
             'marker-end="url(#ah)"/>')
    p.append('<text class="lb" x="648" y="144" text-anchor="middle">'
             'conducted (SMA) &#183; or radiated into a Faraday cage</text>')

    # measurement row, right to left
    p += arrow(584, 219, 618, 219)
    p += arrow(736, 219, 762, 219)
    p.append('<path class="ar" d="M 824 242 V 272 H 366 V 242" fill="none" '
             'marker-end="url(#ah)"/>')
    p.append('<text class="lb" x="595" y="288" text-anchor="middle">'
             'per-epoch fix state and satellite C/N&#8320;</text>')
    p += arrow(302, 219, 262, 219)

    # the trajectory feeds the comparison as well as the simulator
    # From the BOTTOM edge of the trajectory box (y = 92 + BOX_H), not its
    # centre: a connector that starts inside the shape it leaves draws a stub
    # across the label.
    p.append(f'<path class="ar" d="M 78 {92 + BOX_H} V 219 H 110" fill="none" '
             'stroke-dasharray="4 3" marker-end="url(#ah)"/>')
    p.append('<text class="lb" x="70" y="172" text-anchor="middle" '
             'transform="rotate(-90 70 172)">ground truth</text>')

    p.append('</svg>')
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(p) + "\n")
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
