#!/usr/bin/env python3
"""Draw the rig as a block diagram for the report's methodology section.

Boxes are **sized from their text**, and the row is then laid out from those
widths, rather than both being hardcoded. The first version fixed the widths by
eye and several labels overflowed their boxes -- which is the predictable
outcome, because SVG does not wrap or clip text and nothing complains when it
spills. Anything that changes a label now changes the box that holds it.

Widths are estimated from character count and font size. That is approximate,
so PAD is generous and `--check` reports the tightest fit in the drawing; keep
some slack there and the estimate never has to be exact.

Colours come from the report's CSS variables with literal fallbacks, so the
figure themes with the page and still renders opened on its own.

    ./make_block_diagram.py            ->  results/figures/rig_block_diagram.svg
    ./make_block_diagram.py --check    report per-box text fit
"""

from __future__ import annotations

import argparse
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "rig_block_diagram.svg"

W = 900
BOX_H, R, PAD, GAP = 48, 5, 16, 30
LABEL_PX, SUB_PX = 12.5, 9.5

# Average advance width as a fraction of font size. IBM Plex Sans Condensed is
# narrow; IBM Plex Mono is a true monospace at 0.6 em. Both are rounded up.
EM_DISPLAY, EM_MONO = 0.52, 0.62


def text_w(s: str, px: float, mono: bool) -> float:
    return len(s) * px * (EM_MONO if mono else EM_DISPLAY)


class Box:
    """A labelled block. Geometry is rounded to whole pixels at construction and
    layout, so the rect and every connector that references its edges agree.

    Rounding at draw time instead put an arrow one pixel inside the box it left,
    because the rect rounded x and w separately while the connector rounded
    their sum.
    """

    def __init__(self, label, sub, kind):
        self.label, self.sub, self.kind = label, sub, kind
        self.w = round(max(text_w(label, LABEL_PX, False),
                           text_w(sub, SUB_PX, True)) + 2 * PAD)
        self.x = self.y = 0

    @property
    def cx(self): return self.x + self.w // 2
    @property
    def cy(self): return self.y + BOX_H // 2
    @property
    def right(self): return self.x + self.w
    @property
    def bottom(self): return self.y + BOX_H

    def slack(self):
        widest = max(text_w(self.label, LABEL_PX, False),
                     text_w(self.sub, SUB_PX, True))
        return self.w - widest


FILL = {"src": "var(--surface-2, #EEF1F5)", "proc": "var(--surface, #FFFFFF)",
        "data": "var(--surface-2, #EEF1F5)", "hw": "var(--surface, #FFFFFF)",
        "out": "var(--accent-soft, #E7EDF6)"}
STROKE = {"src": "var(--rule, #DDE2E9)", "proc": "var(--rule-strong, #C3CAD5)",
          "data": "var(--rule, #DDE2E9)", "hw": "var(--rule-strong, #C3CAD5)",
          "out": "var(--accent, #2C5CA8)"}

# --- the drawing ------------------------------------------------------------
eph = Box("Ephemeris", "BKG broadcast", "src")
traj = Box("Trajectory", "10 Hz lat/lon/alt", "src")
sim = Box("gps-sdr-sim", "GPS L1 C/A", "proc")
c8 = Box("Baseband", "2.6 MSa/s .C8", "data")
hrf = Box("HackRF One", "1575.42 MHz", "hw")
att = Box("Attenuators", "70 or 100 dB", "hw")

rx = Box("Receiver", "one of five", "hw")
cap = Box("Capture", "UART or USB", "proc")
cls = Box("Classifier", "FIX / BLOCKED / NO_LOCK", "proc")
cmp_ = Box("Compare", "against the trajectory", "proc")
out = Box("Gate thresholds", "velocity and altitude", "out")

M = 36                      # left margin, leaving a routing channel at CHAN
CHAN = 15                   # x of the vertical channel left of every box
TOP_A, TOP_B = 26, 214      # row baselines
STACK_GAP = 14


def layout():
    # column 0 of row A is a stack of two, vertically centred on the row
    col0 = max(eph.w, traj.w)
    eph.w = traj.w = col0
    eph.x = traj.x = M
    eph.y = TOP_A
    traj.y = TOP_A + BOX_H + STACK_GAP
    mid_a = round((eph.y + traj.bottom) / 2)

    x = M + col0 + GAP
    for b in (sim, c8, hrf, att):
        b.x, b.y = x, mid_a - BOX_H // 2
        x += b.w + GAP

    x = M
    for b in (rx, cap, cls, cmp_, out):
        b.x, b.y = x, TOP_B
        x += b.w + GAP
    return x - GAP          # right-most extent


def box_svg(b):
    o = [f'<rect x="{b.x:.0f}" y="{b.y:.0f}" width="{b.w:.0f}" height="{BOX_H}" '
         f'rx="{R}" fill="{FILL[b.kind]}" stroke="{STROKE[b.kind]}" stroke-width="1"/>',
         f'<text class="bl" x="{b.cx:.0f}" y="{b.y + 20:.0f}" text-anchor="middle">'
         f'{b.label}</text>']
    if b.sub:
        o.append(f'<text class="sb" x="{b.cx:.0f}" y="{b.y + 35:.0f}" '
                 f'text-anchor="middle">{b.sub}</text>')
    return o


def h_arrow(a, b, dashed=False):
    d = ' stroke-dasharray="4 3"' if dashed else ''
    return [f'<line class="ar" x1="{a.right:.0f}" y1="{a.cy:.0f}" '
            f'x2="{b.x:.0f}" y2="{b.cy:.0f}"{d} marker-end="url(#ah)"/>']


def elbow_into(a, b):
    """From a's right edge, across and down/up into b's left edge."""
    midx = (a.right + b.x) // 2
    return [f'<path class="ar" d="M {a.right:.0f} {a.cy:.0f} H {midx:.0f} '
            f'V {b.cy:.0f} H {b.x:.0f}" fill="none" marker-end="url(#ah)"/>']


def build():
    extent = layout()
    height = 330
    p = [f'<svg viewBox="0 0 {W} {height}" xmlns="http://www.w3.org/2000/svg" '
         f'role="img" aria-label="Block diagram of the GNSS receiver test rig: '
         f'ephemeris and trajectory into gps-sdr-sim, baseband to a HackRF through '
         f'attenuators into the receiver, then capture, classification and '
         f'comparison against the injected trajectory">']
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
    p.append(f'<text class="hd" x="{M}" y="16">Signal generation</text>')
    p.append(f'<text class="hd" x="{M}" y="204">Measurement</text>')

    for b in (eph, traj, sim, c8, hrf, att, rx, cap, cls, cmp_, out):
        p += box_svg(b)

    p += elbow_into(eph, sim)
    p += elbow_into(traj, sim)
    for a, b in ((sim, c8), (c8, hrf), (hrf, att)):
        p += h_arrow(a, b)
    for a, b in ((rx, cap), (cap, cls), (cls, cmp_), (cmp_, out)):
        p += h_arrow(a, b)

    # attenuators wrap down into the receiver
    wrap_y = (att.bottom + rx.y) // 2 - 12
    p.append(f'<path class="ar" d="M {att.cx:.0f} {att.bottom:.0f} '
             f'V {wrap_y:.0f} H {rx.cx:.0f} V {rx.y:.0f}" fill="none" '
             f'marker-end="url(#ah)"/>')
    p.append(f'<text class="lb" x="{(att.cx + rx.cx)/2:.0f}" y="{wrap_y - 7:.0f}" '
             f'text-anchor="middle">conducted (SMA), or radiated inside a '
             f'Faraday cage</text>')

    # The trajectory is also the ground truth the capture is compared against.
    # It leaves by the LEFT edge and drops down the routing channel: dropping
    # from the box's centre put the vertical segment straight through the
    # Receiver box, which sits directly below it.
    gt_y = out.bottom + 34
    p.append(f'<path class="ar" d="M {traj.x:.0f} {traj.cy:.0f} H {CHAN} '
             f'V {gt_y:.0f} H {cmp_.cx:.0f} V {cmp_.bottom:.0f}" fill="none" '
             f'stroke-dasharray="4 3" marker-end="url(#ah)"/>')
    p.append(f'<text class="lb" x="{(CHAN + cmp_.cx)//2:.0f}" y="{gt_y + 14:.0f}" '
             f'text-anchor="middle">the same trajectory is the ground truth</text>')

    p.append('</svg>')
    return "\n".join(p) + "\n", extent, height


def segments(svg: str):
    """Every connector segment as (x1, y1, x2, y2), from lines and elbow paths."""
    import re
    segs = []
    for m in re.findall(r'<line class="ar" x1="([\d.]+)" y1="([\d.]+)" '
                        r'x2="([\d.]+)" y2="([\d.]+)"', svg):
        segs.append(tuple(float(v) for v in m))
    # Capture the whole d attribute, M included: matching after the M left
    # the parser with no move-to and a None cursor.
    for d in re.findall(r'<path class="ar" d="([^"]+)"', svg):
        toks, x, y = d.split(), None, None
        i = 0
        while i < len(toks):
            t = toks[i]
            if t == "M":
                x, y = float(toks[i + 1]), float(toks[i + 2]); i += 3
            elif t == "H":
                nx = float(toks[i + 1]); segs.append((x, y, nx, y)); x = nx; i += 2
            elif t == "V":
                ny = float(toks[i + 1]); segs.append((x, y, x, ny)); y = ny; i += 2
            else:
                i += 1
    return segs


def crossings(svg: str, boxes):
    """Segments that pass through a box interior, excluding the box they touch.

    Endpoint checks alone are not enough: a connector can start and end in clear
    space and still run straight through a block on the way, which is what the
    ground-truth line did to the Receiver.
    """
    out = []
    for (x1, y1, x2, y2) in segments(svg):
        lo_x, hi_x = min(x1, x2), max(x1, x2)
        lo_y, hi_y = min(y1, y2), max(y1, y2)
        for b in boxes:
            bx0, by0, bx1, by1 = b.x, b.y, b.right, b.bottom
            # strict overlap on both axes means the segment enters the interior
            if lo_x < bx1 - 0.5 and bx0 + 0.5 < hi_x and \
               lo_y < by1 - 0.5 and by0 + 0.5 < hi_y:
                out.append(((x1, y1, x2, y2), b.label))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="report per-box text fit instead of writing the file")
    args = ap.parse_args()

    svg, extent, height = build()
    boxes = [eph, traj, sim, c8, hrf, att, rx, cap, cls, cmp_, out]
    if args.check:
        print(f"  {'box':<18} {'width':>6} {'slack':>7}")
        for b in boxes:
            flag = "  <- TIGHT" if b.slack() < 8 else ""
            print(f"  {b.label:<18} {b.w:>6.0f} {b.slack():>7.1f}{flag}")
        print(f"\n  widest row extends to x={extent:.0f} of {W}")
        bad = crossings(svg, boxes)
        for seg, name in bad:
            print(f"  !! connector {seg} passes through '{name}'")
        print(f"  connectors crossing a box: {len(bad)}")
        return 0 if (extent <= W and not bad
                     and all(b.slack() >= 0 for b in boxes)) else 1

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(svg)
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes), "
          f"widest row to x={extent:.0f} of {W}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
