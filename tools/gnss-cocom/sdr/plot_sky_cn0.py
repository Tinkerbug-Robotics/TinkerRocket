#!/usr/bin/env python3
"""GPS L1 C/A C/N0 against elevation on the real sky, both receivers on the same outdoor
antenna, for cn0_boost_report.html: every dot one satellite over two minutes (sky_cn0.py
--json), the lines each elevation band's median. Each receiver on its own reading scale.

    python3 plot_sky_cn0.py PX1105R.json NEO-M8T.json   # writes results/figures/cn0_sky.svg
"""
import json
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "cn0_sky.svg"
# colours kept apart from section 01's traveler/hotshot pair; shapes carry identity too
RX = [("SkyTraq PX1105R", "var(--mode-balloon, #2a78d6)", "circle"),
      ("u-blox NEO-M8T", "var(--mode-drone, #1baf7a)", "diamond")]
BANDS = [(15, 30), (30, 60), (60, 90)]

W, H = 900, 470
PAD_L, PAD_R, PAD_T, PAD_B = 58, 24, 26, 96
PW, PH = W - PAD_L - PAD_R, H - PAD_T - PAD_B
X0, X1, Y0, Y1 = 0.0, 90.0, 20.0, 50.0


def X(v): return PAD_L + (min(max(v, X0), X1) - X0) / (X1 - X0) * PW
def Y(v): return PAD_T + (Y1 - min(max(v, Y0), Y1)) / (Y1 - Y0) * PH


def mark(shape, x, y, colr, r=3.4):
    if shape == "circle":
        return (f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{colr}" fill-opacity="0.55" '
                f'stroke="{colr}" stroke-width="1"/>')
    return (f'<path d="M{x:.1f},{y - r - 0.8:.1f} L{x + r + 0.8:.1f},{y:.1f} L{x:.1f},{y + r + 0.8:.1f} '
            f'L{x - r - 0.8:.1f},{y:.1f} Z" fill="{colr}" fill-opacity="0.55" stroke="{colr}" stroke-width="1"/>')


def main():
    data = []
    for path in sys.argv[1:3]:
        d = json.loads(Path(path).read_text())
        data.append([r for r in d["rows"] if r["system"] == "GPS" and r["band"] == "L1" and r["el"] is not None])
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="GPS L1 carrier-to-noise readings against elevation for the SkyTraq PX1105R and the u-blox '
         f'NEO-M8T on the same outdoor antenna, one mark per satellite per two minutes, with the median of each '
         f'elevation band.">',
         '<style>'
         '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-3,#79808F)}'
         '.val{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-2,#4A5261)}'
         '</style>']
    for v in range(20, 51, 5):
        o.append(f'<line class="gl" x1="{PAD_L}" y1="{Y(v):.1f}" x2="{PAD_L + PW}" y2="{Y(v):.1f}"/>')
        o.append(f'<text class="lbl" x="{PAD_L - 6}" y="{Y(v) + 3:.1f}" text-anchor="end">{v}</text>')
    for v in range(0, 91, 15):
        o.append(f'<text class="lbl" x="{X(v):.1f}" y="{PAD_T + PH + 14}" text-anchor="middle">{v}</text>')
    o.append(f'<line class="ax" x1="{PAD_L}" y1="{PAD_T + PH}" x2="{PAD_L + PW}" y2="{PAD_T + PH}"/>')
    o.append(f'<line class="ax" x1="{PAD_L}" y1="{PAD_T}" x2="{PAD_L}" y2="{PAD_T + PH}"/>')
    o.append(f'<text class="lbl" x="{PAD_L + PW / 2:.0f}" y="{PAD_T + PH + 29}" text-anchor="middle">'
             f'elevation, degrees</text>')
    o.append(f'<text class="lbl" transform="translate(16,{PAD_T + PH / 2:.0f}) rotate(-90)" text-anchor="middle">'
             f'C/N0 the receiver reports, dB-Hz</text>')
    for (name, colr, shape), rows in zip(RX, data):
        for r in rows:
            o.append(mark(shape, X(r["el"]), Y(r["cn0"]), colr))
    for lo, hi in BANDS:
        meds = [st.median(b) if b else None
                for b in ([r["cn0"] for r in rows if lo <= r["el"] < hi] for rows in data)]
        top = max(range(len(meds)), key=lambda k: -1e9 if meds[k] is None else meds[k])
        for k, ((name, colr, shape), m) in enumerate(zip(RX, meds)):
            if m is None:
                continue
            o.append(f'<line x1="{X(lo) + 3:.1f}" y1="{Y(m):.1f}" x2="{X(hi) - 3:.1f}" y2="{Y(m):.1f}" '
                     f'stroke="{colr}" stroke-width="2.6" stroke-linecap="round"/>')
            # the higher median's label above its line, the lower one's below: they can sit 1 dB apart
            o.append(f'<text class="val" x="{X(hi) - 6:.1f}" y="{Y(m) + (-6 if k == top else 13):.1f}" '
                     f'text-anchor="end">{m:.1f}</text>')
    ly = H - 34
    lx = PAD_L
    for name, colr, shape in RX:
        o.append(mark(shape, lx + 5, ly - 3, colr))
        o.append(f'<text class="lbl" x="{lx + 16}" y="{ly}">{name}, one satellite over 2 min</text>')
        lx += 16 + (len(name) + 26) * 5.45 + 26
    o.append(f'<line x1="{lx}" y1="{ly - 3}" x2="{lx + 18}" y2="{ly - 3}" stroke="var(--ink-3,#79808F)" '
             f'stroke-width="2.6" stroke-linecap="round"/>')
    o.append(f'<text class="lbl" x="{lx + 24}" y="{ly}">median of the band, 15-30, 30-60, 60-90 deg</text>')
    o.append(f'<text class="lbl" x="{PAD_L}" y="{H - 14}">Same outdoor antenna, 20 min each, back to back '
             f'(29 Sep 2026); no transmitter. Each receiver reads on its own scale.</text>')
    o.append('</svg>')
    OUT.write_text("\n".join(o) + "\n")
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes; "
          + ", ".join(f"{n}: {len(rows)} marks" for (n, _c, _s), rows in zip(RX, data)) + ")")


if __name__ == "__main__":
    main()
