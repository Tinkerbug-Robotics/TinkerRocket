#!/usr/bin/env python3
"""Scatter of per-satellite C/N0 change through the burn against elevation.

Two panels at the same scale, one per acceleration, because the comparison is
the whole argument: if the loss were signal level or geometry it would look the
same in both, and it does not. Doppler rate goes as `a * sin(elevation)`, so
the high-elevation satellites are the stressed ones and only at high g.

  python3 plot_boost_elev.py            # writes results/figures/boost_elevation.svg
"""
import json, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "boost_elevation.svg"

PARTS = [("zed_f9p", "ZED-F9P", "var(--accent, #29457E)"),
         ("neo_m8t", "NEO-M8T", "var(--blocked, #A2660A)"),
         ("quescan_m10", "Quescan M10", "var(--fix, #0E7C66)"),
         ("beitian_bn182", "Beitian BN-182", "var(--nolock, #9B3535)")]
PANELS = [("spaceshot", "13.5 g boost"), ("gentle_alt", "2.0 g boost")]

W, H = 900, 400
PAD_L, PAD_R, PAD_T, PAD_B = 54, 16, 44, 74
GAP = 46
PANEL_W = (W - PAD_L - PAD_R - GAP) / 2
PANEL_H = H - PAD_T - PAD_B
X0, X1 = 0.0, 75.0          # elevation, degrees
Y0, Y1 = -50.0, 15.0        # dC/N0, dB


def collect():
    data = {}
    for scen, _ in PANELS:
        for pid, _lbl, _c in PARTS:
            cap = HERE / "results" / f"{pid}_{scen}.log.gz"
            sc = HERE / "results" / f"{pid}_{scen}.scenario.json"
            if not (cap.exists() and sc.exists()):
                continue
            r = subprocess.run([sys.executable, str(HERE / "boost_sats.py"),
                                str(cap), str(sc), "--json"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print(f"  skip {pid}/{scen}: {r.stderr.strip().splitlines()[-1:]}")
                continue
            d = json.loads(r.stdout)
            data[(scen, pid)] = d
    return data


def main():
    data = collect()
    if not data:
        sys.exit("no captures could be analysed")

    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Per-satellite carrier-to-noise change through the burn '
         f'plotted against satellite elevation, at 13.5 g and at 2.0 g. At 13.5 g '
         f'the high-elevation satellites lose 15 to 47 dB on every receiver; at '
         f'2.0 g there is no elevation trend.">',
         '<style>'
         '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
         '.zl{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-3,#79808F)}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;'
         'font-weight:600;fill:var(--ink-2,#4A5261)}'
         '.rr{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-2,#4A5261)}'
         '</style>']

    for pi, (scen, ptitle) in enumerate(PANELS):
        px = PAD_L + pi * (PANEL_W + GAP)

        def x(e): return px + (e - X0) / (X1 - X0) * PANEL_W
        def y(d): return PAD_T + (Y1 - d) / (Y1 - Y0) * PANEL_H

        o.append(f'<text class="ttl" x="{px:.0f}" y="{PAD_T-26:.0f}">{ptitle}</text>')
        # grid + y labels
        for dv in range(-50, 16, 10):
            o.append(f'<line class="gl" x1="{px:.1f}" y1="{y(dv):.1f}" '
                     f'x2="{px+PANEL_W:.1f}" y2="{y(dv):.1f}"/>')
            if pi == 0:
                o.append(f'<text class="lbl" x="{px-6:.1f}" y="{y(dv)+3:.1f}" '
                         f'text-anchor="end">{dv:+d}</text>')
        o.append(f'<line class="zl" x1="{px:.1f}" y1="{y(0):.1f}" '
                 f'x2="{px+PANEL_W:.1f}" y2="{y(0):.1f}"/>')
        o.append(f'<line class="ax" x1="{px:.1f}" y1="{PAD_T:.1f}" '
                 f'x2="{px:.1f}" y2="{PAD_T+PANEL_H:.1f}"/>')
        for ev in range(0, 76, 15):
            o.append(f'<text class="lbl" x="{x(ev):.1f}" y="{PAD_T+PANEL_H+14:.1f}" '
                     f'text-anchor="middle">{ev}</text>')
        o.append(f'<text class="lbl" x="{px+PANEL_W/2:.1f}" '
                 f'y="{PAD_T+PANEL_H+28:.1f}" text-anchor="middle">'
                 f'satellite elevation, degrees</text>')

        rs = []
        for pid, lbl, col in PARTS:
            d = data.get((scen, pid))
            if not d:
                continue
            rs.append((lbl, d.get("r_sin_elev_vs_delta")))
            for s in d["sats"]:
                cx, cy = x(max(X0, min(X1, s["elev"]))), y(max(Y0, min(Y1, s["delta"])))
                if s["lost"]:
                    o.append(f'<path d="M{cx-4:.1f},{cy-4:.1f} L{cx+4:.1f},{cy+4:.1f} '
                             f'M{cx-4:.1f},{cy+4:.1f} L{cx+4:.1f},{cy-4:.1f}" '
                             f'stroke="{col}" stroke-width="1.8" fill="none"/>')
                else:
                    o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3" '
                             f'fill="{col}" fill-opacity="0.55" stroke="{col}"/>')
        for i, (lbl, r) in enumerate(rs):
            txt = "r = n/a" if r is None else f"r = {r:+.2f}"
            o.append(f'<text class="rr" x="{px+PANEL_W-2:.1f}" '
                     f'y="{PAD_T+12+i*12:.1f}" text-anchor="end">{lbl}  {txt}</text>')

    # legend
    ly = H - 30
    lx = PAD_L
    for pid, lbl, col in PARTS:
        o.append(f'<circle cx="{lx+4:.1f}" cy="{ly-3:.1f}" r="3" fill="{col}" '
                 f'fill-opacity="0.55" stroke="{col}"/>')
        o.append(f'<text class="lbl" x="{lx+13:.1f}" y="{ly:.1f}">{lbl}</text>')
        lx += 16 + len(lbl) * 5.6
    o.append(f'<path d="M{lx+1:.1f},{ly-7:.1f} L{lx+9:.1f},{ly+1:.1f} '
             f'M{lx+1:.1f},{ly+1:.1f} L{lx+9:.1f},{ly-7:.1f}" '
             f'stroke="var(--ink-3,#79808F)" stroke-width="1.8" fill="none"/>')
    o.append(f'<text class="lbl" x="{lx+14:.1f}" y="{ly:.1f}">'
             f'lost lock entirely</text>')
    o.append(f'<text class="lbl" x="{PAD_L:.1f}" y="{H-10:.1f}">'
             f'change in carrier-to-noise from the pad baseline to the burn, '
             f'per satellite; r is against sin(elevation)</text>')
    o.append('</svg>')

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(o))
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes, "
          f"{sum(len(d['sats']) for d in data.values())} points)")


if __name__ == "__main__":
    main()
