#!/usr/bin/env python3
"""Carrier loss against Doppler rate, both accelerations on one axis.

Pooling the 13.5 g and 2.0 g flights onto a single rate axis turns the
acceleration control into a dose-response: the gentle flight is not a separate
condition that happens to show nothing, it is the low end of the same curve.
Its satellites never exceed 79 Hz/s and never lose carrier; the same satellites
on the same sky reach 501 Hz/s under a real boost and shed 15-47 dB.

  python3 plot_boost_rate.py        # writes results/figures/boost_rate.svg
"""
import json, math, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "boost_rate.svg"

PARTS = [("zed_f9p", "ZED-F9P", "var(--accent, #29457E)"),
         ("neo_m8t", "NEO-M8T", "var(--blocked, #A2660A)"),
         ("quescan_m10", "Quescan M10", "var(--fix, #0E7C66)"),
         ("beitian_bn182", "Beitian BN-182", "var(--nolock, #9B3535)")]
SCENARIOS = [("spaceshot", "13.5 g", True), ("gentle_alt", "2.0 g", False)]

W, H = 900, 420
PAD_L, PAD_R, PAD_T, PAD_B = 54, 18, 30, 92
PW, PH = W - PAD_L - PAD_R, H - PAD_T - PAD_B
X0, X1 = 0.0, 520.0
Y0, Y1 = -50.0, 15.0

# u-blox publishes a 4 g dynamics limit for the M10. The loop does not see g, it
# sees Hz/s, and the conversion is worst-case at zenith: 4 g * 5.255 Hz per m/s.
G = 9.80665
HZ_PER_MPS = 1575.42e6 / 299792458.0
SPEC_G = 4.0
SPEC_HZS = SPEC_G * G * HZ_PER_MPS


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx == 0 or sy == 0:
        return None
    return sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / (sx * sy)


def X(v): return PAD_L + (min(max(v, X0), X1) - X0) / (X1 - X0) * PW
def Y(v): return PAD_T + (Y1 - min(max(v, Y0), Y1)) / (Y1 - Y0) * PH


def main():
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Per-satellite carrier-to-noise change plotted against the '
         f'Doppler rate that satellite presented during the burn, pooling a 13.5 g '
         f'and a 2.0 g flight across four receivers. Loss grows with rate; below '
         f'100 Hz per second there is none.">',
         '<style>'
         '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-3,#79808F)}'
         '.rr{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-2,#4A5261)}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;'
         'font-weight:600;fill:var(--ink-2,#4A5261)}'
         '</style>']

    for dv in range(-50, 16, 10):
        o.append(f'<line class="gl" x1="{PAD_L}" y1="{Y(dv):.1f}" '
                 f'x2="{PAD_L+PW}" y2="{Y(dv):.1f}"/>')
        o.append(f'<text class="lbl" x="{PAD_L-6}" y="{Y(dv)+3:.1f}" '
                 f'text-anchor="end">{dv:+d}</text>')
    o.append(f'<line class="ax" x1="{PAD_L}" y1="{Y(0):.1f}" '
             f'x2="{PAD_L+PW}" y2="{Y(0):.1f}"/>')
    o.append(f'<line class="ax" x1="{PAD_L}" y1="{PAD_T}" '
             f'x2="{PAD_L}" y2="{PAD_T+PH}"/>')
    for v in range(0, 521, 65):
        o.append(f'<text class="lbl" x="{X(v):.1f}" y="{PAD_T+PH+14}" '
                 f'text-anchor="middle">{v}</text>')
    o.append(f'<text class="lbl" x="{PAD_L+PW/2:.0f}" y="{PAD_T+PH+29}" '
             f'text-anchor="middle">Doppler rate presented during the burn, Hz/s'
             f'</text>')
    o.append(f'<text class="lbl" transform="translate(14,{PAD_T+PH/2:.0f}) rotate(-90)" '
             f'text-anchor="middle">change in carrier-to-noise, dB</text>')

    # the published 4 g limit, as the Doppler rate it actually implies
    o.append(f'<line class="gl" x1="{X(SPEC_HZS):.1f}" y1="{PAD_T}" '
             f'x2="{X(SPEC_HZS):.1f}" y2="{PAD_T+PH}" '
             f'stroke="var(--ink-3,#79808F)" stroke-dasharray="2 3"/>')
    o.append(f'<text class="lbl" x="{X(SPEC_HZS)+4:.1f}" y="{PAD_T+PH-6:.0f}">'
             f'{SPEC_G:.0f} g at zenith = {SPEC_HZS:.0f} Hz/s</text>')

    allx, ally, per = [], [], {}
    for scen, _g, filled in SCENARIOS:
        refp = HERE / "results" / f"doppler_ref_{scen}.json"
        if not refp.exists():
            sys.exit(f"run doppler_ref.py first ({refp.name} missing)")
        ref = json.loads(refp.read_text())["sats"]
        for pid, _lbl, col in PARTS:
            r = subprocess.run([sys.executable, str(HERE / "boost_sats.py"),
                                str(HERE / "results" / f"{pid}_{scen}.log.gz"),
                                str(HERE / "results" / f"{pid}_{scen}.scenario.json"),
                                "--json"], capture_output=True, text=True)
            if r.returncode != 0:
                continue
            dd = {f"0:{s['sv']}": s for s in json.loads(r.stdout)["sats"]
                  if s["gnss"] == "GPS"}
            for k, v in ref.items():
                if v["rate_hzs"] is None or k not in dd:
                    continue
                x, y = v["rate_hzs"], dd[k]["delta"]
                allx.append(x); ally.append(y)
                per.setdefault(scen, []).append(y)
                cx, cy = X(x), Y(y)
                if dd[k]["lost"]:
                    o.append(f'<path d="M{cx-4:.1f},{cy-4:.1f} L{cx+4:.1f},{cy+4:.1f} '
                             f'M{cx-4:.1f},{cy+4:.1f} L{cx+4:.1f},{cy-4:.1f}" '
                             f'stroke="{col}" stroke-width="1.8" fill="none"/>')
                elif filled:
                    o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3.2" '
                             f'fill="{col}" fill-opacity="0.6" stroke="{col}"/>')
                else:
                    o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3.2" '
                             f'fill="none" stroke="{col}" stroke-opacity="0.75"/>')

    r = pearson(allx, ally)
    o.append(f'<text class="rr" x="{PAD_L+PW-2}" y="{PAD_T+13}" text-anchor="end">'
             f'all four receivers, both flights: n={len(allx)}, '
             f'r = {r:+.2f}</text>')

    ly = H - 46
    lx = PAD_L
    for pid, lbl, col in PARTS:
        o.append(f'<circle cx="{lx+4:.1f}" cy="{ly-3:.1f}" r="3.2" fill="{col}" '
                 f'fill-opacity="0.6" stroke="{col}"/>')
        o.append(f'<text class="lbl" x="{lx+13:.1f}" y="{ly:.1f}">{lbl}</text>')
        lx += 16 + len(lbl) * 5.6
    o.append(f'<circle cx="{lx+4:.1f}" cy="{ly-3:.1f}" r="3.2" fill="none" '
             f'stroke="var(--ink-3,#79808F)"/>')
    o.append(f'<text class="lbl" x="{lx+13:.1f}" y="{ly:.1f}">hollow = 2.0 g flight'
             f'</text>')
    lx += 16 + 21 * 5.6
    o.append(f'<path d="M{lx+1:.1f},{ly-7:.1f} L{lx+9:.1f},{ly+1:.1f} '
             f'M{lx+1:.1f},{ly+1:.1f} L{lx+9:.1f},{ly-7:.1f}" '
             f'stroke="var(--ink-3,#79808F)" stroke-width="1.8" fill="none"/>')
    o.append(f'<text class="lbl" x="{lx+14:.1f}" y="{ly:.1f}">lost lock</text>')
    o.append(f'<text class="lbl" x="{PAD_L}" y="{H-26}">'
             f'Every point is one satellite on one receiver. Doppler rate is measured '
             f'from RXM-RAWX on a byte-identical scenario, so the same satellite '
             f'presents the same rate in all four runs.</text>')
    o.append(f'<text class="lbl" x="{PAD_L}" y="{H-14}">'
             f'GPS:11 is absent: it presented the highest rate of any satellite and '
             f'its raw measurement dropped out through the burn, so its rate cannot '
             f'be measured.</text>')
    o.append('</svg>')

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(o))
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes, {len(allx)} points, "
          f"r={r:+.2f})")


if __name__ == "__main__":
    main()
