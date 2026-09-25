#!/usr/bin/env python3
"""Carrier loss against measured Doppler: rate on the left, shift on the right.

The two panels are a discrimination, not a pair of views. Both quantities are
large during a 13.5 g boost, but they are close to independent of each other
across the sky -- the rate is set by the vehicle and scales with elevation,
while the shift is set mostly by the satellite's own motion and does not. If
carrier loss tracks the rate and not the shift, the mechanism is the tracking
loop failing to slew, not the carrier simply sitting far off nominal.

  python3 plot_boost_doppler.py     # writes results/figures/boost_doppler.svg
"""
import json, math, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REF = HERE / "results" / "doppler_ref_spaceshot.json"   # the 13.5 g flight
OUT = HERE / "results" / "figures" / "boost_doppler.svg"

# One color for every receiver. The point of the figure is that they all fall
# on one curve, and five categorical hues cannot be told apart in a scatter
# (the palette check passes only three all-pairs); identity is in the r list.
SERIES = "var(--accent, #29457E)"
PARTS = [("zed_f9p", "ZED-F9P", SERIES),
         ("neo_m8t", "NEO-M8T", SERIES),
         ("quescan_m10", "Quescan M10", SERIES),
         ("beitian_bn182", "Beitian BN-182", SERIES),
         ("ublox_m10", "SAM-M10Q", SERIES)]

W, H = 900, 422
PAD_L, PAD_R, PAD_T, PAD_B = 54, 16, 46, 96
GAP = 52
PW = (W - PAD_L - PAD_R - GAP) / 2
PH = H - PAD_T - PAD_B
# Headroom above +15 dB holds the per-receiver r list clear of the points.
Y0, Y1 = -50.0, 32.0


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


def main():
    if not REF.exists():
        sys.exit(f"run doppler_ref.py first ({REF.name} missing)")
    ref = json.loads(REF.read_text())
    sats = ref["sats"]

    deltas = {}
    for pid, _l, _c in PARTS:
        r = subprocess.run([sys.executable, str(HERE / "boost_sats.py"),
                            str(HERE / "results" / f"{pid}_spaceshot.log.gz"),
                            str(HERE / "results" / f"{pid}_spaceshot.scenario.json"),
                            "--json"], capture_output=True, text=True)
        if r.returncode != 0:
            continue
        deltas[pid] = {f"0:{s['sv']}": s for s in json.loads(r.stdout)["sats"]
                       if s["gnss"] == "GPS"}

    panels = [("rate_hzs", "Doppler rate through the burn, Hz/s", 0, 560),
              ("peak_shift_hz", "peak Doppler shift, kHz", 0, 7200)]

    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Carrier-to-noise change through the burn against measured '
         f'Doppler rate and against peak Doppler shift. Loss tracks the rate on '
         f'all five receivers and tracks the shift far more weakly.">',
         '<style>'
         '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-3,#79808F)}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;'
         'font-weight:600;fill:var(--ink-2,#4A5261)}'
         '.rr{font-family:var(--f-mono,monospace);font-size:9px;'
         'fill:var(--ink-2,#4A5261)}'
         '</style>']

    for pi, (key, xlabel, x0, x1) in enumerate(panels):
        px = PAD_L + pi * (PW + GAP)

        def X(v): return px + (v - x0) / (x1 - x0) * PW
        def Y(v): return PAD_T + (Y1 - max(Y0, min(Y1, v))) / (Y1 - Y0) * PH

        o.append(f'<text class="ttl" x="{px:.0f}" y="{PAD_T-28:.0f}">'
                 f'{"carrier loss vs Doppler RATE" if pi==0 else "carrier loss vs Doppler SHIFT"}'
                 f'</text>')
        for dv in range(-50, 16, 10):
            o.append(f'<line class="gl" x1="{px:.1f}" y1="{Y(dv):.1f}" '
                     f'x2="{px+PW:.1f}" y2="{Y(dv):.1f}"/>')
            if pi == 0:
                o.append(f'<text class="lbl" x="{px-6:.1f}" y="{Y(dv)+3:.1f}" '
                         f'text-anchor="end">{dv:+d}</text>')
        o.append(f'<line class="ax" x1="{px:.1f}" y1="{Y(0):.1f}" '
                 f'x2="{px+PW:.1f}" y2="{Y(0):.1f}"/>')
        o.append(f'<line class="ax" x1="{px:.1f}" y1="{PAD_T:.1f}" '
                 f'x2="{px:.1f}" y2="{PAD_T+PH:.1f}"/>')
        for k in range(5):
            v = x0 + (x1 - x0) * k / 4
            lab = f"{v:.0f}" if pi == 0 else f"{v/1000:.1f}"
            o.append(f'<text class="lbl" x="{X(v):.1f}" y="{PAD_T+PH+14:.1f}" '
                     f'text-anchor="middle">{lab}</text>')
        o.append(f'<text class="lbl" x="{px+PW/2:.1f}" y="{PAD_T+PH+28:.1f}" '
                 f'text-anchor="middle">{xlabel}</text>')

        for i, (pid, lbl, col) in enumerate(PARTS):
            dd = deltas.get(pid)
            if not dd:
                continue
            xs, ys = [], []
            for sv, s in sats.items():
                v = s[key]
                if v is None or sv not in dd:
                    continue
                d = dd[sv]["delta"]
                xs.append(v); ys.append(d)
                cx, cy = X(min(v, x1)), Y(d)
                if dd[sv]["lost"]:
                    o.append(f'<path d="M{cx-4:.1f},{cy-4:.1f} L{cx+4:.1f},{cy+4:.1f} '
                             f'M{cx-4:.1f},{cy+4:.1f} L{cx+4:.1f},{cy-4:.1f}" '
                             f'stroke="{col}" stroke-width="1.8" fill="none"/>')
                else:
                    o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3" fill="{col}" '
                             f'fill-opacity="0.55" stroke="{col}"/>')
            r = pearson(xs, ys)
            o.append(f'<text class="rr" x="{px+PW-2:.1f}" y="{PAD_T+12+i*12:.1f}" '
                     f'text-anchor="end">{lbl}  '
                     f'{"r = n/a" if r is None else f"r = {r:+.2f}"}</text>')

    ly = H - 54
    lx = PAD_L
    o.append(f'<circle cx="{lx+4:.1f}" cy="{ly-3:.1f}" r="3" fill="{SERIES}" '
             f'fill-opacity="0.55" stroke="{SERIES}"/>')
    o.append(f'<text class="lbl" x="{lx+13:.1f}" y="{ly:.1f}">one satellite on one '
             f'receiver</text>')
    lx += 16 + 30 * 5.6
    o.append(f'<path d="M{lx+1:.1f},{ly-7:.1f} L{lx+9:.1f},{ly+1:.1f} '
             f'M{lx+1:.1f},{ly+1:.1f} L{lx+9:.1f},{ly-7:.1f}" '
             f'stroke="var(--ink-3,#79808F)" stroke-width="1.8" fill="none"/>')
    o.append(f'<text class="lbl" x="{lx+14:.1f}" y="{ly:.1f}">lost lock entirely</text>')
    for dy, line in ((36, 'change in carrier-to-noise from the pad baseline to the burn, '
                          'per satellite. Doppler measured from RXM-RAWX on a '
                          'byte-identical scenario,'),
                     (24, 'so it is the same injected signal in all five runs. GPS:11 '
                          '(the highest rate of any satellite) is absent from the left '
                          'panel: its raw'),
                     (12, 'measurement dropped out through the burn.')):
        o.append(f'<text class="lbl" x="{PAD_L:.1f}" y="{H-dy:.1f}">{line}</text>')
    o.append('</svg>')

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(o))
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
