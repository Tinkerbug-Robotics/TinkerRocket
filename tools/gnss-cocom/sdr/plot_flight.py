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

W, H = 900, 596
LEG_H = 40
SAT_H = 30
PAD_L, PAD_R, PAD_T, PAD_B = 62, 18, 26, 54 + LEG_H
STRIP_H = 16
GAP = 26

VERDICT_FILL = {"FIX": "var(--fix, #0E7C66)",
                "BLOCKED": "var(--blocked, #A2660A)",
                "NO_LOCK": "var(--nolock, #9B3535)"}

# Spelled out in the figure rather than left to a caption: the whole point of the
# strip is telling a gate closure from a lost signal, and "BLOCKED" on its own
# does not say which is which.
VERDICT_MEANING = [
    ("FIX", "position reported"),
    ("BLOCKED", "position withheld, satellites still tracked  =  the gate"),
    ("NO_LOCK", "satellites lost  =  a signal problem, not the gate"),
]


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def build_svg(meta, rows, truth):
    dur = meta["duration_s"]
    # Begin a little before ignition. The prologue exists so the receiver can
    # acquire before the flight starts; on the plot it is 180 s of flat line
    # that squeezes everything interesting into the right-hand third.
    LEAD_IN = 20.0
    t0 = max(0.0, meta.get("prologue_s", 0.0) - LEAD_IN)
    truth = [s_ for s_ in truth if s_["t"] >= t0]
    rows = [r for r in rows if r[0] >= t0]
    alt_max = max(s["alt_m"] for s in truth) / 1000.0
    spd_max = max(s["speed_mps"] for s in truth)
    alt_top = max(10.0, alt_max * 1.12)
    spd_top = max(100.0, spd_max * 1.12)

    plot_w = W - PAD_L - PAD_R
    panel_h = (H - PAD_T - PAD_B - STRIP_H - SAT_H - 3 * GAP - 14) / 3
    y0_alt = PAD_T
    y0_spd = PAD_T + panel_h + GAP
    y0_acc = y0_spd + panel_h + GAP
    y0_strip = y0_acc + panel_h + GAP
    y0_sat = y0_strip + STRIP_H + 14

    def x(t):
        return PAD_L + plot_w * ((t - t0) / (dur - t0))

    def y_alt(a_km):
        return y0_alt + panel_h * (1 - a_km / alt_top)

    def y_spd(v):
        return y0_spd + panel_h * (1 - v / spd_top)

    # Acceleration, differentiated from the injected velocity. This panel is
    # here because the satellite bar dips are not explained by speed or
    # altitude: what breaks tracking is Doppler *rate*, which is proportional
    # to acceleration. 1 g on L1 is 51 Hz/s; the burn is 15 times that.
    acc = [0.0]
    for i in range(1, len(truth)):
        dt = truth[i]["t"] - truth[i - 1]["t"]
        dv = truth[i]["v_up_mps"] - truth[i - 1]["v_up_mps"]
        acc.append(dv / dt if dt > 0 else 0.0)
    if len(acc) > 8:                       # light smoothing; 10 Hz is noisy
        sm = []
        for i in range(len(acc)):
            lo, hi = max(0, i - 4), min(len(acc), i + 5)
            sm.append(sum(acc[lo:hi]) / (hi - lo))
        acc = sm
    acc_top = max(10.0, max(abs(a) for a in acc) * 1.12)

    def y_acc(a):
        return y0_acc + panel_h * (1 - (a + acc_top) / (2 * acc_top))

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
    # Shade the limit on the panel that is over it, not across both. Reading a
    # red band behind the altitude trace invites the conclusion that altitude
    # was the problem, when it was speed; when a flight is over both at once,
    # both panels shade and that is the honest picture.
    for wins, fill, y_top, op in ((meta.get("velocity_windows") or [],
                                   "var(--nolock, #9B3535)", y0_spd, 0.16),
                                  (meta.get("altitude_80km_windows") or [],
                                   "var(--blocked, #A2660A)", y0_alt, 0.30)):
        for a, b in wins:
            if b < t0:
                continue
            out.append(f'<rect x="{x(max(a, t0)):.1f}" y="{y_top:.1f}" '
                       f'width="{max(1.0, x(b)-x(max(a, t0))):.1f}" '
                       f'height="{panel_h:.1f}" '
                       f'fill="{fill}" opacity="{op}"/>')

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
    pacc = " ".join(f"{x(truth[i]['t']):.1f},{y_acc(acc[i]):.1f}"
                    for i in range(0, len(truth), step))
    out.append(f'<line class="ax" x1="{PAD_L}" y1="{y0_acc+panel_h:.1f}" '
               f'x2="{W-PAD_R}" y2="{y0_acc+panel_h:.1f}"/>')
    out.append(f'<line class="ax" x1="{PAD_L}" y1="{y0_acc:.1f}" '
               f'x2="{PAD_L}" y2="{y0_acc+panel_h:.1f}"/>')
    out.append(f'<line class="gl" x1="{PAD_L}" y1="{y_acc(0):.1f}" '
               f'x2="{W-PAD_R}" y2="{y_acc(0):.1f}"/>')
    out.append(f'<polyline class="trace" points="{pacc}" '
               f'stroke="var(--blocked, #A2660A)"/>')
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_acc-8:.1f}">'
               f'acceleration (m/s&#178;) &#183; Doppler rate = 5.25 Hz/s per m/s&#178;'
               f'</text>')
    for v, lab in ((acc_top, f"{acc_top:.0f}"), (0.0, "0"), (-acc_top, f"-{acc_top:.0f}")):
        out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y_acc(v)+3:.1f}" '
                   f'text-anchor="end">{lab}</text>')

    # lock strip: one bar per epoch, width set by the gap to the next
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_strip-5:.1f}">'
               f'receiver lock state</text>')
    # Merge runs of the same verdict into one rect. One rect per epoch is ~40 kB
    # of markup for a 470 s flight, and it renders as hairline seams where
    # adjacent bars round to different pixels.
    spans = []
    for i, (t, verdict, _n) in enumerate(rows):
        nxt = rows[i + 1][0] if i + 1 < len(rows) else min(t + 1.0, dur)
        if spans and spans[-1][2] == verdict and abs(spans[-1][1] - t) < 3.0:
            spans[-1][1] = nxt
        else:
            spans.append([t, nxt, verdict])
    for a, b, verdict in spans:
        out.append(f'<rect x="{x(a):.1f}" y="{y0_strip:.1f}" '
                   f'width="{max(1.0, x(b)-x(a)):.1f}" '
                   f'height="{STRIP_H}" fill="{VERDICT_FILL[verdict]}"/>')

    # satellites actually tracked, under the lock strip. The generator always
    # transmits the same ~10 at equal power, so every dip here is the receiver
    # losing them -- which is what separates "the gate shut" from "the signal
    # went away", and it is the first thing to check when a latency looks long.
    sat_max = max((n for _, _, n in rows), default=1) or 1
    sat_max = max(sat_max, 8)
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_sat-4:.1f}">'
               f'satellites tracked (dashed = 4, the minimum for a 3-D fix)</text>')
    out.append(f'<line class="ax" x1="{PAD_L}" y1="{y0_sat+SAT_H:.1f}" '
               f'x2="{W-PAD_R}" y2="{y0_sat+SAT_H:.1f}"/>')
    y4 = y0_sat + SAT_H * (1 - 4.0 / sat_max)
    out.append(f'<line class="gl" x1="{PAD_L}" y1="{y4:.1f}" '
               f'x2="{W-PAD_R}" y2="{y4:.1f}"/>')
    for i, (t, _v, n) in enumerate(rows):
        nx = rows[i + 1][0] if i + 1 < len(rows) else min(t + 1.0, dur)
        h = SAT_H * (n / sat_max)
        if h <= 0:
            continue
        out.append(f'<rect x="{x(t):.1f}" y="{y0_sat+SAT_H-h:.1f}" '
                   f'width="{max(1.0, x(nx)-x(t)):.1f}" height="{h:.1f}" '
                   f'fill="var(--accent, #29457E)" opacity="0.55"/>')
    out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y0_sat+4:.0f}" '
               f'text-anchor="end">{sat_max}</text>')
    out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y0_sat+SAT_H:.0f}" '
               f'text-anchor="end">0</text>')

    # axis ticks
    for frac in (0, .25, .5, .75, 1.0):
        t = t0 + (dur - t0) * frac
        out.append(f'<text class="lbl" x="{x(t):.1f}" y="{y0_sat+SAT_H+13:.1f}" '
                   f'text-anchor="middle">{t:.0f}s</text>')
    for frac in (0, .5, 1.0):
        out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y_alt(alt_top*frac)+3:.1f}" '
                   f'text-anchor="end">{alt_top*frac:.0f}</text>')
        out.append(f'<text class="lbl" x="{PAD_L-6}" y="{y_spd(spd_top*frac)+3:.1f}" '
                   f'text-anchor="end">{spd_top*frac:.0f}</text>')
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_alt-8:.1f}">altitude (km)</text>')
    out.append(f'<text class="ttl" x="{PAD_L}" y="{y0_spd-8:.1f}">speed (m/s)</text>')

    ly = H - LEG_H + 4
    for i, (name, meaning) in enumerate(VERDICT_MEANING):
        yy = ly + i * 12
        out.append(f'<rect x="{PAD_L}" y="{yy-6:.0f}" width="16" height="8" '
                   f'fill="{VERDICT_FILL[name]}"/>')
        out.append(f'<text class="lbl" x="{PAD_L+22}" y="{yy+1:.0f}">'
                   f'{name} &#183; {esc(meaning)}</text>')
    lx = PAD_L + 400
    for i, (fill, op, meaning) in enumerate(
            [("var(--nolock, #9B3535)", 0.16,
              "shaded on the SPEED panel: over 515 m/s"),
             ("var(--blocked, #A2660A)", 0.30,
              "shaded on the ALTITUDE panel: over 80 km")]):
        yy = ly + i * 12
        out.append(f'<rect x="{lx}" y="{yy-6:.0f}" width="16" height="8" '
                   f'fill="{fill}" opacity="{op*1.6:.2f}"/>')
        out.append(f'<text class="lbl" x="{lx+22}" y="{yy+1:.0f}">{esc(meaning)}</text>')
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
        rows.append((t, s[4], s[5].tracked_sats()))
    rows.sort()
    if not rows:
        raise SystemExit(f"no placeable epochs in {args.capture}")

    # A capture only means anything against the trajectory it was actually
    # flown with. Regenerating a scenario with different parameters silently
    # invalidates every earlier capture of it, and the plot still renders --
    # just wrong. A capture that covers far less of the scenario than it should
    # is the signature.
    covered = (rows[-1][0] - rows[0][0]) / max(1.0, meta["duration_s"])
    if covered < 0.6:
        raise SystemExit(
            f"{args.capture.name} spans {rows[-1][0]-rows[0][0]:.0f}s of a "
            f"{meta['duration_s']:.0f}s scenario.\nThat capture was probably "
            f"flown against a different version of {args.scenario.name}; "
            f"re-plot it\nagainst the archived scenario it was flown with, or "
            f"re-fly it.")

    args.out.write_text(build_svg(meta, rows, truth.samples))
    n = {v: sum(1 for _, x, _n in rows if x == v) for v in VERDICT_FILL}
    print(f"{args.out}  ({len(rows)} epochs: "
          + ", ".join(f"{k} {v}" for k, v in n.items() if v) + ")")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
