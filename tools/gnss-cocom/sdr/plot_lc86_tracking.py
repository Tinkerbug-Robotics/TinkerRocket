#!/usr/bin/env python3
"""The LC86G tracking figures: carrier lock by navigation mode, bench and sky.

    results/figures/lc86g_mode_survey.svg    every valid mode, cold-started on the
                                             same first 3 minutes of the pad signal
    results/figures/lc86g_sky_coldstart.svg  the real sky: cold starts in Balloon,
                                             Drone, Balloon, Drone
    results/figures/lc86g_level_sweep.svg    Balloon mode against transmitted level

One row per satellite: its MSM7 C/N0, solid while the carrier loop holds phase and
dotted while the half-cycle flag says it does not; a triangle where the lock-time
counter restarts; a grey bar where the satellite is not measured at all.
"""
from __future__ import annotations

import json
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
OUT = RES / "figures"
sys.path.insert(0, str(HERE))
from lc86_tracking import Capture, MODE_NAME, metrics      # noqa: E402

W = 900
PAD_L, PAD_R = 74, 14
INK, INK3 = "var(--ink-2, #4A5261)", "var(--ink-3, #79808F)"
RESET = "var(--nolock, #9B3535)"
GAPC = "var(--rule-strong, #C3CAD5)"
MODE_COL = {3: "var(--mode-balloon, #2a78d6)", 5: "var(--mode-drone, #1baf7a)",
            0: "var(--mode-normal, #eb6834)"}
MODE_TINT = {3: "var(--mode-balloon, #2a78d6)", 5: "var(--mode-drone, #1baf7a)",
             0: "var(--mode-normal, #eb6834)"}
# The report's `.panel svg text {font-family: mono; fill: ink-3}` outranks any
# scoped class rule and a fill attribute, so the colored headings carry inline style.
HD = "font-family:var(--f-display,sans-serif);font-size:11px;font-weight:600;"
STYLE = ('<style>.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-3,#79808F)}'
         '.sat{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-2,#4A5261)}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;font-weight:600;'
         'fill:var(--ink-2,#4A5261)}'
         '.hd{font-family:var(--f-display,sans-serif);font-size:11px;font-weight:600}'
         '.tr{fill:none;stroke:var(--ink-2,#4A5261);stroke-width:1.1;stroke-linejoin:round}'
         '.un{stroke-dasharray:1.2 2.2}'
         '.rs{fill:var(--nolock,#9B3535)}'
         '.gp{fill:var(--rule-strong,#C3CAD5)}</style>')


def satrow(o, rows, x, y0, h, t_lo, t_hi):
    """One satellite's row: C/N0 10-50 dBHz, lock state, resets, dropouts."""
    y = lambda cn: y0 + h - 2 - (h - 4) * (max(10.0, min(cn, 50.0)) - 10.0) / 40.0
    R = [(t, c) for t, c in rows if t_lo <= t <= t_hi]
    runs, cur, locked, prev_lock, last = [], [], None, None, None
    for t, c in R:
        if last is not None and t - last > 1.6:
            if cur:
                runs.append((locked, cur))
            cur, locked, prev_lock = [], None, None
            o.append(f'<rect class="gp" x="{x(last):.1f}" y="{y0 + h - 3:.1f}" '
                     f'width="{max(0.8, x(t) - x(last)):.1f}" height="3"/>')
        if prev_lock is not None and c["lock_ms"] < prev_lock:
            xx = x(t)
            o.append(f'<path class="rs" d="M{xx - 2.4:.1f},{y0 + 0.5:.1f}h4.8l-2.4,4z"/>')
        prev_lock, last = c["lock_ms"], t
        if not c.get("cn0"):
            continue
        state = not c["halfcyc"]
        pt = f"{x(t):.1f},{y(c['cn0']):.1f}"
        if locked is None or state == locked:
            cur.append(pt)
            locked = state
        else:
            runs.append((locked, cur + [pt]))
            cur, locked = [pt], state
    if cur:
        runs.append((locked, cur))
    for lk, pts in runs:
        if len(pts) > 1:
            o.append(f'<polyline class="tr{"" if lk else " un"}" points="{" ".join(pts)}"/>')


def legend(o, y, x0=PAD_L):
    o.append(f'<line class="tr" x1="{x0}" y1="{y - 3}" x2="{x0 + 18}" y2="{y - 3}"/>'
             f'<text class="lbl" x="{x0 + 24}" y="{y}">carrier locked</text>')
    x1 = x0 + 118
    o.append(f'<line class="tr un" x1="{x1}" y1="{y - 3}" x2="{x1 + 18}" y2="{y - 3}"/>'
             f'<text class="lbl" x="{x1 + 24}" y="{y}">no carrier lock (half-cycle flag)</text>')
    x2 = x1 + 214
    o.append(f'<path class="rs" d="M{x2 + 6:.1f},{y - 7:.1f}h4.8l-2.4,4z"/>'
             f'<text class="lbl" x="{x2 + 16}" y="{y}">lock reset</text>')
    x3 = x2 + 90
    o.append(f'<rect class="gp" x="{x3}" y="{y - 5}" width="16" height="3"/>'
             f'<text class="lbl" x="{x3 + 22}" y="{y}">not measured</text>')


def mode_survey():
    runs = [(m, Capture(RES / f"lc86g_mode{m}_pad_static.log.gz")) for m in (3, 0, 4, 1, 7, 5)]
    elev = {}
    for _m, cap in runs:
        elev.update(cap.elev)
    keys = sorted({k for _m, cap in runs for k in cap.sats if k != ("G", 13)},
                  key=lambda k: elev.get(k, 0))
    gap = 14
    col_w = (W - PAD_L - PAD_R - 5 * gap) / 6
    row_h, top = 19, 70
    H = top + 26 + len(keys) * row_h + 40
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         'aria-label="The LC86G in each of its six navigation modes, cold-started on the '
         'same three minutes of the simulated pad signal. Balloon, Normal and Stationary '
         'hold carrier lock on every satellite. Fitness and Swimming never gain carrier '
         'lock. Drone restarts carrier lock on every satellite many times a minute and '
         'drops satellites for seconds at a time.">', STYLE]
    for i, (m, cap) in enumerate(runs):
        x0 = PAD_L + i * (col_w + gap)
        x = lambda t, x0=x0, cap=cap: x0 + col_w * (t + cap.offset) / 185.0
        tint = MODE_TINT.get(m)
        if tint:
            o.append(f'<rect x="{x0:.1f}" y="{top - 4}" width="{col_w:.1f}" '
                     f'height="{26 + len(keys) * row_h + 4}" fill="{tint}" opacity="0.07"/>')
        o.append(f'<text class="hd" x="{x0 + col_w / 2:.1f}" y="{top - 34}" '
                 f'text-anchor="middle" style="{HD}fill:{MODE_COL.get(m, INK)}">{MODE_NAME[m]}</text>')
        ff = cap.first_fix()
        w = metrics(cap, ff + 10, 184 - cap.offset)
        lockpct = 100 - (w["halfcyc"] or 0)
        o.append(f'<text class="lbl" x="{x0 + col_w / 2:.1f}" y="{top - 21}" '
                 f'text-anchor="middle">{lockpct:.0f}% locked</text>'
                 f'<text class="lbl" x="{x0 + col_w / 2:.1f}" y="{top - 10}" '
                 f'text-anchor="middle">{w["resets_min"]:.0f} resets/min</text>')
        # satellites used in the fix, a sparkline
        P = [(t, u) for t, _ft, _fm, u in cap.pvt if 0 <= t + cap.offset <= 185][::10]
        yu = lambda u: top + 20 - 18 * min(u, 15) / 15.0
        if P:
            o.append(f'<polyline class="tr" points="'
                     + " ".join(f"{x(t):.1f},{yu(u):.1f}" for t, u in P) + '"/>')
        for j, k in enumerate(keys):
            satrow(o, cap.sats.get(k, []), x, top + 26 + j * row_h, row_h - 3,
                   -cap.offset, 185 - cap.offset)
    o.append(f'<text class="lbl" x="{PAD_L - 6}" y="{top + 14}" text-anchor="end">used</text>')
    for j, k in enumerate(keys):
        yy = top + 26 + j * row_h + row_h / 2 + 1
        o.append(f'<text class="sat" x="{PAD_L - 6}" y="{yy:.1f}" text-anchor="end">'
                 f'{k[0]}{k[1]:02d} {elev.get(k, 0):>2}°</text>')
    yb = top + 26 + len(keys) * row_h + 12
    for i in range(6):
        x0 = PAD_L + i * (col_w + gap)
        for s, anc in ((0, "start"), (60, "middle"), (120, "middle"), (180, "end")):
            o.append(f'<text class="lbl" x="{x0 + col_w * s / 185:.1f}" y="{yb}" '
                     f'text-anchor="{anc}">{s}{"s" if s == 180 else ""}</text>')
    legend(o, yb + 22)
    o.append("</svg>")
    return "\n".join(o)


def sky_coldstart():
    cap = Capture(RES / "lc86g_sky_20260925.log.gz", start_mode=5)
    c0 = cap.colds[0] - 15
    c1 = cap.colds[-1] + 300
    keys = [k for k in cap.sats if k[0] in "GC"
            and sum(1 for t, _ in cap.sats[k] if c0 <= t <= c1) > 0.4 * (c1 - c0)]
    keys.sort(key=lambda k: ("GC".index(k[0]), -cap.elev.get(k, 0)))
    row_h, top = 19, 58
    H = top + 26 + len(keys) * row_h + 40
    pw = W - PAD_L - PAD_R
    x = lambda t: PAD_L + pw * (t - c0) / (c1 - c0)
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         'aria-label="The LC86G on the real sky, cold-started four times, five minutes '
         'each: Balloon, Drone, Balloon, Drone. In both Drone phases the GPS and BeiDou '
         'satellites restart carrier lock many times a minute; in both Balloon phases '
         'they hold it.">', STYLE]
    edges = cap.colds + [c1]
    for i, c in enumerate(cap.colds):
        m = cap.mode_at(c + 5)
        o.append(f'<rect x="{x(c):.1f}" y="{top - 4}" width="{x(edges[i + 1]) - x(c):.1f}" '
                 f'height="{26 + len(keys) * row_h + 4}" fill="{MODE_TINT.get(m, INK3)}" '
                 f'opacity="0.07"/>')
        o.append(f'<line class="ax" x1="{x(c):.1f}" y1="{top - 30}" x2="{x(c):.1f}" '
                 f'y2="{top + 26 + len(keys) * row_h}" stroke-dasharray="3 3"/>')
        s, e = c, edges[i + 1]
        # the same window lc86_tracking.py sky uses: first fix + 10 s to the next
        # switch, at most 300 s after the cold start
        stop = min([t for t, _m in cap.modes if t > c + 5] + [c + 300])
        ff = cap.first_fix(after=c + 3)
        w = metrics(cap, ff + 10, stop) if ff else None
        o.append(f'<text class="hd" x="{(x(s) + x(e)) / 2:.1f}" y="{top - 22}" '
                 f'text-anchor="middle" style="{HD}fill:{MODE_COL.get(m, INK)}">{MODE_NAME[m]}</text>')
        if w and w["s_resets"] is not None:
            o.append(f'<text class="lbl" x="{(x(s) + x(e)) / 2:.1f}" y="{top - 9}" '
                     f'text-anchor="middle">strong sats: {w["s_resets"]:.1f} resets/sat-min'
                     f'</text>')
    P = [(t, u) for t, _ft, _fm, u in cap.pvt if c0 <= t <= c1][::10]
    yu = lambda u: top + 20 - 18 * min(u, 30) / 30.0
    o.append('<polyline class="tr" points="' +
             " ".join(f"{x(t):.1f},{yu(u):.1f}" for t, u in P) + '"/>')
    o.append(f'<text class="lbl" x="{PAD_L - 6}" y="{top + 14}" text-anchor="end">used</text>')
    for j, k in enumerate(keys):
        satrow(o, cap.sats[k], x, top + 26 + j * row_h, row_h - 3, c0, c1)
        yy = top + 26 + j * row_h + row_h / 2 + 1
        o.append(f'<text class="sat" x="{PAD_L - 6}" y="{yy:.1f}" text-anchor="end">'
                 f'{k[0]}{k[1]:02d} {cap.elev.get(k, 0):>2}°</text>')
    yb = top + 26 + len(keys) * row_h + 12
    for mnt in range(0, int((c1 - c0) / 60) + 1, 5):
        o.append(f'<text class="lbl" x="{x(c0 + 60 * mnt):.1f}" y="{yb}" '
                 f'text-anchor="middle">{mnt} min</text>')
    legend(o, yb + 22)
    o.append("</svg>")
    return "\n".join(o)


def level_sweep():
    cap = Capture(RES / "lc86g_levels_pad_static.log.gz", start_mode=3)
    segs = json.loads((RES / "lc86g_levels_pad_static.schedule.json").read_text())["segments"]
    turn = min(range(len(segs)), key=lambda i: segs[i]["db"])
    pts = []
    for i, s in enumerate(segs):
        t0 = s["t0"] + (10 if s["t0"] > 0 else 60)
        w = metrics(cap, t0 - cap.offset, s["t1"] - cap.offset)
        pts.append(("down" if i <= turn else "up", s["db"], w))
    pw = W - PAD_L - PAD_R
    x = lambda db: PAD_L + pw * (db + 37.5) / 39.0
    panels = [("median C/N₀ (dBHz)", 150, lambda w: w["cn0"], 10, 48, (20, 30, 40)),
              ("carrier locked (% of measurements)", 90,
               lambda w: None if w["halfcyc"] is None else 100 - w["halfcyc"], 0, 100, (0, 50, 100)),
              ("fix (% of epochs)", 90, lambda w: w["fix"], 0, 100, (0, 50, 100))]
    H = 30 + sum(p[1] + 34 for p in panels) + 44
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         'aria-label="The LC86G in Balloon mode as the transmitted level is stepped down '
         'from HackRF gain 0 and back up. Reported carrier-to-noise follows the level one '
         'for one. Carrier lock holds to about 35 dBHz and is gone by 28; the fix holds to '
         'about 28 dBHz, is intermittent from 26 to 21, and is lost at 13.">', STYLE]
    y0 = 30
    col = {"down": "var(--mode-balloon, #2a78d6)", "up": "var(--mode-normal, #eb6834)"}
    dash = {"down": "", "up": ' stroke-dasharray="5 3"'}
    for title, h, fn, lo, hi, ticks in panels:
        y = lambda v, y0=y0, h=h, lo=lo, hi=hi: y0 + h * (1 - (v - lo) / (hi - lo))
        o.append(f'<text class="ttl" x="{PAD_L}" y="{y0 - 10}">{title}</text>')
        for tk in ticks:
            o.append(f'<line class="gl" x1="{PAD_L}" y1="{y(tk):.1f}" x2="{W - PAD_R}" '
                     f'y2="{y(tk):.1f}"/><text class="lbl" x="{PAD_L - 6}" y="{y(tk) + 3:.1f}" '
                     f'text-anchor="end">{tk}</text>')
        for leg in ("down", "up"):
            # C/N0 only while the receiver still had some fix: below that the median
            # is one or two channels of search noise
            P = [(db, fn(w)) for lg, db, w in pts if lg == leg and fn(w) is not None
                 and (not title.startswith("median") or w["fix"] > 0)]
            o.append(f'<polyline fill="none" stroke="{col[leg]}" stroke-width="1.8"{dash[leg]} '
                     'points="' + " ".join(f"{x(d):.1f},{y(v):.1f}" for d, v in P) + '"/>')
            for d, v in P:
                o.append(f'<circle cx="{x(d):.1f}" cy="{y(v):.1f}" r="2.6" fill="{col[leg]}">'
                         f'<title>{d:+.0f} dB, stepping {leg}: {v:.1f}</title></circle>')
        y0 += h + 34
    yb = y0 - 20
    for db in range(-36, 1, 6):
        o.append(f'<text class="lbl" x="{x(db):.1f}" y="{yb}" text-anchor="middle">'
                 f'{db:+d} dB</text>')
    o.append(f'<text class="lbl" x="{PAD_L + pw / 2:.1f}" y="{yb + 14}" text-anchor="middle">'
             'transmitted level, relative to HackRF gain 0 (the level every radiated flight used)'
             '</text>')
    ly = yb + 32
    o.append(f'<line x1="{PAD_L}" y1="{ly - 3}" x2="{PAD_L + 18}" y2="{ly - 3}" '
             f'stroke="{col["down"]}" stroke-width="1.8"/><text class="lbl" x="{PAD_L + 24}" '
             f'y="{ly}">stepping down, 45 s a step</text>'
             f'<line x1="{PAD_L + 190}" y1="{ly - 3}" x2="{PAD_L + 208}" y2="{ly - 3}" '
             f'stroke="{col["up"]}" stroke-width="1.8" stroke-dasharray="5 3"/>'
             f'<text class="lbl" x="{PAD_L + 214}" y="{ly}">stepping back up</text>')
    o.append("</svg>")
    return "\n".join(o)


def main() -> int:
    for name, fn in (("lc86g_mode_survey.svg", mode_survey),
                     ("lc86g_sky_coldstart.svg", sky_coldstart),
                     ("lc86g_level_sweep.svg", level_sweep)):
        (OUT / name).write_text(fn())
        print(OUT / name, f"{(OUT / name).stat().st_size / 1e3:.0f} kB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
