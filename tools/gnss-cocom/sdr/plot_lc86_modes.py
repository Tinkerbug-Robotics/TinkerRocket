#!/usr/bin/env python3
"""The LC86G in Normal, Balloon and Drone mode, same 3 g flight, same signal.

Two panels on the one clock, each with a single y-axis, then the lock strips:

    climb rate through the boost   injected and each mode  (m/s)
    altitude, the whole flight     injected and each mode  (km)
    lock state                     one strip per mode, the per-receiver states

Drone is dashed: through the boost it runs almost on top of Balloon, and the
dash keeps both visible (and tells them apart without colour).

Only valid fixes are drawn as reported values -- PQTMPVT FixMode >= 2 with a
nonzero <Quality>, which is what a flight driver reading it accepts -- so a gap
in a line is the receiver publishing nothing usable, and the strip says why.

    ./plot_lc86_modes.py        writes results/figures/lc86g_modes.svg
"""

from __future__ import annotations

import gzip
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from gnss_nmea_monitor import nmea_checksum_ok                      # noqa: E402
from correlate import (Truth, clock_outliers, collect, parse_start,  # noqa: E402
                       trajectory_time, wrong_fix)
from plot_flight import VERDICT_FILL, SILENT_FILL, silent_spans       # noqa: E402

START = "2026/08/18,08:30:00"
MODES = [("normal", "Normal", "var(--mode-normal, #eb6834)"),
         ("balloon", "Balloon", "var(--mode-balloon, #2a78d6)"),
         ("drone", "Drone", "var(--mode-drone, #1baf7a)")]
DASH = {"drone": ' stroke-dasharray="6 3"'}
TRUTH_INK = "var(--ink-2, #4A5261)"

W = 900
PAD_L, PAD_R = 62, 18
T0, T1 = 150.0, 848.0            # whole-flight window
B0, B1 = 176.0, 214.0            # the boost, for the climb-rate panel


def fixes(path):
    """[(traj_t, alt_m, climb_mps)] for every valid PQTMPVT fix in a capture."""
    tow0 = parse_start(START)[1]
    out = []
    for line in gzip.open(path, "rt", errors="replace"):
        _h, _, s = line.partition(" ")
        s = s.strip()
        if not s.startswith("$PQTMPVT") or not nmea_checksum_ok(s):
            continue
        f = s.split("*")[0].split(",")
        if (f[5] or "0") == "0" or (f[6] or "0") not in ("2", "3") or not f[11]:
            continue
        out.append((float(f[2]) / 1000 - tow0, float(f[11]),
                    -float(f[15]) if f[15] else None))
    out.sort()
    return out


def states(path, truth):
    """Lock-strip rows [(t, verdict)] and silent spans, as plot_flight draws them."""
    import tempfile
    start = parse_start(START)
    with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as tmp:
        tmp.write(gzip.open(path, "rt", errors="replace").read())
    samples = collect(tmp.name)
    stale = clock_outliers(samples, start)
    rows = []
    for i, s in enumerate(samples):
        t, src = trajectory_time(s, start, None)
        if src == "host" or i in stale or not (T0 <= t <= T1):
            continue
        v = s[4]
        if v == "FIX" and wrong_fix(s[5], truth.at(t)):
            v = "WRONG"
        rows.append((t, v))
    rows.sort()
    return rows, silent_spans(tmp.name, samples, start, stale)


def polyline_segments(pts, xf, yf, gap=0.6):
    """Split into runs wherever consecutive points are further apart than gap."""
    segs, cur = [], []
    for i, (t, v) in enumerate(pts):
        if cur and t - pts[i - 1][0] > gap:
            segs.append(cur)
            cur = []
        cur.append(f"{xf(t):.1f},{yf(v):.1f}")
    if cur:
        segs.append(cur)
    return [s for s in segs if len(s) > 1]


def build() -> str:
    truth = Truth(json.loads((RES / "lc86g_balloon_gentle_alt.scenario.json").read_text()))
    data = {m: fixes(RES / f"lc86g_{m}_gentle_alt.log.gz") for m, _, _ in MODES}
    st = {m: states(RES / f"lc86g_{m}_gentle_alt.log.gz", truth) for m, _, _ in MODES}

    pw = W - PAD_L - PAD_R
    y_a0, h_a = 30, 150           # climb-rate panel
    y_b0, h_b = y_a0 + h_a + 46, 150   # altitude panel
    y_s0 = y_b0 + h_b + 40        # strips
    strip_h, strip_gap = 14, 22
    y_leg = y_s0 + len(MODES) * strip_gap + 34
    H = y_leg + (len(MODES) + 2) * 14 + 16

    xa = lambda t: PAD_L + pw * (t - B0) / (B1 - B0)
    xb = lambda t: PAD_L + pw * (t - T0) / (T1 - T0)
    ya = lambda v: y_a0 + h_a * (1 - max(-20.0, min(v, 540.0)) / 540.0)
    yb = lambda a: y_b0 + h_b * (1 - a / 90000.0)

    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         'aria-label="The LC86G on the same 3 g flight in Normal, Balloon and Drone '
         'mode. Balloon and Drone report the climb rate and altitude of the boost; '
         'Normal reports a climb rate near zero for the first eighteen seconds. All '
         'three stop all output at 500 meters per second; only Balloon publishes a '
         'valid fix again.">',
         '<style>.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-3,#79808F);'
         'paint-order:stroke;stroke:var(--surface,#FFFFFF);stroke-width:3px}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;font-weight:600;'
         'fill:var(--ink-2,#4A5261)}'
         '.note{font-family:var(--f-mono,monospace);font-size:9.5px;fill:var(--ink-2,#4A5261);'
         'paint-order:stroke;stroke:var(--surface,#FFFFFF);stroke-width:3px}'
         '.tr{fill:none;stroke-width:1.8;stroke-linejoin:round}</style>']

    # ---- panel a: climb rate through the boost
    o.append(f'<text class="ttl" x="{PAD_L}" y="{y_a0-10}">climb rate through the '
             f'3 g boost (m/s)</text>')
    for v in (0, 250, 500):
        o.append(f'<line class="gl" x1="{PAD_L}" y1="{ya(v):.1f}" x2="{W-PAD_R}" '
                 f'y2="{ya(v):.1f}"/>')
        o.append(f'<text class="lbl" x="{PAD_L-6}" y="{ya(v)+3:.1f}" '
                 f'text-anchor="end">{v}</text>')
    o.append(f'<line x1="{PAD_L}" y1="{ya(500):.1f}" x2="{W-PAD_R}" y2="{ya(500):.1f}" '
             f'stroke="var(--ink-3,#79808F)" stroke-width="1" stroke-dasharray="3 3"/>')
    o.append(f'<text class="note" x="{PAD_L+6}" y="{ya(500)-5:.1f}">'
             f'500 m/s: all three modes stop ALL output</text>')
    tru = [(s["t"], s["v_up_mps"]) for s in truth.samples if B0 <= s["t"] <= B1]
    o.append(f'<polyline class="tr" stroke="{TRUTH_INK}" stroke-width="1.2" points="'
             + " ".join(f"{xa(t):.1f},{ya(v):.1f}" for t, v in tru) + '"><title>injected'
             '</title></polyline>')
    for m, name, col in MODES:
        pts = [(t, c) for t, _a, c in data[m] if B0 <= t <= B1 and c is not None]
        for seg in polyline_segments(pts, xa, ya):
            o.append(f'<polyline class="tr" stroke="{col}"{DASH.get(m, "")} '
                     f'points="{" ".join(seg)}">'
                     f'<title>{name}: reported climb rate</title></polyline>')
    o.append(f'<text class="note" x="{xa(177):.1f}" y="{ya(430):.1f}">Normal (orange) '
             f'reads near zero for 18 s</text>')
    o.append(f'<text class="note" x="{xa(177):.1f}" y="{ya(430)+13:.1f}">while the '
             f'vehicle climbs to 320 m/s</text>')
    for t in (180, 190, 200, 210):
        o.append(f'<text class="lbl" x="{xa(t):.1f}" y="{y_a0+h_a+13}" '
                 f'text-anchor="middle">{t}s</text>')

    # ---- panel b: altitude, whole flight
    o.append(f'<text class="ttl" x="{PAD_L}" y="{y_b0-10}">altitude, the whole flight '
             f'(km)</text>')
    for a in (0, 40000, 80000):
        o.append(f'<line class="gl" x1="{PAD_L}" y1="{yb(a):.1f}" x2="{W-PAD_R}" '
                 f'y2="{yb(a):.1f}"/>')
        o.append(f'<text class="lbl" x="{PAD_L-6}" y="{yb(a)+3:.1f}" '
                 f'text-anchor="end">{a//1000}</text>')
    o.append(f'<line x1="{PAD_L}" y1="{yb(80000):.1f}" x2="{W-PAD_R}" y2="{yb(80000):.1f}" '
             f'stroke="var(--ink-3,#79808F)" stroke-width="1" stroke-dasharray="3 3"/>')
    o.append(f'<text class="note" x="{W-PAD_R}" y="{yb(80000)-5:.1f}" text-anchor="end">'
             f'80 km: Balloon stops all output (its own altitude)</text>')
    tru = [(s["t"], s["alt_m"]) for s in truth.samples if T0 <= s["t"] <= T1][::5]
    o.append(f'<polyline class="tr" stroke="{TRUTH_INK}" stroke-width="1.2" points="'
             + " ".join(f"{xb(t):.1f},{yb(a):.1f}" for t, a in tru) + '"><title>injected'
             '</title></polyline>')
    for m, name, col in MODES:
        pts = [(t, a) for t, a, _c in data[m] if T0 <= t <= T1][::3]
        for seg in polyline_segments(pts, xb, yb, gap=1.0):
            o.append(f'<polyline class="tr" stroke="{col}"{DASH.get(m, "")} '
                     f'points="{" ".join(seg)}">'
                     f'<title>{name}: reported altitude</title></polyline>')
    for frac in (0, .25, .5, .75, 1):
        t = T0 + (T1 - T0) * frac
        o.append(f'<text class="lbl" x="{xb(t):.1f}" y="{y_b0+h_b+13}" '
                 f'text-anchor="middle">{t:.0f}s</text>')

    # ---- lock strips
    o.append(f'<text class="ttl" x="{PAD_L}" y="{y_s0-8}">lock state</text>')
    for k, (m, name, col) in enumerate(MODES):
        ys = y_s0 + k * strip_gap
        rows, silent = st[m]
        spans = []
        for i, (t, v) in enumerate(rows):
            nxt = rows[i + 1][0] if i + 1 < len(rows) else min(t + 1.0, T1)
            for s0, _s1 in silent:
                if t < s0 < nxt:
                    nxt = s0
            if spans and spans[-1][2] == v and abs(spans[-1][1] - t) < 3.0:
                spans[-1][1] = nxt
            else:
                spans.append([t, nxt, v])
        for a, b, v in spans:
            o.append(f'<rect x="{xb(a):.1f}" y="{ys:.1f}" width="{max(1.0, xb(b)-xb(a)):.1f}" '
                     f'height="{strip_h}" fill="{VERDICT_FILL[v]}"><title>{name}: {v} '
                     f'{a:.1f}-{b:.1f} s</title></rect>')
        for s0, s1 in silent:
            a, b = max(s0, T0), min(s1, T1)
            if b > a:
                o.append(f'<rect x="{xb(a):.1f}" y="{ys:.1f}" width="{max(1.0, xb(b)-xb(a)):.1f}" '
                         f'height="{strip_h}" fill="{SILENT_FILL}"><title>{name}: SILENT '
                         f'{a:.1f}-{b:.1f} s</title></rect>')
        # a line sample, not a square: squares are the strip states, and Drone's
        # aqua sits near FIX's teal
        o.append(f'<line x1="{PAD_L-60}" y1="{ys+7:.1f}" x2="{PAD_L-47}" y2="{ys+7:.1f}" '
                 f'stroke="{col}" stroke-width="2"{DASH.get(m, "")}/>')
        o.append(f'<text class="lbl" x="{PAD_L-43}" y="{ys+11:.1f}">{name.split(" ")[0]}</text>')

    # ---- legend: the lines stacked, then the strip states in one row
    items = [(TRUTH_INK, "injected trajectory (ground truth)", "")] + \
            [(c, f"{n}: reported, valid fixes only", DASH.get(m, "")) for m, n, c in MODES]
    for i, (c, label, dash) in enumerate(items):
        yy = y_leg + i * 14
        o.append(f'<line x1="{PAD_L}" y1="{yy-3}" x2="{PAD_L+16}" y2="{yy-3}" '
                 f'stroke="{c}" stroke-width="2"{dash}/>')
        o.append(f'<text class="lbl" x="{PAD_L+22}" y="{yy}">{label}</text>')
    yy = y_leg + len(items) * 14 + 6
    for i, (key, label) in enumerate((("FIX", "FIX"), ("WRONG", "WRONG fix"),
                                      ("BLOCKED", "BLOCKED"), ("NO_LOCK", "NO_LOCK"),
                                      ("SILENT", "SILENT, no output"))):
        lx = PAD_L + i * 150
        fill = SILENT_FILL if key == "SILENT" else VERDICT_FILL[key]
        o.append(f'<rect x="{lx}" y="{yy-7}" width="16" height="8" fill="{fill}"/>')
        o.append(f'<text class="lbl" x="{lx+22}" y="{yy}">{label}</text>')
    o.append("</svg>")
    return "\n".join(o)


def main() -> int:
    out = RES / "figures" / "lc86g_modes.svg"
    out.write_text(build() + "\n")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
