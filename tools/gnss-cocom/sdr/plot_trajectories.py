#!/usr/bin/env python3
"""Altitude, speed and acceleration of the two boosts in cn0_boost_report.html.

Left, the flight from ignition to T+300 s, which covers every COCOM window either
trajectory opens; right, the burn. Acceleration is what an accelerometer on the
rocket reads (thrust minus drag, vertical): the kinematic acceleration that sets a
satellite's Doppler rate is 1 g less during the burn. Read from the scenario truth
(scenarios/<name>.json, 10 Hz), so the figure is the files the receivers were fed.

  python3 plot_trajectories.py      # writes results/figures/cn0_trajectories.svg
"""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "figures" / "cn0_trajectories.svg"
G = 9.80665

# (scenario, label, colour); blue and orange stay apart in both themes and under
# colour-vision deficiency, and the report's status hues stay free for the gates
TRAJ = [("traveler", "traveler", "var(--accent, #29457E)"),
        ("hotshot", "hotshot", "var(--mode-normal, #eb6834)")]

W, H = 900, 548
TOP = 40                        # column titles above
PH, GAP = 124, 22               # panel height, gap between rows
COLS = [dict(x0=58, w=430, t0=-10.0, t1=300.0, ticks=range(0, 301, 50),
             title="The flight, ignition to T+300 s"),
        dict(x0=548, w=334, t0=-0.5, t1=14.0, ticks=range(0, 15, 2),
             title="The burn, T−0.5 to T+14 s")]
ROWS = [dict(key="alt", label="altitude, km", lim=[(0, 110, 20), (0, 12, 2)]),
        dict(key="spd", label="speed, m/s", lim=[(0, 1600, 400), (0, 1600, 400)]),
        dict(key="acc", label="acceleration felt, g", lim=[(-5, 40, 10), (-5, 40, 10)])]


def load(name):
    d = json.loads((HERE / "scenarios" / f"{name}.json").read_text())
    pro = d.get("prologue_s", 180.0)
    tr = d["truth"]
    s = {"alt": [(p["t"] - pro, p["alt_m"] / 1000.0) for p in tr],
         "spd": [(p["t"] - pro, p["speed_mps"]) for p in tr],
         "acc": [((a["t"] + b["t"]) / 2 - pro,
                  ((b["v_up_mps"] - a["v_up_mps"]) / (b["t"] - a["t"]) + G) / G)
                 for a, b in zip(tr, tr[1:])]}
    burn = next(p["t"] for p in tr if p["phase"] == "coast") - pro
    return s, burn, d


def decimate(pts, t0, t1, px):
    """Keep each pixel column's first, min, max and last point, so a 10 Hz spike survives."""
    cols = {}
    for t, v in pts:
        if t0 <= t <= t1:
            cols.setdefault(int((t - t0) / (t1 - t0) * px), []).append((t, v))
    out = []
    for k in sorted(cols):
        c = cols[k]
        keep = {c[0], min(c, key=lambda p: p[1]), max(c, key=lambda p: p[1]), c[-1]}
        out.extend(sorted(keep))
    return out


def crossing(pts, level, after=0.0):
    for (ta, va), (tb, vb) in zip(pts, pts[1:]):
        if ta >= after and va < level <= vb:
            return ta + (level - va) / (vb - va) * (tb - ta)
    return None


def main():
    data = {name: load(name) for name, _l, _c in TRAJ}
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         f'aria-label="Altitude, speed and felt acceleration against time for the traveler and '
         f'hotshot trajectories: the whole flight to T+300 seconds on the left, the burn on the '
         f'right, with the 515 metres per second, 18 km and 80 km COCOM lines.">',
         '<style>'
         '.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.gl{stroke:var(--rule,#DDE2E9);stroke-width:1;stroke-dasharray:3 3}'
         '.gate{stroke:var(--blocked,#A2660A);stroke-width:1;stroke-dasharray:5 3}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-3,#79808F)}'
         '.gtl{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--blocked,#A2660A)}'
         '.val{font-family:var(--f-mono,monospace);font-size:9px;fill:var(--ink-2,#4A5261)}'
         '.ttl{font-family:var(--f-display,sans-serif);font-size:11px;font-weight:600;'
         'fill:var(--ink-2,#4A5261)}'
         '.trace{fill:none;stroke-width:1.8;stroke-linejoin:round;stroke-linecap:round}'
         '</style>']

    for ci, col in enumerate(COLS):
        x0, w, t0, t1 = col["x0"], col["w"], col["t0"], col["t1"]

        def X(t, x0=x0, w=w, t0=t0, t1=t1):
            return x0 + (min(max(t, t0), t1) - t0) / (t1 - t0) * w

        o.append(f'<text class="ttl" x="{x0}" y="{TOP - 18}">{col["title"]}</text>')
        for ri, row in enumerate(ROWS):
            y0 = TOP + ri * (PH + GAP)
            lo, hi, step = row["lim"][ci]

            def Y(v, y0=y0, lo=lo, hi=hi):
                return y0 + (hi - min(max(v, lo), hi)) / (hi - lo) * PH

            v = math.ceil(lo / step) * step          # ticks on round values, not on the floor
            while v <= hi + 1e-9:
                o.append(f'<line class="gl" x1="{x0}" y1="{Y(v):.1f}" x2="{x0 + w}" y2="{Y(v):.1f}"/>')
                o.append(f'<text class="lbl" x="{x0 - 6}" y="{Y(v) + 3:.1f}" text-anchor="end">'
                         f'{v:g}</text>')
                v += step
            o.append(f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y0 + PH}"/>')
            base = Y(0) if lo < 0 else y0 + PH
            o.append(f'<line class="ax" x1="{x0}" y1="{base:.1f}" x2="{x0 + w}" y2="{base:.1f}"/>')
            o.append(f'<text class="lbl" transform="translate({x0 - 36},{y0 + PH / 2:.0f}) rotate(-90)" '
                     f'text-anchor="middle">{row["label"]}</text>')
            if ri == len(ROWS) - 1:
                for t in col["ticks"]:
                    o.append(f'<text class="lbl" x="{X(t):.1f}" y="{y0 + PH + 14}" '
                             f'text-anchor="middle">{t}</text>')
                o.append(f'<text class="lbl" x="{x0 + w / 2:.0f}" y="{y0 + PH + 29}" '
                         f'text-anchor="middle">seconds from ignition</text>')

            # the COCOM lines: 515 m/s on speed, 18 km (and 80 km) on altitude
            # each label sits by a stretch of its line that no trace crosses: over it in the
            # flight column, under its right end in the burn column (both traces are above there)
            gates = {"spd": [(515.0, "515 m/s", 150.0)],
                     "alt": [(18.0, "18 km", 200.0), (80.0, "80 km", 20.0)]}
            for gv, gl, tl in gates.get(row["key"], []):
                if lo < gv < hi:
                    o.append(f'<line class="gate" x1="{x0}" y1="{Y(gv):.1f}" x2="{x0 + w}" '
                             f'y2="{Y(gv):.1f}"/>')
                    if ci == 0:
                        o.append(f'<text class="gtl" x="{X(tl):.1f}" y="{Y(gv) - 4:.1f}">{gl}</text>')
                    else:
                        o.append(f'<text class="gtl" x="{x0 + w - 2}" y="{Y(gv) + 11:.1f}" '
                                 f'text-anchor="end">{gl}</text>')

            for name, label, colr in TRAJ:
                s, burn, _d = data[name]
                pts = s[row["key"]]
                pts = decimate(pts, t0, t1, w) if ci == 0 else [p for p in pts if t0 <= p[0] <= t1]
                o.append(f'<polyline class="trace" stroke="{colr}" points="' +
                         " ".join(f"{X(t):.1f},{Y(v):.1f}" for t, v in pts) + '"/>')

    # annotations: where each passes 515 m/s (burn), apogee (flight), peak g (burn)
    def xy(ci, ri, t, v):
        col, row = COLS[ci], ROWS[ri]
        lo, hi, _s = row["lim"][ci]
        y0 = TOP + ri * (PH + GAP)
        x = col["x0"] + (min(max(t, col["t0"]), col["t1"]) - col["t0"]) / (col["t1"] - col["t0"]) * col["w"]
        return x, y0 + (hi - min(max(v, lo), hi)) / (hi - lo) * PH

    notes = {"traveler": dict(dx515=6, dy515=16, anchor515="start"),
             "hotshot": dict(dx515=-6, dy515=-8, anchor515="end")}
    for name, label, colr in TRAJ:
        s, burn, _d = data[name]
        n = notes[name]
        t515 = crossing(s["spd"], 515.0)
        cx, cy = xy(1, 1, t515, 515.0)
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="3.5" fill="{colr}" '
                 f'stroke="var(--surface,#FFFFFF)" stroke-width="1.5"/>')
        o.append(f'<text class="val" x="{cx + n["dx515"]:.1f}" y="{cy + n["dy515"]:.1f}" '
                 f'text-anchor="{n["anchor515"]}">{label} T+{t515:.1f} s</text>')
        ta, va = max(s["alt"], key=lambda p: p[1])
        ax_, ay_ = xy(0, 0, ta, va)
        o.append(f'<circle cx="{ax_:.1f}" cy="{ay_:.1f}" r="3.5" fill="{colr}" '
                 f'stroke="var(--surface,#FFFFFF)" stroke-width="1.5"/>')
        o.append(f'<text class="val" x="{ax_ + 6:.1f}" y="{ay_ - 6:.1f}">{label}: apogee '
                 f'{va:.1f} km, T+{ta:.0f} s</text>')
        tb, vb = max((p for p in s["acc"] if 0 <= p[0] <= burn), key=lambda p: p[1])
        bx, by = xy(1, 2, tb, vb)
        right = bx < COLS[1]["x0"] + COLS[1]["w"] / 2
        o.append(f'<text class="val" x="{bx + (8 if right else -4):.1f}" y="{by + (4 if right else -6):.1f}" '
                 f'text-anchor="{"start" if right else "end"}">{label}: {vb:.0f} g at burnout, '
                 f'T+{burn:.0f} s</text>')
        # where it comes back under 515 m/s on the way up (the hotshot's window closes in the flight)
        back = next((t for (ta_, va_), (t, v) in zip(s["spd"], s["spd"][1:])
                     if ta_ > t515 and va_ >= 515.0 > v), None)
        if back is not None:
            bx2, by2 = xy(0, 1, back, 515.0)
            o.append(f'<circle cx="{bx2:.1f}" cy="{by2:.1f}" r="3.5" fill="{colr}" '
                     f'stroke="var(--surface,#FFFFFF)" stroke-width="1.5"/>')
            o.append(f'<text class="val" x="{bx2 + 5:.1f}" y="{by2 - 5:.1f}">T+{back:.0f} s</text>')

    # legend, one row under the plots
    ly = H - 16
    lx = COLS[0]["x0"]
    for name, label, colr in TRAJ:
        _s, burn, d = data[name]
        o.append(f'<line x1="{lx}" y1="{ly - 3}" x2="{lx + 18}" y2="{ly - 3}" stroke="{colr}" '
                 f'stroke-width="2.4" stroke-linecap="round"/>')
        txt = (f"{label}: {burn:.0f} s burn, peak {d['crossings']['peak_speed_mps']:,.0f} m/s, "
               f"apogee {d['crossings']['peak_alt_m'] / 1000:.1f} km")
        o.append(f'<text class="lbl" x="{lx + 24}" y="{ly}">{txt}</text>')
        lx += 24 + len(txt) * 5.45 + 26
    o.append(f'<line class="gate" x1="{lx}" y1="{ly - 3}" x2="{lx + 18}" y2="{ly - 3}"/>')
    o.append(f'<text class="lbl" x="{lx + 24}" y="{ly}">COCOM lines</text>')
    o.append('</svg>')

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(o) + "\n")
    print(f"  {OUT.relative_to(HERE)}  ({OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
