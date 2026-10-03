#!/usr/bin/env python3
"""The mosaic-G5's export limit on four flights side by side: true speed and altitude, the receiver's own speed and
height while it gives a fix, and shading where PVTGeodetic says "position output prohibited due to export laws"
(error 7). Its limit is 600 m/s with no altitude term, so the 80 km line has no effect and the shading follows the
speed alone. Also writes the edges (last fix before and first fix after each block, receiver and truth) as JSON.
Receiver time is tied to the truth per flight by its own climb rate (mosaic_gate.py's fit: Vu against the truth's
vertical speed while the fix is allowed, 20-450 m/s), since a transmitter underrun shifts it. Underruns from the
capture's .hackrf.txt are marked at the top of each panel: every gap in the fix after one is the rig's.

    mosaic_gate_chart.py OUT.png OUT.json LABEL=CAPTURE=SCENARIO[=END] [...]
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                         # noqa: E402

SDR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDR.parent))
import septentrio_sbf as sbf                                            # noqa: E402

IGN_TOW = 204000.0                       # ignition at 2026-08-18 08:40:00 GPST in every p180 SignalSim file
LIMIT = 600.0
INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
RX_C, LIM, BLOCK = "#a40e26", "#9a6700", "#cf222e"


def read(cap):
    """PVTGeodetic -> [(t from ignition, mode, error, speed, height km)]."""
    out = []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 24:
            continue
        h = p[2].strip()
        if (int(h[10:12] + h[8:10], 16) & 0x1FFF) != sbf.PVT_GEODETIC:
            continue
        q = sbf.pvt_geodetic(bytes.fromhex(h))
        if q["tow"] is None:
            continue
        sp = math.sqrt(q["vn"] ** 2 + q["ve"] ** 2 + q["vu"] ** 2) if q["mode"] and q["vn"] is not None else None
        out.append((q["tow"] / 1000.0 - IGN_TOW, q["mode"], q["error"], sp,
                    q["h"] / 1000.0 if q["mode"] and q["h"] is not None else None))
    return out


def vu_rows(cap):
    """PVTGeodetic -> [(t from ignition, Vu)] while fixed."""
    out = []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 24:
            continue
        h = p[2].strip()
        if (int(h[10:12] + h[8:10], 16) & 0x1FFF) != sbf.PVT_GEODETIC:
            continue
        q = sbf.pvt_geodetic(bytes.fromhex(h))
        if q["tow"] is not None and q["mode"] and q["vu"] is not None:
            out.append((q["tow"] / 1000.0 - IGN_TOW, q["vu"]))
    return out


def tie(cap, tt, tv):
    """Seconds to add to the receiver's time from ignition to get the truth's (0 if too few boost fixes)."""
    rows = vu_rows(cap)
    if not rows:
        return 0.0
    t_top = max(rows, key=lambda r: r[1])[0]
    fit = [(t, v) for t, v in rows if 20 <= v <= 450 and t <= t_top]
    if len(fit) < 5:
        return 0.0
    ft, fv = np.array(fit).T
    rise = (tt > -5) & (tt <= tt[int(np.argmax(tv))])
    offs = np.arange(-1.0, 1.0, 0.01)
    err = [np.median(np.abs(np.interp(ft + o, tt[rise], tv[rise]) - fv)) for o in offs]
    return float(offs[int(np.argmin(err))])


def underruns(cap):
    """Time from ignition of each transmitter underrun (-B per-second lines, ignition ~178.2 s into a p180 file)."""
    import re
    path = Path(str(cap) + ".hackrf.txt")
    if not path.exists():
        return []
    prev, out = 0, []
    lines = [x for x in path.read_text(errors="replace").splitlines() if "MB / " in x]
    for k, x in enumerate(lines[:-1], 1):
        m = re.search(r"(\d+) underruns", x)
        n = int(m.group(1)) if m else prev
        if n > prev:
            out.append(k - 178.2)
        prev = n
    return out


def stretches(pvt, cond):
    runs, cur = [], None
    for row in pvt:
        if cond(row) and cur is None:
            cur = [row[0], row[0]]
        elif cond(row):
            cur[1] = row[0]
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur:
        runs.append(cur)
    return runs


args = sys.argv[1:]
out_png, out_json = args[0], args[1]
flights = []
for a in args[2:]:
    label, cap, scen, *rest = a.split("=")
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    pro = sc["prologue_s"]
    tt = np.array([s["t"] - pro for s in sc["truth"]])
    tv = np.array([s["v_up_mps"] for s in sc["truth"]])
    dt = tie(cap, tt, tv)                                             # receiver -> truth time
    pvt = [(r[0] + dt,) + r[1:] for r in read(cap)]
    flights.append(dict(label=label, scen=scen, pvt=pvt, tt=tt, sp=np.array([s["speed_mps"] for s in sc["truth"]]),
                        alt=np.array([s["alt_m"] for s in sc["truth"]]) / 1000.0, tie=dt,
                        under=[u + dt for u in underruns(cap)], end=float(rest[0]) if rest else float(tt[-1])))

edges = []
for f in flights:
    fixes = [r for r in f["pvt"] if r[1] and r[3] is not None and r[0] >= -5]
    for t0, t1 in stretches(f["pvt"], lambda r: r[2] == 7):
        if t1 - t0 < 0.5:
            continue                                                  # underrun blips (none on these four)
        before = [r for r in fixes if r[0] < t0]
        after = [r for r in fixes if r[0] > t1]
        row = dict(flight=f["label"], start=round(t0, 2), end=round(t1, 2))
        if before:
            b = before[-1]
            row.update(up_t=round(b[0], 2), up_rx=round(b[3], 1), up_truth=round(float(np.interp(b[0], f["tt"], f["sp"])), 1),
                       up_blocked_truth=round(float(np.interp(t0, f["tt"], f["sp"])), 1),
                       up_alt=round(float(np.interp(t0, f["tt"], f["alt"])), 2))
        if after:
            c = after[0]
            row.update(down_t=round(c[0], 2), down_rx=round(c[3], 1),
                       down_truth=round(float(np.interp(c[0], f["tt"], f["sp"])), 1),
                       down_blocked_truth=round(float(np.interp(t1, f["tt"], f["sp"])), 1),
                       down_alt=round(float(np.interp(c[0], f["tt"], f["alt"])), 2))
        edges.append(row)
    above = [r for r in fixes if r[4] is not None and r[4] > 80.0]
    edges.append(dict(flight=f["label"], fixes_above_80km=len(above),
                      top_fix_km=round(max((r[4] for r in above), default=0.0), 2), tie_s=round(f["tie"], 2),
                      underruns=[round(u, 1) for u in f["under"]]))
Path(out_json).write_text(json.dumps(edges, indent=1))

plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": GRID})
n = len(flights)
fig, axs = plt.subplots(2, n, figsize=(3.5 * n + 1.0, 6.4), sharey="row",
                        gridspec_kw={"hspace": 0.12, "wspace": 0.08})
axs = np.array(axs).reshape(2, n)
for k, f in enumerate(flights):
    x0, x1 = -10.0, f["end"]
    blocked = [(a, b) for a, b in stretches(f["pvt"], lambda r: r[2] == 7) if b - a >= 0.5]
    fixes = [r for r in f["pvt"] if r[1] and r[3] is not None]
    for row, (key, idx, lim, unit) in enumerate((("sp", 3, LIMIT, "m/s"), ("alt", 4, 80.0, "km"))):
        ax = axs[row][k]
        for a, b in blocked:
            ax.axvspan(a, b, color=BLOCK, alpha=0.10, lw=0)
        ax.plot(f["tt"], f[key], color=INK, lw=1.2)
        ax.plot([r[0] for r in fixes], [r[idx] for r in fixes], ".", ms=1.6, color=RX_C, alpha=0.8, mew=0)
        ax.axhline(lim, color=LIM, lw=0.9, ls="--")
        for u in f["under"]:
            if x0 <= u <= x1:
                ax.plot(u, 1.0, marker="v", ms=5, color=MUTED, transform=ax.get_xaxis_transform(), clip_on=False)
        ax.text(x1, lim, f"{lim:.0f} {unit} ", color=LIM, va="bottom", ha="right", fontsize=8)
        if row == 0:
            ax.axhline(515, color=MUTED, lw=0.7, ls=":")
            if k == 0:
                ax.text(x1, 515, "515 m/s (the other parts) ", color=MUTED, va="top", ha="right", fontsize=7.5)
            ax.set_title(f["label"], loc="left", fontsize=9.5)
            ax.set_ylim(-40, 1600)
        else:
            ax.set_ylim(-3, 110)
            ax.set_xlabel("time from ignition, s")
        ax.set_xlim(x0, x1)
        ax.grid(axis="y", color=GRID, lw=0.5)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
axs[0][0].set_ylabel("speed, m/s")
axs[1][0].set_ylabel("altitude, km")
h = [plt.Line2D([], [], color=INK, lw=1.2, label="truth"),
     plt.Line2D([], [], marker=".", ls="", color=RX_C, label="mosaic-G5's own fix (speed, height)"),
     plt.matplotlib.patches.Patch(color=BLOCK, alpha=0.18, label="no fix: \"position output prohibited due to export laws\""),
     plt.Line2D([], [], marker="v", ls="", color=MUTED, label="transmitter underrun (the rig)")]
fig.legend(handles=h, loc="lower center", ncol=4, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, -0.04))
fig.suptitle("mosaic-G5: its fix against the export limit on four flights", x=0.06, ha="left", fontsize=10.5)
fig.text(0.06, -0.085, "The receiver blocks when its own speed passes 600 m/s and releases a few m/s lower; altitude plays "
         "no part, and it gives a fix through apogee above 80 km. Its raw measurements stop over the same stretches. "
         "Other gaps follow a transmitter underrun:\nafter one, it rejects channels left on the old clock until it "
         "has a consistent set again (fix error 6), for up to 70 s.", fontsize=8.5, color=MUTED, ha="left")
fig.subplots_adjust(top=0.9, bottom=0.12)
fig.savefig(out_png, dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out_png, out_json)
for e in edges:
    print(" ", e)
