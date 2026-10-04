#!/usr/bin/env python3
"""A SkyTraq receiver's own speed against the truth through a boost, over how many GPS L1
channels it lists in each 0xE5 epoch: the view behind the fix-coupling finding in
cn0_boost_report.html. When its own speed trails the truth, the satellites whose Doppler
disagrees most with it drop out of the raw output; at strong signal they all return in the
epoch after its fix stops.

    ./plot_own_lag.py OUT.svg SCENARIO T_END "LABEL=CAPTURE" [...]

Ignition is file time 600 s (the pad600 files); the truth is scenarios/SCENARIO.json.
"""
import bisect
import json
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402

SDR = Path(__file__).resolve().parent
TOW0, IGN = 203400.0, 600.0
out, scen, t_end = sys.argv[1], sys.argv[2], float(sys.argv[3])
runs = [a.split("=", 1) for a in sys.argv[4:]]
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
tr = sc["truth"]
tt = [s["t"] for s in tr]
PRO = sc.get("prologue_s", 180.0)


def truth_speed(t_ign):
    x = t_ign + PRO
    i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
    a, b = tr[i - 1], tr[i]
    f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    return a["speed_mps"] + f * (b["speed_mps"] - a["speed_mps"])


def load(path):
    """(t, GPS L1 channels listed) per 0xE5 epoch and (t, nav state, speed) per 0xDF, s from ignition."""
    e5, df = [], []
    for line in open(SDR / "captures" / path if not Path(path).exists() else path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "B":
            continue
        try:
            x = bytes.fromhex(p[2].strip())
        except ValueError:
            continue
        if x[0] == 0xE5 and len(x) >= 14:
            t = struct.unpack(">I", x[5:9])[0] / 1000.0 - TOW0 - IGN
            if -0.5 <= t <= t_end:
                e5.append((t, sum(1 for j in range(x[13])
                                  if len(x) >= 14 + 31 * (j + 1) and x[14 + 31 * j] == 0)))
        elif x[0] == 0xDF and len(x) >= 61:
            t = struct.unpack(">d", x[5:13])[0] - TOW0 - IGN
            if -0.5 <= t <= t_end:
                vx, vy, vz = struct.unpack(">fff", x[37:49])
                df.append((t, x[2], (vx * vx + vy * vy + vz * vz) ** 0.5))
    return sorted(e5), sorted(df)


INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
TRUTH, OWN, PRED, CHAN = "#59636e", "#c4561a", "#8250df", "#1f6feb"
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": GRID})
fig, axs = plt.subplots(2, len(runs), figsize=(3.6 * len(runs) + 0.6, 5.2), sharex=True, sharey="row",
                        squeeze=False, gridspec_kw=dict(height_ratios=[1.35, 1], hspace=0.12, wspace=0.08))
ts = [i * 0.01 for i in range(-50, int(t_end * 100) + 1)]
vmax = max(truth_speed(t) for t in ts)
for col, (label, path) in enumerate(runs):
    e5, df = load(path)
    # its last 2D/3D fix in the window: where the fix stopped (the traces figure's dashed line too)
    stop = max((t for t, st, _ in df if t > 0 and st >= 2), default=None)
    ax = axs[0][col]
    ax.plot(ts, [truth_speed(t) for t in ts], color=TRUTH, lw=1.1, label="true speed")
    fx = [(t, v) for t, st, v in df if st >= 2]
    ax.plot([t for t, _ in fx], [v for _, v in fx], ".", ms=3.2, color=OWN, label="its own speed (2D/3D fix)")
    pr = [(t, v) for t, st, v in df if st == 1]
    ax.plot([t for t, _ in pr], [v for _, v in pr], "x", ms=4, color=PRED, label="its own speed (prediction only)")
    ax.set_title(label, loc="left", fontsize=9.5)
    ax.set_ylim(0, vmax * 1.05)
    ax2 = axs[1][col]
    ax2.step([t for t, _ in e5], [n for _, n in e5], where="post", color=CHAN, lw=1.0)
    ax2.set_ylim(0, 14)
    ax2.set_xlabel("time from ignition, s")
    for a_ in (ax, ax2):
        a_.grid(color=GRID, lw=0.5)
        for side in ("top", "right"):
            a_.spines[side].set_visible(False)
        if stop is not None:
            a_.axvline(stop, color=MUTED, lw=0.9, ls="--", zorder=0)
    if stop is not None:
        ax.annotate(f"fix stops T+{stop:.2f}", (stop, 0.97), xycoords=("data", "axes fraction"),
                    xytext=(-3, 0), textcoords="offset points", ha="right", va="top", fontsize=8, color=MUTED)
axs[0][0].set_ylabel("speed, m/s")
axs[1][0].set_ylabel("GPS satellites it lists\n(0xE5, 20 per second)")
axs[0][0].legend(fontsize=7.5, loc="upper left", frameon=False)
axs[0][0].set_xlim(-0.3, t_end)
plt.rcParams["svg.fonttype"] = "none"
fig.savefig(out, bbox_inches="tight", facecolor="white")
print("wrote", out)
