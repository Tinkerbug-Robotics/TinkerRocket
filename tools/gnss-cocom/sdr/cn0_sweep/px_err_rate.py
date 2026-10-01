#!/usr/bin/env python3
"""PX1105R pseudorange error against line-of-sight Doppler rate: every delivered measurement (10 Hz rows of
pr_accuracy.py, Doppler-bridged clock rows left out) from ignition to burnout, all levels, hotshot and traveler. Two
rows: up to the first 515 m/s crossing (its fix running; the part the NEO-M8T also reports) and from there to burnout
(fix stopped, measurements released). From boost_traces_wide.py's JSONs, whose series carry the error.
    px_err_rate.py OUT.png"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                         # noqa: E402

S = Path(__file__).resolve().parent / "data"
SDR = Path(__file__).resolve().parents[1]
WRONG_M, WRONG, FLOOR = 10.0, "#bf3989", 0.03
INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
COL = {"G": "#1f6feb", "E": "#c4561a", "C": "#1a7f37"}
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
MK = {"+12 dB": "o", "+6 dB": "s", "0 dB": "^", "-3 dB": "v", "-6 dB": "D", "-9 dB": "P"}
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": GRID})


def events(scen):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    pro = sc["prologue_s"]
    burn = next(s["t"] for s in sc["truth"] if s["phase"] == "coast") - pro
    return sc["velocity_windows"][0][0] - pro, burn


fig, axs = plt.subplots(2, 2, sharey=True, sharex=True, figsize=(12.5, 8.6))
for col, (fn, scen, name) in enumerate((("hot_boost.json", "hotshot", "Hotshot"),
                                        ("boost_final.json", "traveler_soft25", "Traveler"))):
    over, burn = events(scen)
    for row, (lo, hi, what) in enumerate(((0.0, over, f"ignition to 515 m/s (T+{over:.1f}), fix running"),
                                          (over, burn, f"515 m/s to burnout (T+{over:.1f} to {burn:.1f}), "
                                                       "fix stopped"))):
        ax = axs[row][col]
        n_all = n_bad = 0
        for r in json.load(open(S / fn)):
            mk = MK.get(r["label"].split(" (")[0], "o")
            for q in r["recs"]:
                pts = [(p[2], abs(p[5])) for p in q["series"]
                       if len(p) > 5 and p[1] and p[5] is not None and lo <= p[0] < hi
                       and abs(p[0] * 10 - round(p[0] * 10)) < 0.01]          # the 10 Hz rows only, once each
                n_all += len(pts)
                n_bad += sum(e > WRONG_M for _, e in pts)
                if pts:
                    ax.scatter([p[0] for p in pts], [max(p[1], FLOOR) for p in pts], s=9, marker=mk,
                               color=COL[q["sys"]], alpha=0.45, lw=0, rasterized=True)
        ax.axhline(WRONG_M, color=WRONG, ls="--", lw=1.0)
        ax.annotate(f"{WRONG_M:.0f} m", (1.0, WRONG_M), xycoords=("axes fraction", "data"), xytext=(-2, 3),
                    textcoords="offset points", ha="right", fontsize=8, color=WRONG)
        ax.set_yscale("log")
        ax.set_ylim(FLOOR * 0.8, 1500)
        ax.set_xlim(0, 1700)
        ax.set_title(f"{name}, {what}\n{n_bad} of {n_all} delivered measurements over {WRONG_M:.0f} m off",
                     loc="left", fontsize=9.5)
        ax.grid(color=GRID, lw=0.5)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        if row == 1:
            ax.set_xlabel("line-of-sight Doppler rate at that moment, Hz/s")
        if col == 0:
            ax.set_ylabel("|pseudorange error|, m")
h = [plt.Line2D([], [], marker="o", ls="", color=COL[c], label=NAME[c]) for c in "GEC"]
h += [plt.Line2D([], [], marker=m, ls="", color=MUTED, markerfacecolor="none", label=lv) for lv, m in MK.items()]
fig.legend(handles=h, loc="lower center", ncol=9, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, -0.02))
fig.suptitle("PX1105R: pseudorange error against line-of-sight Doppler rate, every delivered measurement through "
             "the burn, all levels", x=0.06, ha="left", fontsize=10.5)
fig.text(0.06, -0.05, f"Errors under {FLOOR * 100:.0f} cm are drawn at {FLOOR * 100:.0f} cm; rows whose receiver "
         "clock was bridged by the Doppler are left out. While its fix runs the PX1105R withholds many locked "
         "satellites, so the top row holds fewer points.", fontsize=8.5, color=MUTED, ha="left")
fig.subplots_adjust(top=0.9, bottom=0.1, wspace=0.08, hspace=0.32)
fig.savefig(sys.argv[1], dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", sys.argv[1])
