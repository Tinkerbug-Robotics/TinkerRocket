#!/usr/bin/env python3
"""A u-blox receiver's pseudorange error against line-of-sight Doppler rate: every satellite at every 10 Hz epoch from
ignition to its raw cut-off while flagged valid, all four levels, hotshot and traveler side by side on one rate axis
(from m8t_boost_traces.py's JSONs, which carry m8t_accuracy.py's errors). Shows where the receiver's measurements go
wrong and whether the traveler gets there before 515 m/s.
    m8t_err_rate.py OUT.png [HOT_JSON TRAV_JSON [RX [LIMIT]]]   (default m8t_boost.json m8t_trav_boost.json NEO-M8T 515)
    (mosaic-G5 600: it reports to 600 m/s)"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                         # noqa: E402

S = Path(__file__).resolve().parent / "data"
out = sys.argv[1]
hot_json, trav_json = (sys.argv[2], sys.argv[3]) if len(sys.argv) > 3 else (S / "m8t_boost.json",
                                                                          S / "m8t_trav_boost.json")
RX = sys.argv[4] if len(sys.argv) > 4 else "NEO-M8T"
RX_SHORT = "mosaic" if RX.startswith("mosaic") else RX.split("-")[-1]
LIMIT = float(sys.argv[5]) if len(sys.argv) > 5 else 515.0
WRONG_M, WRONG, FLOOR = 10.0, "#bf3989", 0.03
INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
COL = {"G": "#1f6feb", "E": "#c4561a", "C": "#1a7f37"}
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
MK = {"+12 dB": "o", "+6 dB": "s", "0 dB": "^", "-6 dB": "D"}
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": GRID})


def cut_text(runs):
    cuts = sorted({round(r["cut"], 1) for r in runs})
    return f"T+{cuts[0]:.1f}" if len(cuts) == 1 else f"T+{cuts[0]:.1f} to T+{cuts[-1]:.1f}"


fig, axs = plt.subplots(1, 2, sharey=True, sharex=True, figsize=(12.5, 4.8))
vmax = {}
for ax, (fn, name) in zip(axs, ((hot_json, "Hotshot"), (trav_json, "Traveler"))):
    runs = json.load(open(fn))
    n_all = n_bad = 0
    vmax[name] = max((v for r in runs for q in r["recs"] for t, ok, v, x in q["series"]), default=0.0)
    for r in runs:
        for q in r["recs"]:
            pts = [(v, abs(x)) for t, ok, v, x in q["series"] if ok and x is not None and t >= 0]
            n_all += len(pts)
            n_bad += sum(e > WRONG_M for _, e in pts)
            if pts:
                ax.scatter([p[0] for p in pts], [max(p[1], FLOOR) for p in pts], s=9, marker=MK[r["label"]],
                           color=COL[q["sys"]], alpha=0.45, lw=0, rasterized=True)
    ax.axhline(WRONG_M, color=WRONG, ls="--", lw=1.0)
    ax.annotate(f"{WRONG_M:.0f} m", (1.0, WRONG_M), xycoords=("axes fraction", "data"), xytext=(-2, 3),
                textcoords="offset points", ha="right", fontsize=8, color=WRONG)
    ax.set_yscale("log")
    ax.set_ylim(FLOOR * 0.8, 1500)
    ax.set_xlim(0, 1500)
    ax.set_title(f"{name}, ignition to the raw cut-off ({cut_text(runs)})\n{n_bad} of {n_all} valid-flagged "
                 f"measurements over {WRONG_M:.0f} m off", loc="left", fontsize=9.5)
    ax.grid(color=GRID, lw=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.set_xlabel("line-of-sight Doppler rate at that moment, Hz/s")
axs[0].set_ylabel("|pseudorange error|, m")
h = [plt.Line2D([], [], marker="o", ls="", color=COL[c], label=NAME[c]) for c in "GEC"]
h += [plt.Line2D([], [], marker=m, ls="", color=MUTED, markerfacecolor="none", label=lv) for lv, m in MK.items()]
fig.legend(handles=h, loc="lower center", ncol=7, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, -0.06))
fig.suptitle(f"{RX}: pseudorange error against line-of-sight Doppler rate, every satellite and 10 Hz epoch while "
             "flagged valid, all four levels", x=0.06, ha="left", fontsize=10.5)
fig.text(0.06, -0.12, f"Errors under {FLOOR * 100:.0f} cm are drawn at {FLOOR * 100:.0f} cm. The traveler's rates "
         f"stay under about {round(vmax['Traveler'], -1):.0f} Hz/s until {LIMIT:.0f} m/s, when the {RX_SHORT} stops reporting; the hotshot "
         f"reaches {round(vmax['Hotshot'], -1):.0f} Hz/s first.", fontsize=8.5, color=MUTED, ha="left")
fig.subplots_adjust(top=0.82, bottom=0.18, wspace=0.08)
fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out)
