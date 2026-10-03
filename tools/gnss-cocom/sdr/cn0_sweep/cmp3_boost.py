#!/usr/bin/env python3
"""The receivers through both boosts on one chart: of the satellites each tracked at ignition, the share still
tracked AND within 10 m of the truth at its last raw output before 515 m/s -- the PX1105R at its first channel status
after 515 m/s (lock, and the error of what it then delivers), the NEO-M8T and ZED-F9P at their raw cut-off (no
sustained loss, last valid error within 10 m). With the mosaic-G5's JSONs as a fourth pair it is drawn too, judged at
the PX1105R's moment just after 515 m/s, since it keeps reporting to 600 m/s. Rows: hotshot, traveler; columns: GPS,
Galileo, BeiDou; x: signal level. From the boost charts' JSONs (boost_traces_wide.py for the PX1105R,
m8t_boost_traces.py for the others).
    cmp3_boost.py OUT.png PX_HOT M8T_HOT F9P_HOT [MOS_HOT] PX_TRAV M8T_TRAV F9P_TRAV [MOS_TRAV]"""
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                         # noqa: E402

WRONG_M = 10.0
SDR = Path(__file__).resolve().parents[1]
out = sys.argv[1]
n_rx = (len(sys.argv) - 2) // 2                              # 3, or 4 with the mosaic-G5
files = {"hotshot": sys.argv[2:2 + n_rx], "traveler_soft25": sys.argv[2 + n_rx:2 + 2 * n_rx]}
RX = ("PX1105R", "NEO-M8T", "ZED-F9P", "mosaic-G5")[:n_rx]
STYLE = {"PX1105R": ("#6639ba", "o", (0, -13)), "NEO-M8T": ("#bf8700", "s", (0, 7)),
         "ZED-F9P": ("#0f7b6c", "D", (13, -3)), "mosaic-G5": ("#a40e26", "v", (13, 5))}
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
LEVELS = ["+12 dB", "+6 dB", "0 dB", "-6 dB"]
INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"


def over_s(scen):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    return sc["velocity_windows"][0][0] - sc["prologue_s"]


def px_counts(runs, scen):
    """{level: {sys: (kept, total)}}: lock at the first 1 Hz status past 515 m/s and the delivered error within 10 m."""
    o = over_s(scen)
    tcmp = math.floor(o) + (1.0 if o % 1.0 < 0.85 else 2.0)
    res = {}
    for r in runs:
        lv = r["label"].split(" (")[0]
        for c in "GEC":
            tot = [q for q in r["recs"] if q["sys"] == c]
            kept = 0
            for q in tot:
                pts = [s for s in q["series"] if s[0] <= tcmp]
                if pts and pts[-1][4] and len(pts[-1]) > 5 and pts[-1][5] is not None and abs(pts[-1][5]) <= WRONG_M:
                    kept += 1
            if tot:
                res.setdefault(lv, {})[c] = (kept, len(tot))
    return res


def tcmp_of(scen):
    o = over_s(scen)
    return math.floor(o) + (1.0 if o % 1.0 < 0.85 else 2.0)


def mosaic_counts(runs, scen):
    """Still reporting at 515 m/s: no sustained loss before the PX1105R's moment, last valid error there within 10 m."""
    tc = tcmp_of(scen)
    res = {}
    for r in runs:
        for c in "GEC":
            tot = [q for q in r["recs"] if q["sys"] == c]
            kept = 0
            for q in tot:
                ev = [x for t, ok, v, x in q["series"] if ok and t <= tc]
                if (not q["lost"] or q["t"] > tc) and (not ev or ev[-1] is None or abs(ev[-1]) <= WRONG_M):
                    kept += 1
            if tot:
                res.setdefault(r["label"], {})[c] = (kept, len(tot))
    return res


def ublox_counts(runs):
    res = {}
    for r in runs:
        for c in "GEC":
            tot = [q for q in r["recs"] if q["sys"] == c]
            if tot:
                res.setdefault(r["label"], {})[c] = (sum(not q["lost"] and not q.get("wrong_end") for q in tot), len(tot))
    return res


plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": GRID})
fig, axs = plt.subplots(2, 3, sharex=True, sharey=True, figsize=(12.2, 7.4))
table = []
for row, (scen, fs) in enumerate(files.items()):
    counts = {"PX1105R": px_counts(json.load(open(fs[0])), scen), "NEO-M8T": ublox_counts(json.load(open(fs[1]))),
              "ZED-F9P": ublox_counts(json.load(open(fs[2])))}
    if n_rx > 3:
        counts["mosaic-G5"] = mosaic_counts(json.load(open(fs[3])), scen)
    for col, c in enumerate("GEC"):
        ax = axs[row][col]
        pts = {}                                    # level index -> [(y, rx, label)]
        for rx in RX:
            colr, mk, _ = STYLE[rx]
            xs, ys = [], []
            for i, lv in enumerate(LEVELS):
                k = counts[rx].get(lv, {}).get(c)
                if not k:
                    continue
                xs.append(i)
                ys.append(100.0 * k[0] / k[1])
                pts.setdefault(i, []).append((ys[-1], RX.index(rx), rx, f"{k[0]}/{k[1]}"))
                table.append((scen.split("_")[0], NAME[c], rx, lv, k[0], k[1]))
            ax.plot(xs, ys, color=colr, marker=mk, ms=5.5, lw=1.5, label=rx)
        for i, group in pts.items():                # labels by rank at each level: top above, bottom below, middle right
            group.sort(reverse=True)
            for rank, (y, _, rx, lab) in enumerate(group):
                if rank == 0:
                    off, ha = (0, 7), "center"
                elif rank == len(group) - 1:
                    off, ha = (0, -13), "center"
                else:
                    off, ha = (8, -3), "left"
                ax.annotate(lab, (i, y), xytext=off, textcoords="offset points", ha=ha, fontsize=7.5,
                            color=STYLE[rx][0])
        ax.set_ylim(-14, 116)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_xlim(-0.4, 3.6)
        ax.set_xticks(range(4), LEVELS)
        ax.grid(axis="y", color=GRID, lw=0.5)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        if row == 0:
            ax.set_title(NAME[c], loc="left", fontsize=10)
    o = over_s(scen)
    axs[row][0].set_ylabel(f"{scen.split('_')[0].capitalize()} (515 m/s at T+{o:.1f})\n"
                           "tracked and within 10 m, %")
for ax in axs[1]:
    ax.set_xlabel("signal level")
axs[0][2].legend(frameon=False, fontsize=8.5, loc="upper left", bbox_to_anchor=(1.02, 1.0))
fig.suptitle("Of the satellites tracked at ignition, the share still tracked and within 10 m of the truth at the last "
             "raw output before 515 m/s", x=0.06, ha="left", fontsize=10.5)
mos = ("\nmosaic-G5: the same at the PX1105R's moment just after 515 m/s (it reports on to 600 m/s)."
       if n_rx > 3 else "")
fig.text(0.06, 0.005, "PX1105R: locked in its first 1 Hz channel status after 515 m/s, and what it then delivers within "
         "10 m. NEO-M8T and ZED-F9P: no gap >= 0.5 s in valid raw pseudoranges up to their raw cut-off at 515 m/s, and "
         f"the last one within 10 m.{mos}\nThe {'other parts' if n_rx > 3 else 'u-blox parts'} sit behind 10 dB more "
         "attenuation than the PX1105R on the same files; the levels are the files' (SignalSim C/N0 setting, added "
         "noise for -6 dB).", fontsize=8, color=MUTED, ha="left")
fig.subplots_adjust(top=0.9, bottom=0.12, wspace=0.08, hspace=0.18, right=0.86)
fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out)
for r in table:
    print("  %-9s %-8s %-8s %-7s %d/%d" % r)
