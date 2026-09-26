#!/usr/bin/env python3
"""Run-to-run view of the LC86G through the spaceshot boost: one strip per run.

    ./plot_boost_runs.py OUT.png [--title TEXT] LABEL=CAPTURE [LABEL=CAPTURE ...]

Each strip is a satellite-by-time heatmap of MSM7 C/N0 (satellites ordered by
the Doppler rate the 13.5 g burn gives them, gentlest at the top; blank = not
measured), over a one-line fix strip: right, valid-flagged but WRONG (> 5 km or
> 50 m/s off the injection, the reports' test), no fix, or silent (no output
at all -- the receiver's own 500 m/s or 80 km mute). WRONG counts from 181 s, as
in `lc86_tracking.py boost` and the reports: the first second of burn is latency.

A second figure, OUT_held.png, is the burn itself: satellite by run, held through
13.5 g or lost, straight from `lc86_tracking.boost_run` (the table's numbers).
"""
from __future__ import annotations

import bisect
import gzip
import json
import statistics as st
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402
import numpy as np                                                # noqa: E402
from matplotlib.colors import ListedColormap                     # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import rtcm3                                                      # noqa: E402
from lc86_tracking import BURN, boost_run                         # noqa: E402

TOW0 = 203400.0
T_LO, T_HI = 170, 325
INK, INK3, RULE = "#1d2129", "#6b7280", "#e3e6ea"
HELD, SLIP, LOST = "#08519c", "#bcd4ec", "#dfe3e8"   # held; held but lock reset; lost
FIX_COL = {0: "#ffffff", 1: "#dfe3e8", 2: "#0E7C66", 3: "#6D3FA8"}   # silent, no fix, fix, WRONG
EVENTS = [(180.0, "ignition"), (192.1, "burnout"), (260.5, "< 500 m/s"),
          (291.7, "80 km"), (314.4, "apogee")]


def truth_fn():
    tr = json.loads((HERE / "scenarios" / "spaceshot.json").read_text())["truth"]
    tt = [s["t"] for s in tr]

    def at(x, k):
        i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
        a, b = tr[i - 1], tr[i]
        f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
        return a[k] + f * (b[k] - a[k])
    return at


def lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        for raw in fh:
            _ts, _, rest = raw.rstrip("\n").partition(" ")
            yield rest


def load(path, at, order):
    n_t = T_HI - T_LO
    cn = np.full((len(order), n_t), np.nan)
    state = np.zeros(n_t, dtype=int)          # 0 silent until an epoch is seen
    pad = []
    for rest in lines(path):
        if rest.startswith("R "):
            try:
                r = rtcm3.parse_msm7(bytes.fromhex(rest[2:]))
            except ValueError:
                continue
            if not r or r[0] != "GPS":
                continue
            ft = r[1] / 1000.0 - TOW0
            for c in r[2]:
                if not c.get("cn0"):
                    continue
                if 120 <= ft < 179:
                    pad.append(c["cn0"])
                j = int(round(ft)) - T_LO
                if 0 <= j < n_t and c["prn"] in order:
                    cn[order.index(c["prn"]), j] = c["cn0"]
        elif rest.startswith("$PQTMPVT"):
            f = rest.split("*")[0].split(",")
            try:
                ft, ok, fm = int(f[2]) / 1000.0 - TOW0, int(f[5] or 0), int(f[6] or 0)
            except (ValueError, IndexError):
                continue
            j = int(ft) - T_LO
            if not 0 <= j < n_t:
                continue
            s = 1
            if ok and fm >= 2 and f[11] and ft >= BURN[0] + 1:
                da = abs(float(f[11]) - at(ft, "alt_m"))
                dv = abs(-float(f[15]) - at(ft, "v_up_mps")) if f[15] else 0.0
                s = 3 if (da > 5000 or dv > 50) else 2
            elif ok and fm >= 2:
                s = 2
            state[j] = max(state[j], s)
    return cn, state, (st.median(pad) if pad else float("nan"))


def main() -> int:
    out, rest = sys.argv[1], sys.argv[2:]
    title = "LC86G, Balloon mode, through the 13.5 g boost to apogee: one strip per run"
    if rest[:1] == ["--title"]:
        title, rest = rest[1], rest[2:]
    specs = [a.split("=", 1) for a in rest]
    ref = json.loads((HERE / "results" / "doppler_ref_spaceshot.json").read_text())["sats"]
    rate = {int(k.split(":")[1]): v.get("rate_hzs") for k, v in ref.items() if k.startswith("0:")}
    order = sorted(rate, key=lambda p: (rate[p] is None, rate[p] or 0))
    at = truth_fn()
    runs = [(label, *load(Path(p), at, order)) for label, p in specs]

    n = len(runs)
    fig_h = 1.2 + n * 1.3
    fig = plt.figure(figsize=(13, fig_h), dpi=130)
    gs = fig.add_gridspec(n * 2, 1, height_ratios=[1, 0.14] * n, hspace=0.0,
                          left=0.19, right=0.955, top=1 - 0.95 / fig_h, bottom=0.45 / fig_h)
    cmap = plt.get_cmap("Blues")
    ext = [T_LO, T_HI, len(order), 0]
    for i, (label, cn, state, padcn) in enumerate(runs):
        ax = fig.add_subplot(gs[2 * i])
        axf = fig.add_subplot(gs[2 * i + 1], sharex=ax)
        ax.imshow(cn, aspect="auto", extent=ext, cmap=cmap, vmin=15, vmax=48,
                  interpolation="nearest")
        ax.set_facecolor("#fbfbfc")
        axf.imshow(state[None, :], aspect="auto", extent=[T_LO, T_HI, 1, 0],
                   cmap=ListedColormap([FIX_COL[k] for k in range(4)]), vmin=-0.5, vmax=3.5,
                   interpolation="nearest")
        for x, _name in EVENTS:
            for a in (ax, axf):
                a.axvline(x, color=INK3, lw=0.6, ls=(0, (2, 2)))
        held = sum(1 for k, p in enumerate(order) if np.all(~np.isnan(cn[k, 183 - T_LO:192 - T_LO])))
        wrong = int((state == 3).sum())
        ax.text(-0.012, 0.55, label, transform=ax.transAxes, ha="right", va="center",
                fontsize=8.5, color=INK)
        ax.text(-0.012, 0.12, f"pad {padcn:.1f} dBHz · held {held} · wrong {wrong} s",
                transform=ax.transAxes, ha="right", va="center", fontsize=7, color=INK3)
        if i == 0:   # name the satellite rows once
            ax.set_yticks([k + 0.5 for k in range(len(order))])
            ax.set_yticklabels([f"G{p:02d} {('' if rate[p] is None else f'{rate[p]:.0f}')}"
                                for p in order], fontsize=5.6, color=INK3)
            ax.tick_params(axis="y", length=0, pad=1)
            ax.yaxis.tick_right()
        else:
            ax.set_yticks([])
        axf.set_yticks([])
        for a in (ax, axf):
            for sp in a.spines.values():
                sp.set_color(RULE)
        plt.setp(ax.get_xticklabels(), visible=False)
        ax.tick_params(axis="x", length=0)
        if i < n - 1:
            plt.setp(axf.get_xticklabels(), visible=False)
            axf.tick_params(axis="x", length=0)
        else:
            axf.tick_params(axis="x", labelsize=7.5, colors=INK3)
            axf.set_xlabel("file time (s)", fontsize=8, color=INK3)
        if i == 0:
            for x, name in EVENTS:
                ax.text(x, -0.6, name, ha="center", va="bottom", fontsize=7, color=INK3,
                        transform=ax.get_xaxis_transform() if False else ax.transData)
    fig.text(0.19, 1 - 0.22 / fig_h, title, fontsize=11, fontweight="bold", color=INK, va="top")
    fig.text(0.19, 1 - 0.45 / fig_h,
             "heatmap: MSM7 C/N0 per satellite, rows ordered by the Doppler rate the burn gives "
             "them (gentlest at the top); blank = not measured",
             fontsize=7.5, color=INK3, va="top")
    # fix-strip legend, its own line
    y = 1 - 0.62 / fig_h
    fig.text(0.19, y, "fix strip:", fontsize=7.5, color=INK3, va="top")
    x0 = 0.235
    for k, name in ((2, "right"), (3, "valid-flagged but WRONG"), (1, "no fix"),
                    (0, "silent (the 500 m/s or 80 km mute)")):
        fig.patches.append(plt.Rectangle((x0, y - 0.0085), 0.012, 0.007,
                                         transform=fig.transFigure, color=FIX_COL[k],
                                         ec=INK3, lw=0.4))
        fig.text(x0 + 0.016, y, name, fontsize=7.5, color=INK3, va="top")
        x0 += 0.016 + 0.0052 * len(name) + 0.02
    # C/N0 scale
    yc = 1 - 0.30 / fig_h
    cax = fig.add_axes([0.86, yc - 0.011, 0.11, 0.008])
    cb = fig.colorbar(plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(15, 48)),
                      cax=cax, orientation="horizontal")
    cb.set_ticks([20, 30, 40])
    cb.ax.tick_params(labelsize=6.5, colors=INK3, length=2)
    cb.outline.set_edgecolor(RULE)
    fig.text(0.855, yc, "C/N0 dBHz", fontsize=7, color=INK3, va="top", ha="right")
    fig.savefig(out)
    print("wrote", out)
    held_out = str(Path(out).with_name(Path(out).stem + "_held" + Path(out).suffix))
    hold_matrix(held_out, specs, order, rate)
    return 0


def hold_matrix(out, specs, order, rate):
    runs = [(label, boost_run(Path(p))[0]) for label, p in specs]
    nr, nc = len(order), len(runs)
    cw, rh, left, top, foot = 0.74, 0.21, 1.3, 1.35, 0.75      # inches
    W, H = left + nc * cw + 0.25, top + nr * rh + foot
    fig = plt.figure(figsize=(W, H), dpi=150)
    ax = fig.add_axes([left / W, foot / H, nc * cw / W, nr * rh / H])
    ax.set_xlim(0, nc)
    ax.set_ylim(nr, 0)
    ax.axis("off")
    for j, (label, sats) in enumerate(runs):
        ax.text(j + 0.5, -0.4, label.replace(" · ", "\n"), ha="center", va="bottom",
                fontsize=6.8, color=INK, linespacing=1.25, clip_on=False)
        for i, p in enumerate(order):
            x = sats.get(p)
            if x is None:
                ax.text(j + 0.5, i + 0.5, "·", ha="center", va="center", fontsize=8, color=INK3)
                continue
            slip = x["held"] and x["resets"]
            ax.add_patch(plt.Rectangle((j + 0.05, i + 0.07), 0.9, 0.86, ec="none",
                                       fc=(SLIP if slip else HELD) if x["held"] else LOST))
            if x["held"]:
                d = round(x["dmin"])
                ax.text(j + 0.5, i + 0.52, "0 dB" if d == 0 else f"{d:+d} dB".replace("-", "\u2212"),
                        ha="center", va="center", fontsize=6.3, color=INK if slip else "#ffffff")
        pad = st.median(x["pad"] for x in sats.values())
        ax.text(j + 0.5, nr + 0.55, f"{sum(1 for x in sats.values() if x['held'])}",
                ha="center", va="top", fontsize=8, color=INK, fontweight="bold", clip_on=False)
        clean = sum(1 for x in sats.values() if x["held"] and not x["resets"])
        ax.text(j + 0.5, nr + 1.55, f"{clean}", ha="center", va="top", fontsize=7.5,
                color=INK, clip_on=False)
        ax.text(j + 0.5, nr + 2.55, f"{pad:.1f}", ha="center", va="top", fontsize=7,
                color=INK3, clip_on=False)
        if j and label.split(" · ")[0] != runs[j - 1][0].split(" · ")[0]:
            ax.plot([j, j], [-3.3, nr + 3.4], color=INK3, lw=0.6, clip_on=False)
    for i, p in enumerate(order):
        ax.text(-1.62, i + 0.5, f"G{p:02d}", ha="left", va="center", fontsize=7, color=INK)
        ax.text(-0.12, i + 0.5, "\u2013" if rate[p] is None else f"{rate[p]:.0f} Hz/s",
                ha="right", va="center", fontsize=6.5, color=INK3)
    ax.text(-1.62, nr + 0.55, "held", ha="left", va="top", fontsize=7.5, color=INK,
            fontweight="bold", clip_on=False)
    ax.text(-1.62, nr + 1.55, "no lock reset", ha="left", va="top", fontsize=7, color=INK,
            clip_on=False)
    ax.text(-1.62, nr + 2.55, "pad dBHz", ha="left", va="top", fontsize=7, color=INK3,
            clip_on=False)
    fig.text(0.02, 1 - 0.14 / H, "Held through the 13.5 g burn, by satellite and run",
             fontsize=10.5, fontweight="bold", color=INK, va="top")
    fig.text(0.02, 1 - 0.36 / H,
             "rows by the burn's Doppler rate; held = measured in every MSM7 epoch 183-192.1 s, "
             "the number its lowest C/N0 in the burn against the pad; "
             "\u00b7 = not tracked on the pad", fontsize=6.8, color=INK3, va="top")
    y = 1 - 0.56 / H
    x0 = 0.02
    for fc, name in ((HELD, "held"), (SLIP, "held, but its lock counter reset: lost lock and "
                      "got it back between epochs"), (LOST, "lost")):
        fig.patches.append(plt.Rectangle((x0, y - 0.085 / H), 0.1 / W, 0.07 / H,
                                         transform=fig.transFigure, color=fc, lw=0))
        fig.text(x0 + 0.14 / W, y, name, fontsize=6.8, color=INK3, va="top")
        x0 += (0.14 + 0.046 * len(name) + 0.2) / W
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    raise SystemExit(main())
