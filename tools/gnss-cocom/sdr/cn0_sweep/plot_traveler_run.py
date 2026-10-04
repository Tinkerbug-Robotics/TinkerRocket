#!/usr/bin/env python3
"""One PX1105R traveler flight (ignition at 2026-08-18 08:40:00 GPST, TOW 204000): the flight to T+375 on the left,
the boost on the right. Rows: speed (truth, and the receiver's own speed whenever it has a 2D/3D fix, with the 515 m/s
limit), altitude (truth and the receiver's own altitude while fixed, 18 and 80 km), acceleration (from the injected
vertical velocity; Doppler rate is 5.25 Hz/s per m/s^2 along the line of sight at L1), when each satellite's raw
measurement (0xE5) came out (one bar per satellite, coloured by constellation), and measurements per epoch with the fix
periods shaded. With pr_accuracy.py's NPZ, three more: GPS pseudorange error, Galileo/BeiDou pseudorange error,
range-rate error -- on symmetric-log axes (linear near zero, logarithmic beyond) so errors of hundreds of metres stay
on the chart -- and the raster marks in magenta where a delivered measurement was more than WRONG_M (10 m) off the
truth (T-5 to T+30; rows with a Doppler-bridged clock not judged).

    plot_traveler_run.py CAPTURE OUT.png "TITLE" [SCENARIO] [ACC.npz] [end=T] [xmax=T]
      end=T   a shortened file's last second (hatched after it); xmax=T  the flight panels' right edge (default 375)
"""
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                         # noqa: E402

SDR = Path(__file__).resolve().parents[1]
TOW0, IGN = 203400.0, 600.0
cap, out, title = sys.argv[1], sys.argv[2], sys.argv[3]
extra = sys.argv[4:]
acc_path = next((a for a in extra if a.endswith(".npz")), None)
file_end = next((float(a[4:]) for a in extra if a.startswith("end=")), None)
x_max = next((float(a[5:]) for a in extra if a.startswith("xmax=")), 375.0)
scen = next((a for a in extra if not a.endswith(".npz") and "=" not in a), "traveler_soft25")
acc = np.load(acc_path) if acc_path else None
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
PRO = sc.get("prologue_s", 180.0)
tr = sc["truth"]
T_TR = np.array([s["t"] - PRO for s in tr])       # truth time from ignition
V_UP = np.array([s["v_up_mps"] for s in tr])
BURN = next(s["t"] for s in tr if s["phase"] == "coast") - PRO
G = {0: "GPS", 3: "GAL", 5: "BDS"}
# acceleration from the injected vertical velocity (10 Hz rows), at each block's middle, lightly smoothed
A_T = 0.5 * (T_TR[1:] + T_TR[:-1])
A_UP = np.diff(V_UP) / np.diff(T_TR)
A_UP = np.convolve(A_UP, np.ones(3) / 3, mode="same")


def ecef_alt(x, y, z):
    """WGS84 height of an ECEF point, m (0 for the receiver's all-zero no-fix position)."""
    a, e2 = 6378137.0, 6.69437999014e-3
    p = math.hypot(x, y)
    if p < 1.0:
        return 0.0
    lat, h = math.atan2(z, p * (1 - e2)), 0.0
    for _ in range(6):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    return h


# raw measurements (0xE5) and the receiver's own navigation (0xDF), times from ignition
meas, counts, nav = {}, [], []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B":
        continue
    try:
        x = bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if x[0] == 0xE5 and len(x) >= 14:
        t = struct.unpack(">I", x[5:9])[0] / 1000.0 - TOW0 - IGN
        n = {"GPS": 0, "GAL": 0, "BDS": 0}
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and (r[0] & 0x0F) in G and (r[0] >> 4) in (0, 1):
                sysn = G[r[0] & 0x0F]
                meas.setdefault((sysn, r[1]), []).append(t)
                n[sysn] += 1
        counts.append((t, n))
    elif x[0] == 0xDF and len(x) >= 61:
        t = struct.unpack(">d", x[5:13])[0] - TOW0 - IGN
        px, py, pz = struct.unpack(">ddd", x[13:37])
        vx, vy, vz = struct.unpack(">fff", x[37:49])
        nav.append((t, x[2], (vx * vx + vy * vy + vz * vz) ** 0.5, ecef_alt(px, py, pz) / 1000.0))
counts.sort(key=lambda c: c[0])                   # a repeated time tag (replay underruns) must not compare dicts
nav.sort()


def spans(ts, gap=0.3):
    """Contiguous runs of epochs (20 Hz raw) -> (start, length) bars; a gap over 0.3 s breaks a bar."""
    out, s0, prev = [], None, None
    for t in sorted(ts):
        if s0 is None:
            s0 = prev = t
        elif t - prev > gap:
            out.append((s0, max(prev - s0, 0.05)))
            s0 = t
        prev = t
    if s0 is not None:
        out.append((s0, max(prev - s0, 0.05)))
    return out


INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
COL = {"GPS": "#1f6feb", "GAL": "#c4561a", "BDS": "#1a7f37"}
LIM, ACC_C = "#9a6700", "#8250df"
WRONG, WRONG_M = "#bf3989", 10.0                # delivered but this far off the truth
WRONG_WIN = (-5.0, 30.0)                         # marked in the boost only: elsewhere transmitter underruns make
wrong_t = {}                                     # brief errors of the rig's own
if acc is not None:
    for i, name in ((0, "GPS"), (1, "GAL"), (2, "BDS")):
        m = ((acc["sys"] == i) & (np.abs(acc["pr"]) > WRONG_M) & ~acc["bridged"].astype(bool) &
             (acc["t"] >= WRONG_WIN[0]) & (acc["t"] <= WRONG_WIN[1]))
        for p in set(int(x) for x in acc["prn"][m]):
            wrong_t[(name, p)] = list(acc["t"][m & (acc["prn"] == p)])
PR_TICKS = [-1000, -100, -10, 0, 10, 100, 1000]
RR_TICKS = [-100, -10, -1, 0, 1, 10, 100]


def symlog(ax, lin, lim, ticks):
    ax.set_yscale("symlog", linthresh=lin, linscale=0.8)
    ax.set_ylim(-lim, lim)
    ax.set_yticks(ticks)
    ax.set_yticklabels([f"{v:g}" for v in ticks])
    ax.minorticks_off()


plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.edgecolor": GRID})
sats = sorted(meas, key=lambda k: ({"GPS": 0, "GAL": 1, "BDS": 2}[k[0]], k[1]))
R_SPD, R_ALT, R_ACC, R_RAS, R_CNT, R_PRG, R_PRE, R_RR = range(8)
HR = [1, 0.8, 0.7, 2.1, 0.9] + ([1.25, 1.0, 0.9] if acc is not None else [])
NR = len(HR)
fig, axs = plt.subplots(NR, 2, figsize=(13.5, 10.5 * sum(HR) / 4.8), sharex="col",
                        gridspec_kw={"width_ratios": [1.7, 1], "height_ratios": HR,
                                     "hspace": 0.12 * 4.8 / sum(HR) * 1.6, "wspace": 0.14})
fixes = [(t, v, h) for t, st, v, h in nav if st >= 2]
windows = [(a - PRO, b - PRO) for a, b in sc.get("blocked_windows", [])]
t_last = file_end if file_end is not None else max(t for t, n in counts if sum(n.values()) > 0)
for col, (x0, x1) in enumerate(((-60, x_max), (-5, 30))):
    for r in range(NR):
        ax = axs[r][col]
        for a, b in windows:
            ax.axvspan(a, b, color="#eda100", alpha=0.07, lw=0)
        if t_last < x1 - 2:
            ax.axvspan(t_last, x1, color="#8c959f", alpha=0.18, lw=0, hatch="//", fill=False)
            if r == R_SPD and x1 - t_last > 60:
                ax.text((t_last + x1) / 2, 0.5, f"file ends T+{t_last:.0f}\n(not flown)",
                        transform=ax.get_xaxis_transform(), ha="center", va="center", fontsize=8.5, color=MUTED)
        ax.axvline(0, color=MUTED, lw=0.7, ls=":")
        ax.axvline(BURN, color=MUTED, lw=0.7, ls=":")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.set_xlim(x0, x1)
    # speed
    ax = axs[R_SPD][col]
    ax.plot(T_TR, [s["speed_mps"] for s in tr], color=INK, lw=1.2, label="true speed")
    ax.plot([f[0] for f in fixes], [f[1] for f in fixes], ".", ms=2.2, color=COL["GPS"], alpha=0.8,
            label="receiver's own speed (while fixed)")
    ax.axhline(515, color=LIM, lw=0.9, ls="--")
    ax.text(x1, 515, " 515 m/s", color=LIM, va="bottom", ha="right", fontsize=8)
    ax.set_ylim(-50, 1650)
    ax.grid(axis="y", color=GRID, lw=0.6)
    if col == 0:
        ax.set_ylabel("speed, m/s")
        ax.legend(loc="upper right", frameon=False, fontsize=8)
    # altitude
    ax = axs[R_ALT][col]
    ax.plot(T_TR, [s["alt_m"] / 1000 for s in tr], color=INK, lw=1.2, label="true altitude")
    ax.plot([f[0] for f in fixes], [f[2] for f in fixes], ".", ms=2.2, color=COL["GPS"], alpha=0.8,
            label="receiver's own altitude (while fixed)")
    top = 110 if col == 0 else 12
    for h in (18, 80):
        if h < top:                                  # labels outside the panel would spill into the next one
            ax.axhline(h, color=LIM, lw=0.9, ls="--")
            ax.text(x1, h, f" {h} km", color=LIM, va="bottom", ha="right", fontsize=8)
    ax.set_ylim(-2, top)
    ax.grid(axis="y", color=GRID, lw=0.6)
    if col == 0:
        ax.set_ylabel("altitude, km")
        ax.legend(loc="lower center", bbox_to_anchor=(0.55, 0.02), frameon=False, fontsize=8)
    # acceleration
    ax = axs[R_ACC][col]
    ax.plot(A_T, A_UP, color=ACC_C, lw=1.1)
    ax.axhline(0, color=MUTED, lw=0.6)
    inw = (A_T >= x0) & (A_T <= x1)
    lo, hi = float(A_UP[inw].min()), float(A_UP[inw].max())
    pad = 0.08 * (hi - lo)
    ax.set_ylim(lo - pad, hi + pad)
    ax.grid(axis="y", color=GRID, lw=0.6)
    if col == 0:
        ax.set_ylabel("vertical accel.,\nm/s$^2$")
        ax.text(0.99, 0.92, "from the injected velocity; along a line of sight, 1 m/s$^2$ = 5.25 Hz/s of L1 "
                "Doppler rate", transform=ax.transAxes, fontsize=7.5, color=MUTED, va="top", ha="right")
    # measurements per satellite
    ax = axs[R_RAS][col]
    for i, k in enumerate(sats):
        bars = [(s, w) for s, w in spans(meas[k]) if s + w >= x0 and s <= x1]
        ax.broken_barh(bars, (i - 0.35, 0.7), facecolors=COL[k[0]], edgecolor="none")
        wb = [(s, w) for s, w in spans(wrong_t.get(k, []), gap=0.3) if s + w >= x0 and s <= x1]
        if wb:                                       # delivered but over WRONG_M off
            ax.broken_barh(wb, (i - 0.45, 0.9), facecolors=WRONG, edgecolor="none", zorder=3)
    ax.set_yticks(range(len(sats)))
    ax.set_yticklabels([f"{'G' if k[0] == 'GPS' else ('E' if k[0] == 'GAL' else 'C')}{k[1]:02d}" for k in sats],
                       fontsize=7)
    ax.set_ylim(len(sats) - 0.5, -0.5)
    ax.grid(axis="x", color=GRID, lw=0.5)
    if col == 0:
        ax.set_ylabel("raw measurement output, per satellite")
        if acc is not None:
            ax.text(1.0, 1.005, f"magenta: delivered but over {WRONG_M:.0f} m off the truth, T-5 to T+30"
                    + ("" if wrong_t else " (none in this run)"), transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=7.8, color=WRONG)
    # counts and fix state
    ax = axs[R_CNT][col]
    ct = [t for t, _ in counts]
    for sysn in ("GPS", "GAL", "BDS"):
        if any(n[sysn] for _, n in counts):
            ax.plot(ct, [n[sysn] for _, n in counts], color=COL[sysn], lw=0.8, drawstyle="steps-post",
                    label={"GPS": "GPS", "GAL": "Galileo", "BDS": "BeiDou"}[sysn])
    fx = [t for t, st, _, _ in nav if st >= 2]
    for s, w in spans(fx, gap=0.6):
        ax.axvspan(s, s + w, ymin=0.0, ymax=0.06, color="#1a7f37", lw=0)
    ax.set_ylim(-0.8, 15)
    ax.grid(axis="y", color=GRID, lw=0.6)
    if col == 0:
        ax.set_ylabel("measurements\nper epoch")
        ax.legend(loc="lower right", bbox_to_anchor=(1.0, 0.98), frameon=False, fontsize=8, ncol=3)
    if acc is not None:
        at, asys, apr, arr, abr = acc["t"], acc["sys"].astype(int), acc["pr"], acc["rr"], acc["bridged"]
        inw = (at >= x0) & (at <= x1)
        ms = 1.1 if col == 0 else 2.6
        # GPS pseudorange
        ax = axs[R_PRG][col]
        for bridged, alpha in ((False, 0.55), (True, 0.18)):
            m = inw & (asys == 0) & (abr == bridged)
            ax.plot(at[m], apr[m], ".", ms=ms, color=COL["GPS"], alpha=alpha, mew=0, rasterized=True)
        ax.axhline(0, color=MUTED, lw=0.6)
        split = float(acc["split"]) if "split" in acc.files else 4.2
        symlog(ax, 5.0, 1000.0, PR_TICKS)
        for yv in (WRONG_M, -WRONG_M):
            ax.axhline(yv, color=WRONG, lw=0.7, ls="--")
        ax.grid(axis="y", color=GRID, lw=0.6)
        if col == 0:
            ax.set_ylabel("GPS pseudorange\nerror, m")
            note = ("GPS follows the carrier: with the rig's 22 Hz carrier offset uncorrected it slides off the code "
                    "(up to 4.2 m/s)" if abs(split) > 1.0 else
                    f"code and carrier consistent (code minus carrier {split:+.2f} m/s)")
            ax.text(0.005, 0.04, f"linear within ±5 m, logarithmic beyond; dashed: ±{WRONG_M:.0f} m. " + note,
                    transform=ax.transAxes, fontsize=7.5, color=MUTED)
        # Galileo / BeiDou pseudorange
        ax = axs[R_PRE][col]
        for i, name in ((1, "GAL"), (2, "BDS")):
            for bridged, alpha in ((False, 0.55), (True, 0.18)):
                m = inw & (asys == i) & (abr == bridged)
                ax.plot(at[m], apr[m], ".", ms=ms, color=COL[name], alpha=alpha, mew=0, rasterized=True)
        ax.axhline(0, color=MUTED, lw=0.6)
        symlog(ax, 5.0, 1000.0, PR_TICKS)
        for yv in (WRONG_M, -WRONG_M):
            ax.axhline(yv, color=WRONG, lw=0.7, ls="--")
        ax.grid(axis="y", color=GRID, lw=0.6)
        if col == 0:
            present = [n for i, n in ((1, "Galileo"), (2, "BeiDou")) if (asys == i).any()]
            ax.set_ylabel(" / ".join(present) + "\npseudorange error, m")
        # range rate
        ax = axs[R_RR][col]
        for i, name in ((0, "GPS"), (1, "GAL"), (2, "BDS")):
            m = inw & (asys == i)
            ax.plot(at[m], arr[m], ".", ms=ms, color=COL[name], alpha=0.5, mew=0, rasterized=True)
        ax.axhline(0, color=MUTED, lw=0.6)
        symlog(ax, 0.5, 500.0, RR_TICKS)
        ax.grid(axis="y", color=GRID, lw=0.6)
        if col == 0:
            ax.set_ylabel("range-rate error\n(Doppler), m/s")
            ax.text(0.005, 0.04, "linear within ±0.5 m/s, logarithmic beyond", transform=ax.transAxes,
                    fontsize=7.5, color=MUTED)
    if col == 0:
        axs[NR - 1][col].set_xlabel(f"time from ignition, s   (dotted: ignition and burnout T+{BURN:.1f}; amber: over "
                                    f"515 m/s or 80 km; green strip: receiver has a fix)")
    else:
        axs[NR - 1][col].set_xlabel("boost: T-5 to T+30 s")
fig.subplots_adjust(top=1.0 - 0.45 / fig.get_figheight())         # keep the title close to the first row
fig.suptitle(title, x=0.06, y=1.0 - 0.12 / fig.get_figheight(), ha="left", fontsize=11)
fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out)
