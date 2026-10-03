#!/usr/bin/env python3
"""One NEO-M8T or ZED-F9P flight (ignition at 2026-08-18 08:40:00 GPST, TOW 204000), drawn like plot_traveler_run.py's PX1105R
flights: the flight on the left, the boost on the right. Rows: speed (truth, and the receiver's own speed whenever
NAV-PVT has a gnssFixOK 2D/3D fix, with the 515 m/s limit), altitude (truth and the receiver's own height while fixed,
18 and 80 km), acceleration (from the injected vertical velocity), when each satellite's valid raw pseudorange
(RXM-RAWX) came out, and measurements per epoch with the fix periods shaded. With m8t_accuracy.py's NPZ, three more:
GPS pseudorange error, Galileo/BeiDou pseudorange error, range-rate error -- on symmetric-log axes (linear near zero,
logarithmic beyond) so that errors of hundreds of metres stay on the chart -- and the raster marks in magenta where a
satellite was flagged valid but more than WRONG_M (10 m) off the truth.

A mosaic-G5 capture (SBF 't S <hex>' lines) is read the same way: MeasEpoch thinned to 10 Hz with the files' signals
(GPS L1 C/A, Galileo E1, BeiDou B1I), PVTGeodetic for the fix, its speed (3-D) and its height. limit=600 draws and
shades the receiver's own export limit instead of the scenario's 515 m/s and 80 km windows.

    plot_m8t_run.py CAPTURE OUT.png "TITLE" [SCENARIO] [ACC.npz] [end=T] [xmax=T] [limit=V]
"""
import json
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
limit = next((float(a[6:]) for a in extra if a.startswith("limit=")), None)
acc = np.load(acc_path) if acc_path else None
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
PRO = sc.get("prologue_s", 180.0)
tr = sc["truth"]
T_TR = np.array([s["t"] - PRO for s in tr])       # truth time from ignition
V_UP = np.array([s["v_up_mps"] for s in tr])
BURN = next(s["t"] for s in tr if s["phase"] == "coast") - PRO
G = {0: "GPS", 2: "GAL", 3: "BDS"}
# acceleration from the injected vertical velocity (10 Hz rows), at each block's middle, lightly smoothed
A_T = 0.5 * (T_TR[1:] + T_TR[:-1])
A_UP = np.diff(V_UP) / np.diff(T_TR)
A_UP = np.convolve(A_UP, np.ones(3) / 3, mode="same")

# valid raw pseudoranges (RXM-RAWX) and the receiver's own navigation (NAV-PVT), times from ignition
meas, counts, nav = {}, [], []


def read_sbf(path):
    """A mosaic-G5 capture into meas/counts/nav (fix 3 while PVTGeodetic has a position, speed 3-D, height km)."""
    sys.path.insert(0, str(SDR.parent))
    import septentrio_sbf as sbf
    sig = {"GPS L1CA": "GPS", "GAL E1": "GAL", "BDS B1I": "BDS"}
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 24:
            continue
        h = p[2].strip()
        bid = int(h[10:12] + h[8:10], 16) & 0x1FFF
        if bid not in (sbf.MEAS_EPOCH, sbf.PVT_GEODETIC):
            continue
        tow = int.from_bytes(bytes.fromhex(h[16:24]), "little")
        if tow == 0xFFFFFFFF or (bid == sbf.MEAS_EPOCH and tow % 100):
            continue
        t = tow / 1000.0 - TOW0 - IGN
        try:
            b = bytes.fromhex(h)
        except ValueError:
            continue
        if bid == sbf.MEAS_EPOCH:
            n = {"GPS": 0, "GAL": 0, "BDS": 0}
            for x in sbf.meas_epoch(b)["meas"]:
                if x["signal"] in sig and x["pr"] is not None and x["sv"][1:].isdigit():
                    meas.setdefault((sig[x["signal"]], int(x["sv"][1:])), []).append(t)
                    n[sig[x["signal"]]] += 1
            counts.append((t, n))
        else:
            q = sbf.pvt_geodetic(b)
            if q["mode"] and q["vn"] is not None:
                nav.append((t, 3, (q["vn"] ** 2 + q["ve"] ** 2 + q["vu"] ** 2) ** 0.5, q["h"] / 1000.0))
            else:
                nav.append((t, 0, 0.0, 0.0))


with open(cap, errors="replace") as fh:
    kind = next((p[1] for p in (ln.split(" ", 2) for ln in fh) if len(p) == 3 and p[1] in ("S", "U")), "U")
for line in (open(cap, errors="replace") if kind == "U" else []):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "U":
        continue
    h = p[2].strip()
    try:
        b = bytes.fromhex(h[4:])
    except ValueError:
        continue
    if h.startswith("0215") and len(b) >= 16:
        t = struct.unpack_from("<d", b, 0)[0] - TOW0 - IGN
        n = {"GPS": 0, "GAL": 0, "BDS": 0}
        seen = set()                                 # one signal per satellite (a ZED-F9P can report several)
        for j in range(b[11]):
            m = b[16 + 32 * j: 48 + 32 * j]
            if len(m) == 32 and m[20] in G and m[30] & 1 and (m[20], m[21]) not in seen:
                seen.add((m[20], m[21]))
                sysn = G[m[20]]
                meas.setdefault((sysn, m[21]), []).append(t)
                n[sysn] += 1
        counts.append((t, n))
    elif h.startswith("0107") and len(b) >= 92:
        t = struct.unpack_from("<I", b, 0)[0] / 1000.0 - TOW0 - IGN
        fix = b[20] if (b[21] & 1) and b[20] in (2, 3, 4) else 0
        vn, ve, vd = struct.unpack_from("<iii", b, 48)
        nav.append((t, fix, (vn * vn + ve * ve + vd * vd) ** 0.5 / 1000.0,
                    struct.unpack_from("<i", b, 32)[0] / 1e6))
if kind == "S":
    read_sbf(cap)
counts.sort(key=lambda c: c[0])
nav.sort()


def spans(ts, gap=0.3):
    """Contiguous runs of epochs (10 Hz raw) -> (start, length) bars; a gap over 0.3 s breaks a bar."""
    out, s0, prev = [], None, None
    for t in sorted(ts):
        if s0 is None:
            s0 = prev = t
        elif t - prev > gap:
            out.append((s0, max(prev - s0, 0.1)))
            s0 = t
        prev = t
    if s0 is not None:
        out.append((s0, max(prev - s0, 0.1)))
    return out


INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
COL = {"GPS": "#1f6feb", "GAL": "#c4561a", "BDS": "#1a7f37"}
LIM, ACC_C = "#9a6700", "#8250df"
WRONG, WRONG_M = "#bf3989", 10.0                # flagged valid but this far off the truth
WRONG_WIN = (-5.0, 30.0)                         # marked in the boost only: elsewhere transmitter underruns and the
wrong_t = {}                                     # receiver's clock steps make brief errors of the rig's own
if acc is not None:
    for i, name in ((0, "GPS"), (1, "GAL"), (2, "BDS")):
        m = ((acc["sys"] == i) & (np.abs(acc["pr"]) > WRONG_M) & (acc["t"] >= WRONG_WIN[0]) &
             (acc["t"] <= WRONG_WIN[1]))
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
fixes = [(t, v, hh) for t, st, v, hh in nav if st >= 2]
if limit is None:
    windows = [(a - PRO, b - PRO) for a, b in sc.get("blocked_windows", [])]
else:                                            # the receiver's own export limit: over `limit` m/s in the truth
    windows, w0 = [], None
    for s in tr:
        over = s["speed_mps"] > limit
        if over and w0 is None:
            w0 = s["t"] - PRO
        elif not over and w0 is not None:
            windows.append((w0, s["t"] - PRO))
            w0 = None
LIMIT_V = 515 if limit is None else limit
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
    ax.axhline(LIMIT_V, color=LIM, lw=0.9, ls="--")
    ax.text(x1, LIMIT_V, f" {LIMIT_V:.0f} m/s", color=LIM, va="bottom", ha="right", fontsize=8)
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
    for hh in (18, 80):
        if hh < top:
            ax.axhline(hh, color=LIM, lw=0.9, ls="--")
            ax.text(x1, hh, f" {hh} km", color=LIM, va="bottom", ha="right", fontsize=8)
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
        if wb:                                       # flagged valid but over WRONG_M off
            ax.broken_barh(wb, (i - 0.45, 0.9), facecolors=WRONG, edgecolor="none", zorder=3)
    ax.set_yticks(range(len(sats)))
    ax.set_yticklabels([f"{'G' if k[0] == 'GPS' else ('E' if k[0] == 'GAL' else 'C')}{k[1]:02d}" for k in sats],
                       fontsize=7)
    ax.set_ylim(len(sats) - 0.5, -0.5)
    ax.grid(axis="x", color=GRID, lw=0.5)
    if col == 0:
        ax.set_ylabel("valid raw pseudorange, per satellite")
        if acc is not None:
            ax.text(1.0, 1.005, f"magenta: flagged valid but over {WRONG_M:.0f} m off the truth, T-5 to T+30"
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
        split = float(acc["split"]) if "split" in acc.files else 0.0
        symlog(ax, 5.0, 1000.0, PR_TICKS)
        for yv in (WRONG_M, -WRONG_M):
            ax.axhline(yv, color=WRONG, lw=0.7, ls="--")
        ax.grid(axis="y", color=GRID, lw=0.6)
        if col == 0:
            ax.set_ylabel("GPS pseudorange\nerror, m")
            ax.text(0.005, 0.04, f"linear within ±5 m, logarithmic beyond; dashed: ±{WRONG_M:.0f} m. Code and "
                    f"carrier consistent (code minus carrier {split:+.2f} m/s)", transform=ax.transAxes, fontsize=7.5,
                    color=MUTED)
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
        amber = "over 515 m/s or 80 km" if limit is None else f"over {limit:.0f} m/s"
        axs[NR - 1][col].set_xlabel(f"time from ignition, s   (dotted: ignition and burnout T+{BURN:.1f}; amber: "
                                    f"{amber}; green strip: receiver has a fix)")
    else:
        axs[NR - 1][col].set_xlabel("boost: T-5 to T+30 s")
fig.subplots_adjust(top=1.0 - 0.45 / fig.get_figheight())
fig.suptitle(title, x=0.06, y=1.0 - 0.12 / fig.get_figheight(), ha="left", fontsize=11)
fig.savefig(out, dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out)
