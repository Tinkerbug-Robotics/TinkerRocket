#!/usr/bin/env python3
"""One SkyTraq receiver on an original IQ file and on its carrier-corrected twin
(patch_carrier_offset.py): each satellite's pseudorange minus true range, each epoch's median
removed, on one scale for both runs, then both runs' own-fix height error. The figure behind
the PX1125R result in results/README.md ("The HackRF's carrier runs 22 Hz off its own code").

    ./plot_carrier_ab.py SCENARIO ORIGINAL CORRECTED OUT.svg [--pad-window 300,595] [--span 100,885]
    ./plot_carrier_ab.py spaceshot results/px1125r_spaceshot_pad600_stock_gain2_nav9_el3.log.gz \\
        results/px1125r_spaceshot_pad600_stock_cofs_gain2_nav9_el3.log.gz results/figures/px1125r_carrier_ab.svg

The residuals are tinkerrocket-sim's raw_residuals_cocom.py (the capture's own ephemeris and
broadcast ionosphere, no troposphere: the rig has none); the own fix is own_fix.py. Both
captures are *_pad600 files (ignition at file time 600 s, SHIFT 420). The panel notes give
|residual| p90/p99 and the per-satellite spread over --pad-window; the three satellites with
the widest spread on the original are drawn in color in both panels. SCENARIO is read from
scenarios/ (./make_flights.py, then --retime).
"""
import bisect
import json
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parent
ROOT = SDR.parents[2]
sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "src"))
sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "scripts"))
sys.path.insert(0, str(SDR))
import matplotlib                                                   # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                    # noqa: E402
import numpy as np                                                 # noqa: E402
from tc_ekf_cocom import RigTruth, load_capture                    # noqa: E402
from raw_residuals_cocom import residuals                          # noqa: E402
from own_fix import own_fixes                                      # noqa: E402

PAD_S, SHIFT = 600.0, 420.0
PR_LIM = 130.0                 # the pseudorange panels' axis, m (beyond: drawn at the edge)
H_LO, H_HI = -160.0, 110.0     # the height panel's axis, m (likewise)
INK, MUTED, GRID, DOT = "#1f2328", "#59636e", "#d8dee4", "#9aa4ae"
C_ORIG, C_CORR = "#c4561a", "#1f6feb"
C_SATS = ("#8250df", "#1a7f37", "#bf3989")


def opt(name, default):
    if name in sys.argv:
        i = sys.argv.index(name)
        v = sys.argv[i + 1]
        del sys.argv[i:i + 2]
        return tuple(float(x) for x in v.split(","))
    return default


def main() -> int:
    pad_win = opt("--pad-window", (300.0, 595.0))
    span = opt("--span", (100.0, 885.0))
    if len(sys.argv) != 5:
        print(__doc__)
        return 2
    scen = SDR / "scenarios" / f"{sys.argv[1]}.json"
    orig, corr, out = Path(sys.argv[2]), Path(sys.argv[3]), sys.argv[4]
    rx = orig.name.split("_")[0].upper()

    truth = RigTruth(str(scen), PAD_S)
    tr = json.loads(scen.read_text())["truth"]
    tt = [s["t"] for s in tr]

    def injected(ft, key):
        x = ft - SHIFT
        i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
        a, b = tr[i - 1], tr[i]
        f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
        return a[key] + f * (b[key] - a[key])

    def above(key, thr):
        """File-time spans where the injection is above thr: the COCOM windows."""
        spans, start = [], None
        for s in tr:
            t = s["t"] + SHIFT
            if s[key] > thr and start is None:
                start = t
            elif s[key] <= thr and start is not None:
                spans.append((start, t))
                start = None
        return spans

    windows = above("speed_mps", 515.0) + above("alt_m", 80000.0)
    t_launch = SHIFT + next(s["t"] for s in tr if s["phase"] not in ("prologue", "pad"))
    apo = max(tr, key=lambda s: s["alt_m"])

    res = {}
    for key, path in (("orig", orig), ("corr", corr)):
        res[key] = residuals(truth, load_capture(str(path)), [0.0], 4, *span)[0.0]["pr"]

    def pad_stats(rows):
        a, b = pad_win
        r = np.array([x for t, _p, x in rows if a <= t <= b])
        by = {}
        for t, p, x in rows:
            if a <= t <= b:
                by.setdefault(int(p), []).append(x)
        spread = sorted((max(v) - min(v), p) for p, v in by.items() if len(v) > 50)
        q90, q99 = np.percentile(np.abs(r), [90, 99]) if len(r) else (float("nan"),) * 2
        return q90, q99, spread

    notes, spreads = {}, {}
    for key in res:
        q90, q99, spread = pad_stats(res[key])
        spreads[key] = spread
        lo, hi = (spread[0][0], spread[-1][0]) if spread else (float("nan"),) * 2
        notes[key] = (f"pad {pad_win[0]:.0f}-{pad_win[1]:.0f} s: p90 {q90:.1f} m, p99 {q99:.1f} m, "
                      f"{lo:.0f}-{hi:.0f} m per satellite")
    highlight = [p for _s, p in reversed(spreads["orig"])][:3]

    def heights(path):
        t, dh, last = [], [], None
        for ft, _lat, _lon, h in own_fixes(path):
            if last is not None and ft - last > 1.0:
                t.append(float("nan"))
                dh.append(float("nan"))
            t.append(ft)
            dh.append(min(H_HI, max(H_LO, h - injected(ft, "alt_m"))))
            last = ft
        return t, dh

    plt.rcParams.update({"font.size": 9.5, "text.color": INK, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": GRID,
                         "svg.fonttype": "none"})
    fig, axs = plt.subplots(3, 1, figsize=(11, 9.2), sharex=True,
                            gridspec_kw={"height_ratios": [1, 1, 1.1], "hspace": 0.14})
    events = [(t_launch, "launch"), (apo["t"] + SHIFT, f"apogee {apo['alt_m'] / 1000:.1f} km")]

    def frame(ax, ylabel):
        for a, b in windows:
            ax.axvspan(a, b, color="#eaeef2", lw=0, zorder=0)
        for x, _ in events:
            ax.axvline(x, color=MUTED, lw=0.7, ls=":", zorder=1)
        ax.axhline(0, color=MUTED, lw=0.8, zorder=1)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_ylabel(ylabel)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    for ax, key, label in ((axs[0], "orig", "original IQ file"), (axs[1], "corr", "carrier-corrected IQ file")):
        frame(ax, "pseudorange error, m")
        clip = [(t, int(p), min(PR_LIM, max(-PR_LIM, x))) for t, p, x in res[key]]
        rest = [(t, x) for t, p, x in clip if p not in highlight]
        ax.scatter([t for t, _ in rest], [x for _, x in rest], s=1.2, color=DOT, lw=0,
                   rasterized=True, zorder=2, label="other satellites")
        for prn, col in zip(highlight, C_SATS):
            pts = [(t, x) for t, p, x in clip if p == prn]
            ax.scatter([t for t, _ in pts], [x for _, x in pts], s=1.6, color=col, lw=0,
                       rasterized=True, zorder=3, label=f"G{prn}")
        ax.set_ylim(-PR_LIM - 8, PR_LIM + 8)
        ax.text(0.005, 0.97, label, transform=ax.transAxes, va="top", fontsize=10, color=INK,
                weight="bold")
        ax.text(0.005, 0.86, notes[key], transform=ax.transAxes, va="top", fontsize=8.5, color=MUTED)
    for x, name in events:
        axs[0].text(x + 4, PR_LIM + 4, name, color=MUTED, fontsize=8.5, va="top")
    axs[0].legend(loc="lower left", frameon=False, ncol=4, markerscale=5, fontsize=8.5)
    axs[0].set_title(f"{rx}, {sys.argv[1]}: each satellite's pseudorange minus true range "
                     "(epoch median removed), and the own fix's height error",
                     loc="left", fontsize=10.5, color=INK, pad=10)

    ax = axs[2]
    frame(ax, "own height error, m")
    for path, col, label in ((orig, C_ORIG, "original IQ file"), (corr, C_CORR, "carrier-corrected IQ file")):
        t, dh = heights(path)
        ax.plot(t, dh, color=col, lw=1.4, label=label)
    ax.set_ylim(H_LO - 5, H_HI + 5)
    ax.legend(loc="lower left", frameon=False)
    ax.set_xlabel(f"file time, s (GPS TOW - 203400)\nGrey: over 515 m/s or 80 km (injected), own fix "
                  f"withheld.   Drawn at the edge: pseudorange beyond +/-{PR_LIM:.0f} m, height beyond "
                  f"{H_LO:.0f} / +{H_HI:.0f} m.")
    ax.set_xlim(*span)
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
