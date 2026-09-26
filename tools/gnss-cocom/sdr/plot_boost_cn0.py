#!/usr/bin/env python3
"""C/N0 run against run through the spaceshot burn, from LC86G MSM7 captures.

    ./plot_boost_cn0.py OUT.png LABEL=CAPTURE [LABEL=CAPTURE ...]     (up to 8 runs)

Top: each run's pad level (150-179 s) by satellite, which shows a level offset
between runs as a whole row sitting apart. Below: the six satellites the burn moves
least (the ones at or near the tracking limit), 176-200 s, each relative to its own
run's pad median so that offset drops out; filled marker = carrier phase locked,
open = half-cycle flag set (frequency-only tracking); a line ends where MSM7 stops
measuring the satellite. MSM7 is one epoch a second at every fix rate.

Also prints the table behind it: pad C/N0 per run, and the burn (183-192 s) mean
and minimum against the pad.
"""
from __future__ import annotations

import gzip
import json
import statistics as st
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.lines import Line2D                              # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import rtcm3                                                      # noqa: E402

TOW0 = 203400.0
INK, INK2, INK3, RULE = "#1d2129", "#454b54", "#6b7280", "#e3e6ea"
# categorical slots in fixed order (validated for colour-vision deficiency); the two
# low-contrast ones are why every line also has a legend entry and the table prints
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#7b61ff", "#8a5a44"]
BURN_SATS = (29, 12, 30, 19, 14, 15)


def lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        for raw in fh:
            yield raw.rstrip("\n").partition(" ")[2]


def load(path):
    """prn -> {whole second: (cn0, lock s, half-cycle flag)}, 100-330 s."""
    ser = {}
    for rest in lines(path):
        if not rest.startswith("R "):
            continue
        try:
            r = rtcm3.parse_msm7(bytes.fromhex(rest[2:]))
        except ValueError:
            continue
        if not r or r[0] != "GPS":
            continue
        ft = round(r[1] / 1000.0 - TOW0)
        if 100 <= ft <= 330:
            for c in r[2]:
                if c.get("cn0"):
                    ser.setdefault(c["prn"], {})[ft] = (c["cn0"], c["lock_ms"] / 1000.0, c["halfcyc"])
    return ser


def main() -> int:
    out, specs = sys.argv[1], [a.split("=", 1) for a in sys.argv[2:]]
    if not specs or len(specs) > len(PALETTE):
        raise SystemExit(f"1-{len(PALETTE)} LABEL=CAPTURE pairs")
    ref = json.loads((HERE / "results" / "doppler_ref_spaceshot.json").read_text())["sats"]
    rate = {int(k.split(":")[1]): v.get("rate_hzs") for k, v in ref.items() if k.startswith("0:")}
    order = sorted(rate, key=lambda p: (rate[p] is None, rate[p] or 0))
    runs = []
    for i, (label, path) in enumerate(specs):
        ser = load(Path(path))
        pad = {p: st.median(v[0] for t, v in s.items() if 150 <= t < 179)
               for p, s in ser.items() if sum(1 for t in s if 150 <= t < 179) >= 10}
        allpad = st.median(v[0] for p in pad for t, v in ser[p].items() if 150 <= t < 179)
        runs.append((label, PALETTE[i], ser, pad, allpad))

    # the table
    print(f"{'sat':<4}{'Hz/s':>5} | pad C/N0 by run" + " " * max(0, 6 * len(runs) - 15) +
          " spread | burn 183-192 vs own pad: mean (min)")
    for p in order:
        pads = [r[3].get(p) for r in runs]
        cells = []
        for _l, _c, ser, pad, _a in runs:
            b = ([v[0] - pad[p] for t, v in ser.get(p, {}).items() if 183 <= t <= 192]
                 if p in pad else [])
            cells.append(f"{st.mean(b):+5.1f}({min(b):+4.1f})" if len(b) >= 9 else
                         ("   part     " if b else "   lost     "))
        ps = [x for x in pads if x is not None]
        print(f"G{p:02d} {('-' if rate[p] is None else f'{rate[p]:.0f}'):>5} | " +
              " ".join(f"{x:5.1f}" if x else "   --" for x in pads) +
              f"  {(max(ps) - min(ps)) if len(ps) > 1 else 0:4.1f} | " + " ".join(cells))

    fig = plt.figure(figsize=(11.5, 8.4), dpi=150)
    gs = fig.add_gridspec(3, 3, height_ratios=[1.0, 1.25, 1.25], hspace=0.62, wspace=0.16,
                          left=0.07, right=0.985, top=0.855, bottom=0.07)
    ax = fig.add_subplot(gs[0, :])
    w = 0.14 * min(len(runs), 4) / max(len(runs) - 1, 1)
    for i, (_l, col, _s, pad, _a) in enumerate(runs):
        o = (i - (len(runs) - 1) / 2) * w
        pts = [(k + o, pad[p]) for k, p in enumerate(order) if p in pad]
        ax.scatter([x for x, _ in pts], [y for _, y in pts], s=22, color=col,
                   edgecolor="#ffffff", linewidth=0.8, zorder=3)
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels([f"G{p:02d}\n{('–' if rate[p] is None else f'{rate[p]:.0f}')}" for p in order],
                       fontsize=7, color=INK3)
    ax.set_ylabel("pad C/N0, dBHz", fontsize=8, color=INK3)
    ax.tick_params(axis="y", labelsize=7.5, colors=INK3)
    ax.tick_params(axis="x", length=0)
    ax.grid(axis="y", color=RULE, lw=0.6)
    for sp in ax.spines.values():
        sp.set_color(RULE)
    ax.set_title("Pad, 150–179 s: each run's level by satellite "
                 "(x labels: satellite, burn Doppler rate Hz/s)", fontsize=8.5, color=INK2, loc="left")

    for k, p in enumerate(BURN_SATS):
        a = fig.add_subplot(gs[1 + k // 3, k % 3])
        for _l, col, ser, pad, _a in runs:
            if p not in pad:
                continue
            pts = sorted((t, v) for t, v in ser.get(p, {}).items() if 176 <= t <= 200)
            seg = []
            for t, v in pts + [(None, None)]:          # sentinel flushes the last segment
                if seg and (t is None or t - seg[-1][0] > 1):
                    a.plot([x for x, _ in seg], [y[0] - pad[p] for _, y in seg], color=col, lw=1.3)
                    seg = []
                if t is not None:
                    seg.append((t, v))
            for t, v in pts:
                a.plot(t, v[0] - pad[p], marker="o", ms=3.6, color=col,
                       mfc=col if v[2] == 0 else "#ffffff", mew=1.0, zorder=3)
        for x in (180.0, 192.1):
            a.axvline(x, color=INK3, lw=0.7, ls=(0, (2, 2)))
        a.set_xlim(176, 200)
        a.set_ylim(-9, 1.5)
        a.axhline(0, color=RULE, lw=0.8, zorder=0)
        a.set_title(f"G{p:02d} · {rate[p]:.0f} Hz/s", fontsize=8.5, color=INK, loc="left")
        a.tick_params(labelsize=7, colors=INK3)
        for sp in a.spines.values():
            sp.set_color(RULE)
        a.grid(color=RULE, lw=0.5)
        if k % 3 == 0:
            a.set_ylabel("C/N0 vs own pad, dB", fontsize=7.5, color=INK3)
        else:
            a.set_yticklabels([])
        if k >= 3:
            a.set_xlabel("file time (s)", fontsize=7.5, color=INK3)
        if k == 0:
            a.text(180.2, 1.0, "ignition", fontsize=6.5, color=INK3, va="top")
            a.text(192.3, 1.0, "burnout", fontsize=6.5, color=INK3, va="top")
    fig.text(0.07, 0.975, "LC86G through the 13.5 g burn: C/N0 run against run",
             fontsize=11.5, fontweight="bold", color=INK, va="top")
    fig.text(0.07, 0.945, "Burn panels are relative to each run's own pad level, so a level offset "
             "between runs drops out. One MSM7 epoch per second; filled = carrier phase locked, "
             "open = half-cycle flag set (frequency-only tracking); a line ends where the satellite "
             "is no longer measured.", fontsize=7.3, color=INK3, va="top", wrap=True)
    handles = [Line2D([], [], color=col, lw=1.6, marker="o", ms=4, label=f"{label} · pad {allpad:.1f} dBHz")
               for label, col, _s, _p, allpad in runs]
    handles += [Line2D([], [], color=INK3, lw=0, marker="o", ms=4, mfc=INK3, label="phase locked"),
                Line2D([], [], color=INK3, lw=0, marker="o", ms=4, mfc="#ffffff", label="half-cycle flag set")]
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.065, 0.92),
               ncol=min(len(handles), 6), frameon=False, fontsize=7.5, handletextpad=0.4,
               columnspacing=1.4, labelcolor=INK2)
    fig.savefig(out)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
