#!/usr/bin/env python3
"""The boost tracking chart for the wide SignalSim traveler runs, all three systems: each satellite's line-of-sight
Doppler rate through the burn, in its system's colour where the PX1105R reports a valid pseudorange and grey where
not; one panel per run, in the order given. The rules are those of the report's figure
(tools/gnss-cocom/sdr/plot_doppler_traces.py): satellites tracked into ignition (valid through T-0.5..0); a loss is
the first gap in valid pseudoranges lasting >= 0.5 s up to burnout + 0.5 s; a shared drop is >= 3 losses within
0.25 s at rates more than 1.8x apart (a common event, not a common threshold); the fix-stop line is the last 2D/3D
fix before the truth drops back under 515 m/s. Rate = vertical acceleration x sin(elevation) / wavelength (B1I's
own for BeiDou: 51.5 Hz/s per g on L1/E1, 51.1 on B1I); elevation from the run's pr_accuracy npz.

The npz also gives each delivered measurement's pseudorange error (10 Hz rows; each 20 Hz epoch takes the nearest row
within 0.06 s). A delivered measurement more than WRONG_M (10 m) off is WRONG and gets a magenta band under the trace
(a diamond where it first goes wrong); the panel titles count those satellites. Rows whose clock was bridged by the
Doppler (no code-following epoch within 2 s) are not judged. Each series point carries the error as its 6th element.

    boost_traces_wide.py OUT_STEM LABEL=CAPTURE=ACC.npz [...]      -> OUT_STEM.png, OUT_STEM.json
    (BOOST_SCEN=hotshot in the environment for the hotshot runs; default traveler_soft25)
"""
import json
import math
import os
import struct
import sys
from pathlib import Path

import numpy as np

WT = Path(__file__).resolve().parents[4]
SDR = WT / "tools" / "gnss-cocom" / "sdr"
sys.path[:0] = [str(WT / "tinkerrocket-sim" / "src"), str(WT / "tinkerrocket-sim" / "scripts")]
from tc_ekf_cocom import load_capture, TOW_FILE0                        # noqa: E402

C = 299792458.0
LAM = {"G": C / 1575.42e6, "E": C / 1575.42e6, "C": C / 1561.098e6}
# The same receiver on the real sky (sky_levels.py on captures/px1105r_sky_20260929_1644.log, 2 h, roof antenna,
# elevation >= 10 deg): median C/N0 over all elevations, and above 60 deg -- where the boost's steepest rates are.
SKY = {"G": (39, 42), "C": (41, 47), "E": (40, 41)}
SKY_NOTE = ("Sky reference: the same PX1105R on the roof antenna (2 h, 09-29), median C/N0 GPS 39, BeiDou 41, Galileo "
            "40 dB-Hz; above 60 deg elevation GPS 42, BeiDou 47, Galileo 41. Bench satellites all share one level.")
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
SYSN = "GEC"
IGN, SUSTAIN, JOLT = 600.0, 0.5, 0.5
WRONG_M, WRONG = 10.0, "#bf3989"           # a delivered pseudorange this far off the truth is wrong; magenta
SC = json.loads((SDR / "scenarios" / (os.environ.get("BOOST_SCEN", "traveler_soft25") + ".json")).read_text())
TR, PRO = SC["truth"], SC["prologue_s"]
TT = [s["t"] - PRO + IGN for s in TR]                                   # file time


def a_up(ft):
    i = min(max(int(np.searchsorted(TT, ft)), 1), len(TT) - 1)
    return (TR[i]["v_up_mps"] - TR[i - 1]["v_up_mps"]) / (TT[i] - TT[i - 1])


BURN = next(t for s, t in zip(TR, TT) if s["phase"] == "coast")         # burnout, file time


def thrust_off():
    prev = None
    for a, b, ta, tb in zip(TR, TR[1:], TT, TT[1:]):
        if ta < IGN:
            continue
        acc = (b["v_up_mps"] - a["v_up_mps"]) / (tb - ta)
        if prev is not None and prev > 20.0 and acc < 0.5 * prev:
            return ta
        prev = acc
    return BURN


STEP = thrust_off()


def back_under():
    over = False
    for s, t in zip(TR, TT):
        if t < IGN:
            continue
        if s["speed_mps"] >= 515.0:
            over = True
        elif over:
            return t
    return TT[-1]


def last_fix(path):
    horizon = back_under() - 1.0 - IGN
    last = None
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) == 3 and p[1] == "B" and p[2].startswith("df"):
            x = bytes.fromhex(p[2].strip())
            if len(x) >= 61 and x[2] >= 2:
                t = struct.unpack(">d", x[5:13])[0] - TOW_FILE0 - IGN
                if 0.0 <= t <= horizon:
                    last = t
    return last


LOCK_DROP = 6.0     # locked = channel-status C/N0 within this of the system's pad median (tracked channels hold their
LOCK_FLOOR = 20     # full reading under the boost and fall away in one or two seconds when they go; reacquisition
                    # attempts read 1-22); never below LOCK_FLOOR


def lock_epochs(path):
    """[(t from ignition, {(sys, prn): C/N0})] from the 1 Hz channel status (0xE7), each timed by the latest 0xDF."""
    sysmap = {0: "G", 3: "E", 5: "C"}
    t_df, out = None, []
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "B" or not p[2].startswith(("df", "e7")):
            continue
        x = bytes.fromhex(p[2].strip())
        if x[0] == 0xDF and len(x) >= 61:
            t_df = struct.unpack(">d", x[5:13])[0] - TOW_FILE0 - IGN
        elif x[0] == 0xE7 and len(x) >= 4 and t_df is not None and -5.0 <= t_df <= BURN - IGN + 5.0:
            ch = {}
            for j in range(x[3]):
                b = x[4 + 7 * j: 4 + 7 * (j + 1)]
                if len(b) == 7 and (b[1] & 0x0F) in sysmap and (b[1] >> 4) in (0, 1):
                    ch[(sysmap[b[1] & 0x0F], b[2])] = int.from_bytes(b[5:6], "big", signed=True)
            out.append((t_df, ch))
    return out


def run(label, cap, npz):
    z = np.load(npz)
    errs = {}                                  # (sys, prn) -> (times from ignition, errors, bridged), by time
    for i, c in enumerate(SYSN):
        for prn in set(int(p) for p in z["prn"][z["sys"] == i]):
            m = (z["sys"] == i) & (z["prn"] == prn)
            o = np.argsort(z["t"][m])
            errs[(c, prn)] = (z["t"][m][o], z["pr"][m][o], z["bridged"][m][o])

    def err_at(key, t_rel):
        """(error, bridged) of the nearest 10 Hz row within 0.06 s, else (None, False)."""
        e = errs.get(key)
        if e is None or not len(e[0]):
            return None, False
        j = int(np.clip(np.searchsorted(e[0], t_rel), 1, len(e[0]) - 1))
        j = j - 1 if abs(e[0][j - 1] - t_rel) <= abs(e[0][j] - t_rel) else j
        return (float(e[1][j]), bool(e[2][j])) if abs(e[0][j] - t_rel) <= 0.06 else (None, False)

    el, pad_cn0 = {}, {}
    for i, c in enumerate(SYSN):
        m = (z["sys"] == i) & (z["t"] >= -5.0) & (z["t"] <= 0.0)
        for prn in set(int(p) for p in z["prn"][m]):
            el[(c, prn)] = float(np.median(z["el"][m & (z["prn"] == prn)]))
        mp = (z["sys"] == i) & (z["t"] >= -60.0) & (z["t"] <= -5.0)
        if mp.any():
            pad_cn0[c] = float(np.median(z["cn0"][mp]))
    eph, ep, _own, _kind = load_capture(str(cap), systems=SYSN)
    win = BURN + JOLT
    grid, seen = {}, {}
    for k in ep:
        tow, obs = ep[k]
        t = round(tow - TOW_FILE0, 2)
        if IGN - 1.0 <= t <= win + SUSTAIN:
            grid[t] = {(o[0], o[1]): o[2] is not None for o in obs}
            seen[t] = {(o[0], o[1]) for o in obs}                    # reported at all, valid or not
    times = sorted(grid)
    t_last = max(times)
    ends = t_last + 0.05 if t_last < win + SUSTAIN - 0.06 else win + 60.0
    locks = lock_epochs(cap)
    lt = [t for t, _ in locks]
    thr = {c: max(LOCK_FLOOR, pad_cn0.get(c, 30.0) - LOCK_DROP) for c in SYSN}

    def locked(key, t_rel):
        """Sample-and-hold of the latest channel status at or before t_rel (s from ignition)."""
        i = int(np.searchsorted(lt, t_rel, side="right")) - 1
        return i >= 0 and locks[i][1].get(key, 0) >= thr[key[0]]

    recs = []
    for key in sorted({kk for t in times for kk in grid[t]}):
        pre = [grid[t].get(key, False) for t in times if IGN - 0.5 <= t < IGN]
        if not pre or not all(pre) or key not in el:
            continue
        s = math.sin(math.radians(el[key]))
        post = [(t, grid[t].get(key, False)) for t in times if t >= IGN]
        loss = None
        for i, (t, ok) in enumerate(post):
            if t > win:
                break
            if ok:
                continue
            nxt = next((tt for tt, ok_ in post[i:] if ok_), ends)
            if nxt - t >= SUSTAIN:
                loss = t
                break
        lam = LAM[key[0]]
        series = []
        for t, ok in post:
            if t > win:
                continue
            e, br = err_at(key, round(t - IGN, 2)) if ok else (None, False)
            series.append((round(t - IGN, 2), bool(ok), a_up(t + 0.001) * s / lam, key in seen[t],
                           locked(key, t - IGN), None if br else e))      # bridged rows are not judged
        bad = [p for p in series if p[1] and p[5] is not None and abs(p[5]) > WRONG_M and p[0] < STEP - IGN]
        t_end = loss if loss is not None else min(STEP - 0.05, t_last)
        # loss of LOCK from the channel status: first 1 Hz epoch after ignition without it, placed half way back to
        # the last epoch that had it (the status is 1 Hz, so +-0.5 s)
        lock_loss = None
        prev_t = None
        for tl, ch in locks:
            if tl < 0.0:
                prev_t = tl
                continue
            if tl > STEP - IGN:
                break
            if ch.get(key, 0) < thr[key[0]]:
                lock_loss = 0.5 * (tl + (prev_t if prev_t is not None and prev_t >= 0.0 else 0.0))
                break
            prev_t = tl
        lt_end = lock_loss if lock_loss is not None else STEP - 0.05 - IGN
        # state at burnout: locked in the last channel status before the thrust step; raw delivered in the last 0.5 s
        lock_end = locked(key, STEP - IGN - 0.05)
        raw_end = any(ok for t, ok in post if STEP - 0.5 <= t < STEP)
        recs.append(dict(run=label, sys=key[0], prn=key[1], el=el[key], lost=loss is not None, t=t_end - IGN,
                         rate=a_up(min(t_end, STEP - 0.05) + 0.001) * s / lam,
                         lock_lost=lock_loss is not None, lock_t=lt_end,
                         lock_rate=a_up(min(lt_end + IGN, STEP - 0.05) + 0.001) * s / lam,
                         lock_end=lock_end, raw_end=raw_end, series=series, wrong=bool(bad),
                         wrong_t=bad[0][0] if bad else None, wrong_rate=bad[0][2] if bad else None,
                         err_max=max((abs(p[5]) for p in series if p[1] and p[5] is not None and p[0] < STEP - IGN),
                                     default=None)))
    return dict(label=label, recs=recs, fix_end=last_fix(cap), pad_cn0=pad_cn0)


args = sys.argv[1:]
out = args[0]
runs = []
for a in args[1:]:
    label, cap, npz = a.split("=")
    runs.append(run(label, cap, npz))
Path(out + ".json").write_text(json.dumps([{k: v for k, v in r.items()} for r in runs], indent=1))

import matplotlib                                                       # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                        # noqa: E402

INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
COL = {"G": "#1f6feb", "E": "#c4561a", "C": "#1a7f37"}
GROUPC = "#8c959f"
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": GRID})
n = len(runs)
ncol = 2 if n > 1 else 1
nrow = math.ceil(n / ncol)
fig, axs = plt.subplots(nrow, ncol, sharex=True, sharey=True, figsize=(6.2 * ncol, 4.25 * nrow + 0.7), squeeze=False)
X_END = BURN + JOLT - IGN + 0.3
STEP_T, BURN_T = STEP - IGN, BURN - IGN
for ax, r in zip(axs.flat, runs):
    rs = r["recs"]
    for q in sorted(rs, key=lambda q: q["el"]):
        ser = [s for s in q["series"] if s[0] < STEP_T]
        ts = [s[0] for s in ser]
        vs = [s[2] for s in ser]
        ax.plot(ts, vs, color=GRID, lw=0.9, zorder=1)
        ax.plot([s[0] if (s[4] and not s[1]) else float("nan") for s in ser], vs, color=COL[q["sys"]], lw=2.6,
                alpha=0.28, solid_capstyle="butt", zorder=2)                # locked, measurement withheld
        ax.plot([s[0] if s[1] else float("nan") for s in ser], vs, color=COL[q["sys"]], lw=1.3, zorder=3)
        if q.get("wrong"):                            # delivered but over WRONG_M off: a magenta band under the trace
            ax.plot([s[0] if s[1] and len(s) > 5 and s[5] is not None and abs(s[5]) > WRONG_M else float("nan")
                     for s in ser], vs, color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt", zorder=2)
            ax.scatter(q["wrong_t"], q["wrong_rate"], marker="D", s=22, color=WRONG, zorder=6)
        if q["lock_lost"] and q["lock_end"]:          # dropped, then locked again by burnout
            ax.scatter(q["lock_t"], q["lock_rate"], marker="o", s=30, facecolors="none", edgecolors=COL[q["sys"]],
                       lw=1.4, zorder=5)
        elif q["lock_lost"]:
            ax.scatter(q["lock_t"], q["lock_rate"], marker="x", s=34, lw=1.6, color=COL[q["sys"]], zorder=5)
        else:
            ax.scatter(STEP_T - 0.05, q["lock_rate"], marker="^", s=28, facecolors="none", edgecolors=COL[q["sys"]],
                       lw=1.1, zorder=5)
    fe = r["fix_end"]
    if fe is not None and fe < X_END - 0.2:
        ax.axvline(fe, color=MUTED, lw=0.9, ls="--", zorder=0)
        ax.annotate(f"fix stops T+{fe:.1f}", (fe, 0.98), xycoords=("data", "axes fraction"), xytext=(3, 0),
                    textcoords="offset points", fontsize=8, color=MUTED, va="top")
    elif fe is not None:
        ax.annotate(f"fix stops T+{fe:.1f} →", (0.99, 0.98), xycoords="axes fraction", ha="right", fontsize=8,
                    color=MUTED, va="top")
    ax.axvline(BURN_T, color=MUTED, lw=0.7, ls=":", zorder=0)
    ax.annotate("burnout", (BURN_T, 0.02), xycoords=("data", "axes fraction"), xytext=(-3, 0), ha="right",
                textcoords="offset points", fontsize=8, color=MUTED, va="bottom")
    ab = {"G": "GPS", "C": "BDS", "E": "GAL"}
    kept, raw, bad = [], [], []
    for c in "GCE":
        tot = [q for q in rs if q["sys"] == c]
        if tot:
            kept.append(f"{ab[c]} {sum(q['lock_end'] for q in tot)}/{len(tot)}")
            raw.append(f"{ab[c]} {sum(q['raw_end'] for q in tot)}/{len(tot)}")
            nb = sum(bool(q.get("wrong")) for q in tot)
            if nb:
                bad.append(f"{ab[c]} {nb}")
    gc = [c for c in "GC" if c in r["pad_cn0"]]
    cn = ", ".join(f"{ab[c]} {r['pad_cn0'][c]:.0f}" for c in gc)
    d_med = "/".join(f"{r['pad_cn0'][c] - SKY[c][0]:+.0f}" for c in gc)
    d_hi = "/".join(f"{r['pad_cn0'][c] - SKY[c][1]:+.0f}" for c in gc)
    ax.set_title(f"{r['label']}\npad C/N0 {cn} dB-Hz;  vs sky {d_med} dB (median), {d_hi} dB (above 60°)\n"
                 f"at burnout: locked {', '.join(kept)};  raw {', '.join(raw)}\n"
                 f"over {WRONG_M:.0f} m off while delivered: {', '.join(bad) if bad else 'none'}", loc="left",
                 fontsize=8.4)
    ax.grid(axis="y", color=GRID, lw=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
for ax in list(axs.flat)[n:]:
    ax.set_visible(False)
for ax in axs[-1]:
    ax.set_xlabel("time from ignition, s")
for ax in axs[:, 0]:
    ax.set_ylabel("line-of-sight Doppler rate, Hz/s")
y_top = max((s[2] for r in runs for q in r["recs"] for s in q["series"] if s[0] <= X_END), default=900.0)
axs[0][0].set_ylim(0, max(950.0, 1.06 * y_top))
axs[0][0].set_xlim(-0.3, X_END)
h = [plt.Line2D([], [], color=COL["G"], lw=1.4, label="GPS: raw pseudorange delivered"),
     plt.Line2D([], [], color=COL["C"], lw=1.4, label="BeiDou: delivered"),
     plt.Line2D([], [], color=COL["E"], lw=1.4, label="Galileo: delivered"),
     plt.Line2D([], [], color=INK, lw=2.6, alpha=0.28, label="locked (channel status), measurement withheld"),
     plt.Line2D([], [], color=GRID, lw=2, label="lock lost"),
     plt.Line2D([], [], marker="x", ls="", color=INK, label="lock lost here, for good (1 Hz status, +-0.5 s)"),
     plt.Line2D([], [], marker="o", ls="", markerfacecolor="none", markeredgecolor=INK,
                label="lock lost here, back by burnout"),
     plt.Line2D([], [], marker="^", ls="", markerfacecolor="none", markeredgecolor=INK, label="never lost lock"),
     plt.Line2D([], [], color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt",
                label=f"delivered but over {WRONG_M:.0f} m off the truth"),
     plt.Line2D([], [], marker="D", ls="", color=WRONG, label=f"where it first goes over {WRONG_M:.0f} m")]
fig.legend(handles=h, loc="lower center", ncol=4, frameon=False, fontsize=8.5, bbox_to_anchor=(0.5, -0.05))
fig.text(0.06, -0.09 if nrow > 1 else -0.18, SKY_NOTE + " Pseudorange errors from pr_accuracy.py (truth, solved clock; "
         "rows with a Doppler-bridged clock not judged).", fontsize=8.5, color=MUTED, ha="left")
fig.subplots_adjust(hspace=0.72, wspace=0.12, bottom=0.11 if nrow > 1 else 0.2, top=0.93 if nrow >= 3 else 0.88)
BURN_NAME = {"traveler_soft25": "the traveler burn (13 s)", "hotshot": "the hotshot burn (4 s, 10 to 40 g)"}
fig.suptitle(f"PX1105R through {BURN_NAME.get(os.environ.get('BOOST_SCEN', 'traveler_soft25'), 'the burn')}, by signal "
             "level: each satellite's line-of-sight Doppler rate (wide SignalSim file, carrier corrected)",
             x=0.06, y=1.0 + 0.1 / nrow, ha="left", fontsize=10.5)
fig.savefig(out + ".png", dpi=140, bbox_inches="tight", facecolor="white")
for r in runs:
    rs = r["recs"]
    parts = []
    for c in "GCE":
        tot = [q for q in rs if q["sys"] == c]
        if tot:
            lo = sorted((q for q in tot if q["lock_lost"]), key=lambda q: q["lock_t"])
            parts.append(f"{NAME[c]} {len(tot)} in, locked at burnout {sum(q['lock_end'] for q in tot)}, delivering "
                         f"{sum(q['raw_end'] for q in tot)}; first losses"
                         + (" (" + ", ".join(f"{c}{q['prn']:02d} T+{q['lock_t']:.1f}/{q['lock_rate']:.0f}"
                                             + ("↺" if q["lock_end"] else "") for q in lo) + ")" if lo else " none"))
    fe = r["fix_end"]
    print(f"{r['label']}: " + "; ".join(parts) + f"; fix stops " + (f"T+{fe:.2f}" if fe is not None else "-"))
print("wrote", out + ".png")
