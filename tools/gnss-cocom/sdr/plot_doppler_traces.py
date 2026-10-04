#!/usr/bin/env python3
"""Each satellite's line-of-sight Doppler rate through a boost, dark while the receiver reports a
valid pseudorange for it: the view behind the boost-versus-signal report (cn0_boost_report.html).

Per satellite tracked at ignition: its first SUSTAINED loss of a valid pseudorange (>= 0.5 s)
between ignition and half a second after burnout (the burnout jolt, thrust off in one step;
burnout is the scenario's own), and the Doppler rate it was under then -- for a loss in the
jolt, the rate just before burnout (vertical kinematic
acceleration x sin(elevation) / L1 wavelength; 51.5 Hz/s per g along the line of sight), or,
if it never dropped, the rate it held at burnout -- or at the last raw epoch, for a receiver that
withholds raw above 515 m/s (u-blox). Reads SkyTraq (0xE5/0xDF) and u-blox (RXM-RAWX/NAV-PVT)
captures. Writes a JSON of the records and figures:
  OUT_scatter.png   Doppler rate at loss vs expected C/N0 (ladder mode only)
  OUT_survival.png  share of satellites still tracked vs the Doppler rate reached, per run
  OUT_traces.png    each satellite's rate through the burn, per run (OUT_traces.svg with --svg)

    ./plot_doppler_traces.py OUT [--compare] [--svg] LABEL:CN0[:SCENARIO]=CAPTURE [...]
CN0 is the expected dB-Hz of an equal-power run, or plS (e.g. pl-17.3) for a gps-sdr-sim
path-loss file scaled by S dB: each satellite then gets 65 + 20 log10(20200 km / range)
- antenna pattern(elevation) + S (bench calibration: 10 dB pad, HackRF gain 3). SCENARIO
defaults to traveler (traveler_soft, traveler_soft25, hotshot for the others). A LABEL ending
in * marks a capture with the SkyTraq 13 s collapse cycle. --compare lays runs out in the order
given (a same-level A/B, or one receiver's ladder) and skips the scatter.
"""
import bisect
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

SDR = Path(__file__).resolve().parent
ROOT = SDR.parents[2]
sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "src"))
sys.path.insert(0, str(ROOT / "tinkerrocket-sim" / "scripts"))
from tc_ekf_cocom import RigTruth, load_capture, TOW_FILE0          # noqa: E402
from tinkerrocket_sim.estimation.gnss_raw import sat_pos             # noqa: E402

LAM = 299792458.0 / 1575.42e6
IGN = 600.0          # file time of ignition (the pad600 files); burnout comes from the scenario
SUSTAIN = 0.5
JOLT = 0.5           # the window runs this far past burnout
ANT_PAT_DB = [0.00, 0.00, 0.22, 0.44, 0.67, 1.11, 1.56, 2.00, 2.44, 2.89, 3.56, 4.22, 4.89, 5.56, 6.22, 6.89,
              7.56, 8.22, 8.89, 9.78, 10.67, 11.56, 12.44, 13.33, 14.44, 15.56, 16.67, 17.78, 18.89, 20.00,
              21.33, 22.67, 24.00, 25.56, 27.33, 29.33, 31.56]           # gps-sdr-sim ant_pat_db
_SC = {}


def scen(name):
    if name not in _SC:
        path = SDR / f"scenarios/{name}.json"
        tr_ = json.loads(path.read_text())["truth"]
        _SC[name] = (RigTruth(str(path), 600.0), tr_, [s["t"] + 420.0 for s in tr_])
    return _SC[name]


def a_up(ft, name):
    _truth, tr_, tt_ = scen(name)
    i = min(max(bisect.bisect_left(tt_, ft), 1), len(tt_) - 1)
    return (tr_[i]["v_up_mps"] - tr_[i - 1]["v_up_mps"]) / (tt_[i] - tt_[i - 1])


def burn_end(name):
    """File time of burnout: the scenario truth's first coast sample (traveler 613, hotshot 604)."""
    _truth, tr_, tt_ = scen(name)
    return next(t for s_, t in zip(tr_, tt_) if s_["phase"] == "coast")


def thrust_off(name):
    """File time the truth's thrust step begins: the first 0.1 s interval after ignition whose
    vertical acceleration is under half the one before (hotshot 603.9, traveler 613.0). Rates are
    read, and traces drawn, up to here: the interval after it is already part way into coast."""
    _truth, tr_, tt_ = scen(name)
    prev = None
    for a, b, ta, tb in zip(tr_, tr_[1:], tt_, tt_[1:]):
        if ta < IGN:
            continue
        acc = (b["v_up_mps"] - a["v_up_mps"]) / (tb - ta)
        if prev is not None and prev > 20.0 and acc < 0.5 * prev:
            return ta
        prev = acc
    return burn_end(name)


def back_under(name):
    """File time the truth first drops back under 515 m/s after passing it (hotshot 622.3,
    traveler 698.9); the flight's end when it never does."""
    _truth, tr_, tt_ = scen(name)
    over = False
    for s_, t in zip(tr_, tt_):
        if t < IGN:
            continue
        if s_["speed_mps"] >= 515.0:
            over = True
        elif over:
            return t
    return tt_[-1]


def last_fix(path, sname):
    """Time from ignition of the receiver's last 2D/3D fix (0xDF state >= 2) before the truth
    drops back under 515 m/s: where its own COCOM check stopped the solution. On a weak signal
    that can be after burnout -- its own speed lags."""
    horizon = back_under(sname) - 1.0 - IGN
    last = None
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3:
            continue
        if p[1] == "B" and p[2].startswith("df"):                  # SkyTraq 0xDF
            x = bytes.fromhex(p[2].strip())
            if len(x) >= 81 and x[2] >= 2:
                t = struct.unpack(">d", x[5:13])[0] - TOW_FILE0 - IGN
                if 0.0 <= t <= horizon:
                    last = t
        elif p[1] == "U" and p[2].startswith("0107"):              # u-blox NAV-PVT
            x = bytes.fromhex(p[2].strip())[2:]
            if len(x) >= 24 and x[20] >= 2 and x[21] & 1:           # fixType 2D/3D, gnssFixOK
                t = int.from_bytes(x[0:4], "little") / 1000.0 - TOW_FILE0 - IGN
                if 0.0 <= t <= horizon:
                    last = t
    return last


FIX_END = {}
DATA_END = {}      # label -> time from ignition the raw stream stopped inside the burn (u-blox withholding)
BURN_T = {}        # label -> burnout, s from ignition
STEP_T = {}        # label -> start of the thrust step, s from ignition (traces stop here)


def records(label, cn0spec, sname, path):
    BURN = burn_end(sname)
    WIN = BURN + JOLT
    BURN_T[label] = BURN - IGN
    STEP = thrust_off(sname)
    STEP_T[label] = STEP - IGN
    FIX_END[label] = last_fix(path, sname)
    truth, _tr, _tt = scen(sname)
    eph, ep, _own, _kind = load_capture(str(path))
    grid = {}                                    # epoch time -> {prn: valid}
    for k in ep:
        tow, obs = ep[k]
        t = round(tow - TOW_FILE0, 2)
        if IGN - 1.0 <= t <= WIN + SUSTAIN:
            grid[t] = {o[1]: o[2] is not None for o in obs if o[0] == "G"}
    times = sorted(grid)
    t_last = max(times) if times else IGN
    DATA_END[label] = (t_last - IGN) if t_last < WIN else None
    # a loss needs SUSTAIN s without the satellite; where the stream itself ends first, that end
    # counts as a recovery (the receiver stopped reporting everything, not this satellite)
    ends = t_last + 0.05 if t_last < WIN + SUSTAIN - 0.06 else WIN + 60.0
    r = truth.pos_ecef(IGN)
    up = r / np.linalg.norm(r)
    out = []
    for prn in sorted({p for t in times for p in grid[t]}):
        pre = [grid[t].get(prn, False) for t in times if IGN - 0.5 <= t < IGN]
        if not pre or not all(pre):
            continue                             # not tracked going into ignition
        e = eph.pick("G", prn, IGN + TOW_FILE0)
        if not e:
            continue
        rs, _ = sat_pos(e, IGN + TOW_FILE0 - 0.075)
        rng = float(np.linalg.norm(rs - r))
        u = (rs - r) / rng
        el = math.degrees(math.asin(float(u @ up)))
        if isinstance(cn0spec, str):             # path-loss file: this satellite's own level
            ibs = int((90.0 - el) / 5.0)
            cn0 = 65.0 + 20 * math.log10(20200000.0 / rng) - ANT_PAT_DB[ibs] + float(cn0spec[2:])
        else:
            cn0 = cn0spec
        s = math.sin(math.radians(el))
        post = [(t, grid[t].get(prn, False)) for t in times if t >= IGN]
        loss = None
        for i, (t, ok) in enumerate(post):
            if t > WIN:
                break
            if ok:
                continue
            nxt = next((tt_ for tt_, ok_ in post[i:] if ok_), ends)
            if nxt - t >= SUSTAIN:
                loss = t
                break
        series = [(round(t - IGN, 2), bool(ok), a_up(t + 0.001, sname) * s / LAM) for t, ok in post
                  if t <= WIN]
        # held: to burnout, or to the last raw epoch when the receiver stopped reporting (a u-blox
        # withholds raw above 515 m/s) -- then "held" means held for as long as it told us
        t_end = loss if loss is not None else min(STEP - 0.05, t_last)
        out.append(dict(run=label, cn0=cn0, spread=isinstance(cn0spec, str), prn=prn, el=el,
                        lost=loss is not None, t=t_end - IGN,
                        rate=a_up(min(t_end, STEP - 0.05) + 0.001, sname) * s / LAM,
                        series=series))
    return out


args = sys.argv[1:]
compare = "--compare" in args
svg_out = "--svg" in args          # also write OUT_traces.svg for the report (lines rasterized, no suptitle)
args = [a for a in args if a not in ("--compare", "--svg")]
out_stem = args[0]
recs, order = [], []
for arg in args[1:]:
    head, path = arg.split("=", 1)
    parts = head.split(":")
    label, c = parts[0], parts[1]
    sname = parts[2] if len(parts) > 2 else "traveler"
    got = records(label, c if c.startswith("pl") else float(c), sname, path)
    if not got:
        print(f"{label}: no satellite tracked into ignition at file 600 s -- skipped")
        continue
    recs += got
    order.append(label)
Path(out_stem + ".json").write_text(json.dumps(recs, indent=1))

import matplotlib                                                   # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                    # noqa: E402

INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
ONSET, LATER, HELD = "#c4561a", "#1f6feb", "#59636e"
CATEG = ["#1f6feb", "#c4561a", "#1a7f37", "#8250df", "#bf8700", "#59636e"]
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": GRID})


def run_cn0(lab):
    rs = [r for r in recs if r["run"] == lab]
    return float(np.median([r["cn0"] for r in rs])) if rs else float("nan")


# ---- 1: Doppler rate at loss vs expected C/N0 (x axis broken between 50 and 73 dB-Hz) ----
if not compare:
    fig, (ax, ax2) = plt.subplots(1, 2, sharey=True, figsize=(9.0, 5.4),
                                  gridspec_kw={"width_ratios": [5, 1], "wspace": 0.05})
    for lab in order:
        rs = sorted((r for r in recs if r["run"] == lab), key=lambda r: r["el"])
        spread = rs and rs[0]["spread"]
        cn0 = run_cn0(lab)
        a_ = ax2 if cn0 > 60 else ax
        for j, r in enumerate(rs):
            x = r["cn0"] if spread else cn0 + (j - (len(rs) - 1) / 2) * 0.16
            mk = "D" if spread else "o"
            if not r["lost"]:
                a_.scatter(x, r["rate"], marker="^", s=34, facecolors="none", edgecolors=HELD, lw=1.0, zorder=3)
            else:
                a_.scatter(x, r["rate"], marker=mk, s=48 if spread else 24,
                           color=ONSET if r["t"] < 1.0 else LATER,
                           edgecolors=INK if spread else "white", lw=0.9 if spread else 0.6, zorder=5 if spread else 4)
        if not spread:
            a_.annotate(lab, (cn0, 1.0), xycoords=("data", "axes fraction"), ha="center", va="bottom",
                        fontsize=8.5, color=INK)
    ax.set_xlim(33, 50)
    ax2.set_xlim(73, 77)
    ax.set_ylim(0, 1000)
    ax.set_xticks([35, 38, 41, 44, 47, 50])
    ax2.set_xticks([75])
    for a_ in (ax, ax2):
        a_.grid(axis="y", color=GRID, lw=0.6)
        a_.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax2.spines["left"].set_visible(False)
    ax2.spines["right"].set_visible(False)
    ax2.tick_params(axis="y", left=False)
    for a_, xs in ((ax, (1, 1)), (ax2, (0, 0))):          # break marks on the x axis
        a_.plot(xs, (0, 0), transform=a_.transAxes, marker=[(-1, -1.6), (1, 1.6)], ms=9, mew=1,
                ls="", color=MUTED, clip_on=False)
    ax.set_xlabel("expected C/N0, dB-Hz (bench calibration; axis broken 50 to 73)")
    ax.set_ylabel("line-of-sight Doppler rate, Hz/s   (51.5 Hz/s = 1 g along the line of sight)")
    h = [plt.Line2D([], [], marker="o", ls="", color=ONSET,
                    label="lost in the first second: the ignition step (its rate is an upper bound)"),
         plt.Line2D([], [], marker="o", ls="", color=LATER, label="lost later, as its rate climbed (T+1 to 13 s)"),
         plt.Line2D([], [], marker="^", ls="", markerfacecolor="none", markeredgecolor=HELD,
                    label="held to burnout: tracked at least this rate")]
    if any(r["spread"] for r in recs):
        h.append(plt.Line2D([], [], marker="D", ls="", color=MUTED,
                            label="spread-power run: each satellite at its own level"))
    ax.legend(handles=h, frameon=False, loc="upper left", fontsize=8.5)
    fig.suptitle("PX1105R through the traveler burn: the Doppler rate each satellite was under when it dropped",
                 x=0.08, ha="left", fontsize=10.5)
    fig.text(0.08, 0.915, "first loss of a valid pseudorange lasting >= 0.5 s; equal-power runs side by side in "
             "elevation order (lowest left); * = the 13 s collapse cycle runs in that capture",
             fontsize=8.5, color=MUTED)
    fig.savefig(out_stem + "_scatter.png", dpi=140, bbox_inches="tight", facecolor="white")

# ---- 2: survival vs Doppler rate, per run (Kaplan-Meier on the rate axis) ----
fig, ax = plt.subplots(figsize=(8.6, 5.0))
ramp = plt.get_cmap("Blues")
levels = sorted(order, key=run_cn0)
for lab in order:
    rs = [r for r in recs if r["run"] == lab]
    at_risk, s, xs, ys = len(rs), 1.0, [0.0], [1.0]
    for r in sorted(rs, key=lambda r: (r["rate"], not r["lost"])):
        if r["lost"]:
            s *= (at_risk - 1) / at_risk
            xs += [r["rate"], r["rate"]]
            ys += [ys[-1], s]
        at_risk -= 1
    xs.append(max(r["rate"] for r in rs))
    ys.append(s)
    if compare:
        col = CATEG[order.index(lab) % len(CATEG)]
    else:
        col = ramp(0.35 + 0.65 * levels.index(lab) / max(1, len(levels) - 1))
    half = next((x for x, y in zip(xs, ys) if y <= 0.5), None)
    note = f"half lost by {half:.0f} Hz/s" if half is not None else "more than half held to burnout"
    name = lab if compare else f"{lab} dB-Hz"
    ax.plot(xs, ys, color=col, lw=1.8, ls="--" if rs[0]["spread"] else "-", label=f"{name}: {note}")
ax.axhline(0.5, color=MUTED, lw=0.8, ls="--")
handles, labels_ = ax.get_legend_handles_labels()
if not compare:
    idx = sorted(range(len(labels_)), key=lambda i: -run_cn0(order[i]))
    handles, labels_ = [handles[i] for i in idx], [labels_[i] for i in idx]
ax.legend(handles, labels_, frameon=False, loc="upper right", fontsize=8.5)
ax.set_xlim(0, 1050)
ax.set_ylim(-0.02, 1.02)
ax.set_xlabel("line-of-sight Doppler rate reached, Hz/s")
ax.set_ylabel("share of satellites still tracked")
ax.grid(color=GRID, lw=0.6)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
ax.set_title("Share of satellites still tracked vs the Doppler rate they have been pushed to\n"
             "(ignition to burnout + 0.5 s; a satellite that never dropped leaves the count at its burnout rate)",
             loc="left", fontsize=9.5)
fig.savefig(out_stem + "_survival.png", dpi=140, bbox_inches="tight", facecolor="white")

# ---- 3: each satellite's Doppler rate through the burn, per run ----
panels = order if compare else sorted(order, key=run_cn0, reverse=True)
ncol = 3 if len(panels) > 4 else len(panels)
nrow = math.ceil(len(panels) / ncol)
fig, axs = plt.subplots(nrow, ncol, sharex=True, sharey=True,
                        figsize=(max(3.85 * ncol, 10.5), 3.3 * nrow + 0.6), squeeze=False)
GROUPC = "#8c959f"
# axes fitted to what was observed: to the last raw epoch when every run stops early (a u-blox
# withholding above 515 m/s), else to the end of the window past burnout
X_END = max((DATA_END[lab] if DATA_END.get(lab) is not None else BURN_T[lab] + JOLT) for lab in panels) + 0.3
for a_, lab in zip(axs.flat, panels):
    rs = [r for r in recs if r["run"] == lab]
    lost = [(r["t"], r["rate"]) for r in rs if r["lost"]]

    def shared(r):
        """Dropped with >= 2 others within 0.25 s at rates spread > 1.8x: a common event, not a
        common threshold (satellites crossing one threshold together drop at similar rates)."""
        near = [v for t, v in lost if abs(t - r["t"]) <= 0.25]
        return len(near) >= 3 and max(near) > 1.8 * min(near)

    for r in rs:
        ser = [s_ for s_ in r["series"] if s_[0] < STEP_T[lab]]           # stop before the burnout step
        ts = [t for t, _ok, _v in ser]
        vs = [v for _t, _ok, v in ser]
        a_.plot(ts, vs, color=GRID, lw=0.9, zorder=1, rasterized=svg_out)  # the rate it was put under
        a_.plot([t if ok else float("nan") for t, ok, _v in ser], vs, color=INK, lw=1.1, zorder=2,
                rasterized=svg_out)
        if r["lost"]:
            if shared(r):
                a_.scatter(r["t"], r["rate"], marker="o", s=30, facecolors="none", edgecolors=GROUPC,
                           lw=1.3, zorder=4)
            else:
                a_.scatter(r["t"], r["rate"], marker="x", s=30, lw=1.4,
                           color=ONSET if r["t"] < 1.0 else LATER, zorder=4)
        else:
            a_.scatter(r["t"], r["rate"], marker="^", s=26, facecolors="none", edgecolors=HELD, lw=1.0, zorder=4)
        if r["spread"]:
            a_.annotate(f"{r['cn0']:.0f}", (ts[-1], vs[-1]), xytext=(3, 0), textcoords="offset points",
                        fontsize=7, color=MUTED, va="center")
    fe = FIX_END.get(lab)
    if fe is not None and fe < X_END - 0.2:
        a_.axvline(fe, color=MUTED, lw=0.9, ls="--", zorder=0)
        edge = fe > X_END - 0.25 * (X_END + 0.3)             # too near the right edge: label on the left
        a_.annotate(f"fix stops T+{fe:.1f}", (fe, 0.98), xycoords=("data", "axes fraction"),
                    xytext=(-3 if edge else 3, 0), ha="right" if edge else "left",
                    textcoords="offset points", fontsize=8, color=MUTED, va="top")
    elif fe is not None:                         # its own speed lagged: the fix ran past the window
        a_.annotate(f"fix stops T+{fe:.1f} →", (0.99, 0.98), xycoords="axes fraction", ha="right",
                    fontsize=8, color=MUTED, va="top")
    a_.axvline(BURN_T[lab], color=MUTED, lw=0.7, ls=":", zorder=0)
    a_.annotate("burnout", (BURN_T[lab], 0.02), xycoords=("data", "axes fraction"), xytext=(3, 0),
                textcoords="offset points", fontsize=8, color=MUTED, va="bottom")
    de = DATA_END.get(lab)
    if de is not None:
        a_.axvspan(de, de + 100.0, color="#eaeef2", alpha=0.6, lw=0, zorder=0)
        a_.annotate(f"no raw after T+{de:.1f}\n(withheld >515 m/s)", (de, 0.9), xycoords=("data", "axes fraction"),
                    xytext=(4, 0), textcoords="offset points", fontsize=8, color=MUTED, va="top")
    a_.set_title(lab if compare else f"{lab} dB-Hz", loc="left", fontsize=9.5)
    a_.grid(axis="y", color=GRID, lw=0.5)
    for side in ("top", "right"):
        a_.spines[side].set_visible(False)
for a_ in list(axs.flat)[len(panels):]:
    a_.set_visible(False)
for a_ in axs[-1]:
    a_.set_xlabel("time from ignition, s")
for a_ in axs[:, 0]:
    a_.set_ylabel("line-of-sight Doppler rate, Hz/s")
y_top = max(v for r in recs if r["run"] in panels for t, _ok, v in r["series"] if t <= X_END)
axs[0][0].set_ylim(0, max(950.0, 1.06 * y_top))
axs[0][0].set_xlim(-0.3, X_END)
h = [plt.Line2D([], [], color=INK, lw=1.1, label="tracked (valid pseudorange)"),
     plt.Line2D([], [], color=GRID, lw=2, label="not tracked"),
     plt.Line2D([], [], marker="x", ls="", color=ONSET, label="own loss (>= 0.5 s), first second"),
     plt.Line2D([], [], marker="x", ls="", color=LATER, label="own loss, later"),
     plt.Line2D([], [], marker="o", ls="", markerfacecolor="none", markeredgecolor=GROUPC,
                label="shared drop: >= 3 within 0.25 s at rates > 1.8x apart"),
     plt.Line2D([], [], marker="^", ls="", markerfacecolor="none", markeredgecolor=HELD,
                label="held to burnout, or to the last raw epoch")]
fig.legend(handles=h, loc="lower center", ncol=6 if ncol >= 3 else 3, frameon=False, fontsize=8.5,
           bbox_to_anchor=(0.5, -0.04 if nrow == 1 else -0.02))
fig.subplots_adjust(hspace=0.3, wspace=0.08, bottom=0.2 if nrow == 1 else 0.12)
if svg_out:
    plt.rcParams["svg.fonttype"] = "none"
    fig.savefig(out_stem + "_traces.svg", dpi=110, bbox_inches="tight", facecolor="white")
fig.suptitle("Each satellite's line-of-sight Doppler rate through the burn (steepest = highest elevation); "
             "dark where the receiver tracks it", x=0.06, ha="left", fontsize=10.5)
fig.savefig(out_stem + "_traces.png", dpi=140, bbox_inches="tight", facecolor="white")

for lab in order:
    rs = [r for r in recs if r["run"] == lab]
    lost = [r for r in rs if r["lost"]]
    print(f"{lab:>12}: {len(rs)} tracked at ignition, {len(lost)} dropped by burnout + 0.5 s "
          f"({sum(r['t'] < 1.0 for r in lost)} in the first second); fix stops "
          + (f"T+{FIX_END[lab]:.2f}" if FIX_END.get(lab) is not None else "-")
          + "; losses (t, rate): " + ", ".join(f"{r['t']:.2f}/{r['rate']:.0f}" for r in sorted(lost, key=lambda r: r['t'])))
