#!/usr/bin/env python3
"""A u-blox receiver's boost chart (the NEO-M8T; RX=ZED-F9P for that one), in the style of boost_traces_wide.py
(the PX1105R's): each satellite's
line-of-sight Doppler rate through the burn (vertical acceleration x sin(elevation) / wavelength; B1I's own for BeiDou),
in its system's colour while the M8T reports a valid pseudorange (RXM-RAWX prValid, 10 Hz), grey where not. The M8T
withholds ALL raw measurements above 515 m/s, so the chart is shaded from its last raw epoch to burnout. Elevations
from the capture's own NAV-SAT. A loss = a gap in valid pseudoranges of >= 0.5 s that starts at least 0.3 s before the
raw cut-off (later gaps cannot be told from the cut-off). Then a comparison figure: share of the satellites tracked at
ignition still tracked at 515 m/s, PX1105R (its channel-status lock, from BOOST_JSON) vs M8T, per system and level.

With m8t_accuracy.py's NPZ per run (LABEL=CAPTURE=ACC.npz), each valid epoch also carries its pseudorange error, and an
epoch more than WRONG_M (10 m) off is WRONG: flagged valid by the receiver but not right. Those stretches get a magenta
band under the trace (a diamond where it first goes wrong), the panel titles count them, and the comparison adds a dashed
M8T line for the satellites still flagged valid AND within WRONG_M at the cut-off.

A Septentrio mosaic-G5 capture (mosaic_run.py, SBF 't S <hex>' lines; RX=mosaic-G5 RX_LIMIT=600) is read the same
way: MeasEpoch thinned to 10 Hz, the files' signals only (GPS L1 C/A, Galileo E1, BeiDou B1I), elevations from
ChannelStatus or SatVisibility where the receiver has them, the fix from PVTGeodetic. After a cold start on the rig
the mosaic knows the elevation only of the satellites in its almanac (GPS and a few Galileo); the rest come from the
accuracy NPZ, which has every satellite's elevation from the broadcast ephemeris and the true position. It withholds above 600 m/s, not 515, so its cut-off and shading follow
RX_LIMIT while the comparison with the PX1105R stays just after 515 m/s, where it is still reporting.

    m8t_boost_traces.py OUT_STEM PX_BOOST_JSON LABEL=CAPTURE[=ACC.npz] [...]  -> OUT_STEM.png/.json, OUT_STEM_compare.png
    (BOOST_SCEN=traveler_soft25 for the traveler runs; default hotshot)
"""
import json
import math
import os
import statistics as st
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
IGN_TOW, SUSTAIN, EDGE = 204000.0, 0.5, 0.3
WRONG_M = 10.0                          # a valid-flagged pseudorange this far off the truth is wrong
C = 299792458.0
LAM = {"G": C / 1575.42e6, "E": C / 1575.42e6, "C": C / 1561.098e6}
SYS = {0: "G", 2: "E", 3: "C"}
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
AB = {"G": "GPS", "E": "GAL", "C": "BDS"}
SCEN = os.environ.get("BOOST_SCEN", "hotshot")
RX = os.environ.get("RX", "NEO-M8T")                        # the receiver the captures came from (RX=ZED-F9P)
RX_SHORT = "mosaic" if RX.startswith("mosaic") else RX.split("-")[-1]   # "M8T", "F9P", "mosaic"
LIMIT = float(os.environ.get("RX_LIMIT", "515"))            # the speed above which the receiver withholds raw
BURN_NAME = {"hotshot": "the hotshot burn (4 s, 10 to 40 g)", "traveler_soft25": "the traveler burn (13 s)"}
sc = json.loads((SDR / "scenarios" / f"{SCEN}.json").read_text())
PRO = sc["prologue_s"]
TR = [(s["t"] - PRO, s["v_up_mps"]) for s in sc["truth"]]
BURN = next(s["t"] for s in sc["truth"] if s["phase"] == "coast") - PRO
OVER = sc["velocity_windows"][0][0] - PRO                     # 515 m/s on the way up (T+2.8)
UNDER = sc["velocity_windows"][0][1] - PRO
if LIMIT == 515:
    OVER_CUT, UNDER_CUT = OVER, UNDER
else:                                                         # the receiver's own limit, crossed in the truth
    _sp = [(s["t"] - PRO, s["speed_mps"]) for s in sc["truth"]]
    OVER_CUT = next(t for t, v in _sp if t >= 0 and v > LIMIT)
    UNDER_CUT = next(t for t, v in _sp if t > OVER_CUT and v <= LIMIT)


def v_up(t):
    lo, hi = 1, len(TR) - 1
    while lo < hi:                                            # first sample after t
        mid = (lo + hi) // 2
        if TR[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    (t0, v0), (t1, v1) = TR[lo - 1], TR[lo]
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def a_up(t):
    """Vertical acceleration as a centred 0.2 s difference: the receiver's epochs fall at either edge of the 10 Hz
    truth samples, and a one-segment slope there jitters by +-10 Hz/s in the chart."""
    return (v_up(t + 0.1) - v_up(t - 0.1)) / 0.2


def parse_sbf(cap):
    """A mosaic-G5 capture -> the same (raw, el, fixes) as parse(): raw at 10 Hz, one signal per satellite."""
    sys.path.insert(0, str(SDR.parent))
    import septentrio_sbf as sbf
    sig = {"GPS L1CA": "G", "GAL E1": "E", "BDS B1I": "C"}
    raw, el, el_ch, fixes = [], {}, {}, []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 24:
            continue
        h = p[2].strip()
        bid = int(h[10:12] + h[8:10], 16) & 0x1FFF
        if bid not in (sbf.MEAS_EPOCH, sbf.SAT_VISIBILITY, sbf.CHANNEL_STATUS, sbf.PVT_GEODETIC):
            continue
        tow = int.from_bytes(bytes.fromhex(h[16:24]), "little")
        if tow == 0xFFFFFFFF:
            continue
        t = round(tow / 1000.0 - IGN_TOW, 2)
        try:
            b = bytes.fromhex(h)
        except ValueError:
            continue
        if bid == sbf.MEAS_EPOCH:
            if tow % 100:
                continue
            obs = {}
            for x in sbf.meas_epoch(b)["meas"]:
                if x["signal"] in sig and x["sv"][1:].isdigit():
                    obs[(sig[x["signal"]], int(x["sv"][1:]))] = (x["pr"] is not None, x["cn0"] or 0.0)
            raw.append((t, obs))
        elif bid == sbf.CHANNEL_STATUS:
            if -6.0 <= t <= 0.5:
                for s in sbf.channel_status(b):
                    if s["sv"][0] in "GEC" and s["sv"][1:].isdigit() and s["el"] is not None and s["el"] > 0:
                        el_ch[(s["sv"][0], int(s["sv"][1:]))] = s["el"]
        elif bid == sbf.SAT_VISIBILITY:
            if -6.0 <= t <= 0.5:
                for name, (e, a) in sbf.sat_visibility(b).items():
                    if name[0] in "GEC" and name[1:].isdigit() and e > 0:
                        el[(name[0], int(name[1:]))] = int(round(e))
        elif sbf.pvt_geodetic(b)["mode"]:
            fixes.append(t)
    el.update(el_ch)                                # ChannelStatus wins where both have one
    return raw, el, fixes


def parse(cap):
    with open(cap, errors="replace") as fh:
        for line in fh:
            p = line.split(" ", 2)
            if len(p) == 3 and p[1] in ("S", "U"):
                if p[1] == "S":
                    return parse_sbf(cap)
                break
    raw, el, fixes = [], {}, []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "U":
            continue
        h = p[2].strip()
        try:
            b = bytes.fromhex(h[4:])
        except ValueError:
            continue
        if h.startswith("0215") and len(b) >= 16:
            t = round(struct.unpack("<d", b[0:8])[0] - IGN_TOW, 2)
            obs = {}
            for j in range(b[11]):
                m = b[16 + 32 * j: 48 + 32 * j]
                if len(m) == 32 and m[20] in SYS:
                    k, v = (SYS[m[20]], m[21]), (bool(m[30] & 1), m[26])
                    if k not in obs or (v[0] and not obs[k][0]):     # one signal per satellite (F9P: several)
                        obs[k] = v
            raw.append((t, obs))
        elif h.startswith("0135") and len(b) >= 8:
            t = struct.unpack("<I", b[0:4])[0] / 1000.0 - IGN_TOW
            if -6.0 <= t <= 0.5:
                for j in range(b[5]):
                    s = b[8 + 12 * j: 20 + 12 * j]
                    if len(s) == 12 and s[0] in SYS and s[2] > 0:
                        el[(SYS[s[0]], s[1])] = struct.unpack("b", s[3:4])[0]
        elif h.startswith("0107") and len(b) >= 64:
            t = struct.unpack("<I", b[0:4])[0] / 1000.0 - IGN_TOW
            if b[20] >= 2 and b[21] & 1:
                fixes.append(t)
    return raw, el, fixes


def errors(acc):
    """m8t_accuracy NPZ -> {(sys, prn): (times, errors)}; times from ignition, rows sorted by time."""
    if not acc:
        return {}
    import numpy as np
    z = np.load(acc)
    out = {}
    for i, c in enumerate("GEC"):
        for p in set(int(x) for x in z["prn"][z["sys"] == i]):
            m = (z["sys"] == i) & (z["prn"] == p)
            o = np.argsort(z["t"][m])
            out[(c, p)] = (z["t"][m][o], z["pr"][m][o])
    return out


def err_at(e, t):
    """The error logged within 0.03 s of t, or None."""
    if e is None or not len(e[0]):
        return None
    import numpy as np
    i = int(np.clip(np.searchsorted(e[0], t), 1, len(e[0]) - 1))
    j = i - 1 if abs(e[0][i - 1] - t) <= abs(e[0][i] - t) else i
    v = float(e[1][j])
    return v if abs(e[0][j] - t) <= 0.03 and math.isfinite(v) else None   # NaN: a rig clock step, not an error


def el_from_acc(acc, el):
    """Fill missing ignition elevations from the accuracy NPZ (ephemeris and truth, T-6 to T+0.5)."""
    if not acc:
        return el
    import numpy as np
    z = np.load(acc)
    m = (z["t"] >= -6.0) & (z["t"] <= 0.5)
    for i, c in enumerate("GEC"):
        for p in set(int(x) for x in z["prn"][m & (z["sys"] == i)]):
            if (c, p) not in el:
                el[(c, p)] = int(round(float(np.median(z["el"][m & (z["sys"] == i) & (z["prn"] == p)]))))
    return el


def run(label, cap, acc=None):
    raw, el, fixes = parse(cap)
    el = el_from_acc(acc, el)
    errs = errors(acc)
    pad = {c: [v[1] for t, o in raw if -60 <= t <= -5 for k, v in o.items() if k[0] == c and v[0]] for c in "GEC"}
    pad_cn0 = {c: st.median(v) for c, v in pad.items() if v}
    during = [(t, o) for t, o in raw if -1.0 <= t <= UNDER_CUT - 1.0]
    last_raw = max((t for t, o in during if 0 <= t <= OVER_CUT + 1.0 and any(v[0] for v in o.values())),
                   default=0.0)                             # last raw epoch before the first crossing of the limit
    # raw that stops at the limit is the receiver withholding it; raw that stops well before is the receiver losing
    # every satellite, and those are losses (judged up to the crossing)
    cut = last_raw if last_raw >= OVER_CUT - 0.4 else OVER_CUT
    recs = []
    for key in sorted({k for t, o in during for k in o}):
        pre = [o.get(key, (False, 0))[0] for t, o in during if -0.5 <= t < 0.0]
        if not pre or not all(pre) or key not in el or el[key] <= 0:
            continue
        s = math.sin(math.radians(el[key]))
        post = [(t, o.get(key, (False, 0))[0]) for t, o in during if 0.0 <= t <= cut]
        loss = None
        for i, (t, ok) in enumerate(post):
            if ok or t > cut - EDGE:
                continue
            nxt = next((tt for tt, ok_ in post[i:] if ok_), cut + 99.0)
            if nxt - t >= SUSTAIN:
                loss = t
                break
        rate = lambda t: a_up(t + 0.001) * s / LAM[key[0]]       # noqa: E731
        e = errs.get(key)
        series = [(t, ok, rate(t), err_at(e, t) if ok else None) for t, ok in post]
        bad = [(t, v, x) for t, ok, v, x in series if ok and x is not None and abs(x) > WRONG_M]
        ev = [x for t, ok, v, x in series if ok and x is not None]             # errors of the valid epochs
        recs.append(dict(sys=key[0], prn=key[1], el=el[key], lost=loss is not None,
                         t=loss if loss is not None else cut, rate=rate(loss if loss is not None else cut),
                         series=series, wrong=bool(bad), wrong_t=bad[0][0] if bad else None,
                         wrong_rate=bad[0][1] if bad else None, err_max=max((abs(x) for x in ev), default=None),
                         wrong_end=bool(ev) and abs(ev[-1]) > WRONG_M))
    last_fix = max((t for t in fixes if 0.0 <= t <= OVER_CUT + 1.0), default=None)
    return dict(label=label, recs=recs, cut=cut, fix_end=last_fix, pad_cn0=pad_cn0, has_err=bool(errs))


args = sys.argv[1:]
out, px_json = args[0], args[1]
runs = [run(*a.split("=", 2)) for a in args[2:]]
Path(out + ".json").write_text(json.dumps(runs, indent=1))

import matplotlib                                                       # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                        # noqa: E402

INK, MUTED, GRID, SHADE = "#1f2328", "#59636e", "#d8dee4", "#f3f4f6"
COL = {"G": "#1f6feb", "E": "#c4561a", "C": "#1a7f37"}
WRONG = "#bf3989"                       # magenta: apart from GPS blue, Galileo orange and BeiDou green
plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": GRID})
n = len(runs)
ncol = 2 if n > 1 else 1
nrow = math.ceil(n / ncol)
any_err = any(r["has_err"] for r in runs)
fig, axs = plt.subplots(nrow, ncol, sharex=True, sharey=True, figsize=(6.2 * ncol, (4.25 if any_err else 3.9) * nrow + 0.7),
                        squeeze=False)
X_END = BURN + 0.8
ymax = 0.0
for ax, r in zip(axs.flat, runs):
    rs = r["recs"]
    cut = r["cut"]
    ax.axvspan(cut, BURN, color=SHADE, zorder=0)
    ax.annotate(f"raw withheld\nabove {LIMIT:.0f} m/s", ((cut + BURN) / 2, 0.97), xycoords=("data", "axes fraction"),
                ha="center", va="top", fontsize=7.8, color=MUTED)
    for q in sorted(rs, key=lambda q: q["el"]):
        s = math.sin(math.radians(q["el"]))
        full = [k / 50.0 for k in range(0, int((BURN - 0.06) * 50) + 1)]           # stop before the thrust step
        vfull = [a_up(t + 0.001) * s / LAM[q["sys"]] for t in full]
        ymax = max(ymax, max(vfull))
        ax.plot(full, vfull, color=GRID, lw=0.9, zorder=1)
        ser = q["series"]
        ax.plot([t if ok else float("nan") for t, ok, v, *_ in ser], [v for t, ok, v, *_ in ser], color=COL[q["sys"]],
                lw=1.3, zorder=3)
        if q.get("wrong"):                          # flagged valid but over WRONG_M off: a band under the trace
            ax.plot([t if ok and x is not None and abs(x) > WRONG_M else float("nan") for t, ok, v, x in ser],
                    [v for t, ok, v, x in ser], color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt", zorder=2)
            ax.scatter(q["wrong_t"], q["wrong_rate"], marker="D", s=22, color=WRONG, zorder=6)
        if q["lost"]:
            ax.scatter(q["t"], q["rate"], marker="x", s=34, lw=1.6, color=COL[q["sys"]], zorder=5)
        else:
            ax.scatter(cut, q["rate"], marker="^", s=28, facecolors="none", edgecolors=COL[q["sys"]], lw=1.1,
                       zorder=5)
    fe = r["fix_end"]
    if fe is not None:
        ax.axvline(fe, color=MUTED, lw=0.9, ls="--", zorder=0)
        ax.annotate(f"fix stops T+{fe:.1f}", (fe, 0.80), xycoords=("data", "axes fraction"), xytext=(-3, 0),
                    textcoords="offset points", fontsize=8, color=MUTED, va="top", ha="right")
    ax.axvline(BURN, color=MUTED, lw=0.7, ls=":", zorder=0)
    ax.annotate("burnout", (BURN, 0.02), xycoords=("data", "axes fraction"), xytext=(-3, 0), ha="right",
                textcoords="offset points", fontsize=8, color=MUTED, va="bottom")
    held, bad = [], []
    for c in "GEC":
        tot = [q for q in rs if q["sys"] == c]
        if tot:
            held.append(f"{AB[c]} {sum(not q['lost'] for q in tot)}/{len(tot)}")
            nb = sum(bool(q.get("wrong")) for q in tot)
            if nb:
                bad.append(f"{AB[c]} {nb}")
    cn = ", ".join(f"{AB[c]} {r['pad_cn0'][c]:.0f}" for c in "GEC" if c in r["pad_cn0"])
    wrong_line = (f"\nover {WRONG_M:.0f} m off while flagged valid: {', '.join(bad) if bad else 'none'}"
                  if r["has_err"] else "")
    ax.set_title(f"{r['label']}\npad C/N0 {cn} dB-Hz\nstill flagged valid at the raw cut-off (T+{cut:.1f}): "
                 f"{', '.join(held)}{wrong_line}", loc="left", fontsize=8.4)
    ax.grid(axis="y", color=GRID, lw=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
for ax in list(axs.flat)[n:]:
    ax.set_visible(False)
for ax in axs[-1]:
    ax.set_xlabel("time from ignition, s")
for ax in axs[:, 0]:
    ax.set_ylabel("line-of-sight Doppler rate, Hz/s")
axs[0][0].set_ylim(0, 1.06 * ymax)
axs[0][0].set_xlim(-0.3, X_END)
h = [plt.Line2D([], [], color=COL["G"], lw=1.4, label="GPS: valid pseudorange"),
     plt.Line2D([], [], color=COL["C"], lw=1.4, label="BeiDou: valid pseudorange"),
     plt.Line2D([], [], color=COL["E"], lw=1.4, label="Galileo: valid pseudorange"),
     plt.Line2D([], [], color=GRID, lw=2, label="not reported / unobserved"),
     plt.Line2D([], [], marker="x", ls="", color=INK, label="lost (gap >= 0.5 s before the cut-off)"),
     plt.Line2D([], [], marker="^", ls="", markerfacecolor="none", markeredgecolor=INK,
                label="still flagged valid at the cut-off")]
if any_err:
    h += [plt.Line2D([], [], color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt",
                     label=f"flagged valid but over {WRONG_M:.0f} m off the truth"),
          plt.Line2D([], [], marker="D", ls="", color=WRONG, label=f"where it first goes over {WRONG_M:.0f} m")]
one_row = nrow == 1
fig.legend(handles=h, loc="lower center", ncol=4 if any_err else 3, frameon=False, fontsize=8.5,
           bbox_to_anchor=(0.5, -0.13 if one_row else -0.04))
fig.text(0.06, -0.22 if one_row else -0.085, f"The {RX} sits behind 10 dB more attenuation than the PX1105R on the "
         f"same files (wide SignalSim {SCEN.split('_')[0]}, carrier corrected, 180 s pad)."
         + (" Pseudorange errors from m8t_accuracy.py (truth, solved clock)." if any_err else ""),
         fontsize=8.5, color=MUTED, ha="left")
fig.subplots_adjust(hspace=0.78 if any_err else 0.62, wspace=0.12, bottom=0.2 if one_row else 0.12,
                    top=0.78 if one_row else 0.85)
fig.suptitle(f"{RX} through {BURN_NAME.get(SCEN, 'the burn')}, by signal level: each satellite's line-of-sight "
             "Doppler rate", x=0.06, y=1.0, ha="left", fontsize=10.5)
fig.savefig(out + ".png", dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out + ".png")

# ---- comparison: share still tracked at 515 m/s, PX1105R (lock) vs M8T (valid raw)
px = {r["label"].split(" (")[0]: r for r in json.loads(Path(px_json).read_text())}   # "+12 dB (57 dB-Hz file)" -> "+12 dB"
levels = [r["label"] for r in runs]
TCMP = math.floor(OVER) + (1.0 if OVER % 1.0 < 0.85 else 2.0)   # after the PX's first 1 Hz status past 515 m/s
PX_STATUS = TCMP - 0.1                                            # (statuses land at about x.9 s)
RX_COL = {"NEO-M8T": "#bf8700", "ZED-F9P": "#0f7b6c", "mosaic-G5": "#a40e26"}.get(RX, "#bf8700")
REC = {"PX1105R": ("#6639ba", "o"), RX: (RX_COL, "s")}
if any_err:
    REC[f"{RX} within 10 m"] = (RX_COL, "s")
def ok_at(q, t_cmp):
    """The last valid epoch at or before t_cmp is within WRONG_M (or has no truth to judge it by)."""
    ev = [x for t, ok, v, x in q["series"] if ok and t <= t_cmp]
    return not ev or ev[-1] is None or abs(ev[-1]) <= WRONG_M


fig, axs = plt.subplots(1, 3, sharey=True, figsize=(11.5, 3.6))
table = []
for ax, c in zip(axs, "GEC"):
    solid = {}
    for rec, (col, mk) in REC.items():
        xs, ys, labs = [], [], []
        for i, lv in enumerate(levels):
            if rec == "PX1105R":                    # its 1 Hz channel-status lock just after 515 m/s (the T+2.9
                r = px.get(lv)                      # status; re-locks count), sampled at T+3.0
                if r is None:
                    continue
                tot = [q for q in r["recs"] if q["sys"] == c]
                kept = [q for q in tot if [s for s in q["series"] if s[0] <= TCMP] and
                        [s for s in q["series"] if s[0] <= TCMP][-1][4]]
            elif rec == RX:                  # no sustained loss up to its raw cut-off (T+2.9-3.0), as in
                r = runs[i]                         # its own chart (a drop in the very last epoch is not a loss)
                tot = [q for q in r["recs"] if q["sys"] == c]
                kept = [q for q in tot if not q["lost"] or (LIMIT != 515 and q["t"] > TCMP)]
            else:                                   # still flagged valid AND within WRONG_M at its last epoch
                r = runs[i]
                tot = [q for q in r["recs"] if q["sys"] == c]
                if LIMIT == 515:
                    kept = [q for q in tot if not q["lost"] and not q.get("wrong_end")]
                else:                               # still reporting at 515 m/s: judged at TCMP, like the PX1105R
                    kept = [q for q in tot if (not q["lost"] or q["t"] > TCMP) and ok_at(q, TCMP)]
            if not tot:
                continue
            xs.append(i)
            ys.append(100.0 * len(kept) / len(tot))
            labs.append(f"{len(kept)}/{len(tot)}")
            table.append((rec, lv, NAME[c], len(kept), len(tot)))
        dashed = rec == f"{RX} within 10 m"
        ax.plot(xs, ys, color=col, lw=1.6, marker=mk, ms=6, label=rec, zorder=3 if not dashed else 4,
                ls="--" if dashed else "-", markerfacecolor="white" if dashed else col)
        for x, y, lab in zip(xs, ys, labs):
            if rec == RX:
                solid[x] = lab
            if dashed and solid.get(x) == lab:      # same as the solid line: one label is enough
                continue
            ax.annotate(lab, (x, y), xytext=(0, 7 if rec == RX else -13), textcoords="offset points",
                        ha="center", fontsize=8, color=col, style="italic" if dashed else "normal")
    ax.set_title(NAME[c], loc="left", fontsize=10)
    ax.set_xticks(range(len(levels)), levels)
    ax.set_ylim(-12, 115)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.grid(axis="y", color=GRID, lw=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.set_xlabel("signal level")
axs[0].set_ylabel("still tracked just after 515 m/s, %")
axs[2].legend(frameon=False, fontsize=8.5, loc="upper left", bbox_to_anchor=(1.03, 1.0))   # clear of the labels
fig.suptitle(f"{SCEN.split('_')[0].capitalize()} burn: share of the satellites tracked at ignition still tracked just "
             f"after 515 m/s (T+{PX_STATUS:.1f}), PX1105R vs {RX}", x=0.06, ha="left", fontsize=10.5)
cuts = [r["cut"] for r in runs]
if LIMIT == 515:
    rx_how = (f"{RX}: no gap >= 0.5 s in valid raw pseudoranges up to where it withholds raw "
              f"(T+{min(cuts):.1f} to T+{max(cuts):.1f})."
              + (f" Dashed: of those, the ones still within {WRONG_M:.0f} m of the truth there." if any_err else "")
              + f"\nNeither receiver can be compared past this point: the {RX_SHORT} reports nothing above 515 m/s.")
else:
    rx_how = (f"{RX}: no gap >= 0.5 s in valid raw pseudoranges up to T+{TCMP:.1f}; it keeps reporting to "
              f"{LIMIT:.0f} m/s (T+{min(cuts):.1f} to T+{max(cuts):.1f})."
              + (f"\nDashed: of those, the ones within {WRONG_M:.0f} m of the truth at T+{TCMP:.1f}." if any_err else "\n"))
fig.text(0.06, -0.06, f"515 m/s is reached at T+{OVER:.1f}. PX1105R: lock in its 1 Hz channel status at T+{PX_STATUS:.1f} "
         f"(dropped-and-relocked counts as tracked). " + rx_how + f" The {RX_SHORT} has 10 dB more attenuation.",
         fontsize=8.3, color=MUTED, ha="left")
fig.subplots_adjust(top=0.82, bottom=0.2, wspace=0.2)
fig.savefig(out + "_compare.png", dpi=140, bbox_inches="tight", facecolor="white")
print("wrote", out + "_compare.png")
for row in table:
    print("  %-20s %-7s %-8s %d/%d" % row)
