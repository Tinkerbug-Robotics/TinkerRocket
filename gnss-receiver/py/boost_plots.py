#!/usr/bin/env python3
"""Figures of gnssrx runs through a boost, drawn like the rig's reports on the bought receivers.

  rates     each satellite's line-of-sight Doppler rate from ignition to past burnout, on the rig's
            key for the bought receivers: coloured by constellation where a pseudorange was
            delivered, magenta where it was over 10 m off the truth, markers on carrier lock. One
            panel per run, in a grid (rows: signal levels, columns: loop configurations).
  timeline  one run in full: speed and altitude (truth and fix), vertical acceleration (and what
            the IMU aiding was given), per-satellite output, measurements per epoch, pseudorange
            and range-rate errors per satellite, and the fix's errors; over the whole run and
            zoomed on the boost.

    boost_plots.py rates --traj SCEN.csv --nav BRDC.rnx --liftoff 600.0 --burnout 613.0 \\
        --rows 45,38,35,33,31 --cols Q,B,QA,BA --run 'runs/m7c/{col}_trav_{row}' -o rates.png
    boost_plots.py timeline RUN --traj SCEN.csv --nav BRDC.rnx --liftoff 600.0 --burnout 613.0 -o run.png

Times are file seconds; the run's t_s plus its start (run.ini). Errors are judged as
boost_track.py judges them: each satellite's own pre-launch level taken out, then the per-epoch
median over satellites (the receiver clock).
"""
from __future__ import annotations

import argparse
import math
import sys
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from boost_track import enu_basis, read_csv, single_diff  # noqa: E402
from gnssrx import iqio, rinex, truth  # noqa: E402

# The rig's palette and trace style (its C/N0 sweep report, cn0_sweep/boost_traces_wide.py).
SYS_COLOR = {"G": "#1f6feb", "E": "#c4561a", "C": "#1a7f37"}
SYS_NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
GRAY = "#d8dee4"
INK = "#1f2328"
WRONG_M, WRONG = 10.0, "#bf3989"  # a delivered pseudorange this far off the truth is wrong (the rig's magenta)
SYS_ABBR = {"G": "GPS", "E": "GAL", "C": "BDS"}
COCOM_V, COCOM_H = 515.0, 80e3
LAM = truth.LAMBDA_L1

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": "#e4e7ea",
    "grid.linewidth": 0.6, "axes.edgecolor": "#9aa3ab", "axes.titlelocation": "left",
})


def sys_of(prn: int) -> str:
    return "C" if prn >= 200 else ("E" if prn >= 100 else "G")


def sat_name(prn: int) -> str:
    return f"{sys_of(prn)}{prn % 100:02d}"


def key(t) -> np.ndarray:
    """0.1 s epoch index of file time(s) t."""
    return np.round(np.asarray(t) * 10.0).astype(np.int64)


class Run:
    """A gnssrx output directory, its times in file seconds."""

    def __init__(self, path: Path):
        self.path = path
        self.ini = {}
        spans = []  # (on, off) for each span the boost profile was on; the gated profile logs two
        for line in (path / "run.ini").read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                self.ini[k.strip()] = v.strip()
                if k.strip() == "boost_detected":
                    on, _, off = (float(x) for x in v.split(","))
                    spans.append((on, off))
        self.start = float(self.ini["start_s"])
        self.trk = read_csv(path / "trk.csv")
        self.obs = read_csv(path / "obs.csv")
        self.pvt = read_csv(path / "pvt.csv")
        self.imu = read_csv(path / "imu.csv") if (path / "imu.csv").exists() else None
        self.tf_trk = self.trk["t_s"] + self.start
        self.tf_obs = self.obs["t_s"] + self.start
        self.tf_pvt = self.pvt["t_s"] + self.start
        # Satellites that delivered something at some point.
        self.prns = sorted({int(p) for p in self.obs["prn"]})
        self.t0 = float(np.round(self.obs["rx_tow"][0] - self.obs["t_s"][0] - self.start, 3))
        b = self.ini.get("boost_at")
        d = self.ini.get("boost_detected")  # on, burnout, off: what the emulated IMU detected
        self.boost = (tuple(float(x) for x in b.split(",")) if b else
                      tuple(float(x) for x in d.split(","))[0::2] if d else None)
        self.boost_spans = [self.boost] if b else spans

    def status(self, prn: int, keys: np.ndarray) -> np.ndarray:
        """Per epoch: 2 carrier-locked observables, 1 code and Doppler only, 0 nothing."""
        m = self.obs["prn"] == prn
        ko = key(self.tf_obs[m])
        lock = self.obs["lock_s"][m] > 0
        out = np.zeros(keys.size, dtype=int)
        pos = {int(k): (2 if l else 1) for k, l in zip(ko, lock)}
        for j, k in enumerate(keys):
            out[j] = pos.get(int(k), 0)
        return out

    def locked(self, prn: int, keys: np.ndarray) -> np.ndarray:
        """Per epoch: the channel's PLL in lock (trk.csv state 2), whether or not it delivered."""
        m = self.trk["prn"] == prn
        on = {int(k) for k, st in zip(key(self.tf_trk[m]), self.trk["state"][m]) if st == 2}
        return np.array([int(k) in on for k in keys], dtype=bool)

    def pad_cn0(self, prn: int, t0: float, t1: float) -> float:
        m = (self.trk["prn"] == prn) & (self.tf_trk >= t0) & (self.tf_trk < t1) & (self.trk["state"] == 2)
        return float(np.median(self.trk["cn0"][m])) if m.any() else float("nan")


class Truth:
    """Line-of-sight truth per satellite on the 0.1 s grid, computed once per flight."""

    def __init__(self, traj: Path, nav: Path):
        self.traj = truth.Trajectory(traj)
        self.nav = rinex.read_nav(nav, "GEC")
        self.cache: dict[tuple[int, float], dict[int, np.ndarray]] = {}

    def los(self, prn: int, keys: np.ndarray, t0: float) -> np.ndarray:
        """(range, Doppler, Doppler rate, elevation) at the epochs `keys` (file s * 10)."""
        c = self.cache.setdefault((prn, t0), {})
        need = [int(k) for k in keys if int(k) not in c]
        if need:
            vals = truth.los(self.nav, prn, self.traj, np.array(need) / 10.0, t0)
            for k, v in zip(need, vals):
                c[k] = v
        return np.array([c[int(k)] for k in keys]) if len(keys) else np.zeros((0, 4))


def gapped(t: np.ndarray, y: np.ndarray, gap: float = 0.15):
    """t, y with a NaN wherever the samples are more than `gap` apart, so a line breaks there."""
    if t.size < 2:
        return t, y
    cut = np.where(np.diff(t) > gap)[0] + 1
    return np.insert(t.astype(float), cut, np.nan), np.insert(y.astype(float), cut, np.nan)


def segments(x: np.ndarray, cat: np.ndarray):
    """(category, start, stop) for each run of equal category; stop is exclusive."""
    out, s = [], 0
    for j in range(1, cat.size + 1):
        if j == cat.size or cat[j] != cat[s]:
            out.append((int(cat[s]), s, j))
            s = j
    return out


# ------------------------------------------------------------------------------------------- rates

def file_tropo(run: Run) -> float | None:
    """Where the troposphere the run's file carries stops (m; the manifest's tropo_top_m), None for none."""
    try:
        m = iqio.manifest(Path(run.ini.get("source", "")).name)
    except KeyError:
        return None
    return None if m.tropo == "none" else m.tropo_top_m


def pr_truth(run: Run, tr: Truth, prn: int, k: np.ndarray, tropo: float | None) -> np.ndarray:
    """What a perfect receiver's pseudorange would read at epoch keys k, less its clock: the geometric range,
    the troposphere where the file carries one (up to its ceiling, tropo), less the satellite's clock (which
    drifts by metres over a flight). What stays is constant per satellite (the ionosphere, the generator's
    offsets)."""
    L = tr.los(int(prn), k, run.t0)
    ref = L[:, 0] - truth.C * truth.sat_clock(tr.nav, int(prn), run.t0 + k / 10.0)
    if tropo is not None:
        lat, h = tr.traj.lat_h(k / 10.0)
        ref = ref + truth.tropo_saastamoinen(lat, h, np.radians(L[:, 3]), tropo)
    return ref


def tropo_rate(run: Run, tr: Truth, prn: int, k: np.ndarray, tropo: float | None) -> np.ndarray:
    """The troposphere's part of the range rate (m/s) at epoch keys k: its delay thins as the rocket climbs.
    Below a ceiling only; the step there is the pseudorange's, not a rate."""
    if tropo is None:
        return np.zeros(k.size)
    t = k / 10.0
    lat, h = tr.traj.lat_h(t)
    el = np.radians(tr.los(int(prn), k, run.t0)[:, 3])
    up = 0.5 * (np.interp(t + 0.05, tr.traj.t, tr.traj.h) - np.interp(t - 0.05, tr.traj.t, tr.traj.h)) / 0.05
    g = 0.5 * (truth.tropo_saastamoinen(lat, np.minimum(h + 1.0, tropo), el, tropo) -
               truth.tropo_saastamoinen(lat, np.minimum(h - 1.0, tropo), el, tropo))
    return np.where(h < tropo, g * up, 0.0)


def judged(run: Run, tr: Truth, liftoff: float, t_end: float) -> dict[int, dict[int, float]]:
    """Each delivered pseudorange's error (m), per satellite and epoch key, from 30 s before liftoff to t_end:
    the smoothed pseudorange less the geometric range (and the troposphere where the file carries one), each
    satellite's own pre-launch level taken out, then the per-epoch median over satellites (the receiver
    clock). A satellite with no pre-launch level is not judged."""
    tropo = file_tropo(run)
    o, tf = run.obs, run.tf_obs
    w = (tf >= liftoff - 30.0) & (tf <= t_end + 0.05)
    prn, k, x = o["prn"][w].astype(int), key(tf[w]), o["pr_m"][w].astype(float)
    for p in np.unique(prn):
        m = prn == p
        x[m] -= pr_truth(run, tr, p, k[m], tropo)
        pre = m & (k < key(liftoff - 1.0))
        x[m] -= np.median(x[pre]) if pre.any() else np.nan
    out: dict[int, dict[int, float]] = {}
    for ke in np.unique(k):
        m = (k == ke) & np.isfinite(x)
        if m.sum() >= 4:
            med = np.median(x[m])
            for p, v in zip(prn[m], x[m]):
                out.setdefault(int(p), {})[int(ke)] = float(v - med)
    return out


def panel_rates(ax, run: Run, tr: Truth, liftoff: float, burnout: float, t_end: float, label: str,
                count_after: float = 1.0):
    """One run on the rig's key: solid where a pseudorange was delivered, a pale tint where the channel
    held lock but delivered nothing, grey where it delivered nothing; magenta where what it delivered
    was over WRONG_M off the truth; markers on carrier lock."""
    keys = np.arange(key(liftoff - 0.5), key(t_end) + 1)
    x = keys / 10.0 - liftoff
    jb = min(int(np.searchsorted(keys, key(burnout + count_after))), keys.size - 1)
    after = keys >= key(liftoff)
    err = judged(run, tr, liftoff, t_end)
    n, locked_b, raw_b, bad = {}, {}, {}, {}
    lost = []
    for prn in run.prns:
        s = sys_of(prn)
        rate = tr.los(prn, keys, run.t0)[:, 2]
        st = run.status(prn, keys)  # 2 carrier-locked, 1 code and Doppler only, 0 nothing
        delivered = st >= 1
        withheld = run.locked(prn, keys) & ~delivered
        e = err.get(prn, {})
        wrong = delivered & (np.abs(np.array([e.get(int(k), np.nan) for k in keys])) > WRONG_M)
        col = SYS_COLOR[s]
        ax.plot(x, rate, color=GRAY, lw=0.9, zorder=1)
        ax.plot(x, np.where(withheld, rate, np.nan), color=col, lw=2.6, alpha=0.28, solid_capstyle="butt", zorder=2)
        ax.plot(x, np.where(delivered, rate, np.nan), color=col, lw=1.3, zorder=3)
        if wrong.any():  # a magenta band under the trace
            ax.plot(x, np.where(wrong, rate, np.nan), color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt",
                    zorder=2)
            jw = np.flatnonzero(wrong & after)
            if jw.size:
                ax.plot(x[jw[0]], rate[jw[0]], ls="", marker="D", ms=4.7, color=WRONG, zorder=6)
                bad[s] = bad.get(s, 0) + 1
        # Markers on carrier lock, from ignition to the end of the panel.
        locked = st[after] == 2
        xa, ra = x[after], rate[after]
        drops = np.where(locked[:-1] & ~locked[1:])[0]
        if locked.all():
            ax.plot(xa[-1], ra[-1], marker="^", ms=5.3, mfc="none", mec=col, mew=1.1, zorder=5)
        elif locked[-1]:
            j = drops[0] if drops.size else 0
            ax.plot(xa[j + 1], ra[j + 1], marker="o", ms=5.5, mfc="none", mec=col, mew=1.4, zorder=5)
        else:
            j = drops[-1] + 1 if drops.size else 0
            ax.plot(xa[j], ra[j], marker="x", ms=5.8, color=col, mew=1.6, zorder=5)
            lost.append(sat_name(prn))
        n[s] = n.get(s, 0) + 1
        locked_b[s] = locked_b.get(s, 0) + int(st[jb] == 2)
        raw_b[s] = raw_b.get(s, 0) + int(st[jb] >= 1)
    ax.axvline(burnout - liftoff, color="#555", ls=":", lw=0.9)
    ax.axvline(0.0, color="#555", ls=":", lw=0.6)
    yl = ax.get_ylim()
    ax.text(burnout - liftoff - 0.08, yl[0] + 0.03 * (yl[1] - yl[0]), "burnout", ha="right", va="bottom",
            fontsize=8, color="#555")
    top = file_tropo(run)
    if top is not None and top < 4e4:
        # Where the file's troposphere stops (SignalSim's 10 km): every satellite's code steps there.
        th = keys / 10.0
        up = np.flatnonzero(np.diff((np.interp(th, tr.traj.t, tr.traj.h) > top).astype(int)) != 0)
        for j in up:
            ax.axvline(th[j + 1] - liftoff, color="#1a7f8a", ls="-.", lw=0.9)
            ax.text(th[j + 1] - liftoff + 0.08, yl[0] + 0.03 * (yl[1] - yl[0]), "10 km", ha="left", va="bottom",
                    fontsize=8, color="#1a7f8a")
    sys_list = [s for s in "GEC" if s in n]
    pad = {}
    for s in sys_list:
        v = [c for c in (run.pad_cn0(p, liftoff - 6, liftoff - 1) for p in run.prns if sys_of(p) == s) if np.isfinite(c)]
        pad[s] = f"{np.median(v):.0f}" if v else "-"  # "-": none of them locked on the pad
    pad_txt = ", ".join(f"{SYS_ABBR[s]} {pad[s]}" for s in sys_list)
    locked_txt = ", ".join(f"{SYS_ABBR[s]} {locked_b[s]}/{n[s]}" for s in sys_list)
    raw_txt = ", ".join(f"{SYS_ABBR[s]} {raw_b[s]}/{n[s]}" for s in sys_list)
    bad_txt = ", ".join(f"{SYS_ABBR[s]} {bad[s]}" for s in sys_list if s in bad) or "none"
    unl = 0.0
    for prn in run.prns:
        m = (run.trk["prn"] == prn) & (run.tf_trk >= liftoff) & (run.tf_trk <= t_end)
        unl += float(np.sum(run.trk["state"][m] != 2)) / 10.0
    gap = np.diff(np.sort(run.tf_pvt[(run.tf_pvt >= liftoff - 1) & (run.tf_pvt <= t_end)]))
    fix_txt = f"fix every epoch" if gap.size and gap.max() < 0.15 else (
        f"longest fix gap {gap.max():.1f} s" if gap.size else "no fix")
    ax.set_title(f"{label}\npad C/N0 {pad_txt} dB-Hz (median, measured)\n"
                 f"at burnout+{count_after:g} s: locked {locked_txt};  raw {raw_txt}\n"
                 f"over {WRONG_M:.0f} m off while delivered: {bad_txt}\n"
                 f"PLL unlocked {unl:.1f} sat-s after ignition; {len(lost)} not back by T+{t_end - liftoff:.1f}; "
                 f"{fix_txt}", fontsize=8.4)
    ax.set_xlim(x[0], x[-1] + 0.03 * (x[-1] - x[0]))


def cmd_rates(a) -> int:
    tr = Truth(a.traj, a.nav)
    rows, cols = a.rows.split(","), a.cols.split(",")
    rlab = dict(zip(rows, a.row_labels.split("|"))) if a.row_labels else {r: r for r in rows}
    clab = dict(zip(cols, a.col_labels.split("|"))) if a.col_labels else {c: c for c in cols}
    t_end = a.burnout + a.after
    # The rig report's panel size, so its line widths read the same.
    fig, axs = plt.subplots(len(rows), len(cols), figsize=(6.2 * len(cols), 4.4 * len(rows) + 1.9),
                            sharex=True, sharey=True, squeeze=False)
    systems = set()
    for i, r in enumerate(rows):
        for j, c in enumerate(cols):
            p = Path(a.run.format(row=r, col=c))
            ax = axs[i][j]
            if not (p / "trk.csv").exists():
                ax.set_visible(False)
                continue
            run = Run(p)
            systems |= {sys_of(q) for q in run.prns}
            label = clab[c] if len(rows) == 1 else f"{clab[c]}  ·  {rlab[r]}"
            panel_rates(ax, run, tr, a.liftoff, a.burnout, t_end, textwrap.fill(label, 78), a.count_after)
            if j == 0:
                ax.set_ylabel("line-of-sight Doppler rate, Hz/s")
            if i == len(rows) - 1:
                ax.set_xlabel("time from ignition, s")
    fig.suptitle(a.title + (f"\n{rlab[rows[0]]}" if len(rows) == 1 else ""), x=0.01, ha="left", fontsize=10.5)
    # The rig's key (cn0_sweep/boost_traces_wide.py), its lock read as our PLL's.
    names = {"G": "GPS: raw pseudorange delivered", "C": "BeiDou: delivered", "E": "Galileo: delivered"}
    handles = [Line2D([], [], color=SYS_COLOR[s], lw=1.4, label=names[s]) for s in "GCE" if s in systems]
    handles += [Line2D([], [], color=INK, lw=2.6, alpha=0.28, solid_capstyle="butt",
                       label="locked (channel status), measurement withheld"),
                Line2D([], [], color=GRAY, lw=2, label="lock lost, nothing delivered"),
                Line2D([], [], ls="", marker="x", color=INK, label="carrier lock lost here, for good"),
                Line2D([], [], ls="", marker="o", mfc="none", mec=INK,
                       label="carrier lock lost here, back by the panel's end"),
                Line2D([], [], ls="", marker="^", mfc="none", mec=INK, label="never lost carrier lock"),
                Line2D([], [], color=WRONG, lw=6.5, alpha=0.30, solid_capstyle="butt",
                       label=f"delivered but over {WRONG_M:.0f} m off the truth"),
                Line2D([], [], ls="", marker="D", color=WRONG, label=f"where it first goes over {WRONG_M:.0f} m")]
    H = fig.get_figheight()
    fig.legend(handles=handles, loc="lower left", ncol=4, frameon=False, fontsize=8.5, bbox_to_anchor=(0.01, 0.0))
    note = (a.note + " " if a.note else "") + (
        "Pseudorange errors against the truth (the geometric range, the troposphere where the file carries one, and "
        "the satellite's clock), each satellite's pre-launch level and the per-epoch median over satellites (the "
        "receiver clock) taken out.")
    note = textwrap.fill(note, 58 * len(cols))
    fig.text(0.01, 0.78 / H, note, fontsize=8.5, color="#555", va="bottom")
    fig.tight_layout(rect=(0, (0.95 + 0.145 * (note.count("\n") + 1)) / H, 1, 1 - 0.6 / H))
    fig.savefig(a.o, dpi=a.dpi)
    print(f"wrote {a.o}")
    return 0


# ---------------------------------------------------------------------------------------- timeline

def errors(run: Run, tr: Truth, liftoff: float):
    """Per satellite: (t, code error m, raw code error m, range-rate error m/s), as boost_track judges them."""
    o, tf = run.obs, run.tf_obs
    rng = np.full(tf.size, np.nan)
    dop = np.full(tf.size, np.nan)
    tropo = file_tropo(run)
    for p in np.unique(o["prn"]).astype(int):
        m = o["prn"] == p
        rng[m] = pr_truth(run, tr, p, key(tf[m]), tropo)
        dop[m] = tr.los(p, key(tf[m]), run.t0)[:, 1] - tropo_rate(run, tr, p, key(tf[m]), tropo) / LAM
    out = {}
    for name, x in (("code", o["pr_m"] - rng), ("raw", o["pr_raw_m"] - rng), ("rate", -(o["dop_hz"] - dop) * LAM)):
        x = x.copy()
        for p in np.unique(o["prn"]):
            m = o["prn"] == p
            ref = m & (tf > liftoff - 30) & (tf < liftoff - 1)
            x[m] -= np.median(x[ref]) if ref.any() else x[m][0]
        out[name] = single_diff(tf, o["prn"], x, np.isfinite(x))
    return out


def cmd_timeline(a) -> int:
    run = Run(a.run)
    tr = Truth(a.traj, a.nav)
    L0, B = a.liftoff, a.burnout
    t_all = np.arange(key(run.tf_trk.min()), key(run.tf_trk.max()) + 1) / 10.0
    tp, tv = tr.traj.state(t_all)
    speed = np.linalg.norm(tv, axis=1)
    h_true = np.interp(t_all, tr.traj.t, tr.traj.h)
    acc_up = tr.traj.up_accel(t_all)
    err = errors(run, tr, L0)
    prns = run.prns
    has = {s: any(sys_of(p) == s for p in prns) for s in "GEC"}

    rows = ["speed", "alt", "acc", "sats", "count", "codeG"] + (["codeE"] if has["E"] else []) + ["rate", "pos", "vel"]
    hr = {"speed": 1, "alt": 1, "acc": 1, "sats": 0.13 * len(prns) + 0.4, "count": 1, "codeG": 1.1, "codeE": 1.1,
          "rate": 1.1, "pos": 1.1, "vel": 1.1}
    fig, axs = plt.subplots(len(rows), 2, figsize=(15, 2.0 * sum(hr[r] for r in rows) + 0.8),
                            gridspec_kw={"height_ratios": [hr[r] for r in rows], "width_ratios": [1.5, 1]},
                            sharex="col")
    zoom = tuple(float(v) for v in a.zoom.split(",")) if a.zoom else (-2.0, B - L0 + a.after)
    full = (t_all[0] - L0, t_all[-1] - L0)
    xs = t_all - L0
    cocom = (speed > COCOM_V) | (h_true > COCOM_H)
    # Where the flight crosses the height the file's troposphere stops at (SignalSim's 10 km): every
    # satellite's code steps there, and the carrier slips with it.
    top = file_tropo(run)
    top_cross = []
    if top is not None and top < 4e4:
        above = h_true > top
        top_cross = list(t_all[1:][np.diff(above.astype(int)) != 0])
    R = enu_basis(float(tr.traj.lat[0]), float(tr.traj.lon[0]))
    pos = np.stack([run.pvt["x"], run.pvt["y"], run.pvt["z"]], axis=-1)
    vel = np.stack([run.pvt["vx"], run.pvt["vy"], run.pvt["vz"]], axis=-1)
    ptp, ptv = tr.traj.state(run.tf_pvt)
    dpos = (pos - ptp) @ R.T
    dvel = (vel - ptv) @ R.T
    xp = run.tf_pvt - L0
    # Velocities the fix's own residual test rejected (vel_valid = 0) are not drawn as velocities.
    vbad = (run.pvt["vel_valid"] == 0) if "vel_valid" in run.pvt else np.zeros(xp.size, dtype=bool)
    dvel[vbad] = np.nan

    for col, xl in ((0, full), (1, zoom)):
        A = {r: axs[i][col] for i, r in enumerate(rows)}
        for ax in A.values():
            # Amber where the bought receivers' export limits apply; the boost profile's window; ignition/burnout.
            on = False
            for j in range(xs.size):
                if cocom[j] and not on:
                    j0, on = j, True
                if on and (not cocom[j] or j == xs.size - 1):
                    ax.axvspan(xs[j0], xs[j], color="#f7efd9", lw=0, zorder=0)
                    on = False
            ax.axvline(0.0, color="#555", ls=":", lw=0.8)
            ax.axvline(B - L0, color="#555", ls=":", lw=0.8)
            for tc in top_cross:
                ax.axvline(tc - L0, color="#1a7f8a", ls="-.", lw=0.8)
        ax = A["speed"]
        ax.plot(xs, speed, color="#222", lw=1.2, label="true speed")
        ax.plot(xp[~vbad], np.linalg.norm(vel[~vbad], axis=1), ".", ms=1.6, color=SYS_COLOR["G"],
                label="receiver's own speed (fix)")
        ax.axhline(COCOM_V, color="#b08a18", ls="--", lw=0.8)
        ax.set_ylabel("speed, m/s")
        ax = A["alt"]
        ax.plot(xs, h_true / 1e3, color="#222", lw=1.2, label="true altitude")
        ax.plot(xp, run.pvt["h_m"] / 1e3, ".", ms=1.6, color=SYS_COLOR["G"], label="receiver's own altitude (fix)")
        ax.set_ylabel("altitude, km")
        ax = A["acc"]
        ax.plot(xs, acc_up, color="#7a4fd0", lw=1.0, label="true vertical acceleration")
        if run.imu is not None:
            ti = run.imu["t_s"] + run.start - L0
            on = run.imu["aiding"] > 0
            ax.plot(ti[on], run.imu["acc_up_imu"][on], ".", ms=1.5, color="#e08a00", label="IMU aiding input (emulated)")
        for j, (on, off) in enumerate(run.boost_spans):
            off = off if np.isfinite(off) else t_all[-1]
            ax.axvspan(on - L0, off - L0, color="#e8f1fb", lw=0, zorder=0, label="boost loop profile" if j == 0 else None)
        ax.set_ylabel("vertical accel., m/s²")
        # Per-satellite output, on the rig's key: solid where a pseudorange was delivered, magenta where it
        # was over WRONG_M off the truth.
        ax = A["sats"]
        ks = key(np.arange(run.tf_trk.min(), run.tf_trk.max() + 0.05, 0.1))
        xk = ks / 10.0 - L0
        for i, p in enumerate(prns):
            st = run.status(p, ks)
            cat = (st >= 1).astype(int)
            if p in err["code"]:
                tt, x = err["code"][p]
                bad = set(key(tt[np.abs(x) > WRONG_M]).tolist())
                cat[np.array([int(k) in bad for k in ks]) & (cat == 1)] = 2
            for c, s0, s1 in segments(xk, cat):
                if c == 0:
                    continue
                ax.fill_between([xk[s0] - 0.05, xk[s1 - 1] + 0.05], i - 0.38, i + 0.38, lw=0,
                                color=SYS_COLOR[sys_of(p)] if c == 1 else WRONG)
        ax.set_yticks(range(len(prns)))
        ax.set_yticklabels([sat_name(p) for p in prns], fontsize=6)
        ax.set_ylim(len(prns) - 0.5, -0.5)
        ax.grid(False)
        if col == 0:
            ax.set_ylabel("raw measurement output,\nper satellite")
            ax.text(1.0, 1.01, f"magenta: delivered but over {WRONG_M:.0f} m off the truth", transform=ax.transAxes,
                    ha="right", va="bottom", fontsize=7, color=WRONG)
        # Measurements per epoch.
        ax = A["count"]
        ko = key(run.tf_obs)
        ks = np.unique(ko)
        for s in "GEC":
            if not has[s]:
                continue
            ms = np.array([sys_of(int(p)) == s for p in run.obs["prn"]])
            car = np.array([np.sum((ko == k) & ms & (run.obs["lock_s"] > 0)) for k in ks])
            allm = np.array([np.sum((ko == k) & ms) for k in ks])
            ax.step(ks / 10.0 - L0, car, where="mid", color=SYS_COLOR[s], lw=1.0)
            ax.step(ks / 10.0 - L0, allm, where="mid", color=SYS_COLOR[s], lw=2.2, alpha=0.3)
        kf = key(run.tf_pvt)
        ax.scatter(kf / 10.0 - L0, np.full(kf.size, -0.6), marker="|", s=12, color="#2e9b4b", lw=0.8)
        ax.set_ylim(-1.2, None)
        ax.set_ylabel("measurements\nper epoch")
        if col == 0:
            ax.text(0.005, 0.93, "thin: carrier-locked; pale: all observables; green ticks: a fix", fontsize=6.5,
                    color="#555", transform=ax.transAxes, va="top")
        # Code and range-rate errors.
        for r, sysw in (("codeG", "G"), ("codeE", "E")):
            if r not in A:
                continue
            ax = A[r]
            for p, (tt, x) in err["raw"].items():
                if sys_of(p) == sysw:
                    ax.plot(tt - L0, x, ".", ms=0.8, color=SYS_COLOR[sysw], alpha=0.25)
            for p, (tt, x) in err["code"].items():
                if sys_of(p) == sysw:
                    ax.plot(*gapped(tt - L0, x), "-", lw=0.7, color=SYS_COLOR[sysw])
            ax.set_ylabel(f"{SYS_NAME[sysw]} pseudorange\nerror, m")
            ax.set_yscale("symlog", linthresh=a.code_ylim)
            ax.set_ylim(-1000, 1000)
            for v in (-WRONG_M, WRONG_M):
                ax.axhline(v, color=WRONG, ls="--", lw=0.7)
        ax = A["rate"]
        for p, (tt, x) in err["rate"].items():
            ax.plot(tt - L0, x, ".", ms=0.9, color=SYS_COLOR[sys_of(p)])
        ax.set_yscale("symlog", linthresh=a.rate_ylim)
        ax.set_ylim(-100, 100)
        ax.set_ylabel("range-rate error\n(Doppler), m/s")
        ax = A["pos"]
        for k, c, n in ((0, "#7aa6d8", "E"), (1, "#9bc59d", "N"), (2, "#222", "U")):
            ax.plot(*gapped(xp, dpos[:, k]), lw=0.8, color=c, label=n)
        ax.set_yscale("symlog", linthresh=a.pos_ylim)
        ax.set_ylim(-1000, 1000)
        ax.set_ylabel("fix position\nerror, m")
        ax = A["vel"]
        for k, c, n in ((0, "#7aa6d8", "E"), (1, "#9bc59d", "N"), (2, "#222", "U")):
            ax.plot(*gapped(xp, dvel[:, k]), lw=0.8, color=c, label=n)
        if vbad.any():
            ax.plot(xp[vbad], np.full(vbad.sum(), -50.0), "x", ms=3, color="#c03030",
                    label="velocity failed the fix's residual test")
        ax.set_yscale("symlog", linthresh=a.vel_ylim)
        ax.set_ylim(-100, 100)
        ax.set_ylabel("fix velocity\nerror, m/s")
        axs[-1][col].set_xlim(*xl)
        if col == 0:
            for r in ("speed", "alt", "acc", "pos", "vel"):
                A[r].legend(loc="upper left", fontsize=6.5, frameon=False, ncol=3)
    axs[-1][0].set_xlabel("time from ignition, s  (dotted: ignition and burnout; amber: over 515 m/s or 80 km, where a "
                          "bought receiver stops;\ngreen ticks: a fix; errors linear near zero, logarithmic beyond"
                          + ("; dash-dot: 10 km, where the file's troposphere stops)" if top_cross else ")"))
    axs[-1][1].set_xlabel(f"the boost: T{zoom[0]:+.0f} to T+{zoom[1]:.0f} s")
    title = textwrap.fill(a.title, 190)
    fig.suptitle(title, x=0.01, ha="left", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 1.0 - (0.15 + 0.17 * (title.count("\n") + 1)) / fig.get_figheight()))
    fig.savefig(a.o, dpi=a.dpi)
    print(f"wrote {a.o}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("rates", "timeline"):
        p = sub.add_parser(name)
        p.add_argument("--traj", type=Path, required=True)
        p.add_argument("--nav", type=Path, required=True)
        p.add_argument("--liftoff", type=float, required=True, help="file s")
        p.add_argument("--burnout", type=float, required=True, help="file s")
        p.add_argument("--after", type=float, default=2.5, help="s past burnout to show")
        p.add_argument("--count-after", type=float, default=1.0,
                       help="rates: s past burnout the panel titles count locked satellites at")
        p.add_argument("--title", default="")
        p.add_argument("--dpi", type=int, default=130)
        p.add_argument("-o", type=Path, required=True)
        if name == "rates":
            p.add_argument("--rows", required=True)
            p.add_argument("--cols", required=True)
            p.add_argument("--row-labels")
            p.add_argument("--col-labels")
            p.add_argument("--run", required=True, help="run directory pattern with {row} and {col}")
            p.add_argument("--note", default="")
        else:
            p.add_argument("run", type=Path)
            # The error panels are linear within these and logarithmic beyond.
            p.add_argument("--code-ylim", type=float, default=5.0)
            p.add_argument("--rate-ylim", type=float, default=0.5)
            p.add_argument("--pos-ylim", type=float, default=2.0)
            p.add_argument("--vel-ylim", type=float, default=0.5)
            p.add_argument("--zoom", help="the right-hand column's window, S0,S1 in seconds from ignition")
    a = ap.parse_args()
    return cmd_rates(a) if a.cmd == "rates" else cmd_timeline(a)


if __name__ == "__main__":
    raise SystemExit(main())
