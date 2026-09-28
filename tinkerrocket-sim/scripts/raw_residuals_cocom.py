#!/usr/bin/env python3
"""A COCOM-rig capture's raw measurements against the truth, and how late each kind is.

For every epoch the truth (tc_ekf_cocom.RigTruth, which follows gps-sdr-sim's timing)
is taken at the epoch minus a trial lag; satellites come from the capture's own
ephemeris and the corrections from gnss_raw.corrected (no troposphere: the rig has
none). Each epoch's common part -- the receiver clock bias, or its drift -- is removed
with the median across satellites. The lag that minimises a measurement kind's RMS,
flight phase by phase, is how late that kind is:

  pr   pseudorange
  rr   range rate from the Doppler
  cr   carrier-phase change over 1 s

The PX1105R's Doppler is ~0.22 s late and its carrier ~0.20 s, its pseudorange on
time (power normal, 2026-09-27); in power save the Doppler's lag is 0.05-0.2 s and
moves with the dynamics. The script also reports the clock-rate offset between the
Doppler and the pseudoranges on the pad (4.2 m/s on the HackRF rig).

    PYTHONPATH=src python3 scripts/raw_residuals_cocom.py CAPTURE.log.gz SCENARIO.json \\
        [--pad 600] [--stride 4] [--plot fig.svg --plot-lag 0.22]
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from tc_ekf_cocom import RigTruth, load_capture, shade_limits, TOW_FILE0, EPOCH_DT  # noqa: E402
from tinkerrocket_sim.estimation.gnss_raw import corrected  # noqa: E402
from tinkerrocket_sim.estimation.tc_ekf import los_predict  # noqa: E402

KINDS = ("pr", "rr", "cr")


def residuals(truth, data, lags, stride, t_lo, t_hi):
    """{lag: {kind: [(t, prn, residual)]}}, each epoch's median removed."""
    eph, ep, own, kind = data
    dstep = 1.0 if kind == "skytraq" else 0.0
    out = {lag: {k: [] for k in KINDS} for lag in lags}
    keys = sorted(ep)
    for k in keys[::stride]:
        tow, obs = ep[k]
        t = tow - TOW_FILE0
        if not t_lo <= t <= t_hi:
            continue
        m_now = corrected(tow, obs, eph, truth.pos_ecef(t), use_tropo=False, el_mask_deg=-90.0,
                          doppler_step_hz=dstep)
        k1 = k - int(round(1.0 / EPOCH_DT))
        m_then = {}
        if k1 in ep:
            tow1, obs1 = ep[k1]
            m_then = {m.prn: m for m in corrected(tow1, obs1, eph, truth.pos_ecef(t - 1.0), use_tropo=False,
                                                  el_mask_deg=-90.0, doppler_step_hz=dstep)}
        if len(m_now) < 5:
            continue
        for lag in lags:
            r, v = truth.pos_ecef(t - lag), truth.T.T @ truth.vel_ned(t - lag)
            r1 = truth.pos_ecef(t - 1.0 - lag)
            rows = {kk: [] for kk in KINDS}
            for m in m_now:
                rho, rate, _ = los_predict(m, r, v)
                rows["pr"].append((m.prn, m.pr - rho))
                if m.rr is not None:
                    rows["rr"].append((m.prn, m.rr - rate))
                b = m_then.get(m.prn)
                if b is not None and m.cr is not None and b.cr is not None and not m.cr_slip:
                    rows["cr"].append((m.prn, (m.cr - b.cr) - (rho - los_predict(b, r1, np.zeros(3))[0])))
            for kk, rr in rows.items():
                if len(rr) >= 5:
                    med = float(np.median([x[1] for x in rr]))
                    out[lag][kk].extend((t, p, x - med) for p, x in rr)
    return out


def best_lags(truth, res, lags):
    """Per phase and kind: (best lag, RMS there, RMS at lag 0)."""
    table = []
    for name, a, b in truth.phases()[1:]:
        row = [name]
        for kk in KINDS:
            rms = []
            for lag in lags:
                x = np.array([r for t, p, r in res[lag][kk] if a <= t < b])
                rms.append(math.sqrt(float(np.mean(x ** 2))) if len(x) > 20 else math.nan)
            rms = np.array(rms)
            if np.isfinite(rms).any():
                i = int(np.nanargmin(rms))
                z = rms[int(np.argmin(np.abs(np.array(lags))))]
                row.append((lags[i], rms[i], z))
            else:
                row.append(None)
        table.append(row)
    return table


def clock_rate_offset(truth, data):
    """Doppler clock drift minus the pseudoranges' clock-bias rate on the pad, m/s."""
    eph, ep, own, kind = data
    dstep = 1.0 if kind == "skytraq" else 0.0
    tb, bias, drift = [], [], []
    for k in sorted(ep)[::20]:
        tow, obs = ep[k]
        t = tow - TOW_FILE0
        if not truth.t_ign - 300.0 <= t < truth.t_ign - 5.0:
            continue
        m = corrected(tow, obs, eph, truth.pos_ecef(t), use_tropo=False, el_mask_deg=-90.0, doppler_step_hz=dstep)
        if len(m) < 5:
            continue
        r = truth.pos_ecef(t)
        tb.append(t)
        bias.append(float(np.median([x.pr - los_predict(x, r, np.zeros(3))[0] for x in m])))
        drift.append(float(np.median([x.rr - los_predict(x, r, np.zeros(3))[1] for x in m if x.rr is not None])))
    if len(tb) < 10:
        return math.nan
    b = np.array(bias)
    b = b - np.round((b - b[0]) / (299792458.0e-3)) * 299792458.0e-3      # undo whole-millisecond clock steps
    return float(np.mean(drift) - np.polyfit(tb, b, 1)[0])


def plot(path, truth, data, res0, res_lag, lag, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, ink2, muted, grid, base = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
    blue, orange, aqua = "#2a78d6", "#eb6834", "#1baf7a"
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 8, "axes.edgecolor": base,
                         "axes.labelcolor": ink2, "xtick.color": muted, "ytick.color": muted,
                         "axes.spines.top": False, "axes.spines.right": False, "svg.fonttype": "none",
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})
    t0 = truth.t_ign
    fig, ax = plt.subplots(3, 1, figsize=(9.5, 7.2), sharex=True,
                           gridspec_kw=dict(hspace=0.28, height_ratios=[0.55, 1, 1.2]))
    ax = [ax[1], ax[2], ax[0]]                    # draw order below: Doppler, pseudorange, satellites (on top)
    a0 = np.array([(t, r) for t, p, r in res0["rr"]])
    a1 = np.array([(t, r) for t, p, r in res_lag["rr"]])
    ax[0].scatter(a0[:, 0] - t0, a0[:, 1], s=0.6, color=base, lw=0, rasterized=True)
    ax[0].scatter(a1[:, 0] - t0, a1[:, 1], s=0.6, color=blue, lw=0, rasterized=True)
    ax[0].set_ylim(-3, 3)
    ax[0].set_ylabel("range-rate error (m/s)")
    ax[0].text(0.01, 0.97, f"Doppler, each satellite, epoch common part removed:  gray as reported,  "
               f"blue as the range rate {lag:.2f} s earlier", transform=ax[0].transAxes, va="top",
               fontsize=7.5, color=ink2)
    pr = np.array([(t, p, r) for t, p, r in res0["pr"]])
    # the two highest satellites in color: highest mean elevation is the least-negative mean of -sin(el)
    eph, ep, own, kind = data
    top = []
    if len(pr):
        el = {}
        for k in sorted(ep)[::200]:
            tow, obs = ep[k]
            t = tow - TOW_FILE0
            for m in corrected(tow, obs, eph, truth.pos_ecef(t), use_tropo=False, el_mask_deg=-90.0):
                u = los_predict(m, truth.pos_ecef(t), np.zeros(3))[2]
                el.setdefault(m.prn, []).append(math.degrees(math.asin(-(truth.T @ u)[2])))
        top = sorted(el, key=lambda p: -np.median(el[p]))[:2]
    rest = ~np.isin(pr[:, 1], top)
    ax[1].scatter(pr[rest, 0] - t0, pr[rest, 2], s=0.6, color=base, lw=0, rasterized=True, zorder=2)
    for p, col in zip(top, (orange, aqua)):
        mm = pr[:, 1] == p
        ax[1].scatter(pr[mm, 0] - t0, pr[mm, 2], s=0.8, color=col, lw=0, rasterized=True, zorder=3)
        k = np.flatnonzero(mm)
        if len(k):
            ax[1].text(pr[k[-1], 0] - t0 + 3, pr[k[-1], 2], f"G{int(p)}, {np.median(el[p]):.0f} deg",
                       color=ink2, fontsize=7, va="center")
    ax[1].set_ylim(-45, 45)
    ax[1].set_ylabel("pseudorange error (m)")
    ax[1].text(0.01, 0.97, "pseudorange, each satellite, epoch common part removed (the two highest in color)",
               transform=ax[1].transAxes, va="top", fontsize=7.5, color=ink2)
    t_lo, t_hi = float(a0[:, 0].min()), float(a0[:, 0].max())
    et = np.array([ep[k][0] - TOW_FILE0 for k in sorted(ep)])
    ns = np.array([len(ep[k][1]) for k in sorted(ep)], float)
    sec = np.arange(math.floor(t_lo), math.ceil(t_hi))
    kk = np.searchsorted(sec, et, side="right") - 1
    ok = (kk >= 0) & (kk < len(sec))
    nmin = np.full(len(sec), np.nan)
    np.fmin.at(nmin, kk[ok], ns[ok])
    ax[2].step(sec - t0, nmin, where="post", color=blue, lw=1.0)
    ot = np.array([o[0] - TOW_FILE0 for o in own.values()])
    ko = np.searchsorted(sec, ot, side="right") - 1
    cnt = np.bincount(ko[(ko >= 0) & (ko < len(sec))], minlength=len(sec))
    ax[2].fill_between(sec - t0, 0, 1.2, where=cnt >= 10, color=aqua, lw=0, step="post")
    ax[2].set_ylim(0, 15)
    ax[2].set_ylabel("satellites\n(raw, fewest\nin each second)")
    ax[1].set_xlabel("seconds after ignition")
    for i, a in enumerate(ax):
        shade_limits(a, truth, t0, t_hi - t0, label_events=(i == 2))
        a.grid(color=grid, lw=0.5)
        for sp in a.spines.values():
            sp.set_color(grid)
    ax[0].set_xlim(t_lo - t0, t_hi - t0)
    fig.suptitle(title + "; amber = above 515 m/s, violet = above 80 km",
                 x=0.07, ha="left", fontsize=9.5, color=ink, y=1.0)
    fig.text(0.07, 0.955, "Green bar in the top panel: the receiver's own fix present.", fontsize=7.2,
             color=ink2, ha="left")
    fig.savefig(path, bbox_inches="tight", dpi=100 if path.endswith(".svg") else 110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("scenario")
    ap.add_argument("--pad", type=float, default=600.0)
    ap.add_argument("--stride", type=int, default=4, help="use every Nth 20 Hz epoch (default 4: 5 Hz)")
    ap.add_argument("--lags", default="-0.2:0.8:0.05", help="lag grid, s: start:stop:step")
    ap.add_argument("--plot", help="figure: Doppler as reported and at --plot-lag, pseudoranges, satellites")
    ap.add_argument("--plot-lag", dest="plot_lag", type=float, default=0.22)
    ap.add_argument("--plot-span", dest="plot_span", default="-40,320",
                    help="seconds after ignition the figure covers")
    ap.add_argument("--plot-title", dest="plot_title")
    args = ap.parse_args()
    truth = RigTruth(args.scenario, args.pad)
    data = load_capture(args.capture)
    a, b, c = (float(x) for x in args.lags.split(":"))
    lags = [round(x, 4) for x in np.arange(a, b + 1e-9, c)]
    if 0.0 not in lags:
        lags.append(0.0)
        lags.sort()
    res = residuals(truth, data, lags, args.stride, truth.t_ign - 30.0, truth.t_end)
    print(f"{os.path.basename(args.capture)}: clock-rate offset on the pad (Doppler drift minus the "
          f"pseudoranges' clock-bias rate) {clock_rate_offset(truth, data):+.2f} m/s")
    print(f"\nbest lag per phase, s  [RMS there / at lag 0]:   pr m, rr m/s, cr m over 1 s")
    print(f"{'phase':<22}" + "".join(f"{k:>26}" for k in KINDS))
    for row in best_lags(truth, res, lags):
        cells = []
        for x in row[1:]:
            cells.append(f"{'--':>26}" if x is None else f"{x[0]:+6.2f} [{x[1]:7.2f} /{x[2]:7.2f}]".rjust(26))
        print(f"{row[0]:<22}" + "".join(cells))
    if args.plot:
        s0, s1 = (float(x) for x in args.plot_span.split(","))
        lo, hi = truth.t_ign + s0, truth.t_ign + s1
        lag = args.plot_lag
        extra = residuals(truth, data, [0.0, lag], 8, lo, hi)
        plot(args.plot, truth, data, extra[0.0], extra[lag], lag,
             args.plot_title or os.path.basename(args.capture))
        print("wrote", args.plot)


if __name__ == "__main__":
    main()
