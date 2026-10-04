#!/usr/bin/env python3
"""How a gnssrx run tracked through a boost, against the scenario's truth.

For each satellite, along the trajectory the file was simulated on:
  * frequency error: the loop's Doppler (trk.csv) minus the line-of-sight truth, less the
    common receiver-clock term (the median over satellites, pre-launch);
  * carrier slips: accumulated phase (obs.csv) against the true phase, single-differenced
    against the per-epoch median over satellites; a slip is a step of a half or whole cycle.
    The true phase is the integral of the true Doppler, linear between the trajectory's
    samples, as the rig's smoothed gps-sdr-sim builds the carrier: the range itself, from
    linearly interpolated positions, differs from that by a dt^2 / 8 within a sample, which at
    a burnout's step in acceleration is whole cycles;
  * code error: pseudorange minus the true range, the same way;
  * lock: tracking state, the cos 2phi indicator and C/N0;
and the fix (pvt.csv) against the trajectory.

    boost_track.py RUN_DIR --traj SCEN.csv --nav BRDC.rnx [--from 595 --to 640]

Times are file seconds (the run's t_s plus its start, from run.ini or --start). --t0-gps (GPS
seconds of week at file second 0) defaults to the first observation's receive time. Carrier
phase is judged only where the channel reports PLL lock (lock_s > 0); code and frequency
everywhere.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gnssrx import rinex, truth  # noqa: E402


def read_csv(path: Path) -> dict[str, np.ndarray]:
    with open(path) as f:
        r = csv.reader(f)
        head = next(r)
        rows = [list(map(float, x)) for x in r if x]
    a = np.array(rows) if rows else np.zeros((0, len(head)))
    return {h: a[:, k] for k, h in enumerate(head)}


def doppler_phase(nav, prn: int, traj: truth.Trajectory, tq: np.ndarray, t0: float) -> np.ndarray:
    """The integral of the true Doppler (cycles, from an arbitrary origin) at times tq."""
    ts = traj.t[(traj.t >= tq.min() - 0.2) & (traj.t <= tq.max() + 0.2)]
    dop = truth.los(nav, prn, traj, ts, t0)[:, 1]
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (dop[1:] + dop[:-1]) * np.diff(ts))])
    k = np.clip(np.searchsorted(ts, tq, side="right") - 1, 0, ts.size - 2)
    u = tq - ts[k]
    f_at = dop[k] + (dop[k + 1] - dop[k]) * u / (ts[k + 1] - ts[k])
    return cum[k] + 0.5 * (dop[k] + f_at) * u


def enu_basis(lat_deg: float, lon_deg: float) -> np.ndarray:
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    return np.array([[-math.sin(lo), math.cos(lo), 0.0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def single_diff(t: np.ndarray, prn: np.ndarray, x: np.ndarray, valid: np.ndarray):
    """x minus the per-epoch median over satellites (removes the receiver clock); per PRN."""
    out = {}
    epochs = np.unique(t)
    med = {}
    for te in epochs:
        m = (t == te) & valid
        if m.sum() >= 4:
            med[te] = np.median(x[m])
    for p in np.unique(prn):
        m = (prn == p) & valid
        tt = t[m]
        keep = np.array([te in med for te in tt])
        tt = tt[keep]
        out[int(p)] = (tt, x[m][keep] - np.array([med[te] for te in tt]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--traj", type=Path, required=True)
    ap.add_argument("--nav", type=Path, required=True)
    ap.add_argument("--start", type=float, help="file second of the run's t_s = 0 (default: run.ini)")
    ap.add_argument("--t0-gps", type=float, help="GPS seconds of week at file second 0")
    ap.add_argument("--from", dest="t_from", type=float, help="report window start, file s")
    ap.add_argument("--to", dest="t_to", type=float, help="report window end, file s")
    ap.add_argument("--liftoff", type=float, default=600.3, help="file s (pre-launch reference ends here)")
    ap.add_argument("--burnout", type=float, help="file s: count losses as the bench reports do, from liftoff "
                    "to 0.5 s past burnout, a loss being no pseudorange for 0.5 s or more")
    ap.add_argument("--slip", type=float, default=0.25, help="carrier step (cycles) counted as a slip")
    ap.add_argument("--series", type=Path, help="write per-satellite series CSV here")
    a = ap.parse_args()
    if a.start is None:
        ini = a.run / "run.ini"
        if not ini.exists():
            ap.error("--start is needed: the run has no run.ini")
        for line in ini.read_text().splitlines():
            if line.startswith("start_s"):
                a.start = float(line.split("=")[1])

    trk = read_csv(a.run / "trk.csv")
    obs = read_csv(a.run / "obs.csv")
    pvt = read_csv(a.run / "pvt.csv")
    nav = rinex.read_nav(a.nav, "GEC")
    traj = truth.Trajectory(a.traj)
    lam = truth.LAMBDA_L1

    t0 = a.t0_gps
    if t0 is None:
        t0 = float(np.round(obs["rx_tow"][0] - obs["t_s"][0] - a.start, 3))
    tf_trk = trk["t_s"] + a.start
    t_from = a.t_from if a.t_from is not None else a.liftoff - 5
    t_to = a.t_to if a.t_to is not None else float(tf_trk.max())
    print(f"{a.run}: file {t_from:.1f}-{t_to:.1f} s, GPS TOW at file 0 = {t0:.3f}")

    # --- frequency error and lock, at trk.csv's rate ---
    prns = sorted({int(p) for p in trk["prn"]})  # GPS, Galileo (+100) and BeiDou (+200)
    los = {}
    for p in prns:
        m = trk["prn"] == p
        los[p] = truth.los(nav, p, traj, tf_trk[m], t0)
    ferr_raw = np.full(tf_trk.size, np.nan)
    for p in prns:
        m = trk["prn"] == p
        ferr_raw[m] = trk["dop_hz"][m] - los[p][:, 1]
    pre = (tf_trk > a.liftoff - 30) & (tf_trk < a.liftoff - 1) & (trk["state"] >= 2)
    clk = float(np.median(ferr_raw[pre])) if pre.any() else 0.0
    ferr = ferr_raw - clk

    # --- carrier and code against the truth, at obs.csv's rate (from 30 s before liftoff) ---
    keep = obs["t_s"] + a.start >= min(t_from, a.liftoff - 30)
    obs = {k: v[keep] for k, v in obs.items()}
    tf_obs = obs["t_s"] + a.start
    rng = np.full(tf_obs.size, np.nan)
    ph = np.full(tf_obs.size, np.nan)
    for p in np.unique(obs["prn"]).astype(int):
        m = obs["prn"] == p
        rng[m] = truth.los(nav, p, traj, tf_obs[m], t0)[:, 0]
        ph[m] = doppler_phase(nav, p, traj, tf_obs[m], t0)
    # Carrier and code in metres less the true range; each satellite's own pre-launch level (its
    # clock, the phase's arbitrary start) taken out, then the per-epoch median (the receiver clock).
    adr_m = (obs["adr_cyc"] + ph) * lam  # RINEX phase grows with range, i.e. falls with Doppler
    code_m = obs["pr_m"] - rng
    for p in np.unique(obs["prn"]):
        m = obs["prn"] == p
        ref = m & (tf_obs > a.liftoff - 30) & (tf_obs < a.liftoff - 1)
        adr_m[m] -= np.median(adr_m[ref]) if ref.any() else adr_m[m][0]
        code_m[m] -= np.median(code_m[ref]) if ref.any() else code_m[m][0]
    locked = np.isfinite(adr_m) & (obs["lock_s"] > 0)
    sd_adr = single_diff(tf_obs, obs["prn"], adr_m, locked)
    sd_code = single_diff(tf_obs, obs["prn"], code_m, np.isfinite(code_m))
    relocks = {}
    for p in np.unique(obs["prn"]).astype(int):
        m = (obs["prn"] == p) & (tf_obs >= t_from) & (tf_obs <= t_to)
        relocks[p] = int(np.sum(np.diff(obs["lock_s"][m]) < 0))

    win = (tf_trk >= t_from) & (tf_trk <= t_to)
    print(f"receiver clock (pre-launch frequency offset): {clk:+.2f} Hz")
    print(" PRN  el  |rate|max  |ferr|max  at        unlocked  relocks  carrier net  steps (cyc@s)                 code |dev|max m")
    rows = []
    for p in prns:
        m = (trk["prn"] == p) & win
        if not m.any():
            continue
        idx = np.where(trk["prn"] == p)[0]
        sel = win[idx]
        L = los[p][sel]
        rate = np.abs(L[:, 2])
        fe = ferr[idx][sel]
        lock = trk["pll_lock"][idx][sel]
        cn0 = trk["cn0"][idx][sel]
        st = trk["state"][idx][sel]
        k = int(np.nanargmax(np.abs(fe)))
        slips, net = [], float("nan")
        if p in sd_adr:
            tt, x = sd_adr[p]
            w = (tt >= t_from) & (tt <= t_to)
            if w.sum() > 2:
                d = np.diff(x[w]) / lam
                for j in np.where(np.abs(d) >= a.slip)[0]:
                    slips.append((tt[w][j + 1], d[j]))
                net = x[w][-1] / lam
        cmax = float("nan")
        if p in sd_code:
            tt, x = sd_code[p]
            w = (tt >= t_from) & (tt <= t_to)
            if w.any():
                cmax = float(np.max(np.abs(x[w])))
        slip_txt = " ".join(f"{s[1]:+.1f}@{s[0]:.1f}" for s in slips[:4]) + (" ..." if len(slips) > 4 else "")
        print(f" {p:3d}  {np.median(L[:, 3]):2.0f}  {rate.max():8.0f}  {np.nanmax(np.abs(fe)):9.1f}  "
              f"{tf_trk[idx][sel][k]:6.1f}  {(st < 2).sum() / 10:6.1f} s  {relocks.get(p, 0):7d}  {net:+10.1f}  "
              f"{slip_txt:<29s} {cmax:8.2f}")
        rows.append((p, L, fe, lock, cn0, st, tf_trk[idx][sel]))

    # --- losses as the bench reports count them (cn0_boost_report): no pseudorange for >= 0.5 s ---
    if a.burnout is not None:
        w0, w1 = a.liftoff, a.burnout + 0.5
        lost = []
        for p in prns:
            tt = np.sort(tf_obs[(obs["prn"] == p) & np.isfinite(obs["pr_m"])])
            tt = tt[(tt >= w0 - 0.5) & (tt <= w1 + 0.5)]
            edges = np.concatenate([[w0], tt[(tt > w0) & (tt < w1)], [w1]])
            gaps = np.diff(edges)
            if tt.size == 0 or gaps.max() >= 0.5:
                at = edges[int(np.argmax(gaps))] if tt.size else w0
                idx = np.where(trk["prn"] == p)[0]
                rate = float(np.interp(at, tf_trk[idx], np.abs(los[p][:, 2])))
                lost.append(f"PRN {p} at {at:.1f} s ({rate:.0f} Hz/s)")
        print(f"lost (no pseudorange for 0.5 s, {w0:.1f}-{w1:.1f} s): {len(lost)} of {len(prns)}"
              + (": " + ", ".join(lost) if lost else ""))

    # --- the fix against the trajectory ---
    tf_pvt = pvt["t_s"] + a.start
    w = (tf_pvt >= t_from) & (tf_pvt <= t_to)
    if w.any():
        pos = np.stack([pvt["x"], pvt["y"], pvt["z"]], axis=-1)[w]
        vel = np.stack([pvt["vx"], pvt["vy"], pvt["vz"]], axis=-1)[w]
        tp, tv = traj.state(tf_pvt[w])
        R = enu_basis(float(traj.lat[0]), float(traj.lon[0]))
        dp = (pos - tp) @ R.T
        dv = (vel - tv) @ R.T
        gap = np.diff(tf_pvt[w])
        print(f"fix: {w.sum()} epochs in the window, longest gap {gap.max() if gap.size else 0:.2f} s; "
              f"position error E/N/U rms {np.sqrt(np.mean(dp**2, axis=0)).round(2)} m, max |U| {np.abs(dp[:, 2]).max():.1f} m; "
              f"velocity error rms {np.sqrt(np.mean(dv**2, axis=0)).round(2)} m/s, max |U| {np.abs(dv[:, 2]).max():.2f} m/s")
    else:
        print("fix: none in the window")

    if a.series:
        with open(a.series, "w") as f:
            f.write("t_file,prn,el_deg,dop_true,rate_true,ferr_hz,pll_lock,cn0,state\n")
            for p, L, fe, lock, cn0, st, tt in rows:
                for j in range(tt.size):
                    f.write(f"{tt[j]:.3f},{p},{L[j, 3]:.2f},{L[j, 1]:.3f},{L[j, 2]:.2f},{fe[j]:.3f},"
                            f"{lock[j]:.3f},{cn0[j]:.2f},{int(st[j])}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
