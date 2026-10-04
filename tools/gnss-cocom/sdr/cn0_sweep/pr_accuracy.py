#!/usr/bin/env python3
"""Pseudorange and range-rate accuracy of a PX1105R SignalSim traveler capture, GPS / Galileo / BeiDou.

Truth is the trajectory SignalSim was given (signalsim/make_2026_configs.py: the pad as one Const segment, then one
0.1 s VerticalAcc segment per CSV row, speed linear inside each), rebuilt here from the same CSV. Satellites come
from the capture's own broadcast ephemeris; gnss_raw.corrected adds back the satellite clock and group delay and
removes the broadcast Klobuchar ionosphere (SignalSim applies that one model, with GPS's parameters, to all three
systems, scaled to B1I's frequency); SignalSim's troposphere (Saastamoinen, RH 0.7, nothing above 10 km) is removed
here. What is left is the receiver clock, a constant offset per constellation, and the measurement error.

The receiver clock. The rig's code and carrier disagree by ~4.2 m/s (the HackRF's LO against its sample clock);
the PX1105R's Galileo and BeiDou pseudoranges follow the code, most of its GPS channels follow the carrier and slide
off the code at that rate until they lose lock. So the clock comes from the code-following systems only: per epoch
the median of the Galileo and BeiDou pseudoranges (BeiDou's offset to Galileo measured where both are present),
taken relative to the carrier clock (the Doppler drift integrated), smoothed over +-SMOOTH_S and interpolated across
gaps -- the Doppler bridges the epochs with fewer than two code-following satellites. GPS's offset is the level its
channels come back at after a lock (0.5-2 s after, on the pad), before they slide. The Doppler is compared with the
range rate RR_LAG earlier (0.03 s on this rig; the gps-sdr-sim rig's 0.22 s does not apply), epoch median removed.

    pr_accuracy.py CAPTURE OUT.npz [--rr-lag 0.03] [--stride 2]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

WT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(WT / "tinkerrocket-sim" / "src"), str(WT / "tinkerrocket-sim" / "scripts")]
from tc_ekf_cocom import load_capture, RelockAge, TOW_FILE0            # noqa: E402
from tinkerrocket_sim.estimation.gnss_raw import corrected             # noqa: E402
from tinkerrocket_sim.estimation.tc_ekf import los_predict, lla2ecef, t_e2ned  # noqa: E402

SDR = WT / "tools" / "gnss-cocom" / "sdr"
IGN = 600.0                                   # ignition, file time
LAT0, LON0 = 0.0, math.radians(-119.0)
SYSN = "GEC"
NAME = {"G": "GPS", "E": "Galileo", "C": "BeiDou"}
SMOOTH_S = 10.0
CMS = 299792.458                              # one millisecond of range, m
STEP_M = 1000.0                               # a code-clock jump this big between epochs is a clock step
GROSS_M = 1000.0                              # a pseudorange this far off is a whole-millisecond (or worse) error


class SignalSimTruth:
    """The vertical motion SignalSim integrated: speed u_k at the end of each 0.1 s segment, linear inside it."""

    def __init__(self, csv):
        rows = [[float(x) for x in line.split(",")] for line in Path(csv).read_text().split()]
        t = [r[0] for r in rows]
        h = [r[3] for r in rows]
        dt = round(t[1] - t[0], 6)
        v = [0.0] + [(h[k] - h[k - 1]) / dt for k in range(1, len(h))]
        lift = next(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)
        last = max(k for k in range(1, len(v)) if abs(v[k]) > 1e-9)
        u = [0.0] * len(v)
        for k in range(lift, last):
            u[k] = 0.5 * (v[k] + v[k + 1])
        u[last] = v[last] if last == len(v) - 1 else 0.0
        t0 = round(t[lift - 1], 6)
        ks = range(lift - 1, last + 1)
        self.tn = np.array([t0 + (k - lift + 1) * dt for k in ks])
        self.un = np.array([u[k] for k in ks])
        self.an = np.concatenate([[h[lift - 1]], h[lift - 1] + np.cumsum(0.5 * (self.un[1:] + self.un[:-1]) * dt)])
        self.dt, self.h0 = dt, h[lift - 1]
        self.T = t_e2ned(LAT0, LON0)
        self.max_alt_err = max(abs(self.an[i] - h[lift - 1 + i]) for i in range(len(self.an)))

    def vup(self, t):
        if t <= self.tn[0]:
            return 0.0
        if t >= self.tn[-1]:
            return float(self.un[-1])
        return float(np.interp(t, self.tn, self.un))

    def alt(self, t):
        if t <= self.tn[0]:
            return self.h0
        if t >= self.tn[-1]:
            return float(self.an[-1] + self.un[-1] * (t - self.tn[-1]))
        i = int(np.searchsorted(self.tn, t, side="right")) - 1
        tau = t - self.tn[i]
        return float(self.an[i] + self.un[i] * tau + 0.5 * (self.un[i + 1] - self.un[i]) / self.dt * tau * tau)

    def pos(self, t):
        return lla2ecef(np.array([LAT0, LON0, self.alt(t)]))

    def vel(self, t):
        return self.T.T @ np.array([0.0, 0.0, -self.vup(t)])


def tropo_signalsim(lat, alt, el):
    """SignalSim's TropoDelay (src/Coordinate.cpp): RTKLIB's Saastamoinen, standard atmosphere, RH 0.7."""
    if alt < -100.0 or alt > 1e4 or el <= 0:
        return 0.0
    alt = max(alt, 0.0)
    p = 1013.25 * (1.0 - 2.2557e-5 * alt) ** 5.2568
    tk = 288.16 - 6.5e-3 * alt
    e = 6.108 * 0.7 * math.exp((17.15 * tk - 4684.0) / (tk - 38.45))
    cz = math.cos(math.pi / 2 - el)
    trph = 0.0022767 * p / (1.0 - 0.00266 * math.cos(2.0 * lat) - 0.00028 * alt / 1e3) / cz
    trpw = 0.002277 * (1255.0 / tk + 0.05) * e / cz
    return trph + trpw


def phases(scen):
    """Flight phases, seconds from ignition, from the scenario's events."""
    S = json.loads(Path(scen).read_text())
    tr, pro = S["truth"], S["prologue_s"]
    burn = next(s["t"] for s in tr if s["phase"] == "coast") - pro
    vw = [(a - pro, b - pro) for a, b in S["velocity_windows"]]
    if len(vw) == 1 and not S["altitude_80km_windows"]:     # hotshot: one spell over 515 m/s, never 80 km
        u0, d1 = vw[0]
        return [("pad, last 60 s", -60.0, 0.0),
                ("burn, below 515 m/s", 0.0, u0),
                ("burn, above 515 m/s", u0, burn),
                ("coast, above 515 m/s", burn, d1),
                ("after 515 m/s, fix allowed", d1, tr[-1]["t"] - pro)]
    (u0, _u1), (_d0, d1) = vw
    a0, a1 = [(a - pro, b - pro) for a, b in S["altitude_80km_windows"]][0]
    return [("pad, last 60 s", -60.0, 0.0),
            ("burn, below 515 m/s", 0.0, u0),
            ("burn, above 515 m/s", u0, burn),
            ("coast up, below 80 km", burn, a0),
            ("above 80 km", a0, a1),
            ("falling, 80 km to 515 m/s", a1, d1),
            ("descent, fix allowed", d1, tr[-1]["t"] - pro)]


def rows_for(tr, eph, ep, keys, ages, rr_lag):
    """[(t, sys, prn, pr - range, rr - range rate, cn0, age, el)], file time."""
    out = []
    for k in keys:
        tow, obs = ep[k]
        t = tow - TOW_FILE0
        r, v = tr.pos(t), tr.vel(t)
        rl, vl = tr.pos(t - rr_lag), tr.vel(t - rr_lag)
        h = tr.alt(t)
        cn = {(o[0], o[1]): o[4] for o in obs}
        for m in corrected(tow, obs, eph, r, use_tropo=False, el_mask_deg=-90.0, doppler_step_hz=1.0):
            rho, rate, u = los_predict(m, r, v)
            el = math.asin(max(-1.0, min(1.0, -float((tr.T @ u)[2]))))
            pr = m.pr - tropo_signalsim(LAT0, h, el) - rho
            rr = math.nan if m.rr is None else m.rr - los_predict(m, rl, vl)[1]
            out.append((t, SYSN.index(m.sys), m.prn, pr, rr, cn[(m.sys, m.prn)],
                        ages.get(k, {}).get((m.sys, m.prn), 0.0), math.degrees(el)))
    return np.array(out, float)


def by_epoch(A):
    ut, inv = np.unique(A[:, 0], return_inverse=True)
    order = np.argsort(inv, kind="stable")
    bounds = np.searchsorted(inv[order], np.arange(len(ut) + 1))
    return ut, [order[bounds[i]:bounds[i + 1]] for i in range(len(ut))]


def running_median(t, y, half):
    lo = np.searchsorted(t, t - half, side="left")
    hi = np.searchsorted(t, t + half, side="right")
    return np.array([np.median(y[a:b]) for a, b in zip(lo, hi)])


def arcs(t, gap=0.5):
    """Index runs split where the time step exceeds gap."""
    brk = np.flatnonzero(np.diff(t) > gap)
    return np.split(np.arange(len(t)), brk + 1)


def stats_line(x):
    x = x[np.isfinite(x)]
    if len(x) < 5:
        return f"{len(x):>6}{'':>24}"
    return (f"{len(x):>6} {math.sqrt(float(np.mean(x ** 2))):>7.2f} {float(np.median(x)):>+7.2f} "
            f"{float(np.percentile(np.abs(x), 95)):>7.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("out")
    ap.add_argument("--csv", default=str(SDR / "scenarios" / "traveler_soft25_pad600.csv"))
    ap.add_argument("--scenario", default=str(SDR / "scenarios" / "traveler_soft25.json"))
    ap.add_argument("--stride", type=int, default=2, help="every Nth 20 Hz epoch (default 2: 10 Hz)")
    ap.add_argument("--rr-lag", dest="rr_lag", type=float, default=0.03)
    args = ap.parse_args()

    tr = SignalSimTruth(args.csv)
    print(f"truth: SignalSim segments rebuilt, lift-off {tr.tn[0]:.1f} s, max |alt - CSV| {tr.max_alt_err:.3f} m")
    eph, ep, own, kind = load_capture(args.capture, systems=SYSN)
    keys = sorted(ep)
    age, ages = RelockAge(gap_s=0.5), {}
    for k in keys:
        tow, obs = ep[k]
        t = tow - TOW_FILE0
        age.update(t, obs)
        if k % args.stride == 0:
            ages[k] = {(o[0], o[1]): age.age(o[0], o[1], t) for o in obs}
    use = [k for k in keys if k % args.stride == 0]
    A = rows_for(tr, eph, ep, use, ages, args.rr_lag)
    t, s, raw, rrr, agec = A[:, 0], A[:, 1].astype(int), A[:, 3], A[:, 4], A[:, 6]
    print("ephemeris-backed satellites:",
          {NAME[c]: len({int(p) for p in A[s == i, 2]}) for i, c in enumerate(SYSN) if (s == i).any()})
    ut, groups = by_epoch(A)
    pad = (ut >= IGN - 300.0) & (ut <= IGN - 5.0)

    # carrier clock: the epoch's Doppler drift (median over every satellite, >= 3), a 2 s running median, the burn
    # (where every Doppler carries metres per second of tracking stress) bridged by a straight line, integrated
    drift = np.array([np.median(rrr[g][np.isfinite(rrr[g])]) if np.isfinite(rrr[g]).sum() >= 3 else np.nan
                      for g in groups])
    ev = phases(args.scenario)
    burn_end = ev[2][2]
    ok = np.isfinite(drift) & ~((ut >= IGN - 0.5) & (ut <= IGN + burn_end + 1.0))
    dsm = running_median(ut[ok], drift[ok], 1.0)
    dfull = np.interp(ut, ut[ok], dsm)
    car = np.concatenate([[0.0], np.cumsum(0.5 * (dfull[1:] + dfull[:-1]) * np.diff(ut))])

    # BeiDou against Galileo, where both have >= 2
    d_ce = []
    for i, g in enumerate(groups):
        ge, gc = g[s[g] == 1], g[s[g] == 2]
        if len(ge) >= 2 and len(gc) >= 2:
            d_ce.append((ut[i], float(np.median(raw[gc]) - np.median(raw[ge]))))
    isb = np.zeros(3)
    if d_ce:
        d = np.array(d_ce)
        on_pad = (d[:, 0] >= IGN - 300.0) & (d[:, 0] <= IGN - 5.0)
        isb[2] = float(np.median(d[on_pad, 1] if on_pad.sum() >= 50 else d[:, 1]))
        print(f"BeiDou offset against Galileo: {isb[2]:+.2f} m ({'pad' if on_pad.sum() >= 50 else 'whole run'}, "
              f"{len(d)} epochs)")

    # code clock from the code-following systems, relative to the carrier clock, smoothed, bridged by the Doppler
    code = np.full(len(ut), np.nan)
    for i, g in enumerate(groups):
        gg = g[s[g] >= 1]
        if len(gg) >= 2:
            code[i] = float(np.median(raw[gg] - isb[s[gg]]))
    k = np.isfinite(code)
    # the receiver's clock steps by whole milliseconds now and then: follow the steps exactly, smooth the rest
    oc = code[k] - car[k]
    # the receiver's clock steps by whole milliseconds now and then, and by odd amounts when a replay underrun
    # delays the signal or the first fix after an outage re-sets its time: follow any step over STEP_M exactly
    d_oc = np.diff(oc)
    steps = np.concatenate([[0.0], np.where(np.abs(d_oc) > STEP_M, d_oc, 0.0)])
    jumps = np.cumsum(steps)
    y, tk = oc - jumps, ut[k]
    # code minus carrier runs at the rig's split (~4.2 m/s): take that trend out before smoothing, or the running
    # median lags wherever its window is one-sided (either side of a gap, at the ends)
    j = np.searchsorted(tk, tk + 20.0)
    ok2 = j < len(tk)
    split = float(np.median((y[j[ok2]] - y[ok2]) / (tk[j[ok2]] - tk[ok2])))
    off = running_median(tk, y - split * tk, SMOOTH_S) + split * tk
    jpos = np.clip(np.searchsorted(tk, ut, side="right") - 1, 0, k.sum() - 1)
    clock = car + np.interp(ut, tk, off) + jumps[jpos]
    print(f"receiver clock: {int(np.count_nonzero(steps))} steps "
          f"({', '.join(f'T{tk[i] - IGN:+.1f} s {steps[i] / CMS:+.3f} ms' for i in np.flatnonzero(steps)[:8])})")
    dist = np.abs(ut - tk[np.clip(np.searchsorted(tk, ut), 0, k.sum() - 1)])
    dist = np.minimum(dist, np.abs(ut - tk[np.clip(np.searchsorted(tk, ut) - 1, 0, k.sum() - 1)]))
    print(f"code-following clock in {k.sum()}/{len(ut)} epochs; code minus carrier {split:+.2f} m/s "
          f"(the rig's split; median of 20 s slopes)")

    # GPS's offset. With code and carrier consistent (a corrected file) GPS holds the code: its epoch median against
    # the code-following clock on the pad. With the rig's split GPS slides, so only the level its channels come
    # back at (0.5-2 s after a lock) is the code's.
    ep_idx = np.searchsorted(ut, t)
    res0 = raw - clock[ep_idx]
    on_pad = (t >= IGN - 200.0) & (t <= IGN - 5.0)
    if abs(split) < 0.5:
        sel = (s == 0) & on_pad & (agec >= 20.0)
        how = "settled channels, pad, code and carrier consistent"
    else:
        sel = (s == 0) & on_pad & (agec >= 0.5) & (agec <= 2.0)
        how = "channels 0.5-2 s after a lock, pad"
    if sel.sum() >= 20:
        isb[0] = float(np.median(res0[sel]))
    print(f"GPS offset against Galileo ({how}): {isb[0]:+.2f} m ({sel.sum()} rows)")
    if abs(split) < 0.5:
        # code and carrier consistent: every system holds the code, so the clock comes from all of them (after
        # their offsets) -- no one system sets it alone and then reads its own errors as small
        code2 = np.full(len(ut), np.nan)
        for i, g in enumerate(groups):
            if len(g) >= 3:
                code2[i] = float(np.median(raw[g] - isb[s[g]]))
        k = np.isfinite(code2)
        oc = code2[k] - car[k]
        d_oc = np.diff(oc)
        steps = np.concatenate([[0.0], np.where(np.abs(d_oc) > STEP_M, d_oc, 0.0)])
        jumps = np.cumsum(steps)
        y, tk = oc - jumps, ut[k]
        off = running_median(tk, y - split * tk, SMOOTH_S) + split * tk
        jpos = np.clip(np.searchsorted(tk, ut, side="right") - 1, 0, k.sum() - 1)
        clock = car + np.interp(ut, tk, off) + jumps[jpos]
        dist = np.abs(ut - tk[np.clip(np.searchsorted(tk, ut), 0, k.sum() - 1)])
        dist = np.minimum(dist, np.abs(ut - tk[np.clip(np.searchsorted(tk, ut) - 1, 0, k.sum() - 1)]))
        res0 = raw - clock[ep_idx]
        print(f"clock re-taken from all systems ({k.sum()} epochs with >= 3 satellites)")
    pr = res0 - isb[s]
    rr = np.full(len(A), np.nan)
    for g in groups:
        y = rrr[g]
        f = np.isfinite(y)
        if f.sum() >= 3:
            rr[g[f]] = y[f] - np.median(y[f])
    bridged = dist[ep_idx] > 2.0

    # GPS arcs: sliding (following the carrier) or holding (following the code)
    sliding = np.zeros(len(A), bool)
    slopes = []
    for q in sorted({int(x) for x in A[s == 0, 2]}):
        idx = np.flatnonzero((s == 0) & (A[:, 2] == q))
        for a in arcs(t[idx]):
            ii = idx[a]
            good = ii[~bridged[ii] & (np.abs(pr[ii]) < GROSS_M)]
            if len(good) >= 50 and t[good[-1]] - t[good[0]] >= 10.0:
                slope = float(np.polyfit(t[good], pr[good], 1)[0])
                slopes.append(slope)
                if slope < -2.0:
                    sliding[ii] = True
    sl_arr = np.array(slopes)
    if len(sl_arr):
        print(f"GPS arcs >= 10 s: {len(sl_arr)}, sliding (< -2 m/s) {int((sl_arr < -2).sum())}, median slope of the "
              f"sliding {np.median(sl_arr[sl_arr < -2]) if (sl_arr < -2).any() else math.nan:+.2f} m/s, of the "
              f"rest {np.median(sl_arr[sl_arr >= -2]) if (sl_arr >= -2).any() else math.nan:+.2f} m/s")

    tI = t - IGN
    gross = np.abs(pr) > GROSS_M
    print(f"\npseudorange error, m (n, RMS, median, 95% |err|; errors over {GROSS_M:.0f} m counted apart); clock "
          f"within 2 s of a code-following epoch")
    print(f"{'phase':<27} {'':<9} {'n':>6} {'RMS':>7} {'median':>7} {'p95':>7}   gross")
    for name, a, b in phases(args.scenario):
        inph = (tI >= a) & (tI < b) & ~bridged
        first = True
        for label, m in (("GPS", s == 0), ("Galileo", s == 1), ("BeiDou", s == 2)):
            mm = inph & m
            if not mm.any():
                continue
            ng = int((mm & gross).sum())
            print(f"{(name if first else ''):<27} {label:<9} {stats_line(pr[mm & ~gross])}   "
                  f"{ng if ng else ''}")
            first = False
        nb = int(((tI >= a) & (tI < b) & bridged).sum())
        if first:
            print(f"{name:<27} {'--':<16} (no epochs with a code-following clock; {nb} bridged rows)")
        elif nb:
            print(f"{'':<27} {'(bridged rows)':<16} {nb:>6}")
    print(f"\nrange-rate error (Doppler vs the truth {args.rr_lag:.2f} s earlier, epoch median removed), m/s")
    for name, a, b in phases(args.scenario):
        inph = (tI >= a) & (tI < b)
        first = True
        for i, c in enumerate(SYSN):
            mm = inph & (s == i)
            if not mm.any():
                continue
            print(f"{(name if first else ''):<27} {NAME[c]:<16} {stats_line(rr[mm])}")
            first = False
    np.savez(args.out, t=tI, sys=s, prn=A[:, 2], pr_raw=raw, rr_raw=rrr, cn0=A[:, 5], age=agec, el=A[:, 7],
             pr=pr, rr=rr, bridged=bridged, sliding=sliding, isb=isb, ut=ut - IGN, clock=clock, car=car,
             code=code, drift=drift, split=split)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
