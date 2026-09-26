#!/usr/bin/env python3
"""Fit the raw-measurement error model from a STATIC capture.

Holds the antenna at the capture's own median fix, removes the receiver clock,
drift and inter-system biases epoch by epoch, and characterises what is left
per satellite arc:

  white          epoch-to-epoch noise, from first differences / sqrt(2)
  correlated     the rest: per-arc offset and wander (multipath, atmosphere,
                 orbit), with its 1/e correlation time from the autocorrelation

then fits sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10) to each, which is the form
``estimation/gnss_raw.py`` uses (PR_WHITE, PR_CORR, RR_WHITE). It also checks
the Doppler against the carrier phase, which is how SkyTraq's whole-hertz,
truncated-toward-zero Doppler was found.

    PYTHONPATH=src python3 scripts/raw_error_model.py static_20hz.log
"""
from __future__ import annotations

import argparse
import collections
import gzip
import math
import os
import struct
import sys
import tempfile

import numpy as np

from tinkerrocket_sim.estimation.gnss_raw import read_capture, corrected, SYS, SKY_GNSS
from tinkerrocket_sim.estimation.tc_ekf import spp_fix, los_predict, ecef2lla, t_e2ned

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "gnss-cocom"))
from gnss_nmea_monitor import replay_source  # noqa: E402

C = 299792458.0


def open_capture(path):
    if path.endswith(".gz"):
        tmp = tempfile.NamedTemporaryFile("wb", suffix=".log", delete=False)
        tmp.write(gzip.open(path, "rb").read()); tmp.close()
        return tmp.name
    return path


def residuals(path, doppler_fix=True):
    eph, epochs, _own, kind = read_capture(path, replay_source, fix_doppler_truncation=doppler_fix)
    x, fixes = None, []
    for tow, obs in epochs[::max(1, len(epochs) // 1200)]:
        for _ in range(2):
            fx = spp_fix(corrected(tow, obs, eph, x), x)
            if fx is None:
                break
            x = fx[0]
        if fx is not None:
            fixes.append(x.copy())
    xref = np.median(np.array(fixes), axis=0)
    T = t_e2ned(*ecef2lla(xref)[:2])
    rows = []
    for tow, obs in epochs:
        m = corrected(tow, obs, eph, xref)
        if len(m) < 5:
            continue
        cn0 = {(o[0], o[1]): o[4] for o in obs}
        pr, rr, H, meta = [], [], [], []
        for mm in m:
            rho, rate, u = los_predict(mm, xref, np.zeros(3))
            meta.append(("GEC".index(mm.sys), mm.prn, cn0.get((mm.sys, mm.prn), 0), math.asin(-(T @ u)[2])))
            pr.append(mm.pr - rho)
            rr.append(np.nan if mm.rr is None else mm.rr - rate)
            H.append([1.0, float(mm.sys == "E"), float(mm.sys == "C")])
        pr, rr, H = np.array(pr), np.array(rr), np.array(H)
        cols = [0] + [k for k in (1, 2) if H[:, k].any()]
        pr = pr - H[:, cols] @ np.linalg.lstsq(H[:, cols], pr, rcond=None)[0]
        ok = ~np.isnan(rr)
        rr = rr - (np.median(rr[ok]) if ok.any() else 0.0)
        for k, (si, prn, c, el) in enumerate(meta):
            rows.append((tow, si, prn, c, el, pr[k], rr[k]))
    return np.array(rows, float), kind


def arcs_of(A, min_len):
    t, si, prn = A[:, 0], A[:, 1], A[:, 2]
    out = []
    for key in sorted(set(zip(si.astype(int), prn.astype(int)))):
        idx = np.where((si == key[0]) & (prn == key[1]))[0]
        for seg in np.split(idx, np.where(np.diff(t[idx]) > 1.5 * np.median(np.diff(t[idx])))[0] + 1):
            if len(seg) >= min_len:
                out.append(seg)
    return out


def rsd(x):
    return 1.4826 * float(np.median(np.abs(x - np.median(x))))


def tau_1e(x, dt, max_s=120.0):
    x = x - x.mean(); n = len(x); v = float(np.dot(x, x)) / n
    for lag in range(1, min(n - 1, int(max_s / dt))):
        if np.dot(x[:-lag], x[lag:]) / (n - lag) / v < math.exp(-1):
            return lag * dt
    return float("nan")


def fit(cn0, y, w):
    """Least squares on log sigma: sigma^2 = a^2 + b^2 10^(-(cn0-35)/10)."""
    from scipy.optimize import least_squares
    f = lambda p: (0.5 * np.log(p[0] ** 2 + p[1] ** 2 * 10 ** (-(cn0 - 35) / 10)) - np.log(y)) * w
    return np.abs(least_squares(f, [1.0, 1.0]).x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", help="static raw capture (rig format, plain or .gz)")
    args = ap.parse_args()
    path = open_capture(args.capture)
    A, kind = residuals(path)
    dt = float(np.median(np.diff(np.unique(A[:, 0]))))
    arcs = arcs_of(A, int(10.0 / dt))
    print(f"{len(A)} residuals, {len(arcs)} arcs of >= 10 s at {1 / dt:.0f} Hz ({kind})")
    stats = []
    for seg in arcs:
        pr, rr = A[seg, 5], A[seg, 6]
        rr = rr[~np.isnan(rr)]
        stats.append(dict(cn0=float(np.median(A[seg, 3])), n=len(seg),
                          pr_w=rsd(np.diff(pr)) / math.sqrt(2), pr_c=float(np.sqrt(np.mean(pr ** 2))),
                          pr_tau=tau_1e(pr, dt), rr_w=rsd(np.diff(rr)) / math.sqrt(2) if len(rr) > 10 else np.nan,
                          rr_tau=tau_1e(rr, dt) if len(rr) > 10 else np.nan))
    cn0 = np.array([s["cn0"] for s in stats]); w = np.sqrt([s["n"] for s in stats])
    print("\n  C/N0    arcs  pr white m  pr correlated m  pr tau s   rr white m/s  rr tau s")
    for lo, hi in ((0, 30), (30, 35), (35, 40), (40, 45), (45, 99)):
        g = [s for s in stats if lo <= s["cn0"] < hi]
        if g:
            med = lambda k: float(np.nanmedian([s[k] for s in g]))
            print(f"  {lo:2d}-{hi:<3d} {len(g):5d}  {med('pr_w'):10.3f}  {med('pr_c'):15.2f}  {med('pr_tau'):8.1f}"
                  f"   {med('rr_w'):12.3f}  {med('rr_tau'):8.2f}")
    for name, key in (("PR_WHITE", "pr_w"), ("PR_CORR", "pr_c"), ("RR (white, incl. quantisation)", "rr_w")):
        y = np.array([s[key] for s in stats]); ok = ~np.isnan(y) & (y > 0)
        a, b = fit(cn0[ok], y[ok], w[ok])
        print(f"  fit {name:32s} a = {a:.3f}  b = {b:.3f}")
    # Doppler vs carrier: resolution and truncation bias, per satellite
    ser = collections.defaultdict(list)
    for _t, k, d in replay_source(path):
        if k != "bin" or not d or d[0] != 0xE5:
            continue
        tow = struct.unpack_from(">I", d, 5)[0] * 1e-3
        for j in range(d[13]):
            r = d[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) < 31 or (r[0] >> 4) != 0 or SKY_GNSS.get(r[0] & 0xF) is None:
                continue
            ind = struct.unpack_from(">H", r, 27)[0]
            if ind & 6 == 6 and not ind & 8:
                ser[(SKY_GNSS[r[0] & 0xF], r[1])].append(
                    (tow, struct.unpack_from(">d", r, 12)[0], struct.unpack_from(">f", r, 20)[0]))
    fracs, bias_pos, bias_neg = [], [], []
    for (sy, prn), v in ser.items():
        v = np.array(v); lam = C / SYS[sy]["freq"]
        t, cyc, dop = v.T
        ok = np.diff(t) < 1.5 * dt
        d = lam * np.diff(cyc) / np.diff(t) + lam * 0.5 * (dop[1:] + dop[:-1])      # carrier - Doppler rate
        fracs += list(np.abs(dop - np.round(dop)))
        (bias_pos if np.mean(dop) > 0 else bias_neg).append(float(np.mean(d[ok])))
    print(f"\n  Doppler: {100 * np.mean(np.array(fracs) < 1e-6):.0f} % of values are whole hertz; "
          f"carrier minus raw-Doppler range rate: {np.mean(bias_pos):+.3f} m/s for approaching, "
          f"{np.mean(bias_neg):+.3f} m/s for receding satellites (truncation toward zero is -+0.095)")


if __name__ == "__main__":
    main()
