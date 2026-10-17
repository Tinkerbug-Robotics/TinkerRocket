#!/usr/bin/env python3
"""Score raw_replay runs of a static or spin-rig V8/V9 + mosaic-G5 log (build unit E4, gate G3).

    score_static.py RUN.csv [--from-s T0] [--to-s T1] [--log OC_LOG] [--rig RIG.npz] [--events EVENTS]
                    [--pvt PVT.csv] [--ref NOFAULT.csv (--cut-from-s A --cut-to-s B | --corrupt-at-s T)]
                    [--json OUT.json]

Times are the CSV's t_s (flight time: MCU seconds plus the events file's t_base_s, the MCU clock itself on a log
with no flight). Every score is a distance or a statistic; nothing here prints or writes a position or a height.
Where the harness wrote RUN.csv.epochs.csv (a mosaic or cold-start replay: one row per epoch the filter closed,
whatever --every), the epoch counts come from it; otherwise from the CSV's last-epoch fields at its rows, which a
CSV written --every N can miss.

- health: rows, non-finite rows, unhealthy rows, and the filter's stops in [T0, T1] (the events JSON's: RawGnssShadow's
  own, with when and why; a stop on the pad restarts from the seed within the step, so the CSV's running column
  alone misses it) beside the whole run's; the events JSON's stage, feed and Doppler counts.
- velocity in [T0, T1]: mean and rms per axis (N, E, D) and the p95 of the speed. On a static log only a run with
  raw_replay --gnss-only-from-s scores the receiver: the pad's zero-velocity updates and the levelling hold the
  velocity otherwise (tools/raw_replay/README.md).
- epochs: satellites tracked, with an ephemeris, Dopplers and pseudoranges used and refused, and the measurement
  rows the update worked (used + refused), as CPU counts; with the epochs file, the longest any one satellite was
  refused running (each kind's: the EKF's own count of its refusals running, times that kind's period).
- --log: tilt and heading against the log's own flight filter (NonSensor q, the auto image's GpsInsEKF): tilt is
  the angle between the two filters' gravity directions in the body; heading is the twist about NED down of
  estimate * reference^-1 (scorelib.att_errors). Heading is reported as is and with the offset at T0 taken out
  (what the run did with the heading since, not where it started: a still receiver with the antenna on the IMU does
  not observe heading, and on the 10/16-10/17 logs no magnetometer sample passes the filter's gates).
- --events: tilt against the accelerometer's own gravity direction, its 1 s means (minus their direction is NED down
  in the body while the payload is still or turns about the vertical), for the run and for the log's filter. The
  accelerometer's bias is in it (0.05 m/s^2 is 0.3 deg): a reference beside GpsInsEKF's, which on a still pad has
  its own split of tilt and accelerometer bias.
- --rig: the spin rig's own bearing of board +Z (deg true; an npz with t_mcu_s and bearing_deg), the run's board +Z
  (body FRD -z) azimuth against it, as is and from T0.
- --pvt: the raw filter's height against the receiver's (harness --pvt-out: each fix's height above the ellipsoid),
  the difference of their means over [T0, T1] and its spread at each fix: distances only.
- clock jumps: each step of the receiver clock bias over 1e5 m (a whole millisecond is 299792 m) in [T0, T1]: the
  velocity step across it (the epoch before against the epoch after) beside the epoch-to-epoch spread elsewhere, the
  step of the 1 s means, and the refusals within 5 s after it -- with the epochs file, the longest any one satellite
  was refused; without it, an upper bound from the runs of epochs with any refusal (the pseudoranges' counted in
  their own 1 Hz epochs, not the Dopplers' between them).
- --ref with --cut-from-s/--cut-to-s or --corrupt-at-s: the velocity (and position) against the no-fault run 2 s
  after GNSS returns (the cut's end, or the epoch after the corrupted one), its 1 s mean from then, its largest
  difference over the 10 s after, the tilt difference, the pad ZUPTs taken around the fault (a ZUPT holds the
  velocity whatever GNSS does), the refusals around it, and for a corrupted epoch how many of its corrupted
  satellites the stage built and how many of those the filter refused (against the no-fault run's same epoch), or
  whether the vertical check dropped the epoch whole.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

C_MS = 299792458.0e-3          # one millisecond of receiver clock, m
G0 = 9.80665
COLS = ("t_mcu_s", "t_s", "running", "finite", "healthy", "gnss_aided", "qw", "qx", "qy", "qz", "h_m", "n_m",
        "e_m", "d_m", "vn", "ve", "vd", "sig_tilt_n_deg", "sig_tilt_e_deg", "sig_hdg_deg", "wb_x_dps", "wb_y_dps",
        "wb_z_dps", "clk_b_m", "clk_d_mps", "epochs", "used_pr", "used_rr", "rej_pr", "rej_rr", "age_ms",
        "eph_sats", "tracked", "levelled", "zupts", "mag_fused")
EPCOLS = ("t_s", "t_close_s", "tow_ms", "used_pr", "rej_pr", "used_rr", "rej_rr", "nis_max", "rej_run_pr_s",
          "rej_run_rr_s", "corrupt_rr", "corrupt_pr")


def load_csv(path):
    with open(path) as f:
        names = f.readline().strip().split(",")
    idx = [names.index(c) for c in COLS]
    d = np.loadtxt(path, delimiter=",", skiprows=1, usecols=idx, ndmin=2)
    return {c: d[:, i] for i, c in enumerate(COLS)}


def load_epochs(csv):
    """The harness's epochs file beside a run's CSV (one row per epoch the filter closed), or None."""
    path = csv + ".epochs.csv"
    if not os.path.exists(path):
        return None
    with open(path) as f:
        names = f.readline().strip().split(",")
    d = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if d.size == 0:
        return {c: np.zeros(0) for c in EPCOLS}
    return {c: d[:, names.index(c)] for c in EPCOLS}


def pct(x, p):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if x.size else None


def dist(x):
    """min, median, p95, max of a count."""
    x = np.asarray(x, float)
    if x.size == 0:
        return None
    return dict(min=float(x.min()), median=float(np.median(x)), p95=float(np.percentile(x, 95)), max=float(x.max()))


def q_at(t_src, q_src, t):
    """The quaternions q_src (n x 4, one sign) at the times t: normalized-linear between the neighbours."""
    # one hemisphere, so neighbours interpolate: each record's sign against the one before, carried along
    flip = np.where(np.sum(q_src[1:] * q_src[:-1], axis=1) < 0.0, -1.0, 1.0)
    q_src = q_src * np.cumprod(np.r_[1.0, flip])[:, None]
    i = np.clip(np.searchsorted(t_src, t), 1, len(t_src) - 1)
    f = np.clip((t - t_src[i - 1]) / np.maximum(t_src[i] - t_src[i - 1], 1e-9), 0.0, 1.0)[:, None]
    return S.qnorm(q_src[i - 1] * (1 - f) + q_src[i] * f)


def down_in_body(q):
    """NED down seen in the body (the third row of body -> NED)."""
    w, x, y, z = q.T
    return np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z], axis=1)


def tilt_deg(qe, qr):
    c = np.sum(down_in_body(qe) * down_in_body(qr), axis=1)
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def tilt_vs_down_deg(q, down_b):
    """The angle between q's gravity direction in the body and a measured one (unit, body)."""
    c = np.sum(down_in_body(q) * down_b, axis=1)
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def board_z_bearing_deg(q):
    """Azimuth (deg true, clockwise from north) of board +Z: body FRD -z, horizontal when the board stands nose-up."""
    w, x, y, z = q.T
    n = -2 * (x * z + w * y)
    e = -2 * (y * z - w * x)
    return np.degrees(np.arctan2(e, n))


def ang_stats(e, signed=True):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return None
    a = np.abs(e)
    out = dict(p50=float(np.median(a)), p95=float(np.percentile(a, 95)), max=float(a.max()))
    if signed:
        out["median_signed"] = float(np.median(e))
    return out


def log_attitude(log_path, cache_dir):
    """The log's own flight filter (NonSensor: q, the ENU velocity), on the MCU clock the events file uses
    (extract.py's unwrap from the first IMU stamp). Cached beside the runs: a 250 MB log takes a minute to parse."""
    os.makedirs(cache_dir, exist_ok=True)
    cache = os.path.join(cache_dir, os.path.basename(log_path) + ".ns.npz")
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(log_path):
        z = np.load(cache)
        return z["t"], z["q"], z["v_enu"]
    sys.path.insert(0, os.path.join(S.WT, "Data_Analysis"))
    import plot_flight_data_mini as P  # noqa: E402
    import extract as X  # noqa: E402
    recs, _s, _c = P.parse_binary_file(log_path)
    t_ref = int(recs["ISM6HG256"][0]["time_us"])
    ns = recs["NonSensor"]
    t = X.unwrap([r["time_us"] for r in ns], t_ref) * 1e-6
    q = np.array([[r["q0"], r["q1"], r["q2"], r["q3"]] for r in ns], float)
    v = np.array([[r["e_vel"], r["n_vel"], r["u_vel"]] for r in ns], float)
    ok = np.abs(np.linalg.norm(q, axis=1) - 1.0) < 0.01
    t, q, v = t[ok], S.qnorm(q[ok]), v[ok]
    o = np.argsort(t, kind="stable")
    np.savez(cache, t=t[o], q=q[o], v_enu=v[o])
    return t[o], q[o], v[o]


def accel_means(events_path, cache_dir, bin_s=1.0):
    """The IMU ticks' specific force (the events file's I lines: body FRD, m/s^2, as the filter takes them) in bins of
    bin_s on the MCU clock: each bin's middle (s), mean (n x 3) and tick count. Cached: a rig log's events file is
    380 MB."""
    os.makedirs(cache_dir, exist_ok=True)
    cache = os.path.join(cache_dir, f"{os.path.basename(events_path)}.acc{bin_s:g}.npz")
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(events_path):
        z = np.load(cache)
        return z["t"], z["f"], z["n"]
    t, f = [], []
    with open(events_path) as fh:
        for line in fh:
            if line.startswith("I "):
                p = line.split()
                t.append(int(p[1]))
                f.append((float(p[2]), float(p[3]), float(p[4])))
    t = np.asarray(t, float) * 1e-6
    f = np.asarray(f, float).reshape(-1, 3)
    if t.size == 0:
        z = dict(t=np.zeros(0), f=np.zeros((0, 3)), n=np.zeros(0))
    else:
        k = np.floor((t - t[0]) / bin_s).astype(np.int64)
        n = np.bincount(k).astype(float)
        sums = np.stack([np.bincount(k, weights=f[:, i], minlength=n.size) for i in range(3)], axis=1)
        keep = n > 0
        z = dict(t=(t[0] + (np.arange(n.size) + 0.5) * bin_s)[keep], f=sums[keep] / n[keep, None], n=n[keep])
    np.savez(cache, **z)
    return z["t"], z["f"], z["n"]


def epoch_rows(d, sel):
    """One row per closed epoch: the last CSV row before the epoch counter moves on (its 'last epoch' fields)."""
    ep = d["epochs"]
    k = np.flatnonzero(sel)
    if k.size == 0:
        return k
    last = np.r_[ep[k][1:] != ep[k][:-1], True]
    k = k[last & (ep[k] > 0)]
    return k


def longest_run(flags):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best


def score(a):
    d = load_csv(a.csv)
    E = load_epochs(a.csv)
    t = d["t_s"]
    run = d["running"] > 0
    out = dict(csv=os.path.basename(a.csv))
    # ---- health
    starts = int(np.sum(np.diff(run.astype(int)) > 0) + (1 if run.size and run[0] else 0))
    dropouts = int(np.sum(np.diff(run.astype(int)) < 0))
    out["health"] = dict(rows=int(t.size), running_rows=int(run.sum()), starts=starts, running_dropouts=dropouts,
                         first_running_t=float(t[run][0]) if run.any() else None,
                         gnss_aided_rows=int(np.sum(d["gnss_aided"][run] > 0)),
                         mag_fused=int(d["mag_fused"][-1]) if t.size else 0)
    stop_at = None
    j = {}
    ej = a.csv + ".events.json"
    if os.path.exists(ej):
        j = json.load(open(ej))
        if "filter" in j:
            # the filter's own count: a stop on the pad restarts from the seed within the step, which the running
            # column never shows
            stop_at = [list(x) for x in j["filter"]["stop_at"]]
            out["health"]["stops_run"] = int(j["filter"]["stops"])
            out["health"]["stop_at_run"] = stop_at
        keep = ("stage", "sbf", "doppler", "mag", "eph_from", "time_tag", "pad_zupts", "landed_zupts", "cut",
                "corrupted_sats", "faults_clock", "sat_cap", "fuse_ms", "unaided", "vert", "vert_latched", "filter",
                "gnss_only", "corrupt")
        out["events"] = {k: j[k] for k in keep if k in j}
        fw = j.get("fw") or {}
        out["events"]["levels"] = fw.get("levels")
        out["events"]["levels_held"] = fw.get("levels_held")
    t0 = a.from_s if a.from_s is not None else (out["health"]["first_running_t"] or t[0])
    t1 = a.to_s if a.to_s is not None else t[-1]
    win = run & (t >= t0) & (t <= t1)
    out["window"] = dict(from_s=float(t0), to_s=float(t1), rows=int(win.sum()))
    h = out["health"]
    h["nonfinite_rows"] = int(np.sum(d["finite"][win] == 0))
    h["unhealthy_rows"] = int(np.sum(d["healthy"][win] == 0))
    h["nonfinite_rows_run"] = int(np.sum(d["finite"][run] == 0))
    h["unhealthy_rows_run"] = int(np.sum(d["healthy"][run] == 0))
    # the stops G3 counts: those in the scored window (a stop after it -- the payload carried indoors, say -- is the
    # whole run's, beside it)
    if stop_at is not None:
        h["stop_at"] = [s for s in stop_at if t0 <= s[0] <= t1]
        h["stops"] = len(h["stop_at"])
    else:
        k = np.flatnonzero(np.diff(run.astype(int)) < 0)
        h["stops"] = int(np.sum((t[k] >= t0) & (t[k] <= t1)))
        h["stops_run"] = dropouts
    # ---- velocity
    v = np.stack([d["vn"], d["ve"], d["vd"]], axis=1)[win]
    if v.size:
        out["velocity"] = dict(mean=[float(x) for x in v.mean(axis=0)],
                               rms=[float(x) for x in np.sqrt(np.mean(v * v, axis=0))],
                               std=[float(x) for x in v.std(axis=0)],
                               speed_p95=pct(np.linalg.norm(v, axis=1), 95),
                               speed_max=float(np.linalg.norm(v, axis=1).max()))
    # ---- epochs: satellites and measurement rows (CPU), each satellite's refusals
    ke = epoch_rows(d, win)
    if ke.size:
        out["epochs"] = dict(n_rows=int(ke.size), tracked=dist(d["tracked"][ke]), eph_sats=dist(d["eph_sats"][ke]),
                             age_ms=dist(d["age_ms"][ke]))
    if E is not None:
        se = (E["t_s"] >= t0) & (E["t_s"] <= t1)
        src = {k: E[k][se] for k in ("used_pr", "rej_pr", "used_rr", "rej_rr")}
        out.setdefault("epochs", {})["source"] = "epochs file"
        out["epochs"]["sat_refused_longest_s"] = dict(
            pr=float(np.max(E["rej_run_pr_s"][se])) if se.any() else 0.0,
            rr=float(np.max(E["rej_run_rr_s"][se])) if se.any() else 0.0)
    elif ke.size:
        src = {k: d[k][ke] for k in ("used_pr", "rej_pr", "used_rr", "rej_rr")}
        out["epochs"]["source"] = "CSV rows"
    else:
        src = None
    if src is not None and src["used_rr"].size:
        rows = src["used_rr"] + src["rej_rr"] + src["used_pr"] + src["rej_pr"]
        pr_ep = (src["used_pr"] + src["rej_pr"]) > 0
        out["epochs"].update(n=int(rows.size), used_rr=dist(src["used_rr"]), rej_rr=dist(src["rej_rr"]),
                             used_pr_on_pr_epochs=dist(src["used_pr"][pr_ep]) if pr_ep.any() else None,
                             pr_epochs=int(pr_ep.sum()), rows=dist(rows),
                             rows_on_pr_epochs=dist(rows[pr_ep]) if pr_ep.any() else None,
                             epochs_with_refusal=int(np.sum((src["rej_rr"] + src["rej_pr"]) > 0)),
                             refused_rr=int(np.sum(src["rej_rr"])), refused_pr=int(np.sum(src["rej_pr"])))
    # ---- attitude
    q = S.qnorm(np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], axis=1))
    out["sigma_end"] = dict(tilt_n_deg=float(d["sig_tilt_n_deg"][win][-1]) if win.any() else None,
                            tilt_e_deg=float(d["sig_tilt_e_deg"][win][-1]) if win.any() else None,
                            heading_deg=float(d["sig_hdg_deg"][win][-1]) if win.any() else None)
    out["gyro_bias_dps"] = dict(start=[float(d[k][run][0]) for k in ("wb_x_dps", "wb_y_dps", "wb_z_dps")],
                                end=[float(d[k][win][-1]) for k in ("wb_x_dps", "wb_y_dps", "wb_z_dps")]) \
        if win.any() else None
    tl = ql = None
    if a.log:
        tl, ql, vl = log_attitude(a.log, a.cache or os.path.dirname(os.path.abspath(a.csv)))
        tm = d["t_mcu_s"][win]
        inside = (tm >= tl[0]) & (tm <= tl[-1])
        if inside.any():
            qe = q[win][inside]
            qr = q_at(tl, ql, tm[inside])
            _tot, _nose, _roll, hdg, _pitch = S.att_errors(qe, qr)
            hdg_rel = S.wrap_deg(hdg - hdg[0])
            out["vs_log_filter"] = dict(n=int(inside.sum()), tilt=ang_stats(tilt_deg(qe, qr), signed=False),
                                        heading=ang_stats(hdg), heading_from_t0=ang_stats(hdg_rel),
                                        heading_detrended=ang_stats(S.wrap_deg(hdg - np.median(hdg))))
            # the log filter's own velocity on the same window, for scale (ENU -> NED)
            vi = np.stack([np.interp(tm[inside], tl, vl[:, k]) for k in (1, 0, 2)], axis=1) * np.array([1, 1, -1])
            out["log_filter_velocity_rms"] = [float(x) for x in np.sqrt(np.mean(vi * vi, axis=0))]
    if getattr(a, "events", None):
        # the accelerometer's own gravity direction, its 1 s means: bins whose size is gravity's within 1 % (still,
        # or turning slowly about the vertical), inside the window
        tb, fb, nb = accel_means(a.events, a.cache or os.path.dirname(os.path.abspath(a.csv)))
        tm_run = d["t_mcu_s"][win]
        if tm_run.size and tb.size:
            nf = np.linalg.norm(fb, axis=1)
            inb = (tb >= tm_run[0]) & (tb <= tm_run[-1])
            ok = inb & (np.abs(nf - G0) < 0.01 * G0)
            if ok.any():
                down_b = -fb[ok] / nf[ok, None]
                qr = q_at(d["t_mcu_s"][run], q[run], tb[ok])
                va = dict(bins=int(ok.sum()), bins_left_out=int(np.sum(inb & ~ok)),
                          tilt=ang_stats(tilt_vs_down_deg(qr, down_b), signed=False))
                if tl is not None:
                    li = (tb[ok] >= tl[0]) & (tb[ok] <= tl[-1])
                    if li.any():
                        va["log_filter_tilt"] = ang_stats(tilt_vs_down_deg(q_at(tl, ql, tb[ok][li]), down_b[li]),
                                                          signed=False)
                out["vs_accel"] = va
    if a.rig:
        z = np.load(a.rig)
        tm = d["t_mcu_s"][win]
        inside = (tm >= z["t_mcu_s"][0]) & (tm <= z["t_mcu_s"][-1])
        if inside.any():
            tb = np.degrees(np.unwrap(np.radians(z["bearing_deg"])))
            truth = np.interp(tm[inside], z["t_mcu_s"], tb)
            e = S.wrap_deg(board_z_bearing_deg(q[win][inside]) - truth)
            out["vs_rig"] = dict(n=int(inside.sum()), heading=ang_stats(e),
                                 heading_from_t0=ang_stats(S.wrap_deg(e - e[0])),
                                 heading_detrended=ang_stats(S.wrap_deg(e - np.median(e))))
    # ---- the pad's height against the receiver's (distances only)
    if getattr(a, "pvt", None):
        p = np.loadtxt(a.pvt, delimiter=",", skiprows=1, ndmin=2)
        with open(a.pvt) as f:
            pn = f.readline().strip().split(",")
        tp, hp = p[:, pn.index("t_s")], p[:, pn.index("h_ell_m")]
        sp = (tp >= t0) & (tp <= t1)
        hr = d["h_m"][win]
        if sp.any() and hr.size:
            at_fix = np.interp(tp[sp], t[win], hr) - hp[sp]
            out["pad_height"] = dict(raw_minus_pvt_means_m=float(np.mean(hr) - np.mean(hp[sp])),
                                     raw_sd_m=float(np.std(hr)), pvt_sd_m=float(np.std(hp[sp])), fixes=int(sp.sum()),
                                     at_fix_abs=ang_stats(at_fix))
    # ---- clock jumps
    clk = d["clk_b_m"]
    jumps = []
    kj = np.flatnonzero(win[1:] & win[:-1] & (np.abs(np.diff(clk)) > 1e5)) + 1
    if ke.size > 2:
        ve = np.stack([d["vn"], d["ve"], d["vd"]], axis=1)[ke]
        dv_all = np.linalg.norm(np.diff(ve, axis=0), axis=1)
    for k in kj:
        tj = float(t[k])
        i = int(np.searchsorted(t[ke], tj))          # the first closed epoch at or after the step
        rec = dict(t_s=tj, step_ms=float((clk[k] - clk[k - 1]) / C_MS))
        if 1 <= i < ke.size:
            vb, va = ve[i - 1], ve[i]
            rec["dv_across"] = float(np.linalg.norm(va - vb))
            rec["dv_across_axes"] = [float(x) for x in (va - vb)]
            rec["dv_epoch_p95_elsewhere"] = pct(dv_all, 95)
            rec["dv_epoch_max_elsewhere"] = float(np.max(dv_all))
            # the mean over the 1 s after against the 1 s before: a step, not the epoch-to-epoch noise; beside the
            # same difference at every other epoch (its spread is what a step must stand out from)
            n1 = 10
            if n1 <= i <= ke.size - n1:
                cs = np.vstack([np.zeros(3), np.cumsum(ve, axis=0)])
                jj = np.arange(n1, ke.size - n1 + 1)
                diffs = (cs[jj + n1] - cs[jj]) / n1 - (cs[jj] - cs[jj - n1]) / n1
                dm = diffs[i - n1]
                rec["dv_1s_means"] = float(np.linalg.norm(dm))
                rec["dv_1s_means_axes"] = [float(x) for x in dm]
                other = np.abs(jj - i) > 2 * n1
                rec["dv_1s_means_p95_elsewhere"] = pct(np.linalg.norm(diffs[other], axis=1), 95)
                rec["dv_1s_means_max_elsewhere"] = float(np.max(np.linalg.norm(diffs[other], axis=1)))
        # the refusals in the 5 s after it
        if E is not None:
            ne = (E["t_s"] >= tj - 0.2) & (E["t_s"] <= tj + 5.0)
            pr_ep = ne & ((E["used_pr"] + E["rej_pr"]) > 0)
            rec["refusals_from"] = "epochs file"
            rec["epochs_5s"] = int(ne.sum())
            rec["epochs_with_refusal_5s"] = int(np.sum(ne & ((E["rej_rr"] + E["rej_pr"]) > 0)))
            rec["refused_rr_5s"] = int(np.sum(E["rej_rr"][ne]))
            rec["refused_pr_5s"] = int(np.sum(E["rej_pr"][ne]))
            rec["used_rr_min_5s"] = float(np.min(E["used_rr"][ne])) if ne.any() else None
            rec["pr_epochs_5s"] = [(float(E["t_s"][jx]), int(E["used_pr"][jx]), int(E["rej_pr"][jx]))
                                   for jx in np.flatnonzero(pr_ep)]
            # each satellite's own: the longest it was refused running, in the 5 s after (1 s is the bar)
            rec["sat_refused_longest_s"] = dict(pr=float(np.max(E["rej_run_pr_s"][ne])) if ne.any() else 0.0,
                                                rr=float(np.max(E["rej_run_rr_s"][ne])) if ne.any() else 0.0)
        elif ke.size:
            near = (t[ke] >= tj - 0.2) & (t[ke] <= tj + 5.0)
            pr_ep = near & ((d["used_pr"][ke] + d["rej_pr"][ke]) > 0)
            rec["refusals_from"] = "CSV rows"
            rec["epochs_with_refusal_5s"] = int(np.sum(near & ((d["rej_rr"][ke] + d["rej_pr"][ke]) > 0)))
            rec["refused_rr_5s"] = int(np.sum(d["rej_rr"][ke][near]))
            rec["refused_pr_5s"] = int(np.sum(d["rej_pr"][ke][near]))
            rec["pr_epochs_5s"] = [(float(t[ke][jx]), int(d["used_pr"][ke][jx]), int(d["rej_pr"][ke][jx]))
                                   for jx in np.flatnonzero(pr_ep)]
            # without the epochs file, upper bounds on any one satellite's refusal: the runs of epochs with any
            # refusal of each kind, the pseudoranges' in their own 1 Hz epochs (a code refused at two running is
            # refused for 2 s; the Dopplers' epochs between them would hide that)
            rec["sat_refused_longest_s_bound"] = dict(
                pr=float(longest_run(d["rej_pr"][ke][pr_ep] > 0)) * 1.0,
                rr=float(longest_run(d["rej_rr"][ke][near] > 0)) * a.fuse_s)
        jumps.append(rec)
    out["clock_jumps"] = jumps
    # ---- recovery against the no-fault run
    if a.ref:
        r = load_csv(a.ref)
        if a.cut_to_s is not None:
            t_ret = a.cut_to_s
            fault = (a.cut_from_s, a.cut_to_s)
        else:
            t_ret = a.corrupt_at_s + a.fuse_s
            fault = (a.corrupt_at_s - a.fuse_s, t_ret)
            # the corrupted epoch as the harness found it (--corrupt-pr-epoch takes the next whole second)
            c = j.get("corrupt") or {}
            if c.get("t_s") is not None:
                t_ret = float(c["t_s"]) + a.fuse_s
                fault = (float(c["t_s"]) - a.fuse_s, t_ret)
        rr = np.flatnonzero(r["running"] > 0)
        kr = np.flatnonzero(run)

        def pair(t_from, t_to):
            """This run's rows in [t_from, t_to] and the no-fault run's row at the same filter step (the same MCU
            time: both runs step on the same IMU ticks, so a CSV written --every N still pairs exactly)."""
            ja = kr[(t[kr] >= t_from) & (t[kr] <= t_to)]
            i = np.clip(np.searchsorted(r["t_mcu_s"][rr], d["t_mcu_s"][ja]), 1, rr.size - 1)
            near = np.where(np.abs(r["t_mcu_s"][rr][i - 1] - d["t_mcu_s"][ja]) <
                            np.abs(r["t_mcu_s"][rr][i] - d["t_mcu_s"][ja]), i - 1, i)
            jb = rr[near]
            ok = np.abs(r["t_mcu_s"][jb] - d["t_mcu_s"][ja]) < 1.5e-3
            return ja[ok], jb[ok]

        def vel(dd, jx):
            return np.stack([dd["vn"][jx], dd["ve"][jx], dd["vd"][jx]], axis=-1)

        def pos(dd, jx):
            return np.stack([dd["n_m"][jx], dd["e_m"][jx], dd["d_m"][jx]], axis=-1)

        qa = S.qnorm(np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], axis=1))
        qb = S.qnorm(np.stack([r["qw"], r["qx"], r["qy"], r["qz"]], axis=1))
        rec = dict(fault=list(fault), t_return=float(t_ret))
        ja, jb = pair(t_ret + 2.0, t_ret + 2.2)
        if ja.size:
            dv = vel(d, ja[0]) - vel(r, jb[0])
            rec.update(t_at_2s=float(t[ja[0]]), dv_at_2s=[float(x) for x in dv], dv_at_2s_norm=float(np.linalg.norm(dv)),
                       dpos_at_2s_m=float(np.linalg.norm(pos(d, ja[0]) - pos(r, jb[0]))),
                       tilt_vs_ref_at_2s_deg=float(tilt_deg(qa[ja[:1]], qb[jb[:1]])[0]))
        # the same over the second from +2 s, averaged: each run's epoch-to-epoch noise (0.1 m/s p95 on these logs)
        # is not the difference the fault left
        ja, jb = pair(t_ret + 2.0, t_ret + 3.0)
        if ja.size:
            dm = np.mean(vel(d, ja) - vel(r, jb), axis=0)
            rec["dv_1s_mean_from_2s"] = [float(x) for x in dm]
            rec["dv_1s_mean_from_2s_norm"] = float(np.linalg.norm(dm))
        ja, jb = pair(t_ret + 2.0, t_ret + 10.0)
        if ja.size:
            rec["dv_max_2_10s"] = float(np.max(np.linalg.norm(vel(d, ja) - vel(r, jb), axis=1)))
            rec["dpos_max_2_10s_m"] = float(np.max(np.linalg.norm(pos(d, ja) - pos(r, jb), axis=1)))
            rec["tilt_vs_ref_max_2_10s_deg"] = float(np.max(tilt_deg(qa[ja], qb[jb])))
        ja, jb = pair(fault[0], t_ret + 2.0)
        if ja.size:
            rec["dv_max_during"] = float(np.max(np.linalg.norm(vel(d, ja) - vel(r, jb), axis=1)))
            rec["tilt_vs_ref_max_during_deg"] = float(np.max(tilt_deg(qa[ja], qb[jb])))
        # the pad ZUPTs from 1 s before the fault to 6 s after GNSS returns: a ZUPT holds the velocity whatever GNSS
        # does, so a fault among them scores the ZUPT
        zr = kr[(t[kr] >= fault[0] - 1.0) & (t[kr] <= t_ret + 6.0)]
        rec["zupts_around"] = int(d["zupts"][zr[-1]] - d["zupts"][zr[0]]) if zr.size > 1 else 0
        # the run's own refusals around the fault (a corrupted epoch should be refused, not fused), from every epoch
        if E is not None:
            kf = (E["t_s"] >= fault[0] - 0.2) & (E["t_s"] <= t_ret + 2.0)
            rec["refusals_from"] = "epochs file"
            rec["refused_rr_around"] = int(np.sum(E["rej_rr"][kf]))
            rec["refused_pr_around"] = int(np.sum(E["rej_pr"][kf]))
        else:
            kf = epoch_rows(d, run & (t >= fault[0] - 0.2) & (t <= t_ret + 2.0))
            rec["refusals_from"] = "CSV rows"
            rec["refused_rr_around"] = int(np.sum(d["rej_rr"][kf])) if kf.size else 0
            rec["refused_pr_around"] = int(np.sum(d["rej_pr"][kf])) if kf.size else 0
        # the corrupted epoch: its satellites as injected, as the stage built them, and as the filter took them
        # (refused = this run's refusals at that epoch less the no-fault run's at the same epoch)
        c = j.get("corrupt")
        if c is not None:
            cr = dict(injected=len(c.get("sats", [])), built_rr=c.get("built_rr"), built_pr=c.get("built_pr"),
                      pr_epoch_asked=bool(c.get("pr_epoch_asked")))
            if c.get("sats"):
                cr["size_rr_mps"] = [float(min(abs(s[2]) for s in c["sats"])), float(max(abs(s[2]) for s in c["sats"]))]
                cr["size_pr_m"] = [float(min(abs(s[3]) for s in c["sats"])), float(max(abs(s[3]) for s in c["sats"]))]
            Er = load_epochs(a.ref)
            if E is not None and c.get("tow_ms", -1) >= 0:
                k = np.flatnonzero(E["tow_ms"] == c["tow_ms"])
                if k.size:
                    k = k[0]
                    cr["closed"] = True
                    cr["pr_fused_epoch"] = bool(E["used_pr"][k] + E["rej_pr"][k] > 0)
                    base_rr = base_pr = 0
                    if Er is not None:
                        kb = np.flatnonzero(Er["tow_ms"] == c["tow_ms"])
                        if kb.size:
                            base_rr, base_pr = int(Er["rej_rr"][kb[0]]), int(Er["rej_pr"][kb[0]])
                    cr["refused_rr_ref"], cr["refused_pr_ref"] = base_rr, base_pr
                    cr["refused_rr"] = int(E["rej_rr"][k]) - base_rr
                    cr["refused_pr"] = int(E["rej_pr"][k]) - base_pr
                    # how many corrupted ones were fused, where the no-fault run refused none of that kind at that
                    # epoch (else its own refusals blur the count: None)
                    if c.get("built_rr") is not None and c["built_rr"] >= 0:
                        cr["fused_rr"] = int(c["built_rr"]) - cr["refused_rr"] if base_rr == 0 else None
                        cr["fused_pr"] = ((int(c["built_pr"]) - cr["refused_pr"]) if base_pr == 0 else None) \
                            if cr["pr_fused_epoch"] else 0
                else:
                    # never closed: dropped whole (the vertical check's fit of its Dopplers, say), or stale
                    cr["closed"] = False
                    vd = (j.get("vert") or {}).get("dropped_t") or []
                    cr["dropped_by_vertical_check"] = any(abs(x - float(c["t_s"])) < 0.06 for x in vd)
            rec["corrupt"] = cr
        out["recovery"] = rec
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--from-s", type=float)
    ap.add_argument("--to-s", type=float)
    ap.add_argument("--log", help="the OC log (.bin) the run came from: tilt and heading against its NonSensor q")
    ap.add_argument("--cache", help="where the log's NonSensor cache goes (default: beside the CSV)")
    ap.add_argument("--events", help="the events file the run replayed: tilt against the accelerometer's 1 s means")
    ap.add_argument("--rig", help="npz: t_mcu_s, bearing_deg (the spin rig's bearing of board +Z, deg true)")
    ap.add_argument("--pvt", help="the harness's --pvt-out file: the pad height against the receiver's")
    ap.add_argument("--ref", help="the no-fault run's CSV, for --cut-* / --corrupt-at-s")
    ap.add_argument("--cut-from-s", type=float)
    ap.add_argument("--cut-to-s", type=float)
    ap.add_argument("--corrupt-at-s", type=float)
    ap.add_argument("--fuse-s", type=float, default=0.1, help="the fused epoch period (the epoch after a corrupt one)")
    ap.add_argument("--json", help="write the scores here (default: stdout)")
    a = ap.parse_args()
    if a.ref and a.cut_to_s is None and a.corrupt_at_s is None:
        ap.error("--ref needs --cut-from-s/--cut-to-s or --corrupt-at-s")
    out = score(a)
    s = json.dumps(out, indent=1)
    if a.json:
        with open(a.json, "w") as f:
            f.write(s + "\n")
    else:
        print(s)
    h = out["health"]
    v = out.get("velocity") or {}
    msg = (f"{out['csv']}: stops {h['stops']} (run {h.get('stops_run', h['stops'])}), "
           f"non-finite {h['nonfinite_rows']}, unhealthy {h['unhealthy_rows']}; "
           f"velocity rms {'/'.join(f'{x:.3f}' for x in v.get('rms', []))} m/s")
    if "vs_log_filter" in out:
        L = out["vs_log_filter"]
        msg += f"; vs log filter tilt p95 {L['tilt']['p95']:.2f}, heading-from-T0 p95 {L['heading_from_t0']['p95']:.2f}"
    if "vs_accel" in out:
        msg += f"; vs accelerometer tilt p95 {out['vs_accel']['tilt']['p95']:.2f}"
    if "recovery" in out:
        msg += f"; recovery |dv| at +2 s {out['recovery'].get('dv_at_2s_norm', float('nan')):.3f} m/s"
    print(msg, file=sys.stderr)


if __name__ == "__main__":
    main()
