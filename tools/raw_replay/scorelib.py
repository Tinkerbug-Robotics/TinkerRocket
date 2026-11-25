"""Score a raw-GNSS navigation run against the rig flight's truth: the harness's CSV (raw_replay) or an OC log's
own on-board records (0x96), with the same metrics on the same phases.

Truth: the sensors CSV sim_flight_3d.py wrote (1 kHz; NED at the pad; qw..qz body FRD -> NED; phase column).
Phases: boost (T+0 to burnout), coast (burnout to apogee), descent (apogee to the end of the run, at most the
landing). Each metric is sampled every 0.1 s of flight time: the run's sample nearest that time, the truth at the
sample's own time.
"""
import gzip
import math
import os
import sys

import numpy as np

def _repo():
    """The repository: the one this script sits in (tools/raw_replay), else $RAW_REPLAY_WT."""
    up = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    if os.path.isdir(os.path.join(up, "Data_Analysis")):
        return up
    return os.environ.get("RAW_REPLAY_WT", up)


WT = _repo()
TRUTH_CSV = os.path.join(os.environ.get("COCOM_SDR", os.path.join(WT, "gnss", "cocom-rig", "sdr")),
                         "scenarios", "sim_rp5_0829_sensors.csv.gz")
PAD = (39.4683565, -75.29286495, -0.712)     # the scenario's origin: lat, lon (deg), ellipsoidal height (m)
A, E2 = 6378137.0, 0.0066943799901
PITCH_VOTE_DEG = -5.0                        # TR_KinematicChecks: pitch_rad < -0.087


# ------------------------------------------------------------------ quaternions (scalar first, body FRD -> NED)

def qnorm(q):
    return q / np.linalg.norm(q, axis=1)[:, None]


def qmul(a, b):
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=1)


def qconj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def body_x(q):
    """The nose (body x) in NED."""
    w, x, y, z = q.T
    return np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)], axis=1)


def pitch_deg(q):
    """Nose elevation (GpsInsEKF::Quat2Euler's pitch): asin(2 (q0 q2 - q3 q1))."""
    w, x, y, z = q.T
    return np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))


def wrap_deg(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def att_errors(qe, qt):
    """Per sample: total error rotation, nose-pointing error, roll error about the nose, heading error (twist about
    NED down), pitch error (deg; the last three signed)."""
    qe, qt = qnorm(qe), qnorm(qt)
    # body-frame error: truth^-1 * estimate
    eb = qmul(qconj(qt), qe)
    eb *= np.sign(eb[:, :1] + 1e-30)
    total = np.degrees(2.0 * np.arccos(np.clip(np.abs(eb[:, 0]), 0.0, 1.0)))
    xe, xt = body_x(qe), body_x(qt)
    nose = np.degrees(np.arccos(np.clip(np.sum(xe * xt, axis=1), -1.0, 1.0)))
    roll = wrap_deg(np.degrees(2.0 * np.arctan2(eb[:, 1], eb[:, 0])))   # twist about body x
    # heading: twist about NED down of estimate * truth^-1 (score_heading.twist_deg)
    a, b = qe, qt
    r0 = a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1] + a[:, 2] * b[:, 2] + a[:, 3] * b[:, 3]
    r3 = -a[:, 0] * b[:, 3] - a[:, 1] * b[:, 2] + a[:, 2] * b[:, 1] + a[:, 3] * b[:, 0]
    heading = wrap_deg(np.degrees(2.0 * np.arctan2(r3, r0)))
    pitch = pitch_deg(qe) - pitch_deg(qt)
    return total, nose, roll, heading, pitch


# ------------------------------------------------------------------ truth

class Truth:
    def __init__(self, path=TRUTH_CSV):
        lines = [ln for ln in gzip.open(path, "rt") if not ln.startswith("#")]
        names = lines[0].strip().split(",")
        cols = ("t_s", "pn_m", "pe_m", "pd_m", "vn_mps", "ve_mps", "vd_mps", "phase", "qw", "qx", "qy", "qz")
        d = np.loadtxt(lines[1:], delimiter=",", usecols=[names.index(c) for c in cols])
        self.t = d[:, 0]
        self.pos = d[:, 1:4]
        self.vel = d[:, 4:7]
        self.phase = d[:, 7].astype(int)
        self.q = qnorm(d[:, 8:12])
        self.burnout = float(self.t[np.argmax(self.phase >= 3)])
        self.apogee = float(self.t[int(np.argmin(self.pos[:, 2]))])
        self.landed = float(self.t[np.argmax(self.phase >= 6)])
        self.end = float(self.t[-1])
        p = pitch_deg(self.q)
        k = np.flatnonzero((self.t > self.burnout) & (p < PITCH_VOTE_DEG))
        self.pitch_vote = float(self.t[k[0]]) if k.size else None

    def at(self, t):
        """pos, vel, q at the times t (linear; the quaternion normalized-linear, 1 kHz)."""
        t = np.asarray(t, float)
        pos = np.stack([np.interp(t, self.t, self.pos[:, i]) for i in range(3)], axis=1)
        vel = np.stack([np.interp(t, self.t, self.vel[:, i]) for i in range(3)], axis=1)
        i = np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1)
        f = ((t - self.t[i - 1]) / (self.t[i] - self.t[i - 1]))[:, None]
        q0, q1 = self.q[i - 1], self.q[i]
        q1 = q1 * np.sign(np.sum(q0 * q1, axis=1))[:, None]
        return pos, vel, qnorm(q0 * (1 - f) + q1 * f)

    def phases(self, t_end):
        end = min(t_end, self.landed)
        return [("boost", 0.0, self.burnout), ("coast", self.burnout, self.apogee), ("descent", self.apogee, end)]


def ned(lat_deg, lon_deg, h, pad=PAD):
    lat0, lon0, h0 = pad
    s0 = math.sin(math.radians(lat0))
    w0 = math.sqrt(1 - E2 * s0 * s0)
    rn = A * (1 - E2) / w0 ** 3 + h0
    re = A / w0 + h0
    return np.stack([np.radians(np.asarray(lat_deg) - lat0) * rn,
                     np.radians(np.asarray(lon_deg) - lon0) * re * math.cos(math.radians(lat0)),
                     -(np.asarray(h) - h0)], axis=1)


# ------------------------------------------------------------------ runs

def unwrap_us(arr):
    """A log's u32 microsecond stamps, unwrapped (int64), as tools/rig_replay/truth.py does."""
    a = np.asarray(arr, dtype=np.int64)
    wraps = np.cumsum(np.diff(a, prepend=a[0]) < -2 ** 31)
    return a + wraps * 2 ** 32


def read_meta(events_path):
    meta = {}
    with open(events_path) as f:
        for ln in f:
            if ln.startswith("META "):
                parts = ln.split(None, 2)
                meta[parts[1]] = parts[2].strip() if len(parts) > 2 else ""
            elif ln[:2] in ("I ", "B ", "M ", "R ", "K ", "F "):
                break
    return meta


def harness_series(csv_path, t_base):
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    t = d["t_mcu_s"] + t_base
    q = np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], axis=1)
    pos = ned(d["lat_deg"], d["lon_deg"], d["h_m"])
    vel = np.stack([d["vn"], d["ve"], d["vd"]], axis=1)
    ok = (d["healthy"] > 0) & (d["finite"] > 0)
    extra = {k: d[k] for k in ("sig_tilt_n_deg", "sig_tilt_e_deg", "sig_hdg_deg", "epochs", "used_pr", "used_rr",
                                "cr_built", "cr_in", "cr_used", "hdg_handoffs", "gnss_aided", "ab_x", "ab_y", "ab_z",
                                "wb_x_dps", "wb_y_dps", "wb_z_dps")}
    out = dict(t=t, q=q, pos=pos, vel=vel, ok=ok, extra=extra, kind="harness")
    ej = csv_path + ".events.json"         # the harness's own apogee votes, at every step (any CSV rate)
    if os.path.exists(ej):
        import json
        out["events"] = json.load(open(ej))
    return out


def onboard_series(recs, t_base):
    """The log's own raw filter records (0x96, 10 Hz), and the flight loop's view (NonSensor: the pitch it voted
    on, the voter's own flag)."""
    sh = [r for r in recs["RawGnssEkf"]]
    t = unwrap_us([r["time_us"] for r in sh]) * 1e-6 + t_base
    q = np.array([[r["q0"], r["q1"], r["q2"], r["q3"]] for r in sh], float)
    pos = ned([r["lat_deg"] for r in sh], [r["lon_deg"] for r in sh], [r["alt_m"] for r in sh])
    vel = np.array([[r["vel_n"], r["vel_e"], r["vel_d"]] for r in sh], float)
    ok = np.array([bool(r["running"]) for r in sh]) & np.all(np.isfinite(q), axis=1) & np.all(np.isfinite(pos), axis=1)
    out = dict(t=t, q=q, pos=pos, vel=vel, ok=ok, extra={}, kind="onboard")
    att = recs.get("RawGnssAtt") or []
    if att:
        # 0x9B rides the same tick as its 0x96: matched by time
        by_t = {r["time_us"]: r for r in att}
        keys = (("sig_tilt_n_deg", "att_sigma_n_deg"), ("sig_tilt_e_deg", "att_sigma_e_deg"),
                ("sig_hdg_deg", "heading_sigma_deg"), ("ab_x", "acc_bias_x"), ("ab_y", "acc_bias_y"),
                ("ab_z", "acc_bias_z"), ("wb_x_dps", "gyro_bias_x_dps"), ("wb_y_dps", "gyro_bias_y_dps"),
                ("wb_z_dps", "gyro_bias_z_dps"))
        for k, src in keys:
            out["extra"][k] = np.array([by_t[r["time_us"]][src] if r["time_us"] in by_t else np.nan for r in sh],
                                       float)
    ns = recs.get("NonSensor") or []
    if ns:
        nt = unwrap_us([r["time_us"] for r in ns]) * 1e-6 + t_base
        out["ns_t"] = nt
        out["ns_pitch"] = np.array([r["pitch"] for r in ns], float)
        out["ns_pitch_apogee"] = np.array([bool(r.get("pitch_apogee", False)) for r in ns])
        out["ns_apogee"] = np.array([bool(r.get("apogee_flag", False)) for r in ns])
    return out


def stats3(x):
    x = np.abs(np.asarray(x, float))
    x = x[np.isfinite(x)]
    if x.size == 0:
        return (np.nan, np.nan, np.nan)
    return (float(np.median(x)), float(np.percentile(x, 90)), float(np.max(x)))


METRICS = ("att_total", "nose", "roll", "heading", "pitch", "vz", "pos_h", "pos_v", "vel")


def sample_errors(series, truth, step=0.1, grid=None, tol=None):
    """Every 0.1 s of flight time from T+0 (or at the given times, e.g. another run's records): the run's nearest
    sample (within 60 ms, or tol), each error against the truth at that sample's own time, and the phase it is in
    ("" outside boost/coast/descent)."""
    t = series["t"]
    t_end = float(t[-1])
    if grid is None:
        grid = np.arange(0.0, min(t_end, truth.end) + 1e-9, step)
    else:
        grid = np.asarray(grid, float)
        grid = grid[(grid >= 0.0) & (grid <= min(t_end, truth.end))]
    idx = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    idx = np.where(np.abs(t[idx - 1] - grid) <= np.abs(t[idx] - grid), idx - 1, idx)
    near = np.abs(t[idx] - grid) <= (tol if tol is not None else max(0.06, step * 0.6))
    idx, grid = idx[near], grid[near]
    ts = t[idx]
    pt, vt, qt = truth.at(ts)
    q = series["q"][idx]
    total, nose, roll, heading, pitch = att_errors(q, qt)
    perr = series["pos"][idx] - pt
    verr = series["vel"][idx] - vt
    phase = np.full(len(grid), "", dtype=object)
    for name, lo, hi in truth.phases(t_end):
        phase[(grid >= lo) & (grid < hi)] = name
    return {"t": grid, "phase": phase, "att_total": total, "nose": nose, "roll": roll, "heading": heading,
            "pitch": pitch, "vz": verr[:, 2], "pos_h": np.hypot(perr[:, 0], perr[:, 1]), "pos_v": perr[:, 2],
            "vel": np.linalg.norm(verr, axis=1), "ok": series["ok"][idx]}


def score(series, truth, step=0.1, grid=None, tol=None):
    """Metrics per phase, the pitch voter's crossing, health."""
    t = series["t"]
    t_end = float(t[-1])
    res = {"t_first": float(t[0]), "t_last": t_end, "phases": {}}
    e = sample_errors(series, truth, step, grid, tol)
    for name, lo, hi in truth.phases(t_end):
        m = e["phase"] == name
        if not m.any():
            continue
        res["phases"][name] = {"n": int(m.sum()), "from": lo, "to": hi, "unhealthy": int((~e["ok"][m]).sum())}
        for k in METRICS:
            res["phases"][name][k] = stats3(e[k][m])
    # the apogee pitch voter: the first time after burnout the estimated pitch is below -5 deg (at the run's own rate)
    pe = pitch_deg(qnorm(series["q"]))
    k = np.flatnonzero((t > truth.burnout) & (pe < PITCH_VOTE_DEG) & series["ok"])
    res["pitch_vote_t"] = float(t[k[0]]) if k.size else None
    # the velocity voter: above 15 m and descending (TR_KinematicChecks test 1, on the filter's state)
    k = np.flatnonzero((t > truth.burnout) & (-series["pos"][:, 2] > 15.0) & (series["vel"][:, 2] > 0.0)
                       & series["ok"])
    res["vel_vote_t"] = float(t[k[0]]) if k.size else None
    ev = series.get("events")
    if ev:
        res["pitch_vote_t"] = ev.get("pitch_vote_t")
        res["vel_vote_t"] = ev.get("vel_vote_t")
        res["gate_switch_t"] = ev.get("gate_switch_t")
    if "ns_t" in series:
        nt = series["ns_t"]
        k = np.flatnonzero((nt > truth.burnout) & (series["ns_pitch"] < PITCH_VOTE_DEG))
        res["ns_pitch_cross_t"] = float(nt[k[0]]) if k.size else None
        k = np.flatnonzero(series["ns_pitch_apogee"])
        res["ns_pitch_flag_t"] = float(nt[k[0]]) if k.size else None
        k = np.flatnonzero(series["ns_apogee"])
        res["ns_apogee_t"] = float(nt[k[0]]) if k.size else None
    # health over the whole run from T+0, and any non-finite sample
    m0 = t >= 0.0
    res["unhealthy_samples"] = int((~series["ok"][m0]).sum())
    res["samples"] = int(m0.sum())
    gaps = np.diff(t[m0])
    res["max_gap_s"] = float(gaps.max()) if gaps.size else 0.0
    return res


def diff_vs(series_a, series_b, truth, t_from=-1.0):
    """Run a less run b, at run b's samples (the on-board 10 Hz records), a's nearest within 6 ms: per phase (and the
    pad before ignition) the attitude difference (total rotation, nose direction, heading twist), the velocity
    (3-D and vertical) and position (horizontal, vertical) differences, median / p90 / max."""
    tb = series_b["t"]
    ta = series_a["t"]
    m = (tb >= t_from) & (tb <= ta[-1]) & (tb >= ta[0])
    tb = tb[m]
    i = np.clip(np.searchsorted(ta, tb), 1, len(ta) - 1)
    i = np.where(np.abs(ta[i - 1] - tb) <= np.abs(ta[i] - tb), i - 1, i)
    near = np.abs(ta[i] - tb) <= 0.006
    i, tb = i[near], tb[near]
    jb = np.flatnonzero(m)[near]
    qa, qb = qnorm(series_a["q"][i]), qnorm(series_b["q"][jb])
    total, nose, _roll, heading, pitch = att_errors(qa, qb)
    dv = series_a["vel"][i] - series_b["vel"][jb]
    dp = series_a["pos"][i] - series_b["pos"][jb]
    out = {}
    windows = [("pad", -5.0, 0.0)] + truth.phases(float(tb[-1]) + 1e-6 if tb.size else 0.0)
    for name, lo, hi in windows:
        w = (tb >= lo) & (tb < hi)
        if not w.any():
            continue
        out[name] = {"n": int(w.sum()), "att_total": stats3(total[w]), "nose": stats3(nose[w]),
                     "heading": stats3(heading[w]), "pitch": stats3(pitch[w]),
                     "vel": stats3(np.linalg.norm(dv[w], axis=1)), "vz": stats3(dv[w, 2]),
                     "pos_h": stats3(np.hypot(dp[w, 0], dp[w, 1])), "pos_v": stats3(dp[w, 2])}
    return out


EXTRA_DIFF = (("sig_hdg_deg", "heading 1-sigma (deg)"), ("sig_tilt_n_deg", "tilt 1-sigma north (deg)"),
              ("sig_tilt_e_deg", "tilt 1-sigma east (deg)"), ("ab_x", "accel bias x (m/s^2)"),
              ("ab_y", "accel bias y (m/s^2)"), ("ab_z", "accel bias z (m/s^2)"), ("wb_x_dps", "gyro bias x (deg/s)"),
              ("wb_y_dps", "gyro bias y (deg/s)"), ("wb_z_dps", "gyro bias z (deg/s)"))


def diff_extra(series_a, series_b, truth):
    """The filter's own sigmas and IMU biases (0x9B): run a less run b at run b's samples, |difference| per phase."""
    tb, ta = series_b["t"], series_a["t"]
    m = (tb >= ta[0]) & (tb <= ta[-1])
    i = np.clip(np.searchsorted(ta, tb[m]), 1, len(ta) - 1)
    i = np.where(np.abs(ta[i - 1] - tb[m]) <= np.abs(ta[i] - tb[m]), i - 1, i)
    near = np.abs(ta[i] - tb[m]) <= 0.006
    jb = np.flatnonzero(m)[near]
    i = i[near]
    out = {}
    windows = [("pad", -5.0, 0.0)] + truth.phases(float(tb[jb][-1]) + 1e-6 if jb.size else 0.0)
    for k, _name in EXTRA_DIFF:
        if k not in series_a["extra"] or k not in series_b["extra"]:
            continue
        d = series_a["extra"][k][i] - series_b["extra"][k][jb]
        for name, lo, hi in windows:
            w = (tb[jb] >= lo) & (tb[jb] < hi)
            if w.any():
                out.setdefault(name, {})[k] = stats3(d[w])
    return out
