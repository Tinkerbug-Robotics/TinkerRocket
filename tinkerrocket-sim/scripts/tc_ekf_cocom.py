#!/usr/bin/env python3
"""GNSS-only and IMU + GNSS filtering of a COCOM-rig raw capture, scored against the truth.

The rig replays a simulated flight through a real receiver; the capture holds the
receiver's raw pseudorange + Doppler (SkyTraq 0xE5, 20 Hz on the PX1105R), the
navigation subframes the ephemeris is decoded from, and the receiver's own fix. The
filters here track straight through the windows where the receiver withholds its
fix, because its raw measurements keep flowing there.

Modes (--mode, repeatable):
  spp    epoch-by-epoch least squares: position from the pseudoranges, velocity
         from the Doppler; no filter at all
  own    the receiver's own fix, where it gives one
  cv     GNSS-only EKF, constant-velocity model (tc_ekf_capture.py's)
  ca     GNSS-only EKF, constant-acceleration model (non-gravitational part, gravity
         added); after a gap it restarts on a checked least-squares fix if its
         prediction disagrees with one
  ins    tightly coupled: an IMU synthesized from the truth (the sim's IMUModel
         noise, WGS84 gravity, the Earth's rotation) propagates TcEkf, raw
         pseudorange + Doppler update it
  ins-lc loosely coupled: the same IMU, the receiver's own fix, always with the
         flight filter's mechanization -- what flies today -- so nothing inside the
         withheld windows
  imu    the same IMU with no GNSS after the pad (what the IMU alone carries)

The PX1105R's Doppler is the range rate ~0.22 s before its time tag (power
normal; 0.05-0.2 s in power save) while its pseudorange is on time: --rr-lag.
Its pseudoranges are receiver-smoothed, wander ~2x the still-antenna fit in
flight (--pr-corr-scale) and start 30-40 m off after every (re)lock
(--relock-sigma) -- on this rig only: a carrier-smoothed pseudorange restarts at
the raw code while settled channels sit the rig's 4.19 m/s carrier-vs-code split
times the smoothing time below it, and on the real sky there is no such offset. The IMU modes mirror the flight filter's mechanization unless
told otherwise: --gravity wgs84 --earth-rate --no-att-gate is the one a
flight to 80 km wants (see the results README, 2026-09-27).

Truth. gps-sdr-sim takes one motion row per 0.1 s: the code phase runs linearly
between rows, and the smooth-carrier build sweeps the carrier between block-edge
rates. make_flights.py integrates h_k = h_(k-1) + v_k dt, so its v_k is the mean
velocity over the block before t_k: the signal's velocity at t_k - 0.05 s. Scenario
JSONs written before 2026-09-27 carry that v_k at t_k, half a block late; newer ones,
and the archive since PR #1534, carry the signal's velocity at t_k. RigTruth tells
the two apart from the rows (h_k = h_(k-1) + v_k dt holds exactly only for the old
kind) and places the velocity where the signal had it.

    PYTHONPATH=src python3 scripts/tc_ekf_cocom.py CAPTURE.log.gz SCENARIO.json \\
        --pad 600 --mode own --mode ca --mode ins --gravity wgs84 --earth-rate --no-att-gate

Captures and truth: tools/gnss-cocom/sdr/results/ (README "Data for filter work").
The flights are vertical (no horizontal motion), so horizontal errors are the
filter's own; the sky is GPS L1 only, noiseless, without troposphere.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import sys
import tempfile

import numpy as np

from tinkerrocket_sim.estimation.tc_ekf import (TcEkf, TcEkfParams, G, OMGE, ecef2lla, lla2ecef, t_e2ned,
                                                spp_fix, los_predict, quat_from_accel_heading, quat2dcm,
                                                gravity_wgs84)
from tinkerrocket_sim.estimation.gnss_raw import read_capture, corrected

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "gnss-cocom"))
from gnss_nmea_monitor import replay_source  # noqa: E402

TOW_FILE0 = 203400.0          # every COCOM IQ file starts at 2026/08/18 08:30:00 GPS
EPOCH_DT = 0.05               # 20 Hz grid everything is scored on


# ----------------------------------------------------------------- truth
class RigTruth:
    """A padded make_flights.py flight in FILE time (receiver TOW - 203400)."""

    def __init__(self, scenario_json, pad_s=600.0):
        S = json.load(open(scenario_json))
        self.S = S
        self.shift = pad_s - S["prologue_s"]
        tr = S["truth"]
        tk = np.array([s["t"] for s in tr]) + self.shift
        h = np.array([s["alt_m"] for s in tr])
        vu = np.array([s["v_up_mps"] for s in tr])
        if any(s["v_east_mps"] for s in tr):
            raise SystemExit("this truth model is for vertical flights (v_east = 0)")
        if self.shift > 0:
            tp = np.arange(0.0, self.shift - 1e-9, 0.1)
            tk, h, vu = (np.concatenate([tp, tk]), np.concatenate([np.full(len(tp), h[0]), h]),
                         np.concatenate([np.zeros(len(tp)), vu]))
        self.tk, self.h, self.vu = tk, h, vu
        # Old JSONs give each row the 0.1 s block ENDING there (the signal had it half a
        # block earlier); newer ones give the signal's velocity at the row. See the docstring.
        t0 = np.array([s["t"] for s in tr])
        h0 = np.array([s["alt_m"] for s in tr])
        v0 = np.array([s["v_up_mps"] for s in tr])
        dt = np.diff(t0)
        ok = dt > 0
        block = bool(ok.any()) and float(np.max(np.abs(np.diff(h0)[ok] / dt[ok] - v0[1:][ok]))) < 1e-6
        self.tv = tk - (0.05 if block else 0.0)
        self.lat0 = math.radians(S["origin"]["lat_deg"])
        self.lon0 = math.radians(S["origin"]["lon_deg"])
        self.T = t_e2ned(self.lat0, self.lon0)
        ph = [s["phase"] for s in tr]
        ts = [s["t"] for s in tr]
        self.t_ign = S["prologue_s"] + self.shift
        self.t_burnout = ts[ph.index("coast")] + self.shift
        self.t_apogee = ts[int(np.argmax([s["alt_m"] for s in tr]))] + self.shift
        dv = np.diff([s["v_up_mps"] for s in tr])
        k = int(np.argmax(dv[len(dv) // 2:])) + len(dv) // 2
        self.t_main = ts[k] + self.shift       # main chute: the biggest late velocity step
        self.windows = [(a + self.shift, b + self.shift) for a, b in S["blocked_windows"]]
        self.vel_windows = [(a + self.shift, b + self.shift) for a, b in S.get("velocity_windows", [])]
        self.alt_windows = [(a + self.shift, b + self.shift) for a, b in S.get("altitude_80km_windows", [])]
        self.t_end = float(tk[-1])

    def alt(self, t):
        return float(np.interp(t, self.tk, self.h))

    def v_up(self, t):
        return float(np.interp(t, self.tv, self.vu))

    def a_up(self, t):
        """Kinematic acceleration, constant between velocity nodes."""
        i = max(1, min(len(self.tv) - 1, int(np.searchsorted(self.tv, t, side="right"))))
        return (self.vu[i] - self.vu[i - 1]) / (self.tv[i] - self.tv[i - 1])

    def pos_ecef(self, t):
        return lla2ecef(np.array([self.lat0, self.lon0, self.alt(t)]))

    def vel_ned(self, t):
        return np.array([0.0, 0.0, -self.v_up(t)])

    def phases(self, t_first=None):
        w = self.windows
        return [("pad (last 60 s)", self.t_ign - 60.0, self.t_ign),
                ("boost, fix allowed", self.t_ign, w[0][0]),
                ("W1 >515 m/s up", w[0][0], w[0][1]),
                ("coast, fix allowed", w[0][1], w[1][0]),
                ("W2 >80 km", w[1][0], w[1][1]),
                ("descent, fix allowed", w[1][1], w[2][0]),
                ("W3 >515 m/s down", w[2][0], w[2][1]),
                ("descent to main", w[2][1], self.t_main),
                ("under main", self.t_main, self.t_end),
                ("boost (ign-burnout)", self.t_ign, self.t_burnout),
                ("flight (ign-end)", self.t_ign, self.t_end)]


# ----------------------------------------------------------------- IMU synthesis
class SynthIMU:
    """Specific force and angular rate of the truth trajectory, body nose-up and
    not rotating relative to the ground (FRD: X up, Y east, Z north), with the
    sim's IMUModel errors: white noise, first-order Markov bias (sensors/imu_model.py
    defaults, given there per 1200 Hz sample), optional turn-on bias and scale
    factor. The world is the real Earth: WGS84 normal gravity, the Earth's rate on
    the gyros, and the Coriolis force an Earth-fixed ground track has to push
    against. (make_flights.py integrated the trajectory with a spherical
    9.80665 m/s^2 gravity, so in this world the coast reads ~0.03 m/s^2 of
    non-gravitational force -- as a whisper of drag would.)"""

    def __init__(self, truth, rate_hz, seed, accel_noise_1200=0.05, gyro_noise_1200=0.00175,
                 accel_bias_sigma=0.01, accel_bias_tau=100.0, gyro_bias_sigma=0.00025, gyro_bias_tau=50.0,
                 turn_on_accel=0.0, turn_on_gyro=0.0, scale_factor=0.0):
        self.tr, self.dt = truth, 1.0 / rate_hz
        self.rng = np.random.default_rng(seed)
        k = math.sqrt(1200.0 / rate_hz)        # the same noise density, sampled at rate_hz
        self.sa, self.sg = accel_noise_1200 / k, gyro_noise_1200 / k
        self.abs, self.abt, self.gbs, self.gbt = accel_bias_sigma, accel_bias_tau, gyro_bias_sigma, gyro_bias_tau
        self.ab = self.rng.normal(0, turn_on_accel, 3) if turn_on_accel else np.zeros(3)
        self.gb = self.rng.normal(0, turn_on_gyro, 3) if turn_on_gyro else np.zeros(3)
        self.ab_m, self.gb_m = np.zeros(3), np.zeros(3)
        self.sf = 1.0 + (self.rng.normal(0, scale_factor, 3) if scale_factor else np.zeros(3))
        # nose up: pitch +90 deg, yaw 0 -> NED to body
        self.q_true = quat_from_accel_heading([G, 0.0, 0.0], 0.0)
        self.C = quat2dcm(self.q_true)
        lat = truth.lat0
        self.w_ie = OMGE * np.array([math.cos(lat), 0.0, -math.sin(lat)])

    def sample(self, t):
        """IMU sample for the interval ending at t (constant over the step)."""
        tm = t - 0.5 * self.dt
        a_ned = np.array([0.0, 0.0, -self.tr.a_up(tm)])
        v_ned = self.tr.vel_ned(tm)
        g_ned = gravity_wgs84(self.tr.lat0, self.tr.alt(tm))
        f_ned = a_ned - g_ned + np.cross(2.0 * self.w_ie, v_ned)
        f_b = self.C @ f_ned
        w_b = self.C @ self.w_ie
        a = self.dt
        self.ab_m = (1 - a / self.abt) * self.ab_m + self.rng.normal(0, math.sqrt(2 * self.abs ** 2 * a / self.abt), 3)
        self.gb_m = (1 - a / self.gbt) * self.gb_m + self.rng.normal(0, math.sqrt(2 * self.gbs ** 2 * a / self.gbt), 3)
        acc = self.sf * f_b + self.ab + self.ab_m + self.rng.normal(0, self.sa, 3)
        gyr = w_b + self.gb + self.gb_m + self.rng.normal(0, self.sg, 3)
        return acc, gyr


# ----------------------------------------------------------------- data
def open_capture(path):
    if path.endswith(".gz"):
        tmp = tempfile.NamedTemporaryFile("wb", suffix=".log", delete=False)
        tmp.write(gzip.open(path, "rb").read())
        tmp.close()
        return tmp.name
    return path


def load_capture(path, systems="G"):
    eph, epochs, own, kind = read_capture(open_capture(path), replay_source, systems=systems,
                                          fix_doppler_truncation=True)
    ep = {}
    for tow, obs in epochs:
        ep[int(round((tow - TOW_FILE0) / EPOCH_DT))] = (tow, obs)
    own_d = {int(round((o[0] - TOW_FILE0) / EPOCH_DT)): o for o in own}
    return eph, ep, own_d, kind


class RelockAge:
    """Seconds since each satellite last (re)appeared in the raw output: the
    PX1105R's smoothed pseudorange starts 20-40 m off after a (re)lock and
    decays over ~10-20 s (COCOM rig, 2026-09-27). That offset is the rig's
    4.19 m/s carrier-vs-code split times the receiver's smoothing time; the real
    sky shows none."""

    def __init__(self, gap_s=0.5):
        self.first, self.last, self.gap = {}, {}, gap_s

    def update(self, t, obs):
        for o in obs:
            k = (o[0], o[1])
            if k not in self.last or t - self.last[k] > self.gap:
                self.first[k] = t
            self.last[k] = t

    def age(self, sys_, prn, t):
        return t - self.first.get((sys_, prn), t)


# ----------------------------------------------------------------- one run
def spp_checked(meas, x0, min_sats, max_spread_m=30.0):
    """A least-squares epoch fit good enough to restart a GNSS-only filter on.
    The worst satellite is dropped while the pseudorange residuals disagree
    (a satellite just back from a re-lock is tens of metres off), and at
    least ``min_sats`` must be left: a 4-5 satellite epoch can fit anything."""
    use = [m for m in meas if m.pr is not None]
    while len(use) >= min_sats:
        fx = spp_fix(use, x0)
        if fx is None or fx[1] is None:
            return None
        res = np.array([m.pr - los_predict(m, fx[0], np.zeros(3))[0] for m in use])
        res -= np.median(res)
        if np.sqrt(np.mean(res ** 2)) <= max_spread_m:
            return fx
        use.pop(int(np.argmax(np.abs(res))))
    return None


def restart_kinematic(ekf, r_ecef, v_ecef):
    """Put a GNSS-only filter back on a least-squares fix after a long gap:
    position, velocity and acceleration start over; the clock is kept."""
    lla = ecef2lla(r_ecef)
    ekf.lla = lla
    ekf.v = t_e2ned(lla[0], lla[1]) @ v_ecef
    ekf.acc = np.zeros(3)
    idx = list(range(0, 6)) + ([9, 10, 11] if ekf.kin_acc else [])
    ekf.P[idx, :] = 0.0
    ekf.P[:, idx] = 0.0
    for i in range(3):
        ekf.P[i, i] = 30.0 ** 2
        ekf.P[3 + i, 3 + i] = 10.0 ** 2
        if ekf.kin_acc:
            ekf.P[9 + i, 9 + i] = ekf.p.p0_kin_acc ** 2
    ekf.reclone()


def att_err(q_est, q_true):
    """Attitude error as a small rotation of the NED frame (rad): N, E, D."""
    C = quat2dcm(q_est).T @ quat2dcm(q_true)            # body->NED est, NED->body true
    return np.array([C[2, 1] - C[1, 2], C[0, 2] - C[2, 0], C[1, 0] - C[0, 1]]) * 0.5


def score_row(truth, t, r_ecef, v_ned):
    xt = truth.pos_ecef(t)
    d = truth.T @ (r_ecef - xt)
    dv = v_ned - truth.vel_ned(t)
    return (t, d[0], d[1], d[2], dv[0], dv[1], dv[2])


def run(args, mode, truth, data, seed=1):
    eph, ep, own, kind = data
    rows, sig = [], []
    keys = sorted(ep)
    k0, k1 = keys[0], min(keys[-1], int(round(truth.t_end / EPOCH_DT)))
    dstep = 1.0 if kind == "skytraq" else 0.0
    ages = RelockAge()

    def meas_at(k, r_ref):
        tow, obs = ep[k]
        ages.update(tow - TOW_FILE0, obs)
        m = corrected(tow, obs, eph, r_ref, use_tropo=False, el_mask_deg=args.el_mask, doppler_step_hz=dstep)
        t = tow - TOW_FILE0
        for x in m:
            x.sigma_pr_corr *= args.pr_corr_scale
            x.sigma_rr *= args.rr_scale
            if args.relock_sigma > 0:
                a = ages.age(x.sys, x.prn, t)
                x.sigma_pr_corr = math.hypot(x.sigma_pr_corr, args.relock_sigma * math.exp(-a / args.relock_tau))
        return m

    if mode == "own":
        for k in range(k0, k1 + 1):
            o = own.get(k)
            if o is not None:
                lla = ecef2lla(o[1])
                rows.append(score_row(truth, k * EPOCH_DT, o[1], t_e2ned(lla[0], lla[1]) @ o[2]))
        return np.array(rows), None, {}

    if mode == "spp":
        x0 = None
        for k in range(k0, k1 + 1):
            if k not in ep:
                continue
            tow, obs = ep[k]
            m = corrected(tow, obs, eph, x0, use_tropo=False, el_mask_deg=args.el_mask, doppler_step_hz=dstep)
            fx = spp_fix(m, x0)
            if fx is None or fx[1] is None:
                continue
            if x0 is None:        # redo with elevation mask / iono at the first position
                m = corrected(tow, obs, eph, fx[0], use_tropo=False, el_mask_deg=args.el_mask, doppler_step_hz=dstep)
                fx = spp_fix(m, fx[0])
                if fx is None or fx[1] is None:
                    continue
            x0 = fx[0]
            lla = ecef2lla(fx[0])
            rows.append(score_row(truth, k * EPOCH_DT, fx[0], t_e2ned(lla[0], lla[1]) @ fx[1]))
        return np.array(rows), None, {}

    prm = TcEkfParams()
    prm.clk_bias_psd = args.clk_bias_psd
    prm.rr_lag_s = args.rr_lag
    prm.p0_rr_lag = args.p0_rr_lag
    prm.rr_lag_psd = args.rr_lag_psd
    prm.p0_rate_ofs = args.p0_rate_ofs
    prm.raw_gate_chi2 = args.gate
    prm.kin_gravity = args.kin_gravity
    prm.kin_acc_tau = args.acc_tau
    if mode.startswith("ins") or mode == "imu":
        prm.a_noise = args.a_noise
    if mode in ("ins", "imu"):                   # ins-lc stays the flight filter as it flies
        prm.gravity_model = args.gravity
        prm.earth_rate = args.earth_rate
        prm.gnss_att_gate = "none" if args.no_att_gate else args.att_gate
    ekf = TcEkf(prm)
    stats = dict(used_pr=0, used_rr=0, rej_pr=0, rej_rr=0, jumps=0, restarts=0)

    # ---- first fix: position, velocity, clock from a least-squares epoch
    k_init = None
    for k in range(k0, k1 + 1):
        if k not in ep or k * EPOCH_DT < args.t_init:
            continue
        tow, obs = ep[k]
        m = corrected(tow, obs, eph, None, use_tropo=False, el_mask_deg=args.el_mask)
        fx = spp_fix(m)
        if fx is None or fx[1] is None:
            continue
        m = corrected(tow, obs, eph, fx[0], use_tropo=False, el_mask_deg=args.el_mask, doppler_step_hz=dstep)
        fx = spp_fix(m, fx[0])
        if fx is None or fx[1] is None:
            continue
        k_init, fx0, m0 = k, fx, m
        break
    if k_init is None:
        raise SystemExit("never got a first fix")
    t0 = k_init * EPOCH_DT
    lla = ecef2lla(fx0[0])
    kin = mode in ("cv", "ca")
    if kin:
        ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx0[1], [1.0, 0.0, 0.0, 0.0])
        ekf.freeze_imu_states()
        if mode == "ca":
            ekf.enable_kinematic_acceleration()
    else:
        imu = SynthIMU(truth, args.imu_rate, seed, turn_on_accel=args.turn_on_accel,
                       turn_on_gyro=args.turn_on_gyro, scale_factor=args.scale_factor)
        # pad seed as the flight computer: gyro bias = mean stationary gyro,
        # attitude from the mean specific force and the pad heading, taken as known
        n = int(args.imu_rate * 2.0)
        s = [imu.sample(t0 - 2.0 + (i + 1) / args.imu_rate) for i in range(n)]
        a_mean = np.mean([x[0] for x in s], axis=0)
        g_mean = np.mean([x[1] for x in s], axis=0)
        ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx0[1], [1.0, 0.0, 0.0, 0.0])
        ekf.set_quaternion(quat_from_accel_heading(a_mean, 0.0))
        ekf.seed_gyro_bias(g_mean)
    ekf.t = t0
    ekf.init_clock_from(m0)
    ages.update(t0, ep[k_init][1])

    nsub = int(round(args.imu_rate * EPOCH_DT))
    t_ok = t0                                    # last epoch with 4+ pseudoranges used
    for k in range(k_init + 1, k1 + 1):
        t = k * EPOCH_DT
        if kin:
            ekf.propagate_kinematic(EPOCH_DT, args.accel_psd, args.jerk_psd)
        else:
            for i in range(nsub):
                ti = t - EPOCH_DT + (i + 1) * EPOCH_DT / nsub
                acc, gyr = imu.sample(ti)
                ekf.propagate(acc, gyr, EPOCH_DT / nsub)
                if ti < truth.t_ign - 0.5:          # levelling on the pad, as the flight computer
                    ekf.update_accel_level(acc)
        use_gnss = (mode != "imu") or t < truth.t_ign
        if mode == "ins-lc":
            o = own.get(k)
            if o is not None and use_gnss:
                lla_o = ecef2lla(o[1])
                ekf.update_gnss_fix(lla_o, t_e2ned(lla_o[0], lla_o[1]) @ o[2])
        elif k in ep and use_gnss:
            r, _, _ = ekf.receiver_state_ecef()
            m = meas_at(k, r)
            if kin and args.reinit_gap > 0 and t - t_ok > args.reinit_gap:
                # GNSS-only after a gap: if a checked fix disagrees with the
                # prediction, restart on it rather than drag the state across
                fx = spp_checked(m, r, args.reinit_min_sats)
                if fx is not None:
                    sig_p = math.sqrt(max(0.0, np.trace(ekf.P[0:3, 0:3])))
                    if np.linalg.norm(fx[0] - r) > 3.0 * sig_p + 100.0:
                        restart_kinematic(ekf, fx[0], fx[1])
                        stats["restarts"] += 1
            if m:
                st = ekf.update_gnss_raw(m)
                if st.used_pr >= 4:
                    t_ok = t
                stats["used_pr"] += st.used_pr
                stats["used_rr"] += st.used_rr
                stats["rej_pr"] += st.rejected_pr
                stats["rej_rr"] += st.rejected_rr
                stats["jumps"] += int(st.clock_jump_ms != 0)
        r, v_e, T = ekf.receiver_state_ecef()
        rows.append(score_row(truth, t, r, ekf.v.copy()))
        sd = ekf.sigmas()
        att = att_err(ekf.q, imu.q_true) if not kin else np.zeros(3)
        sig.append((t, sd[0], sd[1], sd[2], sd[3], sd[4], sd[5], *att, *(ekf.ab if not kin else np.zeros(3)),
                    sd[6], sd[7], sd[8]))
    stats["rate_ofs"] = ekf.rate_ofs
    stats["rr_lag"] = ekf.rr_lag
    stats["clk_drift"] = ekf.clk[1]
    return np.array(rows), np.array(sig), stats


# ----------------------------------------------------------------- scoring
def phase_table(truth, A, S=None):
    out = []
    for nm, a, b in truth.phases():
        m = (A[:, 0] >= a) & (A[:, 0] < b)
        if m.sum() < 2:
            out.append((nm, 0) + (math.nan,) * 8)
            continue
        hz = np.hypot(A[m, 1], A[m, 2])
        up = -A[m, 3]
        vh = np.hypot(A[m, 4], A[m, 5])
        vu = -A[m, 6]
        cover = m.sum() / max(1, round((min(b, truth.t_end) - a) / EPOCH_DT))
        nees = math.nan
        if S is not None:
            ms = (S[:, 0] >= a) & (S[:, 0] < b)
            if ms.any():
                nees = float(np.mean((A[m, 3] / S[ms, 3]) ** 2))    # vertical position NEES
        out.append((nm, cover, np.sqrt(np.mean(hz ** 2)), np.sqrt(np.mean(up ** 2)), np.mean(up),
                    np.percentile(np.abs(up), 95), np.sqrt(np.mean(vh ** 2)), np.sqrt(np.mean(vu ** 2)),
                    np.percentile(np.abs(vu), 95), nees))
    return out


def print_table(title, tab, log=print):
    log(f"\n== {title}")
    log(f"{'phase':<22}{'cover':>6}{'horiz':>8}{'up rms':>8}{'up mean':>9}{'up p95':>8}"
        f"{'vh rms':>8}{'vu rms':>8}{'vu p95':>8}{'NEES_u':>8}")
    for r in tab:
        nm, cov = r[0], r[1]
        if not cov:
            log(f"{nm:<22}{'--':>6}")
            continue
        log(f"{nm:<22}{cov:6.0%}{r[2]:8.1f}{r[3]:8.1f}{r[4]:+9.1f}{r[5]:8.1f}{r[6]:8.2f}{r[7]:8.2f}{r[8]:8.2f}{r[9]:8.1f}")



# ----------------------------------------------------------------- figure
# categorical slots in a fixed order per mode, so a mode keeps its color whatever else is plotted
PLOT_STYLE = {"ins": ("#2a78d6", "IMU + raw, tightly coupled"), "ca": ("#eb6834", "GNSS only, raw"),
              "own": ("#1baf7a", "receiver's own fix"), "ins-lc": ("#eda100", "IMU + own fix (flight filter today)"),
              "cv": ("#e87ba4", "GNSS only, constant velocity"), "spp": ("#008300", "epoch least squares"),
              "imu": ("#4a3aa7", "IMU alone after ignition")}


def _nice_up(x):
    """Round up to 1, 2, 2.5 or 5 x 10^k."""
    if not x > 0:
        return 1.0
    k = 10 ** math.floor(math.log10(x))
    return next(m * k for m in (1, 2, 2.5, 5, 10) if m * k >= x)


def flight_events(truth, t0, t_hi):
    """(time after ignition, name) of the events inside the plotted span."""
    return [(t - t0, n) for t, n in ((truth.t_ign, "ignition"), (truth.t_burnout, "burnout"),
                                     (truth.t_apogee, "apogee"), (truth.t_main, "main chute"))
            if t - t0 < t_hi]


def shade_limits(ax, truth, t0, t_hi, label_events=False):
    """px_limits.py's convention -- amber above the speed limit, violet above
    80 km -- with a solid line at ignition and dotted ones at the other events,
    named along the top of the panel when ``label_events``."""
    muted = "#898781"
    for windows, col, alpha in ((truth.vel_windows, "#eda100", 0.10), (truth.alt_windows, "#7b61ff", 0.08)):
        for w0, w1 in windows:
            ax.axvspan(w0 - t0, w1 - t0, color=col, alpha=alpha, lw=0, zorder=0)
    for t, n in flight_events(truth, t0, t_hi):
        ax.axvline(t, color=muted, lw=0.6, ls="-" if n == "ignition" else (0, (1, 3)), zorder=1)
        if label_events:                          # ignition reads leftward, so burnout never runs into it
            ax.text(t + (-1.0 if n == "ignition" else 1.0), 1.012, n, transform=ax.get_xaxis_transform(),
                    ha="right" if n == "ignition" else "left", va="bottom", fontsize=7, color="#52514e")


def plot_tracks(path, truth, data, tracks, title, lims=None):
    """The satellites the receiver reports, the trajectory (truth and each
    estimate), then the signed errors, one panel each. ``tracks``: mode -> the
    run()'s error rows (t, dN, dE, dD, dvN, dvE, dvD)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, ink2, muted, grid, base, truth_c = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#b9b8b0"
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 8, "axes.edgecolor": base,
                         "axes.labelcolor": ink2, "xtick.color": muted, "ytick.color": muted,
                         "axes.spines.top": False, "axes.spines.right": False, "svg.fonttype": "none",
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})
    t0 = truth.t_ign
    series = []
    for mode in PLOT_STYLE:
        if mode not in tracks:
            continue
        A = tracks[mode][::4]                             # 5 Hz is plenty on paper
        t = A[:, 0]
        c = dict(t=t - t0, alt=np.array([truth.alt(x) for x in t]) - A[:, 3],
                 vup=np.array([truth.v_up(x) for x in t]) - A[:, 6],
                 ealt=-A[:, 3], evup=-A[:, 6], ehor=np.hypot(A[:, 1], A[:, 2]))
        gap = np.flatnonzero(np.diff(t) > 0.5)            # withheld windows stay gaps
        for k in c:
            c[k] = np.insert(c[k].astype(float), gap + 1, np.nan)
        series.append((mode,) + PLOT_STYLE[mode] + (c,))
    t_hi = max(float(np.nanmax(c["t"])) for *_, c in series)
    if lims is None:                                      # every trace in view, up to a cap
        def lim(key, floor, cap):
            p = [np.nanpercentile(np.abs(c[key][c["t"] > 0]), 99) for *_, c in series if np.isfinite(c[key]).any()]
            return min(cap, _nice_up(max(floor, max(p))))
        lims = (lim("ealt", 5.0, 100.0), lim("evup", 0.5, 5.0), lim("ehor", 5.0, 100.0))
    fig, ax = plt.subplots(6, 1, figsize=(9.8, 13.6), sharex=True,
                           gridspec_kw=dict(hspace=0.24, height_ratios=[0.5, 1.2, 1.2, 1, 1, 1]))
    fig.subplots_adjust(top=0.915)
    sat, alt, vel, e_alt, e_vel, e_hor = ax
    # satellites with raw measurements (fewest in each second), and the receiver's own fix
    _, ep, own, _ = data
    et = np.array([ep[k][0] - TOW_FILE0 for k in sorted(ep)])
    ns = np.array([len(ep[k][1]) for k in sorted(ep)], float)
    sec = np.arange(math.floor(t0 - 20.0), math.ceil(t0 + t_hi))
    k = np.searchsorted(sec, et, side="right") - 1
    ok = (k >= 0) & (k < len(sec))
    nmin = np.full(len(sec), np.nan)
    np.fmin.at(nmin, k[ok], ns[ok])
    sat.step(sec - t0, nmin, where="post", color=ink2, lw=0.9)
    ot = np.array([o[0] - TOW_FILE0 for o in own.values()])
    ko = np.searchsorted(sec, ot, side="right") - 1
    cnt = np.bincount(ko[(ko >= 0) & (ko < len(sec))], minlength=len(sec))
    sat.fill_between(sec - t0, 0, 1.5, where=cnt >= 10, color=PLOT_STYLE["own"][0], lw=0, step="post")
    sat.set_ylim(0, 15)
    sat.set_yticks([0, 5, 10, 15])
    sat.set_ylabel("satellites\n(raw, fewest\nin each second)")
    # trajectory
    tt = np.arange(t0 - 20.0, t0 + t_hi, 0.1)
    alt.plot(tt - t0, [truth.alt(x) / 1000 for x in tt], color=truth_c, lw=4.5, solid_capstyle="butt",
             zorder=2, label="truth (injected)")
    vel.plot(tt - t0, [truth.v_up(x) for x in tt], color=truth_c, lw=4.5, solid_capstyle="butt", zorder=2)
    for mode, col, lab, c in series:
        lw = 1.3 if mode == "ins" else 1.0
        z = 4 if mode == "ins" else 3
        alt.plot(c["t"], c["alt"] / 1000, color=col, lw=0.9 * lw, label=lab, zorder=z + 1, rasterized=True)
        vel.plot(c["t"], c["vup"], color=col, lw=0.9 * lw, zorder=z + 1, rasterized=True)
        for a, key in ((e_alt, "ealt"), (e_vel, "evup"), (e_hor, "ehor")):
            a.plot(c["t"], c[key], color=col, lw=0.8 * lw, zorder=z, rasterized=True)
    alt.axhline(80, color=muted, lw=0.7, ls=(0, (1, 2)))
    alt.text(1.0, 80, " 80 km", transform=alt.get_yaxis_transform(), color=muted, fontsize=6.8, va="center")
    v_lim = truth.S.get("limits", {}).get("velocity_mps", 515.0)
    for v in (v_lim, -v_lim):
        vel.axhline(v, color=muted, lw=0.7, ls=(0, (1, 2)))
        vel.text(1.0, v, f" {v:+.0f} m/s", transform=vel.get_yaxis_transform(), color=muted, fontsize=6.8,
                 va="center")
    alt.set_ylabel("altitude (km)")
    vel.set_ylabel("vertical velocity (m/s)")
    # errors
    for a in (e_alt, e_vel):
        a.axhline(0, color=base, lw=0.8, zorder=2)
    for a, (key, lab, unit), L in zip((e_alt, e_vel, e_hor), (("ealt", "altitude error (m)", "m"),
                                                            ("evup", "vertical velocity\nerror (m/s)", "m/s"),
                                                            ("ehor", "horizontal position\nerror (m)", "m")), lims):
        a.set_ylim(0 if key == "ehor" else -L, L)
        a.set_ylabel(lab)
        notes = []
        for mode, col, name, c in series:
            if np.isfinite(c[key]).any() and np.nanmax(np.abs(c[key])) > L:
                v = float(np.nanmax(np.abs(c[key])))
                notes.append(f"{name} (peak {v / 1000:.1f} km)" if unit == "m" and v >= 1000
                             else f"{name} (peak {v:.0f} {unit})")
        if notes:
            a.text(1.0, 1.015, "off scale: " + ";  ".join(notes), transform=a.transAxes, fontsize=6.8,
                   color=ink2, ha="right", va="bottom")
    e_hor.set_xlabel("time after ignition (s)")
    for i, a in enumerate(ax):
        shade_limits(a, truth, t0, t_hi, label_events=(i == 0))
        a.grid(color=grid, lw=0.5)
        for sp in a.spines.values():
            sp.set_color(grid)
    sat.set_xlim(-20.0, t_hi)
    h, l = alt.get_legend_handles_labels()
    sat.legend(h, l, loc="lower left", bbox_to_anchor=(0.0, 1.3), ncol=5, frameon=False, fontsize=7.3,
               handlelength=1.8, borderaxespad=0.0, columnspacing=1.2)
    fig.text(0.125, 0.978, title + "; amber = above 515 m/s, violet = above 80 km", fontsize=9.5, color=ink,
             ha="left")
    fig.text(0.125, 0.964, "The receiver withholds its own fix in the shaded windows (green bar: where it gives "
             "one). Errors are estimate minus truth, drawn at 5 Hz from the 20 Hz tracks.",
             fontsize=7.2, color=ink2, ha="left")
    fig.savefig(path, bbox_inches="tight", dpi=100 if path.endswith(".svg") else 110)
    plt.close(fig)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("scenario")
    ap.add_argument("--pad", type=float, default=600.0, help="ignition time in file seconds")
    ap.add_argument("--mode", action="append", choices=("spp", "own", "cv", "ca", "ins", "ins-lc", "imu"))
    ap.add_argument("--el-mask", dest="el_mask", type=float, default=3.0)
    ap.add_argument("--t-init", dest="t_init", type=float, default=None,
                    help="start the filters here (file s; default 120 s before ignition)")
    ap.add_argument("--clk-bias-psd", dest="clk_bias_psd", type=float, default=0.009)
    ap.add_argument("--p0-rate-ofs", dest="p0_rate_ofs", type=float, default=10.0,
                    help="Doppler clock-rate offset state, initial sigma m/s (0 = off)")
    ap.add_argument("--rr-lag", dest="rr_lag", type=float, default=0.22,
                    help="Doppler measurement lag, s: the value, or the start value when estimated "
                         "(PX1105R 0.20-0.23 in power normal; 0.05-0.2, moving with the dynamics, "
                         "in power save -- use ~0.08 there)")
    ap.add_argument("--p0-rr-lag", dest="p0_rr_lag", type=float, default=0.0,
                    help="estimate the Doppler lag, initial sigma s (0 = hold --rr-lag fixed; GNSS alone "
                         "cannot separate it from the pseudorange errors)")
    ap.add_argument("--rr-lag-psd", dest="rr_lag_psd", type=float, default=1e-5, help="s^2/s")
    ap.add_argument("--accel-psd", dest="accel_psd", type=float, default=None,
                    help="kinematic white-acceleration PSD (default 400 cv, 0.01 ca)")
    ap.add_argument("--jerk-psd", dest="jerk_psd", type=float, default=100.0,
                    help="ca: white-jerk PSD m^2/s^5")
    ap.add_argument("--pr-corr-scale", dest="pr_corr_scale", type=float, default=2.0,
                    help="scale the correlated pseudorange sigma of gnss_raw's error model (fitted on "
                         "a still antenna; in flight the PX1105R's smoothed code wanders ~2x as far)")
    ap.add_argument("--rr-scale", dest="rr_scale", type=float, default=1.0, help="scale the Doppler sigma")
    ap.add_argument("--relock-sigma", dest="relock_sigma", type=float, default=40.0,
                    help="extra correlated pseudorange sigma right after a (re)lock, m, decaying "
                         "over --relock-tau (0 = off). A COCOM-rig artifact: use 0 on real-sky data")
    ap.add_argument("--relock-tau", dest="relock_tau", type=float, default=10.0)
    ap.add_argument("--gate", type=float, default=10.83, help="chi-square gate per measurement")
    ap.add_argument("--no-kin-gravity", dest="kin_gravity", action="store_false",
                    help="ca: carry the total acceleration instead of the non-gravitational part with "
                         "gravity added (the default, which bridges a gap ballistically)")
    ap.add_argument("--acc-tau", dest="acc_tau", type=float, default=0.0,
                    help="ca: Singer decay of the acceleration state, s (0 = random walk)")
    ap.add_argument("--reinit-gap", dest="reinit_gap", type=float, default=2.0,
                    help="GNSS-only: after this long without an update, restart on a checked fix (0 = off)")
    ap.add_argument("--reinit-min-sats", dest="reinit_min_sats", type=int, default=5)
    ap.add_argument("--imu-rate", dest="imu_rate", type=float, default=200.0)
    ap.add_argument("--a-noise", dest="a_noise", type=float, default=0.20, help="filter's accel noise (flight 0.20)")
    ap.add_argument("--gravity", choices=("const", "wgs84"), default="const",
                    help="IMU mechanization gravity: the flight filter's constant G, or WGS84 normal gravity")
    ap.add_argument("--earth-rate", dest="earth_rate", action="store_true")
    ap.add_argument("--att-gate", dest="att_gate", choices=("cos4", "heading", "none"), default="cos4",
                    help="what GNSS updates may do to the attitude: cos4 = the flight filter (scaled by "
                         "cos^4 pitch, 0 when vertical); heading = all but the rotation about the "
                         "specific force; none = everything")
    ap.add_argument("--no-att-gate", dest="no_att_gate", action="store_true", help="same as --att-gate none")
    ap.add_argument("--turn-on-accel", dest="turn_on_accel", type=float, default=0.0)
    ap.add_argument("--turn-on-gyro", dest="turn_on_gyro", type=float, default=0.0)
    ap.add_argument("--scale-factor", dest="scale_factor", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--npz", help="save every mode's error track here")
    ap.add_argument("--plot", action="append",
                    help="draw the trajectory and each mode's errors to this .png or .svg (repeatable)")
    ap.add_argument("--plot-title", dest="plot_title", help="figure title (default: the capture name)")
    ap.add_argument("--plot-lims", dest="plot_lims",
                    help="error axes: altitude m, vertical velocity m/s, horizontal m (default: every "
                         "trace in view, capped at 100 m and 5 m/s)")
    return ap


def main():
    args = build_parser().parse_args()
    truth = RigTruth(args.scenario, args.pad)
    if args.t_init is None:
        args.t_init = truth.t_ign - 120.0
    data = load_capture(args.capture)
    print(f"{os.path.basename(args.capture)}: {len(data[1])} raw epochs, {len(data[2])} own fixes; "
          f"ignition {truth.t_ign:.1f}, burnout {truth.t_burnout:.1f}, apogee {truth.t_apogee:.1f}, "
          f"windows {', '.join(f'{a:.1f}-{b:.1f}' for a, b in truth.windows)} (file s)")
    out = {}
    for mode in args.mode or ["spp", "own", "ca"]:
        a = argparse.Namespace(**vars(args))
        if a.accel_psd is None:
            a.accel_psd = 400.0 if mode == "cv" else 0.01
        A, S, st = run(a, mode, truth, data, seed=args.seed)
        out[mode] = A
        print_table(f"{mode}  {st if st else ''}", phase_table(truth, A, S))
    if args.npz:
        np.savez(args.npz, **out)
    for path in args.plot or []:
        lims = tuple(float(x) for x in args.plot_lims.split(",")) if args.plot_lims else None
        plot_tracks(path, truth, data, out, args.plot_title or os.path.basename(args.capture), lims)
        print("wrote", path)


if __name__ == "__main__":
    main()
