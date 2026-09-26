"""Standalone GNSS/INS EKF that can take raw pseudorange and Doppler.

A Python mirror of the flight filter (``TR_GpsInsEKF``) -- same nominal state,
same error-state layout and frame conventions, same IMU mechanization, pad
levelling, baro and GNSS-fix updates -- extended with two receiver-clock states
so GNSS can enter as per-satellite measurements instead of a finished fix:

    error state (17)
      0:3   position error, NED (m)
      3:6   velocity error, NED (m/s)
      6:9   attitude error, quaternion vector part (half-angle, as the C++)
      9:12  accelerometer bias (m/s^2)
      12:15 gyro bias (rad/s)
      15    receiver clock bias (m)                    <- new
      16    receiver clock drift, at 1 g (m/s)         <- new
      17    clock g-sensitivity (m/s of drift per g)   <- new

The clock's crystal shifts frequency with acceleration (~1 ppb/g on a TCXO,
0.3 m/s of range-rate per g). Modelled as drift = d0 + kg * (|f|/g - 1), so
the filter expects the drift to jump at ignition and come back at burnout
instead of having to rediscover it from whatever satellites survive.

      18    Galileo - GPS inter-system bias (m)        <- multi-GNSS
      19    BeiDou  - GPS inter-system bias (m)

Each constellation reaches the receiver through its own hardware delays and
time scale, so its pseudoranges share the GPS clock bias plus a near-constant
offset of a few to tens of metres. The Doppler shares one drift.

``update_gnss_fix`` is the loosely coupled path the flight code uses today;
``update_gnss_raw`` is the tightly coupled one. Keeping both in one class means
a comparison between them changes nothing but the GNSS update.

Body frame FRD, world frame NED, ``q`` scalar-first with Quat2DCM(q) = NED->body,
exactly as TR_GpsInsEKF.h. Not flight code: numpy, float64, dense covariance.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.807
ECC2 = 0.0066943799901
EARTH_RADIUS = 6378137.0
OMGE = 7.2921151467e-5
C_LIGHT = 299792458.0
N = 20
ISB_INDEX = {"E": 18, "C": 19}


# ----------------------------------------------------------------- helpers
def quat2dcm(q):
    """NED->body DCM from the body attitude quaternion (TR_GpsInsEKF Quat2DCM)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)],
        [2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)],
        [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_mult(a, b):
    return np.array([
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def earth_rad(lat):
    d = abs(1.0 - ECC2 * math.sin(lat) ** 2)
    return EARTH_RADIUS / math.sqrt(d), EARTH_RADIUS * (1 - ECC2) / (d * math.sqrt(d))


def lla2ecef(lla):
    lat, lon, alt = lla
    rew, _ = earth_rad(lat)
    return np.array([(rew + alt) * math.cos(lat) * math.cos(lon),
                     (rew + alt) * math.cos(lat) * math.sin(lon),
                     (rew * (1 - ECC2) + alt) * math.sin(lat)])


def ecef2lla(x):
    p = math.hypot(x[0], x[1])
    lon = math.atan2(x[1], x[0])
    lat = math.atan2(x[2], p * (1 - ECC2))
    alt = 0.0
    for _ in range(8):
        rew, _ = earth_rad(lat)
        alt = p / math.cos(lat) - rew
        lat = math.atan2(x[2], p * (1 - ECC2 * rew / (rew + alt)))
    return np.array([lat, lon, alt])


def t_e2ned(lat, lon):
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return np.array([[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]])


def los_predict(m: "RawMeas", r_rx, v_rx):
    """Predicted pseudorange (without receiver clock), range rate (without
    drift) and line-of-sight unit vector, with the Earth-rotation (Sagnac)
    correction: the satellite position is at transmit time in the ECEF frame
    of that instant, so it is turned by omega_e * travel time."""
    rs = np.asarray(m.sat_pos, float)
    vs = np.asarray(m.sat_vel, float)
    rho = np.linalg.norm(rs - r_rx)
    for _ in range(2):
        th = OMGE * rho / C_LIGHT
        c, s = math.cos(th), math.sin(th)
        rs_r = np.array([c * rs[0] + s * rs[1], -s * rs[0] + c * rs[1], rs[2]])
        rho = np.linalg.norm(rs_r - r_rx)
    vs_r = np.array([c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]])
    u = (rs_r - r_rx) / rho
    return rho, float(u @ (vs_r - v_rx)), u


def spp_fix(meas, x0=None, iters=10):
    """Epoch-by-epoch weighted least squares, as a receiver's own fix would be
    formed before its navigation filter: position + clock bias from the
    pseudoranges, velocity + clock drift from the range rates. Returns
    (r_ecef, v_ecef, n_used) or None with fewer than four satellites."""
    use = [m for m in meas if m.pr is not None]
    systems = sorted({getattr(m, "sys", "G") for m in use}, key="GEC".index)
    if len(use) < 3 + len(systems):
        return None
    col = {sy: 3 + k for k, sy in enumerate(systems)}      # one clock per system
    x = np.zeros(3 + len(systems))
    if x0 is not None:
        x[:3] = np.asarray(x0, float)
    for _ in range(iters):
        H, y, w = [], [], []
        for m in use:
            rho, _, u = los_predict(m, x[:3], np.zeros(3))
            row = [-u[0], -u[1], -u[2]] + [0.0] * len(systems)
            c = col[getattr(m, "sys", "G")]
            row[c] = 1.0
            H.append(row); y.append(m.pr - (rho + x[c])); w.append(1.0 / m.sigma_pr ** 2)
        H, y, W = np.array(H), np.array(y), np.diag(w)
        dx = np.linalg.solve(H.T @ W @ H, H.T @ W @ y)
        x += dx
        if np.linalg.norm(dx) < 1e-3:
            break
    rr = [m for m in use if m.rr is not None]
    if len(rr) < 4:
        return x[:3], None, len(use)
    Hv, yv, wv = [], [], []
    for m in rr:
        _, rate, u = los_predict(m, x[:3], np.zeros(3))
        Hv.append([-u[0], -u[1], -u[2], 1.0]); yv.append(m.rr - rate); wv.append(1.0 / m.sigma_rr ** 2)
    Hv, yv, Wv = np.array(Hv), np.array(yv), np.diag(wv)
    v = np.linalg.solve(Hv.T @ Wv @ Hv, Hv.T @ Wv @ yv)
    return x[:3], v[:3], len(use)


def nis_scale(ekf, H, R, y, key):
    """R inflation for a let-back-in measurement: NIS brought down to the gate."""
    if ekf._rejects.get(key, 0) < ekf.p.raw_reject_persist:
        return 1.0
    S = float((H @ ekf.P @ H.T).item()) + R
    nis = y * y / S
    return max(1.0, nis / ekf.p.raw_gate_chi2) if nis > ekf.p.raw_gate_chi2 else 1.0


# ----------------------------------------------------------------- tuning
@dataclass
class TcEkfParams:
    # IMU noise model: the flight filter's defaults (TR_GpsInsEKF.h)
    a_noise: float = 0.20
    a_markov_sigma: float = 0.01
    a_markov_tau: float = 100.0
    w_noise: float = 0.00175
    w_markov_sigma: float = 0.00025
    w_markov_tau: float = 50.0
    # GNSS fix update (loosely coupled), as the flight filter
    fix_pos_sigma_ne: float = 3.0
    fix_pos_sigma_d: float = 6.0
    fix_vel_sigma_ne: float = 0.3
    fix_vel_sigma_d: float = 0.3
    baro_var: float = 4.0
    accel_var: float = 0.25
    # Receiver clock: TCXO-class white-frequency + random-walk-frequency noise,
    # q_b = c^2 h0 / 2, q_d = 2 pi^2 c^2 h_-2 with h0 ~ 2e-19, h_-2 ~ 2e-20.
    clk_bias_psd: float = 0.009      # m^2/s
    clk_drift_psd: float = 0.036     # m^2/s^3
    # A TCXO moves ~1 ppb per g; under thrust that is metres per second of drift.
    p0_clk_g: float = 0.5            # m/s per g, initial sigma of the g-sensitivity state
    p0_isb: float = 30.0             # m, inter-system bias before the data sets it
    isb_psd: float = 1e-4            # m^2/s, it is nearly constant
    clk_g_psd: float = 1e-6          # (m/s/g)^2/s, it is a constant of the part
    # Raw measurement gates (chi-square, 1 dof): reject, don't inflate
    raw_gate_chi2: float = 10.83     # p = 0.001
    # a satellite rejected this many epochs running is let back in, its R
    # inflated to the gate (the flight filter's own inflation rule) -- a
    # gate that keeps refusing every satellite means the state is wrong
    raw_reject_persist: int = 3
    # Attitude gain gate on GNSS updates, as the flight filter (cos^4 pitch)
    gnss_att_cos4_gate: bool = True
    # initial sigmas
    p0_pos: float = 10.0
    p0_vel: float = 1.0
    p0_att: float = 0.34906
    p0_hdg: float = 3.14159
    p0_abias: float = 0.981
    p0_wbias: float = 0.01745
    p0_clk_bias: float = 30.0
    p0_clk_drift: float = 5.0


@dataclass
class RawMeas:
    """One satellite at one epoch, already corrected by the receiver-side
    pipeline: satellite clock/relativity/TGD, ionosphere and troposphere are
    removed from ``pr``; satellite clock drift is removed from ``rr``.
    ``sat_pos`` is the satellite at transmit time in the ECEF frame of that
    instant (the filter applies the Earth-rotation correction)."""
    prn: int
    sat_pos: np.ndarray
    sat_vel: np.ndarray
    pr: float | None             # m
    rr: float | None             # m/s, = -lambda * Doppler
    sigma_pr: float = 3.0
    sigma_rr: float = 0.1
    sys: str = "G"               # "G" GPS L1 C/A, "E" Galileo E1, "C" BeiDou B1I


@dataclass
class RawStats:
    used_pr: int = 0
    used_rr: int = 0
    rejected_pr: int = 0
    rejected_rr: int = 0
    rejected_prns: list = field(default_factory=list)
    clock_jump_ms: int = 0


class TcEkf:
    def __init__(self, params: TcEkfParams | None = None):
        self.p = params or TcEkfParams()
        self.lla = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([0.707107, 0.0, 0.707107, 0.0])    # nose up, as the C++
        self.ab = np.zeros(3)
        self.wb = np.zeros(3)
        self.clk = np.zeros(3)                   # bias m, drift at 1 g m/s, g-coef m/s/g
        self.isb = {"E": 0.0, "C": 0.0}          # Galileo/BeiDou minus GPS, m
        self.g_excess = 0.0                      # |f|/g - 1 at the last IMU step
        self.P = np.zeros((N, N))
        self.a_est_b = np.zeros(3)
        self.w_est_b = np.zeros(3)
        self.clk_ready = False
        self._rejects = {}
        pp = self.p
        self.Rw = np.diag([pp.a_noise ** 2] * 3 + [pp.w_noise ** 2] * 3
                          + [2 * pp.a_markov_sigma ** 2 / pp.a_markov_tau] * 3
                          + [2 * pp.w_markov_sigma ** 2 / pp.w_markov_tau] * 3
                          + [pp.clk_bias_psd, pp.clk_drift_psd, pp.clk_g_psd])

    # ------------------------------------------------------------- init
    def init(self, lla, v_ned, q, clk_bias=None, clk_drift=None):
        pp = self.p
        self.lla = np.array(lla, float)
        self.v = np.array(v_ned, float)
        self.q = np.array(q, float) / np.linalg.norm(q)
        self.P = np.diag([pp.p0_pos ** 2] * 3 + [pp.p0_vel ** 2] * 3
                         + [pp.p0_att ** 2, pp.p0_att ** 2, pp.p0_hdg ** 2]
                         + [pp.p0_abias ** 2] * 3 + [pp.p0_wbias ** 2] * 3
                         + [pp.p0_clk_bias ** 2, pp.p0_clk_drift ** 2, pp.p0_clk_g ** 2]
                         + [pp.p0_isb ** 2] * 2)
        if clk_bias is not None:
            self.clk = np.array([clk_bias, clk_drift or 0.0, 0.0])
            self.clk_ready = True

    def set_attitude_sigma(self, att_rad, hdg_rad):
        self.P[6, 6] = self.P[7, 7] = att_rad ** 2
        self.P[8, 8] = hdg_rad ** 2

    # ------------------------------------------------------------- propagate
    def propagate(self, acc_frd, gyro_frd_rps, dt):
        """IMU time update, mirroring GpsInsEKF::updateCore + timeUpdate."""
        pp = self.p
        T_ned2b = quat2dcm(self.q)
        T_b2ned = T_ned2b.T
        self.w_est_b = np.asarray(gyro_frd_rps, float) - self.wb
        self.a_est_b = np.asarray(acc_frd, float) + T_ned2b[:, 2] * G - self.ab
        th = self.w_est_b * dt
        ang = np.linalg.norm(th)
        if ang > 1e-8:
            dq = np.concatenate(([math.cos(0.5 * ang)], math.sin(0.5 * ang) / ang * th))
        else:
            dq = np.concatenate(([1.0], 0.5 * th))
        self.q = quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)
        self.v = self.v + T_b2ned @ self.a_est_b * dt
        rew, rns = earth_rad(self.lla[0])
        self.lla = self.lla + dt * np.array([self.v[0] / (rns + self.lla[2]),
                                             self.v[1] / ((rew + self.lla[2]) * math.cos(self.lla[0])),
                                             -self.v[2]])
        F = np.zeros((N, N))
        F[0:3, 3:6] = np.eye(3)
        F[5, 2] = -2 * G / EARTH_RADIUS
        F[3:6, 6:9] = -2 * T_b2ned @ skew(self.a_est_b)
        F[3:6, 9:12] = -T_b2ned
        F[6:9, 6:9] = -skew(self.w_est_b)
        F[6:9, 12:15] = -0.5 * np.eye(3)
        F[9:12, 9:12] = -np.eye(3) / pp.a_markov_tau
        F[12:15, 12:15] = -np.eye(3) / pp.w_markov_tau
        self.g_excess = float(np.linalg.norm(acc_frd) / G - 1.0)
        F[15, 16] = 1.0
        F[15, 17] = self.g_excess
        Gm = np.zeros((N, 15))
        Gm[3:6, 0:3] = -T_b2ned
        Gm[6:9, 3:6] = -0.5 * np.eye(3)
        Gm[9:12, 6:9] = np.eye(3)
        Gm[12:15, 9:12] = np.eye(3)
        Gm[15, 12] = 1.0
        Gm[16, 13] = 1.0
        Gm[17, 14] = 1.0
        Qc = Gm @ self.Rw @ Gm.T
        Qc[18, 18] = Qc[19, 19] = pp.isb_psd
        Qd = dt * Qc + 0.5 * dt * dt * (F @ Qc + Qc @ F.T)
        Phi = np.eye(N) + F * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.clk[0] += (self.clk[1] + self.clk[2] * self.g_excess) * dt
        self._stabilize()

    def propagate_kinematic(self, dt, accel_psd=4.0):
        """GNSS-only time update (no IMU): constant velocity driven by white
        acceleration noise of ``accel_psd`` (m^2/s^3). Attitude and IMU biases
        are frozen and never observed, so their rows are left alone. For
        receiver-only captures (a bench PX1105R, the COCOM rig)."""
        rew, rns = earth_rad(self.lla[0])
        self.lla = self.lla + dt * np.array([self.v[0] / (rns + self.lla[2]),
                                             self.v[1] / ((rew + self.lla[2]) * math.cos(self.lla[0])),
                                             -self.v[2]])
        self.g_excess = 0.0
        F = np.zeros((N, N))
        F[0:3, 3:6] = np.eye(3)
        F[15, 16] = 1.0
        # Exact discrete noise of an integrated random walk. The first-order
        # (dt*Qc + dt^2/2 ...) form the IMU path uses is fine at 1 ms but is
        # not positive definite at a 1 s GNSS epoch.
        Qd = np.zeros((N, N))
        q, qb, qd = accel_psd, self.p.clk_bias_psd, self.p.clk_drift_psd
        for i in range(3):
            Qd[i, i] = q * dt ** 3 / 3
            Qd[i, i + 3] = Qd[i + 3, i] = q * dt ** 2 / 2
            Qd[i + 3, i + 3] = q * dt
        Qd[15, 15] = qb * dt + qd * dt ** 3 / 3
        Qd[15, 16] = Qd[16, 15] = qd * dt ** 2 / 2
        Qd[16, 16] = qd * dt
        Qd[18, 18] = Qd[19, 19] = self.p.isb_psd * dt
        Phi = np.eye(N) + F * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.clk[0] += self.clk[1] * dt
        self._stabilize()

    def freeze_imu_states(self):
        """Zero the attitude and IMU-bias covariance so GNSS-only updates
        cannot move states nothing observes."""
        for i in range(6, 15):
            self.P[i, :] = 0.0
            self.P[:, i] = 0.0
            self.P[i, i] = 1e-12

    def _stabilize(self):
        self.P = 0.5 * (self.P + self.P.T)
        caps = [1e8] * 3 + [1e4] * 3 + [10.0] * 3 + [10.0] * 3 + [1.0] * 3 + [1e12, 1e6, 100.0, 1e6, 1e6]
        for i, c in enumerate(caps):
            if self.P[i, i] > c:
                s = math.sqrt(c / self.P[i, i])
                self.P[i, :] *= s
                self.P[:, i] *= s
            if self.P[i, i] < 1e-12:
                self.P[i, i] = 1e-12

    # ------------------------------------------------------------- inject
    def _inject(self, dx):
        rew, rns = earth_rad(self.lla[0])
        self.lla[2] -= dx[2]
        self.lla[0] += dx[0] / (rns + self.lla[2])
        self.lla[1] += dx[1] / ((rew + self.lla[2]) * math.cos(self.lla[0]))
        self.v += dx[3:6]
        self.ab += dx[9:12]
        self.wb += dx[12:15]
        self.clk += dx[15:18]
        self.isb["E"] += dx[18]
        self.isb["C"] += dx[19]
        dq = np.array([1.0, dx[6], dx[7], dx[8]])
        dq /= np.linalg.norm(dq)
        self.q = quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)

    def _att_gate(self):
        if not self.p.gnss_att_cos4_gate:
            return 1.0
        T = quat2dcm(self.q)
        sp = -T[0, 2]                      # sin(pitch) for an FRD body in NED
        c2 = 1.0 - sp * sp
        return c2 * c2

    def _update(self, H, y, R, att_scale=1.0, gate=None):
        """Vector or scalar update, Joseph form. Returns (applied, nis)."""
        H = np.atleast_2d(H)
        y = np.atleast_1d(y)
        R = np.atleast_2d(R)
        PHt = self.P @ H.T
        S = H @ PHt + R
        Sinv = np.linalg.inv(S)
        nis = float(y @ Sinv @ y)
        if gate is not None and nis > gate:
            return False, nis
        K = PHt @ Sinv
        K[6:9, :] *= att_scale
        dx = K @ y
        IKH = np.eye(N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._inject(dx)
        self._stabilize()
        return True, nis

    # ------------------------------------------------------------- aids
    def update_accel_level(self, acc_frd):
        """Gravity-reference update (GpsInsEKF::accelMeasUpdate)."""
        T = quat2dcm(self.q)
        ag = T[:, 2] * G
        y = (np.asarray(acc_frd, float) - self.ab) + ag
        H = np.zeros((3, N))
        H[0, 7], H[0, 8] = 2 * ag[2], -2 * ag[1]
        H[1, 6], H[1, 8] = -2 * ag[2], 2 * ag[0]
        H[2, 6], H[2, 7] = 2 * ag[1], -2 * ag[0]
        H[0, 9] = H[1, 10] = H[2, 11] = 1.0
        return self._update(H, y, np.eye(3) * self.p.accel_var)

    def update_baro(self, alt_m):
        H = np.zeros((1, N))
        H[0, 2] = -1.0
        return self._update(H, [alt_m - self.lla[2]], [[self.p.baro_var]])

    def update_gnss_fix(self, lla_meas, v_ned_meas, noise_scale=1.0):
        """Loosely coupled fix update, as GpsInsEKF::measUpdate (chi-square
        inflation per block, cos^4 pitch attitude gate)."""
        pp = self.p
        T = t_e2ned(self.lla[0], self.lla[1])
        dp = T @ (lla2ecef(lla_meas) - lla2ecef(self.lla))
        y = np.concatenate([dp, np.asarray(v_ned_meas, float) - self.v])
        H = np.zeros((6, N))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        R = np.diag([pp.fix_pos_sigma_ne ** 2, pp.fix_pos_sigma_ne ** 2, pp.fix_pos_sigma_d ** 2,
                     pp.fix_vel_sigma_ne ** 2, pp.fix_vel_sigma_ne ** 2, pp.fix_vel_sigma_d ** 2])
        R[0:3, 0:3] *= noise_scale ** 2
        R[3:6, 3:6] *= noise_scale
        S = H @ self.P @ H.T + R
        infl = np.ones(6)
        for sl, gate in ((slice(0, 3), 16.27), (slice(3, 5), 13.82), (slice(5, 6), 10.83)):
            Ss = S[sl, sl]
            nis = float(y[sl] @ np.linalg.inv(Ss) @ y[sl])
            if nis > gate:
                infl[sl] = math.sqrt(nis / gate)
        R = R * infl[:, None] * 1.0
        return self._update(H, y, R, att_scale=self._att_gate())

    # ------------------------------------------------------------- raw GNSS
    def receiver_state_ecef(self):
        r = lla2ecef(self.lla)
        T = t_e2ned(self.lla[0], self.lla[1])
        return r, T.T @ self.v, T

    def predict_raw(self, m: RawMeas, r_rx, v_rx):
        return los_predict(m, r_rx, v_rx)

    def init_clock_from(self, meas):
        """Least-squares clock bias/drift at the current position/velocity."""
        r, vr, _ = self.receiver_state_ecef()
        by = {}
        for m in meas:
            if m.pr is not None:
                by.setdefault(m.sys, []).append(m.pr - self.predict_raw(m, r, vr)[0])
        d = [m.rr - self.predict_raw(m, r, vr)[1] for m in meas if m.rr is not None]
        if not by:
            return False
        ref = "G" if "G" in by else next(iter(by))
        b0 = float(np.median(by[ref]))
        self.clk = np.array([b0, float(np.median(d)) if d else 0.0, 0.0])
        for sy in ("E", "C"):
            self.isb[sy] = float(np.median(by[sy])) - b0 if sy in by else 0.0
        self.P[15, :] = self.P[:, 15] = 0.0
        self.P[16, :] = self.P[:, 16] = 0.0
        self.P[17, :] = self.P[:, 17] = 0.0
        self.P[17, 17] = self.p.p0_clk_g ** 2
        self.P[15, 15] = self.p.p0_clk_bias ** 2 + self.P[2, 2]
        self.P[16, 16] = self.p.p0_clk_drift ** 2
        for i in (18, 19):
            self.P[i, :] = self.P[:, i] = 0.0
            self.P[i, i] = self.p.p0_isb ** 2
        self.clk_ready = True
        return True

    def update_gnss_raw(self, meas: list[RawMeas]) -> RawStats:
        """Tightly coupled update: one scalar update per pseudorange and per
        range rate, each gated on its own innovation, so one bad satellite is
        dropped without losing the rest."""
        st = RawStats()
        if not self.clk_ready and not self.init_clock_from(meas):
            return st
        # Receiver clock steering: SkyTraq (0xE5 measurement-indicator bits 1/2)
        # and u-blox (RXM-RAWX clkReset) step the receiver clock by whole
        # milliseconds, and every pseudorange jumps by k * c * 1 ms at once --
        # 299.8 km per step, measured on the PX1105R 2026-09-26 (its clock
        # drifts 188 m/s, so one step every ~27 min). The satellites all agree
        # on the jump, so shift the clock state rather than gate them all out.
        r, vr, _ = self.receiver_state_ecef()
        inn = [m.pr - (self.predict_raw(m, r, vr)[0] + self.clk[0] + self.isb.get(m.sys, 0.0))
               for m in meas if m.pr is not None]
        if inn:
            med = float(np.median(inn))
            ms = C_LIGHT * 1e-3
            k = int(round(med / ms))
            if k != 0 and abs(med - k * ms) < 0.1 * ms:
                self.clk[0] += k * ms
                st.clock_jump_ms = k
        att = self._att_gate()
        for m in meas:
            for kind in ("pr", "rr"):
                z = m.pr if kind == "pr" else m.rr
                if z is None:
                    continue
                r, vr, T = self.receiver_state_ecef()
                rho, rrate, u = self.predict_raw(m, r, vr)
                u_ned = T @ u
                H = np.zeros((1, N))
                if kind == "pr":
                    H[0, 0:3] = -u_ned
                    H[0, 15] = 1.0
                    if m.sys in ISB_INDEX:
                        H[0, ISB_INDEX[m.sys]] = 1.0
                    y = z - (rho + self.clk[0] + self.isb.get(m.sys, 0.0))
                    R = m.sigma_pr ** 2
                else:
                    H[0, 3:6] = -u_ned
                    H[0, 16] = 1.0
                    H[0, 17] = self.g_excess
                    y = z - (rrate + self.clk[1] + self.clk[2] * self.g_excess)
                    R = m.sigma_rr ** 2
                key = (m.sys, m.prn, kind)
                gate = self.p.raw_gate_chi2
                if self._rejects.get(key, 0) >= self.p.raw_reject_persist:
                    gate = None          # persistent disagreement: the filter is the suspect
                ok, nis = self._update(H, [y], [[R * (max(1.0, nis_scale(self, H, R, y, key)))]],
                                       att_scale=att, gate=gate)
                self._rejects[key] = 0 if ok else self._rejects.get(key, 0) + 1
                if ok:
                    setattr(st, f"used_{kind}", getattr(st, f"used_{kind}") + 1)
                else:
                    setattr(st, f"rejected_{kind}", getattr(st, f"rejected_{kind}") + 1)
                    st.rejected_prns.append((m.prn, kind))
        return st

    # ------------------------------------------------------------- outputs
    def ned_from(self, ref_lla):
        """Position in the local NED frame of ``ref_lla`` (m)."""
        return t_e2ned(ref_lla[0], ref_lla[1]) @ (lla2ecef(self.lla) - lla2ecef(ref_lla))

    def sigmas(self):
        return np.sqrt(np.clip(np.diag(self.P), 0, None))
