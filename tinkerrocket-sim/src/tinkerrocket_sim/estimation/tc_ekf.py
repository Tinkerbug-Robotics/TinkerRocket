"""Standalone GNSS/INS EKF that can take raw pseudorange and Doppler.

A Python mirror of the flight filter (``TR_GpsInsEKF``) -- same nominal state,
same error-state layout and frame conventions, same IMU mechanization, pad
levelling, baro and GNSS-fix updates -- extended with two receiver-clock states
so GNSS can enter as per-satellite measurements instead of a finished fix:

    error state (24)
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

      20    Doppler clock-rate offset (m/s)            <- off unless p0 > 0

The rate the Doppler reports for the clock minus the rate at which the code's
clock bias actually moves. One oscillator drives both in a receiver, so this is
~0 on a real sky; the HackRF COCOM rig shifts the carrier alone (its LO and
sample clock are synthesised separately): +4.2 m/s on the PX1105R, 2026-09-27.

      21    Doppler lag (s)                            <- off unless p0 > 0

How late the reported Doppler is: the range rate it reports is the one of that
long before the epoch's tag (see below). A constant of the receiver's setup,
but not the same in every setup, so it can be estimated.

      22:25 CLONE of the position error at the previous carrier epoch (NED)
      25    CLONE of the clock bias at that epoch

The clone is what makes the carrier-phase delta-range exact (stochastic
cloning): a carrier phase is a range with an unknown whole-cycle offset, so only
its change between two epochs is usable, and that change is predicted from the
position NOW and the position THEN. The clone keeps "then" in the state with
its full covariance, has no dynamics of its own, and is refreshed after every
carrier epoch (``reclone``).

``update_gnss_fix`` is the loosely coupled path the flight code uses today;
``update_gnss_raw`` is the tightly coupled one. Keeping both in one class means
a comparison between them changes nothing but the GNSS update.

Without an IMU (``propagate_kinematic``) the filter runs a constant-velocity
model, or with ``enable_kinematic_acceleration`` a constant-acceleration one
whose NED acceleration takes the accelerometer-bias rows 9:12 (unused without
an IMU).

A receiver may report its Doppler late: the PX1105R's is the range rate of
~0.22 s before its time tag, while its pseudorange is on time (COCOM rig,
2026-09-27; 0.05-0.2 s with its power save on). The Doppler is predicted at that
earlier instant -- from the IMU-propagated velocity history, or from the
kinematic acceleration -- with the lag fixed (``rr_lag_s``) or estimated
(``p0_rr_lag`` > 0), observable whenever the vehicle accelerates.

The mechanization mirrors the flight filter: constant gravity, no Earth rate.
``gravity_model="wgs84"`` (normal gravity by latitude and height) and
``earth_rate=True`` (Coriolis, transport rate, Earth rate on the gyros) are what
a flight to 80 km needs: gravity is 2.5 % weaker there, and Coriolis at 1 km/s
is 0.15 m/s^2.

Body frame FRD, world frame NED, ``q`` scalar-first with Quat2DCM(q) = NED->body,
exactly as TR_GpsInsEKF.h. Not flight code: numpy, float64, dense covariance.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

G = 9.807
ECC2 = 0.0066943799901
EARTH_RADIUS = 6378137.0
OMGE = 7.2921151467e-5
C_LIGHT = 299792458.0
N = 26
N_CORE = 22
RATE_OFS = 20               # Doppler clock-rate offset
RR_LAG = 21                 # Doppler lag
KIN_ACC = slice(9, 12)      # NED acceleration in the kinematic constant-acceleration mode
CLONE = [22, 23, 24, 25]
CLONED = [0, 1, 2, 15]      # what the clone copies: position, clock bias
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


def quat_from_accel_heading(acc_frd, heading_rad):
    """Pad attitude seed, as TR_Orientation's quatFromAccelHeading (the flight
    computer's EKF init): pitch from the X component of a stationary specific
    force, roll from Y/Z (left at zero within 10 deg of vertical, where it is
    ill-conditioned), yaw from the known pad heading. Scalar-first, the
    quaternion Quat2DCM takes."""
    ax, ay, az = (float(a) for a in acc_frd)
    g = math.sqrt(ax * ax + ay * ay + az * az)
    if g < 0.1:
        g = G
    pitch = math.asin(min(1.0, max(-1.0, ax / g)))
    roll = math.atan2(-ay, -az) if abs(pitch) < math.radians(80.0) else 0.0
    cy, sy = math.cos(0.5 * heading_rad), math.sin(0.5 * heading_rad)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


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


def earth_rates_ned(lla, v_ned):
    """(Earth rate, transport rate) in NED (rad/s): the rotation of the Earth,
    and of the local NED frame as the vehicle moves over it."""
    lat, h = lla[0], lla[2]
    rew, rns = earth_rad(lat)
    w_ie = OMGE * np.array([math.cos(lat), 0.0, -math.sin(lat)])
    w_en = np.array([v_ned[1] / (rew + h), -v_ned[0] / (rns + h),
                     -v_ned[1] * math.tan(lat) / (rew + h)])
    return w_ie, w_en


WGS84_GAMMA_E = 9.7803253359          # normal gravity on the equator, m/s^2
WGS84_K = 0.00193185265241             # Somigliana's constant
WGS84_F = 1.0 / 298.257223563          # flattening
WGS84_M = 0.00344978650684             # omega^2 a^2 b / GM


def gravity_wgs84(lat, h):
    """WGS84 normal gravity (NED, m/s^2): gravitation plus the centrifugal part,
    which is what a mechanization in the rotating Earth frame needs.
    Somigliana's formula on the ellipsoid, the second-order height correction
    (NIMA TR8350.2 eq. 4-1 and 4-3; the next term is ~1e-4 m/s^2 at 80 km),
    and the small north component above the ellipsoid (Groves eq. 2.140)."""
    s2 = math.sin(lat) ** 2
    g0 = WGS84_GAMMA_E * (1.0 + WGS84_K * s2) / math.sqrt(1.0 - ECC2 * s2)
    gd = g0 * (1.0 - 2.0 / EARTH_RADIUS * (1.0 + WGS84_F + WGS84_M - 2.0 * WGS84_F * s2) * h
               + 3.0 / EARTH_RADIUS ** 2 * h * h)
    return np.array([-8.08e-9 * h * math.sin(2.0 * lat), 0.0, gd])


def los_predict(m: "RawMeas", r_rx, v_rx):
    """Predicted pseudorange (without receiver clock), range rate (without
    drift) and line-of-sight unit vector, with the Earth-rotation (Sagnac)
    correction: the satellite position is at transmit time in the ECEF frame
    of that instant, so it is turned by omega_e * travel time."""
    rho, rate, u, _ = _los(m, r_rx, v_rx)
    return rho, rate, u


def _los(m, r_rx, v_rx):
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
    return rho, float(u @ (vs_r - v_rx)), u, vs_r


def sat_range_accel(m: "RawMeas", r_rx, v_rx):
    """The range acceleration a still receiver would see from the satellite's
    own motion (m/s^2): its acceleration along the line of sight plus the
    turning of the line of sight, (|dv|^2 - (u.dv)^2)/rho. Up to ~0.2 m/s^2."""
    rho, rate, u, vs_r = _los(m, r_rx, v_rx)
    dv = vs_r - np.asarray(v_rx, float)
    a_s = np.zeros(3) if m.sat_acc is None else np.asarray(m.sat_acc, float)
    return float(u @ a_s) + (float(dv @ dv) - rate * rate) / rho


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


def chi2_999(n):
    """0.999 quantile of chi-square with n degrees of freedom, Wilson-Hilferty
    (11.2 at n = 1 where the exact value is 10.8, within 1 % from n = 5)."""
    a = 2.0 / (9.0 * n)
    return n * (1.0 - a + 3.090 * math.sqrt(a)) ** 3


def corr_inflation(dt, tau):
    """Variance factor for a first-order Gauss-Markov error sampled every dt
    and fused as if white: successive samples correlate with rho = exp(-dt/tau),
    so each carries (1-rho)/(1+rho) of an independent sample's information.
    ~2 tau/dt when sampled fast, -> 1 when samples are far apart."""
    if tau <= 0 or dt <= 0:
        return 1.0
    rho = math.exp(-dt / tau)
    return (1.0 + rho) / (1.0 - rho) if rho < 1.0 - 1e-12 else 1e12


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
    # How much of a GNSS update's correction reaches the attitude (_att_gate):
    # "cos4" as the flight filter, "heading" all but the unobservable part, "none"
    gnss_att_gate: str = "cos4"
    # initial sigmas
    p0_pos: float = 10.0
    p0_vel: float = 1.0
    p0_att: float = 0.34906
    p0_hdg: float = 3.14159
    p0_abias: float = 0.981
    p0_wbias: float = 0.01745
    p0_clk_bias: float = 30.0
    p0_clk_drift: float = 5.0
    # Doppler clock-rate offset (state 20): 0 leaves the state out
    p0_rate_ofs: float = 0.0         # m/s
    rate_ofs_psd: float = 0.0        # (m/s)^2/s
    # The reported Doppler is the range rate this long before the epoch's tag (s):
    # the value, or the initial value when p0_rr_lag > 0 makes it a state
    rr_lag_s: float = 0.0
    p0_rr_lag: float = 0.0           # s
    rr_lag_psd: float = 1e-6         # s^2/s
    # Kinematic constant-acceleration mode: initial acceleration sigma (m/s^2).
    # kin_gravity: the state is the non-gravitational acceleration and gravity
    # (inverse-square) is added, so a gap is bridged ballistically;
    # kin_acc_tau > 0 decays that acceleration (Singer) toward zero in a gap.
    p0_kin_acc: float = 10.0
    kin_gravity: bool = False
    kin_acc_tau: float = 0.0
    # INS mechanization. "const": the flight filter's fixed G. "wgs84": normal
    # gravity by latitude and height (gravity_wgs84).
    gravity_model: str = "const"
    earth_rate: bool = False         # Coriolis, transport rate, Earth rate on the gyros


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
    sigma_pr: float = 3.0        # white part (m)
    sigma_rr: float = 0.1        # white part (m/s)
    sys: str = "G"               # "G" GPS L1 C/A, "E" Galileo E1, "C" BeiDou B1I
    # Time-correlated part, first-order Gauss-Markov: multipath, atmosphere and
    # orbit errors wander over seconds to minutes, so successive epochs are not
    # independent. 0 = treat as white (the default, and the sim model's case).
    sigma_pr_corr: float = 0.0   # m
    tau_pr: float = 15.0         # s
    tau_rr: float = 0.1          # s
    # Carrier range (lambda * accumulated cycles, corrected like the
    # pseudorange but with the ionosphere's opposite sign). Its absolute value
    # carries an unknown whole-cycle offset; only its change between epochs is
    # used (update_carrier). cr_slip marks a possible break in the count.
    cr: float | None = None      # m
    cr_slip: bool = False
    sigma_cr: float = 0.003      # m, white per-epoch phase noise
    # Carrier errors that grow with the differencing interval (tracking noise
    # correlated over a few tenths of a second, then a slow random walk:
    # multipath, ionosphere): a difference over dt has variance
    # 2 sigma_cr^2 + 2 sigma_cr_corr^2 (1 - exp(-dt/tau_cr)) + q_cr dt.
    sigma_cr_corr: float = 0.0   # m
    tau_cr: float = 0.3          # s
    q_cr: float = 0.0            # m^2/s
    # Satellite acceleration (ECEF, m/s^2), for predicting a late Doppler
    sat_acc: np.ndarray | None = None


@dataclass
class RawStats:
    used_pr: int = 0
    used_rr: int = 0
    rejected_pr: int = 0
    rejected_rr: int = 0
    rejected_prns: list = field(default_factory=list)
    clock_jump_ms: int = 0
    nis_pr: list = field(default_factory=list)
    nis_rr: list = field(default_factory=list)
    used_cr: int = 0
    rejected_cr: int = 0
    nis_cr: list = field(default_factory=list)
    cr_clock_jump_ms: int = 0
    cr_common_rejected: int = 0      # carrier dropped by the joint test (a shared error)
    cr_joint: tuple = (0.0, 0)       # the joint test's (NIS, degrees of freedom)


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
        self.rate_ofs = 0.0                      # Doppler clock rate minus code clock rate, m/s
        self.rr_lag = self.p.rr_lag_s            # Doppler lag, s
        self.g_excess = 0.0                      # |f|/g - 1 at the last IMU step
        self.P = np.zeros((N, N))
        self.a_est_b = np.zeros(3)
        self.w_est_b = np.zeros(3)
        self.a_ned = np.zeros(3)                 # kinematic acceleration at the last IMU step
        self.f_b = np.zeros(3)                   # bias-corrected specific force at the last IMU step
        self.kin_acc = False                     # rows 9:12 hold the kinematic acceleration
        self.acc = np.zeros(3)                   # that acceleration, NED m/s^2
        self._dv_sum = np.zeros(3)               # IMU velocity increments, summed (no updates)
        self._dv_hist = deque(maxlen=8192)       # (t, _dv_sum) after each IMU step
        self.clk_ready = False
        self._rejects = {}
        self._last_upd = {}                      # (sys, prn, kind) -> filter time of last update
        self.clone_lla = None                    # nominal position at the clone epoch
        self.clone_b = 0.0                       # nominal clock bias at the clone epoch
        self.clone_t = 0.0                       # filter time of the clone epoch
        self._clone_epoch = 0
        self._prev_cr = {}                       # (sys, prn) -> (clone epoch, carrier range, RawMeas)
        self.t = 0.0                             # filter time, advanced by the propagations
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
                         + [pp.p0_isb ** 2] * 2 + [pp.p0_rate_ofs ** 2, pp.p0_rr_lag ** 2]
                         + [0.0] * (N - N_CORE))
        self.rr_lag = pp.rr_lag_s
        if clk_bias is not None:
            self.clk = np.array([clk_bias, clk_drift or 0.0, 0.0])
            self.clk_ready = True
        self.reclone()

    def set_quaternion(self, q):
        """GpsInsEKF::setQuaternion: take the attitude as known. The attitude
        variance goes to 1e-6 and its cross-covariances to zero, which is how
        the flight computer seeds the filter on the pad (with
        quat_from_accel_heading). Levelling can then only move the accelerometer
        bias, so the unobservable pad split between tilt and horizontal bias is
        settled by this seed, not by whatever noise the first velocity updates
        carry."""
        self.q = np.array(q, float) / np.linalg.norm(q)
        self.P[6:9, :] = 0.0
        self.P[:, 6:9] = 0.0
        self.P[6, 6] = self.P[7, 7] = self.P[8, 8] = 1e-6

    def seed_gyro_bias(self, gyro_mean):
        """Gyro bias from the mean of a still gyro, as the flight computer seeds
        it on the pad (#297). A still gyro also reads the Earth's rotation; when
        the mechanization models that (``earth_rate``), it is not bias and comes
        out here -- otherwise it is subtracted twice. Call after the attitude
        seed (set_quaternion)."""
        wb = np.asarray(gyro_mean, float)
        if self.p.earth_rate:
            w_ie, _ = earth_rates_ned(self.lla, np.zeros(3))
            wb = wb - quat2dcm(self.q) @ w_ie
        self.wb = wb.copy()

    # ------------------------------------------------------------- propagate
    def gravity_ned(self, model=None):
        """Gravity (NED, m/s^2) the mechanization uses at the current position."""
        if (model or self.p.gravity_model) == "wgs84":
            return gravity_wgs84(self.lla[0], self.lla[2])
        return np.array([0.0, 0.0, G])

    def kin_total_acc(self):
        """The kinematic mode's total NED acceleration: the state, plus gravity
        when the state is the non-gravitational part."""
        if not self.kin_acc:
            return np.zeros(3)
        if self.p.kin_gravity:
            return self.acc + self.gravity_ned(model="wgs84")
        return self.acc.copy()

    def propagate(self, acc_frd, gyro_frd_rps, dt):
        """IMU time update, mirroring GpsInsEKF::updateCore + timeUpdate."""
        pp = self.p
        T_ned2b = quat2dcm(self.q)
        T_b2ned = T_ned2b.T
        g_ned = self.gravity_ned()
        self.w_est_b = np.asarray(gyro_frd_rps, float) - self.wb
        if pp.earth_rate:
            w_ie, w_en = earth_rates_ned(self.lla, self.v)
            self.w_est_b = self.w_est_b - T_ned2b @ (w_ie + w_en)
        self.f_b = np.asarray(acc_frd, float) - self.ab
        self.a_est_b = self.f_b + T_ned2b @ g_ned
        th = self.w_est_b * dt
        ang = np.linalg.norm(th)
        if ang > 1e-8:
            dq = np.concatenate(([math.cos(0.5 * ang)], math.sin(0.5 * ang) / ang * th))
        else:
            dq = np.concatenate(([1.0], 0.5 * th))
        self.q = quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)
        a_ned = T_b2ned @ self.a_est_b
        if pp.earth_rate:
            a_ned = a_ned - np.cross(2.0 * w_ie + w_en, self.v)
        self.a_ned = a_ned
        self.v = self.v + a_ned * dt
        self._dv_sum = self._dv_sum + a_ned * dt
        rew, rns = earth_rad(self.lla[0])
        self.lla = self.lla + dt * np.array([self.v[0] / (rns + self.lla[2]),
                                             self.v[1] / ((rew + self.lla[2]) * math.cos(self.lla[0])),
                                             -self.v[2]])
        F = np.zeros((N, N))
        F[0:3, 3:6] = np.eye(3)
        F[5, 2] = +2 * g_ned[2] / EARTH_RADIUS    # x2 is delta(down): +2g/R, as the C++ since #1154
        if pp.earth_rate:
            F[3:6, 3:6] = -skew(2.0 * w_ie + w_en)
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
        Qc[RATE_OFS, RATE_OFS] = pp.rate_ofs_psd
        if pp.p0_rr_lag > 0:
            Qc[RR_LAG, RR_LAG] = pp.rr_lag_psd
        Qd = dt * Qc + 0.5 * dt * dt * (F @ Qc + Qc @ F.T)
        Phi = np.eye(N) + F * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.clk[0] += (self.clk[1] + self.clk[2] * self.g_excess) * dt
        self.t += dt
        self._dv_hist.append((self.t, self._dv_sum.copy()))
        self._stabilize()

    def enable_kinematic_acceleration(self, acc_ned=None):
        """GNSS-only constant-acceleration mode: rows 9:12 carry the NED
        acceleration (there are no accelerometer biases without an IMU). Call
        after freeze_imu_states; then propagate_kinematic takes a jerk PSD."""
        self.kin_acc = True
        self.acc = np.zeros(3) if acc_ned is None else np.array(acc_ned, float)
        self.P[KIN_ACC, :] = 0.0
        self.P[:, KIN_ACC] = 0.0
        for i in range(9, 12):
            self.P[i, i] = self.p.p0_kin_acc ** 2

    def propagate_kinematic(self, dt, accel_psd=4.0, jerk_psd=0.0):
        """GNSS-only time update (no IMU). Constant velocity driven by white
        acceleration noise of ``accel_psd`` (m^2/s^3); after
        ``enable_kinematic_acceleration``, constant acceleration driven by
        white jerk of ``jerk_psd`` (m^2/s^5) as well. Attitude and IMU biases
        are frozen and never observed, so their rows are left alone. For
        receiver-only captures (a bench PX1105R, the COCOM rig)."""
        a = self.kin_total_acc()
        dp = self.v * dt + 0.5 * a * dt * dt
        rew, rns = earth_rad(self.lla[0])
        self.lla = self.lla + np.array([dp[0] / (rns + self.lla[2]),
                                        dp[1] / ((rew + self.lla[2]) * math.cos(self.lla[0])),
                                        -dp[2]])
        self.v = self.v + a * dt
        self.a_ned = a
        self.g_excess = 0.0
        Phi = np.eye(N)
        Phi[0:3, 3:6] = dt * np.eye(3)
        Phi[15, 16] = dt
        if self.kin_acc and self.p.kin_acc_tau > 0:
            decay = math.exp(-dt / self.p.kin_acc_tau)
            self.acc = self.acc * decay
            Phi[KIN_ACC, KIN_ACC] = decay * np.eye(3)
        # Exact discrete noise of an integrated random walk. The first-order
        # (dt*Qc + dt^2/2 ...) form the IMU path uses is fine at 1 ms but is
        # not positive definite at a 1 s GNSS epoch.
        Qd = np.zeros((N, N))
        q, qb, qd = accel_psd, self.p.clk_bias_psd, self.p.clk_drift_psd
        for i in range(3):
            Qd[i, i] = q * dt ** 3 / 3
            Qd[i, i + 3] = Qd[i + 3, i] = q * dt ** 2 / 2
            Qd[i + 3, i + 3] = q * dt
        if self.kin_acc:
            Phi[3:6, 9:12] = dt * np.eye(3)
            Phi[0:3, 9:12] = 0.5 * dt * dt * np.eye(3)
            j = jerk_psd
            for i in range(3):
                k = 9 + i
                Qd[i, i] += j * dt ** 5 / 20
                Qd[i, i + 3] += j * dt ** 4 / 8
                Qd[i + 3, i] = Qd[i, i + 3]
                Qd[i, k] = Qd[k, i] = j * dt ** 3 / 6
                Qd[i + 3, i + 3] += j * dt ** 3 / 3
                Qd[i + 3, k] = Qd[k, i + 3] = j * dt ** 2 / 2
                Qd[k, k] = j * dt
        Qd[15, 15] = qb * dt + qd * dt ** 3 / 3
        Qd[15, 16] = Qd[16, 15] = qd * dt ** 2 / 2
        Qd[16, 16] = qd * dt
        Qd[18, 18] = Qd[19, 19] = self.p.isb_psd * dt
        Qd[RATE_OFS, RATE_OFS] = self.p.rate_ofs_psd * dt
        if self.p.p0_rr_lag > 0:
            Qd[RR_LAG, RR_LAG] = self.p.rr_lag_psd * dt
        self.P = Phi @ self.P @ Phi.T + Qd
        self.clk[0] += self.clk[1] * dt
        self.t += dt
        self._stabilize()

    def velocity_change(self, tau):
        """v(now) - v(now - tau), NED, for a measurement that is tau late: from
        the IMU's own velocity increments when they reach back that far (no
        filter corrections in them), else the acceleration estimate times tau.
        Returns (dv, True if it came from the kinematic acceleration state)."""
        h, t0 = self._dv_hist, self.t - tau
        if len(h) > 1 and h[0][0] <= t0 and h[-1][0] >= self.t - 1e-9:
            for k in range(len(h) - 1, 0, -1):
                if h[k - 1][0] <= t0:
                    (ta, sa), (tb, sb) = h[k - 1], h[k]
                    w = (t0 - ta) / (tb - ta) if tb > ta else 0.0
                    return h[-1][1] - (sa + w * (sb - sa)), False
        a = self.kin_total_acc() if self.kin_acc else self.a_ned
        return a * tau, self.kin_acc

    def freeze_imu_states(self):
        """Zero the attitude and IMU-bias covariance so GNSS-only updates
        cannot move states nothing observes."""
        for i in range(6, 15):
            self.P[i, :] = 0.0
            self.P[:, i] = 0.0
            self.P[i, i] = 1e-12

    def _stabilize(self):
        self.P = 0.5 * (self.P + self.P.T)
        caps = ([1e8] * 3 + [1e4] * 3 + [10.0] * 3 + ([1e5] if self.kin_acc else [10.0]) * 3 + [1.0] * 3
                + [1e12, 1e6, 100.0, 1e6, 1e6, 1e4, 1.0] + [1e8] * 3 + [1e12])
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
        if self.kin_acc:
            self.acc += dx[KIN_ACC]
        else:
            self.ab += dx[9:12]
        self.wb += dx[12:15]
        self.clk += dx[15:18]
        self.isb["E"] += dx[18]
        self.isb["C"] += dx[19]
        self.rate_ofs += dx[RATE_OFS]
        self.rr_lag += dx[RR_LAG]
        if self.clone_lla is not None:
            cn, ce, cd, cb = CLONE
            rew_c, rns_c = earth_rad(self.clone_lla[0])
            self.clone_lla[2] -= dx[cd]
            self.clone_lla[0] += dx[cn] / (rns_c + self.clone_lla[2])
            self.clone_lla[1] += dx[ce] / ((rew_c + self.clone_lla[2]) * math.cos(self.clone_lla[0]))
            self.clone_b += dx[cb]
        dq = np.array([1.0, dx[6], dx[7], dx[8]])
        dq /= np.linalg.norm(dq)
        self.q = quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)

    def _att_gate(self):
        """What a GNSS update may do to the attitude, as a 3x3 applied to the
        attitude rows of its gain. GNSS sees attitude only through the specific
        force: a tilt points thrust or drag sideways and the velocity drifts by
        tilt x |f|. Rotation about f itself (heading, when the thrust is vertical)
        it never sees, and in zero g it sees nothing.
          "cos4"     the flight filter: everything scaled by cos^4 pitch, which
                     is ~0 when vertical -- tilt included
          "heading"  only the rotation about f (about the vertical near zero g)
                     removed; tilt corrections kept
          "none"     everything kept"""
        mode = self.p.gnss_att_gate
        if mode == "none":
            return np.eye(3)
        T = quat2dcm(self.q)
        if mode == "heading":
            n = float(np.linalg.norm(self.f_b))
            e = self.f_b / n if n > 0.5 * G else T[:, 2]
            return np.eye(3) - np.outer(e, e)
        sp = -T[0, 2]                      # sin(pitch) for an FRD body in NED
        c2 = 1.0 - sp * sp
        return c2 * c2 * np.eye(3)

    def _update(self, H, y, R, att_scale=None, gate=None):
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
        if att_scale is not None:
            K[6:9, :] = att_scale @ K[6:9, :]
        dx = K @ y
        IKH = np.eye(N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self._inject(dx)
        self._stabilize()
        return True, nis

    # ------------------------------------------------------------- aids
    def update_accel_level(self, acc_frd):
        """Gravity-reference update (GpsInsEKF::accelMeasUpdate). Referenced to
        the mechanization's own gravity: levelling against a constant G while
        propagating with WGS84 parks the difference (0.03 m/s^2 at the equator)
        in the accelerometer bias, which then acts as a real acceleration."""
        T = quat2dcm(self.q)
        ag = T @ self.gravity_ned()
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
        self.reclone()
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
        # A late Doppler is the range rate at t - tau: the receiver velocity
        # then, and the satellite's own range acceleration backed out.
        tau = max(0.0, self.rr_lag)
        lag_state = self.p.p0_rr_lag > 0
        dv_lag, lag_on_acc = self.velocity_change(tau) if tau > 0 else (np.zeros(3), False)
        a_rx = None                              # receiver acceleration, ECEF, for the lag state's row
        if lag_state:
            _, _, T_now = self.receiver_state_ecef()
            a_rx = T_now.T @ (self.kin_total_acc() if self.kin_acc else self.a_ned)
        for m in meas:
            for kind in ("pr", "rr"):
                z = m.pr if kind == "pr" else m.rr
                if z is None:
                    continue
                r, vr, T = self.receiver_state_ecef()
                rdd_sat = 0.0
                if kind == "rr" and (tau > 0 or lag_state):
                    v_lag = vr - T.T @ dv_lag
                    rho, rrate, u = self.predict_raw(m, r, v_lag)
                    rdd_sat = sat_range_accel(m, r, v_lag)
                    rrate -= tau * rdd_sat
                else:
                    rho, rrate, u = self.predict_raw(m, r, vr)
                u_ned = T @ u
                H = np.zeros((1, N))
                key = (m.sys, m.prn, kind)
                dt_i = self.t - self._last_upd.get(key, -1e9)
                if kind == "pr":
                    H[0, 0:3] = -u_ned
                    H[0, 15] = 1.0
                    if m.sys in ISB_INDEX:
                        H[0, ISB_INDEX[m.sys]] = 1.0
                    y = z - (rho + self.clk[0] + self.isb.get(m.sys, 0.0))
                    R = m.sigma_pr ** 2 + m.sigma_pr_corr ** 2 * corr_inflation(dt_i, m.tau_pr)
                else:
                    H[0, 3:6] = -u_ned
                    if lag_on_acc:
                        H[0, KIN_ACC] = tau * u_ned
                    H[0, 16] = 1.0
                    H[0, 17] = self.g_excess
                    H[0, RATE_OFS] = 1.0
                    if lag_state:            # d(range rate at t - tau)/d tau = -range acceleration
                        H[0, RR_LAG] = -(rdd_sat - float(u @ a_rx))
                    y = z - (rrate + self.clk[1] + self.clk[2] * self.g_excess + self.rate_ofs)
                    R = m.sigma_rr ** 2 * corr_inflation(dt_i, m.tau_rr)
                gate = self.p.raw_gate_chi2
                if self._rejects.get(key, 0) >= self.p.raw_reject_persist:
                    gate = None          # persistent disagreement: the filter is the suspect
                ok, nis = self._update(H, [y], [[R * (max(1.0, nis_scale(self, H, R, y, key)))]],
                                       att_scale=att, gate=gate)
                self._rejects[key] = 0 if ok else self._rejects.get(key, 0) + 1
                (st.nis_pr if kind == "pr" else st.nis_rr).append(nis)
                if ok:
                    self._last_upd[key] = self.t
                    setattr(st, f"used_{kind}", getattr(st, f"used_{kind}") + 1)
                else:
                    setattr(st, f"rejected_{kind}", getattr(st, f"rejected_{kind}") + 1)
                    st.rejected_prns.append((m.prn, kind))
        return st

    # ------------------------------------------------------------- carrier
    def reclone(self):
        """Copy position and clock bias into the clone, with their covariance:
        the clone starts perfectly correlated with the state it copies."""
        self.clone_lla = self.lla.copy()
        self.clone_b = float(self.clk[0])
        self.clone_t = self.t
        P = self.P
        P[CLONE, :] = P[CLONED, :]
        P[:, CLONE] = P[:, CLONED]
        P[np.ix_(CLONE, CLONE)] = P[np.ix_(CLONED, CLONED)]
        self._clone_epoch += 1

    def update_carrier(self, meas: list[RawMeas], stats: RawStats | None = None) -> RawStats:
        """Carrier-phase delta-range: for every satellite whose count ran
        unbroken since the clone epoch, fuse

            cr(now) - cr(then) = [rho(now) + b(now)] - [rho(then) + b(then)]

        with the geometry evaluated at the current position and at the clone.
        The whole-cycle offset cancels, and so do inter-system biases. Its
        noise grows with the time since the clone (RawMeas: white, correlated
        and random-walk parts).

        The epoch goes in as one vector update. A satellite-by-satellite gate
        would let the first slipped satellite through: the clock can explain
        19 cm on its own, and every clean satellite after it would then fail.
        So the bad satellite is found first, with Baarda's w-test
        w_i = (S^-1 y)_i / sqrt((S^-1)_ii), the worst removed while it fails,
        and the gate is never relaxed. A slip is a one-epoch event in the
        difference; the pair after it is clean again. An error shared by
        every satellite passes that test (the clock would take it), so what is
        left must also pass a joint chi-square test, or the epoch's carrier is
        dropped. A whole-millisecond receiver clock step that the carrier did
        not share is taken out before either. Ends by storing this epoch's
        carrier and re-cloning."""
        st = stats if stats is not None else RawStats()
        rows = []
        if self.clone_lla is not None and self.clk_ready:
            r_now, _, T_now = self.receiver_state_ecef()
            r_then = lla2ecef(self.clone_lla)
            T_then = t_e2ned(self.clone_lla[0], self.clone_lla[1])
            for m in meas:
                if m.cr is None or m.cr_slip:
                    continue
                prev = self._prev_cr.get((m.sys, m.prn))
                if prev is None or prev[0] != self._clone_epoch:
                    continue
                _, cr_then, m_then = prev
                rho_now, _, u_now = self.predict_raw(m, r_now, np.zeros(3))
                rho_then, _, u_then = self.predict_raw(m_then, r_then, np.zeros(3))
                dt = self.t - self.clone_t
                h = np.zeros(N)
                h[0:3] = -(T_now @ u_now)
                h[15] = 1.0
                h[CLONE[:3]] = T_then @ u_then
                h[CLONE[3]] = -1.0
                h[RATE_OFS] = dt             # the carrier runs on the Doppler's clock rate
                y = (m.cr - cr_then) - ((rho_now + self.clk[0]) - (rho_then + self.clone_b)
                                        + self.rate_ofs * dt)
                R = (m.sigma_cr ** 2 + m_then.sigma_cr ** 2 + m.q_cr * dt
                     + 2.0 * m.sigma_cr_corr ** 2 * (1.0 - math.exp(-dt / max(m.tau_cr, 1e-6))))
                rows.append((m, h, y, R))
        if rows:
            H = np.array([r[1] for r in rows])
            y = np.array([r[2] for r in rows])
            R = np.array([r[3] for r in rows])
            k_ms = round(float(np.median(y)) / (C_LIGHT * 1e-3))
            y = y - k_ms * C_LIGHT * 1e-3
            st.cr_clock_jump_ms = k_ms
            keep = np.ones(len(rows), bool)
            gate = self.p.raw_gate_chi2
            while keep.any():
                idx = np.flatnonzero(keep)
                Si = np.linalg.inv(H[idx] @ self.P @ H[idx].T + np.diag(R[idx]))
                w2 = (Si @ y[idx]) ** 2 / np.diag(Si)
                j = int(np.argmax(w2))
                if w2[j] <= gate:
                    break
                keep[idx[j]] = False
                st.rejected_cr += 1
                st.rejected_prns.append((rows[idx[j]][0].prn, "cr"))
            if keep.any():
                st.cr_joint = (float(y[idx] @ Si @ y[idx]), len(idx))
            if keep.any() and st.cr_joint[0] > chi2_999(len(idx)):
                st.cr_common_rejected += len(idx)
                st.rejected_cr += len(idx)
            elif keep.any():
                st.nis_cr.extend(w2.tolist())
                self._update(H[keep], y[keep], np.diag(R[keep]), att_scale=self._att_gate())
                st.used_cr += int(keep.sum())
        nxt = self._clone_epoch + 1
        self._prev_cr = {(m.sys, m.prn): (nxt, m.cr, m) for m in meas if m.cr is not None}
        self.reclone()
        return st

    # ------------------------------------------------------------- outputs
    def ned_from(self, ref_lla):
        """Position in the local NED frame of ``ref_lla`` (m)."""
        return t_e2ned(ref_lla[0], ref_lla[1]) @ (lla2ecef(self.lla) - lla2ecef(ref_lla))

    def sigmas(self):
        return np.sqrt(np.clip(np.diag(self.P), 0, None))
