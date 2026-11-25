#!/usr/bin/env python3
"""A simulated flight as raw_replay's events file, with a mosaic-G5-like receiver (Grid B: G4 and G5).

Turns one of E0's drogue flights (a tinkerrocket-sim pickle with the drogue spliced on, its truth phases in
truth_phases.json) into what the V8/V9's flight computer would be handed, for raw_replay_mosaic's cold start and
its four-voter quorum:

  I  the EKF ticks: every second 1 kHz row (EKF_DECIMATION 2), body FRD, m/s^2 and rad/s;
  B  the barometer at 500 Hz, as pressure on META ground_pressure_pa (optionally behind a pneumatic lag);
  M  the magnetometer at 100 Hz: the truth's field in the body, 0.3 uT of noise (the no-magnetometer arm runs the
     same file with --mag-flags 0);
  X  each receiver epoch on the fused 10 Hz grid of GPS time, released 30 ms after it (the GNSS task's build,
     until the bench measures it): every satellite placed (position, velocity and acceleration, ECEF of the
     transmit time), its corrected pseudorange, range rate and C/N0, PRNs mapped to GPS, Galileo and BeiDou by
     shell -- the stage's cap, floors, mask and noise model then act on it in the harness (sim_epoch.h);
  V  the receiver's own fix 25 ms after each of its epochs, at its own 20 Hz (the mosaic streams PVTGeodetic at
     msec50, config::GNSS_UPDATE_RATE; only the raw epochs are cut to the fused grid): a least-squares solution from
     the same (faulted) measurements, its velocity the truth's of 4-6 ms before the epoch (High dynamics) plus the
     solution's error, so a receiver fault reaches the GPS voter as it reaches the filter;
  META the cold start's (seed_lla, seed_quat -- the truth at the seed, the harness turns it by the magnetometer's
     error -- pad_lla, declination_rad, field_ned_ut, decimal_year, ground_pressure_pa), state_machine none (the
     quorum's own launch and landing), t_base_s, the truth's phase times, and what the pattern did (sim_*).

The receiver (MosaicRx, from the gnss repo's RawGNSSModel): 3 Walker shells (GPS, Galileo, BeiDou stand-ins on one
clock), a 3 deg mask, lock kept to 200 m/s^2 along the line of sight, a dipole-like C/N0 along the nose (46 dB-Hz
broadside, 6 dB on the axis), and noise at the mosaic profile's v0 fits for that C/N0 -- the code's white
(0.10/0.30) and correlated (1.94/3.5 x 0.5, 15 s) parts, the Doppler's white (0.01/0.075, without the profile's 1.5
margin), 0.05 m/s per g of stress (the profile allows 0.5), a TCXO with 1 ppb/g, no Doppler lag. It measures at
its own 20 Hz (--pvt-rate), as the mosaic does; every second epoch is on the fused 10 Hz grid (--rate) and goes to
the filter. The antenna is --lever-arm m along the nose from the IMU (0.3 by default; the filter keeps r = 0 unless
told the offset: raw_replay --antenna-m), or --lever-arm-b x,y,z m anywhere in the body (FRD; it wins over
--lever-arm).

A spinning body (the windup study; every option off by default, and the files then as before but for META's floats,
which numpy 2 wrote as np.float64(x)): --roll lays a roll profile over the flight (sim_roll.py: const:F, spinup:F,
rev:F[,T[,D]], F rev/s), the truth's attitude turned about its nose and the IMU with it, the gyro's chip clipped at
--gyro-clip-dps an axis (the ISM6's rail, 4587, at 45 deg to the nose), its scale off by --gyro-scale-ppm on the added
rate; --roll-axis-tilt-deg D rolls it about an axis D deg off the nose toward body +y instead (a spin axis off the
board's x: the nose cones about it); --windup-k K puts the carrier wind-up of an antenna on the nose on every Doppler
(Wu's rate form, K cycles a turn, each system's wavelength: L1/E1 on GPS and Galileo, B1I on BeiDou).

Patterns (G4/G5):
  none;
  cutA (A = 0.5, 1.5, 3: every epoch from T+A to T+8 cut, 43 % of the Dopplers of the fused epoch before it off by
     36-70 m/s, as c30's and the Beetle report's -- GNSS comes back at T+8, at or after the true apogee, so the
     coast flies unaided);
  gapD (D = 0.5, 1.5, 3: the same cut D s long from burnout, the epoch before it corrupted -- GNSS comes back in
     coast, where a velocity error turns the heading);
  stale_rx (every Doppler follows a receiver held at the pad's velocity through the boost, closing on the truth
     with tau 2 s from burnout, 3 m/s of scatter: the closing tail with GNSS after);
  stale_sats (two Galileo satellites so, tau 4 s -- the filter is not told: no firmware path widens one
     satellite's Doppler sigma, so rr_sig_add stays 0);
  settle15, settle40 (from burnout each Doppler off by a fresh draw of A exp(-dt/1.5 s) m/s, A 1.5 and 4; settle40
     also 2 m/s rms common to every Doppler over the first 5 s of coast: tc_ekf_eval --post-thrust 1.5,1.5 and
     4,1.5,2, the rig's rp5);
  lolN (N = 0..3: every satellite loses lock N s before the true apogee and re-acquires as the model does; no fix
     while under 4);
  coherent (the receiver's velocity biased down by a ramp 0 -> 5 m/s over the 3 s before the true apogee, then
     held: every Doppler and the fix's vU);
  coherent15 (a fast coherent step: 0 -> 15 m/s down over 1 s from 3 s before the true apogee, then held -- the
     filter's Doppler gate refuses it, so it is not the raw vote guard's case);
  slow12, slow15 (a slow coherent ramp, 2 or 2.5 m/s a second, ending at 12 or 15 m/s at the true apogee: it
     starts 6 s before it, never before burnout, then held), slow15b (0 -> 15 m/s over 6 s from burnout, then held):
     the filter accepts these and reads more than 10 m/s low before apogee -- the raw vote guard's own case;
     slow08 (the same ramp to 8 m/s: between the plan's 5 m/s and the guard's 10 m/s, its gap);
  baro_lagT (T = 0.5, 0.7, 1.0 s: the static port's first-order lag, GNSS clean -- a sensitivity sweep; nothing
     on the Goblin's port is measured).
The faults are tc_ekf_eval.run's (Grid A's) where it has them, applied before the PRN mapping as it applies them.

    PYTHONPATH=<simbuild>/tinkerrocket-sim/src:<gnss-hf-sim>/sim \\
        python3 tools/raw_replay/sim_events.py FLIGHT.pkl OUT_DIR --pattern none [--truth-phases J]

writes OUT_DIR/<flight>__<pattern>.events and, once a flight, OUT_DIR/truth/<flight>.npz (the truth sim_tables.py
scores against: flight time, attitude, NED velocity and position, the phase times) and OUT_DIR/truth/<flight>.nav
(the same truth at the IMU as raw_replay --truth-nav reads it: the quorum on a perfect filter, the run's own
barometer and fixes, for the truth's own call). Nothing is printed of the pad's position; it goes only into the
events file.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
import sys

import numpy as np

from tinkerrocket_gnss.raw_gnss_model import MU as MU_EARTH, RawGNSSModel
from tinkerrocket_gnss.tc_ekf import C_LIGHT, OMGE, ecef2lla, lla2ecef, quat2dcm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_roll  # noqa: E402

G0 = 9.807
T_OFS = 40.0                 # MCU s = flight s + T_OFS (META t_base_s = -T_OFS)
GPS_WEEK = 2441              # 2026-10-24, the flight's Saturday
TOW0 = 573786.0              # T+0's time of week (15:23 UTC), as the rig scenario's
DECIMAL_YEAR = 2026.81
RELEASE_S = 0.030            # the GNSS task hands a built epoch over this long after its GPS instant
FIX_ARRIVAL_S = 0.025        # the receiver's PVT reaches the flight loop this long after its epoch
CORRUPT = (0.43, 36.0, 70.0)  # the epoch before a cut: this share of its Dopplers off by 36-70 m/s (c30's)
W_EARTH = np.array([0.0, 0.0, OMGE])

PATTERNS = ("none", "cut05", "cut15", "cut30", "gap05", "gap15", "gap30", "stale_rx", "stale_sats", "settle15",
            "settle40", "lol0", "lol1", "lol2", "lol3", "coherent", "coherent15", "slow08", "slow12", "slow15",
            "slow15b", "baro_lag05", "baro_lag07", "baro_lag10")


def cn0_sigma(a, b, cn0):
    """The builder's noise model: sigma^2 = a^2 + b^2 10^(-(C/N0 - 35)/10)."""
    return math.sqrt(a * a + b * b * 10.0 ** (-(cn0 - 35.0) / 10.0))


def stable_seed(*parts):
    return int(hashlib.sha256("|".join(str(p) for p in parts).encode()).hexdigest()[:8], 16)


# ------------------------------------------------------------------ the receiver

class MosaicRx(RawGNSSModel):
    """RawGNSSModel's sky, clock and lock logic, with the mosaic-G5 profile's noise for each satellite's C/N0.

    measure() returns (meas, info): meas a list of dicts (prn as the model numbers it, sat_pos/sat_vel at the transmit
    time, pr, rr, cn0, s_pr, s_rr, s_corr), info the geometry the fix is solved from (each one's range, unit line
    of sight and rotated satellite velocity) and the tracked/visible counts."""

    # the mosaic profile v0's fits (ReceiverProfile.h mosaicG5Profile), the receiver's own (no rr_scale margin)
    PR_WHITE = (0.10, 0.30)
    PR_CORR = (1.94, 3.5)
    PR_CORR_SCALE = 0.5
    PR_TAU_S = 15.0
    RR_WHITE = (0.01, 0.075)

    def __init__(self, ref_lla_deg, seed, n_shells=3, rate_hz=10.0, rr_dyn_true=0.05, el_mask_deg=3.0,
                 cn0_broadside=46.0, cn0_axis_loss=6.0):
        super().__init__(ref_lat_deg=ref_lla_deg[0], ref_lon_deg=ref_lla_deg[1], ref_alt_m=ref_lla_deg[2],
                         rate_hz=rate_hz, el_mask_deg=el_mask_deg, atmo_sigma_m=0.0,
                         lock_los_accel_mps2=200.0, rr_dyn_sigma=0.0, rr_relock_sigma=0.0, n_shells=n_shells,
                         cn0_model=True, cn0_broadside=cn0_broadside, cn0_axis_loss=cn0_axis_loss,
                         epoch_offset_s=None, seed=seed)
        self.rr_dyn_true = rr_dyn_true
        self.gm = {}                                  # prn -> (wander m, time s)

    def measure_full(self, t, pos_ned, vel_ned, acc_ned, specific_force_g, g_peak, body_x_ned, windup=None):
        """windup: None, or (k, C_n2b, w_body): every range rate takes the carrier wind-up an antenna on the body's
        +x carries at body rate w_body (rad/s, FRD), C_n2b the NED -> body matrix (sim_roll.windup_rate, each shell's
        wavelength)."""
        dt = 0.0 if self.t_last is None else t - self.t_last
        self.t_last, self.acc_last = t, np.asarray(acc_ned, float)
        drift = self._clock(dt, specific_force_g - 1.0)
        Tn2e = self.T_ecef2ned.T
        r = self.ref_ecef + Tn2e @ np.asarray(pos_ned, float)
        v = Tn2e @ np.asarray(vel_ned, float)
        a = Tn2e @ np.asarray(acc_ned, float)
        T_local = self.T_ecef2ned
        c0, axis_loss, low_el, slope, noise = self.cn0_par
        out, info = [], dict(visible=0, tracked=0, rho=[], u=[], vs_r=[])
        for sat in self.sats:
            prn = sat["prn"]
            tau = 0.07
            for _ in range(3):
                rs, vs = self.sat_ecef(sat, t - tau)
                th = OMGE * tau
                c, s = math.cos(th), math.sin(th)
                rs_r = np.array([c * rs[0] + s * rs[1], -s * rs[0] + c * rs[1], rs[2]])
                rho = float(np.linalg.norm(rs_r - r))
                tau = rho / C_LIGHT
            vs_r = np.array([c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]])
            u = (rs_r - r) / rho
            el = math.asin(-(T_local @ u)[2])
            st = self.state[prn]
            if el < self.el_mask:
                st.update(locked=True, clear_since=None, relock_t=None, amb=None)
                continue
            info["visible"] += 1
            a_los = float(-u @ a)
            if st["locked"] and abs(a_los) > self.lock_acc:
                st.update(locked=False, clear_since=None)
            elif not st["locked"]:
                if abs(a_los) < self.relock_frac * self.lock_acc:
                    if st["clear_since"] is None:
                        mu_, sd, lo = self.reacq
                        st["clear_since"], st["delay"] = t, max(lo, self.rng.normal(mu_, sd))
                    elif t - st["clear_since"] >= st["delay"]:
                        st.update(locked=True, relock_t=t, clear_since=None)
                else:
                    st["clear_since"] = None
            if not st["locked"]:
                continue
            info["tracked"] += 1
            cos_ax = float((T_local @ u) @ np.asarray(body_x_ned, float))
            cn0 = (c0 - axis_loss * cos_ax * cos_ax - slope * max(0.0, low_el - math.degrees(el))
                   + self.cn0_ofs[prn] + self.rng_cn0.normal(0.0, noise))
            scale = 1.0
            if st["relock_t"] is not None:
                age = t - st["relock_t"]
                if age < self.relock_settle:
                    scale = 1.0 + (self.relock_noise - 1.0) * math.exp(-3.0 * age / self.relock_settle)
                else:
                    st["relock_t"] = None
            s_pr = cn0_sigma(*self.PR_WHITE, cn0) * scale
            s_rr = math.hypot(cn0_sigma(*self.RR_WHITE, cn0) * math.sqrt(scale),
                              self.rr_dyn_true * max(0.0, g_peak - 1.0))
            s_corr = cn0_sigma(*self.PR_CORR, cn0) * self.PR_CORR_SCALE
            w, tw = self.gm.get(prn, (self.rng.normal(0.0, s_corr), t))
            if t > tw:
                phi = math.exp(-(t - tw) / self.PR_TAU_S)
                w = phi * w + self.rng.normal(0.0, s_corr * math.sqrt(1.0 - phi * phi))
            self.gm[prn] = (w, t)
            pr = rho + self.clk_bias + w + self.rng.normal(0.0, s_pr)
            rr = float(u @ (vs_r - v)) + drift + self.rng.normal(0.0, s_rr)
            if windup is not None:
                k_w, C_w, w_w = windup
                e_b = C_w @ (T_local @ u)
                rr += float(sim_roll.windup_rate(sim_roll.SHELL_LAMBDA[min(int(prn) // 100, 2)], k_w, w_w, e_b))
            out.append(dict(prn=prn, sat_pos=rs, sat_vel=vs, pr=pr, rr=rr, cn0=cn0, s_pr=s_pr, s_rr=s_rr,
                            s_corr=s_corr, rr_sig_add=0.0, k=len(info["rho"])))
            info["rho"].append(rho)
            info["u"].append(u)
            info["vs_r"].append(vs_r)
        return out, info


def sat_acc_ecef(rs, vs):
    """A satellite's ECEF acceleration (gravity, Coriolis, centrifugal), as the builder has it from the orbit."""
    rs, vs = np.asarray(rs, float), np.asarray(vs, float)
    return (-MU_EARTH * rs / np.linalg.norm(rs) ** 3 - 2.0 * np.cross(W_EARTH, vs)
            - np.cross(W_EARTH, np.cross(W_EARTH, rs)))


def map_prn(prn):
    """The model's PRN (shell * 100 + 1..24) as (gnss_sys_t, PRN): GPS 1-24, Galileo 1-24, BeiDou MEO 19-42."""
    shell, k = divmod(int(prn), 100)
    if shell == 0:
        return 0, k
    if shell == 1:
        return 1, k
    return 2, k + 18


# ------------------------------------------------------------------ the flight

def load_flight(path, phases):
    d = pickle.load(open(path, "rb"))
    df, cfg = d["df"], d["cfg"]
    f = dict(cfg=cfg, phases=phases)
    f["t"] = df["time"].to_numpy(float)
    f["p"] = df[["true_pn", "true_pe", "true_pd"]].to_numpy(float)
    f["v"] = df[["true_vn", "true_ve", "true_vd"]].to_numpy(float)
    q = df[["true_q0_ned", "true_q1_ned", "true_q2_ned", "true_q3_ned"]].to_numpy(float)
    f["q"] = q / np.linalg.norm(q, axis=1)[:, None]
    f["acc"] = np.column_stack([df["imu_acc_x"], -df["imu_acc_y"], -df["imu_acc_z"]]).astype(float)   # FLU -> FRD
    f["gyr"] = np.radians(np.column_stack([df["imu_gyro_x"], -df["imu_gyro_y"], -df["imu_gyro_z"]])).astype(float)
    f["baro"] = df["baro_alt"].to_numpy(float)
    return f


def append_landing(f, rng, tau_s=0.05, rest_s=5.0, acc_noise=0.05, gyro_noise=0.00175, baro_noise=0.084):
    """The touchdown: a constant deceleration from the last row's velocity to rest over tau_s, then rest_s at rest
    (attitude held), the IMU consistent with it -- the drogue flights end at touchdown, still falling at 15 m/s."""
    t, p, v, q = f["t"], f["p"], f["v"], f["q"]
    dt = float(np.median(np.diff(t[-100:])))
    n_imp = int(round(tau_s / dt))
    n_rest = int(round(rest_s / dt))
    v0, p0, q0 = v[-1].copy(), p[-1].copy(), q[-1].copy()
    a_imp = -v0 / tau_s
    C = quat2dcm(q0)                                   # NED -> body
    rows_t, rows_p, rows_v, rows_acc, rows_gyr, rows_baro = [], [], [], [], [], []
    pk, vk = p0.copy(), v0.copy()
    for k in range(1, n_imp + n_rest + 1):
        a = a_imp if k <= n_imp else np.zeros(3)
        vk = vk + a * dt
        if k == n_imp:
            vk = np.zeros(3)
        pk = pk + vk * dt
        fb = C @ (a - np.array([0.0, 0.0, G0])) + rng.normal(0.0, acc_noise, 3)
        rows_t.append(t[-1] + k * dt)
        rows_p.append(pk.copy())
        rows_v.append(vk.copy())
        rows_acc.append(fb)
        rows_gyr.append(rng.normal(0.0, gyro_noise, 3))
        rows_baro.append(-pk[2] + rng.normal(0.0, baro_noise))
    f["touchdown_s"] = float(t[-1])
    f["t"] = np.concatenate([t, rows_t])
    f["p"] = np.vstack([p, rows_p])
    f["v"] = np.vstack([v, rows_v])
    f["q"] = np.vstack([q, np.repeat(q0[None, :], len(rows_t), axis=0)])
    f["acc"] = np.vstack([f["acc"], rows_acc])
    f["gyr"] = np.vstack([f["gyr"], rows_gyr])
    f["baro"] = np.concatenate([f["baro"], rows_baro])


def body_rates(q, t):
    """The truth's body rate (rad/s, FRD) at each row, from its attitude (forward differences)."""
    C = np.stack([quat2dcm(x) for x in q])            # NED -> body
    dC = np.einsum("nij,nkj->nik", C[1:], C[:-1])      # C1 C0^T
    w = np.stack([dC[:, 1, 2] - dC[:, 2, 1], dC[:, 2, 0] - dC[:, 0, 2], dC[:, 0, 1] - dC[:, 1, 0]], axis=1) / 2.0
    dt = np.diff(t)[:, None]
    w = w / dt
    return np.vstack([w, w[-1:]]), C


# ------------------------------------------------------------------ the fix (V)

def solve_fix(meas, info, v_true, T_e2n):
    """The receiver's least-squares fix from this epoch's measurements (all it tracks): its position and velocity
    errors (NED, m and m/s), solved linearly at the true antenna state of the epoch (v_true: ECEF m/s, the velocity
    each range rate was formed at). The caller adds the velocity error to the velocity the fix reports, which is
    the truth's of a few ms before (the PVT filter's lag): linearizing at that lagged velocity instead would give
    back the epoch's own velocity, since the range rates carry it."""
    n = len(meas)
    if n < 4:
        return None
    H = np.zeros((n, 4))
    yp, yv, wp, wv = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    for i, m in enumerate(meas):
        u = info["u"][m["k"]]
        H[i, :3] = -u
        H[i, 3] = 1.0
        yp[i] = m["pr"] - info["rho"][m["k"]]
        yv[i] = m["rr"] - float(u @ (info["vs_r"][m["k"]] - v_true))
        wp[i] = 1.0 / (m["s_pr"] ** 2 + m["s_corr"] ** 2)
        wv[i] = 1.0 / (m["s_rr"] ** 2 + m["rr_sig_add"] ** 2)
    try:
        dx = np.linalg.solve(H.T @ (wp[:, None] * H), H.T @ (wp * yp))
        dv = np.linalg.solve(H.T @ (wv[:, None] * H), H.T @ (wv * yv))
    except np.linalg.LinAlgError:
        return None
    return T_e2n @ dx[:3], T_e2n @ dv[:3]


def coherent_bias(te, coh):
    """The coherent fault's downward bias at te (m/s): amp * clip((te - t_start) / ramp_s, 0, 1)."""
    amp, ramp_s, t_start = coh
    return amp * min(1.0, max(0.0, (te - t_start) / ramp_s))


def settle_offsets(rng, n, dt_b, post):
    """The tracking loops settling after the thrust (tc_ekf_eval --post-thrust A,TAU[,COMMON[,WINDOW]]), dt_b s
    after burnout: n Doppler offsets (m/s), each a fresh draw of A exp(-dt_b/TAU) rms, plus one draw of COMMON rms
    shared by all of them over the first WINDOW s of coast (None: nothing left to add)."""
    a, tau, common, window = post
    s_sat = a * math.exp(-dt_b / tau)
    c = float(rng.normal(0.0, common)) if common > 0.0 and dt_b <= window else 0.0
    if s_sat <= 1e-3 and c == 0.0:
        return None
    return [float(rng.normal(0.0, s_sat)) + c for _ in range(n)]


def pattern_params(pattern, t_burn, t_apo):
    """A pattern's parameters: dict(cut=(from, to) flight s, corrupt=CORRUPT, stale=(tau, scatter),
    stale_sats=(n, tau), post=(A, tau, common, window), lol_s, coh=(amp, ramp_s, t_start), baro_tau)."""
    p = dict(cut=None, corrupt=None, stale=None, stale_sats=None, post=None, lol_s=None, coh=None, baro_tau=None)
    if pattern.startswith("cut"):
        p.update(cut=(float(pattern[3:]) / 10.0, 8.0), corrupt=CORRUPT)
    elif pattern.startswith("gap"):
        p.update(cut=(t_burn, t_burn + float(pattern[3:]) / 10.0), corrupt=CORRUPT)
    elif pattern == "stale_rx":
        p["stale"] = (2.0, 3.0)
    elif pattern == "stale_sats":
        p["stale_sats"] = (2, 4.0)
    elif pattern == "settle15":
        p["post"] = (1.5, 1.5, 0.0, 5.0)
    elif pattern == "settle40":
        p["post"] = (4.0, 1.5, 2.0, 5.0)
    elif pattern.startswith("lol"):
        p["lol_s"] = float(pattern[3:])
    elif pattern == "coherent":
        p["coh"] = (5.0, 3.0, t_apo - 3.0)
    elif pattern == "coherent15":
        p["coh"] = (15.0, 1.0, t_apo - 3.0)
    elif pattern in ("slow08", "slow12", "slow15"):
        p["coh"] = (float(pattern[4:]), 6.0, max(t_apo - 6.0, t_burn))
    elif pattern == "slow15b":
        p["coh"] = (15.0, 6.0, t_burn)
    elif pattern.startswith("baro_lag"):
        p["baro_tau"] = float(pattern[8:]) / 10.0
    elif pattern != "none":
        raise SystemExit(f"unknown pattern {pattern}")
    return p


# ------------------------------------------------------------------ the events file

def roll_overlay(f, ph, roll, gyro_clip_dps=None, gyro_scale_ppm=0.0, tilt_deg=0.0):
    """The flight's body turned about its nose by the profile roll (sim_roll.parse_profile), and its IMU with it (the
    sensor's own errors kept in the sensor's frame; the gyro's chip clipped at gyro_clip_dps an axis, None: no clip;
    gyro_scale_ppm on the added rate); with tilt_deg, about an axis that far off the nose toward body +y
    (sim_roll.roll_axis: the nose cones about it). In place: f's q, acc, gyr; returns the rolled body's rate (rad/s,
    FRD, each row's forward interval), C_n2b, and the overlay's facts for META."""
    t, q, v = f["t"], f["q"], f["v"]
    w0, C0 = body_rates(q, t)
    facts = dict(peak_rps=0.0, clipped_s=0.0)
    if roll is None:
        return w0, C0, facts
    rate = sim_roll.profile_rate(t, roll, ph)
    phi = sim_roll.integrate(t, rate)
    phi_mid = np.concatenate([0.5 * (phi[1:] + phi[:-1]), phi[-1:]])
    phi_dot = np.concatenate([np.diff(phi) / np.diff(t), rate[-1:]])
    # the truth: q' = q (x) qx(phi); its rate over each row's interval Rx(phi_mid)^T w + phi_dot x (qa, Ra and a about
    # a tilted axis)
    ax = sim_roll.roll_axis(tilt_deg)       # None: the nose itself
    q1 = sim_roll.roll_quat(q, phi, ax)
    Rm = sim_roll.rot(phi_mid, ax)
    w1 = sim_roll.turn(Rm, w0)
    if ax is None:
        w1[:, 0] += phi_dot
    else:
        w1 += phi_dot[:, None] * ax
    C1 = np.einsum("nji,njk->nik", sim_roll.rot(phi, ax), C0)     # Ra(phi)^T C (NED -> rolled body)
    # the IMU: the truth's part turned, the sensor's errors kept. The flight filter takes each IMU row (every second
    # one of the 1 kHz table: EKF_DECIMATION 2) as the mean over the 2 ms before it, so the added roll is that mean
    # and the turn the angle at its middle: taken as the flight's own rows are (the 1 ms after), a spin-up would
    # leave every filter 1.5 ms of roll behind -- 5.4 deg at 10 rev/s, the same in every arm
    a_ned = np.gradient(v, t, axis=0)
    f_true = np.einsum("nij,nj->ni", C0, a_ned - np.array([0.0, 0.0, G0]))
    phi_b = phi.copy()                      # the angle at the middle of the 2 ms before each row
    phi_b[2:] = phi[1:-1]
    rate_b = rate.copy()                    # the added rate's mean over them
    rate_b[2:] = (phi[2:] - phi[:-2]) / (t[2:] - t[:-2])
    Rb = sim_roll.rot(phi_b, ax)
    f["acc"] = f["acc"] + sim_roll.turn(Rb, f_true) - f_true
    added = np.zeros_like(w0)
    if ax is None:
        added[:, 0] = rate_b * (1.0 + 1e-6 * gyro_scale_ppm)
    else:
        added = (rate_b * (1.0 + 1e-6 * gyro_scale_ppm))[:, None] * ax
    gyr = f["gyr"] + sim_roll.turn(Rb, w0) - w0 + added
    if gyro_clip_dps is not None:
        gyr, clipped = sim_roll.chip_clip(gyr, math.radians(gyro_clip_dps))
        facts["clipped_s"] = float(np.sum(np.diff(t)[clipped[:-1]]))
    f["gyr"] = gyr
    f["q"] = q1
    facts["peak_rps"] = float(np.max(np.abs(rate)) / sim_roll.TWO_PI)
    return w1, C1, facts


def antenna_truth(p, v, C_n2b, w_body, r_b):
    """The antenna's position and velocity (NED, m and m/s) at r_b in the body (FRD, m), each row's: p + R r_b and
    v + R (w x r_b), R the body -> NED matrix (C_n2b transposed) and w the body's rate (rad/s, FRD) -- as rolled by
    roll_overlay, so a roll turns an antenna off the nose's axis about it."""
    R_b2n = np.transpose(C_n2b, (0, 2, 1))
    return p + np.einsum("nij,j->ni", R_b2n, r_b), v + np.einsum("nij,nj->ni", R_b2n, np.cross(w_body, r_b))


def build(flight_path, pattern, out_dir, phases_all, lever_arm=0.3, shells=3, rate_hz=10.0, pvt_hz=20.0,
          lever_b=None, roll=None, windup_k=0.0, gyro_clip_dps=None, gyro_scale_ppm=0.0, roll_axis_tilt_deg=0.0):
    name = os.path.splitext(os.path.basename(flight_path))[0]
    ph = phases_all[name]
    rng_land = np.random.default_rng(stable_seed(name, "landing"))
    f = load_flight(flight_path, ph)
    append_landing(f, rng_land)
    cfg = f["cfg"]
    roll_spec = sim_roll.parse_profile(roll)
    w_body, C_n2b, roll_facts = roll_overlay(f, ph, roll_spec, gyro_clip_dps, gyro_scale_ppm, roll_axis_tilt_deg)
    t, p, v, q = f["t"], f["p"], f["v"], f["q"]
    acc, gyr = f["acc"], f["gyr"]
    a_ned = np.gradient(v, t, axis=0)
    sf_g = np.linalg.norm(acc, axis=1) / G0
    t_burn, t_apo = ph["burnout_s"], ph["apogee_s"]
    ref = (cfg["ref_lat_deg"], cfg["ref_lon_deg"], cfg["ref_alt_m"])

    # the antenna: lever_arm m along the nose from the IMU, or lever_b anywhere in the body (FRD; truth only: the
    # filter keeps r = 0 unless the replay gives it one)
    r_b = np.array([lever_arm, 0.0, 0.0]) if lever_b is None else np.asarray(lever_b, float)
    p_ant, v_ant = antenna_truth(p, v, C_n2b, w_body, r_b)
    a_ant = np.gradient(v_ant, t, axis=0)
    nose = C_n2b[:, 0, :]                              # body x in NED

    rx = MosaicRx(ref, seed=stable_seed(name, "receiver"), n_shells=shells, rate_hz=pvt_hz)
    rng_f = np.random.default_rng(stable_seed(name, pattern, "fault"))
    T_e2n = rx.T_ecef2ned
    Tn2e = T_e2n.T
    lag_v = 0.004 + 0.002 * np.random.default_rng(stable_seed(name, "pvt-lag")).random()   # the fix's 4-6 ms

    pp = pattern_params(pattern, t_burn, t_apo)
    cut, corrupt, stale, stale_sats, post = pp["cut"], pp["corrupt"], pp["stale"], pp["stale_sats"], pp["post"]
    lol_s, coh, baro_tau = pp["lol_s"], pp["coh"], pp["baro_tau"]

    # sim_*_epochs count the fused epochs (the filter's); sim_rx_epochs every receiver epoch, sim_fixes its fixes
    counts = dict(sim_epochs_cut=0, sim_sats_corrupted=0, sim_stale_epochs=0, sim_post_epochs=0,
                  sim_lol_epochs=0, sim_coherent_epochs=0, sim_epochs=0, sim_rx_epochs=0, sim_fixes=0)
    stale_prns = None
    lol_done = False
    lol_t = None
    i_launch = int(np.searchsorted(t, 0.0))
    ev = []                                            # (t_us, order, text)

    def us(x):
        return int(round((x + T_OFS) * 1e6))

    # ---- the receiver: an epoch every 50 ms of GPS time (its PVT rate), every second one on the fused grid
    t_ms = np.round(t * 1000.0).astype(np.int64)
    x_ms = int(round(1000.0 / rate_hz))
    rx_ms = int(round(1000.0 / pvt_hz))
    if x_ms % rx_ms != 0:
        raise SystemExit(f"the fused grid ({x_ms} ms) must fall on the receiver's epochs ({rx_ms} ms)")
    g_pk = 0.0
    first_fix = None
    for i in range(len(t)):
        g_pk = max(g_pk, sf_g[i])
        if t_ms[i] % rx_ms != 0:
            continue
        te = t_ms[i] / 1000.0
        on_grid = int(t_ms[i] % x_ms == 0)             # this epoch goes to the filter as well as to the fix
        t_rel = te                                     # flight time (T+0 = the launch edge)
        if lol_s is not None and not lol_done and te >= t_apo - lol_s - 1e-9:
            # every channel lost at once; each re-acquires as the model re-acquires (its delay, its relock noise)
            for st in rx.state.values():
                st.update(locked=False, clear_since=None)
            lol_done, lol_t = True, te
        meas, info = rx.measure_full(te, p_ant[i], v_ant[i], a_ant[i], sf_g[i], g_pk, nose[i],
                                     windup=(windup_k, C_n2b[i], w_body[i]) if windup_k > 0.0 else None)
        g_pk = sf_g[i]
        counts["sim_rx_epochs"] += 1
        counts["sim_epochs"] += on_grid
        if lol_done and len(meas) < info["visible"]:
            counts["sim_lol_epochs"] += on_grid
        # the stale receiver (tc_ekf_eval --stale-doppler): from launch every Doppler follows a receiver at
        # v_true - lag v_true, lag 1 through the boost then exp(-(t - burnout)/tau), with scatter
        if stale is not None and i >= i_launch:
            lag = 1.0 if te < t_burn else math.exp(-(te - t_burn) / stale[0])
            if lag > 1e-3:
                dv_e = Tn2e @ (lag * v_ant[i])
                scat = stale[1] * lag
                for m in meas:
                    m["rr"] += float(info["u"][m["k"]] @ dv_e) + float(rng_f.normal(0.0, scat))
                counts["sim_stale_epochs"] += on_grid
        # two stale satellites of the second shell (tc_ekf_eval --stale-sats N,TAU): their Dopplers lag so, and
        # nothing tells the filter -- the firmware has no path that widens one satellite's Doppler sigma (the
        # builder's relock term is from a gap it sees; the mosaic's own Doppler variance is decoded, not used)
        if stale_sats is not None and i >= i_launch and meas:
            if stale_prns is None:
                pool = [m["prn"] for m in meas if 100 <= m["prn"] < 200] or [m["prn"] for m in meas]
                k = min(int(stale_sats[0]), len(pool))
                stale_prns = set(rng_f.choice(pool, size=k, replace=False).tolist()) if k else set()
            lag = 1.0 if te < t_burn else math.exp(-(te - t_burn) / stale_sats[1])
            if lag > 1e-3 and stale_prns:
                dv_e = Tn2e @ (lag * v_ant[i])
                for m in meas:
                    if m["prn"] in stale_prns:
                        m["rr"] += float(info["u"][m["k"]] @ dv_e)
                counts["sim_stale_epochs"] += on_grid
        # the tracking loops settling after the thrust (tc_ekf_eval --post-thrust A,TAU[,COMMON])
        if post is not None and te >= t_burn and meas:
            offs = settle_offsets(rng_f, len(meas), te - t_burn, post)
            if offs is not None:
                for m, o in zip(meas, offs):
                    m["rr"] += o
                counts["sim_post_epochs"] += on_grid
        # the coherent fault: the receiver's velocity biased down by a ramp, every Doppler and its fix
        if coh is not None:
            b = coherent_bias(te, coh)
            if b > 0.0:
                db_e = Tn2e @ np.array([0.0, 0.0, b])
                for m in meas:
                    m["rr"] -= float(info["u"][m["k"]] @ db_e)
                counts["sim_coherent_epochs"] += on_grid
        # a cut, and the fused epoch before it corrupted (tc_ekf_eval --gnss-cut A,B --corrupt-before-cut F,LO,HI)
        if cut is not None and corrupt is not None and on_grid and t_rel < cut[0] <= t_rel + x_ms / 1000.0 + 1e-6:
            ks = list(range(len(meas)))
            nb = max(1, int(round(corrupt[0] * len(ks)))) if ks else 0
            bad = set(rng_f.choice(ks, size=min(nb, len(ks)), replace=False).tolist()) if nb else set()
            for k in bad:
                meas[k]["rr"] += float(rng_f.choice([-1.0, 1.0]) * rng_f.uniform(corrupt[1], corrupt[2]))
            counts["sim_sats_corrupted"] += len(bad)
        if cut is not None and cut[0] <= t_rel < cut[1]:
            meas = []
            counts["sim_epochs_cut"] += on_grid
        if not meas:
            continue                                   # an empty epoch is not handed over (nor a fix made)
        # the receiver's fix from the same measurements: their errors, on the truth of lag_v before the epoch
        v_lag = np.array([np.interp(te - lag_v, t, v_ant[:, k]) for k in range(3)])
        fx = solve_fix(meas, info, Tn2e @ v_ant[i], T_e2n)
        if fx is not None:
            dpos, dvel = fx
            pa = p_ant[i] + dpos
            vfix = v_lag + dvel
            alt = ref[2] - pa[2]
            ev.append((us(te + FIX_ARRIVAL_S), 2,
                       f"V {us(te + FIX_ARRIVAL_S)} 3 {len(meas)} {alt:.3f} {vfix[0]:.4f} {vfix[1]:.4f} {-vfix[2]:.4f}"))
            counts["sim_fixes"] += 1
            if first_fix is None:
                first_fix = ecef2lla(rx.ref_ecef + Tn2e @ pa)
        if not on_grid:
            continue
        # the epoch, its satellites mapped to systems
        parts = []
        for m in meas:
            sys_id, prn = map_prn(m["prn"])
            sa = sat_acc_ecef(m["sat_pos"], m["sat_vel"])
            sp, sv = m["sat_pos"], m["sat_vel"]
            parts.append(f"{sys_id} {prn} 1 1 {sp[0]:.3f} {sp[1]:.3f} {sp[2]:.3f} {sv[0]:.4f} {sv[1]:.4f} "
                         f"{sv[2]:.4f} {sa[0]:.5f} {sa[1]:.5f} {sa[2]:.5f} {m['pr']:.4f} {m['rr']:.5f} "
                         f"{m['cn0']:.2f} {m['rr_sig_add']:.3f}")
        rel = us(te + RELEASE_S)
        ev.append((rel, 3, f"X {rel} {us(te)} {TOW0 + te:.3f} {GPS_WEEK} {len(meas)} " + " ".join(parts)))

    # ---- the IMU ticks, barometer and magnetometer
    thrust_start = ph["thrust_start_s"]
    touch = f["touchdown_s"]
    for i in range(0, len(t), 2):
        flags = 0x01
        if t[i] >= thrust_start:
            flags = 0x02
        if t[i] >= touch + 1.0:
            flags = 0x08 | 0x01
        a, g = acc[i], gyr[i]
        ev.append((us(t[i]), 4, f"I {us(t[i])} {a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {g[0]:.7f} {g[1]:.7f} "
                                f"{g[2]:.7f} {flags}"))
    P0 = 101325.0 * (1.0 - ref[2] / 44330.0) ** 5.255
    baro = f["baro"].copy()
    if baro_tau is not None:
        # the static port's first-order lag
        lagged = baro.copy()
        for k in range(1, len(baro)):
            dtk = t[k] - t[k - 1]
            lagged[k] = lagged[k - 1] + (baro[k] - lagged[k - 1]) * (dtk / (baro_tau + dtk))
        baro = lagged
    for i in range(1, len(t), 2):
        pr_pa = P0 * (1.0 - baro[i] / 44330.0) ** 5.255
        ev.append((us(t[i]), 0, f"B {us(t[i])} {pr_pa:.3f}"))
    import tinkerrocket_sim._ekf as ekf_ext
    lat_r, lon_r = math.radians(ref[0]), math.radians(ref[1])
    field = np.array(ekf_ext.geomag_field_ned_uT(lat_r, lon_r, ref[2], DECIMAL_YEAR))
    decl = ekf_ext.declination_rad(lat_r, lon_r, ref[2], DECIMAL_YEAR)
    rng_m = np.random.default_rng(stable_seed(name, "mag"))
    for i in range(0, len(t), 10):
        mb = C_n2b[i] @ field + rng_m.normal(0.0, 0.3, 3)
        ev.append((us(t[i]), 1, f"M {us(t[i])} {mb[0]:.4f} {mb[1]:.4f} {mb[2]:.4f}"))

    ev.sort(key=lambda e: (e[0], e[1]))
    # the seed: the receiver's first fix, and the truth's attitude at the first tick
    if first_fix is None:
        raise SystemExit(f"{name} {pattern}: the receiver never had a fix")
    q0 = q[0]
    lines = [
        f"# sim_events.py: {name}, pattern {pattern}; flight time = MCU s + t_base_s",
        f"META t_base_s {-T_OFS}",
        "META state_machine none",
        f"META ground_pressure_pa {P0:.3f}",
        f"META pad_lla {ref[0]!r} {ref[1]!r} {ref[2]!r}",
        f"META seed_lla {float(math.degrees(first_fix[0]))!r} {float(math.degrees(first_fix[1]))!r} "
        f"{float(first_fix[2]):.3f}",
        f"META seed_quat {float(q0[0])!r} {float(q0[1])!r} {float(q0[2])!r} {float(q0[3])!r}",
        "META seed_quat_truth 1",
        f"META decimal_year {DECIMAL_YEAR}",
        f"META declination_rad {float(decl)!r}",
        f"META field_ned_ut {float(field[0])!r} {float(field[1])!r} {float(field[2])!r}",
        f"META flight {name}",
        f"META pattern {pattern}",
        f"META lever_arm_m {float(r_b[0])!r}",
        f"META receiver_model mosaic-like shells {shells} rate_hz {rate_hz} pvt_hz {pvt_hz} lock_los_accel 200 "
        f"rr_dyn_true 0.05 cn0_broadside 46 cn0_axis_loss 6 el_mask 3",
        f"META fix_vel_lag_s {lag_v:.4f}",
        f"META t_inflight_us {us(thrust_start)}",
        f"META t_landed_flag_us {us(touch + 1.0)}",
    ]
    for k in ("thrust_start_s", "burnout_s", "thrust_end_s", "apogee_s", "vz_below_2mps_s", "deploy_s",
              "landing_s"):
        lines.append(f"META truth_{k} {float(ph[k])!r}")
    lines.append(f"META truth_touchdown_s {float(touch)!r}")
    if lever_b is not None:
        lines.append("META lever_arm_b_m " + " ".join(repr(float(x)) for x in r_b))
    if roll_spec is not None:
        lines.append(f"META sim_roll_profile {roll}")
        lines.append(f"META sim_roll_peak_rev_s {roll_facts['peak_rps']:.4f}")
        lines.append(f"META sim_gyro_clip_dps {gyro_clip_dps if gyro_clip_dps is not None else 'none'}")
        lines.append(f"META sim_gyro_clipped_s {roll_facts['clipped_s']:.4f}")
        if gyro_scale_ppm:
            lines.append(f"META sim_gyro_scale_ppm {gyro_scale_ppm}")
        if roll_axis_tilt_deg:
            lines.append(f"META sim_roll_axis_tilt_deg {roll_axis_tilt_deg}")
    if windup_k > 0.0:
        lines.append(f"META sim_windup_k {windup_k}")
    if lol_t is not None:
        lines.append(f"META sim_lol_t {lol_t!r}")
    if baro_tau is not None:
        lines.append(f"META sim_baro_lag_tau_s {baro_tau}")
    if coh is not None:
        lines.append(f"META sim_coherent_mps {coh[0]} over_s {coh[1]} from_t_s {coh[2]!r}")
    if stale_prns:
        lines.append("META sim_stale_prns " + " ".join("%d:%d" % map_prn(x) for x in sorted(stale_prns)))
    for k, val in counts.items():
        lines.append(f"META {k} {val}")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{name}__{pattern}.events")
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
        for e in ev:
            fh.write(e[2] + "\n")
    # the truth, once a flight (the same for every pattern)
    tdir = os.path.join(out_dir, "truth")
    os.makedirs(tdir, exist_ok=True)
    tpath = os.path.join(tdir, f"{name}.npz")
    if not os.path.exists(tpath):
        tmp = f"{tpath}.{os.getpid()}.{pattern}.npz"   # several patterns of a flight may be written at once
        np.savez_compressed(tmp, t=t, q=q, v=v, pd=p[:, 2], phases=json.dumps(dict(ph, touchdown_s=touch)))
        os.replace(tmp, tpath)
    npath = os.path.join(tdir, f"{name}.nav")
    if not os.path.exists(npath):
        write_truth_nav(npath, [us(x) for x in t], v, q, ref[2] - p[:, 2], tag=pattern)
    return out, counts


def write_truth_nav(path, t_us, v, q, h, tag=""):
    """The truth at the IMU as raw_replay --truth-nav reads it (truth_nav.h): a line a row, the MCU time (us), the
    NED velocity (m/s), the attitude (body FRD -> NED, scalar first) and the height on the receiver's fix's datum
    (m). Written to a temporary name first: several patterns of a flight may write it at once."""
    tmp = f"{path}.{os.getpid()}.{tag}.tmp"
    with open(tmp, "w") as fh:
        fh.write("# sim_events.py truth: t_us vn ve vd qw qx qy qz h\n")
        fh.write("".join(f"{tu} {vv[0]:.4f} {vv[1]:.4f} {vv[2]:.4f} {qq[0]:.7f} {qq[1]:.7f} {qq[2]:.7f} {qq[3]:.7f} "
                         f"{hh:.3f}\n" for tu, vv, qq, hh in zip(t_us, v, q, h)))
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("flight", help="a drogue flight pickle (E0's grid/flights/*.pkl)")
    ap.add_argument("out_dir")
    ap.add_argument("--pattern", default="none", choices=PATTERNS)
    ap.add_argument("--truth-phases", help="truth_phases.json (default: beside the flight)")
    ap.add_argument("--lever-arm", type=float, default=0.3, help="the antenna along the nose from the IMU, m")
    ap.add_argument("--lever-arm-b", help="x,y,z: the antenna anywhere in the body (FRD, m; wins over --lever-arm)")
    ap.add_argument("--roll", help="a roll profile over the flight (sim_roll.parse_profile: const:F, spinup:F, "
                                   "rev:F[,T[,D]], F in rev/s)")
    ap.add_argument("--windup-k", type=float, default=0.0, help="the carrier wind-up on every Doppler, cycles a turn "
                                                                "about the nose (0: none)")
    ap.add_argument("--gyro-clip-dps", type=float, help="the gyro's chip clipped at this an axis (the ISM6's rail: "
                                                        "4587)")
    ap.add_argument("--gyro-scale-ppm", type=float, default=0.0, help="the gyro's scale error on the added roll")
    ap.add_argument("--roll-axis-tilt-deg", type=float, default=0.0, help="the roll's axis this far off the nose, "
                                                                          "toward body +y (deg; 0: the nose)")
    ap.add_argument("--shells", type=int, default=3)
    ap.add_argument("--rate", type=float, default=10.0, help="the fused epochs a second (the filter's)")
    ap.add_argument("--pvt-rate", type=float, default=20.0, help="the receiver's epochs and fixes a second")
    a = ap.parse_args()
    tp = a.truth_phases or os.path.join(os.path.dirname(os.path.abspath(a.flight)), "truth_phases.json")
    phases = json.load(open(tp))["flights"]
    lever_b = [float(x) for x in a.lever_arm_b.split(",")] if a.lever_arm_b else None
    out, counts = build(a.flight, a.pattern, a.out_dir, phases, a.lever_arm, a.shells, a.rate, a.pvt_rate, lever_b,
                        a.roll, a.windup_k, a.gyro_clip_dps, a.gyro_scale_ppm, a.roll_axis_tilt_deg)
    print(out, json.dumps(counts))


if __name__ == "__main__":
    sys.exit(main())
