"""Raw GNSS measurement model: per-satellite pseudorange and range rate.

Where ``GNSSModel`` hands the filter a finished fix, this hands it what a
raw-output receiver sends (after its ephemeris has been applied): for each
tracked satellite, the satellite's position and velocity at transmit time, a
pseudorange and a range rate (-lambda * Doppler).

What it models, and why:
  * A synthetic GPS-like constellation (Walker 24/6/1, 55 deg, 26 560 km), so
    geometry and elevation behave like the real sky without an ephemeris file.
  * A receiver clock: bias and drift random walks at TCXO levels, plus drift
    that moves with acceleration (g-sensitivity, ~1 ppb/g = 0.3 m/s per g).
  * Per-satellite loss of lock from that satellite's own line-of-sight
    acceleration. Measured on the COCOM rig: the LC86G held channels whose
    Doppler rate stayed at or below 128 Hz/s (24 m/s^2 along the line of
    sight) and lost those at or above 171 Hz/s (32 m/s^2); the SAM-M10Q was
    flat to 312 Hz/s (59 m/s^2). ``lock_los_accel_mps2`` is that knee.
    Under a vertical boost the line-of-sight acceleration is a*sin(elev), so
    overhead satellites go first and low ones survive -- which is the case a
    tightly coupled filter can use and a fix-level filter cannot.
  * Optional common-mode loss on a jerk spike (the LC86G dropped every channel
    at a 13.5 g ignition, then re-locked by Doppler rate).
  * Re-acquisition after a randomized delay, with inflated noise just after.
  * Residual atmosphere/ephemeris error per satellite (slowly varying bias).
  * The error structure measured on a PX1105R (20 min static, 20 Hz, 2026-09-26):
    the receiver smooths its code, so epoch-to-epoch pseudorange noise is
    centimetres and the metres are a per-satellite wander (Gauss-Markov,
    ~2 m, tau ~15 s) plus a fixed offset; optionally the SkyTraq's whole-hertz,
    truncated Doppler (``doppler_step_hz=1``), corrected as the reader does.
"""
from __future__ import annotations

import math
import numpy as np

from ..estimation.tc_ekf import RawMeas, OMGE, C_LIGHT, lla2ecef, t_e2ned

MU = 3.986005e14
A_GPS = 26_559_700.0
INC = math.radians(55.0)
L1_WAVELENGTH = C_LIGHT / 1575.42e6


class RawGNSSModel:
    def __init__(self, ref_lat_deg=38.0, ref_lon_deg=-122.0, ref_alt_m=0.0,
                 rate_hz=10.0, el_mask_deg=10.0,
                 sigma_pr_m=0.1, sigma_rr_mps=0.05, atmo_sigma_m=1.5,
                 pr_wander_sigma_m=2.0, pr_wander_tau_s=15.0, doppler_step_hz=0.0,
                 sigma_cr_m=0.0015, cr_corr_sigma_m=0.0018, cr_corr_tau_s=0.3, cr_rw_m=0.0015,
                 clk_bias0_m=None, clk_drift0_mps=None,
                 clk_h0=2e-19, clk_hm2=2e-20, clk_g_ppb=1.0,
                 lock_los_accel_mps2=30.0, relock_fraction=0.7,
                 reacq_mean_s=2.0, reacq_std_s=0.7, reacq_min_s=0.8,
                 relock_noise_scale=5.0, relock_settle_s=2.0,
                 common_mode_jerk_mps3=None,
                 epoch_offset_s=None, seed=None):
        self.rng = np.random.default_rng(seed)
        self.ref_lla = np.array([math.radians(ref_lat_deg), math.radians(ref_lon_deg), ref_alt_m])
        self.ref_ecef = lla2ecef(self.ref_lla)
        self.T_ecef2ned = t_e2ned(self.ref_lla[0], self.ref_lla[1])
        self.rate_hz = rate_hz
        self.el_mask = math.radians(el_mask_deg)
        self.sigma_pr, self.sigma_rr, self.atmo_sigma = sigma_pr_m, sigma_rr_mps, atmo_sigma_m
        self.wander_sigma, self.wander_tau = pr_wander_sigma_m, pr_wander_tau_s
        self.doppler_step = doppler_step_hz
        self.sigma_cr = sigma_cr_m
        # carrier noise as fitted to a PX1105R at ~42 dB-Hz (gnss_raw CP_*):
        # white, a part correlated over tenths of a second, a slow random walk
        self.cr_corr_sigma, self.cr_corr_tau, self.cr_rw = cr_corr_sigma_m, cr_corr_tau_s, cr_rw_m
        self.wander = {}                                   # prn -> (value m, time s)
        self.q_b = C_LIGHT ** 2 * clk_h0 / 2.0
        self.q_d = 2 * math.pi ** 2 * C_LIGHT ** 2 * clk_hm2
        self.clk_g = C_LIGHT * clk_g_ppb * 1e-9            # m/s per g
        self.clk_bias = self.rng.uniform(-3e5, 3e5) if clk_bias0_m is None else clk_bias0_m
        self.clk_drift = self.rng.uniform(-150, 150) if clk_drift0_mps is None else clk_drift0_mps
        self.lock_acc = lock_los_accel_mps2
        self.relock_frac = relock_fraction
        self.reacq = (reacq_mean_s, reacq_std_s, reacq_min_s)
        self.relock_noise, self.relock_settle = relock_noise_scale, relock_settle_s
        self.cm_jerk = common_mode_jerk_mps3
        self.epoch = self.rng.uniform(0, 86400) if epoch_offset_s is None else epoch_offset_s
        # Walker 24/6/1
        self.sats = []
        for k in range(6):
            for j in range(4):
                self.sats.append(dict(prn=k * 4 + j + 1, raan=math.radians(60.0 * k),
                                      u0=math.radians(90.0 * j + 15.0 * k)))
        self.atmo_unit = {s["prn"]: self.rng.normal(0.0, 1.0) for s in self.sats}
        self.state = {s["prn"]: dict(locked=True, clear_since=None, delay=0.0, relock_t=None)
                      for s in self.sats}
        self.t_last = None
        self.acc_last = None
        self.next_t = None

    # ------------------------------------------------------------ geometry
    def sat_ecef(self, sat, t):
        """Satellite position and velocity (ECEF of time t) at GPS time t."""
        n = math.sqrt(MU / A_GPS ** 3)
        u = sat["u0"] + n * (t + self.epoch)
        O = sat["raan"]
        cu, su, cO, sO, ci, si = math.cos(u), math.sin(u), math.cos(O), math.sin(O), math.cos(INC), math.sin(INC)
        r = A_GPS * np.array([cO * cu - sO * su * ci, sO * cu + cO * su * ci, su * si])
        v = A_GPS * n * np.array([-cO * su - sO * cu * ci, -sO * su + cO * cu * ci, cu * si])
        th = OMGE * (t + self.epoch)
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        r_e = R @ r
        v_e = R @ v - np.cross([0.0, 0.0, OMGE], r_e)
        return r_e, v_e

    # ------------------------------------------------------------ clock
    def _clock(self, dt, g_excess):
        if dt > 0:
            self.clk_drift += self.rng.normal(0.0, math.sqrt(self.q_d * dt))
            d_eff = self.clk_drift + self.clk_g * g_excess
            self.clk_bias += d_eff * dt + self.rng.normal(0.0, math.sqrt(self.q_b * dt))
        return self.clk_drift + self.clk_g * g_excess

    # ------------------------------------------------------------ measure
    def due(self, t):
        if self.next_t is None:
            self.next_t = t
        return t >= self.next_t - 1e-9

    def measure(self, t, pos_ned, vel_ned, acc_ned, specific_force_g):
        """Raw measurements at receiver time t (s), from the true state in the
        reference NED frame. Returns (list[RawMeas], diagnostics dict)."""
        self.next_t = (self.next_t or t) + 1.0 / self.rate_hz
        dt = 0.0 if self.t_last is None else t - self.t_last
        jerk = 0.0
        if self.acc_last is not None and dt > 0:
            jerk = float(np.linalg.norm(np.asarray(acc_ned) - self.acc_last) / dt)
        self.t_last, self.acc_last = t, np.asarray(acc_ned, float)
        drift = self._clock(dt, specific_force_g - 1.0)

        Tn2e = self.T_ecef2ned.T
        r = self.ref_ecef + Tn2e @ np.asarray(pos_ned, float)
        v = Tn2e @ np.asarray(vel_ned, float)
        a = Tn2e @ np.asarray(acc_ned, float)
        lat, lon = self.ref_lla[0], self.ref_lla[1]
        T_local = t_e2ned(lat, lon)

        common_drop = self.cm_jerk is not None and jerk > self.cm_jerk
        out, diag = [], dict(visible=0, tracked=0, lost=[], jerk=jerk)
        for sat in self.sats:
            prn = sat["prn"]
            tau = 0.07
            for _ in range(3):
                rs, vs = self.sat_ecef(sat, t - tau)
                th = OMGE * tau
                c, s = math.cos(th), math.sin(th)
                rs_r = np.array([c * rs[0] + s * rs[1], -s * rs[0] + c * rs[1], rs[2]])
                rho = np.linalg.norm(rs_r - r)
                tau = rho / C_LIGHT
            vs_r = np.array([c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]])
            u = (rs_r - r) / rho
            el = math.asin(-(T_local @ u)[2])
            st = self.state[prn]
            if el < self.el_mask:
                st.update(locked=True, clear_since=None, relock_t=None, amb=None)   # re-acquired on rise
                continue
            diag["visible"] += 1
            a_los = float(-u @ a)
            # lock logic
            if st["locked"] and (abs(a_los) > self.lock_acc or common_drop):
                st.update(locked=False, clear_since=None)
            elif not st["locked"]:
                if abs(a_los) < self.relock_frac * self.lock_acc and not common_drop:
                    if st["clear_since"] is None:
                        mu_, sd, lo = self.reacq
                        st["clear_since"], st["delay"] = t, max(lo, self.rng.normal(mu_, sd))
                    elif t - st["clear_since"] >= st["delay"]:
                        st.update(locked=True, relock_t=t, clear_since=None, amb=None)
                else:
                    st["clear_since"] = None
            if not st["locked"]:
                diag["lost"].append(prn)
                continue
            diag["tracked"] += 1
            scale = 1.0
            if st["relock_t"] is not None:
                age = t - st["relock_t"]
                if age < self.relock_settle:
                    scale = 1.0 + (self.relock_noise - 1.0) * math.exp(-3.0 * age / self.relock_settle)
                else:
                    st["relock_t"] = None
            obliq = 1.0 / max(math.sin(el), 0.2)
            s_pr = self.sigma_pr * math.sqrt(obliq) * scale
            s_rr = self.sigma_rr * math.sqrt(obliq) * math.sqrt(scale)
            atmo = self.atmo_sigma * self.atmo_unit[prn] * obliq
            w, tw = self.wander.get(prn, (self.rng.normal(0.0, self.wander_sigma), t))
            if self.wander_tau > 0 and t > tw:              # first-order Gauss-Markov step
                phi = math.exp(-(t - tw) / self.wander_tau)
                w = phi * w + self.rng.normal(0.0, self.wander_sigma * math.sqrt(1.0 - phi * phi))
            self.wander[prn] = (w, t)
            pr = rho + self.clk_bias + atmo + w + self.rng.normal(0.0, s_pr)
            rr = float(u @ (vs_r - v)) + drift + self.rng.normal(0.0, s_rr)
            if self.doppler_step > 0:
                lam = L1_WAVELENGTH
                d = -rr / lam / self.doppler_step
                d = math.trunc(d)                           # the receiver truncates toward zero
                d = d + 0.5 * math.copysign(1.0, d) if d != 0 else 0.0   # ... the reader re-centres
                rr = -d * self.doppler_step * lam
                s_rr = math.hypot(s_rr, lam * self.doppler_step / math.sqrt(12.0))
            # carrier: the same range and clock, an unknown offset drawn afresh
            # at every (re)lock -- the count restarts -- and millimetres of noise
            slip = st.get("amb") is None
            k = math.sqrt(obliq)                            # weaker signal low in the sky
            if slip:
                st.update(amb=self.rng.normal(0.0, 1e4), cr_gm=self.rng.normal(0.0, self.cr_corr_sigma * k),
                          cr_rw=0.0, cr_t=t)
            elif t > st["cr_t"]:
                ddt = t - st["cr_t"]
                phi = math.exp(-ddt / self.cr_corr_tau) if self.cr_corr_tau > 0 else 0.0
                st["cr_gm"] = phi * st["cr_gm"] + self.rng.normal(0.0, self.cr_corr_sigma * k * math.sqrt(1 - phi * phi))
                st["cr_rw"] += self.rng.normal(0.0, self.cr_rw * k * math.sqrt(ddt))
                st["cr_t"] = t
            cr = (rho + self.clk_bias + atmo + st["amb"] + st["cr_gm"] + st["cr_rw"]
                  + self.rng.normal(0.0, self.sigma_cr * k))
            # the white part is fresh each epoch; the wander and the atmosphere
            # are not, and the filter is told so
            out.append(RawMeas(prn=prn, sat_pos=rs, sat_vel=vs, pr=pr, rr=rr,
                               sigma_pr=s_pr, sigma_rr=s_rr,
                               sigma_pr_corr=math.hypot(self.atmo_sigma * obliq, self.wander_sigma),
                               tau_pr=self.wander_tau, tau_rr=0.0,
                               cr=cr if self.sigma_cr >= 0 else None, cr_slip=slip,
                               sigma_cr=max(self.sigma_cr * k, 1e-4), sigma_cr_corr=self.cr_corr_sigma * k,
                               tau_cr=self.cr_corr_tau, q_cr=(self.cr_rw * k) ** 2))
        return out, diag
