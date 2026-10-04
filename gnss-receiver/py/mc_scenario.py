#!/usr/bin/env python3
"""Monte Carlo scenarios for mcsim: a flight launched at random times of day under the real satellites, every
signal the L1/L5 receiver design tracks, and each one's C/N0 from a link budget.

    mc_scenario.py --traj SCEN.csv --ignition 600 --nav BRDC.rnx --day-sow 172800 --runs 100 --out DIR

For run i (seeded by i) it draws:
  - the launch time, uniform over the broadcast file's day;
  - each signal's power above its specification minimum, and the antenna's and receiver's spread;
  - the oscillator's g-sensitivity along the thrust (lognormal around the TCXO's typical 0.07 ppb/g),
    the learnt clock estimate's first error, and the seeds mcsim uses;
and writes DIR/scen_NNN.csv (ch,sig,sat,t_s,dop_hz,el_deg,cn0_dbhz every 0.1 s, t from ignition) and a line in
DIR/runs.csv. DIR/flight.csv is the flight's axial acceleration per 0.1 s interval, for mcsim.

Signals: GPS L1 C/A from every healthy satellite and L5Q from the Block IIF and III ones; Galileo E1-C and
E5a-Q; BeiDou B1C and B2a pilots from BDS-3 (C19 and up). A satellite is tracked if it is 5 deg or more up
at ignition, on up to 32 channels a band (the FPGA's L1 bank), the lowest left out.

The link budget, per signal and 0.1 s:
  received power = the specification minimum (at 5 deg elevation, 0 dBic) + the satellite's excess
                   + its antenna's gain change from the off-nadir angle at the 5 deg reference (a shaped
                     beam, 2 dB down at nadir from the earth's edge)
                   - the path loss change from the reference range - the atmosphere's loss;
  C/N0 = received power + the patch's gain at the elevation (pointing up, no spin) - kT_sys - losses.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gnssrx import rinex, truth  # noqa: E402

C = 299792458.0
F_L1, F_L5 = 1575.42e6, 1176.45e6
RE = 6378137.0
# Signal: band, specification minimum received power (dBW, 5 deg, 0 dBic) of the tracked component, the
# receiver's losses on it (dB: front end, 2-bit quantization, and BOC(1,1) band-limiting on L1).
SIGNALS = {
    'L1CA': (F_L1, -158.5, 1.5),   # IS-GPS-200
    'L5Q': (F_L5, -157.9, 1.5),    # IS-GPS-705: each of I5 and Q5
    'E1C': (F_L1, -160.0, 2.0),    # OS SIS ICD: E1 -157.0, the pilot half
    'E5AQ': (F_L5, -158.0, 1.5),   # E5a -155.0, the pilot half
    'B1CP': (F_L1, -160.2, 2.0),   # BDS-SIS-ICD-B1C: -159.0, the pilot three quarters
    'B2AP': (F_L5, -159.0, 1.5),   # BDS-SIS-ICD-B2a: -156.0, the pilot half
}
SYS_SIGNALS = {'G': ('L1CA', 'L5Q'), 'E': ('E1C', 'E5AQ'), 'C': ('B1CP', 'B2AP')}
# Typical power above the minimum, dB (satellites usually beat the specification by a few dB).
EXCESS = {'G': 3.0, 'E': 2.5, 'C': 2.0}
# GPS Block IIR and IIR-M satellites, which send no L5 (assumed for the file's 2026 constellation).
GPS_NO_L5 = {2, 5, 7, 12, 15, 16, 17, 19, 20, 21, 22, 29, 31}
# The up-looking patch, dBic against elevation (deg); L5 1 dB lower. Below the horizon the body blocks.
ANT_EL = np.array([-90.0, -10.0, 0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0])
ANT_G = np.array([-30.0, -20.0, -10.0, -7.5, -5.5, -4.0, -2.5, -0.5, 1.5, 3.0, 4.0, 4.5])
T_ANT, NF_DB = 100.0, 2.0    # antenna temperature (K) and the receiver's noise figure (dB)
GAMMA_MEDIAN, GAMMA_SIGMA_LN = 0.07, 0.84   # ppb/g: 1 in 100 above 0.5
MASK_DEG = 5.0
CHANNELS_PER_BAND = 32


def kepler_pos(e: rinex.Eph, t: np.ndarray) -> np.ndarray:
    """ECEF positions (m) at the satellite's system times t (s of week), vectorized rinex.sat_pos."""
    a = e.sqrt_a ** 2
    tk = t - e.toe
    tk = np.where(tk > 302400, tk - 604800, np.where(tk < -302400, tk + 604800, tk))
    n = math.sqrt((rinex.GPS_MU if e.sys == 'G' else rinex.GAL_MU) / a ** 3) + e.delta_n
    m = e.m0 + n * tk
    ek = m.copy()
    for _ in range(30):
        ek = m + e.e * np.sin(ek)
    nu = np.arctan2(math.sqrt(1 - e.e ** 2) * np.sin(ek), np.cos(ek) - e.e)
    phi = nu + e.omega
    du = e.cus * np.sin(2 * phi) + e.cuc * np.cos(2 * phi)
    dr = e.crs * np.sin(2 * phi) + e.crc * np.cos(2 * phi)
    di = e.cis * np.sin(2 * phi) + e.cic * np.cos(2 * phi)
    u, r, i = phi + du, a * (1 - e.e * np.cos(ek)) + dr, e.i0 + di + e.idot * tk
    xp, yp = r * np.cos(u), r * np.sin(u)
    we = rinex.BDS_OMEGA_E if e.sys == 'C' else rinex.OMEGA_E
    om = e.omega0 + (e.omega_dot - we) * tk - we * e.toe
    return np.stack([xp * np.cos(om) - yp * np.cos(i) * np.sin(om), xp * np.sin(om) + yp * np.cos(i) * np.cos(om),
                     yp * np.sin(i)], -1)


def sat_rx(e: rinex.Eph, t_gps: np.ndarray, rx: np.ndarray) -> np.ndarray:
    """Satellite positions in the receive-time ECEF frame for signals received at t_gps by rx (n x 3)."""
    tau = np.full(t_gps.shape, 0.075)
    shift = rinex.BDT_MINUS_GPST if e.sys == 'C' else 0.0
    for _ in range(3):
        s = kepler_pos(e, t_gps - tau + shift)
        a = rinex.OMEGA_E * tau
        s = np.stack([np.cos(a) * s[:, 0] + np.sin(a) * s[:, 1], -np.sin(a) * s[:, 0] + np.cos(a) * s[:, 1],
                      s[:, 2]], -1)
        tau = np.linalg.norm(s - rx, axis=1) / C
    return s


class Flight:
    """The trajectory (t, lat, lon, h at 10 Hz) around ignition: ECEF positions, central-difference velocities
    and forward-difference accelerations, as gnssrx's IMU builds them."""

    def __init__(self, path: Path, ignition: float, t0: float, t1: float):
        d = np.loadtxt(path, delimiter=',')
        m = (d[:, 0] >= ignition + t0 - 0.25) & (d[:, 0] <= ignition + t1 + 0.25)
        d = d[m]
        self.t = d[:, 0] - ignition
        self.lat, self.lon = d[0, 1], d[0, 2]
        self.p = np.array([truth.geo_to_ecef(la, lo, h) for la, lo, h in d[:, 1:4]]).reshape(len(d), 3)
        n = len(self.t)
        a_idx = np.r_[0, np.arange(n - 2), n - 2]
        b_idx = np.r_[1, np.arange(2, n), n - 1]
        self.v = (self.p[b_idx] - self.p[a_idx]) / (self.t[b_idx] - self.t[a_idx])[:, None]
        la, lo = math.radians(self.lat), math.radians(self.lon)
        self.up = np.array([math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)])
        acc = np.zeros_like(self.p)
        acc[:-1] = (self.v[1:] - self.v[:-1]) / (self.t[1:] - self.t[:-1])[:, None]
        self.acc_up = acc @ self.up


def edge_angle(orbit_r: float) -> float:
    return math.asin(RE / orbit_r)


def off_nadir_at(orbit_r: float, el_deg: float) -> tuple[float, float]:
    """The off-nadir angle (rad) and range (m) for a user on the ground at elevation el_deg."""
    el = math.radians(el_deg)
    alpha = math.asin(RE * math.cos(el) / orbit_r)
    rng = math.sqrt(orbit_r ** 2 - (RE * math.cos(el)) ** 2) - RE * math.sin(el)
    return alpha, rng


def sat_gain(alpha: np.ndarray, orbit_r: float) -> np.ndarray:
    """The shaped beam's gain change, dB: 2 dB down at nadir from the earth's edge, quadratic between."""
    return -2.0 * (1.0 - np.clip(alpha / edge_angle(orbit_r), 0.0, 1.5) ** 2)


def atm_loss(el_deg: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Atmospheric absorption, dB: 0.035 dB at the zenith from the ground, thinning with height."""
    s = np.sin(np.radians(np.clip(el_deg, 2.0, 90.0)))
    return 0.035 / s * np.exp(-np.clip(h, 0.0, None) / 7000.0)


def run_one(i: int, fl: Flight, nav_by: dict, day_sow: float, out: Path, rows: list):
    rng = np.random.default_rng(1000 + i)
    t_rel = np.round(np.arange(-12.5, 25.55, 0.1), 3)
    k_src = np.searchsorted(np.round(fl.t, 3), t_rel)
    rx = fl.p[k_src]
    vrx = fl.v[k_src]
    up = fl.up
    t_launch = day_sow + rng.uniform(0.0, 86400.0 - 60.0)
    t_gps = t_launch + t_rel
    sys_ant = rng.normal(0.0, 0.5)
    nf = rng.normal(NF_DB, 0.3)
    n0 = -228.6 + 10 * math.log10(T_ANT + 290.0 * (10 ** (nf / 10) - 1.0))
    gamma = float(np.exp(rng.normal(math.log(GAMMA_MEDIAN), GAMMA_SIGMA_LN))) * float(rng.choice([-1.0, 1.0]))
    clk_err = float(rng.normal(0.0, 0.05))
    k0 = int(np.argmin(np.abs(t_rel)))
    hgt = (rx - rx[0]) @ up     # height above the pad, m
    sigs = []   # (band, elevation at ignition, signal, satellite, rows)
    for (sys_, prn), eph in sorted(nav_by.items()):
        if sys_ == 'C' and not (19 <= prn <= 46):
            continue
        e = min(eph, key=lambda c: abs(rinex.tdiff(rinex.sys_time(c, t_launch), c.toe)))
        if e.health != 0 or abs(rinex.tdiff(rinex.sys_time(e, t_launch), e.toe)) > 4 * 3600:
            continue
        s = sat_rx(e, t_gps, rx)
        los = s - rx
        rng_m = np.linalg.norm(los, axis=1)
        u = los / rng_m[:, None]
        el = np.degrees(np.arcsin(u @ up))
        if el[k0] < MASK_DEG:
            continue
        sp = sat_rx(e, t_gps + 0.05, rx)
        sm = sat_rx(e, t_gps - 0.05, rx)
        vsat = (sp - sm) / 0.1
        rr = np.einsum('ij,ij->i', vsat - vrx, u)            # range rate, m/s
        orbit_r = float(np.linalg.norm(s[k0]))
        cos_a = np.einsum('ij,ij->i', -los, -s) / (rng_m * np.linalg.norm(s, axis=1))
        alpha = np.arccos(np.clip(cos_a, -1.0, 1.0))
        a_ref, r_ref = off_nadir_at(orbit_r, 5.0)
        for sig in SYS_SIGNALS[sys_]:
            if sig == 'L5Q' and prn in GPS_NO_L5:
                continue
            fc, p_min, loss = SIGNALS[sig]
            lam = C / fc
            excess = rng.normal(EXCESS[sys_], 1.0)
            p_rx = (p_min + excess + sat_gain(alpha, orbit_r) - sat_gain(np.array([a_ref]), orbit_r)[0]
                    - 20 * np.log10(rng_m / r_ref) - atm_loss(el, hgt) + atm_loss(np.array([5.0]), np.array([0.0]))[0])
            g_ant = np.interp(el, ANT_EL, ANT_G) + sys_ant - (1.0 if fc == F_L5 else 0.0)
            cn0 = p_rx + g_ant - n0 - loss
            dop = -rr / lam
            sigs.append((fc, el[k0], sig, f"{sys_}{prn:02d}", (dop, el, cn0)))
    keep = []
    for band in (F_L1, F_L5):
        b = sorted((s for s in sigs if s[0] == band), key=lambda s: -s[1])
        keep += b[:CHANNELS_PER_BAND]
    lines, nsig = [], {s: 0 for s in SIGNALS}
    for ch, (_, _, sig, name, (dop, el, cn0)) in enumerate(keep):
        for tt, f, ee, c in zip(t_rel, dop, el, cn0):
            lines.append(f"{ch},{sig},{name},{tt:.2f},{f:.4f},{ee:.4f},{c:.3f}\n")
        nsig[sig] += 1
    (out / f"scen_{i:03d}.csv").write_text("ch,sig,sat,t_s,dop_hz,el_deg,cn0_dbhz\n" + "".join(lines))
    rows.append(f"{i},{t_launch:.1f},{gamma:.5f},{clk_err:.5f},{nf:.3f},{sys_ant:.3f},{1000 + i},{2000 + i},"
                + ",".join(str(nsig[s]) for s in SIGNALS) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--traj', type=Path, required=True)
    ap.add_argument('--ignition', type=float, required=True, help='ignition in the trajectory file, s')
    ap.add_argument('--nav', type=Path, required=True)
    ap.add_argument('--day-sow', type=float, required=True, help="the day's start, GPS seconds of week")
    ap.add_argument('--runs', type=int, default=100)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    fl = Flight(a.traj, a.ignition, -12.5, 25.6)
    with open(a.out / 'flight.csv', 'w') as f:
        f.write('t_s,acc_up\n')
        for t, acc in zip(fl.t, fl.acc_up):
            f.write(f'{t:.2f},{acc:.5f}\n')
    nav_by = {}
    for e in rinex.read_nav(a.nav, 'GEC'):
        nav_by.setdefault((e.sys, e.prn), []).append(e)
    rows = []
    for i in range(a.runs):
        run_one(i, fl, nav_by, a.day_sow, a.out, rows)
        print(f'run {i}: {rows[-1].strip()}', flush=True)
    (a.out / 'runs.csv').write_text('run,t_launch_sow,gamma_ppb,clk_err,nf_db,ant_db,seed,imu_seed,'
                                    + ','.join(SIGNALS) + '\n' + ''.join(rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
