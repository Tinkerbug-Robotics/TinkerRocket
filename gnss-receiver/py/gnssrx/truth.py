"""Truth for the rig's scenarios: the trajectory (10 Hz t, lat, lon, h CSV) and each
satellite's line-of-sight dynamics along it (range rate, Doppler, Doppler rate), from a
RINEX broadcast file.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import rinex

A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)
C = 299792458.0
LAMBDA_L1 = C / 1575.42e6


def geo_to_ecef(lat_deg, lon_deg, h):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
    return np.stack([(n + h) * np.cos(lat) * np.cos(lon), (n + h) * np.cos(lat) * np.sin(lon),
                     (n * (1 - E2) + h) * np.sin(lat)], axis=-1)


class Trajectory:
    """ECEF position, velocity and acceleration along a scenario CSV (time = file seconds)."""

    def __init__(self, csv_path: str | Path):
        d = np.loadtxt(csv_path, delimiter=",")
        self.t = d[:, 0]
        self.lat, self.lon, self.h = d[:, 1], d[:, 2], d[:, 3]
        self.pos = geo_to_ecef(self.lat, self.lon, self.h)
        self.vel = np.gradient(self.pos, self.t, axis=0)
        self.acc = np.gradient(self.vel, self.t, axis=0)

    def state(self, t):
        """Position and velocity at times t (linear interpolation of the 10 Hz samples)."""
        t = np.atleast_1d(t)
        p = np.stack([np.interp(t, self.t, self.pos[:, k]) for k in range(3)], axis=-1)
        v = np.stack([np.interp(t, self.t, self.vel[:, k]) for k in range(3)], axis=-1)
        return p, v

    def up_accel(self, t):
        """Acceleration along the local vertical, m/s^2."""
        t = np.atleast_1d(t)
        a = np.stack([np.interp(t, self.t, self.acc[:, k]) for k in range(3)], axis=-1)
        lat = np.radians(np.interp(t, self.t, self.lat))
        lon = np.radians(np.interp(t, self.t, self.lon))
        up = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=-1)
        return np.sum(a * up, axis=-1)

    def lat_h(self, t):
        """Geodetic latitude (rad) and height (m) at times t."""
        t = np.atleast_1d(t)
        return np.radians(np.interp(t, self.t, self.lat)), np.interp(t, self.t, self.h)


def sys_num(prn: int) -> tuple[str, int]:
    """gnssrx's satellite number as (RINEX system, PRN): Galileo 101-136, BeiDou 201-263."""
    return ("C", prn - 200) if prn >= 200 else (("E", prn - 100) if prn >= 100 else ("G", prn))


def sat_clock(nav: list[rinex.Eph], prn: int, t_gps) -> np.ndarray:
    """Each satellite's clock offset (s; broadcast polynomial, relativistic term and group delay) at GPS
    times t_gps; prn as gnssrx numbers it. Over a whole flight some drift by metres."""
    sys_, num = sys_num(prn)
    cands = [e for e in nav if e.prn == num and e.sys == sys_]
    t_gps = np.atleast_1d(t_gps)
    out = np.zeros(t_gps.size)
    for i, tg in enumerate(t_gps):
        e = min(cands, key=lambda c: abs(rinex.tdiff(rinex.sys_time(c, tg), c.toe)))
        out[i] = rinex.sat_pos(e, rinex.sys_time(e, tg - 0.075))[1]
    return out


def tropo_saastamoinen(lat, h, el, top=4e4):
    """Saastamoinen delay (m) with a standard atmosphere, as the receiver models it (core/pvt/pvt.c):
    lat and el in radians, h in metres, up to top (the receiver's 40 km). SignalSim's files carry one like
    it that stops at 10 km, all at once (the manifest's tropo_top_m)."""
    lat, h, el = np.broadcast_arrays(np.asarray(lat, float), np.asarray(h, float), np.asarray(el, float))
    hh = np.clip(h, 0.0, None)
    p = 1013.25 * (1.0 - 2.2557e-5 * hh) ** 5.2568
    tk = 15.0 - 6.5e-3 * hh + 273.16
    # Above the tropopause the water holds its temperature and thins with the pressure (the lapse rate
    # carried on divides by zero at 38.4 km).
    tw = np.where(hh > 11000.0, 15.0 - 6.5e-3 * 11000.0 + 273.16, tk)
    pw = np.where(hh > 11000.0, p / (1013.25 * (1.0 - 2.2557e-5 * 11000.0) ** 5.2568), 1.0)
    e = 6.108 * 0.7 * np.exp((17.15 * tw - 4684.0) / (tw - 38.45)) * pw
    cz = np.cos(np.pi / 2.0 - el)
    dry = 0.0022768 * p / (1.0 - 0.00266 * np.cos(2.0 * lat) - 0.00028 * hh / 1e3) / cz
    wet = 0.002277 * (1255.0 / tw + 0.05) * e / cz
    return np.where((h < -100.0) | (h > top) | (el <= 0.0), 0.0, dry + wet)


def _sat_rx_frame(e: rinex.Eph, t_rx_gps: float, rx: np.ndarray):
    """Satellite position (receive-time ECEF frame) for a signal received at t_rx_gps by rx."""
    tau = 0.075
    for _ in range(4):
        (x, y, z), _clk = rinex.sat_pos(e, rinex.sys_time(e, t_rx_gps - tau))
        a = rinex.OMEGA_E * tau
        s = np.array([math.cos(a) * x + math.sin(a) * y, -math.sin(a) * x + math.cos(a) * y, z])
        tau = float(np.linalg.norm(s - rx)) / C
    return s, tau


def _doppler(e: rinex.Eph, traj: Trajectory, tf: float, tg: float):
    """Doppler (Hz) from the satellite-minus-receiver velocity along the line of sight; also range and u."""
    p, v = traj.state(tf)
    p, v = p[0], v[0]
    s, tau = _sat_rx_frame(e, tg, p)
    s1, _ = _sat_rx_frame(e, tg + 0.5, p)
    s0, _ = _sat_rx_frame(e, tg - 0.5, p)
    vs = s1 - s0
    u = (s - p) / np.linalg.norm(s - p)
    return -float((vs - v) @ u) / LAMBDA_L1, float(np.linalg.norm(s - p)), u, p


def los(nav: list[rinex.Eph], prn: int, traj: Trajectory, t_file, t0_gps: float, dt: float = 0.05,
        with_az: bool = False):
    """Per sample time t_file: geometric range (m), Doppler (Hz), Doppler rate (Hz/s), elevation (deg),
    and with with_az the azimuth (deg from north, east positive) as a fifth column.

    t0_gps is the GPS time (s of week) of file second 0; prn as gnssrx numbers it (Galileo + 100,
    BeiDou + 200).
    """
    t_file = np.atleast_1d(t_file)
    sys, num = sys_num(prn)
    cands = [e for e in nav if e.prn == num and e.sys == sys]
    if not cands:
        raise KeyError(f"no ephemeris for PRN {prn}")
    out = np.zeros((t_file.size, 5 if with_az else 4))
    for i, tf in enumerate(t_file):
        tg = t0_gps + tf
        e = min(cands, key=lambda c: abs(rinex.tdiff(rinex.sys_time(c, tg), c.toe)))
        d0, rng, u, p = _doppler(e, traj, tf, tg)
        dm, _, _, _ = _doppler(e, traj, tf - dt, tg - dt)
        dp, _, _, _ = _doppler(e, traj, tf + dt, tg + dt)
        lat = math.atan2(p[2], math.hypot(p[0], p[1]))
        lon = math.atan2(p[1], p[0])
        up = np.array([math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])
        out[i, :4] = (rng, d0, (dp - dm) / (2 * dt), math.degrees(math.asin(float(u @ up))))
        if with_az:
            east = np.array([-math.sin(lon), math.cos(lon), 0.0])
            north = np.cross(up, east)
            out[i, 4] = math.degrees(math.atan2(float(u @ east), float(u @ north))) % 360.0
    return out
