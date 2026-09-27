#!/usr/bin/env python3
"""Tightly-coupled INS/GNSS on the PX1105R SLR capture through the 3 g gentle flight.

The real receiver's raw pseudorange + Doppler (SLR nav mode, 20 Hz) are fused as
per-satellite updates; a simulated IMU (100 Hz) and barometer (25 Hz), synthesized
from the injected truth with realistic noise, propagate the state between them and
coast it through the windows where the receiver withholds output. Scored against the
3 g truth, and run a second time with GNSS switched off (INS + baro only) so the
GNSS contribution is the difference.

    PYTHONPATH=src python3 scripts/tc_ekf_slr.py CAPTURE.log GENTLE_ALT.json \
        --pad-shift 180 --tow0 203400 [--csv out.csv] [--plot out.png]

Clocks: every COCOM IQ file starts at 2026/08/18 08:30:00 GPS (TOW 203400), whatever
its pad, so --tow0 stays 203400 and t is file time; --pad-shift is the pad minus the
scenario's 180 s prologue (180 for *_pad360, 420 for *_pad600). tc_ekf_capture.py
counts differently: its --tow0 is the TOW at the scenario's own t = 0 (203400 + pad
shift). The scoring phases and the plot range below are hard-coded for gentle_alt on
the 360 s pad (ignition at 360 s file time); shift them for anything else. The
captures and truth are under tools/gnss-cocom/sdr/results/ (see its README, "Data for
filter work"). Written 2026-09-26 for the results in that README's PX1105R section.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys

import numpy as np
from pathlib import Path

from tinkerrocket_sim.estimation.tc_ekf import (TcEkf, TcEkfParams, ecef2lla, lla2ecef,
                                                t_e2ned, G, spp_fix)
from tinkerrocket_sim.estimation.gnss_raw import read_capture, corrected

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "gnss-cocom"))
from gnss_nmea_monitor import replay_source  # noqa: E402


class Truth:
    """gentle_alt truth (10 Hz), shifted by pad_shift s into the padded file's clock.
    Vertical flight from origin: fixed latitude, small east drift, altitude."""

    def __init__(self, path, pad_shift):
        S = json.load(open(path))
        self.tr = S["truth"]
        self.shift = pad_shift
        self.ts = [s["t"] for s in self.tr]
        self.lat0 = math.radians(S["origin"]["lat_deg"])
        self.lon0 = math.radians(S["origin"]["lon_deg"])
        self.east = [0.0]
        for a, b in zip(self.tr, self.tr[1:]):
            self.east.append(self.east[-1] + 0.5 * (a["v_east_mps"] + b["v_east_mps"]) * (b["t"] - a["t"]))
        # NED acceleration by finite difference of (v_east, -v_up)
        self.a = [np.zeros(3)]
        for i in range(1, len(self.tr)):
            dt = self.ts[i] - self.ts[i - 1]
            dvn = 0.0
            dve = (self.tr[i]["v_east_mps"] - self.tr[i - 1]["v_east_mps"]) / dt
            dvd = (-self.tr[i]["v_up_mps"] - -self.tr[i - 1]["v_up_mps"]) / dt
            self.a.append(np.array([dvn, dve, dvd]))

    def _idx(self, t_orig):
        return max(1, min(len(self.ts) - 1, bisect.bisect_left(self.ts, t_orig)))

    def state(self, t_file):
        """(ecef pos, v_ned, a_ned, alt_m) at padded-file time t_file."""
        t = max(0.0, t_file - self.shift)
        i = self._idx(t)
        a, b = self.tr[i - 1], self.tr[i]
        w = (t - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
        alt = a["alt_m"] + w * (b["alt_m"] - a["alt_m"])
        ve = a["v_east_mps"] + w * (b["v_east_mps"] - a["v_east_mps"])
        vup = a["v_up_mps"] + w * (b["v_up_mps"] - a["v_up_mps"])
        e = self.east[i - 1] + w * (self.east[i] - self.east[i - 1])
        rew = 6378137.0 / math.sqrt(1 - 0.0066943799901 * math.sin(self.lat0) ** 2)
        lon = self.lon0 + e / ((rew + alt) * math.cos(self.lat0))
        pos = lla2ecef(np.array([self.lat0, lon, alt]))
        a_ned = self.a[i - 1] + w * (self.a[i] - self.a[i - 1])
        return pos, np.array([0.0, ve, -vup]), a_ned, alt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("scenario")
    ap.add_argument("--pad-shift", type=float, default=180.0)
    ap.add_argument("--tow0", type=float, default=203400.0)
    ap.add_argument("--imu-rate", type=float, default=100.0)
    ap.add_argument("--baro-rate", type=float, default=25.0)
    ap.add_argument("--accel-noise", type=float, default=0.05)     # m/s^2, tactical-ish
    ap.add_argument("--accel-bias", type=float, default=0.02)      # m/s^2 fixed offset
    ap.add_argument("--baro-noise", type=float, default=2.0)       # m
    ap.add_argument("--baro-max-hpa", type=float, default=300.0,
                    help="baro operating floor pressure; BMP585 spec is 300 hPa (~9.16 km). "
                         "Above this altitude the sensor is out of range and stops aiding.")
    ap.add_argument("--gyro-noise", type=float, default=0.002)     # rad/s
    ap.add_argument("--clk-bias-psd", type=float, default=1.0)
    ap.add_argument("--gnss-only", action="store_true",
                    help="no IMU/baro: constant-velocity kinematic model + raw GNSS only")
    ap.add_argument("--accel-psd", type=float, default=400.0, help="kinematic white-accel PSD m^2/s^3 (3 g ~400)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--csv")
    ap.add_argument("--plot")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    # BMP585 floor pressure -> altitude via the ISA troposphere law
    baro_ceiling = (1.0 - (a.baro_max_hpa / 1013.25) ** (1 / 5.25588)) / 2.25577e-5
    print(f"baro cuts off above {baro_ceiling:.0f} m ({a.baro_max_hpa:.0f} hPa, BMP585 spec)")

    truth = Truth(a.scenario, a.pad_shift)
    flight = "13.5 g spaceshot" if "spaceshot" in a.scenario else ("3 g gentle flight" if "gentle" in a.scenario else Path(a.scenario).stem)
    eph, epochs, own, kind = read_capture(a.capture, replay_source, systems="GEC", fix_doppler_truncation=True)
    epochs = [(tow, obs) for tow, obs in epochs if obs]        # keep only epochs that carry measurements
    et = [e[0] for e in epochs]
    print(f"{os.path.basename(a.capture)}: {kind}, {len(epochs)} raw epochs with measurements, "
          f"ephemeris for {len(eph.eph)} PRNs")

    def run(use_gnss):
        prm = TcEkfParams()
        prm.clk_bias_psd = a.clk_bias_psd
        ekf = TcEkf(prm)
        # first raw fix -> init position/velocity/clock; attitude level (self-consistent
        # with the synthesized specific force, gyro 0)
        started = False
        ab_true = rng.normal(0, a.accel_bias, 3)
        dt = 1.0 / a.imu_rate
        t0 = et[0] - a.tow0
        t_end = min(et[-1] - a.tow0 + 5.0, epochs[-1][0] - a.tow0 + 5.0)
        ei = 0
        next_baro = None
        track = []
        t = t0
        while t <= t_end:
            tow = t + a.tow0
            # bring in any raw epoch whose time we've reached
            while ei < len(epochs) and epochs[ei][0] <= tow + 1e-6:
                e_tow, obs = epochs[ei]; ei += 1
                if not started:
                    meas = corrected(e_tow, obs, eph, None, use_tropo=False)
                    fx = spp_fix(meas)
                    if fx and fx[1] is not None:
                        meas = corrected(e_tow, obs, eph, fx[0], use_tropo=False)
                        fx = spp_fix(meas, fx[0])
                    if not fx or fx[1] is None:
                        continue
                    lla = ecef2lla(fx[0])
                    ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx[1], [1.0, 0.0, 0.0, 0.0])
                    ekf.init_clock_from(meas)
                    started = True
                    t = e_tow - a.tow0
                    next_baro = t
                    continue
                if use_gnss:
                    r, _, _ = ekf.receiver_state_ecef()
                    meas = corrected(e_tow, obs, eph, r, use_tropo=False,
                                     doppler_step_hz=1.0 if kind == "skytraq" else 0.0)
                    if meas:
                        ekf.update_gnss_raw(meas)
            if not started:
                t += dt
                continue
            # IMU propagate one step with synthesized specific force (level frame)
            _, _, a_ned, _ = truth.state(t)
            acc_frd = a_ned - np.array([0.0, 0.0, G]) + ab_true + rng.normal(0, a.accel_noise, 3)
            gyro = rng.normal(0, a.gyro_noise, 3)
            ekf.propagate(acc_frd, gyro, dt)
            # baro update
            if next_baro is not None and t >= next_baro:
                _, _, _, alt = truth.state(t)
                if alt <= baro_ceiling:                 # BMP585 out of range above ~9.16 km
                    ekf.update_baro(alt + rng.normal(0, a.baro_noise))
                next_baro += 1.0 / a.baro_rate
            # record vs truth
            pos_t, v_t, _, alt_t = truth.state(t)
            r, v, _ = ekf.receiver_state_ecef()
            T = t_e2ned(truth.lat0, math.atan2(pos_t[1], pos_t[0]))
            d_ned = T @ (r - pos_t)
            dv_ned = (T @ v - v_t) if v is not None else np.zeros(3)   # v is ECEF -> NED, then minus NED truth
            has_g = use_gnss and (ei > 0 and abs(epochs[max(0, ei - 1)][0] - tow) < 0.15)
            track.append((t, d_ned.copy(), dv_ned.copy(), alt_t, ecef2lla(r)[2], has_g))
            t += dt
        return track

    def run_gnss_only():
        """GNSS-only: constant-velocity kinematic model, raw pseudorange+Doppler
        updates, no IMU and no baro. Runs at the raw-epoch rate (tc_ekf_capture's path)."""
        prm = TcEkfParams(); prm.clk_bias_psd = a.clk_bias_psd
        ekf = TcEkf(prm)
        started = False; t_prev = None; track = []
        for e_tow, obs in epochs:
            t = e_tow - a.tow0
            if not started:
                meas = corrected(e_tow, obs, eph, None, use_tropo=False)
                fx = spp_fix(meas)
                if not fx or fx[1] is None:
                    continue
                meas = corrected(e_tow, obs, eph, fx[0], use_tropo=False); fx = spp_fix(meas, fx[0])
                if not fx or fx[1] is None:
                    continue
                lla = ecef2lla(fx[0])
                ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx[1], [1.0, 0.0, 0.0, 0.0])
                ekf.freeze_imu_states(); ekf.init_clock_from(meas)
                started, t_prev = True, e_tow
                continue
            ekf.propagate_kinematic(e_tow - t_prev, a.accel_psd); t_prev = e_tow
            r, _, _ = ekf.receiver_state_ecef()
            meas = corrected(e_tow, obs, eph, r, use_tropo=False,
                             doppler_step_hz=1.0 if kind == "skytraq" else 0.0)
            if meas:
                ekf.update_gnss_raw(meas)
            pos_t, v_t, _, alt_t = truth.state(t)
            r, v, _ = ekf.receiver_state_ecef()
            T = t_e2ned(truth.lat0, math.atan2(pos_t[1], pos_t[0]))
            track.append((t, (T @ (r - pos_t)).copy(), (T @ v - v_t).copy(), alt_t, ecef2lla(r)[2], True))
        return track

    if a.gnss_only:
        tg = run_gnss_only(); ti = None
    else:
        tg = run(True); ti = run(False)

    def score(track, label):
        A = np.array([(t, d[0], d[1], d[2], dv[0], dv[1], dv[2]) for t, d, dv, *_ in track])
        # phases in padded-file time
        phases = [("pad", 300, 360), ("boost 360-420", 360, 420), (">500 up 390-471", 390, 471),
                  (">80km 501-546", 501, 546), ("coast/descent 471-576", 471, 576),
                  (">500 down 576-636", 576, 636), ("low descent 636-720", 636, 720)]
        print(f"\n== {label}: RMS error by phase (file time)")
        print(f"{'phase':<20}{'n':>6}{'horiz m':>9}{'vert m':>9}{'3D m':>8}{'vel m/s':>9}")
        for nm, t0, t1 in phases:
            m = (A[:, 0] >= t0) & (A[:, 0] < t1)
            if not m.any():
                continue
            h = np.sqrt(np.mean(A[m, 1] ** 2 + A[m, 2] ** 2))
            v = np.sqrt(np.mean(A[m, 3] ** 2))
            d3 = np.sqrt(np.mean(A[m, 1] ** 2 + A[m, 2] ** 2 + A[m, 3] ** 2))
            vv = np.sqrt(np.mean(A[m, 4] ** 2 + A[m, 5] ** 2 + A[m, 6] ** 2))
            print(f"{nm:<20}{m.sum():6d}{h:9.1f}{v:9.1f}{d3:8.1f}{vv:9.2f}")
        return A

    lbl = "GNSS only (SLR raw, kinematic)" if a.gnss_only else "INS + baro + GNSS (SLR raw)"
    Ag = score(tg, lbl)
    Ai = score(ti, "INS + baro only (no GNSS)") if ti is not None else None

    if a.csv:
        import csv
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "gnss_dN", "gnss_dE", "gnss_dD", "ins_dN", "ins_dE", "ins_dD"])
            gi = {round(t, 3): (d) for t, d, *_ in tg}
            for t, d, *_ in ti:
                g = gi.get(round(t, 3))
                w.writerow([f"{t:.3f}"] + [f"{x:.2f}" for x in (g if g is not None else [float('nan')] * 3)]
                           + [f"{x:.2f}" for x in d])
        print("wrote", a.csv)

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        INK, INK3, RULE = "#1d2129", "#6b7280", "#e3e6ea"
        tt = np.linspace(300, 720, 1600)
        # baro-ceiling crossings (ascent up, descent down)
        alts = np.array([truth.state(x)[3] for x in tt])
        fig, axs = plt.subplots(3, 1, figsize=(11, 8.2), dpi=140, sharex=True,
                                gridspec_kw=dict(hspace=0.13))
        # altitude
        axs[0].plot(tt, alts / 1000, color=INK3, lw=1.2, label="truth")
        axs[0].plot(Ag[:, 0], [tr[4] / 1000 for tr in tg], color="#1baf7a", lw=0.9, label=lbl)
        if not a.gnss_only:
            axs[0].axhline(baro_ceiling / 1000, color="#c0392b", lw=0.8, ls=(0, (4, 3)))
            axs[0].text(305, baro_ceiling / 1000 + 1.5, f"BMP585 ceiling {baro_ceiling/1000:.1f} km",
                        fontsize=6.5, color="#c0392b")
        axs[0].set_ylabel("altitude km", fontsize=8, color=INK3)
        axs[0].legend(fontsize=7, frameon=False, loc="center right")
        # position error, GNSS-aided, horizontal vs vertical (LINEAR)
        axs[1].plot(Ag[:, 0], np.sqrt(Ag[:, 1] ** 2 + Ag[:, 2] ** 2), color="#2a78d6", lw=0.9, label="horizontal")
        axs[1].plot(Ag[:, 0], np.abs(Ag[:, 3]), color="#eb6834", lw=0.9, label="vertical")
        axs[1].set_ylabel("position error m", fontsize=8, color=INK3)
        if not a.gnss_only:
            pk = np.sqrt(Ag[:, 1] ** 2 + Ag[:, 2] ** 2 + Ag[:, 3] ** 2).max()
            axs[1].set_ylim(0, 2500)
            axs[1].annotate(f"diverges to {pk/1000:.0f} km:\nno GNSS (>500 m/s) and\nabove baro ceiling",
                            xy=(636, 2500), xytext=(560, 1950), fontsize=6.8, color="#c0392b",
                            arrowprops=dict(arrowstyle="->", color="#c0392b", lw=0.8))
        axs[1].legend(fontsize=7, frameon=False, loc="upper left", title=lbl, title_fontsize=7)
        # velocity error, GNSS-aided (LINEAR)
        axs[2].plot(Ag[:, 0], np.sqrt(Ag[:, 4] ** 2 + Ag[:, 5] ** 2 + Ag[:, 6] ** 2),
                    color="#1baf7a", lw=0.9, label=lbl)
        axs[2].set_ylabel("velocity error m/s", fontsize=8, color=INK3)
        if not a.gnss_only:
            vpk = np.sqrt(Ag[:, 4] ** 2 + Ag[:, 5] ** 2 + Ag[:, 6] ** 2).max()
            axs[2].set_ylim(0, 250)
            axs[2].annotate(f"to {vpk:.0f} m/s", xy=(636, 250), xytext=(560, 195), fontsize=6.8,
                            color="#c0392b", arrowprops=dict(arrowstyle="->", color="#c0392b", lw=0.8))
        axs[2].set_xlabel("file time s (ignition 360)", fontsize=8, color=INK3)
        axs[2].legend(fontsize=7, frameon=False, loc="upper left")
        for ax in axs:
            for t0, t1 in ((390, 471), (576, 636)):
                ax.axvspan(t0, t1, color="#eda100", alpha=0.10, lw=0)
            ax.axvspan(501, 546, color="#7b61ff", alpha=0.08, lw=0)
            # baro-available spans (truth alt <= ceiling): dotted verticals where it cuts in/out
            for k in range(1, len(tt)):
                if (alts[k] > baro_ceiling) != (alts[k - 1] > baro_ceiling):
                    ax.axvline(tt[k], color="#c0392b", lw=0.6, ls=(0, (2, 3)))
            ax.tick_params(labelsize=7, colors=INK3)
            ax.grid(color=RULE, lw=0.4)
            for sp in ax.spines.values():
                sp.set_color(RULE)
        ttl = (f"PX1105R SLR raw, GNSS-only kinematic filter, {flight}" if a.gnss_only else
               f"PX1105R SLR raw + simulated IMU/baro (BMP585 cuts off above ceiling), tightly coupled, {flight}")
        axs[0].set_title(ttl + "\namber >500 m/s, violet >80 km" + ("" if a.gnss_only else ", red dashes = baro ceiling"),
                         fontsize=8.5, color=INK, loc="left")
        axs[0].set_xlim(300, 720)
        fig.savefig(a.plot, bbox_inches="tight")
        print("wrote", a.plot)


if __name__ == "__main__":
    main()
