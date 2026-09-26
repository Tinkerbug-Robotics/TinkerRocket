#!/usr/bin/env python3
"""Loosely vs tightly coupled GNSS in the standalone EKF, on a recorded sim flight.

Replays one closed-loop flight's IMU and baro into ``estimation.tc_ekf.TcEkf``
twice, changing nothing but the GNSS input:

  lc_model -- fixes from the sim's GNSSModel (18.18 Hz, white noise around
              truth, everything dropped above 5 g) -- the sim as it stands;
  lc       -- fixes formed each epoch from the SAME raw measurements the tc run
              gets (least squares, >= 4 satellites or no fix), fused as the
              flight filter fuses fixes -- so lc vs tc differs ONLY in coupling;
  tc       -- the raw measurements themselves, one satellite at a time.

The flight filter's own logged estimate (C++ TR_GpsInsEKF, LC) rides along as a
reference. Errors are against the sim's truth, per flight phase.

    PYTHONPATH=src python3 scripts/tc_ekf_eval.py --run G80T
    PYTHONPATH=src python3 scripts/tc_ekf_eval.py --log flight.pkl --lock 60
"""
from __future__ import annotations

import argparse
import math
import pickle
import sys

import numpy as np

from tinkerrocket_sim.estimation.tc_ekf import TcEkf, TcEkfParams, ecef2lla, lla2ecef, t_e2ned, G, spp_fix
from tinkerrocket_sim.sensors.gnss_model import GNSSModel
from tinkerrocket_sim.sensors.raw_gnss_model import RawGNSSModel


def load(args):
    if args.log:
        d = pickle.load(open(args.log, "rb"))
        return d["df"], d["cfg"]
    from tinkerrocket_sim.simulation.scenarios import build_rollypolly_iii
    from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop
    cfg = SimConfig(pad_time=30.0, duration=45.0, physics_dt=1e-3, imu_rate=1000.0,
                    log_interval=0.001, launch_angle_deg=87.0, control_enabled=False,
                    guidance_enabled=False, sensor_seed=args.seed)
    res = run_closed_loop(build_rollypolly_iii(motor=args.run), cfg)
    return res.df, cfg.__dict__


def run(df, cfg, mode, args):
    ref = np.array([math.radians(cfg["ref_lat_deg"]), math.radians(cfg["ref_lon_deg"]), cfg["ref_alt_m"]])
    T_ref = t_e2ned(ref[0], ref[1])
    ref_ecef = lla2ecef(ref)
    t = df["time"].to_numpy()
    p_ned = df[["true_pn", "true_pe", "true_pd"]].to_numpy()
    v_ned = df[["true_vn", "true_ve", "true_vd"]].to_numpy()
    a_ned = np.gradient(v_ned, t, axis=0)
    acc = np.column_stack([df["imu_acc_x"], -df["imu_acc_y"], -df["imu_acc_z"]])       # FLU -> FRD
    gyr = np.radians(np.column_stack([df["imu_gyro_x"], -df["imu_gyro_y"], -df["imu_gyro_z"]]))
    sf_g = np.linalg.norm(acc, axis=1) / G
    phase = df["flight_phase"].to_numpy()
    baro = df["baro_alt"].to_numpy() if "baro_alt" in df else None
    baro_ok = df["baro_valid"].to_numpy() if "baro_valid" in df else None
    enu = lambda x: np.array([x[1], x[0], -x[2]])

    gnss = GNSSModel(rate_hz=cfg.get("gnss_rate", 18.18), ref_lat_deg=cfg["ref_lat_deg"],
                     ref_lon_deg=cfg["ref_lon_deg"], ref_alt_m=cfg["ref_alt_m"], seed=args.seed + 3)
    raw = RawGNSSModel(ref_lat_deg=cfg["ref_lat_deg"], ref_lon_deg=cfg["ref_lon_deg"],
                       ref_alt_m=cfg["ref_alt_m"], rate_hz=args.raw_rate,
                       lock_los_accel_mps2=args.lock, common_mode_jerk_mps3=args.cm_jerk,
                       seed=args.seed + 7)
    ekf = TcEkf(TcEkfParams())
    baro_mode = args.baro
    i0 = int(np.searchsorted(t, args.t_init))
    q0 = df[["ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3"]].iloc[i0].to_numpy()
    # position from a fix, as a receiver would give it at power-up
    fix0 = gnss.measure(enu(p_ned[i0]), enu(v_ned[i0]))
    ekf.init(ecef2lla(np.array([fix0["ecef_x"], fix0["ecef_y"], fix0["ecef_z"]])), [0, 0, 0], q0)
    ekf.set_attitude_sigma(math.radians(2.0), math.radians(5.0))   # q0 comes from a settled filter
    gnss_dt, next_fix = 1.0 / gnss.rate_hz, t[i0]
    rec, n_sats, fix_ok, rej = [], [], [], 0
    last_baro = None
    for i in range(i0 + 1, len(t)):
        dt = t[i] - t[i - 1]
        ekf.propagate(acc[i], gyr[i], dt)
        if phase[i] in args.level_phases and 0.5 <= sf_g[i] <= 1.5:
            ekf.update_accel_level(acc[i])
        if baro is not None and baro[i] == baro[i] and baro[i] != last_baro:
            last_baro = baro[i]
            if (baro_ok is None or baro_ok[i]) and (baro_mode == "all" or
                                                    (baro_mode == "pad" and phase[i] == "PAD")):
                ekf.update_baro(baro[i] + cfg["ref_alt_m"])
        gyro_mag = float(np.linalg.norm(gyr[i]))
        if mode == "lc_model":
            if t[i] >= next_fix:
                next_fix += gnss_dt
                m = gnss.measure(enu(p_ned[i]), enu(v_ned[i]), accel_magnitude=sf_g[i] * G,
                                 gyro_magnitude_rps=gyro_mag, sim_time=t[i])
                fix_ok.append((t[i], m is not None))
                if m is not None:
                    x = np.array([m["ecef_x"], m["ecef_y"], m["ecef_z"]])
                    lla = ecef2lla(x)
                    v = t_e2ned(lla[0], lla[1]) @ np.array([m["ecef_vx"], m["ecef_vy"], m["ecef_vz"]])
                    ekf.update_gnss_fix(lla, v, noise_scale=m.get("noise_scale", 1.0))
        elif raw.due(t[i]):
            meas, diag = raw.measure(t[i], p_ned[i], v_ned[i], a_ned[i], sf_g[i])
            n_sats.append((t[i], diag["tracked"], diag["visible"]))
            if mode == "tc":
                st = ekf.update_gnss_raw(meas)
                rej += st.rejected_pr + st.rejected_rr
            else:
                fx = spp_fix(meas, lla2ecef(ekf.lla))
                fix_ok.append((t[i], fx is not None and fx[1] is not None))
                if fx is not None and fx[1] is not None:
                    lla = ecef2lla(fx[0])
                    ekf.update_gnss_fix(lla, t_e2ned(lla[0], lla[1]) @ fx[1])
        if i % 10 == 0:
            pe = T_ref @ (lla2ecef(ekf.lla) - ref_ecef)
            rec.append((t[i], *(pe - p_ned[i]), *(ekf.v - v_ned[i]), phase[i], ekf.clk[1]))
    return rec, n_sats, fix_ok, rej


def summarize(name, rows):
    """rows: (t, dpn, dpe, dpd, dvn, dve, dvd, phase) -> per-phase errors."""
    out = {}
    for ph in ("PAD", "BOOST", "COAST", "DESCENT"):
        r = [x for x in rows if x[7] == ph]
        if ph == "PAD":
            r = [x for x in r if x[0] > -5.0]
        if not r:
            continue
        a = np.array([x[1:7] for x in r], float)
        hz = np.hypot(a[:, 0], a[:, 1]); up = np.abs(a[:, 2])
        vh = np.hypot(a[:, 3], a[:, 4]); vu = np.abs(a[:, 5])
        out[ph] = (len(r), np.sqrt(np.mean(hz ** 2)), hz.max(), np.sqrt(np.mean(up ** 2)), up.max(),
                   np.sqrt(np.mean(vh ** 2)), np.sqrt(np.mean(vu ** 2)), vu.max())
    print(f"\n{name}")
    print("  phase     n   horiz rms/max m   alt rms/max m   vh rms m/s   vd rms/max m/s")
    for ph, v in out.items():
        print(f"  {ph:8s}{v[0]:5d}   {v[1]:6.2f} {v[2]:7.2f}   {v[3]:6.2f} {v[4]:7.2f}   {v[5]:7.2f}   {v[6]:6.2f} {v[7]:7.2f}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="pickle with df + cfg from a previous sim run")
    ap.add_argument("--run", default="G80T", help="motor to fly if no --log")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--t-init", dest="t_init", type=float, default=-20.0)
    ap.add_argument("--raw-rate", dest="raw_rate", type=float, default=10.0)
    ap.add_argument("--lock", type=float, default=30.0, help="LOS-acceleration knee, m/s^2")
    ap.add_argument("--cm-jerk", dest="cm_jerk", type=float, default=None)
    ap.add_argument("--baro", choices=("all", "pad", "none"), default="all",
                    help="baro updates: whole flight, pad only (GNSS carries altitude), or none")
    ap.add_argument("--level-phases", dest="level_phases", default="PAD",
                    type=lambda x: tuple(x.split(",")),
                    help="flight phases where the gravity-levelling update runs. Default PAD: "
                         "the sim flies without a parachute, so its descent is ballistic and "
                         "tumbling, and levelling there (the flight filter's PAD,DESCENT rule, "
                         "0.5-1.5 g gate) moved the tightly coupled position 11 m at apogee")
    ap.add_argument("--save", help="write the per-step records here (pickle)")
    args = ap.parse_args()
    df, cfg = load(args)
    print(f"flight: {len(df)} rows, t {df['time'].iloc[0]:.1f}..{df['time'].iloc[-1]:.1f} s, "
          f"apogee {-df['true_pd'].min():.0f} m, max speed {df['speed'].max():.0f} m/s, "
          f"peak specific force {np.linalg.norm(df[['imu_acc_x','imu_acc_y','imu_acc_z']].to_numpy(), axis=1).max()/G:.1f} g")
    ref_rows = []
    for _, r in df.iloc[::10].iterrows():
        if r.get("ekf_pn") == r.get("ekf_pn"):
            ref_rows.append((r["time"], r["ekf_pn"] - r["true_pn"], r["ekf_pe"] - r["true_pe"],
                             r["ekf_pd"] - r["true_pd"], r["ekf_vn"] - r["true_vn"],
                             r["ekf_ve"] - r["true_ve"], r["ekf_vd"] - r["true_vd"], r["flight_phase"]))
    summarize("flight filter (C++ TR_GpsInsEKF, logged; fixes, loosely coupled)", ref_rows)
    lcm, _, fix_model, _ = run(df, cfg, "lc_model", args)
    summarize("standalone EKF, loosely coupled, sim GNSSModel fixes", lcm)
    lc, _, fix_ok, _ = run(df, cfg, "lc", args)
    summarize("standalone EKF, loosely coupled, fixes from the raw measurements", lc)
    tc, n_sats, _, rej = run(df, cfg, "tc", args)
    summarize(f"standalone EKF, tightly coupled (raw, {args.raw_rate:g} Hz, LOS knee {args.lock:g} m/s^2)", tc)
    lost_fix = [x[0] for x in fix_ok if not x[1]]
    lost_m = [x[0] for x in fix_model if not x[1]]
    print(f"\nGNSSModel fixes withheld: {len(lost_m)} of {len(fix_model)}"
          + (f" ({min(lost_m):.2f}..{max(lost_m):.2f} s)" if lost_m else ""))
    print(f"fixes from raw withheld (<4 satellites): {len(lost_fix)} of {len(fix_ok)}"
          + (f" ({min(lost_fix):.2f}..{max(lost_fix):.2f} s)" if lost_fix else ""))
    fl = [x for x in n_sats if x[0] >= -0.5]
    if fl:
        print("raw: tracked/visible satellites from launch: " +
              " ".join(f"{x[0]:.1f}:{x[1]}/{x[2]}" for x in fl[:40:2]))
    print(f"raw measurements rejected by the innovation gate: {rej}")
    if args.save:
        pickle.dump(dict(ref=ref_rows, lc_model=lcm, lc=lc, tc=tc, n_sats=n_sats, fix_ok=fix_ok,
                         fix_model=fix_model, args=vars(args)), open(args.save, "wb"))


if __name__ == "__main__":
    sys.exit(main())
