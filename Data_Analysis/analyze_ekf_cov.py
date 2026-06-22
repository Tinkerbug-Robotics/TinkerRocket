#!/usr/bin/env python3
"""Replay a (stationary-bench) flight log through the EKF and report the
CONVERGED attitude/velocity covariance.

Used to set the #303 sensor-health scorecard thresholds
config::EKF_ATT_VAR_OK / EKF_VEL_VAR_OK from real numbers instead of guesses.
The firmware EKF bit reads OK when fmaxf(getCovOrient) < EKF_ATT_VAR_OK and
fmaxf(getCovVel) < EKF_VEL_VAR_OK, so this prints the worst-of-3-axes variance
the replayed filter (same TR_GpsInsEKF.cpp) settles to.

Usage: python analyze_ekf_cov.py <binary_file>
"""
import sys, math
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import parse_binary_file, get_array, pressure_to_altitude
from tinkerrocket_sim._ekf import GpsInsEKF, IMUData, GNSSDataLLA, MagData, BaroData

G = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
LOW_G_SAT = 15.5 * G


def main(binpath):
    records, stats, config = parse_binary_file(binpath)
    nmag = len(records.get("MMC5983MA", [])) + len(records.get("IIS2MDC", []))
    print(f"Frames: {stats['good_crc']:,} good, {stats['bad_crc']} bad CRC")
    print(f"IMU {len(records['ISM6HG256']):,}  GNSS {len(records['GNSS']):,}  "
          f"Baro {len(records['BMP585']):,}  Mag {nmag:,}")

    ev = []
    for r in records["ISM6HG256"]:        ev.append((r["time_us"], "imu", r))
    for r in records["GNSS"]:             ev.append((r["time_us"], "gnss", r))
    for r in records["BMP585"]:           ev.append((r["time_us"], "baro", r))
    for r in records.get("MMC5983MA", []): ev.append((r["time_us"], "mag", r))
    for r in records.get("IIS2MDC", []):   ev.append((r["time_us"], "mag", r))
    ev.sort(key=lambda e: e[0])
    t0 = records["ISM6HG256"][0]["time_us"]

    ekf = GpsInsEKF()
    init = False
    latest_gnss = None
    latest_mag = None
    gnss_counter = 0
    baro_ref = None
    baro_samp = []
    baro_off = 0.0
    baro_off_set = False
    max_sats = 0

    t = []
    covP = []
    covV = []
    covO = []
    n_imu = 0

    for time_us, etype, rec in ev:
        if etype == "gnss":
            max_sats = max(max_sats, rec.get("num_sats", 0))
            if rec.get("num_sats", 0) >= 4 and (latest_gnss is None or
                    rec["second"] != latest_gnss["second"] or
                    rec["milli_sec"] != latest_gnss["milli_sec"]):
                latest_gnss = rec
                gnss_counter += 1
        elif etype == "mag":
            latest_mag = rec
        elif etype == "baro":
            if baro_ref is None:
                baro_samp.append(rec["pressure_pa"])
                if len(baro_samp) >= 20:
                    baro_ref = float(np.mean(baro_samp))
                continue
            if not baro_off_set and latest_gnss is not None:
                baro_off = latest_gnss["alt_m"]
                baro_off_set = True
            balt = pressure_to_altitude(rec["pressure_pa"], baro_ref) + baro_off
            if init:
                b = BaroData(); b.time_us = time_us; b.altitude_m = balt
                ekf.baro_meas_update(b)
        elif etype == "imu":
            if latest_gnss is None:
                continue
            lx, ly, lz = rec["low_acc_x"], rec["low_acc_y"], rec["low_acc_z"]
            if abs(lx) > LOW_G_SAT or abs(ly) > LOW_G_SAT or abs(lz) > LOW_G_SAT:
                ax, ay, az = rec["high_acc_x"], rec["high_acc_y"], rec["high_acc_z"]
            else:
                ax, ay, az = lx, ly, lz
            imu = IMUData(); imu.time_us = time_us
            imu.acc_x = ax; imu.acc_y = -ay; imu.acc_z = -az
            imu.gyro_x = rec["gyro_x"]; imu.gyro_y = -rec["gyro_y"]; imu.gyro_z = -rec["gyro_z"]
            g = GNSSDataLLA(); g.time_us = gnss_counter
            g.lat_rad = latest_gnss["lat"] * DEG2RAD
            g.lon_rad = latest_gnss["lon"] * DEG2RAD
            g.alt_m = latest_gnss["alt_m"]
            g.vel_n_mps = latest_gnss["vel_n"]; g.vel_e_mps = latest_gnss["vel_e"]
            g.vel_d_mps = -latest_gnss["vel_u"]
            m = MagData()
            if latest_mag is not None:
                m.time_us = latest_mag["time_us"]
                m.mag_x = latest_mag["mag_x"]; m.mag_y = -latest_mag["mag_y"]; m.mag_z = -latest_mag["mag_z"]
            if not init:
                ekf.init_lla(imu, g, m); init = True; continue
            ekf.update_lla(True, imu, g, m)   # stationary bench → AHRS accel always
            n_imu += 1
            if n_imu % 20 == 0:
                t.append((time_us - t0) / 1e6)
                covP.append(ekf.get_cov_pos())
                covV.append(ekf.get_cov_vel())
                covO.append(ekf.get_cov_orient())

    t = np.array(t); covP = np.array(covP); covV = np.array(covV); covO = np.array(covO)
    if len(t) < 5:
        print(f"\nNot enough converged samples (max_sats={max_sats}). Aborting.")
        return
    print(f"replayed {n_imu:,} IMU updates  |  {len(t)} cov samples  |  "
          f"duration {t[-1]:.1f}s  |  max sats {max_sats}")

    k = max(1, int(len(t) * 0.7))   # converged = last 30% of a stationary run

    def report(name, seg, unit, ang=False):
        worst = seg.max(axis=1)
        med, p95, mx = np.median(worst), np.percentile(worst, 95), worst.max()
        s = (f"\n{name} (worst-of-3-axes variance, converged t>{t[k]:.0f}s):\n"
             f"  median={med:.3e}  p95={p95:.3e}  max={mx:.3e}  [{unit}^2]")
        if ang:
            s += (f"\n  -> 1sigma: median={math.sqrt(med)*RAD2DEG:.2f}deg  "
                  f"p95={math.sqrt(p95)*RAD2DEG:.2f}deg  max={math.sqrt(mx)*RAD2DEG:.2f}deg")
        else:
            s += (f"\n  -> 1sigma: median={math.sqrt(med):.3g}  "
                  f"p95={math.sqrt(p95):.3g}  max={math.sqrt(mx):.3g} {unit}")
        s += (f"\n  per-axis median var: [{np.median(seg[:,0]):.3e}, "
              f"{np.median(seg[:,1]):.3e}, {np.median(seg[:,2]):.3e}]")
        print(s)

    report("ATTITUDE  (getCovOrient -> EKF_ATT_VAR_OK)", covO, "rad", ang=True)
    report("VELOCITY  (getCovVel    -> EKF_VEL_VAR_OK)", covV, "m/s")
    report("POSITION  (getCovPos)", covP, "m")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
