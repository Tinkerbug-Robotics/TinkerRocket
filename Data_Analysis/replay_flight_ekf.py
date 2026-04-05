#!/usr/bin/env python3
"""Replay real flight sensor data through the EKF.

Parses a binary flight log, feeds IMU/GNSS/baro/mag packets to the
EKF at native timestamps, and compares EKF output against GNSS truth.

Usage:
    python replay_flight_ekf.py <binary_file> [--plot-dir DIR]
"""

import sys, math, argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import (parse_binary_file, get_array,
                                   pressure_to_altitude)

from tinkerrocket_sim._ekf import (GpsInsEKF, IMUData, GNSSDataLLA,
                                    MagData, BaroData)

G_MS2 = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Low-g saturation threshold: ±(16 - 0.5)g = ±15.5g
LOW_G_SAT_THRESH = 15.5 * G_MS2


def build_event_list(records):
    """Merge all sensor records into a single time-sorted event list.

    Each event is (time_us, type_str, record_dict).
    """
    events = []
    for r in records["ISM6HG256"]:
        events.append((r["time_us"], "imu", r))
    for r in records["GNSS"]:
        events.append((r["time_us"], "gnss", r))
    for r in records["BMP585"]:
        events.append((r["time_us"], "baro", r))
    for r in records["MMC5983MA"]:
        events.append((r["time_us"], "mag", r))
    events.sort(key=lambda e: e[0])
    return events


def detect_flight_phases(records, t0_us):
    """Detect boost start/end from accel magnitude.

    Returns (boost_start_us, apogee_us) in absolute time_us.
    """
    imu = records["ISM6HG256"]
    imu_times = get_array(imu, "time_us")

    # Downsample to ~100 Hz for phase detection
    step = max(1, len(imu) // (len(imu) // 20))
    accel_mag = []
    for i in range(0, len(imu), step):
        r = imu[i]
        a = math.sqrt(r["low_acc_x"]**2 + r["low_acc_y"]**2 + r["low_acc_z"]**2)
        accel_mag.append((imu_times[i], a))

    # Boost: sustained accel > 3g
    boost_start = None
    boost_end = None
    for t_us, a in accel_mag:
        if a > 3.0 * G_MS2 and boost_start is None:
            boost_start = t_us
        if boost_start is not None and a < 2.0 * G_MS2 and boost_end is None:
            boost_end = t_us

    # Apogee: find GNSS peak altitude after boost
    gnss = records["GNSS"]
    if gnss and boost_start:
        gnss_times = get_array(gnss, "time_us")
        gnss_alt = np.array([g["alt_m"] for g in gnss])
        flight_mask = gnss_times > boost_start
        if flight_mask.any():
            peak_idx = np.argmax(gnss_alt[flight_mask])
            apogee_us = gnss_times[flight_mask][peak_idx]
        else:
            apogee_us = boost_end + 5_000_000 if boost_end else None
    else:
        apogee_us = None

    return boost_start, boost_end, apogee_us


def replay(binary_file, plot_dir=None):
    print(f"Parsing: {binary_file}")
    records, stats, config = parse_binary_file(str(binary_file))
    print(f"  Frames: {stats['good_crc']:,} good, {stats['bad_crc']} bad CRC")
    print(f"  IMU: {len(records['ISM6HG256']):,}  GNSS: {len(records['GNSS']):,}  "
          f"Baro: {len(records['BMP585']):,}  Mag: {len(records['MMC5983MA']):,}")

    # Detect flight phases
    imu_times = get_array(records["ISM6HG256"], "time_us")
    t0_us = imu_times[0]
    boost_start, boost_end, apogee_us = detect_flight_phases(records, t0_us)
    if boost_start:
        print(f"  Boost: {(boost_start-t0_us)/1e6:.2f}s - {(boost_end-t0_us)/1e6:.2f}s")
        print(f"  Apogee: {(apogee_us-t0_us)/1e6:.2f}s")
    else:
        print("  WARNING: No boost detected")

    # Build time-sorted event list
    events = build_event_list(records)
    print(f"  Total events: {len(events):,}")

    # ---- Initialize EKF ----
    ekf = GpsInsEKF()
    ekf_initialized = False

    # Track latest sensor data for EKF
    latest_gnss = None
    latest_mag = None
    last_gnss_time_us = 0
    gnss_counter = 0  # monotonic counter for EKF new-data detection

    # Track latest baro reference pressure (from pad)
    baro_ref_pa = None
    baro_samples_for_ref = []
    baro_alt_offset = 0.0  # offset to align baro (pad=0) with GNSS altitude frame

    # Recording arrays
    log_time_us = []
    log_ekf_lat = []
    log_ekf_lon = []
    log_ekf_alt = []
    log_ekf_vn = []
    log_ekf_ve = []
    log_ekf_vd = []
    log_ekf_roll = []
    log_ekf_pitch = []
    log_ekf_yaw = []
    log_ekf_q = []
    log_ekf_gyro_bias = []
    log_ekf_accel_bias = []
    log_cov_pos = []
    log_cov_vel = []
    log_cov_att = []

    # GNSS truth arrays
    gnss_log_time = []
    gnss_log_lat = []
    gnss_log_lon = []
    gnss_log_alt = []
    gnss_log_vn = []
    gnss_log_ve = []
    gnss_log_vd = []

    # Baro truth
    baro_log_time = []
    baro_log_alt = []

    n_imu = 0
    n_gnss_updates = 0
    n_baro_updates = 0

    print("\nReplaying...")
    for time_us, etype, rec in events:
        t_rel = (time_us - t0_us) / 1e6

        if etype == "gnss":
            # Skip packets without a solid fix (matches onboard GNSS_MIN_SATS)
            has_fix = rec.get("num_sats", 0) >= 4
            if has_fix:
                # Record GNSS truth only when fix is valid
                gnss_log_time.append(time_us)
                gnss_log_lat.append(rec["lat"])
                gnss_log_lon.append(rec["lon"])
                gnss_log_alt.append(rec["alt_m"])
                gnss_log_vn.append(rec["vel_n"])
                gnss_log_ve.append(rec["vel_e"])
                gnss_log_vd.append(-rec["vel_u"])  # Up → Down

            # De-duplicate: only count as new if fix is valid and the GPS
            # fix timestamp (second + milli_sec) changed.  The MCU time_us
            # is different every poll even for the same receiver fix.
            if has_fix and (latest_gnss is None or
                    rec["second"] != latest_gnss["second"] or
                    rec["milli_sec"] != latest_gnss["milli_sec"]):
                latest_gnss = rec
                gnss_counter += 1

        elif etype == "mag":
            latest_mag = rec

        elif etype == "baro":
            # Collect pad baro samples for reference pressure (first 20 samples)
            if baro_ref_pa is None:
                baro_samples_for_ref.append(rec["pressure_pa"])
                if len(baro_samples_for_ref) >= 20:
                    baro_ref_pa = np.mean(baro_samples_for_ref)
                    print(f"  Baro ref pressure: {baro_ref_pa:.2f} Pa "
                          f"(from {len(baro_samples_for_ref)} samples at t={t_rel:.2f}s)")
                continue

            # Defer baro offset until we have a valid GNSS fix
            if baro_alt_offset == 0.0 and latest_gnss is not None:
                baro_alt_offset = latest_gnss["alt_m"]
                print(f"  Baro alt offset: {baro_alt_offset:.1f}m "
                      f"(from GNSS at t={t_rel:.1f}s)")

            baro_alt = pressure_to_altitude(rec["pressure_pa"], baro_ref_pa) + baro_alt_offset
            baro_log_time.append(time_us)
            baro_log_alt.append(baro_alt)

            # Feed baro to EKF (if initialized and not during transonic)
            if ekf_initialized:
                baro_d = BaroData()
                baro_d.time_us = time_us
                baro_d.altitude_m = baro_alt
                ekf.baro_meas_update(baro_d)
                n_baro_updates += 1

        elif etype == "imu":
            if latest_gnss is None:
                continue  # Need at least one GNSS fix before init

            # Select accel source (low-g or high-g based on saturation)
            lx = rec["low_acc_x"]
            ly = rec["low_acc_y"]
            lz = rec["low_acc_z"]
            if (abs(lx) > LOW_G_SAT_THRESH or
                abs(ly) > LOW_G_SAT_THRESH or
                abs(lz) > LOW_G_SAT_THRESH):
                ax_board = rec["high_acc_x"]
                ay_board = rec["high_acc_y"]
                az_board = rec["high_acc_z"]
            else:
                ax_board = lx
                ay_board = ly
                az_board = lz

            # Board frame (FLU) → EKF body frame (FRD)
            imu_d = IMUData()
            imu_d.time_us = time_us
            imu_d.acc_x = ax_board           # X same
            imu_d.acc_y = -ay_board           # FLU Y=Left → FRD Y=Right
            imu_d.acc_z = -az_board           # FLU Z=Up → FRD Z=Down
            imu_d.gyro_x = rec["gyro_x"]      # X same (dps → the EKF converts internally?)
            imu_d.gyro_y = -rec["gyro_y"]      # FLU→FRD
            imu_d.gyro_z = -rec["gyro_z"]      # FLU→FRD

            # Prepare GNSS data (LLA path)
            gnss_d = GNSSDataLLA()
            gnss_d.time_us = gnss_counter  # EKF detects new data when this changes
            gnss_d.lat_rad = latest_gnss["lat"] * DEG2RAD
            gnss_d.lon_rad = latest_gnss["lon"] * DEG2RAD
            gnss_d.alt_m = latest_gnss["alt_m"]
            gnss_d.vel_n_mps = latest_gnss["vel_n"]
            gnss_d.vel_e_mps = latest_gnss["vel_e"]
            gnss_d.vel_d_mps = -latest_gnss["vel_u"]  # Up → Down

            # Prepare mag data (board FLU → FRD)
            mag_d = MagData()
            if latest_mag is not None:
                mag_d.time_us = latest_mag["time_us"]
                mag_d.mag_x = latest_mag["mag_x"]        # X same
                mag_d.mag_y = -latest_mag["mag_y"]        # FLU→FRD
                mag_d.mag_z = -latest_mag["mag_z"]        # FLU→FRD

            if not ekf_initialized:
                ekf.init_lla(imu_d, gnss_d, mag_d)

                # Pad heading init from accel + known heading
                g_mag = math.sqrt(imu_d.acc_x**2 + imu_d.acc_y**2 +
                                  imu_d.acc_z**2)
                if g_mag > 0.1:
                    pitch_rad = math.asin(max(-1, min(1, imu_d.acc_x / g_mag)))
                    if abs(pitch_rad) > math.radians(80.0):
                        roll_rad = 0.0
                    else:
                        roll_rad = math.atan2(-imu_d.acc_y, -imu_d.acc_z)
                    # Use 0° heading (north) as default — we don't know actual heading
                    yaw_rad = 0.0
                    cy = math.cos(yaw_rad/2); sy = math.sin(yaw_rad/2)
                    cp = math.cos(pitch_rad/2); sp = math.sin(pitch_rad/2)
                    cr = math.cos(roll_rad/2); sr = math.sin(roll_rad/2)
                    qw = cr*cp*cy + sr*sp*sy
                    qx = sr*cp*cy - cr*sp*sy
                    qy = cr*sp*cy + sr*cp*sy
                    qz = cr*cp*sy - sr*sp*cy
                    ekf.set_quaternion(qw, qx, qy, qz)

                ekf_initialized = True
                print(f"  EKF initialized at t={t_rel:.2f}s  "
                      f"pitch={math.degrees(pitch_rad):.1f}°")
                continue

            # Determine use_ahrs_acc based on flight phase
            if boost_start and boost_end and apogee_us:
                in_boost = boost_start <= time_us <= boost_end
                in_coast = boost_end < time_us <= apogee_us
                use_ahrs_acc = not (in_boost or in_coast)
            else:
                use_ahrs_acc = True  # no flight detected, pad only

            ekf.update_lla(use_ahrs_acc, imu_d, gnss_d, mag_d)
            n_imu += 1

            if gnss_d.time_us != last_gnss_time_us:
                n_gnss_updates += 1
                last_gnss_time_us = gnss_d.time_us

            # Log EKF output (every 20th sample ≈ 100 Hz)
            if n_imu % 20 == 0:
                log_time_us.append(time_us)
                pos = ekf.get_position()
                vel = ekf.get_velocity()
                ori = ekf.get_orientation()
                q = ekf.get_quaternion()
                gb = ekf.get_rot_rate_bias()
                ab = ekf.get_accel_bias()
                cp = ekf.get_cov_pos()
                cv = ekf.get_cov_vel()
                ca = ekf.get_cov_orient()
                log_ekf_lat.append(pos[0])  # rad
                log_ekf_lon.append(pos[1])  # rad
                log_ekf_alt.append(pos[2])  # m
                log_ekf_vn.append(vel[0])
                log_ekf_ve.append(vel[1])
                log_ekf_vd.append(vel[2])
                log_ekf_roll.append(ori[0] * RAD2DEG)
                log_ekf_pitch.append(ori[1] * RAD2DEG)
                log_ekf_yaw.append(ori[2] * RAD2DEG)
                log_ekf_q.append(q)
                log_ekf_gyro_bias.append(
                    (gb[0]*RAD2DEG, gb[1]*RAD2DEG, gb[2]*RAD2DEG))
                log_ekf_accel_bias.append(ab)
                log_cov_pos.append(cp)
                log_cov_vel.append(cv)
                log_cov_att.append(ca)

    print(f"\n  Processed: {n_imu:,} IMU updates, {n_gnss_updates} GNSS updates, "
          f"{n_baro_updates} baro updates")

    # ---- Convert to numpy ----
    t_ekf = (np.array(log_time_us) - t0_us) / 1e6
    ekf_lat = np.array(log_ekf_lat)
    ekf_lon = np.array(log_ekf_lon)
    ekf_alt = np.array(log_ekf_alt)
    ekf_vn = np.array(log_ekf_vn)
    ekf_ve = np.array(log_ekf_ve)
    ekf_vd = np.array(log_ekf_vd)
    ekf_roll = np.array(log_ekf_roll)
    ekf_pitch = np.array(log_ekf_pitch)
    ekf_yaw = np.array(log_ekf_yaw)
    gyro_bias = np.array(log_ekf_gyro_bias)
    accel_bias = np.array(log_ekf_accel_bias)
    cov_pos = np.array(log_cov_pos)
    cov_vel = np.array(log_cov_vel)
    cov_att = np.array(log_cov_att)

    t_gnss = (np.array(gnss_log_time) - t0_us) / 1e6
    g_lat = np.array(gnss_log_lat)
    g_lon = np.array(gnss_log_lon)
    g_alt = np.array(gnss_log_alt)
    g_vn = np.array(gnss_log_vn)
    g_ve = np.array(gnss_log_ve)
    g_vd = np.array(gnss_log_vd)

    # Convert EKF lat/lon to local NE (m) relative to first GNSS fix
    R_earth = 6378137.0
    ref_lat = g_lat[0] * DEG2RAD
    ref_lon = g_lon[0] * DEG2RAD
    ref_alt = g_alt[0]

    ekf_n = (ekf_lat - ref_lat) * R_earth
    ekf_e = (ekf_lon - ref_lon) * R_earth * math.cos(ref_lat)
    ekf_u = ekf_alt - ref_alt

    gnss_n = (g_lat * DEG2RAD - ref_lat) * R_earth
    gnss_e = (g_lon * DEG2RAD - ref_lon) * R_earth * math.cos(ref_lat)
    gnss_u = g_alt - ref_alt

    # Print summary
    print("\n" + "=" * 60)
    print("REPLAY SUMMARY")
    print("=" * 60)
    print(f"  EKF altitude range: {ekf_u.min():.1f} to {ekf_u.max():.1f} m")
    print(f"  GNSS altitude range: {gnss_u.min():.1f} to {gnss_u.max():.1f} m")
    print(f"  EKF pitch range: {ekf_pitch.min():.1f} to {ekf_pitch.max():.1f} deg")
    print(f"  Gyro bias final: X={gyro_bias[-1,0]:.3f} Y={gyro_bias[-1,1]:.3f} "
          f"Z={gyro_bias[-1,2]:.3f} dps")
    print(f"  Accel bias final: X={accel_bias[-1,0]:.3f} Y={accel_bias[-1,1]:.3f} "
          f"Z={accel_bias[-1,2]:.3f} m/s²")

    # ---- Plot ----
    if plot_dir is None:
        plot_dir = Path(__file__).parent.parent / "plots"
    plot_dir = Path(plot_dir)
    plot_dir.mkdir(exist_ok=True)

    # Flight phase shading helper
    def shade_phases(ax):
        if boost_start and boost_end:
            bs = (boost_start - t0_us) / 1e6
            be = (boost_end - t0_us) / 1e6
            ax.axvspan(bs, be, color='red', alpha=0.08, label='Boost')
        if apogee_us:
            ap = (apogee_us - t0_us) / 1e6
            ax.axvline(ap, color='blue', ls=':', lw=0.8, alpha=0.5, label='Apogee')

    # --- Fig 1: Position (NE + altitude) ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Flight Replay: EKF Position vs GNSS', fontsize=14, fontweight='bold')

    axes[0].plot(t_gnss, gnss_n, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[0].plot(t_ekf, ekf_n, '-', lw=1, color='C3', label='EKF')
    axes[0].set_ylabel('North (m)')
    axes[0].legend(fontsize=9); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_gnss, gnss_e, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[1].plot(t_ekf, ekf_e, '-', lw=1, color='C3', label='EKF')
    axes[1].set_ylabel('East (m)')
    axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

    axes[2].plot(t_gnss, gnss_u, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
    axes[2].plot(t_ekf, ekf_u, '-', lw=1, color='C3', label='EKF')
    if baro_log_time:
        t_baro = (np.array(baro_log_time) - t0_us) / 1e6
        baro_agl = np.array(baro_log_alt) - ref_alt
        axes[2].plot(t_baro, baro_agl, '.', ms=1, color='C2', alpha=0.2,
                     label='Baro')
    axes[2].set_ylabel('Altitude AGL (m)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(fontsize=9); axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    plt.tight_layout()
    out1 = plot_dir / 'replay_position.png'
    plt.savefig(out1, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 2: Velocity ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Flight Replay: EKF Velocity vs GNSS', fontsize=14, fontweight='bold')

    for i, (label, ekf_v, gnss_v) in enumerate([
        ('North', ekf_vn, g_vn),
        ('East', ekf_ve, g_ve),
        ('Down', ekf_vd, g_vd),
    ]):
        axes[i].plot(t_gnss, gnss_v, '.', ms=2, color='C0', alpha=0.3, label='GNSS')
        axes[i].plot(t_ekf, ekf_v, '-', lw=1, color='C3', label='EKF')
        axes[i].set_ylabel(f'Vel {label} (m/s)')
        axes[i].legend(fontsize=9); axes[i].grid(True, alpha=0.3)
        shade_phases(axes[i])

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    out2 = plot_dir / 'replay_velocity.png'
    plt.savefig(out2, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 3: Attitude + gyro bias ---
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Flight Replay: EKF Attitude & Gyro Bias', fontsize=14, fontweight='bold')

    axes[0].plot(t_ekf, ekf_roll, '-', lw=1, color='C0')
    axes[0].set_ylabel('Roll (deg)'); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_ekf, ekf_pitch, '-', lw=1, color='C1')
    axes[1].set_ylabel('Pitch (deg)'); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

    axes[2].plot(t_ekf, ekf_yaw, '-', lw=1, color='C2')
    axes[2].set_ylabel('Yaw (deg)'); axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    axes[3].plot(t_ekf, gyro_bias[:, 0], '-', lw=1, label='X')
    axes[3].plot(t_ekf, gyro_bias[:, 1], '-', lw=1, label='Y')
    axes[3].plot(t_ekf, gyro_bias[:, 2], '-', lw=1, label='Z')
    axes[3].set_ylabel('Gyro bias (dps)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(fontsize=9); axes[3].grid(True, alpha=0.3)
    shade_phases(axes[3])

    plt.tight_layout()
    out3 = plot_dir / 'replay_attitude.png'
    plt.savefig(out3, dpi=180, bbox_inches='tight')
    plt.close()

    # --- Fig 4: Covariance (3-sigma bounds) ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle('Flight Replay: EKF Covariance (3σ)', fontsize=14, fontweight='bold')

    sig_pos = 3 * np.sqrt(cov_pos[:, 0]**2 + cov_pos[:, 1]**2 + cov_pos[:, 2]**2)
    sig_vel = 3 * np.sqrt(cov_vel[:, 0]**2 + cov_vel[:, 1]**2 + cov_vel[:, 2]**2)
    sig_att = 3 * np.sqrt(cov_att[:, 0]**2 + cov_att[:, 1]**2 + cov_att[:, 2]**2) * RAD2DEG

    axes[0].plot(t_ekf, sig_pos, '-', lw=1, color='C0')
    axes[0].set_ylabel('3σ Position (m)'); axes[0].grid(True, alpha=0.3)
    shade_phases(axes[0])

    axes[1].plot(t_ekf, sig_vel, '-', lw=1, color='C1')
    axes[1].set_ylabel('3σ Velocity (m/s)'); axes[1].grid(True, alpha=0.3)
    shade_phases(axes[1])

    axes[2].plot(t_ekf, sig_att, '-', lw=1, color='C2')
    axes[2].set_ylabel('3σ Attitude (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)
    shade_phases(axes[2])

    plt.tight_layout()
    out4 = plot_dir / 'replay_covariance.png'
    plt.savefig(out4, dpi=180, bbox_inches='tight')
    plt.close()

    plots = [out1, out2, out3, out4]
    print(f"\nPlots saved to {plot_dir}/")
    return plots


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay flight data through EKF")
    parser.add_argument("binary_file", nargs="?",
                        default=str(Path(__file__).parent.parent.parent /
                                    "TestFlights/2026_03_08/Raw Downloads/"
                                    "Goblin Flight 2 F52/flight_20260308_190239.bin"))
    parser.add_argument("--plot-dir", default=None)
    args = parser.parse_args()

    plots = replay(Path(args.binary_file), args.plot_dir)

    import subprocess
    subprocess.run(["open"] + [str(p) for p in plots])
