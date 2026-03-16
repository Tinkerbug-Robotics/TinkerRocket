#!/usr/bin/env python3
"""Analyze sensor noise from binary flight log to validate sim error models.

Parses binary log, identifies pad (stationary) phase, computes noise statistics
for IMU, GNSS, barometer, and magnetometer.
"""

import sys, math
from pathlib import Path
import numpy as np

# Reuse the existing parser
sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import parse_binary_file, get_array, pressure_to_altitude

BINARY_FILE = Path(__file__).parent.parent.parent / \
    "TestFlights/2026_03_08/Raw Downloads/Goblin Flight 2 F52/flight_20260308_190239.bin"


def analyze():
    print(f"Parsing: {BINARY_FILE}")
    records, stats, config = parse_binary_file(str(BINARY_FILE))

    print(f"\nParsing stats:")
    print(f"  File size: {stats['file_size']:,} bytes")
    print(f"  Total frames: {stats['total_frames']:,}")
    print(f"  Good CRC: {stats['good_crc']:,}  Bad CRC: {stats['bad_crc']:,}")
    print(f"  Type counts: {stats['type_counts']}")
    print(f"\nSensor config from log:")
    for k, v in config.items():
        print(f"  {k}: {v}")

    # --- Find pad phase ---
    # Use NonSensor records to find when rocket_state transitions from READY to INFLIGHT
    ns = records["NonSensor"]
    if not ns:
        print("ERROR: No NonSensor records found")
        return

    ns_times = get_array(ns, "time_us")
    ns_states = np.array([r["rocket_state"] for r in ns])
    t0_us = ns_times[0]

    # Find launch time (first INFLIGHT=3 transition)
    inflight_idx = np.where(ns_states == 3)[0]
    if len(inflight_idx) > 0:
        launch_time_us = ns_times[inflight_idx[0]]
        print(f"\nLaunch detected at t={((launch_time_us - t0_us)/1e6):.2f}s (time_us={launch_time_us})")
    else:
        print("WARNING: No INFLIGHT state found, using last 5s as pad phase")
        launch_time_us = ns_times[-1]

    # Use 3 seconds of pad data ending 0.5s before launch for clean stationary data
    pad_end_us = launch_time_us - 500_000  # 0.5s before launch
    pad_start_us = pad_end_us - 3_000_000   # 3s window

    print(f"Pad analysis window: [{(pad_start_us-t0_us)/1e6:.2f}, {(pad_end_us-t0_us)/1e6:.2f}]s")

    # ============================================================
    # IMU ANALYSIS
    # ============================================================
    print("\n" + "=" * 70)
    print("IMU (ISM6HG256) - Pad Noise Analysis")
    print("=" * 70)

    imu = records["ISM6HG256"]
    imu_times = get_array(imu, "time_us")

    # Filter to pad phase
    pad_mask = (imu_times >= pad_start_us) & (imu_times <= pad_end_us)
    n_pad_imu = pad_mask.sum()
    print(f"  Pad IMU samples: {n_pad_imu}")

    if n_pad_imu > 100:
        # Effective sample rate
        pad_imu_dt = np.diff(imu_times[pad_mask]) / 1e6
        imu_rate = 1.0 / np.median(pad_imu_dt)
        print(f"  IMU rate: {imu_rate:.0f} Hz (median dt={np.median(pad_imu_dt)*1e3:.3f} ms)")

        for prefix, label in [("low_acc", "Low-G Accel"), ("high_acc", "High-G Accel"), ("gyro", "Gyro")]:
            print(f"\n  {label}:")
            for axis in ['x', 'y', 'z']:
                key = f"{prefix}_{axis}"
                vals = np.array([imu[i][key] for i in range(len(imu)) if pad_mask[i]])
                unit = "m/s²" if "acc" in prefix else "dps"
                print(f"    {axis}: mean={vals.mean():10.4f} std={vals.std():8.4f} {unit}")

        # Compute accel magnitude on pad (should be ~9.81 m/s²)
        ax = np.array([imu[i]["low_acc_x"] for i in range(len(imu)) if pad_mask[i]])
        ay = np.array([imu[i]["low_acc_y"] for i in range(len(imu)) if pad_mask[i]])
        az = np.array([imu[i]["low_acc_z"] for i in range(len(imu)) if pad_mask[i]])
        a_mag = np.sqrt(ax**2 + ay**2 + az**2)
        print(f"\n  Accel magnitude: mean={a_mag.mean():.4f} std={a_mag.std():.4f} m/s²")

        # Gyro bias (mean) and white noise (std)
        gx = np.array([imu[i]["gyro_x"] for i in range(len(imu)) if pad_mask[i]])
        gy = np.array([imu[i]["gyro_y"] for i in range(len(imu)) if pad_mask[i]])
        gz = np.array([imu[i]["gyro_z"] for i in range(len(imu)) if pad_mask[i]])

        print(f"\n  Gyro bias (mean on pad):")
        print(f"    X: {gx.mean():.4f} dps = {gx.mean()*math.pi/180:.6f} rad/s")
        print(f"    Y: {gy.mean():.4f} dps = {gy.mean()*math.pi/180:.6f} rad/s")
        print(f"    Z: {gz.mean():.4f} dps = {gz.mean()*math.pi/180:.6f} rad/s")
        print(f"    |bias|: {math.sqrt(gx.mean()**2 + gy.mean()**2 + gz.mean()**2):.4f} dps")

        print(f"\n  Gyro white noise (std on pad):")
        print(f"    X: {gx.std():.4f} dps = {gx.std()*math.pi/180:.6f} rad/s")
        print(f"    Y: {gy.std():.4f} dps = {gy.std()*math.pi/180:.6f} rad/s")
        print(f"    Z: {gz.std():.4f} dps = {gz.std()*math.pi/180:.6f} rad/s")

        # Allan variance at 1s (rough estimate for rate random walk)
        # For white noise: σ_allan(τ) ≈ σ_white / sqrt(τ * rate)
        # So σ_white ≈ σ_allan(τ=1s) * sqrt(rate)
        print(f"\n  Gyro noise spectral density (approx):")
        for axis_name, gdata in [("X", gx), ("Y", gy), ("Z", gz)]:
            # White noise PSD ≈ std / sqrt(rate)  [dps/sqrt(Hz)]
            arw = gdata.std() / math.sqrt(imu_rate)  # dps/sqrt(Hz)
            print(f"    {axis_name}: {arw:.4f} dps/√Hz = {arw*math.pi/180:.6f} rad/s/√Hz")

    # ============================================================
    # GNSS ANALYSIS
    # ============================================================
    print("\n" + "=" * 70)
    print("GNSS - Pad Noise Analysis")
    print("=" * 70)

    gnss = records["GNSS"]
    gnss_times = get_array(gnss, "time_us")
    pad_gnss_mask = (gnss_times >= pad_start_us) & (gnss_times <= pad_end_us)
    n_pad_gnss = pad_gnss_mask.sum()
    print(f"  Pad GNSS samples: {n_pad_gnss}")

    if n_pad_gnss > 5:
        gnss_dt = np.diff(gnss_times[pad_gnss_mask]) / 1e6
        gnss_rate = 1.0 / np.median(gnss_dt)
        print(f"  GNSS rate: {gnss_rate:.1f} Hz (median dt={np.median(gnss_dt)*1e3:.1f} ms)")

        lat = np.array([gnss[i]["lat"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        lon = np.array([gnss[i]["lon"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        alt = np.array([gnss[i]["alt_m"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        h_acc = np.array([gnss[i]["h_acc_m"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        v_acc = np.array([gnss[i]["v_acc_m"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        sats = np.array([gnss[i]["num_sats"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        pdop = np.array([gnss[i]["pdop"] for i in range(len(gnss)) if pad_gnss_mask[i]])

        vel_e = np.array([gnss[i]["vel_e"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        vel_n = np.array([gnss[i]["vel_n"] for i in range(len(gnss)) if pad_gnss_mask[i]])
        vel_u = np.array([gnss[i]["vel_u"] for i in range(len(gnss)) if pad_gnss_mask[i]])

        # Convert lat/lon to local NE meters relative to mean
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        mean_lat = lat.mean()
        R_earth = 6371000.0
        n_m = (lat - mean_lat) * math.pi / 180.0 * R_earth
        e_m = (lon - lon.mean()) * math.pi / 180.0 * R_earth * math.cos(math.radians(mean_lat))
        d_m = -(alt - alt.mean())  # NED down

        print(f"\n  Position noise (pad, relative to mean):")
        print(f"    North: std={n_m.std():.3f} m")
        print(f"    East:  std={e_m.std():.3f} m")
        print(f"    Down:  std={d_m.std():.3f} m")
        print(f"    Horizontal: std={np.sqrt(n_m.std()**2 + e_m.std()**2):.3f} m")

        print(f"\n  Velocity noise (pad, should be ~0):")
        print(f"    East:  mean={vel_e.mean():.3f} std={vel_e.std():.3f} m/s")
        print(f"    North: mean={vel_n.mean():.3f} std={vel_n.std():.3f} m/s")
        print(f"    Up:    mean={vel_u.mean():.3f} std={vel_u.std():.3f} m/s")

        print(f"\n  Reported accuracy:")
        print(f"    h_acc: mean={h_acc.mean():.1f} min={h_acc.min():.0f} max={h_acc.max():.0f} m")
        print(f"    v_acc: mean={v_acc.mean():.1f} min={v_acc.min():.0f} max={v_acc.max():.0f} m")
        print(f"    sats:  mean={sats.mean():.1f} min={sats.min()} max={sats.max()}")
        print(f"    pdop:  mean={pdop.mean():.2f} min={pdop.min():.1f} max={pdop.max():.1f}")

    # Also look at GNSS during flight for dropout behavior
    print("\n  --- GNSS during flight ---")
    flight_gnss_mask = gnss_times >= launch_time_us
    n_flight_gnss = flight_gnss_mask.sum()
    if n_flight_gnss > 0:
        flight_h_acc = np.array([gnss[i]["h_acc_m"] for i in range(len(gnss)) if flight_gnss_mask[i]])
        flight_sats = np.array([gnss[i]["num_sats"] for i in range(len(gnss)) if flight_gnss_mask[i]])
        flight_t = (gnss_times[flight_gnss_mask] - launch_time_us) / 1e6

        print(f"  Flight GNSS fixes: {n_flight_gnss}")
        print(f"  Time range: {flight_t[0]:.2f} to {flight_t[-1]:.2f}s after launch")

        # Look for gaps (dropout)
        if n_flight_gnss > 1:
            flight_dt = np.diff(flight_t)
            gaps = np.where(flight_dt > 0.5)[0]  # gaps > 0.5s
            for gi in gaps:
                print(f"  GAP: {flight_t[gi]:.2f}s -> {flight_t[gi+1]:.2f}s ({flight_dt[gi]:.2f}s)")

        # h_acc after recovery
        print(f"\n  Flight h_acc: mean={flight_h_acc.mean():.1f} max={flight_h_acc.max():.0f} m")
        print(f"  Flight sats:  mean={flight_sats.mean():.1f} min={flight_sats.min()} max={flight_sats.max()}")

        # Print first 10 flight fixes with h_acc
        print(f"\n  First flight GNSS fixes (t, h_acc, sats):")
        flight_indices = np.where(flight_gnss_mask)[0]
        for j in range(min(15, len(flight_indices))):
            i = flight_indices[j]
            t_s = (gnss[i]["time_us"] - launch_time_us) / 1e6
            print(f"    t={t_s:6.2f}s  h_acc={gnss[i]['h_acc_m']:3.0f}m  sats={gnss[i]['num_sats']}  fix={gnss[i]['fix_mode']}")

    # ============================================================
    # BAROMETER ANALYSIS
    # ============================================================
    print("\n" + "=" * 70)
    print("Barometer (BMP585) - Pad Noise Analysis")
    print("=" * 70)

    baro = records["BMP585"]
    baro_times = get_array(baro, "time_us")
    pad_baro_mask = (baro_times >= pad_start_us) & (baro_times <= pad_end_us)
    n_pad_baro = pad_baro_mask.sum()
    print(f"  Pad baro samples: {n_pad_baro}")

    if n_pad_baro > 10:
        baro_dt = np.diff(baro_times[pad_baro_mask]) / 1e6
        baro_rate = 1.0 / np.median(baro_dt)
        print(f"  Baro rate: {baro_rate:.0f} Hz (median dt={np.median(baro_dt)*1e3:.3f} ms)")

        press = np.array([baro[i]["pressure_pa"] for i in range(len(baro)) if pad_baro_mask[i]])
        temp = np.array([baro[i]["temperature"] for i in range(len(baro)) if pad_baro_mask[i]])

        print(f"\n  Pressure: mean={press.mean():.2f} std={press.std():.3f} Pa")
        print(f"  Temperature: mean={temp.mean():.3f} std={temp.std():.4f} C")

        # Convert to altitude noise
        alt_baro = np.array([pressure_to_altitude(p, press.mean()) for p in press])
        print(f"\n  Altitude noise (relative to mean): std={alt_baro.std():.3f} m")
        print(f"    (R_baro in sim = 4.0 m² → σ = 2.0 m)")

    # ============================================================
    # MAGNETOMETER ANALYSIS
    # ============================================================
    print("\n" + "=" * 70)
    print("Magnetometer (MMC5983MA) - Pad Noise Analysis")
    print("=" * 70)

    mag = records["MMC5983MA"]
    mag_times = get_array(mag, "time_us")
    pad_mag_mask = (mag_times >= pad_start_us) & (mag_times <= pad_end_us)
    n_pad_mag = pad_mag_mask.sum()
    print(f"  Pad mag samples: {n_pad_mag}")

    if n_pad_mag > 10:
        mag_dt = np.diff(mag_times[pad_mag_mask]) / 1e6
        mag_rate = 1.0 / np.median(mag_dt)
        print(f"  Mag rate: {mag_rate:.0f} Hz (median dt={np.median(mag_dt)*1e3:.3f} ms)")

        mx = np.array([mag[i]["mag_x"] for i in range(len(mag)) if pad_mag_mask[i]])
        my = np.array([mag[i]["mag_y"] for i in range(len(mag)) if pad_mag_mask[i]])
        mz = np.array([mag[i]["mag_z"] for i in range(len(mag)) if pad_mag_mask[i]])
        m_mag = np.sqrt(mx**2 + my**2 + mz**2)

        print(f"\n  Mag field (board frame, µT):")
        print(f"    X: mean={mx.mean():.2f} std={mx.std():.3f}")
        print(f"    Y: mean={my.mean():.2f} std={my.std():.3f}")
        print(f"    Z: mean={mz.mean():.2f} std={mz.std():.3f}")
        print(f"    |B|: mean={m_mag.mean():.2f} std={m_mag.std():.3f} µT")
        print(f"    (Earth's field at launch site should be ~45-55 µT)")

    # ============================================================
    # COMPARISON WITH SIM DEFAULTS
    # ============================================================
    print("\n" + "=" * 70)
    print("COMPARISON: Real sensor noise vs Sim defaults")
    print("=" * 70)

    if n_pad_imu > 100:
        print("\n  --- IMU ---")
        real_accel_std = math.sqrt(ax.std()**2 + ay.std()**2 + az.std()**2) / math.sqrt(3)
        print(f"  Accel white noise (per-axis avg): {real_accel_std:.4f} m/s²")
        print(f"  Sim aNoiseSigma_mps2:             0.20 m/s²")
        print(f"  Ratio sim/real:                   {0.20/real_accel_std:.1f}x")
        print(f"    (Sim is a process noise, not sensor noise — should be larger)")

        real_gyro_std = math.sqrt(gx.std()**2 + gy.std()**2 + gz.std()**2) / math.sqrt(3)
        print(f"\n  Gyro white noise (per-axis avg): {real_gyro_std:.4f} dps = {real_gyro_std*math.pi/180:.6f} rad/s")
        print(f"  Sim wNoiseSigma_rps:             0.00175 rad/s = {0.00175*180/math.pi:.4f} dps")
        print(f"  Ratio sim/real:                  {0.00175/(real_gyro_std*math.pi/180):.2f}x")

        real_gyro_bias = math.sqrt(gx.mean()**2 + gy.mean()**2 + gz.mean()**2)
        print(f"\n  Gyro bias magnitude:             {real_gyro_bias:.4f} dps = {real_gyro_bias*math.pi/180:.6f} rad/s")
        print(f"  Sim initial P gyro bias (1σ):    1.0 dps = 0.01745 rad/s")

    if n_pad_gnss > 5:
        print("\n  --- GNSS ---")
        print(f"  Real pos noise NE: {np.sqrt(n_m.std()**2 + e_m.std()**2)/math.sqrt(2):.2f} m (per-axis)")
        print(f"  Sim pNoiseSigma_NE_m: 3.0 m")
        print(f"  Real pos noise D:  {d_m.std():.2f} m")
        print(f"  Sim pNoiseSigma_D_m:  6.0 m")
        print(f"  Real vel noise NE: {np.sqrt(vel_n.std()**2 + vel_e.std()**2)/math.sqrt(2):.3f} m/s (per-axis)")
        print(f"  Sim vNoiseSigma_NE_mps: 0.5 m/s")
        print(f"  Real vel noise D:  {vel_u.std():.3f} m/s")
        print(f"  Sim vNoiseSigma_D_mps:  1.0 m/s")
        print(f"  Real GNSS rate:    {gnss_rate:.1f} Hz")
        print(f"  Sim GNSS rate:     25 Hz")

    if n_pad_baro > 10:
        print("\n  --- Barometer ---")
        print(f"  Real baro alt noise: {alt_baro.std():.3f} m")
        print(f"  Sim R_baro = 4.0 → σ = 2.0 m")
        print(f"  Suggested R_baro:    {alt_baro.std()**2:.3f} m²")


if __name__ == "__main__":
    analyze()
