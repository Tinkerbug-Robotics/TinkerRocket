//
//  SensorConverter.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Convert raw sensor values to SI units
//

import Foundation

/// nonisolated: SensorConverter is used by CSVGenerator on a background queue.
nonisolated class SensorConverter {

    // ISM6HG256 configuration (match hardware defaults)
    private let lowGFullScale: Double = 16.0    // ±16g
    private let highGFullScale: Double = 256.0  // ±256g
    private let gyroFullScale: Double = 4000.0  // ±4000 deg/s

    private let g_ms2: Double = 9.80665  // Standard gravity in m/s²

    // Sensitivity values (calculated from full scales)
    private let acc_low_ms2_per_lsb: Double
    private let acc_high_ms2_per_lsb: Double
    private let gyro_dps_per_lsb: Double

    // Mini sensor-frame → board-frame rotation angles (configurable per device)
    private var ism6_rot_z_rad: Double = 0.0
    private var mmc_rot_z_rad: Double = 0.0

    init() {
        // Calculate sensitivity values
        // ISM6HG256 outputs 16-bit two's-complement samples:
        //   value = raw * (full_scale / 32768)
        let denom: Double = 32768.0

        let low_mg_per_lsb = (lowGFullScale * 1000.0) / denom
        let high_mg_per_lsb = (highGFullScale * 1000.0) / denom
        let gyro_mdps_per_lsb = (gyroFullScale * 1000.0) / denom

        // mg/LSB -> m/s² per LSB
        acc_low_ms2_per_lsb = (low_mg_per_lsb * 1e-3) * g_ms2
        acc_high_ms2_per_lsb = (high_mg_per_lsb * 1e-3) * g_ms2

        // mdps/LSB -> dps per LSB
        gyro_dps_per_lsb = gyro_mdps_per_lsb * 1e-3
    }

    // MARK: - Rotation Configuration

    /// Configure Mini sensor rotation angles from status query data.
    /// Called when a 0xA0 (statusQuery) frame is found in the binary log.
    func configureMiniRotation(imuDeg: Double, magDeg: Double) {
        ism6_rot_z_rad = imuDeg * .pi / 180.0
        mmc_rot_z_rad = magDeg * .pi / 180.0
    }

    /// Configure Legacy sensor rotation.
    /// Legacy firmware logs raw (unrotated) sensor data, so the app
    /// applies 180° rotation to convert from sensor frame to rocket body frame.
    func configureLegacyRotation(imuDeg: Double = 180.0, magDeg: Double = 180.0) {
        let imuRad = imuDeg * .pi / 180.0
        legacy_rot_c = cos(imuRad)
        legacy_rot_s = sin(imuRad)
        // Mag rotation for Legacy uses the same legacy_rot_c/s (same angle for all sensors)
    }

    // MARK: - GNSS Conversion

    func convertGNSS(_ raw: GNSSData) -> GNSSDataSI {
        return GNSSDataSI(
            time_us: raw.time_us,
            year: raw.year,
            month: raw.month,
            day: raw.day,
            hour: raw.hour,
            minute: raw.minute,
            second: raw.second,
            milli_second: raw.milli_second,
            fix_mode: raw.fix_mode,
            num_sats: raw.num_sats,
            pdop: Double(raw.pdop_x10) / 10.0,
            lat: Double(raw.lat_e7) * 1e-7,       // degrees
            lon: Double(raw.lon_e7) * 1e-7,       // degrees
            alt: Double(raw.alt_mm) * 1e-3,       // meters
            vel_e: Double(raw.vel_e_mmps) * 1e-3, // m/s
            vel_n: Double(raw.vel_n_mmps) * 1e-3, // m/s
            vel_u: Double(raw.vel_u_mmps) * 1e-3, // m/s
            h_acc: Double(raw.h_acc_m),           // meters
            v_acc: Double(raw.v_acc_m)            // meters
        )
    }

    // MARK: - Power Conversion

    func convertPOWER(_ raw: POWERData) -> POWERDataSI {
        let voltage = (Double(raw.voltage_raw) / 65535.0) * 10.0
        let current = (Double(raw.current_raw) / 32767.0) * 10000.0
        let soc = (Double(raw.soc_raw) * 150.0 / 32767.0) - 25.0

        return POWERDataSI(
            time_us: raw.time_us,
            voltage: voltage,  // V
            current: current,  // mA
            soc: soc           // %
        )
    }

    // MARK: - BMP585 Conversion

    func convertBMP585(_ raw: BMP585Data) -> BMP585DataSI {
        let temperature = Double(raw.temp_q16) / 65536.0  // °C
        let pressure = Double(raw.press_q6) / 64.0        // Pa

        return BMP585DataSI(
            time_us: raw.time_us,
            temperature: temperature,
            pressure: pressure
        )
    }

    // MARK: - ISM6HG256 Conversion

    func convertISM6HG256(_ raw: ISM6HG256Data) -> ISM6HG256DataSI {
        let c = cos(ism6_rot_z_rad)
        let s = sin(ism6_rot_z_rad)

        // Low-g accel (m/s²), then rotate about +Z to board frame
        let lg_x = Double(raw.acc_low_x) * acc_low_ms2_per_lsb
        let lg_y = Double(raw.acc_low_y) * acc_low_ms2_per_lsb
        let low_g_acc_x = (lg_x * c) - (lg_y * s)
        let low_g_acc_y = (lg_x * s) + (lg_y * c)
        let low_g_acc_z = Double(raw.acc_low_z) * acc_low_ms2_per_lsb

        // High-g accel (m/s²), then rotate about +Z to board frame
        let hg_x = Double(raw.acc_high_x) * acc_high_ms2_per_lsb
        let hg_y = Double(raw.acc_high_y) * acc_high_ms2_per_lsb
        let high_g_acc_x = (hg_x * c) - (hg_y * s)
        let high_g_acc_y = (hg_x * s) + (hg_y * c)
        let high_g_acc_z = Double(raw.acc_high_z) * acc_high_ms2_per_lsb

        // Gyro (deg/s), then rotate about +Z to board frame
        let g_x = Double(raw.gyro_x) * gyro_dps_per_lsb
        let g_y = Double(raw.gyro_y) * gyro_dps_per_lsb
        let gyro_x = (g_x * c) - (g_y * s)
        let gyro_y = (g_x * s) + (g_y * c)
        let gyro_z = Double(raw.gyro_z) * gyro_dps_per_lsb

        return ISM6HG256DataSI(
            time_us: raw.time_us,
            low_g_acc_x: low_g_acc_x,
            low_g_acc_y: low_g_acc_y,
            low_g_acc_z: low_g_acc_z,
            high_g_acc_x: high_g_acc_x,
            high_g_acc_y: high_g_acc_y,
            high_g_acc_z: high_g_acc_z,
            gyro_x: gyro_x,
            gyro_y: gyro_y,
            gyro_z: gyro_z
        )
    }

    // MARK: - MMC5983MA Conversion

    func convertMMC5983MA(_ raw: MMC5983MAData) -> MMC5983MADataSI {
        // Center the raw counts (18-bit value centered at 2^17 = 131072)
        let cx = Int32(raw.mag_x & 0x3FFFF) - 131072
        let cy = Int32(raw.mag_y & 0x3FFFF) - 131072
        let cz = Int32(raw.mag_z & 0x3FFFF) - 131072

        // Gauss = centered * (8 / 131072)
        // SI: μT = Gauss * 100
        let UT_PER_COUNT = (8.0 * 100.0) / 131072.0  // 0.006103515625

        let mx = Double(cx) * UT_PER_COUNT
        let my = Double(cy) * UT_PER_COUNT
        let mz = Double(cz) * UT_PER_COUNT

        let c = cos(mmc_rot_z_rad)
        let s = sin(mmc_rot_z_rad)

        // Rotate sensor frame -> board frame about +Z axis
        let mag_x = (mx * c) - (my * s)
        let mag_y = (mx * s) + (my * c)
        let mag_z = mz

        return MMC5983MADataSI(
            time_us: raw.time_us,
            mag_x: mag_x,  // μT
            mag_y: mag_y,
            mag_z: mag_z
        )
    }

    // MARK: - Legacy Sensor Conversions
    // Legacy uses different sensors (ICM45686, H3LIS331, MS5611, LIS3MDL).
    // All converters output the same SI types as Mini so CSVGenerator stays unified.

    // ICM45686 ±32g accel, ±4000 dps gyro (low-G only — high-G from separate H3LIS331)
    private let icm45686_acc_ms2_per_lsb: Double = 0.00059855
    private let icm45686_gyro_dps_per_lsb: Double = 0.00762777

    // H3LIS331 ±200g high-G accelerometer
    private let h3lis331_acc_ms2_per_lsb: Double = 0.95788

    // LIS3MDL magnetometer
    private let lis3mdl_ut_per_lsb: Double = 0.014

    // Legacy rotation: 180° by default (raw data logged, rotate in app)
    private var legacy_rot_c: Double = -1.0  // cos(180°)
    private var legacy_rot_s: Double =  0.0  // sin(180°)

    /// Convert Legacy ICM45686 → ISM6HG256DataSI (high-G fields = 0; they come from H3LIS331)
    /// Legacy data is raw (unrotated). Apply 180° rotation to body frame.
    private var legacyDebugCount = 0
    func convertLegacyICM45686(_ raw: LegacyICM45686Data) -> ISM6HG256DataSI {
        // Low-G accel (m/s²) — scale then rotate to body frame
        let ax = Double(raw.acc_x) * icm45686_acc_ms2_per_lsb
        let ay = Double(raw.acc_y) * icm45686_acc_ms2_per_lsb
        let low_g_x = (ax * legacy_rot_c) - (ay * legacy_rot_s)
        let low_g_y = (ax * legacy_rot_s) + (ay * legacy_rot_c)
        let low_g_z = Double(raw.acc_z) * icm45686_acc_ms2_per_lsb

        // Debug: print first 5 samples to trace the transform
        if legacyDebugCount < 5 {
            print("[LegacyICM] raw_acc_x=\(raw.acc_x) rot_c=\(legacy_rot_c) rot_s=\(legacy_rot_s) → low_g_x=\(String(format: "%.4f", low_g_x))")
            legacyDebugCount += 1
        }

        // Gyro (deg/s) — scale then rotate to body frame
        let gx = Double(raw.gyro_x) * icm45686_gyro_dps_per_lsb
        let gy = Double(raw.gyro_y) * icm45686_gyro_dps_per_lsb
        let gyro_x = (gx * legacy_rot_c) - (gy * legacy_rot_s)
        let gyro_y = (gx * legacy_rot_s) + (gy * legacy_rot_c)
        let gyro_z = Double(raw.gyro_z) * icm45686_gyro_dps_per_lsb

        return ISM6HG256DataSI(
            time_us: raw.time_us,
            low_g_acc_x: low_g_x,
            low_g_acc_y: low_g_y,
            low_g_acc_z: low_g_z,
            high_g_acc_x: 0.0,  // Comes from separate H3LIS331 sensor
            high_g_acc_y: 0.0,
            high_g_acc_z: 0.0,
            gyro_x: gyro_x,
            gyro_y: gyro_y,
            gyro_z: gyro_z
        )
    }

    /// Convert Legacy H3LIS331 → high-G acceleration tuple (m/s²)
    struct HighGDataSI {
        let time_us: UInt32
        let acc_x: Double  // m/s²
        let acc_y: Double
        let acc_z: Double
    }

    func convertLegacyH3LIS331(_ raw: LegacyH3LIS331Data) -> HighGDataSI {
        // High-G accel (m/s²) — scale then rotate to body frame
        let ax = Double(raw.acc_x) * h3lis331_acc_ms2_per_lsb
        let ay = Double(raw.acc_y) * h3lis331_acc_ms2_per_lsb
        let hg_x = (ax * legacy_rot_c) - (ay * legacy_rot_s)
        let hg_y = (ax * legacy_rot_s) + (ay * legacy_rot_c)
        let hg_z = Double(raw.acc_z) * h3lis331_acc_ms2_per_lsb

        return HighGDataSI(
            time_us: raw.time_us,
            acc_x: hg_x,
            acc_y: hg_y,
            acc_z: hg_z
        )
    }

    /// Convert Legacy MS5611 → BMP585DataSI
    func convertLegacyMS5611(_ raw: LegacyMS5611Data) -> BMP585DataSI {
        // Pressure: hPa = 10 + code * 1190 / 4294967295, then convert to Pa
        let pressure_hpa = 10.0 + (Double(raw.pressure) * 1190.0 / 4294967295.0)
        let pressure_pa = pressure_hpa * 100.0

        // Temperature: °C = ((raw + 32768) / 65535) * 100 - 40
        let temperature = ((Double(raw.temperature) + 32768.0) / 65535.0) * 100.0 - 40.0

        return BMP585DataSI(
            time_us: raw.time_us,
            temperature: temperature,
            pressure: pressure_pa
        )
    }

    /// Convert Legacy LIS3MDL → MMC5983MADataSI
    /// Legacy data is raw (unrotated). Apply 180° rotation to body frame.
    func convertLegacyLIS3MDL(_ raw: LegacyLIS3MDLData) -> MMC5983MADataSI {
        // Raw LSB to μT — scale then rotate to body frame
        let mx = Double(raw.mag_x) * lis3mdl_ut_per_lsb
        let my = Double(raw.mag_y) * lis3mdl_ut_per_lsb
        let mag_x = (mx * legacy_rot_c) - (my * legacy_rot_s)
        let mag_y = (mx * legacy_rot_s) + (my * legacy_rot_c)
        let mag_z = Double(raw.mag_z) * lis3mdl_ut_per_lsb

        return MMC5983MADataSI(
            time_us: raw.time_us,
            mag_x: mag_x,
            mag_y: mag_y,
            mag_z: mag_z
        )
    }

    /// Convert Legacy NonSensorData → NonSensorDataSI
    func convertLegacyNonSensor(_ raw: LegacyNonSensorData) -> NonSensorDataSI {
        // Roll/pitch/yaw are already in degrees (float), not centidegrees
        let roll = Double(raw.roll)
        let pitch = Double(raw.pitch)
        let yaw = Double(raw.yaw)
        let roll_cmd = Double(raw.roll_cmd)

        // cm → m
        let e_pos = Double(raw.e_pos) * 0.01
        let n_pos = Double(raw.n_pos) * 0.01
        let u_pos = Double(raw.u_pos) * 0.01

        // cm/s → m/s
        let e_vel = Double(raw.e_vel) * 0.01
        let n_vel = Double(raw.n_vel) * 0.01
        let u_vel = Double(raw.u_vel) * 0.01

        // altitude_rate is in dm/s (decimeters per second) → m/s
        let altitude_rate = Double(raw.altitude_rate) * 0.1

        let rocket_state = RocketState(rawValue: raw.rocket_state) ?? .initialization

        return NonSensorDataSI(
            time_us: raw.time_us,
            roll: roll,
            pitch: pitch,
            yaw: yaw,
            roll_cmd: roll_cmd,
            e_pos: e_pos,
            n_pos: n_pos,
            u_pos: u_pos,
            e_vel: e_vel,
            n_vel: n_vel,
            u_vel: u_vel,
            altitude_rate: altitude_rate,
            alt_landed_flag: raw.alt_landed_flag,
            alt_apogee_flag: raw.alt_apogee_flag,
            vel_u_apogee_flag: raw.vel_u_apogee_flag,
            launch_flag: raw.launch_flag,
            rocket_state: rocket_state,
            // Legacy wire format never carried pyro_status — leave unset.
            pyro1_continuity: false,
            pyro2_continuity: false,
            pyro1_fired: false,
            pyro2_fired: false,
            reboot_recovery: false,
            guidance_enabled: false
        )
    }

    // MARK: - Mini NonSensor Conversion

    func convertNonSensor(_ raw: NonSensorData) -> NonSensorDataSI {
        // Quaternion int16*10000 -> float, then derive Euler (ZYX convention)
        let q0 = Double(raw.q0) / 10000.0
        let q1 = Double(raw.q1) / 10000.0
        let q2 = Double(raw.q2) / 10000.0
        let q3 = Double(raw.q3) / 10000.0
        let roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0 / .pi
        let pitch = asin(max(-1, min(1, 2*(q0*q2 - q3*q1)))) * 180.0 / .pi
        let yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0 / .pi
        let roll_cmd = Double(raw.roll_cmd) / 100.0

        // Centimeters -> meters
        let e_pos = Double(raw.e_pos) * 0.01
        let n_pos = Double(raw.n_pos) * 0.01
        let u_pos = Double(raw.u_pos) * 0.01

        // cm/s -> m/s
        let e_vel = Double(raw.e_vel) * 0.01
        let n_vel = Double(raw.n_vel) * 0.01
        let u_vel = Double(raw.u_vel) * 0.01

        // Extract flags (bitfield)
        let NSF_ALT_LANDED: UInt8   = (1 << 0)
        let NSF_ALT_APOGEE: UInt8   = (1 << 1)
        let NSF_VEL_APOGEE: UInt8   = (1 << 2)
        let NSF_LAUNCH: UInt8       = (1 << 3)

        let alt_landed_flag = (raw.flags & NSF_ALT_LANDED) != 0
        let alt_apogee_flag = (raw.flags & NSF_ALT_APOGEE) != 0
        let vel_u_apogee_flag = (raw.flags & NSF_VEL_APOGEE) != 0
        let launch_flag = (raw.flags & NSF_LAUNCH) != 0

        let rocket_state = RocketState(rawValue: raw.rocket_state) ?? .initialization

        // dm/s -> m/s
        let altitude_rate = Double(raw.baro_alt_rate_dmps) * 0.1

        // Pyro status byte — mirror the C++ PSF_ masks in RocketComputerTypes.h
        let PSF_CH1_CONT: UInt8         = (1 << 0)
        let PSF_CH2_CONT: UInt8         = (1 << 1)
        let PSF_CH1_FIRED: UInt8        = (1 << 2)
        let PSF_CH2_FIRED: UInt8        = (1 << 3)
        let PSF_REBOOT_RECOVERY: UInt8  = (1 << 4)
        let PSF_GUIDANCE_ENABLED: UInt8 = (1 << 5)

        return NonSensorDataSI(
            time_us: raw.time_us,
            roll: roll,
            pitch: pitch,
            yaw: yaw,
            roll_cmd: roll_cmd,
            e_pos: e_pos,
            n_pos: n_pos,
            u_pos: u_pos,
            e_vel: e_vel,
            n_vel: n_vel,
            u_vel: u_vel,
            altitude_rate: altitude_rate,
            alt_landed_flag: alt_landed_flag,
            alt_apogee_flag: alt_apogee_flag,
            vel_u_apogee_flag: vel_u_apogee_flag,
            launch_flag: launch_flag,
            rocket_state: rocket_state,
            pyro1_continuity: (raw.pyro_status & PSF_CH1_CONT) != 0,
            pyro2_continuity: (raw.pyro_status & PSF_CH2_CONT) != 0,
            pyro1_fired:      (raw.pyro_status & PSF_CH1_FIRED) != 0,
            pyro2_fired:      (raw.pyro_status & PSF_CH2_FIRED) != 0,
            reboot_recovery:  (raw.pyro_status & PSF_REBOOT_RECOVERY) != 0,
            guidance_enabled: (raw.pyro_status & PSF_GUIDANCE_ENABLED) != 0
        )
    }
}
