//
//  SensorTypes.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Binary sensor data structures matching C++ packed formats
//

import Foundation

// MARK: - Message Types

enum MessageType: UInt8 {
    case statusQuery = 0xA0
    case gnss = 0xA1
    case imu = 0xA2        // ISM6HG256 (Mini, 22B) or ICM45686 (Legacy, 36B)
    case baro = 0xA3       // BMP585 (Mini, 12B) or MS5611 (Legacy, 10B)
    case mag = 0xA4        // MMC5983MA (Mini, 16B) or LIS3MDL (Legacy, 10B)
    case nonSensor = 0xA5  // Computed data (Mini 43–50B by fw era, Legacy 65B)
    case power = 0xA6      // Battery
    case startLogging = 0xA7
    case endFlight = 0xA8
    case h3lis331 = 0xA9   // Legacy-only high-G accelerometer (10B)
    case cameraStart = 0xAA
    case cameraStop = 0xAB
    case flightSettings = 0xE1  // FlightSettingsData (188B v1 / 200B v2 / 208B v3 / 210B v5 / 219B v6 / 220B v7) — runtime settings snapshot at launch (#165)
    case iis2mdc = 0xD1    // IIS2MDC magnetometer (new Mini PCB rev, 10B)
    case lora = 0xF1
}

// MARK: - Rocket State

enum RocketState: UInt8 {
    case initialization = 0
    case ready = 1
    case prelaunch = 2
    case inflight = 3
    case landed = 4
}

// MARK: - Status Query Data (10 bytes) — sensor config from FlightComputer

nonisolated struct OutStatusQueryData {
    let ism6_low_g_fs_g: UInt8       // e.g. 16
    let ism6_high_g_fs_g: UInt16     // e.g. 256
    let ism6_gyro_fs_dps: UInt16     // e.g. 4000
    let ism6_rot_z_cdeg: Int16       // centi-degrees
    let mmc_rot_z_cdeg: Int16        // centi-degrees
    let format_version: UInt8
    let iis2mdc_rot_z_cdeg: Int16?   // centi-degrees; format_version >= 4 (#204)
    let mag_type: UInt8?             // MAG_TYPE_*; format_version >= 6

    /// MAG_TYPE_* wire values (RocketComputerTypes.h): which chip is behind
    /// the IIS2MDC-named (0xD1) count stream.
    static let magTypeIIS2MDC: UInt8 = 0
    static let magTypeQMC5883P: UInt8 = 1
    /// IIS2MDC sensitivity: 1.5 mgauss/LSB = 0.15 µT/LSB (datasheet 9.13).
    static let iis2mdcUtPerLsb: Double = 0.15
    /// QMC5883P at ±8 G, 3750 LSB/gauss (QST Table 2) — the mini's #797 mag.
    static let qmc5883pUtPerLsb: Double = 100.0 / 3750.0

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }

        var offset = 0
        ism6_low_g_fs_g = data.readUInt8(at: &offset)
        ism6_high_g_fs_g = data.readUInt16LE(at: &offset)
        ism6_gyro_fs_dps = data.readUInt16LE(at: &offset)
        ism6_rot_z_cdeg = data.readInt16LE(at: &offset)
        mmc_rot_z_cdeg = data.readInt16LE(at: &offset)
        format_version = data.readUInt8(at: &offset)

        // v4 (#204): per-chip IIS2MDC rotation appended after the b2r block.
        // Byte offset 26 = ism6(1+2+2) + mmc(2) + ver(1) + hg_bias(6) + b2r(2+8).
        if format_version >= 4 && data.count >= 28 {
            var iisOffset = 26
            iis2mdc_rot_z_cdeg = data.readInt16LE(at: &iisOffset)
        } else {
            iis2mdc_rot_z_cdeg = nil
        }

        // v6: mag_type byte at fixed offset 41 (after the v5 guidance-target
        // echo, undecoded here).  Same dual gate shape as the v4 field.
        if format_version >= 6 && data.count >= 42 {
            var magOffset = 41
            mag_type = data.readUInt8(at: &magOffset)
        } else {
            mag_type = nil
        }
    }

    /// IMU rotation in degrees
    var imuRotationDeg: Double { Double(ism6_rot_z_cdeg) / 100.0 }
    /// Magnetometer (MMC5983MA) rotation in degrees
    var magRotationDeg: Double { Double(mmc_rot_z_cdeg) / 100.0 }
    /// IIS2MDC rotation in degrees (format_version >= 4); nil on older logs
    var iisRotationDeg: Double? { iis2mdc_rot_z_cdeg.map { Double($0) / 100.0 } }
    /// Count→µT scale of the IIS2MDC-named mag stream, keyed off mag_type.
    /// Pre-v6 logs (nil) and unknown values fall back to the big board's
    /// IIS2MDC — every pre-v6 log came from one.
    var magUtPerLsb: Double {
        mag_type == Self.magTypeQMC5883P ? Self.qmc5883pUtPerLsb : Self.iis2mdcUtPerLsb
    }
}

// MARK: - Flight Settings Snapshot (176 bytes) — runtime config at launch (#165)

/// One roll-profile waypoint as stored in the flight settings frame.
nonisolated struct RollWaypointRaw {
    let time_s: Float
    let angle_deg: Float
    let mode: UInt8       // 0 = ROLL_SEG_ANGLE, 1 = ROLL_SEG_NULL_RATE
}

/// How the FC determined its board→rocket mounting orientation
/// (mirrors TR_Orientation ORIENT_MODE_*; "imu_orient" config message).
nonisolated enum IMUOrientationMode: Int, Sendable {
    case unknown = -1
    case defaultMounting = 0   // identity, nothing configured
    case manual = 1            // user/config supplied
    case autoSnap = 2          // pad-gravity detect, snapped to an axis
    case autoExact = 3         // pad-gravity detect, exact off-axis rotation

    var label: String {
        switch self {
        case .unknown:         return "—"
        case .defaultMounting: return "default"
        case .manual:          return "manual"
        case .autoSnap:        return "auto"
        case .autoExact:       return "auto, off-axis"
        }
    }
}

/// Decoded FlightSettingsData wire frame. Layout mirrors the packed C++
/// struct in RocketComputerTypes.h byte-for-byte (verified by offset).
nonisolated struct FlightSettingsData {
    // flags bit positions (match FlightSettingsData::F_* in firmware)
    static let fUseAngleControl: UInt8 = 0
    static let fGainSchedule: UInt8    = 1
    static let fGuidance: UInt8        = 2
    static let fServoEnabled: UInt8    = 3
    static let fFwDirty: UInt8         = 4
    static let fSounds: UInt8          = 5
    static let fImuRateDynamic: UInt8  = 7

    let time_us: UInt32
    let version: UInt8
    let flags: UInt8
    let roll_delay_ms: UInt16

    let kp: Float
    let ki: Float
    let kd: Float
    let d_lpf_hz: Float
    let min_cmd_deg: Float
    let max_cmd_deg: Float

    let kp_angle: Float
    let kp_angle_rate_cap_dps: Float

    let gs_v_ref: Float
    let gs_v_min: Float
    let gs_scale_cap: Float

    let roll_rate_set_point: Float

    let ism6_low_g_fs_g: UInt8
    let ism6_high_g_fs_g: UInt16
    let ism6_gyro_fs_dps: UInt16

    let servo_bias_us: [Int16]   // 4 entries
    let servo_hz: Int16
    let servo_min_us: Int16
    let servo_max_us: Int16

    let camera_type: UInt8

    // 4 pyro channels (new PCB). Arrays indexed 0..3 → channels 1..4.
    let pyro_enabled: [Bool]
    let pyro_trigger_mode: [UInt8]   // 0 = time-after-apogee, 1 = altitude-on-descent
    let pyro_trigger_value: [Float]

    let fw_git_sha: String
    let num_waypoints: UInt8
    let waypoints: [RollWaypointRaw]

    // v2 tail: board→rocket mounting orientation, appended at fixed offset
    // 188 after the full v1 layout. nil on v1 frames (pre-orientation
    // firmware, which always assumed the +X-nose mounting).
    // v3 tail: fin-angle calibration (#267), appended at offset 200. nil on
    // pre-v3 frames.
    let fin_min_deg: Float?
    let fin_max_deg: Float?
    /// v5+: IMU logging rate (ISM6HG256 ODR, Hz) that actually flew.
    let ism6_update_rate_hz: UInt16?

    /// v6+ (#435): the horizontal guidance aim point that actually FLEW —
    /// pad-relative ENU meters, re-converted from the frozen GNSS reference
    /// at launch (so it can differ slightly from the receipt-time echo).
    /// `guid_tgt_src` is the GUID_TGT_* code: 0 none/overhead, 1 cmd-28
    /// geodetic (Drift-Cast) point, 2 cmd-65 profile E/N point. nil on
    /// pre-v6 frames.
    let guid_tgt_e_m: Float?
    let guid_tgt_n_m: Float?
    let guid_tgt_src: UInt8?
    /// GNSS high-perf-clock OTP state at boot (v7+, #837 item 6); nil on
    /// pre-v7 logs, where the state was determined and then discarded.
    let gnss_otp_state: UInt8?

    let b2r_code: UInt8?
    let b2r_mode: UInt8?            // 0 default, 1 manual, 2 auto-snap, 3 auto-exact
    let b2r_residual_deg: Float?    // auto-snap residual angle
    let b2r_quat: [Float]?          // board→rocket quaternion, scalar-first

    /// "−Z r90"-style mounting name (mirrors firmware orientCodeName):
    /// which board axis points at the nose + quarter-turn clocking.
    static func b2rName(code: UInt8) -> String {
        let axes = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
        guard code < 24 else { return "?" }
        let base = axes[Int(code) / 4]
        let clock = (Int(code) % 4) * 90
        return clock == 0 ? base : "\(base) r\(clock)"
    }
    var b2rDisplayName: String? { b2r_code.map { FlightSettingsData.b2rName(code: $0) } }

    var useAngleControl: Bool { flags & (1 << FlightSettingsData.fUseAngleControl) != 0 }
    var gainScheduleEnabled: Bool { flags & (1 << FlightSettingsData.fGainSchedule) != 0 }
    var guidanceEnabled: Bool { flags & (1 << FlightSettingsData.fGuidance) != 0 }
    var servoEnabled: Bool { flags & (1 << FlightSettingsData.fServoEnabled) != 0 }
    var fwDirty: Bool { flags & (1 << FlightSettingsData.fFwDirty) != 0 }
    var soundsEnabled: Bool { flags & (1 << FlightSettingsData.fSounds) != 0 }
    /// The flight ran the dynamic logging rate, so `ism6_update_rate_hz` is
    /// the rate at the snapshot (the boost rate) and the log steps down at the
    /// first frame carrying NSF2_DEPLOYED. Flights predating dynamic mode
    /// decode as false and ran one fixed rate throughout.
    var imuRateDynamic: Bool { flags & (1 << FlightSettingsData.fImuRateDynamic) != 0 }

    init(from data: Data) throws {
        guard data.count >= 188 else {
            throw ParseError.invalidSize(expected: 188, got: data.count)
        }

        var offset = 0
        time_us = data.readUInt32LE(at: &offset)
        version = data.readUInt8(at: &offset)
        flags = data.readUInt8(at: &offset)
        roll_delay_ms = data.readUInt16LE(at: &offset)

        kp = data.readFloat32LE(at: &offset)
        ki = data.readFloat32LE(at: &offset)
        kd = data.readFloat32LE(at: &offset)
        d_lpf_hz = data.readFloat32LE(at: &offset)
        min_cmd_deg = data.readFloat32LE(at: &offset)
        max_cmd_deg = data.readFloat32LE(at: &offset)

        kp_angle = data.readFloat32LE(at: &offset)
        kp_angle_rate_cap_dps = data.readFloat32LE(at: &offset)

        gs_v_ref = data.readFloat32LE(at: &offset)
        gs_v_min = data.readFloat32LE(at: &offset)
        gs_scale_cap = data.readFloat32LE(at: &offset)

        roll_rate_set_point = data.readFloat32LE(at: &offset)

        ism6_low_g_fs_g = data.readUInt8(at: &offset)
        ism6_high_g_fs_g = data.readUInt16LE(at: &offset)
        ism6_gyro_fs_dps = data.readUInt16LE(at: &offset)

        // Servo trim + timing (offset 61)
        servo_bias_us = [
            data.readInt16LE(at: &offset),
            data.readInt16LE(at: &offset),
            data.readInt16LE(at: &offset),
            data.readInt16LE(at: &offset),
        ]
        servo_hz = data.readInt16LE(at: &offset)
        servo_min_us = data.readInt16LE(at: &offset)
        servo_max_us = data.readInt16LE(at: &offset)

        camera_type = data.readUInt8(at: &offset)   // offset 75

        // PyroConfigData (offset 76, 24 bytes — 4 channels)
        var en: [Bool]   = []
        var mode: [UInt8] = []
        var val: [Float]  = []
        en.reserveCapacity(4); mode.reserveCapacity(4); val.reserveCapacity(4)
        for _ in 0..<4 {
            en.append(data.readUInt8(at: &offset) != 0)
            mode.append(data.readUInt8(at: &offset))
            val.append(data.readFloat32LE(at: &offset))
        }
        pyro_enabled = en
        pyro_trigger_mode = mode
        pyro_trigger_value = val

        // fw_git_sha: 12-byte NUL-terminated char array (offset 100..111)
        let shaBytes = data.subdata(in: offset..<(offset + 12))
        fw_git_sha = String(bytes: shaBytes.prefix(while: { $0 != 0 }), encoding: .utf8) ?? ""
        offset += 12

        // roll_profile: RollProfileData @ offset 112
        let nRaw = data.readUInt8(at: &offset)      // num_waypoints (offset 112)
        offset += 3                                  // _pad[3]
        let n = min(Int(nRaw), 8)
        num_waypoints = UInt8(n)
        var wps: [RollWaypointRaw] = []
        wps.reserveCapacity(n)
        for _ in 0..<n {                             // waypoints @ offset 116, 9 bytes each
            let t = data.readFloat32LE(at: &offset)
            let a = data.readFloat32LE(at: &offset)
            let m = data.readUInt8(at: &offset)
            wps.append(RollWaypointRaw(time_s: t, angle_deg: a, mode: m))
        }
        waypoints = wps

        // v2 board→rocket orientation tail at fixed offset 188 (after the
        // full 76-byte roll profile, independent of num_waypoints).
        if version >= 2 && data.count >= 200 {
            var o = 188
            b2r_code = data.readUInt8(at: &o)
            b2r_mode = data.readUInt8(at: &o)
            b2r_residual_deg = Float(data.readInt16LE(at: &o)) / 100.0
            b2r_quat = (0..<4).map { _ in Float(data.readInt16LE(at: &o)) / 10000.0 }
        } else {
            b2r_code = nil
            b2r_mode = nil
            b2r_residual_deg = nil
            b2r_quat = nil
        }

        // v3 fin-angle calibration tail at fixed offset 200 (#267).
        if version >= 3 && data.count >= 208 {
            var o = 200
            fin_min_deg = data.readFloat32LE(at: &o)
            fin_max_deg = data.readFloat32LE(at: &o)
        } else {
            fin_min_deg = nil
            fin_max_deg = nil
        }

        // v5 IMU logging rate tail at fixed offset 208.
        if version >= 5 && data.count >= 210 {
            var o = 208
            ism6_update_rate_hz = data.readUInt16LE(at: &o)
        } else {
            ism6_update_rate_hz = nil
        }

        // v6 flown-guidance-target tail at fixed offset 210 (#435).
        if version >= 6 && data.count >= 219 {
            var o = 210
            guid_tgt_e_m = data.readFloat32LE(at: &o)
            guid_tgt_n_m = data.readFloat32LE(at: &o)
            guid_tgt_src = data.readUInt8(at: &o)
        } else {
            guid_tgt_e_m = nil
            guid_tgt_n_m = nil
            guid_tgt_src = nil
        }

        // v7 GNSS OTP state at fixed offset 219 (#837 item 6). Pre-v7 logs
        // decode as nil, which is exactly what they are: the FC determined
        // this at boot and recorded it nowhere.
        if version >= 7 && data.count >= 220 {
            var o = 219
            gnss_otp_state = data.readUInt8(at: &o)
        } else {
            gnss_otp_state = nil
        }
    }
}

// MARK: - Raw Packed Data Structures

// GNSS Data (42 bytes)
nonisolated struct GNSSData {
    let time_us: UInt32

    let year: UInt16
    let month: UInt8
    let day: UInt8

    let hour: UInt8
    let minute: UInt8
    let second: UInt8
    let milli_second: UInt16

    let fix_mode: UInt8     // 0: No Fix, 1: Dead Reckoning, 2: 2D, 3: 3D, 4: GNSS+DR, 5: Time Only
    let num_sats: UInt8
    let pdop_x10: UInt8     // PDOP * 10

    let lat_e7: Int32       // degrees * 1e7
    let lon_e7: Int32       // degrees * 1e7
    let alt_mm: Int32       // mm

    let vel_e_mmps: Int32   // mm/s
    let vel_n_mmps: Int32   // mm/s
    let vel_u_mmps: Int32   // mm/s

    let h_acc_m: UInt8      // horizontal accuracy (m)
    let v_acc_m: UInt8      // vertical accuracy (m)

    init(from data: Data) throws {
        guard data.count >= 42 else {
            throw ParseError.invalidSize(expected: 42, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)

        year = data.readUInt16LE(at: &offset)
        month = data.readUInt8(at: &offset)
        day = data.readUInt8(at: &offset)

        hour = data.readUInt8(at: &offset)
        minute = data.readUInt8(at: &offset)
        second = data.readUInt8(at: &offset)
        milli_second = data.readUInt16LE(at: &offset)

        fix_mode = data.readUInt8(at: &offset)
        num_sats = data.readUInt8(at: &offset)
        pdop_x10 = data.readUInt8(at: &offset)

        lat_e7 = data.readInt32LE(at: &offset)
        lon_e7 = data.readInt32LE(at: &offset)
        alt_mm = data.readInt32LE(at: &offset)

        vel_e_mmps = data.readInt32LE(at: &offset)
        vel_n_mmps = data.readInt32LE(at: &offset)
        vel_u_mmps = data.readInt32LE(at: &offset)

        h_acc_m = data.readUInt8(at: &offset)
        v_acc_m = data.readUInt8(at: &offset)
    }
}

// Power Data (10 bytes)
nonisolated struct POWERData {
    let time_us: UInt32
    let voltage_raw: UInt16  // (V / 10.0) * 65535
    let current_raw: Int16   // (mA / 10000.0) * 32767
    let soc_raw: Int16       // (soc + 25) * (32767/150)

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)
        voltage_raw = data.readUInt16LE(at: &offset)
        current_raw = data.readInt16LE(at: &offset)
        soc_raw = data.readInt16LE(at: &offset)
    }
}

// BMP585 Barometer Data (12 bytes)
nonisolated struct BMP585Data {
    let time_us: UInt32
    let temp_q16: Int32   // degC * 65536
    let press_q6: UInt32  // Pa * 64

    init(from data: Data) throws {
        guard data.count >= 12 else {
            throw ParseError.invalidSize(expected: 12, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)
        temp_q16 = data.readInt32LE(at: &offset)
        press_q6 = data.readUInt32LE(at: &offset)
    }
}

// ISM6HG256 IMU Data (22 bytes)
nonisolated struct ISM6HG256Data {
    let time_us: UInt32

    // Low-G accelerometer (±16g)
    let acc_low_x: Int16
    let acc_low_y: Int16
    let acc_low_z: Int16

    // High-G accelerometer (±256g)
    let acc_high_x: Int16
    let acc_high_y: Int16
    let acc_high_z: Int16

    // Gyroscope (±4000 deg/s)
    let gyro_x: Int16
    let gyro_y: Int16
    let gyro_z: Int16

    init(from data: Data) throws {
        guard data.count >= 22 else {
            throw ParseError.invalidSize(expected: 22, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)

        acc_low_x = data.readInt16LE(at: &offset)
        acc_low_y = data.readInt16LE(at: &offset)
        acc_low_z = data.readInt16LE(at: &offset)

        acc_high_x = data.readInt16LE(at: &offset)
        acc_high_y = data.readInt16LE(at: &offset)
        acc_high_z = data.readInt16LE(at: &offset)

        gyro_x = data.readInt16LE(at: &offset)
        gyro_y = data.readInt16LE(at: &offset)
        gyro_z = data.readInt16LE(at: &offset)
    }
}

// MMC5983MA Magnetometer Data (16 bytes)
nonisolated struct MMC5983MAData {
    let time_us: UInt32
    let mag_x: UInt32  // Raw counts
    let mag_y: UInt32
    let mag_z: UInt32

    init(from data: Data) throws {
        guard data.count >= 16 else {
            throw ParseError.invalidSize(expected: 16, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)
        mag_x = data.readUInt32LE(at: &offset)
        mag_y = data.readUInt32LE(at: &offset)
        mag_z = data.readUInt32LE(at: &offset)
    }
}

// IIS2MDC Magnetometer Data (10 bytes) — new Mini PCB rev, replaces MMC5983MA.
// Raw int16 per axis; scale is per-board (IIS2MDC 0.15 µT/LSB, mini QMC5883P
// 100/3750 µT/LSB #797 — keyed off the status query's v6 mag_type).
nonisolated struct IIS2MDCData {
    let time_us: UInt32
    let mag_x: Int16  // Raw counts (signed)
    let mag_y: Int16
    let mag_z: Int16

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)
        mag_x = data.readInt16LE(at: &offset)
        mag_y = data.readInt16LE(at: &offset)
        mag_z = data.readInt16LE(at: &offset)
    }
}

// Non-Sensor Data (43/44/48/50-byte layouts, oldest → newest)
// Wire layout mirrors C++ NonSensorData in RocketComputerTypes.h — bump the
// size guard + add the matching field here whenever a byte is appended.
nonisolated struct NonSensorData {
    let time_us: UInt32

    // Quaternion (scalar-first, unit quaternion, int16 * 10000)
    let q0: Int16
    let q1: Int16
    let q2: Int16
    let q3: Int16
    let roll_cmd: Int16   // deg * 100

    // Position (cm)
    let e_pos: Int32
    let n_pos: Int32
    let u_pos: Int32

    // Velocity (cm/s)
    let e_vel: Int32
    let n_vel: Int32
    let u_vel: Int32

    // Flags and state
    let flags: UInt8
    let rocket_state: UInt8

    // KF-filtered barometric altitude rate (dm/s = 0.1 m/s)
    let baro_alt_rate_dmps: Int16

    // Pyro channel status bitfield — reclaimed for 4 channels (new PCB):
    //   bit 0 PSF_CH1_CONT
    //   bit 1 PSF_CH1_FIRED
    //   bit 2 PSF_CH2_CONT
    //   bit 3 PSF_CH2_FIRED
    //   bit 4 PSF_CH3_CONT
    //   bit 5 PSF_CH3_FIRED
    //   bit 6 PSF_CH4_CONT
    //   bit 7 PSF_CH4_FIRED
    let pyro_status: UInt8

    // Apogee detector outputs + master vote + relocated reboot/guidance bits:
    //   bit 0 NSF2_GPS_APOGEE
    //   bit 1 NSF2_PITCH_APOGEE
    //   bit 2 NSF2_MASTER_APOGEE  — voted result driving pyro logic
    //   bit 3 NSF2_REBOOT_RECOVERY  — moved from pyro_status bit 4
    //   bit 4 NSF2_GUIDANCE_ENABLED — moved from pyro_status bit 5
    //   bit 7 NSF2_DEPLOYED         — recovery deployment detected (sticky)
    // Older logs (43-byte struct) decode this field as 0.
    let apogee_flags: UInt8

    // #529 (50-byte layout): free-running count of EKF update ticks, uint16
    // wrap.  The EKF replay tool derives the achieved EKF rate from deltas of
    // this counter, so it must survive into the CSV export.  nil on logs that
    // predate the field — 0 is a real value (EKF not yet initialized).
    let ekf_ticks: UInt16?

    init(from data: Data) throws {
        // Accept every layout since 43 bytes (pre-#142/#143 legacy) — 44
        // (+apogee_flags), 48 (+sensor_health), 50 (+ekf_ticks, #529).
        // apogee_flags reads as 0 on legacy logs, which matches the
        // historical "we never recorded these bits" behavior.
        guard data.count >= 43 else {
            throw ParseError.invalidSize(expected: 43, got: data.count)
        }
        let has_apogee_flags = data.count >= 44

        var offset = 0

        time_us = data.readUInt32LE(at: &offset)

        q0 = data.readInt16LE(at: &offset)
        q1 = data.readInt16LE(at: &offset)
        q2 = data.readInt16LE(at: &offset)
        q3 = data.readInt16LE(at: &offset)
        roll_cmd = data.readInt16LE(at: &offset)

        e_pos = data.readInt32LE(at: &offset)
        n_pos = data.readInt32LE(at: &offset)
        u_pos = data.readInt32LE(at: &offset)

        e_vel = data.readInt32LE(at: &offset)
        n_vel = data.readInt32LE(at: &offset)
        u_vel = data.readInt32LE(at: &offset)

        flags = data.readUInt8(at: &offset)
        rocket_state = data.readUInt8(at: &offset)

        baro_alt_rate_dmps = data.readInt16LE(at: &offset)

        pyro_status = data.readUInt8(at: &offset)

        apogee_flags = has_apogee_flags ? data.readUInt8(at: &offset) : 0

        // #529: ekf_ticks sits AFTER sensor_health (#303, 4 bytes, offset 44),
        // which the app has never surfaced — skip over it.
        if data.count >= 50 {
            offset += 4  // sensor_health
            ekf_ticks = data.readUInt16LE(at: &offset)
        } else {
            ekf_ticks = nil
        }
    }
}

// MARK: - Legacy Raw Data Structures
// Legacy uses different sensors with different payload formats.
// Auto-detection: IMU payload size — 36 bytes = Legacy ICM45686, 22 bytes = Mini ISM6HG256.

// Legacy ICM45686 IMU Data (36 bytes)
nonisolated struct LegacyICM45686Data {
    let time_us: UInt32
    let time_sync: UInt32
    let acc_x: Int32    // raw LSB (±32g: 0.00059855 m/s²/LSB)
    let acc_y: Int32
    let acc_z: Int32
    let gyro_x: Int32   // raw LSB (±4000 dps: 0.00762777 deg/s/LSB)
    let gyro_y: Int32
    let gyro_z: Int32
    let temp: Int32

    init(from data: Data) throws {
        guard data.count >= 36 else {
            throw ParseError.invalidSize(expected: 36, got: data.count)
        }
        var offset = 0
        time_us = data.readUInt32LE(at: &offset)
        time_sync = data.readUInt32LE(at: &offset)
        acc_x = data.readInt32LE(at: &offset)
        acc_y = data.readInt32LE(at: &offset)
        acc_z = data.readInt32LE(at: &offset)
        gyro_x = data.readInt32LE(at: &offset)
        gyro_y = data.readInt32LE(at: &offset)
        gyro_z = data.readInt32LE(at: &offset)
        temp = data.readInt32LE(at: &offset)
    }
}

// Legacy H3LIS331 High-G Accelerometer Data (10 bytes)
nonisolated struct LegacyH3LIS331Data {
    let time_us: UInt32
    let acc_x: Int16     // raw LSB (±200g: 0.95788 m/s²/LSB)
    let acc_y: Int16
    let acc_z: Int16

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }
        var offset = 0
        time_us = data.readUInt32LE(at: &offset)
        acc_x = data.readInt16LE(at: &offset)
        acc_y = data.readInt16LE(at: &offset)
        acc_z = data.readInt16LE(at: &offset)
    }
}

// Legacy MS5611 Barometer Data (10 bytes)
nonisolated struct LegacyMS5611Data {
    let time_us: UInt32
    let pressure: UInt32   // encoded: hPa = 10 + code * 1190 / 4294967295
    let temperature: Int16 // encoded: °C = ((raw+32768)/65535)*100 - 40

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }
        var offset = 0
        time_us = data.readUInt32LE(at: &offset)
        pressure = data.readUInt32LE(at: &offset)
        temperature = data.readInt16LE(at: &offset)
    }
}

// Legacy LIS3MDL Magnetometer Data (10 bytes)
nonisolated struct LegacyLIS3MDLData {
    let time_us: UInt32
    let mag_x: Int16     // raw LSB (0.014 μT/LSB)
    let mag_y: Int16
    let mag_z: Int16

    init(from data: Data) throws {
        guard data.count >= 10 else {
            throw ParseError.invalidSize(expected: 10, got: data.count)
        }
        var offset = 0
        time_us = data.readUInt32LE(at: &offset)
        mag_x = data.readInt16LE(at: &offset)
        mag_y = data.readInt16LE(at: &offset)
        mag_z = data.readInt16LE(at: &offset)
    }
}

// Legacy NonSensor Data (65 bytes)
nonisolated struct LegacyNonSensorData {
    let time_us: UInt32

    // Attitude (already in degrees as float, NOT centidegrees)
    let roll: Float
    let pitch: Float
    let yaw: Float
    let roll_cmd: Float

    // Position (cm)
    let e_pos: Int32
    let n_pos: Int32
    let u_pos: Int32

    // Velocity (cm/s)
    let e_vel: Int32
    let n_vel: Int32
    let u_vel: Int32

    // Additional fields (not in Mini)
    let pressure_alt: Int32   // meters
    let altitude_rate: Int32  // dm/s (0.1 m/s)
    let max_alt: Int32        // meters
    let max_speed: Int32      // m/s

    // Flags (separate bools, not a bitfield)
    let alt_landed_flag: Bool
    let alt_apogee_flag: Bool
    let vel_u_apogee_flag: Bool
    let launch_flag: Bool
    let rocket_state: UInt8

    init(from data: Data) throws {
        guard data.count >= 65 else {
            throw ParseError.invalidSize(expected: 65, got: data.count)
        }
        var offset = 0
        time_us = data.readUInt32LE(at: &offset)

        roll = data.readFloat32LE(at: &offset)
        pitch = data.readFloat32LE(at: &offset)
        yaw = data.readFloat32LE(at: &offset)
        roll_cmd = data.readFloat32LE(at: &offset)

        e_pos = data.readInt32LE(at: &offset)
        n_pos = data.readInt32LE(at: &offset)
        u_pos = data.readInt32LE(at: &offset)

        e_vel = data.readInt32LE(at: &offset)
        n_vel = data.readInt32LE(at: &offset)
        u_vel = data.readInt32LE(at: &offset)

        pressure_alt = data.readInt32LE(at: &offset)
        altitude_rate = data.readInt32LE(at: &offset)
        max_alt = data.readInt32LE(at: &offset)
        max_speed = data.readInt32LE(at: &offset)

        alt_landed_flag = data.readUInt8(at: &offset) != 0
        alt_apogee_flag = data.readUInt8(at: &offset) != 0
        vel_u_apogee_flag = data.readUInt8(at: &offset) != 0
        launch_flag = data.readUInt8(at: &offset) != 0
        rocket_state = data.readUInt8(at: &offset)
    }
}

// MARK: - SI Unit Structures

nonisolated struct GNSSDataSI {
    let time_us: UInt32

    let year: UInt16
    let month: UInt8
    let day: UInt8
    let hour: UInt8
    let minute: UInt8
    let second: UInt8
    let milli_second: UInt16

    let fix_mode: UInt8
    let num_sats: UInt8
    let pdop: Double

    let lat: Double        // degrees
    let lon: Double        // degrees
    let alt: Double        // meters

    let vel_e: Double      // m/s
    let vel_n: Double      // m/s
    let vel_u: Double      // m/s

    let h_acc: Double      // meters
    let v_acc: Double      // meters
}

nonisolated struct POWERDataSI {
    let time_us: UInt32
    let voltage: Double    // V
    let current: Double    // mA
    let soc: Double        // %
}

nonisolated struct BMP585DataSI {
    let time_us: UInt32
    let temperature: Double  // °C
    let pressure: Double     // Pa
}

nonisolated struct ISM6HG256DataSI {
    let time_us: UInt32

    let low_g_acc_x: Double   // m/s²
    let low_g_acc_y: Double
    let low_g_acc_z: Double

    let high_g_acc_x: Double  // m/s²
    let high_g_acc_y: Double
    let high_g_acc_z: Double

    let gyro_x: Double        // deg/s
    let gyro_y: Double
    let gyro_z: Double
}

nonisolated struct MMC5983MADataSI {
    let time_us: UInt32
    let mag_x: Double  // μT
    let mag_y: Double
    let mag_z: Double
}

nonisolated struct NonSensorDataSI {
    let time_us: UInt32

    // #514: the raw attitude quaternion, carried through verbatim.
    //
    // roll/pitch/yaw below are NOT a reconstructable Euler triple: `roll` is the
    // body-Z azimuth (it matches the FC roll controller and doesn't gimbal-lock),
    // while `pitch`/`yaw` are ZYX-Euler. Mixing the two conventions silently
    // produces a garbage attitude — so anything that needs the actual orientation
    // must use the quaternion, which is unambiguous.
    let q0: Double        // scalar-first, unit
    let q1: Double
    let q2: Double
    let q3: Double

    let roll: Double      // degrees — BODY-Z AZIMUTH, not the Euler roll (see above)
    let pitch: Double     // degrees — ZYX Euler
    let yaw: Double       // degrees — ZYX Euler
    let roll_cmd: Double

    let e_pos: Double     // meters
    let n_pos: Double
    let u_pos: Double

    let e_vel: Double     // m/s
    let n_vel: Double
    let u_vel: Double

    let altitude_rate: Double  // m/s (KF-filtered baro rate from FlightComputer)

    let alt_landed_flag: Bool
    let alt_apogee_flag: Bool
    let vel_u_apogee_flag: Bool
    let launch_flag: Bool
    // Firmware-set burnout flag (NSF_BURNOUT, bit 4). Source of truth for
    // burnout_time_s in the .json sidecar (#196). Pre-existing logs that
    // predate the firmware setting this bit decode as false.
    let burnout_flag: Bool

    // Per #142/#143: full apogee detector set + master voted result.
    // Decoded from NonSensorData.apogee_flags; legacy 43-byte logs decode all
    // three as false (those flights pre-date the field).
    let gps_apogee_flag: Bool
    let pitch_apogee_flag: Bool
    let apogee_flag: Bool       // master voted result

    let rocket_state: RocketState

    // Derived from NonSensorData.pyro_status (4 channels, new PCB).
    // reboot_recovery and guidance_enabled were relocated to apogee_flags.
    let pyro1_continuity: Bool
    let pyro2_continuity: Bool
    let pyro3_continuity: Bool
    let pyro4_continuity: Bool
    let pyro1_fired: Bool
    let pyro2_fired: Bool
    let pyro3_fired: Bool
    let pyro4_fired: Bool
    let reboot_recovery: Bool
    let guidance_enabled: Bool

    /// Recovery deployment detected (NSF2_DEPLOYED, bit 7 of apogee_flags).
    /// Sticky once the rocket's deployment detector latches, so the first
    /// frame carrying it is the detection time. In dynamic logging mode this
    /// is also where the IMU rate steps down from 3840 to 960 Hz. Logs
    /// predating the detector decode as false.
    let deployed_flag: Bool

    // #529: free-running EKF update-tick counter (uint16 wrap), carried through
    // verbatim for the CSV so the achieved EKF rate is recoverable from an app
    // export.  nil on logs that predate the 50-byte layout.
    let ekf_ticks: UInt16?
}

// MARK: - Parsing Errors

enum ParseError: Error {
    case invalidSize(expected: Int, got: Int)
    case invalidPreamble
    case invalidCRC
    case unknownMessageType(UInt8)
}

// MARK: - Data Extension for Binary Parsing

nonisolated extension Data {
    func readUInt8(at offset: inout Int) -> UInt8 {
        let value = self[offset]
        offset += 1
        return value
    }

    func readInt16LE(at offset: inout Int) -> Int16 {
        // Read bytes manually to avoid alignment issues
        let byte0 = UInt16(self[offset])
        let byte1 = UInt16(self[offset + 1])
        let value = Int16(bitPattern: byte0 | (byte1 << 8))
        offset += 2
        return value
    }

    func readUInt16LE(at offset: inout Int) -> UInt16 {
        // Read bytes manually to avoid alignment issues
        let byte0 = UInt16(self[offset])
        let byte1 = UInt16(self[offset + 1])
        let value = byte0 | (byte1 << 8)
        offset += 2
        return value
    }

    func readInt32LE(at offset: inout Int) -> Int32 {
        // Read bytes manually to avoid alignment issues
        let byte0 = UInt32(self[offset])
        let byte1 = UInt32(self[offset + 1])
        let byte2 = UInt32(self[offset + 2])
        let byte3 = UInt32(self[offset + 3])
        let value = Int32(bitPattern: byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24))
        offset += 4
        return value
    }

    func readUInt32LE(at offset: inout Int) -> UInt32 {
        // Read bytes manually to avoid alignment issues
        let byte0 = UInt32(self[offset])
        let byte1 = UInt32(self[offset + 1])
        let byte2 = UInt32(self[offset + 2])
        let byte3 = UInt32(self[offset + 3])
        let value = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24)
        offset += 4
        return value
    }

    func readFloat32LE(at offset: inout Int) -> Float {
        let bits = readUInt32LE(at: &offset)
        return Float(bitPattern: bits)
    }
}
