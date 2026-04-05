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
    case nonSensor = 0xA5  // Computed data (Mini 38/40B, Legacy 65B)
    case power = 0xA6      // Battery
    case startLogging = 0xA7
    case endFlight = 0xA8
    case h3lis331 = 0xA9   // Legacy-only high-G accelerometer (10B)
    case cameraStart = 0xAA
    case cameraStop = 0xAB
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
    }

    /// IMU rotation in degrees
    var imuRotationDeg: Double { Double(ism6_rot_z_cdeg) / 100.0 }
    /// Magnetometer rotation in degrees
    var magRotationDeg: Double { Double(mmc_rot_z_cdeg) / 100.0 }
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

// Non-Sensor Data (38 bytes)
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

    init(from data: Data) throws {
        guard data.count >= 42 else {
            throw ParseError.invalidSize(expected: 42, got: data.count)
        }

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

    let roll: Double      // degrees
    let pitch: Double
    let yaw: Double
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

    let rocket_state: RocketState
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
