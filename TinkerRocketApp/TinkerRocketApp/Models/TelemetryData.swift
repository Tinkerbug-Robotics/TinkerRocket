//
//  TelemetryData.swift
//  TinkerRocketApp
//
//  Data model matching ESP32 telemetry JSON structure
//

import Foundation

struct TelemetryData: Codable {
    var soc: Float?                   // Battery state of charge %
    var current: Float?               // Battery current mA
    var voltage: Float?               // Battery voltage V
    var latitude: Double?             // GPS latitude degrees
    var longitude: Double?            // GPS longitude degrees
    var gdop: Float?                  // GPS dilution of precision
    var num_sats: Int = 0             // Number of GPS satellites
    var state: String = "UNKNOWN"     // Rocket state
    var camera_recording: Bool = false
    var logging_active: Bool = false
    var active_file: String = ""
    var rx_kbs: Float?                // I2C RX rate kB/s
    var wr_kbs: Float?                // Flash write rate kB/s
    var frames_rx: UInt32 = 0         // Frames received
    var frames_drop: UInt32 = 0       // Frames dropped
    var max_alt_m: Float?             // Maximum altitude meters
    var max_speed_mps: Float?         // Maximum speed m/s
    var pressure_alt: Float?          // Barometric pressure altitude meters
    var altitude_rate: Float?         // Vertical rate m/s
    var gnss_alt: Float?              // GNSS altitude meters (from ECEF, base station only)

    // IMU data (ISM6HG256)
    var low_g_x: Float?               // Low-G accelerometer X m/s²
    var low_g_y: Float?               // Low-G accelerometer Y m/s²
    var low_g_z: Float?               // Low-G accelerometer Z m/s²
    var high_g_x: Float?              // High-G accelerometer X m/s²
    var high_g_y: Float?              // High-G accelerometer Y m/s²
    var high_g_z: Float?              // High-G accelerometer Z m/s²
    var gyro_x: Float?                // Gyroscope X deg/s
    var gyro_y: Float?                // Gyroscope Y deg/s
    var gyro_z: Float?                // Gyroscope Z deg/s

    // Attitude (from FlightComputer onboard estimation)
    var roll_cmd: Float?              // Roll command degrees (PID output)
    var q0: Float?                    // Quaternion w (scalar-first, body-to-NED)
    var q1: Float?                    // Quaternion x
    var q2: Float?                    // Quaternion y
    var q3: Float?                    // Quaternion z

    // Roll/pitch/yaw derived from quaternion (not sent over BLE)
    // Guard: reject malformed quaternions (e.g. all zeros from uninitialized EKF)
    var roll: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        let sinr = 2.0 * (w * x + y * z)
        let cosr = 1.0 - 2.0 * (x * x + y * y)
        return atan2(sinr, cosr) * 180.0 / .pi
    }
    var pitch: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        let sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0 { return copysign(90.0, sinp) }
        return asin(sinp) * 180.0 / .pi
    }
    var yaw: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        let siny = 2.0 * (w * z + x * y)
        let cosy = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny, cosy) * 180.0 / .pi
    }

    // LoRa signal quality (base station only)
    var rssi: Float?                  // LoRa RSSI dBm
    var snr: Float?                   // LoRa SNR dB

    // Base station (base station only)
    var bs_soc: Float?                // Base station SOC %
    var bs_voltage: Float?            // Base station voltage V
    var bs_current: Float?            // Base station current mA
    var bs_logging_active: Bool = false // Base station CSV logging active

    // Flight event flags (optional for backward compat with old firmware)
    var launch_flag: Bool?            // Launch detected
    var vel_apo: Bool?                // Velocity apogee (vertical vel crossed zero)
    var alt_apo: Bool?                // Altitude apogee (alt started decreasing)
    var landed_flag: Bool?            // Landing detected

    // Power rail state
    var pwr_pin_on: Bool = false      // FlightComputer + sensors powered on

    // Source rocket identity (base station relay only, nil for direct BLE)
    var source_rocket_id: Int?        // rocket_id from LoRa header
    var source_unit_name: String?     // rocket unit name from LoRa beacon

    // Telemetry freshness status (#95).  Sent by the BS in periodic-push
    // payloads; absent from RX-path payloads (which are always live) and
    // from direct rocket connections.  iOS treats a missing "ds" as live.
    enum DataStatus: Int, Codable { case live = 0, stale = 1, syncing = 2 }
    var data_status: DataStatus = .live
    var data_age_ms: UInt32 = 0      // only meaningful when .stale

    // Pyro channel status (packed bitfield from "ps" JSON key, decoded in init)
    var pyro_status_bits: Int = 0
    var pyro1_armed: Bool { (pyro_status_bits & 0x01) != 0 }
    var pyro1_cont: Bool  { (pyro_status_bits & 0x02) != 0 }
    var pyro1_fired: Bool { (pyro_status_bits & 0x04) != 0 }
    var pyro2_armed: Bool { (pyro_status_bits & 0x08) != 0 }
    var pyro2_cont: Bool  { (pyro_status_bits & 0x10) != 0 }
    var pyro2_fired: Bool { (pyro_status_bits & 0x20) != 0 }

    // Short JSON keys → Swift property names (saves ~150 bytes in BLE payload)
    enum CodingKeys: String, CodingKey {
        case soc
        case current = "cur"
        case voltage = "vol"
        case latitude = "lat"
        case longitude = "lon"
        case num_sats = "nsat"
        case state = "st"
        case camera_recording = "cam"
        case logging_active = "log"
        case active_file = "af"
        case rx_kbs = "rxk"
        case wr_kbs = "wrk"
        case frames_rx = "frx"
        case frames_drop = "fdr"
        case max_alt_m = "malt"
        case max_speed_mps = "mspd"
        case pressure_alt = "palt"
        case altitude_rate = "arate"
        case gnss_alt = "galt"
        case low_g_x = "lx"
        case low_g_y = "ly"
        case low_g_z = "lz"
        case high_g_x = "hx"
        case high_g_y = "hy"
        case high_g_z = "hz"
        case gyro_x = "gx"
        case gyro_y = "gy"
        case gyro_z = "gz"
        case roll_cmd = "rcmd"
        case q0, q1, q2, q3
        case rssi, snr
        case bs_soc = "bsoc"
        case bs_voltage = "bvol"
        case bs_current = "bcur"
        case bs_logging_active = "bslog"
        case launch_flag = "lnch"
        case vel_apo = "vapo"
        case alt_apo = "aapo"
        case landed_flag = "land"
        case pwr_pin_on = "pwr"
        case pyro_status_bits = "ps"  // packed bitfield: b0=ch1_armed, b1=ch1_cont, b2=ch1_fired, b3=ch2_armed, b4=ch2_cont, b5=ch2_fired
        case source_rocket_id = "rid"
        case source_unit_name = "run"
        case data_status = "ds"        // #95
        case data_age_ms = "age"       // #95
    }

    // Custom decoder: non-optional fields with defaults need decodeIfPresent
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        soc = try c.decodeIfPresent(Float.self, forKey: .soc)
        current = try c.decodeIfPresent(Float.self, forKey: .current)
        voltage = try c.decodeIfPresent(Float.self, forKey: .voltage)
        latitude = try c.decodeIfPresent(Double.self, forKey: .latitude)
        longitude = try c.decodeIfPresent(Double.self, forKey: .longitude)
        num_sats = try c.decodeIfPresent(Int.self, forKey: .num_sats) ?? 0
        state = try c.decodeIfPresent(String.self, forKey: .state) ?? "UNKNOWN"
        camera_recording = try c.decodeIfPresent(Bool.self, forKey: .camera_recording) ?? false
        logging_active = try c.decodeIfPresent(Bool.self, forKey: .logging_active) ?? false
        active_file = try c.decodeIfPresent(String.self, forKey: .active_file) ?? ""
        rx_kbs = try c.decodeIfPresent(Float.self, forKey: .rx_kbs)
        wr_kbs = try c.decodeIfPresent(Float.self, forKey: .wr_kbs)
        frames_rx = try c.decodeIfPresent(UInt32.self, forKey: .frames_rx) ?? 0
        frames_drop = try c.decodeIfPresent(UInt32.self, forKey: .frames_drop) ?? 0
        max_alt_m = try c.decodeIfPresent(Float.self, forKey: .max_alt_m)
        max_speed_mps = try c.decodeIfPresent(Float.self, forKey: .max_speed_mps)
        pressure_alt = try c.decodeIfPresent(Float.self, forKey: .pressure_alt)
        altitude_rate = try c.decodeIfPresent(Float.self, forKey: .altitude_rate)
        gnss_alt = try c.decodeIfPresent(Float.self, forKey: .gnss_alt)
        low_g_x = try c.decodeIfPresent(Float.self, forKey: .low_g_x)
        low_g_y = try c.decodeIfPresent(Float.self, forKey: .low_g_y)
        low_g_z = try c.decodeIfPresent(Float.self, forKey: .low_g_z)
        high_g_x = try c.decodeIfPresent(Float.self, forKey: .high_g_x)
        high_g_y = try c.decodeIfPresent(Float.self, forKey: .high_g_y)
        high_g_z = try c.decodeIfPresent(Float.self, forKey: .high_g_z)
        gyro_x = try c.decodeIfPresent(Float.self, forKey: .gyro_x)
        gyro_y = try c.decodeIfPresent(Float.self, forKey: .gyro_y)
        gyro_z = try c.decodeIfPresent(Float.self, forKey: .gyro_z)
        roll_cmd = try c.decodeIfPresent(Float.self, forKey: .roll_cmd)
        q0 = try c.decodeIfPresent(Float.self, forKey: .q0)
        q1 = try c.decodeIfPresent(Float.self, forKey: .q1)
        q2 = try c.decodeIfPresent(Float.self, forKey: .q2)
        q3 = try c.decodeIfPresent(Float.self, forKey: .q3)
        rssi = try c.decodeIfPresent(Float.self, forKey: .rssi)
        snr = try c.decodeIfPresent(Float.self, forKey: .snr)
        bs_soc = try c.decodeIfPresent(Float.self, forKey: .bs_soc)
        bs_voltage = try c.decodeIfPresent(Float.self, forKey: .bs_voltage)
        bs_current = try c.decodeIfPresent(Float.self, forKey: .bs_current)
        bs_logging_active = try c.decodeIfPresent(Bool.self, forKey: .bs_logging_active) ?? false
        launch_flag = try c.decodeIfPresent(Bool.self, forKey: .launch_flag)
        vel_apo = try c.decodeIfPresent(Bool.self, forKey: .vel_apo)
        alt_apo = try c.decodeIfPresent(Bool.self, forKey: .alt_apo)
        landed_flag = try c.decodeIfPresent(Bool.self, forKey: .landed_flag)
        pwr_pin_on = try c.decodeIfPresent(Bool.self, forKey: .pwr_pin_on) ?? false
        pyro_status_bits = try c.decodeIfPresent(Int.self, forKey: .pyro_status_bits) ?? 0
        source_rocket_id = try c.decodeIfPresent(Int.self, forKey: .source_rocket_id)
        source_unit_name = try c.decodeIfPresent(String.self, forKey: .source_unit_name)
        // #95: missing "ds" → .live (older firmware doesn't emit it)
        let dsRaw = try c.decodeIfPresent(Int.self, forKey: .data_status) ?? 0
        data_status = DataStatus(rawValue: dsRaw) ?? .live
        data_age_ms = try c.decodeIfPresent(UInt32.self, forKey: .data_age_ms) ?? 0
    }

    // Default memberwise init (for creating empty telemetry)
    init() {}

    // Computed properties for display
    var socDisplay: String {
        if let soc = soc {
            return String(format: "%.1f%%", soc)
        }
        return "N/A"
    }

    var voltageDisplay: String {
        if let voltage = voltage {
            return String(format: "%.2f V", voltage)
        }
        return "N/A"
    }

    var currentDisplay: String {
        if let current = current {
            return String(format: "%.0f mA", current)
        }
        return "N/A"
    }

    var coordinatesDisplay: String {
        if let lat = latitude, let lon = longitude {
            return String(format: "%.6f, %.6f", lat, lon)
        }
        return "N/A"
    }

    var maxAltDisplay: String {
        if let max_alt_m = max_alt_m {
            return String(format: "%.1f m", max_alt_m)
        }
        return "N/A"
    }

    var maxSpeedDisplay: String {
        if let max_speed_mps = max_speed_mps {
            return String(format: "%.1f m/s", max_speed_mps)
        }
        return "N/A"
    }

    var pressureAltDisplay: String {
        if let alt = pressure_alt {
            return String(format: "%.1f m", alt)
        }
        return "N/A"
    }

    var altitudeRateDisplay: String {
        if let rate = altitude_rate {
            return String(format: "%.1f m/s", rate)
        }
        return "N/A"
    }

    // IMU display helpers
    var lowGDisplay: String {
        if let x = low_g_x, let y = low_g_y, let z = low_g_z {
            return String(format: "%.2f  %.2f  %.2f", x, y, z)
        }
        return "N/A"
    }

    var highGDisplay: String {
        if let x = high_g_x, let y = high_g_y, let z = high_g_z {
            return String(format: "%.1f  %.1f  %.1f", x, y, z)
        }
        return "N/A"
    }

    var gyroDisplay: String {
        if let x = gyro_x, let y = gyro_y, let z = gyro_z {
            return String(format: "%.1f  %.1f  %.1f", x, y, z)
        }
        return "N/A"
    }

    // Attitude display helpers
    var rollDisplay: String {
        if let r = roll { return String(format: "%.1f\u{00B0}", r) }
        return "N/A"
    }

    var pitchDisplay: String {
        if let p = pitch { return String(format: "%.1f\u{00B0}", p) }
        return "N/A"
    }

    var yawDisplay: String {
        if let y = yaw { return String(format: "%.1f\u{00B0}", y) }
        return "N/A"
    }

    var rollCmdDisplay: String {
        if let r = roll_cmd { return String(format: "%.1f\u{00B0}", r) }
        return "N/A"
    }

    // LoRa signal display helpers
    var rssiDisplay: String {
        if let rssi = rssi {
            return String(format: "%.0f dBm", rssi)
        }
        return "N/A"
    }

    var snrDisplay: String {
        if let snr = snr {
            return String(format: "%.1f dB", snr)
        }
        return "N/A"
    }

    // Base station battery display helpers
    var bsSocDisplay: String {
        if let soc = bs_soc {
            return String(format: "%.1f%%", soc)
        }
        return "N/A"
    }

    var bsVoltageDisplay: String {
        if let voltage = bs_voltage {
            return String(format: "%.2f V", voltage)
        }
        return "N/A"
    }

    var bsCurrentDisplay: String {
        if let current = bs_current {
            return String(format: "%.0f mA", current)
        }
        return "N/A"
    }
}
