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
        // Body-Z roll, matching the FC roll controller (flight_computer/main.cpp:
        // actual_roll_deg = -atan2(z_east, z_north)). Stays well-defined when the
        // rocket is vertical, unlike the ZYX-Euler roll which gimbal-locks and
        // cycles near pitch ±90°. Keep in sync with SensorConverter.convertNonSensor.
        let zNorth = 2.0 * (x * z + w * y)
        let zEast  = 2.0 * (y * z - w * x)
        return -atan2(zEast, zNorth) * 180.0 / .pi
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
    // Seconds remaining until the BS silence-timeout closes the active log.
    // Sent only when bs_logging_active is true (BS omits the JSON key
    // otherwise) — nil here = no countdown to show.
    var bs_log_silence_remaining_s: UInt16?

    // Packed flight-status bits ("fs" JSON key, decoded in init).  Replaces
    // 8 separate boolean keys to keep the BLE notify payload under MTU —
    // see TR_BLE_To_APP.cpp:buildTelemetryJSON.  Bit layout must stay in
    // sync with the firmware side.
    var flight_status_bits: Int = 0
    var launch_flag: Bool       { (flight_status_bits & 0x01) != 0 }
    var vel_apo: Bool           { (flight_status_bits & 0x02) != 0 }
    var alt_apo: Bool           { (flight_status_bits & 0x04) != 0 }
    var landed_flag: Bool       { (flight_status_bits & 0x08) != 0 }
    var pwr_pin_on: Bool        { (flight_status_bits & 0x10) != 0 }
    var camera_recording: Bool  { (flight_status_bits & 0x20) != 0 }
    var logging_active: Bool    { (flight_status_bits & 0x40) != 0 }
    var bs_logging_active: Bool { (flight_status_bits & 0x80) != 0 }

    // Source rocket identity (base station relay only, nil for direct BLE)
    var source_rocket_id: Int?        // rocket_id from LoRa header
    var source_unit_name: String?     // rocket unit name from LoRa beacon

    // Telemetry freshness status (#95).  Sent by the BS in periodic-push
    // payloads; absent from RX-path payloads (which are always live) and
    // from direct rocket connections.  iOS treats a missing "ds" as live.
    enum DataStatus: Int, Codable { case live = 0, stale = 1, syncing = 2 }
    var data_status: DataStatus = .live
    var data_age_ms: UInt32 = 0      // only meaningful when .stale

    // #282: base station sets "tr":1 when a worst-case in-flight frame was
    // trimmed to fit the BLE MTU window — low-priority tail fields (link
    // stats, unit name, active filename, some IMU detail) were dropped so the
    // recovery/dashboard-critical fields still got through.  The frame is
    // still valid; this just flags that it's partial so the absence of those
    // fields reads as "trimmed for bandwidth", not "sensor offline".
    var fields_trimmed: Bool = false

    // Pyro channel status (packed bitfield from "ps" JSON key, decoded in init).
    // New PCB: bit 0 = global armed (single shared ARM FET), then per-channel
    // (cont, fired) pairs for channels 1..4. 9 bits total.
    var pyro_status_bits: Int = 0
    var pyro_armed: Bool { (pyro_status_bits & 0x001) != 0 }
    var pyro1_cont:  Bool { (pyro_status_bits & 0x002) != 0 }
    var pyro1_fired: Bool { (pyro_status_bits & 0x004) != 0 }
    var pyro2_cont:  Bool { (pyro_status_bits & 0x008) != 0 }
    var pyro2_fired: Bool { (pyro_status_bits & 0x010) != 0 }
    var pyro3_cont:  Bool { (pyro_status_bits & 0x020) != 0 }
    var pyro3_fired: Bool { (pyro_status_bits & 0x040) != 0 }
    var pyro4_cont:  Bool { (pyro_status_bits & 0x080) != 0 }
    var pyro4_fired: Bool { (pyro_status_bits & 0x100) != 0 }
    func pyroCont(channel: Int) -> Bool {
        switch channel { case 1: return pyro1_cont; case 2: return pyro2_cont
                          case 3: return pyro3_cont; case 4: return pyro4_cont
                          default: return false }
    }
    func pyroFired(channel: Int) -> Bool {
        switch channel { case 1: return pyro1_fired; case 2: return pyro2_fired
                          case 3: return pyro3_fired; case 4: return pyro4_fired
                          default: return false }
    }

    // ── Sensor health scorecard (#303) ────────────────────────────────────
    // Packed "h" bitfield: FC sets sensors + EKF, OC sets battery.  2 bits per
    // item; layout mirrors RocketComputerTypes.h (SH_*_SHIFT).  0 = nothing
    // reported (older firmware or a BS self-frame) → card hidden.
    var sensor_health: Int = 0
    enum SensorHealth: Int { case na = 0, ok = 1, degraded = 2, bad = 3
        var label: String {
            switch self { case .na: return "N/A"; case .ok: return "OK"
                          case .degraded: return "DEGRADED"; case .bad: return "BAD" }
        }
    }
    private func shState(_ shift: Int) -> SensorHealth {
        SensorHealth(rawValue: (sensor_health >> shift) & 0x3) ?? .na
    }
    var baroHealth: SensorHealth { shState(0) }
    var imuHealth:  SensorHealth { shState(2) }
    var ekfHealth:  SensorHealth { shState(4) }   // filter health: init + isHealthy + covariance converged
    var magHealth:  SensorHealth { shState(6) }   // advisory only — never gates go/no-go
    var gnssHealth: SensorHealth { shState(8) }
    var battHealth: SensorHealth { shState(10) }
    var storageHealth: SensorHealth { shState(20) }   // #281/#278: flight-log NAND — BAD = full/failing, won't record
    func pyroHealth(channel: Int) -> SensorHealth {   // channel 1...4; .na = not configured
        guard (1...4).contains(channel) else { return .na }
        return shState(12 + (channel - 1) * 2)
    }
    var hasSensorHealth: Bool { sensor_health != 0 }

    // Rows for the pre-launch health card.  Core sensors always shown; a pyro
    // channel appears only when configured for the flight (state != .na).
    struct SensorHealthRow: Identifiable { let name: String; let state: SensorHealth; var id: String { name } }
    var sensorHealthRows: [SensorHealthRow] {
        var rows = [
            SensorHealthRow(name: "Baro",    state: baroHealth),
            SensorHealthRow(name: "IMU",     state: imuHealth),
            SensorHealthRow(name: "EKF",     state: ekfHealth),
            SensorHealthRow(name: "GNSS",    state: gnssHealth),
            SensorHealthRow(name: "Battery", state: battHealth),
            SensorHealthRow(name: "Mag",     state: magHealth),
        ]
        if storageHealth != .na {   // OC-reported only; absent on older firmware / BS self-frames
            rows.append(SensorHealthRow(name: "Storage", state: storageHealth))
        }
        for ch in 1...4 where pyroHealth(channel: ch) != .na {
            rows.append(SensorHealthRow(name: "Pyro \(ch)", state: pyroHealth(channel: ch)))
        }
        return rows
    }

    // Go/no-go rollup (#303).  Red = a hard fault waiting won't fix (baro/IMU/
    // battery BAD, a full/failing flight-log store, or a configured pyro with no
    // continuity).  Green needs every required item OK — including EKF
    // initialized/converged and a GNSS fix.  Mag is advisory and never gates.
    // Storage is OC-reported: absent (N/A) on older firmware → ignored, not red.
    // Amber = anything in between.
    enum FlightReadiness { case unknown, ready, caution, notReady
        var label: String {
            switch self { case .unknown:  return "Waiting for telemetry…"
                          case .ready:    return "Ready to fly"
                          case .caution:  return "Not ready — check sensors"
                          case .notReady: return "Do not fly" }
        }
    }
    var flightReadiness: FlightReadiness {
        guard hasSensorHealth else { return .unknown }
        let hardFault = [baroHealth, imuHealth, battHealth, storageHealth].contains(.bad)
            || (1...4).contains { pyroHealth(channel: $0) == .bad }
        if hardFault { return .notReady }
        var mustBeOK = [baroHealth, imuHealth, battHealth, ekfHealth, gnssHealth]
        if storageHealth != .na { mustBeOK.append(storageHealth) }   // #281/#278: low/full space gates green
        for ch in 1...4 where pyroHealth(channel: ch) != .na { mustBeOK.append(pyroHealth(channel: ch)) }
        return mustBeOK.allSatisfy { $0 == .ok } ? .ready : .caution
    }

    // Short JSON keys → Swift property names (saves ~150 bytes in BLE payload)
    enum CodingKeys: String, CodingKey {
        case soc
        case current = "cur"
        case voltage = "vol"
        case latitude = "lat"
        case longitude = "lon"
        case num_sats = "nsat"
        case state = "st"
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
        case bs_log_silence_remaining_s = "slrm"
        // Packed flight-status bitfield (replaces lnch/vapo/aapo/land/pwr/
        // cam/log/bslog).  Bit layout mirrors TR_BLE_To_APP.cpp.
        case flight_status_bits = "fs"
        case pyro_status_bits = "ps"  // packed bitfield: b0=armed (global), then (cont,fired) per channel 1..4
        case sensor_health = "h"      // #303 scorecard: 2 bits/sensor, see RocketComputerTypes.h
        case source_rocket_id = "rid"
        case source_unit_name = "run"
        case data_status = "ds"        // #95
        case data_age_ms = "age"       // #95
        case fields_trimmed = "tr"     // #282: frame trimmed to fit MTU
    }

    // Custom decoder: non-optional fields with defaults need decodeIfPresent
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        // #293: tolerate a high-value field emitted as a float or string instead
        // of an int — firmware-contract drift on one field would otherwise throw
        // and the catch discards the WHOLE telemetry frame. Mirrors the defensive
        // config decode. Float fields already accept int JSON, so only the ints
        // below need this.
        func flexInt(_ key: CodingKeys) -> Int? {
            if let i = try? c.decodeIfPresent(Int.self, forKey: key) { return i }
            if let d = try? c.decodeIfPresent(Double.self, forKey: key) { return Int(d) }
            if let s = try? c.decodeIfPresent(String.self, forKey: key) {
                return Int(s) ?? Double(s).map { Int($0) }
            }
            return nil
        }
        soc = try c.decodeIfPresent(Float.self, forKey: .soc)
        current = try c.decodeIfPresent(Float.self, forKey: .current)
        voltage = try c.decodeIfPresent(Float.self, forKey: .voltage)
        latitude = try c.decodeIfPresent(Double.self, forKey: .latitude)
        longitude = try c.decodeIfPresent(Double.self, forKey: .longitude)
        num_sats = try c.decodeIfPresent(Int.self, forKey: .num_sats) ?? 0
        state = try c.decodeIfPresent(String.self, forKey: .state) ?? "UNKNOWN"
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
        bs_log_silence_remaining_s = try c.decodeIfPresent(UInt16.self, forKey: .bs_log_silence_remaining_s)
        flight_status_bits = flexInt(.flight_status_bits) ?? 0   // #293: tolerant
        pyro_status_bits = flexInt(.pyro_status_bits) ?? 0       // #293: tolerant
        sensor_health = try c.decodeIfPresent(Int.self, forKey: .sensor_health) ?? 0
        source_rocket_id = try c.decodeIfPresent(Int.self, forKey: .source_rocket_id)
        source_unit_name = try c.decodeIfPresent(String.self, forKey: .source_unit_name)
        // #95: missing "ds" → .live (older firmware doesn't emit it)
        let dsRaw = try c.decodeIfPresent(Int.self, forKey: .data_status) ?? 0
        data_status = DataStatus(rawValue: dsRaw) ?? .live
        data_age_ms = UInt32(clamping: flexInt(.data_age_ms) ?? 0)   // #293: tolerant
        fields_trimmed = (flexInt(.fields_trimmed) ?? 0) != 0        // #282
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
            return UnitFormatter.altitude(Double(max_alt_m))
        }
        return "N/A"
    }

    var maxSpeedDisplay: String {
        if let max_speed_mps = max_speed_mps {
            return UnitFormatter.speed(Double(max_speed_mps))
        }
        return "N/A"
    }

    var pressureAltDisplay: String {
        if let alt = pressure_alt {
            return UnitFormatter.altitude(Double(alt))
        }
        return "N/A"
    }

    var altitudeRateDisplay: String {
        if let rate = altitude_rate {
            return UnitFormatter.speed(Double(rate))
        }
        return "N/A"
    }

    // IMU display helpers.  Values are converted to the display unit (m/s² or
    // g); the unit label is shown separately by the IMU row.
    var lowGDisplay: String {
        if let x = low_g_x, let y = low_g_y, let z = low_g_z {
            let s = UnitSystem.current
            return String(format: "%.2f  %.2f  %.2f",
                          UnitFormatter.accelerationValue(Double(x), system: s),
                          UnitFormatter.accelerationValue(Double(y), system: s),
                          UnitFormatter.accelerationValue(Double(z), system: s))
        }
        return "N/A"
    }

    var highGDisplay: String {
        if let x = high_g_x, let y = high_g_y, let z = high_g_z {
            let s = UnitSystem.current
            return String(format: "%.1f  %.1f  %.1f",
                          UnitFormatter.accelerationValue(Double(x), system: s),
                          UnitFormatter.accelerationValue(Double(y), system: s),
                          UnitFormatter.accelerationValue(Double(z), system: s))
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

    /// Whether the rocket itself is currently writing its onboard flight
    /// log.  The raw `logging_active` flag comes from the OC's
    /// `logger.isLoggingActive()` which returns true while the end-flight
    /// queue is still draining — so for several seconds post-landing it
    /// stays true, and if the drain wedges (END_FLIGHT lost over I2C, etc.)
    /// it can stick true indefinitely.
    ///
    /// Only suppress the indicator during the post-flight drain window
    /// (state == LANDED).  Trust the flag in every other state — manual
    /// pre-flight bench-test logging from READY/PRELAUNCH is a real
    /// scenario and the button label needs to flip to "Stop Logging"
    /// when it succeeds (#137 originally over-gated this to INFLIGHT only).
    /// A stuck-true that survives the user resetting to READY is a genuine
    /// anomaly; surfacing it is more useful than hiding it.
    var rocketLoggingActive: Bool {
        if state == "LANDED" { return false }
        return logging_active
    }
}

/// Format an elapsed-time interval as H:MM:SS (or M:SS when under an hour).
/// Used in place of bare seconds so multi-minute waits read as durations
/// the operator can scan at a glance — e.g. "4:32" remaining on the BS
/// silence-close, or "1:03:15" since the last received packet during a
/// long stale stretch.  Negative or implausibly large values clamp to 0.
func formatElapsed(seconds: Int) -> String {
    let s = max(0, seconds)
    let h = s / 3600
    let m = (s % 3600) / 60
    let sec = s % 60
    if h > 0 {
        return String(format: "%d:%02d:%02d", h, m, sec)
    }
    return String(format: "%d:%02d", m, sec)
}

/// Convenience: same as the Int version but for milliseconds, rounded to
/// the nearest second.  Avoids divisor noise at the call site.
func formatElapsed(ms: UInt32) -> String {
    return formatElapsed(seconds: Int((ms + 500) / 1000))
}
