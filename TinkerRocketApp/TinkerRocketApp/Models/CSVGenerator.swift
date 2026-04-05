//
//  CSVGenerator.swift
//  TinkerRocketApp
//
//  Created by Claude Code
//  Generate CSV files from binary log files.
//  One row per IMU update; other sensors forward-filled.
//  Supports both Mini (ISM6HG256) and Legacy (ICM45686 + H3LIS331) devices.
//

import Foundation

/// Device type auto-detected from binary log data
enum DeviceType {
    case mini    // ISM6HG256 22-byte IMU payload
    case legacy  // ICM45686 36-byte IMU payload + separate H3LIS331 high-G
}

/// nonisolated: CSV generation runs on a background DispatchQueue — must not
/// inherit the module-level @MainActor default isolation.
nonisolated class CSVGenerator {

    typealias ProgressCallback = (Double) -> Void

    // MARK: - Device Auto-Detection

    /// Detect device type by inspecting the first IMU frame's payload size.
    /// Legacy ICM45686 = 36 bytes, Mini ISM6HG256 = 22 bytes.
    func detectDeviceType(frames: [MessageFrame]) -> DeviceType {
        if let imuFrame = frames.first(where: { $0.type == MessageType.imu.rawValue }) {
            return imuFrame.payload.count >= 36 ? .legacy : .mini
        }
        return .mini  // default
    }

    // MARK: - Main CSV Generation

    @discardableResult
    func generateCSV(
        from binaryURL: URL,
        to csvURL: URL,
        progressCallback: ProgressCallback? = nil
    ) throws -> FlightSummary {
        // 1. Parse binary file → [MessageFrame]
        let parser = MessageParser()
        let frames = try parser.parseLogFile(binaryURL)

        guard !frames.isEmpty else {
            throw CSVError.noData
        }

        // 2. Detect device type
        let deviceType = detectDeviceType(frames: frames)

        // 3. Configure sensor converter with rotation from status query
        let converter = SensorConverter()
        configureRotation(converter: converter, deviceType: deviceType, frames: frames)

        // 4. Convert frames to SI units
        var gnssData: [(time_us: UInt32, data: GNSSDataSI)] = []
        var imuData: [(time_us: UInt32, data: ISM6HG256DataSI)] = []
        var baroData: [(time_us: UInt32, data: BMP585DataSI)] = []
        var magData: [(time_us: UInt32, data: MMC5983MADataSI)] = []
        var powerData: [(time_us: UInt32, data: POWERDataSI)] = []
        var nonSensorData: [(time_us: UInt32, data: NonSensorDataSI)] = []
        var highGData: [(time_us: UInt32, data: SensorConverter.HighGDataSI)] = []  // Legacy only

        for frame in frames {
            guard let msgType = MessageType(rawValue: frame.type) else {
                continue  // Skip unknown message types
            }

            do {
                switch msgType {
                case .gnss:
                    // GNSS struct is the same for both devices
                    let raw = try GNSSData(from: frame.payload)
                    let si = converter.convertGNSS(raw)
                    gnssData.append((raw.time_us, si))

                case .imu:
                    switch deviceType {
                    case .mini:
                        let raw = try ISM6HG256Data(from: frame.payload)
                        let si = converter.convertISM6HG256(raw)
                        imuData.append((raw.time_us, si))
                    case .legacy:
                        let raw = try LegacyICM45686Data(from: frame.payload)
                        let si = converter.convertLegacyICM45686(raw)
                        imuData.append((raw.time_us, si))
                    }

                case .baro:
                    switch deviceType {
                    case .mini:
                        let raw = try BMP585Data(from: frame.payload)
                        let si = converter.convertBMP585(raw)
                        baroData.append((raw.time_us, si))
                    case .legacy:
                        let raw = try LegacyMS5611Data(from: frame.payload)
                        let si = converter.convertLegacyMS5611(raw)
                        baroData.append((raw.time_us, si))
                    }

                case .mag:
                    switch deviceType {
                    case .mini:
                        let raw = try MMC5983MAData(from: frame.payload)
                        let si = converter.convertMMC5983MA(raw)
                        magData.append((raw.time_us, si))
                    case .legacy:
                        let raw = try LegacyLIS3MDLData(from: frame.payload)
                        let si = converter.convertLegacyLIS3MDL(raw)
                        magData.append((raw.time_us, si))
                    }

                case .power:
                    // Power encoding is the same for both devices
                    let raw = try POWERData(from: frame.payload)
                    let si = converter.convertPOWER(raw)
                    powerData.append((raw.time_us, si))

                case .nonSensor:
                    switch deviceType {
                    case .mini:
                        let raw = try NonSensorData(from: frame.payload)
                        let si = converter.convertNonSensor(raw)
                        nonSensorData.append((raw.time_us, si))
                    case .legacy:
                        let raw = try LegacyNonSensorData(from: frame.payload)
                        let si = converter.convertLegacyNonSensor(raw)
                        nonSensorData.append((raw.time_us, si))
                    }

                case .h3lis331:
                    // Legacy-only high-G accelerometer
                    let raw = try LegacyH3LIS331Data(from: frame.payload)
                    let si = converter.convertLegacyH3LIS331(raw)
                    highGData.append((raw.time_us, si))

                default:
                    continue  // Skip non-sensor messages
                }
            } catch {
                // Skip frames that fail to parse
                print("Failed to parse frame type \(frame.type): \(error)")
                continue
            }
        }

        // 5. Compute ground pressure from pre-launch baro readings
        let groundPressure = computeGroundPressure(baroData: baroData, nonSensorData: nonSensorData)

        // 6. Determine time range from IMU timestamps (one row per IMU update)
        guard !imuData.isEmpty else {
            throw CSVError.noData
        }

        // Sort all sensor data by timestamp
        gnssData.sort { $0.time_us < $1.time_us }
        imuData.sort { $0.time_us < $1.time_us }
        baroData.sort { $0.time_us < $1.time_us }
        magData.sort { $0.time_us < $1.time_us }
        powerData.sort { $0.time_us < $1.time_us }
        nonSensorData.sort { $0.time_us < $1.time_us }
        highGData.sort { $0.time_us < $1.time_us }

        // If launch was detected, start CSV 4 seconds before launch to avoid
        // hundreds of seconds of dead pre-launch rows when the MRAM buffer
        // spans a long boot period.
        let preLaunchPadding: UInt32 = 4_000_000  // 4 seconds in μs
        let launchTimestamp = nonSensorData.first(where: { $0.data.launch_flag })?.time_us

        let firstIMUTime = imuData.first!.time_us
        let firstTime: UInt32
        if let lt = launchTimestamp, lt > preLaunchPadding {
            firstTime = max(firstIMUTime, lt - preLaunchPadding)
        } else {
            firstTime = firstIMUTime
        }

        // Filter out frames before the valid window
        gnssData.removeAll { $0.time_us < firstTime }
        imuData.removeAll { $0.time_us < firstTime }
        baroData.removeAll { $0.time_us < firstTime }
        magData.removeAll { $0.time_us < firstTime }
        powerData.removeAll { $0.time_us < firstTime }
        nonSensorData.removeAll { $0.time_us < firstTime }
        highGData.removeAll { $0.time_us < firstTime }

        guard !imuData.isEmpty else {
            throw CSVError.noData
        }

        // 7. Create CSV file
        FileManager.default.createFile(atPath: csvURL.path, contents: nil)
        guard let fileHandle = FileHandle(forWritingAtPath: csvURL.path) else {
            throw CSVError.cannotCreateFile
        }

        defer {
            try? fileHandle.close()
        }

        // 8. Write CSV header
        let header = buildCSVHeader()
        fileHandle.write(header.data(using: .utf8)!)

        // 9. Generate one row per IMU update, forward-filling other sensors
        let totalRows = imuData.count
        var rowsWritten = 0

        // Track flight summary stats
        var maxPressureAlt: Double?
        var maxSpeed: Double?
        var maxSpeedTime_us: UInt32?
        var launchTime_us: UInt32?
        var apogeeTime_us: UInt32?

        // Running indices for forward-fill (O(n+m) total)
        var gnssIdx = -1
        var baroIdx = -1
        var magIdx = -1
        var powerIdx = -1
        var nsIdx = -1
        var highGIdx = -1

        // Batch writes for disk performance
        let batchSize = 500
        var batchBuffer = ""
        batchBuffer.reserveCapacity(batchSize * 600)

        for (_, imuSample) in imuData.enumerated() {
            let t = imuSample.time_us

            // Advance each index to the last sample at or before t
            while gnssIdx + 1 < gnssData.count && gnssData[gnssIdx + 1].time_us <= t { gnssIdx += 1 }
            while baroIdx + 1 < baroData.count && baroData[baroIdx + 1].time_us <= t { baroIdx += 1 }
            while magIdx + 1 < magData.count && magData[magIdx + 1].time_us <= t { magIdx += 1 }
            while powerIdx + 1 < powerData.count && powerData[powerIdx + 1].time_us <= t { powerIdx += 1 }
            while nsIdx + 1 < nonSensorData.count && nonSensorData[nsIdx + 1].time_us <= t { nsIdx += 1 }
            while highGIdx + 1 < highGData.count && highGData[highGIdx + 1].time_us <= t { highGIdx += 1 }

            let gnss: GNSSDataSI? = gnssIdx >= 0 ? gnssData[gnssIdx].data : nil
            let baro: BMP585DataSI? = baroIdx >= 0 ? baroData[baroIdx].data : nil
            let mag: MMC5983MADataSI? = magIdx >= 0 ? magData[magIdx].data : nil
            let power: POWERDataSI? = powerIdx >= 0 ? powerData[powerIdx].data : nil
            let nonSensor: NonSensorDataSI? = nsIdx >= 0 ? nonSensorData[nsIdx].data : nil
            let highG: SensorConverter.HighGDataSI? = highGIdx >= 0 ? highGData[highGIdx].data : nil

            // For Legacy: merge H3LIS331 high-G data into the IMU row
            var imu = imuSample.data
            if let hg = highG {
                imu = ISM6HG256DataSI(
                    time_us: imu.time_us,
                    low_g_acc_x: imu.low_g_acc_x,
                    low_g_acc_y: imu.low_g_acc_y,
                    low_g_acc_z: imu.low_g_acc_z,
                    high_g_acc_x: hg.acc_x,
                    high_g_acc_y: hg.acc_y,
                    high_g_acc_z: hg.acc_z,
                    gyro_x: imu.gyro_x,
                    gyro_y: imu.gyro_y,
                    gyro_z: imu.gyro_z
                )
            }

            // Build CSV row
            let time_ms = Double(t - firstTime) / 1000.0
            let row = buildCSVRow(
                time_ms: time_ms,
                gnss: gnss,
                imu: imu,
                baro: baro,
                groundPressure: groundPressure,
                mag: mag,
                power: power,
                nonSensor: nonSensor
            )
            batchBuffer.append(row)

            // Track max pressure altitude
            if let pressure = baro?.pressure, let p0 = groundPressure {
                let alt = pressureToAltitude(pressure: pressure, groundPressure: p0)
                if maxPressureAlt == nil || alt > maxPressureAlt! {
                    maxPressureAlt = alt
                }
            }

            // Track max 3D speed from EKF velocity (peak = burnout)
            // Only update before apogee — IMU integration drifts after apogee
            // so post-apogee speeds are unreliable.
            if let ns = nonSensor {
                if !ns.alt_apogee_flag && !ns.vel_u_apogee_flag {
                    let speed = sqrt(ns.e_vel * ns.e_vel + ns.n_vel * ns.n_vel + ns.u_vel * ns.u_vel)
                    if maxSpeed == nil || speed > maxSpeed! {
                        maxSpeed = speed
                        maxSpeedTime_us = t
                    }
                }

                if ns.launch_flag && launchTime_us == nil {
                    launchTime_us = t
                }

                if (ns.alt_apogee_flag || ns.vel_u_apogee_flag) && apogeeTime_us == nil {
                    apogeeTime_us = t
                }
            }

            rowsWritten += 1

            // Flush batch to disk and report progress
            if rowsWritten % batchSize == 0 {
                fileHandle.write(batchBuffer.data(using: .utf8)!)
                batchBuffer.removeAll(keepingCapacity: true)
                progressCallback?(Double(rowsWritten) / Double(totalRows))
            }
        }

        // Flush remaining rows
        if !batchBuffer.isEmpty {
            fileHandle.write(batchBuffer.data(using: .utf8)!)
        }

        progressCallback?(1.0)

        // Compute event times relative to launch (seconds)
        let burnoutTime: Double? = {
            guard let launch = launchTime_us, let burnout = maxSpeedTime_us,
                  burnout > launch else { return nil }
            return Double(burnout - launch) / 1_000_000.0
        }()
        let apogeeTime: Double? = {
            guard let launch = launchTime_us, let apogee = apogeeTime_us,
                  apogee > launch else { return nil }
            return Double(apogee - launch) / 1_000_000.0
        }()

        return FlightSummary(
            max_altitude_m: maxPressureAlt,
            max_speed_mps: maxSpeed,
            burnout_time_s: burnoutTime,
            apogee_time_s: apogeeTime
        )
    }

    /// Write a flight summary to a JSON file (pretty-printed for human readability)
    func writeSummary(_ summary: FlightSummary, to url: URL) throws {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        let data = try encoder.encode(summary)
        try data.write(to: url)
    }

    // MARK: - Rotation Configuration

    /// Extract rotation config from the binary log and apply to the converter.
    /// Mini: reads 0xA0 (statusQuery) frame for per-board rotation angles.
    /// Legacy: data is raw (unrotated), apply 180° in app.
    private func configureRotation(
        converter: SensorConverter,
        deviceType: DeviceType,
        frames: [MessageFrame]
    ) {
        switch deviceType {
        case .mini:
            // Look for the first statusQuery (0xA0) frame with rotation config
            if let queryFrame = frames.first(where: { $0.type == MessageType.statusQuery.rawValue }),
               let config = try? OutStatusQueryData(from: queryFrame.payload) {
                converter.configureMiniRotation(
                    imuDeg: config.imuRotationDeg,
                    magDeg: config.magRotationDeg
                )
                print("[CSV] Mini rotation config: IMU=\(config.imuRotationDeg)° MAG=\(config.magRotationDeg)°")
            } else {
                print("[CSV] No statusQuery frame found — using default rotation (0°)")
            }

        case .legacy:
            // Legacy data is raw (unrotated), apply 180° rotation in app
            converter.configureLegacyRotation()  // defaults to 180°
            print("[CSV] Legacy device — raw data, app rotation = 180°")
        }
    }

    // MARK: - Helper Functions

    /// Compute ground pressure by averaging the last up-to-100 baro readings before launch.
    /// Uses readings closest to launch for the most stable reference (avoids sensor warmup transients).
    /// Falls back to the last 100 readings if no launch detected.
    private func computeGroundPressure(
        baroData: [(time_us: UInt32, data: BMP585DataSI)],
        nonSensorData: [(time_us: UInt32, data: NonSensorDataSI)]
    ) -> Double? {
        guard !baroData.isEmpty else { return nil }

        let launchTime = nonSensorData.first(where: { $0.data.launch_flag })?.time_us

        let preLaunchPressures: [Double]
        if let lt = launchTime {
            let allPreLaunch = baroData
                .filter { $0.time_us < lt }
                .map { $0.data.pressure }
            preLaunchPressures = Array(allPreLaunch.suffix(100))
        } else {
            preLaunchPressures = Array(baroData.suffix(100).map { $0.data.pressure })
        }

        let pressures = preLaunchPressures.isEmpty
            ? Array(baroData.suffix(10).map { $0.data.pressure })
            : preLaunchPressures

        guard !pressures.isEmpty else { return nil }
        return pressures.reduce(0, +) / Double(pressures.count)
    }

    /// Convert pressure to altitude AGL using the barometric formula.
    private func pressureToAltitude(pressure: Double, groundPressure: Double) -> Double {
        return 44330.0 * (1.0 - pow(pressure / groundPressure, 1.0 / 5.255))
    }

    /// Build CSV header row
    private func buildCSVHeader() -> String {
        var columns: [String] = []

        // Time
        columns.append("Time (ms)")

        // GNSS
        columns.append("Latitude (deg)")
        columns.append("Longitude (deg)")
        columns.append("GNSS Altitude (m)")
        columns.append("Number of Satellites")
        columns.append("PDOP")
        columns.append("GNSS East Velocity (m/s)")
        columns.append("GNSS North Velocity (m/s)")
        columns.append("GNSS Up Velocity (m/s)")
        columns.append("GNSS Horizontal Accuracy (m)")
        columns.append("GNSS Vertical Accuracy (m)")

        // IMU - Low-G Accel
        columns.append("Low-G Acceleration X (m/s2)")
        columns.append("Low-G Acceleration Y (m/s2)")
        columns.append("Low-G Acceleration Z (m/s2)")

        // IMU - High-G Accel
        columns.append("High-G Acceleration X (m/s2)")
        columns.append("High-G Acceleration Y (m/s2)")
        columns.append("High-G Acceleration Z (m/s2)")

        // IMU - Gyro
        columns.append("Gyro X (deg/s)")
        columns.append("Gyro Y (deg/s)")
        columns.append("Gyro Z (deg/s)")

        // Barometer
        columns.append("Pressure (Pa)")
        columns.append("Barometer Temperature (C)")
        columns.append("Pressure Altitude (m)")

        // Magnetometer
        columns.append("Magnetic Field X (uT)")
        columns.append("Magnetic Field Y (uT)")
        columns.append("Magnetic Field Z (uT)")

        // Power
        columns.append("Voltage (V)")
        columns.append("Current (mA)")
        columns.append("State of Charge (%)")

        // NonSensor
        columns.append("Roll (deg)")
        columns.append("Pitch (deg)")
        columns.append("Yaw (deg)")
        columns.append("Roll Command (deg)")
        columns.append("Position East (m)")
        columns.append("Position North (m)")
        columns.append("Position Up (m)")
        columns.append("Velocity East (m/s)")
        columns.append("Velocity North (m/s)")
        columns.append("Velocity Up (m/s)")
        columns.append("Altitude Rate (m/s)")
        columns.append("Landed Flag")
        columns.append("Altitude Apogee Flag")
        columns.append("Velocity Apogee Flag")
        columns.append("Launch Flag")

        return columns.joined(separator: ",") + "\n"
    }

    /// Build a CSV row
    private func buildCSVRow(
        time_ms: Double,
        gnss: GNSSDataSI?,
        imu: ISM6HG256DataSI?,
        baro: BMP585DataSI?,
        groundPressure: Double?,
        mag: MMC5983MADataSI?,
        power: POWERDataSI?,
        nonSensor: NonSensorDataSI?
    ) -> String {
        var values: [String] = []

        // Time
        values.append(String(format: "%.3f", time_ms))

        // GNSS
        values.append(gnss.map { String(format: "%.7f", $0.lat) } ?? "")
        values.append(gnss.map { String(format: "%.7f", $0.lon) } ?? "")
        values.append(gnss.map { String(format: "%.3f", $0.alt) } ?? "")
        values.append(gnss.map { String($0.num_sats) } ?? "")
        values.append(gnss.map { String(format: "%.1f", $0.pdop) } ?? "")
        values.append(gnss.map { String(format: "%.3f", $0.vel_e) } ?? "")
        values.append(gnss.map { String(format: "%.3f", $0.vel_n) } ?? "")
        values.append(gnss.map { String(format: "%.3f", $0.vel_u) } ?? "")
        values.append(gnss.map { String(format: "%.1f", $0.h_acc) } ?? "")
        values.append(gnss.map { String(format: "%.1f", $0.v_acc) } ?? "")

        // IMU - Low-G Accel
        values.append(imu.map { String(format: "%.6f", $0.low_g_acc_x) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.low_g_acc_y) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.low_g_acc_z) } ?? "")

        // IMU - High-G Accel
        values.append(imu.map { String(format: "%.6f", $0.high_g_acc_x) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.high_g_acc_y) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.high_g_acc_z) } ?? "")

        // IMU - Gyro
        values.append(imu.map { String(format: "%.6f", $0.gyro_x) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.gyro_y) } ?? "")
        values.append(imu.map { String(format: "%.6f", $0.gyro_z) } ?? "")

        // Barometer
        values.append(baro.map { String(format: "%.2f", $0.pressure) } ?? "")
        values.append(baro.map { String(format: "%.2f", $0.temperature) } ?? "")
        if let pressure = baro?.pressure, let p0 = groundPressure {
            let alt = pressureToAltitude(pressure: pressure, groundPressure: p0)
            values.append(String(format: "%.2f", alt))
        } else {
            values.append("")
        }

        // Magnetometer
        values.append(mag.map { String(format: "%.6f", $0.mag_x) } ?? "")
        values.append(mag.map { String(format: "%.6f", $0.mag_y) } ?? "")
        values.append(mag.map { String(format: "%.6f", $0.mag_z) } ?? "")

        // Power
        values.append(power.map { String(format: "%.3f", $0.voltage) } ?? "")
        values.append(power.map { String(format: "%.1f", $0.current) } ?? "")
        values.append(power.map { String(format: "%.1f", $0.soc) } ?? "")

        // NonSensor
        values.append(nonSensor.map { String(format: "%.2f", $0.roll) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.pitch) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.yaw) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.roll_cmd) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.e_pos) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.n_pos) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.u_pos) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.e_vel) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.n_vel) } ?? "")
        values.append(nonSensor.map { String(format: "%.2f", $0.u_vel) } ?? "")
        values.append(nonSensor.map { String(format: "%.1f", $0.altitude_rate) } ?? "")
        values.append(nonSensor.map { $0.alt_landed_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.alt_apogee_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.vel_u_apogee_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.launch_flag ? "1" : "0" } ?? "")

        return values.joined(separator: ",") + "\n"
    }
}

// MARK: - Flight Summary

/// nonisolated: FlightSummary is decoded on background queues (FileCache, CSVGenerator).
nonisolated struct FlightSummary: Codable, Sendable {
    /// Maximum pressure altitude above ground level (meters)
    let max_altitude_m: Double?
    /// Maximum 3D speed from EKF velocity estimate (m/s)
    let max_speed_mps: Double?
    /// Time from launch to motor burnout (seconds). Burnout = moment of peak speed.
    let burnout_time_s: Double?
    /// Time from launch to apogee (seconds). Apogee = first altitude or velocity apogee flag.
    let apogee_time_s: Double?
}

// MARK: - CSV Errors

enum CSVError: Error {
    case noData
    case cannotCreateFile
}
