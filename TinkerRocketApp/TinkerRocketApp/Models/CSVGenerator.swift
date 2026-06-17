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

                case .iis2mdc:
                    // New Mini PCB rev: IIS2MDC replaces MMC5983MA.
                    // Converter output type is MMC5983MADataSI so the
                    // downstream Magnetic Field X/Y/Z (µT) columns
                    // emit the same way regardless of which mag chip
                    // is fitted.
                    let raw = try IIS2MDCData(from: frame.payload)
                    let si = converter.convertIIS2MDC(raw)
                    magData.append((raw.time_us, si))

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
        var launchTime_us: UInt32?
        var burnoutTime_us: UInt32?  // first NSF_BURNOUT flag latch (#196)
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

            // Track max 3D speed from EKF velocity for the max_speed_mps
            // metric. Only update before apogee — IMU integration drifts
            // after apogee so post-apogee speeds are unreliable.
            //
            // Gate on the master voted apogee_flag — a single per-detector
            // false positive (#142) previously gated off max-speed tracking
            // ~6s before real apogee, missing the burnout peak entirely.
            // Legacy 43-byte logs predate the master flag, so fall back to
            // requiring BOTH baro and velocity detectors to agree.
            if let ns = nonSensor {
                let postApogee = ns.apogee_flag ||
                                 (ns.alt_apogee_flag && ns.vel_u_apogee_flag)
                if !postApogee {
                    let speed = sqrt(ns.e_vel * ns.e_vel + ns.n_vel * ns.n_vel + ns.u_vel * ns.u_vel)
                    if maxSpeed == nil || speed > maxSpeed! {
                        maxSpeed = speed
                    }
                }

                if ns.launch_flag && launchTime_us == nil {
                    launchTime_us = t
                }

                // burnout_time_s is the first NSF_BURNOUT flag latch (#196).
                // Previously the sidecar used peak-velocity time as a
                // burnout proxy ("peak = burnout"), but that diverges from
                // the firmware's own decision by multiple seconds on flights
                // where peak |v| comes later than motor cutoff (RIM-66 5/17
                // had FW burnout at T+2.13s vs peak |v| at T+9.30s).
                // Read the flag directly so the sidecar timing agrees with
                // pyro / kinematic_checks / every other consumer of the bin.
                // Legacy logs predating the burnout bit set in firmware
                // decode as false → burnoutTime_us stays nil → JSON emits
                // null rather than a wrong proxy value.
                if ns.burnout_flag && burnoutTime_us == nil {
                    burnoutTime_us = t
                }

                // Source apogee timestamp from the master voted flag — not
                // from a per-detector OR — to avoid a single noisy detector
                // (e.g. baro during boost, see #142) polluting the sidecar's
                // canonical apogee_time_s.  Legacy 43-byte logs decode the
                // master flag as false, so apogeeTime_us stays nil and the
                // JSON emits null rather than a per-detector first-fire time.
                if ns.apogee_flag && apogeeTime_us == nil {
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
            guard let launch = launchTime_us, let burnout = burnoutTime_us,
                  burnout > launch else { return nil }
            return Double(burnout - launch) / 1_000_000.0
        }()
        let apogeeTime: Double? = {
            guard let launch = launchTime_us, let apogee = apogeeTime_us,
                  apogee > launch else { return nil }
            return Double(apogee - launch) / 1_000_000.0
        }()

        // Decode the flight settings snapshot (#165), if present. The FC emits
        // it a few times right after launch for redundancy; all copies are
        // identical, so take the first one that decodes.
        let flightSettings: FlightSettings? = frames
            .first(where: { $0.type == MessageType.flightSettings.rawValue })
            .flatMap { try? FlightSettingsData(from: $0.payload) }
            .map { FlightSettings(from: $0) }

        return FlightSummary(
            max_altitude_m: maxPressureAlt,
            max_speed_mps: maxSpeed,
            burnout_time_s: burnoutTime,
            apogee_time_s: apogeeTime,
            settings: flightSettings
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
                    magDeg: config.magRotationDeg,
                    iisDeg: config.iisRotationDeg
                )
                print("[CSV] Mini rotation config: IMU=\(config.imuRotationDeg)° MAG=\(config.magRotationDeg)° IIS=\(config.iisRotationDeg.map { String($0) } ?? "n/a")°")
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
        // Per #142/#143: per-detector outputs renamed for clarity and master
        // voted result added alongside.  Old logs (43-byte NonSensorData)
        // emit 0 for the GPS/Pitch/Master columns.
        columns.append("Apogee Detector: Baro")
        columns.append("Apogee Detector: Velocity")
        columns.append("Apogee Detector: GPS")
        columns.append("Apogee Detector: Pitch")
        columns.append("Apogee Flag (Master)")
        columns.append("Launch Flag")

        // Pyro status bits — 4 channels (legacy files emit 0s for ch3/4)
        columns.append("Pyro 1 Continuity")
        columns.append("Pyro 2 Continuity")
        columns.append("Pyro 3 Continuity")
        columns.append("Pyro 4 Continuity")
        columns.append("Pyro 1 Fired")
        columns.append("Pyro 2 Fired")
        columns.append("Pyro 3 Fired")
        columns.append("Pyro 4 Fired")
        columns.append("Reboot Recovery")
        columns.append("FC Guidance Enabled")

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
        values.append(nonSensor.map { $0.gps_apogee_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pitch_apogee_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.apogee_flag ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.launch_flag ? "1" : "0" } ?? "")

        // Pyro status bits (4 channels)
        values.append(nonSensor.map { $0.pyro1_continuity ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro2_continuity ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro3_continuity ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro4_continuity ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro1_fired ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro2_fired ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro3_fired ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.pyro4_fired ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.reboot_recovery ? "1" : "0" } ?? "")
        values.append(nonSensor.map { $0.guidance_enabled ? "1" : "0" } ?? "")

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
    /// Runtime roll-control / IMU settings the FC flew with, snapshotted into
    /// the log at launch (#165). nil for flights logged before the firmware
    /// emitted the settings frame.
    let settings: FlightSettings?
}

// MARK: - Flight Settings (#165)

/// Round a float32 to `digits` significant figures so the JSON shows clean
/// gains (e.g. 0.04, not 0.039999999105930) instead of float32→Double noise.
private func sigFig(_ v: Float, _ digits: Int = 6) -> Double {
    let d = Double(v)
    if d == 0 || !d.isFinite { return d }
    let mag = floor(log10(abs(d)))
    let factor = pow(10.0, Double(digits - 1) - mag)
    return (d * factor).rounded() / factor
}

/// Settings block written into the per-flight summary JSON. Mirrors every
/// per-rocket setting editable in the app's settings UI (#165) plus the
/// issue's IMU full-scale and outer-loop knobs.
nonisolated struct FlightSettings: Codable, Sendable {
    let fw_git_sha: String
    let fw_dirty: Bool
    let sounds_enabled: Bool
    let roll_control: RollControlSettings
    let servo: ServoSettings
    let camera: CameraSettings
    let pyro: PyroSettings
    let imu: IMUSettings

    init(from raw: FlightSettingsData) {
        fw_git_sha = raw.fw_git_sha
        fw_dirty = raw.fwDirty
        sounds_enabled = raw.soundsEnabled
        roll_control = RollControlSettings(from: raw)
        servo = ServoSettings(from: raw)
        camera = CameraSettings(type: CameraSettings.label(raw.camera_type))
        pyro = PyroSettings(from: raw)
        imu = IMUSettings(
            gyro_fs_dps: Int(raw.ism6_gyro_fs_dps),
            low_g_fs_g: Int(raw.ism6_low_g_fs_g),
            high_g_fs_g: Int(raw.ism6_high_g_fs_g),
            mounting: MountingSettings(from: raw)
        )
    }
}

nonisolated struct RollControlSettings: Codable, Sendable {
    /// "rate" (null-rate inner loop), "angle" (cascaded, single setpoint),
    /// or "angle_profile" (cascaded, follows the waypoint profile).
    let mode: String
    let kp: Double
    let ki: Double
    let kd: Double
    let d_lpf_hz: Double
    let kp_angle: Double
    let cmd_limit_min_deg: Double
    let cmd_limit_max_deg: Double
    let delay_ms: Int
    let rate_cap_dps: Double
    let roll_rate_set_point: Double
    let guidance_enabled: Bool
    let gain_schedule: GainScheduleSettings
    let profile: [RollWaypointJSON]

    init(from raw: FlightSettingsData) {
        if raw.num_waypoints > 0 {
            mode = "angle_profile"
        } else if raw.useAngleControl {
            mode = "angle"
        } else {
            mode = "rate"
        }
        kp = sigFig(raw.kp)
        ki = sigFig(raw.ki)
        kd = sigFig(raw.kd)
        d_lpf_hz = sigFig(raw.d_lpf_hz)
        kp_angle = sigFig(raw.kp_angle)
        cmd_limit_min_deg = sigFig(raw.min_cmd_deg)
        cmd_limit_max_deg = sigFig(raw.max_cmd_deg)
        delay_ms = Int(raw.roll_delay_ms)
        rate_cap_dps = sigFig(raw.kp_angle_rate_cap_dps)
        roll_rate_set_point = sigFig(raw.roll_rate_set_point)
        guidance_enabled = raw.guidanceEnabled
        gain_schedule = GainScheduleSettings(
            enabled: raw.gainScheduleEnabled,
            v_ref: sigFig(raw.gs_v_ref),
            v_min: sigFig(raw.gs_v_min),
            scale_cap: sigFig(raw.gs_scale_cap)
        )
        profile = raw.waypoints.map {
            RollWaypointJSON(
                time_s: sigFig($0.time_s),
                angle_deg: sigFig($0.angle_deg),
                mode: $0.mode == 1 ? "null_rate" : "angle"
            )
        }
    }
}

nonisolated struct GainScheduleSettings: Codable, Sendable {
    let enabled: Bool
    let v_ref: Double
    let v_min: Double
    let scale_cap: Double
}

nonisolated struct RollWaypointJSON: Codable, Sendable {
    let time_s: Double
    let angle_deg: Double
    let mode: String   // "angle" or "null_rate"
}

nonisolated struct ServoSettings: Codable, Sendable {
    let enabled: Bool
    let bias_us: [Int]
    let frequency_hz: Int
    let min_pulse_us: Int
    let max_pulse_us: Int

    init(from raw: FlightSettingsData) {
        enabled = raw.servoEnabled
        bias_us = raw.servo_bias_us.map { Int($0) }
        frequency_hz = Int(raw.servo_hz)
        min_pulse_us = Int(raw.servo_min_us)
        max_pulse_us = Int(raw.servo_max_us)
    }
}

nonisolated struct CameraSettings: Codable, Sendable {
    let type: String   // "none" | "gopro" | "runcam"

    static func label(_ t: UInt8) -> String {
        switch t {
        case 1: return "gopro"
        case 2: return "runcam"
        default: return "none"
        }
    }
}

nonisolated struct PyroSettings: Codable, Sendable {
    let ch1: PyroChannelSettings
    let ch2: PyroChannelSettings
    let ch3: PyroChannelSettings
    let ch4: PyroChannelSettings

    init(from raw: FlightSettingsData) {
        ch1 = PyroChannelSettings(enabled: raw.pyro_enabled[0],
                                  mode: raw.pyro_trigger_mode[0],
                                  value: raw.pyro_trigger_value[0])
        ch2 = PyroChannelSettings(enabled: raw.pyro_enabled[1],
                                  mode: raw.pyro_trigger_mode[1],
                                  value: raw.pyro_trigger_value[1])
        ch3 = PyroChannelSettings(enabled: raw.pyro_enabled[2],
                                  mode: raw.pyro_trigger_mode[2],
                                  value: raw.pyro_trigger_value[2])
        ch4 = PyroChannelSettings(enabled: raw.pyro_enabled[3],
                                  mode: raw.pyro_trigger_mode[3],
                                  value: raw.pyro_trigger_value[3])
    }
}

nonisolated struct PyroChannelSettings: Codable, Sendable {
    let enabled: Bool
    /// "time_after_apogee" (value = seconds) or "altitude_on_descent" (value = meters AGL).
    let trigger_mode: String
    let trigger_value: Double

    init(enabled: Bool, mode: UInt8, value: Float) {
        self.enabled = enabled
        self.trigger_mode = mode == 1 ? "altitude_on_descent" : "time_after_apogee"
        self.trigger_value = sigFig(value)
    }
}

nonisolated struct IMUSettings: Codable, Sendable {
    let gyro_fs_dps: Int
    let low_g_fs_g: Int
    let high_g_fs_g: Int
    /// Board→rocket mounting orientation (v2 settings frames). nil on
    /// pre-orientation logs, which always meant the +X-nose mounting.
    let mounting: MountingSettings?
}

nonisolated struct MountingSettings: Codable, Sendable {
    /// Which board axis pointed at the nose + clocking, e.g. "+X", "-Z r90".
    let orientation: String
    /// How it was determined: "default" | "manual" | "auto_snap" | "auto_exact".
    let mode: String
    /// Auto-snap residual angle (deg); 0 for default/manual.
    let residual_deg: Double

    init?(from raw: FlightSettingsData) {
        guard let code = raw.b2r_code, let mode = raw.b2r_mode else { return nil }
        orientation = FlightSettingsData.b2rName(code: code)
        switch mode {
        case 1: self.mode = "manual"
        case 2: self.mode = "auto_snap"
        case 3: self.mode = "auto_exact"
        default: self.mode = "default"
        }
        residual_deg = sigFig(raw.b2r_residual_deg ?? 0, 3)
    }
}

// MARK: - CSV Errors

enum CSVError: Error {
    case noData
    case cannotCreateFile
}
