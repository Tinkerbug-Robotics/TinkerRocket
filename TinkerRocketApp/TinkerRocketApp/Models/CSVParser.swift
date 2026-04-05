//
//  CSVParser.swift
//  TinkerRocketApp
//
//  Parse cached CSV flight data into columnar format for charting
//

import Foundation

// MARK: - Parsed CSV Data Model

/// Parsed CSV flight data in columnar format for efficient charting
struct FlightCSVData {
    let headers: [String]
    let columns: [String: [Double]]
    let rowCount: Int

    /// Column names grouped by sensor category for the picker UI (rocket on-board CSV)
    static let columnGroups: [(name: String, columns: [String])] = [
        ("Altitude & Position", [
            "Pressure Altitude (m)", "GNSS Altitude (m)",
            "Position East (m)", "Position North (m)", "Position Up (m)"
        ]),
        ("Velocity", [
            "Altitude Rate (m/s)",
            "Velocity East (m/s)", "Velocity North (m/s)", "Velocity Up (m/s)",
            "GNSS East Velocity (m/s)", "GNSS North Velocity (m/s)", "GNSS Up Velocity (m/s)"
        ]),
        ("Acceleration", [
            "Low-G Acceleration X (m/s2)", "Low-G Acceleration Y (m/s2)", "Low-G Acceleration Z (m/s2)",
            "High-G Acceleration X (m/s2)", "High-G Acceleration Y (m/s2)", "High-G Acceleration Z (m/s2)"
        ]),
        ("Rotation", [
            "Gyro X (deg/s)", "Gyro Y (deg/s)", "Gyro Z (deg/s)",
            "Roll (deg)", "Pitch (deg)", "Yaw (deg)", "Roll Command (deg)"
        ]),
        ("Environment", [
            "Pressure (Pa)", "Barometer Temperature (C)",
            "Magnetic Field X (uT)", "Magnetic Field Y (uT)", "Magnetic Field Z (uT)"
        ]),
        ("Power", [
            "Voltage (V)", "Current (mA)", "State of Charge (%)"
        ]),
        ("GPS", [
            "Latitude (deg)", "Longitude (deg)",
            "Number of Satellites", "PDOP",
            "GNSS Horizontal Accuracy (m)", "GNSS Vertical Accuracy (m)"
        ]),
        ("Flags", [
            "Launch Flag", "Altitude Apogee Flag", "Velocity Apogee Flag", "Landed Flag"
        ])
    ]

    /// Column names grouped by category for LoRa base-station CSV files
    static let loraColumnGroups: [(name: String, columns: [String])] = [
        ("Altitude & Speed", [
            "pressure_alt", "alt_m", "alt_rate", "speed",
            "max_alt", "max_speed"
        ]),
        ("Acceleration", [
            "acc_x", "acc_y", "acc_z"
        ]),
        ("Rotation", [
            "gyro_x", "gyro_y", "gyro_z",
            "roll", "pitch", "yaw"
        ]),
        ("Power", [
            "voltage", "current", "soc"
        ]),
        ("GPS", [
            "lat", "lon", "num_sats", "pdop", "h_acc"
        ]),
        ("Radio", [
            "rssi", "snr"
        ]),
        ("State & Flags", [
            "state", "launch", "vel_apo", "alt_apo", "landed"
        ])
    ]

    /// Pick the right column groups based on which headers are present.
    /// LoRa CSVs use short names (e.g. "pressure_alt"), rocket CSVs use
    /// descriptive names (e.g. "Pressure Altitude (m)").
    static func columnGroups(for headers: [String]) -> [(name: String, columns: [String])] {
        if headers.contains("pressure_alt") || headers.contains("rssi") {
            return loraColumnGroups
        }
        return columnGroups
    }
}

// MARK: - Parser Errors

enum CSVParserError: Error, LocalizedError {
    case emptyFile
    case noHeader

    var errorDescription: String? {
        switch self {
        case .emptyFile: return "CSV file is empty"
        case .noHeader: return "CSV file has no header row"
        }
    }
}

// MARK: - CSV Parser

class CSVParser {

    /// Parse a CSV file into columnar FlightCSVData.
    /// Call from a background Task for large files.
    nonisolated static func parse(url: URL) throws -> FlightCSVData {
        let content = try String(contentsOf: url, encoding: .utf8)
        let lines = content.split(separator: "\n", omittingEmptySubsequences: true)

        guard !lines.isEmpty else {
            throw CSVParserError.emptyFile
        }

        let headerLine = lines[0]
        let headers = headerLine.split(separator: ",").map { String($0).trimmingCharacters(in: .whitespaces) }
        let columnCount = headers.count

        guard columnCount > 0 else {
            throw CSVParserError.noHeader
        }

        let dataLines = lines.dropFirst()
        let rowCount = dataLines.count

        // Pre-allocate columnar arrays
        var columns: [[Double]] = Array(repeating: [], count: columnCount)
        for i in 0..<columnCount {
            columns[i].reserveCapacity(rowCount)
        }

        // Parse all rows in a single pass
        for line in dataLines {
            let fields = line.split(separator: ",", omittingEmptySubsequences: false)
            for col in 0..<min(fields.count, columnCount) {
                if let value = Double(fields[col]) {
                    columns[col].append(value)
                } else {
                    columns[col].append(.nan)
                }
            }
            // Pad if row has fewer columns than header
            for col in fields.count..<columnCount {
                columns[col].append(.nan)
            }
        }

        // Build dictionary keyed by column name
        var dict: [String: [Double]] = [:]
        dict.reserveCapacity(columnCount)
        for (i, header) in headers.enumerated() {
            dict[header] = columns[i]
        }

        return FlightCSVData(headers: headers, columns: dict, rowCount: rowCount)
    }

    // MARK: - LTTB Downsampling

    /// Largest Triangle Three Buckets downsampling.
    /// Preserves visually important peaks and troughs (e.g., apogee, max-g)
    /// that naive stride-based decimation would miss.
    static func lttbDecimate(
        x: [Double], y: [Double], targetCount: Int
    ) -> [(x: Double, y: Double)] {
        let count = x.count
        guard count > targetCount, targetCount >= 3 else {
            // No decimation needed
            return zip(x, y).map { ($0, $1) }
        }

        var result: [(x: Double, y: Double)] = []
        result.reserveCapacity(targetCount)

        // Always include first point
        result.append((x[0], y[0]))

        let bucketSize = Double(count - 2) / Double(targetCount - 2)
        var prevIndex = 0

        for i in 1..<(targetCount - 1) {
            // Current bucket range
            let bucketStart = Int(Double(i - 1) * bucketSize) + 1
            let bucketEnd = min(Int(Double(i) * bucketSize), count - 2)

            // Next bucket range (for computing average)
            let nextBucketStart = Int(Double(i) * bucketSize) + 1
            let nextBucketEnd = min(Int(Double(i + 1) * bucketSize) + 1, count)

            // Average of next bucket
            var avgX = 0.0, avgY = 0.0
            var nextCount = 0
            for j in nextBucketStart..<nextBucketEnd {
                avgX += x[j]; avgY += y[j]; nextCount += 1
            }
            if nextCount > 0 { avgX /= Double(nextCount); avgY /= Double(nextCount) }

            // Find point in current bucket that makes largest triangle
            // with the previous selected point and the next bucket average
            var maxArea = -1.0
            var bestIndex = bucketStart
            let px = x[prevIndex], py = y[prevIndex]

            for j in bucketStart...bucketEnd {
                let area = abs((px - avgX) * (y[j] - py) - (px - x[j]) * (avgY - py))
                if area > maxArea {
                    maxArea = area
                    bestIndex = j
                }
            }

            result.append((x[bestIndex], y[bestIndex]))
            prevIndex = bestIndex
        }

        // Always include last point
        result.append((x[count - 1], y[count - 1]))

        return result
    }
}
