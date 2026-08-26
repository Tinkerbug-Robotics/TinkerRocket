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
        // #514: the quaternion is the only reconstructable attitude — Roll is the
        // body-Z azimuth while Pitch/Yaw are ZYX-Euler, so those three are NOT a
        // valid Euler triple. Two name generations appear in the wild: the
        // current semicolon names (the #514 comma names broke naive readers and
        // were re-released with semicolons; repairSplitHeaderNames folds the
        // 7/14–7/16 comma exports into this form) and the pre-#514 plain names.
        ("Rotation", [
            "Gyro X (deg/s)", "Gyro Y (deg/s)", "Gyro Z (deg/s)",
            "Quat q0", "Quat q1", "Quat q2", "Quat q3",
            "Roll (deg; body-Z azimuth)", "Pitch (deg; ZYX Euler)", "Yaw (deg; ZYX Euler)",
            "Roll (deg)", "Pitch (deg)", "Yaw (deg)",           // pre-#514 exports
            "Roll Command (deg)"
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
        // #142/#143 renamed the per-detector apogee columns in the WRITER and
        // added a voted master, but this list kept the old names — "Altitude
        // Apogee Flag" / "Velocity Apogee Flag" — for a year.  The picker
        // intersects these groups with the file's actual headers, so the dead
        // names were silently dropped rather than shown as ghosts, and the 16
        // columns that do exist were unreachable: an operator investigating a
        // deployment-timing anomaly could not plot when the master apogee vote
        // latched or when a pyro fired (#838 item 5).
        ("Flags", [
            "Launch Flag", "Landed Flag", "Deployed Flag",
            "Reboot Recovery", "FC Guidance Enabled"
        ]),
        // Its own group because apogee is not one thing: four independent
        // detectors vote and the master latches on that vote, and telling a
        // deployment story means plotting them against each other.
        ("Apogee", [
            "Apogee Detector: Baro", "Apogee Detector: Velocity",
            "Apogee Detector: GPS", "Apogee Detector: Pitch",
            "Apogee Flag (Master)"
        ]),
        ("Pyro", [
            "Pyro 1 Continuity", "Pyro 2 Continuity",
            "Pyro 3 Continuity", "Pyro 4 Continuity",
            "Pyro 1 Fired", "Pyro 2 Fired", "Pyro 3 Fired", "Pyro 4 Fired"
        ]),
        ("Diagnostics", [
            "EKF Ticks"
        ])
    ]

    /// Column names kept in `columnGroups` that the CURRENT writer no longer
    /// emits, because files written by older builds still carry them and must
    /// stay plottable.
    ///
    /// The parity test that pins `columnGroups` against
    /// `CSVGenerator.buildCSVHeader()` allows exactly these — anything else
    /// absent from the header is a rename that left this list behind, which is
    /// what #838 item 5 was.
    static let legacyColumnAliases: Set<String> = [
        // Pre-#514 attitude names.  #514 renamed these to spell out that Roll
        // is a body-Z azimuth while Pitch/Yaw are ZYX-Euler, so the three are
        // not a valid Euler triple.
        "Roll (deg)", "Pitch (deg)", "Yaw (deg)"
    ]

    /// Header columns deliberately absent from `columnGroups` — not plottable
    /// as a series.
    static let nonPlottableColumns: Set<String> = [
        "Time (ms)"      // the X axis
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

    /// Apps built 2026-07-14…07-16 (#514, pre-fix) emitted three column names
    /// containing literal commas without quoting, so the header row carried
    /// three more comma-separated tokens than every data row and all columns
    /// after "Quat q3" loaded shifted. Re-join those known token pairs into
    /// the current comma-free names so existing exports parse correctly.
    /// (The writer no longer puts commas in column names — see CSVGenerator.)
    private nonisolated static let splitHeaderRepairs: [String: (second: String, joined: String)] = [
        "Roll (deg": ("body-Z azimuth)", "Roll (deg; body-Z azimuth)"),
        "Pitch (deg": ("ZYX Euler)", "Pitch (deg; ZYX Euler)"),
        "Yaw (deg": ("ZYX Euler)", "Yaw (deg; ZYX Euler)"),
    ]

    nonisolated static func repairSplitHeaderNames(_ headers: [String]) -> [String] {
        var repaired: [String] = []
        repaired.reserveCapacity(headers.count)
        var i = 0
        while i < headers.count {
            if i + 1 < headers.count,
               let fix = splitHeaderRepairs[headers[i]],
               headers[i + 1] == fix.second {
                repaired.append(fix.joined)
                i += 2
            } else {
                repaired.append(headers[i])
                i += 1
            }
        }
        return repaired
    }

    /// Parse a CSV file into columnar FlightCSVData.
    /// Call from a background Task for large files.
    nonisolated static func parse(url: URL) throws -> FlightCSVData {
        let content = try String(contentsOf: url, encoding: .utf8)
        let lines = content.split(separator: "\n", omittingEmptySubsequences: true)

        guard !lines.isEmpty else {
            throw CSVParserError.emptyFile
        }

        let headerLine = lines[0]
        let rawHeaders = headerLine.split(separator: ",").map { String($0).trimmingCharacters(in: .whitespaces) }
        let headers = repairSplitHeaderNames(rawHeaders)
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
            // Pad short rows.  min() guards over-long rows (more fields than the
            // header — e.g. garbled LoRa telemetry like "166.65.5" splitting into
            // extra fields): bare `fields.count..<columnCount` traps with
            // "Range requires lowerBound <= upperBound" when fields.count >
            // columnCount (#236).  Extra fields were already consumed above.
            for col in min(fields.count, columnCount)..<columnCount {
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
