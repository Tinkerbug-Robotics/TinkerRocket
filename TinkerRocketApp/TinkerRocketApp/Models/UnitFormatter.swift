//
//  UnitFormatter.swift
//  TinkerRocketApp
//
//  Centralized display-unit formatting (#160).  All telemetry, .bin/.csv
//  logs, and the wire protocol stay SI (canonical) — these helpers convert
//  at render time per the user's chosen UnitSystem, so logged data is never
//  altered and analysis scripts are unaffected.
//
//  Methods default to UnitSystem.current so non-View code (FlightAnnouncer,
//  TelemetryData computed props) can format too.  Views pass their
//  @AppStorage("unitSystem") value so the UI re-renders when the toggle flips.
//

import Foundation

enum UnitSystem: String, CaseIterable, Identifiable {
    case metric
    case imperial

    var id: String { rawValue }

    var label: String {
        switch self {
        case .metric:   return "Metric (SI)"
        case .imperial: return "Imperial"
        }
    }

    /// Persisted choice, readable from non-View code.  Shares the
    /// "unitSystem" UserDefaults key with the @AppStorage in SettingsView.
    static var current: UnitSystem {
        UnitSystem(rawValue: UserDefaults.standard.string(forKey: "unitSystem") ?? "") ?? .metric
    }
}

// Exact conversion factors.
private let feetPerMeter = 1.0 / 0.3048
private let feetPerMile = 5280.0
private let standardGravity = 9.80665   // m/s² per g

enum UnitFormatter {

    // MARK: - Altitude
    //
    // Never switches to miles: rocket apogees routinely exceed 5280 ft and
    // "1.2 mi" is useless for altitude.  Horizontal distances use distance().

    static func altitude(_ meters: Double, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:
            return meters >= 1000
                ? String(format: "%.2f km", meters / 1000.0)
                : String(format: "%.0f m", meters)
        case .imperial:
            return String(format: "%.0f ft", meters * feetPerMeter)
        }
    }

    // MARK: - Horizontal distance (promotes to km / mi when large)

    static func distance(_ meters: Double, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:
            return meters >= 1000
                ? String(format: "%.1f km", meters / 1000.0)
                : String(format: "%.0f m", meters)
        case .imperial:
            let feet = meters * feetPerMeter
            return feet >= feetPerMile
                ? String(format: "%.2f mi", feet / feetPerMile)
                : String(format: "%.0f ft", feet)
        }
    }

    // MARK: - Speed

    static func speed(_ mps: Double, decimals: Int = 1, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return String(format: "%.\(decimals)f m/s", mps)
        case .imperial: return String(format: "%.\(decimals)f ft/s", mps * feetPerMeter)
        }
    }

    static func speedUnit(_ system: UnitSystem = .current) -> String {
        system == .metric ? "m/s" : "ft/s"
    }

    // MARK: - Acceleration (g in imperial)

    static func acceleration(_ mps2: Double, decimals: Int = 2, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return String(format: "%.\(decimals)f m/s\u{00B2}", mps2)
        case .imperial: return String(format: "%.\(decimals)f g", mps2 / standardGravity)
        }
    }

    /// Unit suffix only — for table headers / IMU row labels that format the
    /// triplet of axis values separately.
    static func accelerationUnit(_ system: UnitSystem = .current) -> String {
        system == .metric ? "m/s\u{00B2}" : "g"
    }

    /// Scale an acceleration triplet into the display unit (m/s² or g) without
    /// formatting — callers lay out the three axes themselves.
    static func accelerationValue(_ mps2: Double, system: UnitSystem = .current) -> Double {
        system == .metric ? mps2 : mps2 / standardGravity
    }

    // MARK: - Temperature

    static func temperature(_ celsius: Double, decimals: Int = 1, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return String(format: "%.\(decimals)f \u{00B0}C", celsius)
        case .imperial: return String(format: "%.\(decimals)f \u{00B0}F", celsius * 9.0 / 5.0 + 32.0)
        }
    }

    // MARK: - Spoken (FlightAnnouncer voice callouts)

    static func spokenAltitude(_ meters: Double, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return "\(Int(meters.rounded())) meters"
        case .imperial: return "\(Int((meters * feetPerMeter).rounded())) feet"
        }
    }

    static func spokenSpeed(_ mps: Double, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return "\(Int(mps.rounded())) meters per second"
        case .imperial: return "\(Int((mps * feetPerMeter).rounded())) feet per second"
        }
    }

    static func spokenDistance(_ meters: Double, system: UnitSystem = .current) -> String {
        switch system {
        case .metric:   return "\(Int(meters.rounded())) meters"
        case .imperial: return "\(Int((meters * feetPerMeter).rounded())) feet"
        }
    }

    // MARK: - Raw numeric conversions (charts, input fields)

    static func metersToDisplay(_ meters: Double, system: UnitSystem = .current) -> Double {
        system == .metric ? meters : meters * feetPerMeter
    }

    static func displayToMeters(_ value: Double, system: UnitSystem = .current) -> Double {
        system == .metric ? value : value / feetPerMeter
    }

    static func mpsToDisplay(_ mps: Double, system: UnitSystem = .current) -> Double {
        system == .metric ? mps : mps * feetPerMeter
    }

    static func displayToMps(_ value: Double, system: UnitSystem = .current) -> Double {
        system == .metric ? value : value / feetPerMeter
    }

    // MARK: - CSV column conversion (FlightChartView)
    //
    // Logs are canonical SI; the chart converts at render time keyed on the
    // trailing "(unit)" token in the column header.  Non-convertible columns
    // (deg, deg/s, uT, Pa, mA, V, %, ms, flags) pass through unchanged.

    /// Returns the display label + converted values for a plotted CSV column.
    static func convertSeries(column: String, values: [Double],
                              system: UnitSystem = .current) -> (label: String, values: [Double]) {
        guard system == .imperial else { return (column, values) }

        switch columnUnit(column) {
        case "m":
            return (relabel(column, to: "ft"), values.map { $0 * feetPerMeter })
        case "m/s":
            return (relabel(column, to: "ft/s"), values.map { $0 * feetPerMeter })
        case "m/s2":
            return (relabel(column, to: "g"), values.map { $0 / standardGravity })
        case "C":
            return (relabel(column, to: "\u{00B0}F"), values.map { $0 * 9.0 / 5.0 + 32.0 })
        default:
            return (column, values)
        }
    }

    /// Extract the unit token inside the trailing "(...)" of a column header.
    private static func columnUnit(_ column: String) -> String? {
        guard let open = column.lastIndex(of: "("),
              let close = column.lastIndex(of: ")"),
              open < close else { return nil }
        return String(column[column.index(after: open)..<close])
    }

    /// Replace the trailing "(unit)" of a column header with a new unit.
    private static func relabel(_ column: String, to unit: String) -> String {
        guard let open = column.lastIndex(of: "(") else { return column }
        return String(column[..<open]) + "(\(unit))"
    }
}
