import XCTest
@testable import TinkerRocketApp

/// Conversions are display-layer only (#160); these lock down the math and
/// the unit labels for both systems so a future refactor can't silently
/// change what the user sees.
final class UnitFormatterTests: XCTestCase {

    // MARK: - Altitude (never promotes to miles)

    func testAltitudeMetric() {
        XCTAssertEqual(UnitFormatter.altitude(500, system: .metric), "500 m")
        XCTAssertEqual(UnitFormatter.altitude(1000, system: .metric), "1.00 km")
        XCTAssertEqual(UnitFormatter.altitude(2540, system: .metric), "2.54 km")
    }

    func testAltitudeImperial() {
        // 500 m * 3.28084 = 1640.4 ft
        XCTAssertEqual(UnitFormatter.altitude(500, system: .imperial), "1640 ft")
        // 3000 m = 9842 ft — stays in feet, never miles
        XCTAssertEqual(UnitFormatter.altitude(3000, system: .imperial), "9843 ft")
    }

    // MARK: - Distance (promotes to km / mi)

    func testDistanceMetric() {
        XCTAssertEqual(UnitFormatter.distance(500, system: .metric), "500 m")
        XCTAssertEqual(UnitFormatter.distance(2000, system: .metric), "2.0 km")
    }

    func testDistanceImperial() {
        // 500 m = 1640 ft, below the 5280 ft mile threshold
        XCTAssertEqual(UnitFormatter.distance(500, system: .imperial), "1640 ft")
        // 2000 m = 6561.7 ft → 1.24 mi
        XCTAssertEqual(UnitFormatter.distance(2000, system: .imperial), "1.24 mi")
    }

    // MARK: - Speed

    func testSpeed() {
        XCTAssertEqual(UnitFormatter.speed(10, decimals: 1, system: .metric), "10.0 m/s")
        // 10 m/s * 3.28084 = 32.8 ft/s
        XCTAssertEqual(UnitFormatter.speed(10, decimals: 1, system: .imperial), "32.8 ft/s")
        XCTAssertEqual(UnitFormatter.speed(100, decimals: 0, system: .imperial), "328 ft/s")
    }

    // MARK: - Acceleration (g in imperial)

    func testAcceleration() {
        XCTAssertEqual(UnitFormatter.acceleration(9.80665, decimals: 2, system: .metric), "9.81 m/s\u{00B2}")
        XCTAssertEqual(UnitFormatter.acceleration(9.80665, decimals: 2, system: .imperial), "1.00 g")
        XCTAssertEqual(UnitFormatter.acceleration(98.0665, decimals: 1, system: .imperial), "10.0 g")
    }

    func testAccelerationUnitAndValue() {
        XCTAssertEqual(UnitFormatter.accelerationUnit(.metric), "m/s\u{00B2}")
        XCTAssertEqual(UnitFormatter.accelerationUnit(.imperial), "g")
        XCTAssertEqual(UnitFormatter.accelerationValue(9.80665, system: .metric), 9.80665, accuracy: 1e-6)
        XCTAssertEqual(UnitFormatter.accelerationValue(9.80665, system: .imperial), 1.0, accuracy: 1e-6)
    }

    // MARK: - Temperature

    func testTemperature() {
        XCTAssertEqual(UnitFormatter.temperature(0, decimals: 1, system: .metric), "0.0 \u{00B0}C")
        XCTAssertEqual(UnitFormatter.temperature(0, decimals: 1, system: .imperial), "32.0 \u{00B0}F")
        XCTAssertEqual(UnitFormatter.temperature(100, decimals: 1, system: .imperial), "212.0 \u{00B0}F")
    }

    // MARK: - Spoken (voice)

    func testSpoken() {
        XCTAssertEqual(UnitFormatter.spokenAltitude(1000, system: .metric), "1000 meters")
        XCTAssertEqual(UnitFormatter.spokenAltitude(1000, system: .imperial), "3281 feet")
        XCTAssertEqual(UnitFormatter.spokenSpeed(10, system: .metric), "10 meters per second")
        XCTAssertEqual(UnitFormatter.spokenSpeed(10, system: .imperial), "33 feet per second")
        XCTAssertEqual(UnitFormatter.spokenDistance(300, system: .imperial), "984 feet")
    }

    // MARK: - Chart column conversion

    func testConvertSeriesMetricPassthrough() {
        let (label, values) = UnitFormatter.convertSeries(
            column: "Pressure Altitude (m)", values: [1, 2, 3], system: .metric)
        XCTAssertEqual(label, "Pressure Altitude (m)")
        XCTAssertEqual(values, [1, 2, 3])
    }

    func testConvertSeriesImperialLength() {
        let (label, values) = UnitFormatter.convertSeries(
            column: "Pressure Altitude (m)", values: [100], system: .imperial)
        XCTAssertEqual(label, "Pressure Altitude (ft)")
        XCTAssertEqual(values[0], 328.084, accuracy: 1e-2)
    }

    func testConvertSeriesImperialSpeed() {
        let (label, values) = UnitFormatter.convertSeries(
            column: "Velocity Up (m/s)", values: [10], system: .imperial)
        XCTAssertEqual(label, "Velocity Up (ft/s)")
        XCTAssertEqual(values[0], 32.8084, accuracy: 1e-3)
    }

    func testConvertSeriesImperialAccel() {
        let (label, values) = UnitFormatter.convertSeries(
            column: "Low-G Acceleration X (m/s2)", values: [9.80665], system: .imperial)
        XCTAssertEqual(label, "Low-G Acceleration X (g)")
        XCTAssertEqual(values[0], 1.0, accuracy: 1e-6)
    }

    func testConvertSeriesImperialTemperature() {
        let (label, values) = UnitFormatter.convertSeries(
            column: "Barometer Temperature (C)", values: [0, 100], system: .imperial)
        XCTAssertEqual(label, "Barometer Temperature (\u{00B0}F)")
        XCTAssertEqual(values[0], 32, accuracy: 1e-6)
        XCTAssertEqual(values[1], 212, accuracy: 1e-6)
    }

    func testConvertSeriesNonConvertibleColumns() {
        // Angles, percent, voltage etc. must pass through untouched in imperial.
        for col in ["Roll (deg)", "Gyro X (deg/s)", "State of Charge (%)",
                    "Voltage (V)", "Magnetic Field X (uT)", "Pressure (Pa)"] {
            let (label, values) = UnitFormatter.convertSeries(
                column: col, values: [42], system: .imperial)
            XCTAssertEqual(label, col, "\(col) should not be relabeled")
            XCTAssertEqual(values, [42], "\(col) values should be unchanged")
        }
    }

    // MARK: - UnitSystem persistence

    func testUnitSystemRawValues() {
        XCTAssertEqual(UnitSystem(rawValue: "metric"), .metric)
        XCTAssertEqual(UnitSystem(rawValue: "imperial"), .imperial)
        XCTAssertEqual(UnitSystem(rawValue: "garbage"), nil)
    }
}
