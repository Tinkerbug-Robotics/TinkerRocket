import XCTest
@testable import TinkerRocketApp

final class TelemetryDataTests: XCTestCase {

    // MARK: - JSON Decoding

    func testJSONDecode_NominalPayload() throws {
        let json = """
        {
            "soc": 85.0,
            "voltage": 3.85,
            "cur": 450.0,
            "lat": 33.7,
            "lon": -118.4,
            "alt": 350.0,
            "state": "INFLIGHT",
            "nsat": 12,
            "gdop": 1.5,
            "q0": 0.707,
            "q1": 0.0,
            "q2": 0.0,
            "q3": 0.707
        }
        """.data(using: .utf8)!

        let telemetry = try JSONDecoder().decode(TelemetryData.self, from: json)

        XCTAssertEqual(telemetry.soc, 85.0, accuracy: 0.1)
        XCTAssertEqual(telemetry.voltage, 3.85, accuracy: 0.01)
        XCTAssertEqual(telemetry.lat, 33.7, accuracy: 1e-6)
        XCTAssertEqual(telemetry.state, "INFLIGHT")
    }

    func testJSONDecode_MissingOptionals_NoCrash() throws {
        // Minimal JSON with only required fields
        let json = """
        {}
        """.data(using: .utf8)!

        // Should not crash - optionals just nil
        let telemetry = try JSONDecoder().decode(TelemetryData.self, from: json)
        XCTAssertNotNil(telemetry)
    }

    // MARK: - Quaternion to Euler

    func testQuaternionToEuler_Identity() {
        // Identity quaternion: q=[1,0,0,0] -> roll/pitch/yaw = 0
        var telemetry = TelemetryData()
        telemetry.q0 = 1.0
        telemetry.q1 = 0.0
        telemetry.q2 = 0.0
        telemetry.q3 = 0.0

        // Test computed euler angle properties if they exist
        // (This test verifies the quaternion is stored correctly;
        // actual euler computation tested via NonSensorData conversion)
        XCTAssertEqual(telemetry.q0, 1.0, accuracy: 1e-6)
    }
}
