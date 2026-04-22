import XCTest
@testable import TinkerRocketApp

final class TelemetryDataTests: XCTestCase {

    // MARK: - JSON Decoding

    func testJSONDecode_NominalPayload() throws {
        // Uses the short keys defined in TelemetryData.CodingKeys
        // ("vol" for voltage, "st" for state, etc.) so the decoder wires
        // each field through decodeIfPresent.
        let json = """
        {
            "soc": 85.0,
            "vol": 3.85,
            "cur": 450.0,
            "lat": 33.7,
            "lon": -118.4,
            "st": "INFLIGHT",
            "nsat": 12,
            "gdop": 1.5,
            "q0": 0.707,
            "q1": 0.0,
            "q2": 0.0,
            "q3": 0.707
        }
        """.data(using: .utf8)!

        let telemetry = try JSONDecoder().decode(TelemetryData.self, from: json)

        let soc = try XCTUnwrap(telemetry.soc)
        let voltage = try XCTUnwrap(telemetry.voltage)
        let latitude = try XCTUnwrap(telemetry.latitude)
        XCTAssertEqual(soc, 85.0, accuracy: 0.1)
        XCTAssertEqual(voltage, 3.85, accuracy: 0.01)
        XCTAssertEqual(latitude, 33.7, accuracy: 1e-6)
        XCTAssertEqual(telemetry.state, "INFLIGHT")
    }

    func testJSONDecode_MissingOptionals_NoCrash() throws {
        let json = "{}".data(using: .utf8)!
        // Should not crash - optionals just nil, non-optionals fall back to
        // their defaults ("UNKNOWN", 0, false, …).
        let telemetry = try JSONDecoder().decode(TelemetryData.self, from: json)
        XCTAssertNil(telemetry.soc)
        XCTAssertNil(telemetry.latitude)
        XCTAssertEqual(telemetry.state, "UNKNOWN")
        XCTAssertEqual(telemetry.num_sats, 0)
    }

    // MARK: - Quaternion storage

    func testQuaternionToEuler_Identity() {
        // Identity quaternion: q=[1,0,0,0] -> roll/pitch/yaw = 0.
        // The roll/pitch/yaw computed properties reject near-zero norms so
        // we have to use a valid unit quaternion, not defaults.
        var telemetry = TelemetryData()
        telemetry.q0 = 1.0
        telemetry.q1 = 0.0
        telemetry.q2 = 0.0
        telemetry.q3 = 0.0

        XCTAssertEqual(try XCTUnwrap(telemetry.q0), 1.0, accuracy: 1e-6)
        XCTAssertEqual(try XCTUnwrap(telemetry.roll), 0.0, accuracy: 1e-3)
        XCTAssertEqual(try XCTUnwrap(telemetry.pitch), 0.0, accuracy: 1e-3)
        XCTAssertEqual(try XCTUnwrap(telemetry.yaw), 0.0, accuracy: 1e-3)
    }
}
