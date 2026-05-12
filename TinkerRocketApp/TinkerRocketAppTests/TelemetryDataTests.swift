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

    // MARK: - Flight status bitfield (#138 MTU pack)

    /// "fs" packs 8 flags into one byte to keep the BLE notify under MTU.
    /// Bit layout must match TR_BLE_To_APP.cpp:buildTelemetryJSON.
    ///   b0=lnch b1=vapo b2=aapo b3=land
    ///   b4=pwr  b5=cam  b6=log  b7=bslog
    func testFlightStatusBits_AllBitsResolve() throws {
        // 0xFF — every flag should resolve true.
        let allSet = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"fs":255}"#.data(using: .utf8)!)
        XCTAssertTrue(allSet.launch_flag)
        XCTAssertTrue(allSet.vel_apo)
        XCTAssertTrue(allSet.alt_apo)
        XCTAssertTrue(allSet.landed_flag)
        XCTAssertTrue(allSet.pwr_pin_on)
        XCTAssertTrue(allSet.camera_recording)
        XCTAssertTrue(allSet.logging_active)
        XCTAssertTrue(allSet.bs_logging_active)

        // 0x55 = 0b0101_0101 — even bits: lnch, aapo, pwr, log.
        // Verifies each computed property reads from its assigned bit
        // position (not e.g. an off-by-one bit-shift).
        let evenSet = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"fs":85}"#.data(using: .utf8)!)
        XCTAssertTrue(evenSet.launch_flag)
        XCTAssertFalse(evenSet.vel_apo)
        XCTAssertTrue(evenSet.alt_apo)
        XCTAssertFalse(evenSet.landed_flag)
        XCTAssertTrue(evenSet.pwr_pin_on)
        XCTAssertFalse(evenSet.camera_recording)
        XCTAssertTrue(evenSet.logging_active)
        XCTAssertFalse(evenSet.bs_logging_active)
    }

    /// Missing "fs" key → all flags false (older firmware or BS-self frame
    /// with no flight context).  Must not crash.
    func testFlightStatusBits_MissingKey_AllFalse() throws {
        let json = "{}".data(using: .utf8)!
        let t = try JSONDecoder().decode(TelemetryData.self, from: json)
        XCTAssertEqual(t.flight_status_bits, 0)
        XCTAssertFalse(t.launch_flag)
        XCTAssertFalse(t.alt_apo)
        XCTAssertFalse(t.landed_flag)
        XCTAssertFalse(t.bs_logging_active)
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
