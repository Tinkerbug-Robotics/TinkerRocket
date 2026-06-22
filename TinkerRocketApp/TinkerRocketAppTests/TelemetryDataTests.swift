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

    // MARK: - Sensor health scorecard (#303)

    /// Per-item state, matching firmware SensorHealthState (NA/OK/DEGRADED/BAD).
    private enum SH: Int { case na = 0, ok = 1, deg = 2, bad = 3 }

    /// Pack a "h" bitfield from per-item states.  Shifts mirror
    /// RocketComputerTypes.h SH_*_SHIFT (baro 0, imu 2, ekf 4, mag 6, gnss 8,
    /// batt 10, pyro1..4 at 12/14/16/18).
    private func pack(baro: SH = .na, imu: SH = .na, ekf: SH = .na, mag: SH = .na,
                      gnss: SH = .na, batt: SH = .na,
                      p1: SH = .na, p2: SH = .na, p3: SH = .na, p4: SH = .na) -> Int {
        baro.rawValue | (imu.rawValue << 2) | (ekf.rawValue << 4) | (mag.rawValue << 6)
            | (gnss.rawValue << 8) | (batt.rawValue << 10)
            | (p1.rawValue << 12) | (p2.rawValue << 14) | (p3.rawValue << 16) | (p4.rawValue << 18)
    }
    private func decode(health: Int) throws -> TelemetryData {
        try JSONDecoder().decode(TelemetryData.self,
                                 from: "{\"h\":\(health)}".data(using: .utf8)!)
    }

    /// Raw-int bit positions, pinned independently of `pack` so a systematic
    /// shift error can't hide behind a matching bug in the test helper.
    func testSensorHealth_BitPositions() throws {
        XCTAssertEqual(try decode(health: 1 << 0).baroHealth, .ok)       // baro
        XCTAssertEqual(try decode(health: 1 << 2).imuHealth, .ok)        // imu
        XCTAssertEqual(try decode(health: 1 << 4).ekfHealth, .ok)        // ekf
        XCTAssertEqual(try decode(health: 1 << 6).magHealth, .ok)        // mag
        XCTAssertEqual(try decode(health: 1 << 8).gnssHealth, .ok)       // gnss
        XCTAssertEqual(try decode(health: 1 << 10).battHealth, .ok)      // battery
        XCTAssertEqual(try decode(health: 1 << 18).pyroHealth(channel: 4), .ok)  // pyro4
        // A single field set leaves everything else N/A (no bleed).
        let onlyGnss = try decode(health: 1 << 8)
        XCTAssertEqual(onlyGnss.baroHealth, .na)
        XCTAssertEqual(onlyGnss.pyroHealth(channel: 1), .na)
        // BAD encodes as 0b11 in-place (verifies the 2-bit mask).
        XCTAssertEqual(try decode(health: 3 << 4).ekfHealth, .bad)
    }

    /// Missing "h" (older firmware / BS self-frame) → no scorecard, unknown.
    func testSensorHealth_MissingKey_Unknown() throws {
        let t = try JSONDecoder().decode(TelemetryData.self, from: "{}".data(using: .utf8)!)
        XCTAssertEqual(t.sensor_health, 0)
        XCTAssertFalse(t.hasSensorHealth)
        XCTAssertEqual(t.flightReadiness, .unknown)
    }

    /// Everything OK incl. EKF + GNSS, no pyros configured → green.
    func testReadiness_AllOK_Ready() throws {
        let t = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, mag: .ok, gnss: .ok, batt: .ok))
        XCTAssertTrue(t.hasSensorHealth)
        XCTAssertEqual(t.flightReadiness, .ready)
        XCTAssertEqual(t.sensorHealthRows.count, 6)            // 6 core, no pyros
    }

    /// Hard faults waiting won't fix → red "do not fly".
    func testReadiness_HardFaults_NotReady() throws {
        let baro = try decode(health: pack(baro: .bad, imu: .ok, ekf: .ok, gnss: .ok, batt: .ok))
        XCTAssertEqual(baro.flightReadiness, .notReady)
        let batt = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .ok, batt: .bad))
        XCTAssertEqual(batt.flightReadiness, .notReady)
        // A configured pyro with no continuity is a hard fault.
        let pyro = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .ok, batt: .ok, p1: .bad))
        XCTAssertEqual(pyro.pyroHealth(channel: 1), .bad)
        XCTAssertEqual(pyro.flightReadiness, .notReady)
    }

    /// EKF not converged or GNSS without a fix gate green down to amber, but
    /// aren't hard faults (you wait them out).
    func testReadiness_EkfOrGnssNotOK_Caution() throws {
        let ekf = try decode(health: pack(baro: .ok, imu: .ok, ekf: .deg, gnss: .ok, batt: .ok))
        XCTAssertEqual(ekf.flightReadiness, .caution)
        let gnss = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .bad, batt: .ok))
        XCTAssertEqual(gnss.flightReadiness, .caution)        // no fix → amber, not red
        // A configured-but-untested pyro is amber too.
        let pyro = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .ok, batt: .ok, p1: .deg))
        XCTAssertEqual(pyro.flightReadiness, .caution)
    }

    /// Mag is advisory — BAD mag with everything else OK still recommends fly.
    func testReadiness_MagAdvisory_DoesNotGate() throws {
        let t = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, mag: .bad, gnss: .ok, batt: .ok))
        XCTAssertEqual(t.magHealth, .bad)
        XCTAssertEqual(t.flightReadiness, .ready)
    }

    /// Unconfigured pyros (N/A) are ignored; configured ones add a card row.
    func testSensorHealthRows_OnlyConfiguredPyros() throws {
        let none = try decode(health: pack(baro: .ok, imu: .ok))
        XCTAssertFalse(none.sensorHealthRows.contains { $0.name.hasPrefix("Pyro") })
        let two = try decode(health: pack(baro: .ok, imu: .ok, p1: .ok, p3: .bad))
        XCTAssertEqual(two.sensorHealthRows.count, 6 + 2)
        XCTAssertTrue(two.sensorHealthRows.contains { $0.name == "Pyro 1" })
        XCTAssertTrue(two.sensorHealthRows.contains { $0.name == "Pyro 3" })
        XCTAssertFalse(two.sensorHealthRows.contains { $0.name == "Pyro 2" })
    }
}
