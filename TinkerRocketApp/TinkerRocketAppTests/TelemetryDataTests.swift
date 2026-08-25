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

    func testJSONDecode_IntFieldsTolerateFloatAndString() throws {
        // #571: completes the #293 hardening — EVERY integer key must survive
        // firmware-contract drift emitting it as a float or string. A strict
        // decode threw inside init(from:) and the caller's catch discarded the
        // WHOLE frame, so one off-type field (e.g. "rid" on a relayed frame)
        // made the rocket vanish from the BS dashboard instead of degrading a
        // single field.
        let json = """
        {"st":"INFLIGHT","nsat":7.0,"rid":"3","frx":"1000","fdr":2.0,
         "hch":"5","nidd":4.0,"h":"9","slrm":"120","ds":1.0}
        """.data(using: .utf8)!

        let t = try JSONDecoder().decode(TelemetryData.self, from: json)
        XCTAssertEqual(t.num_sats, 7)
        XCTAssertEqual(t.source_rocket_id, 3, "rid is the multi-rocket demux key")
        XCTAssertEqual(t.frames_rx, 1000)
        XCTAssertEqual(t.frames_drop, 2)
        XCTAssertEqual(t.hop_channel, 5)
        XCTAssertEqual(t.netid_drops, 4)
        XCTAssertEqual(t.sensor_health, 9)
        XCTAssertEqual(t.bs_log_silence_remaining_s, 120)
        XCTAssertEqual(t.data_status, .stale)
        XCTAssertEqual(t.state, "INFLIGHT", "frame must decode as a whole")
    }

    func testJSONDecode_HopStateKeys() throws {
        // #150: "hch" (current hop channel) and "nidd" (network-id
        // mismatch drops) ride the droppable tail of the telemetry JSON —
        // present only while hopping / after drops, so both decode paths
        // (present and absent) matter.
        let with = """
        {"st":"PRELAUNCH","hch":42,"nidd":7}
        """.data(using: .utf8)!
        let t1 = try JSONDecoder().decode(TelemetryData.self, from: with)
        XCTAssertEqual(t1.hop_channel, 42)
        XCTAssertEqual(t1.netid_drops, 7)

        let without = """
        {"st":"PRELAUNCH"}
        """.data(using: .utf8)!
        let t2 = try JSONDecoder().decode(TelemetryData.self, from: without)
        XCTAssertNil(t2.hop_channel, "fixed-mode frames omit hch entirely")
        XCTAssertNil(t2.netid_drops, "healthy-nid frames omit nidd entirely")
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
    /// batt 10, pyro1..4 at 12/14/16/18, storage 20).
    private func pack(baro: SH = .na, imu: SH = .na, ekf: SH = .na, mag: SH = .na,
                      gnss: SH = .na, batt: SH = .na, storage: SH = .na,
                      p1: SH = .na, p2: SH = .na, p3: SH = .na, p4: SH = .na) -> Int {
        baro.rawValue | (imu.rawValue << 2) | (ekf.rawValue << 4) | (mag.rawValue << 6)
            | (gnss.rawValue << 8) | (batt.rawValue << 10) | (storage.rawValue << 20)
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
        XCTAssertEqual(try decode(health: 1 << 20).storageHealth, .ok)   // storage (#281/#278)
        // A single field set leaves everything else N/A (no bleed).
        let onlyGnss = try decode(health: 1 << 8)
        XCTAssertEqual(onlyGnss.baroHealth, .na)
        XCTAssertEqual(onlyGnss.pyroHealth(channel: 1), .na)
        // BAD encodes as 0b11 in-place (verifies the 2-bit mask).
        XCTAssertEqual(try decode(health: 3 << 4).ekfHealth, .bad)
        // #557: GNSS-absent degraded-flight flag at shift 22 (SH_BAD = active),
        // distinct from the gnss fix-health item at shift 8.
        XCTAssertTrue(try decode(health: 3 << 22).gnssAbsentMode)
        XCTAssertFalse(try decode(health: 1 << 8).gnssAbsentMode)   // fix health ≠ absent mode
        XCTAssertFalse(try decode(health: 0).gnssAbsentMode)
    }

    // MARK: - Measured pyro continuity (SH_PYRO_MEAS_SHIFT, bits 24-30)

    /// The measured bits are UNGATED by pyro config, unlike pyroHealth — that
    /// asymmetry is the whole reason they exist (bench 2026-08-17: a ground
    /// test on a channel not armed for flight read CONT over direct BLE and
    /// "no reading" over LoRa, because only the LoRa carrier was config-gated).
    func testPyroMeasuredContinuity_BitPositionsAndStates() throws {
        // OK on ch1 (shift 24), BAD on ch4 (shift 30).
        XCTAssertEqual(try decode(health: 1 << 24).pyroMeasuredContinuity(channel: 1), .ok)
        XCTAssertEqual(try decode(health: 3 << 30).pyroMeasuredContinuity(channel: 4), .bad)
        // Reporting one channel leaves the others "not tested" (.na), NOT open.
        let onlyCh1 = try decode(health: 1 << 24)
        XCTAssertEqual(onlyCh1.pyroMeasuredContinuity(channel: 2), .na)
        // The measured field must not disturb the config-gated go/no-go bits:
        // a measured-open channel is NOT a "Do not fly" on its own.
        XCTAssertEqual(onlyCh1.pyroHealth(channel: 1), .na)
        XCTAssertNotEqual(try decode(health: 3 << 30).flightReadiness, .notReady)
    }

    /// Channel 4's measured bits are 30-31, and the firmware emits "h" through
    /// addInt — `snprintf("%d", (int)sensor_health)` — so a ch4 measured-open
    /// sets bit 31 and the JSON carries a NEGATIVE number. Swift decodes that
    /// into a negative Int and shifts it arithmetically; only the 2-bit mask
    /// keeps every channel correct. Pin it, because the day this silently
    /// breaks is the day a fired ch4 reports "not tested".
    func testPyroMeasuredContinuity_Channel4_NegativeWireValue() throws {
        // ch4 = BAD (0b11 << 30) = 0xC000_0000 -> int32 -1073741824
        let ch4Open = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"h":-1073741824}"#.data(using: .utf8)!)
        XCTAssertEqual(ch4Open.pyroMeasuredContinuity(channel: 4), .bad)
        XCTAssertEqual(ch4Open.pyroMeasuredContinuity(channel: 1), .na)

        // ch1 = OK (0b01 << 24) alongside ch4 = BAD -> 0xC100_0000 = -1056964608.
        // The sign bit must not bleed into the lower channel's 2 bits.
        let both = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"h":-1056964608}"#.data(using: .utf8)!)
        XCTAssertEqual(both.pyroMeasuredContinuity(channel: 1), .ok)
        XCTAssertEqual(both.pyroMeasuredContinuity(channel: 4), .bad)
        XCTAssertEqual(both.baroHealth, .na)      // low items unaffected
    }

    /// All-four-NA means the rocket predates the field: callers must fall back
    /// to pyroHealth rather than render "never tested" forever.
    func testPyroMeasuredContinuity_AbsentOnOlderFirmware_ReturnsNil() throws {
        // Config-gated pyro bits present, measured bits untouched.
        let legacy = try decode(health: 1 << 12)
        XCTAssertNil(legacy.pyroMeasuredContinuity(channel: 1))
        XCTAssertEqual(legacy.pyroHealth(channel: 1), .ok)
        // Out-of-range channels are nil even when the field is present.
        XCTAssertNil(try decode(health: 1 << 24).pyroMeasuredContinuity(channel: 5))
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

    /// Flight-log storage (#281/#278): BAD (full/failing NAND) is a hard fault —
    /// the silent log-loss that motivated the verdict must read "do not fly".
    /// DEGRADED (low space) gates green down to amber; N/A (older firmware) is
    /// ignored so it can't strand the go/no-go.  A reported state adds a card row.
    func testReadiness_Storage() throws {
        let full = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .ok, batt: .ok, storage: .bad))
        XCTAssertEqual(full.storageHealth, .bad)
        XCTAssertEqual(full.flightReadiness, .notReady)
        let low = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, gnss: .ok, batt: .ok, storage: .deg))
        XCTAssertEqual(low.flightReadiness, .caution)
        let ok = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, mag: .ok, gnss: .ok, batt: .ok, storage: .ok))
        XCTAssertEqual(ok.flightReadiness, .ready)
        XCTAssertEqual(ok.sensorHealthRows.count, 7)         // 6 core + Storage
        XCTAssertTrue(ok.sensorHealthRows.contains { $0.name == "Storage" })
        // N/A storage (older firmware / BS frame) is ignored: no row, still green.
        let naStor = try decode(health: pack(baro: .ok, imu: .ok, ekf: .ok, mag: .ok, gnss: .ok, batt: .ok))
        XCTAssertEqual(naStor.storageHealth, .na)
        XCTAssertEqual(naStor.flightReadiness, .ready)
        XCTAssertFalse(naStor.sensorHealthRows.contains { $0.name == "Storage" })
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

    // MARK: - MTU trim flag (#282)

    /// The base station sets "tr":1 when a worst-case in-flight frame was
    /// trimmed to fit the BLE MTU window.  Absent "tr" must read as not-trimmed
    /// (older firmware never emits it); a present "tr":1 flips the flag without
    /// disturbing the fields that did make it through.
    func testTrimFlag_DecodesAndDefaultsFalse() throws {
        let plain = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"st":"INFLIGHT"}"#.data(using: .utf8)!)
        XCTAssertFalse(plain.fields_trimmed)

        let trimmed = try JSONDecoder().decode(
            TelemetryData.self, from: #"{"st":"INFLIGHT","fs":7,"tr":1}"#.data(using: .utf8)!)
        XCTAssertTrue(trimmed.fields_trimmed)
        XCTAssertEqual(trimmed.state, "INFLIGHT")
        XCTAssertTrue(trimmed.launch_flag)   // tail trim must not corrupt earlier fields
    }

    // MARK: - Relayed IMU orientation (#390: "imo" = (mode << 5) | code)

    private func decodeIMO(_ imo: Int?) throws -> TelemetryData {
        let body = imo.map { "\"imo\": \($0)," } ?? ""
        let json = "{ \(body) \"st\": \"PRELAUNCH\" }".data(using: .utf8)!
        return try JSONDecoder().decode(TelemetryData.self, from: json)
    }

    func testRelayedOrientation_AutoSnapCode() throws {
        // mode 3 (auto), code 21 = -Z r90: (3 << 5) | 21 = 117
        let t = try decodeIMO(117)
        XCTAssertEqual(t.relayedOrientationMode, .autoSnap)
        XCTAssertEqual(t.relayedOrientationName, "-Z r90")
    }

    func testRelayedOrientation_ManualAndDefault() throws {
        let manual = try decodeIMO((2 << 5) | 0)     // manual, +X
        XCTAssertEqual(manual.relayedOrientationMode, .manual)
        XCTAssertEqual(manual.relayedOrientationName, "+X")

        let def = try decodeIMO((1 << 5) | 0)        // default mounting
        XCTAssertEqual(def.relayedOrientationMode, .defaultMounting)
        XCTAssertEqual(def.relayedOrientationName, "+X")
    }

    func testRelayedOrientation_AutoExactSentinel() throws {
        // Code 31 = auto-exact, no discrete code — must still produce a
        // non-empty name (the IMU card's line gates on it), not "?" or "".
        let t = try decodeIMO((3 << 5) | 31)
        XCTAssertEqual(t.relayedOrientationMode, .autoExact)
        XCTAssertEqual(t.relayedOrientationName, "custom")
    }

    func testRelayedOrientation_AbsentOrLegacyIsUnknown() throws {
        // Missing key (pre-#390 rocket firmware / trimmed tail) → unknown,
        // empty name → the BS view renders no orientation line at all.
        let absent = try decodeIMO(nil)
        XCTAssertEqual(absent.relayedOrientationMode, .unknown)
        XCTAssertEqual(absent.relayedOrientationName, "")

        // Wire mode 0 should never be emitted, but a zero must not render
        // as a confident "+X default".
        let zero = try decodeIMO(0)
        XCTAssertEqual(zero.relayedOrientationMode, .unknown)
        XCTAssertEqual(zero.relayedOrientationName, "")
    }

    func testSocDisplayNeverShowsANegativePercent() {
        // A rocket on USB sits below the 2S curve, so the OC clamps SOC to 0
        // before packing. The i16 spans -25...125% for headroom, so an exact
        // 0% comes back as -0.00077 and printed as "-0.0%" on the dashboard.
        func display(_ soc: Float?) -> String {
            var t = TelemetryData()
            t.soc = soc
            return t.socDisplay
        }

        // The one that actually shipped: the OC prints SOC to one decimal,
        // so an exact 0% arrives as the literal -0.0, which is == 0 and so
        // survives any clamp untouched.
        XCTAssertEqual(display(-0.0), "0.0%")
        XCTAssertEqual(display(-0.00077), "0.0%")
        XCTAssertEqual(display(-25), "0.0%")
        XCTAssertEqual(display(125), "100.0%")
        // In-range values are untouched, and absent stays absent.
        XCTAssertEqual(display(42.5), "42.5%")
        XCTAssertEqual(display(nil), "N/A")
    }
}

// MARK: - #850 rail currents (camera / servo high-side switches)

extension TelemetryDataTests {

    /// An absent key means the board has NO monitor fitted (V7/V8, mini, or
    /// pre-#850 firmware). That is not the same fact as "measured 0.0 A", and
    /// the difference is the whole point: a camera that is off and a camera
    /// with no monitor both sit at zero, but only one of them is a reading.
    func testRailCurrents_AbsentKeysDecodeAsNilNotZero() throws {
        let json = #"{"soc": 85.0, "vol": 7.4, "cur": -450.0}"#
        let t = try JSONDecoder().decode(TelemetryData.self, from: Data(json.utf8))

        XCTAssertNil(t.cam_current)
        XCTAssertNil(t.servo_current)
        XCTAssertEqual(t.camCurrentDisplay, "—")
        XCTAssertEqual(t.servoCurrentDisplay, "—")
    }

    func testRailCurrents_PresentKeysDecode() throws {
        let json = #"{"soc": 85.0, "ccur": 1.48, "scur": 0.34}"#
        let t = try JSONDecoder().decode(TelemetryData.self, from: Data(json.utf8))

        XCTAssertEqual(t.cam_current ?? 0, 1.48, accuracy: 0.001)
        XCTAssertEqual(t.servo_current ?? 0, 0.34, accuracy: 0.001)
    }

    /// A genuine measured zero is distinct from absent and must render as a
    /// number, not an em dash.
    func testRailCurrents_MeasuredZeroIsNotAbsent() throws {
        let json = #"{"ccur": 0.0}"#
        let t = try JSONDecoder().decode(TelemetryData.self, from: Data(json.utf8))

        XCTAssertNotNil(t.cam_current)
        XCTAssertEqual(t.camCurrentDisplay, "0 mA")
        XCTAssertNil(t.servo_current)
        XCTAssertEqual(t.servoCurrentDisplay, "—")
    }

    /// Amps at and above 1 A, milliamps below — matching the Android twin.
    func testRailCurrents_DisplayCrossesOverAtOneAmp() {
        XCTAssertEqual(TelemetryData.railAmpsDisplay(1.48), "1.48 A")
        XCTAssertEqual(TelemetryData.railAmpsDisplay(1.0), "1.00 A")
        XCTAssertEqual(TelemetryData.railAmpsDisplay(0.999), "999 mA")
        XCTAssertEqual(TelemetryData.railAmpsDisplay(0.34), "340 mA")
        XCTAssertEqual(TelemetryData.railAmpsDisplay(nil), "—")
    }
}
