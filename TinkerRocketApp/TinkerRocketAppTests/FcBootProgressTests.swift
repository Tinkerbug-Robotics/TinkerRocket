import XCTest
@testable import TinkerRocketApp

/// FC boot progress ("bs"/"bt"/"bd").  The FC is silent for the ~10 s of
/// setup_fc(), so the OC's zeroed rocket_state used to render "rail off",
/// "still booting" and "genuinely initializing" as one indistinguishable
/// INITIALIZATION.  These keys ride only the boot window — absent is the
/// normal case on every frame from a running FC, and must never be an error.
final class FcBootProgressTests: XCTestCase {

    private func decode(_ json: String) throws -> TelemetryData {
        try JSONDecoder().decode(TelemetryData.self, from: json.data(using: .utf8)!)
    }

    // MARK: - Decode

    func testJSONDecode_AllThreeBootKeys() throws {
        let t = try decode(#"{"st":"INITIALIZATION","bs":2,"bt":4200,"bd":2}"#)
        XCTAssertEqual(t.fc_boot_step, 2)
        XCTAssertEqual(t.fc_boot_ms, 4200)
        XCTAssertEqual(t.fc_boot_degraded, 2)

        let boot = try XCTUnwrap(t.fcBootProgress)
        XCTAssertEqual(boot.knownStep, .sensors)
        XCTAssertEqual(boot.elapsedMs, 4200)
        XCTAssertEqual(boot.degradedSubsystems, ["GNSS"])
    }

    func testJSONDecode_DegradedKeyAbsent() throws {
        // "bd" is omitted entirely on a normal boot — the common case, and the
        // one that must not read as "degraded bits unknown".
        let t = try decode(#"{"st":"INITIALIZATION","bs":4,"bt":9000}"#)
        XCTAssertEqual(t.fc_boot_degraded, 0)
        let boot = try XCTUnwrap(t.fcBootProgress)
        XCTAssertTrue(boot.degradedSubsystems.isEmpty)
        XCTAssertEqual(boot.line(), "Starting servos")
    }

    func testJSONDecode_AllBootKeysAbsent() throws {
        // Every frame from a running FC. No progress to show, no error.
        let t = try decode(#"{"st":"PRELAUNCH","nsat":9}"#)
        XCTAssertNil(t.fc_boot_step)
        XCTAssertNil(t.fc_boot_ms)
        XCTAssertEqual(t.fc_boot_degraded, 0)
        XCTAssertNil(t.fcBootProgress, "no \"bs\" = the FC is past boot (or silent)")
        XCTAssertEqual(t.state, "PRELAUNCH", "frame must decode as a whole")
    }

    func testJSONDecode_BootKeysTolerateFloatAndString() throws {
        // Same lenient-optional convention as hch/nidd (#571): one off-type key
        // must degrade that key, never throw away the whole frame.
        let t = try decode(#"{"st":"INITIALIZATION","bs":"3","bt":7000.0,"bd":"4"}"#)
        XCTAssertEqual(t.fc_boot_step, 3)
        XCTAssertEqual(t.fc_boot_ms, 7000)
        XCTAssertEqual(t.fc_boot_degraded, 4)
        XCTAssertEqual(t.state, "INITIALIZATION")
    }

    // MARK: - Step mapping

    func testEveryStepHasItsLabel() {
        let expected: [TelemetryData.FcBootStep: String] = [
            .links:    "Linking to flight computer",
            .nvs:      "Loading saved settings",
            .sensors:  "Starting sensors",
            .gnss:     "Starting GNSS",
            .servos:   "Starting servos",
            .complete: "Flight computer ready",
        ]
        for (step, label) in expected {
            XCTAssertEqual(step.label, label)
            XCTAssertEqual(TelemetryData.FcBootProgress(step: step.rawValue,
                                                        elapsedMs: 0,
                                                        degradedBits: 0).line(),
                           label)
        }
    }

    func testUnknownStepRendersGenerically() throws {
        // FcBootStep is append-only: a newer FC will report steps this build has
        // never heard of, and the line must still say something useful.
        let t = try decode(#"{"st":"INITIALIZATION","bs":99,"bt":1200}"#)
        let boot = try XCTUnwrap(t.fcBootProgress)
        XCTAssertNil(boot.knownStep)
        XCTAssertEqual(boot.step, 99, "the raw wire value is kept")
        XCTAssertEqual(boot.line(), "Starting up…")
    }

    // MARK: - Degraded bits

    func testDegradedBitsNameTheirSubsystem() {
        func names(_ bits: Int) -> [String] {
            TelemetryData.FcBootProgress(step: 5, elapsedMs: 0, degradedBits: bits)
                .degradedSubsystems
        }
        XCTAssertEqual(names(0x01), ["sensors"])
        XCTAssertEqual(names(0x02), ["GNSS"])
        XCTAssertEqual(names(0x04), ["servos"])
        XCTAssertEqual(names(0x08), ["saved settings"])
        XCTAssertEqual(names(0x0F), ["sensors", "GNSS", "servos", "saved settings"])
    }

    func testDegradedIsSurfacedOnTheLine() {
        // Boot CONTINUES past a degraded step, so the line is the only place the
        // operator learns the receiver never came up.
        let boot = TelemetryData.FcBootProgress(step: 5, elapsedMs: 9000, degradedBits: 0x02)
        XCTAssertEqual(boot.line(), "Flight computer ready — degraded: GNSS")
    }

    // MARK: - Stall

    func testFreshStepIsNotStalled() {
        let boot = TelemetryData.FcBootProgress(step: 2, elapsedMs: 2000, degradedBits: 0)
        XCTAssertFalse(boot.isStalled(dwell: 3))
        XCTAssertEqual(boot.line(dwell: 3), "Starting sensors")
    }

    func testStalledSensorStepBlamesGnss() {
        // sensor_collector.begin() blocks to its ~35 s deadline when the module
        // is dead, so a boot parked on FCB_SENSORS IS the dead-GNSS stall.
        let boot = TelemetryData.FcBootProgress(step: 2, elapsedMs: 2000, degradedBits: 0)
        XCTAssertTrue(boot.isStalled(dwell: 50))
        XCTAssertEqual(boot.line(dwell: 50),
                       "Starting sensors — no progress for 50s, GNSS may be dead")
    }

    /// Regression, measured on the bench 2026-08-19: a HEALTHY board sits on
    /// FCB_SENSORS for ~21 s, because GNSS bootstrap runs inside
    /// sensor_collector.begin() (it walks 460800 → swapped RX/TX → 9600 →
    /// 38400 before settling, then reads OTP).  This shipped with a single 12 s
    /// threshold and so cried "GNSS may be dead" on every normal boot — the
    /// fastest way to teach an operator to ignore the warning.  Pin the real
    /// measured duration as explicitly NOT a stall.
    func testMeasuredHealthySensorDurationIsNotAStall() {
        let boot = TelemetryData.FcBootProgress(step: 2, elapsedMs: 199, degradedBits: 0)
        XCTAssertFalse(boot.isStalled(dwell: 21))
        XCTAssertEqual(boot.line(dwell: 21), "Starting sensors")
        // The servo step's own measured 4.2 s must not trip its budget either.
        let servos = TelemetryData.FcBootProgress(step: 4, elapsedMs: 21156, degradedBits: 0)
        XCTAssertFalse(servos.isStalled(dwell: 5))
    }

    func testStalledOtherStepSaysSoWithoutBlame() {
        let boot = TelemetryData.FcBootProgress(step: 4, elapsedMs: 5000, degradedBits: 0)
        XCTAssertEqual(boot.line(dwell: 20), "Starting servos — no progress for 20s")
    }

    func testCompleteStepNeverStalls() {
        // FCB_COMPLETE means setup_fc() finished; the app is only waiting for the
        // first real NonSensorData, which is not the FC's fault.
        let boot = TelemetryData.FcBootProgress(step: 5, elapsedMs: 10000, degradedBits: 0)
        XCTAssertFalse(boot.isStalled(dwell: 60))
        XCTAssertEqual(boot.line(dwell: 60), "Flight computer ready")
    }

    // MARK: - The line under the state label

    func testNoBootKeysWhileInitializationReadsAsWaiting() {
        // Absence of "bs" with the state still at INITIALIZATION means the FC has
        // not spoken at all — the OC's zeroed rocket_state, not a rocket that is
        // initializing.
        XCTAssertEqual(RocketStateView(state: "INITIALIZATION").bootLine,
                       "Waiting for flight computer…")
    }

    func testRunningStatesGetNoSecondaryLine() {
        for s in ["READY", "PRELAUNCH", "INFLIGHT", "COMPLETE", "LANDED"] {
            XCTAssertNil(RocketStateView(state: s).bootLine,
                         "\(s) is a live state — nothing to qualify")
        }
    }
}
