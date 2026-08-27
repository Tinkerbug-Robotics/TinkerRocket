import XCTest
@testable import TinkerRocketApp

/// The 15 flight-profile policy cases ported from Android's
/// `FlightAnnouncerTest` — enabled by the `AnnouncerSpeech` seam (parity
/// ledger, Phase 9).  A spy engine records utterances; an injected clock
/// drives the 5 s / 10 s cadence gates.  The existing dispatch tests (#138)
/// and wording tests (#235) stay as they were — this file covers the state
/// machine between them.
final class FlightAnnouncerPolicyTests: XCTestCase {

    final class SpySpeech: AnnouncerSpeech {
        var isBusy = false
        var sessionActive = true
        var lastError: String?
        var onStatusChange: (() -> Void)?
        var spoken: [(text: String, interrupt: Bool)] = []
        var stopCount = 0
        var activateCount = 0
        var deactivateCount = 0
        func speak(_ text: String, interrupt: Bool) { spoken.append((text, interrupt)) }
        func stop() { stopCount += 1 }
        func activate() { activateCount += 1 }
        func deactivate() { deactivateCount += 1 }
        var texts: [String] { spoken.map(\.text) }
    }

    final class Clock {
        var t = Date(timeIntervalSince1970: 1_000_000)
        func advance(_ s: TimeInterval) { t.addTimeInterval(s) }
    }

    private var spy = SpySpeech()
    private var clock = Clock()

    private func makeAnnouncer(enabled: Bool = true,
                               units: UnitSystem = .metric) -> FlightAnnouncer {
        spy = SpySpeech()
        clock = Clock()
        let c = clock
        let a = FlightAnnouncer(speech: spy, now: { c.t },
                                unitSystem: { units }, persistEnabled: false)
        if enabled {
            a.isEnabled = true
            spy.spoken.removeAll()   // drop the "Voice ready" confirmation
            spy.activateCount = 0
        }
        return a
    }

    private func frame(state: String = "INFLIGHT",
                       maxSpeed: Float? = nil,
                       palt: Float? = nil,
                       rate: Float? = nil,
                       maxAlt: Float? = nil,
                       fs: Int = 0,
                       lat: Double? = nil,
                       lon: Double? = nil,
                       stale: Bool = false) -> TelemetryData {
        var t = TelemetryData()
        t.state = state
        t.max_speed_mps = maxSpeed
        t.pressure_alt = palt
        t.altitude_rate = rate
        t.max_alt_m = maxAlt
        t.flight_status_bits = fs
        t.latitude = lat
        t.longitude = lon
        t.data_status = stale ? .stale : .live
        return t
    }

    private let APO = 0x04, LAUNCH = 0x01, LANDED_F = 0x08

    /// Drive a burnout: a baseline frame, a genuinely rising frame, then three
    /// stable ones.  The rise is load-bearing — a plateau with no observed
    /// climb is the "arrived mid-flight" case, which must NOT announce.
    private func driveBurnout(_ a: FlightAnnouncer, speed: Float = 80) {
        a.processTelemetry(frame(maxSpeed: speed - 20))
        a.processTelemetry(frame(maxSpeed: speed))
        for _ in 0..<3 { a.processTelemetry(frame(maxSpeed: speed)) }
    }

    // ── Burnout ──────────────────────────────────────────────────────────

    func testBurnout_announcedAfterThreeStableFrames_withMaxSpeed() {
        let a = makeAnnouncer()
        a.processTelemetry(frame(maxSpeed: 50))
        a.processTelemetry(frame(maxSpeed: 80))     // still rising — resets counter
        a.processTelemetry(frame(maxSpeed: 80))     // stable 1
        a.processTelemetry(frame(maxSpeed: 80))     // stable 2
        XCTAssertTrue(spy.spoken.isEmpty, "not yet — needs 3 stable frames")
        a.processTelemetry(frame(maxSpeed: 80))     // stable 3
        XCTAssertEqual(spy.texts, ["Burnout. Max speed 80 meters per second"])
        XCTAssertTrue(spy.spoken[0].interrupt, "burnout is an immediate callout")
    }

    func testBurnout_ignoredBelowMinimumSpeed() {
        let a = makeAnnouncer()
        for _ in 0..<10 { a.processTelemetry(frame(maxSpeed: 8)) }   // under 10 m/s floor
        XCTAssertTrue(spy.spoken.isEmpty)
    }

    func testBurnout_jitterWithinHalfMps_countsAsStable() {
        let a = makeAnnouncer()
        a.processTelemetry(frame(maxSpeed: 60.0))   // baseline
        a.processTelemetry(frame(maxSpeed: 80.0))   // genuine climb, observed
        a.processTelemetry(frame(maxSpeed: 80.3))   // +0.3 — jitter, counts stable
        a.processTelemetry(frame(maxSpeed: 80.1))
        a.processTelemetry(frame(maxSpeed: 80.2))
        XCTAssertEqual(spy.spoken.count, 1, "jitter under 0.5 m/s must not reset the counter")
    }

    /// The 2026-08-27 sim-flight report: voice switched on part-way through
    /// announced "Burnout" minutes late, just before landing.  `max_speed_mps`
    /// is a RUNNING MAXIMUM, so "stopped increasing" is permanently true once
    /// the motor is out — a detector that keys on the plateau alone fires for
    /// anyone who starts watching late, reading out a stale peak speed.
    func testBurnout_notAnnounced_whenObservationStartsAfterTheBurn() {
        let a = makeAnnouncer()
        // Every frame carries the same already-peaked running max — exactly
        // what the telemetry looks like on the way down under canopy.
        for _ in 0..<10 { a.processTelemetry(frame(maxSpeed: 120)) }
        XCTAssertTrue(spy.spoken.isEmpty,
                      "a plateau with no observed climb is a late arrival, not a burnout")
    }

    /// The case the fix must not break: enabling voice DURING the boost still
    /// produces a correct, timely callout, because the climb is then observed.
    func testBurnout_announced_whenVoiceEnabledDuringBoost() {
        let a = makeAnnouncer()
        a.processTelemetry(frame(maxSpeed: 40))     // baseline, still climbing
        a.processTelemetry(frame(maxSpeed: 70))     // rise observed
        a.processTelemetry(frame(maxSpeed: 95))     // still rising
        for _ in 0..<3 { a.processTelemetry(frame(maxSpeed: 95)) }
        XCTAssertEqual(spy.texts, ["Burnout. Max speed 95 meters per second"])
    }

    // ── Altitude cadence ─────────────────────────────────────────────────

    func testAltitude_notCalledOutBeforeBurnout() {
        let a = makeAnnouncer()
        a.processTelemetry(frame(palt: 120))
        clock.advance(6)
        a.processTelemetry(frame(palt: 150))
        XCTAssertTrue(spy.spoken.isEmpty, "no altitude callouts during powered flight")
    }

    func testAltitude_every5s_withClimbWordFromRateSign() {
        let a = makeAnnouncer()
        driveBurnout(a)
        spy.spoken.removeAll()

        clock.advance(5)
        a.processTelemetry(frame(palt: 300, rate: 40))
        XCTAssertEqual(spy.texts, ["300 meters, climbing 40 meters per second"])

        a.processTelemetry(frame(palt: 320, rate: 40))
        XCTAssertEqual(spy.spoken.count, 1, "inside the 5 s gate — no second callout")

        clock.advance(5)
        a.processTelemetry(frame(palt: 400, rate: 35))
        XCTAssertEqual(spy.texts.last, "400 meters, climbing 35 meters per second")
    }

    func testAltitude_nearLevelRate_omitsDirectionWord() {
        let a = makeAnnouncer()
        driveBurnout(a)
        spy.spoken.removeAll()
        clock.advance(5)
        a.processTelemetry(frame(palt: 450, rate: 0.4))   // inside the 1 m/s deadband
        XCTAssertEqual(spy.texts, ["450 meters"])
    }

    // ── Apogee ───────────────────────────────────────────────────────────

    func testApogee_announcedOnRisingEdge_withMaxAltitude() {
        let a = makeAnnouncer()
        a.processTelemetry(frame())
        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        XCTAssertEqual(spy.texts, ["Apogee. 455 meters"])
        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        XCTAssertEqual(spy.spoken.count, 1, "edge-triggered, not level-triggered")
    }

    func testApogee_withoutMaxAlt_saysAltitudeUnknown() {
        let a = makeAnnouncer()
        a.processTelemetry(frame())
        a.processTelemetry(frame(fs: APO))
        XCTAssertEqual(spy.texts, ["Apogee. altitude unknown"])
    }

    // ── Descent cadence ──────────────────────────────────────────────────

    func testDescent_firstCallout5sAfterApogee_then10sCadence() {
        let a = makeAnnouncer()
        a.processTelemetry(frame())
        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        spy.spoken.removeAll()

        // 3 s after apogee: inside the shortened first window — silent.
        clock.advance(3)
        a.processTelemetry(frame(palt: 420, rate: -5, fs: APO))
        XCTAssertTrue(spy.spoken.isEmpty)

        // 5 s after apogee: first descent callout.
        clock.advance(2)
        a.processTelemetry(frame(palt: 410, rate: -5, fs: APO))
        XCTAssertEqual(spy.texts, ["410 meters, descending 5 meters per second"])

        // Then the full 10 s cadence.
        clock.advance(5)
        a.processTelemetry(frame(palt: 380, rate: -5, fs: APO))
        XCTAssertEqual(spy.spoken.count, 1, "only 5 s since the last one")
        clock.advance(5)
        a.processTelemetry(frame(palt: 360, rate: -4, fs: APO))
        XCTAssertEqual(spy.texts.last, "360 meters, descending 4 meters per second")
    }

    // ── Landing ──────────────────────────────────────────────────────────

    func testLanded_flagEdge_speaksHaversineDistanceFromLaunchSite() {
        let a = makeAnnouncer()
        // Launch fix captured on the launch flag.
        a.processTelemetry(frame(fs: LAUNCH, lat: 40.0, lon: -74.0))
        // Landed 0.001° of latitude away ≈ 111 m.
        a.processTelemetry(frame(fs: LAUNCH | LANDED_F, lat: 40.001, lon: -74.0))
        XCTAssertEqual(spy.texts.last, "Landed. 111 meters away")
        // Edge, not level:
        let n = spy.spoken.count
        a.processTelemetry(frame(fs: LAUNCH | LANDED_F, lat: 40.001, lon: -74.0))
        XCTAssertEqual(spy.spoken.count, n)
    }

    func testLanded_stateFallback_whenFlagNeverArrives() {
        let a = makeAnnouncer()
        a.processTelemetry(frame())
        a.processTelemetry(frame(state: "LANDED"))
        XCTAssertEqual(spy.texts.last, "Landed. ")
    }

    // ── Gates ────────────────────────────────────────────────────────────

    func testStaleFrames_neverAnnounce_evenPastTheTimeGates() {
        let a = makeAnnouncer()
        driveBurnout(a)
        spy.spoken.removeAll()
        clock.advance(60)
        a.processTelemetry(frame(palt: 300, rate: 40, stale: true))
        a.processTelemetry(frame(maxAlt: 400, fs: APO, stale: true))
        XCTAssertTrue(spy.spoken.isEmpty, "STALE relay frames must never re-announce")
    }

    func testDisabled_tracksFramesSilently() {
        let a = makeAnnouncer(enabled: false)
        driveBurnout(a)
        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        XCTAssertTrue(spy.spoken.isEmpty)
        _ = a   // keep alive
    }

    func testPeriodicCallout_skippedWhileSpeaking_immediateInterrupts() {
        let a = makeAnnouncer()
        driveBurnout(a)
        spy.spoken.removeAll()

        spy.isBusy = true
        clock.advance(5)
        a.processTelemetry(frame(palt: 300, rate: 40))
        XCTAssertTrue(spy.spoken.isEmpty, "periodic callouts skip while the engine is busy")

        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        XCTAssertEqual(spy.texts, ["Apogee. 455 meters"], "critical callouts interrupt instead")
        XCTAssertTrue(spy.spoken[0].interrupt)
    }

    func testPrelaunchTransition_resetsOneShots_forTheNextFlight() {
        let a = makeAnnouncer()
        driveBurnout(a)
        a.processTelemetry(frame(maxAlt: 455, fs: APO))
        XCTAssertEqual(spy.spoken.count, 2)

        // Next flight: PRELAUNCH transition clears the one-shots.
        a.processTelemetry(frame(state: "PRELAUNCH"))
        spy.spoken.removeAll()
        driveBurnout(a, speed: 90)
        XCTAssertEqual(spy.texts, ["Burnout. Max speed 90 meters per second"],
                       "burnout announces again on the next flight")
    }

    func testSetEnabled_speaksReadyConfirmation_disableStopsSpeech() {
        let a = FlightAnnouncer(speech: spy, now: { self.clock.t },
                                unitSystem: { .metric }, persistEnabled: false)
        a.isEnabled = true
        XCTAssertEqual(spy.texts, ["Voice ready"])
        XCTAssertGreaterThanOrEqual(spy.activateCount, 1)

        a.isEnabled = false
        XCTAssertGreaterThanOrEqual(spy.stopCount, 1)
        XCTAssertEqual(spy.deactivateCount, 1)
    }

    func testImperialUnits_spokenInFeet() {
        let a = makeAnnouncer(units: .imperial)
        driveBurnout(a, speed: 100)   // 100 m/s = 328 ft/s
        XCTAssertEqual(spy.texts, ["Burnout. Max speed 328 feet per second"])
    }
}
