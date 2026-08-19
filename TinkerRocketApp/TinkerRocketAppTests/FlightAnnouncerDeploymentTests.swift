import XCTest
@testable import TinkerRocketApp

/// Recovery-deployment callouts (#813), the Swift twin of the `#813` section
/// in Android's `FlightAnnouncerTest`.  Same ten cases in the same order, so a
/// behaviour that drifts between the platforms fails on one side and shows up
/// as a missing case on the other.
///
/// Frames are delivered at 2 Hz throughout — the LoRa downlink rate, which is
/// the slowest link the detector has to work on and the one that matters at
/// range.
final class FlightAnnouncerDeploymentTests: XCTestCase {

    final class SpySpeech: AnnouncerSpeech {
        var isBusy = false
        var sessionActive = true
        var lastError: String?
        var onStatusChange: (() -> Void)?
        var spoken: [(text: String, interrupt: Bool)] = []
        func speak(_ text: String, interrupt: Bool) { spoken.append((text, interrupt)) }
        func stop() {}
        func activate() {}
        func deactivate() {}
        var texts: [String] { spoken.map(\.text) }
    }

    final class Clock {
        var t = Date(timeIntervalSince1970: 1_000_000)
        func advance(_ s: TimeInterval) { t.addTimeInterval(s) }
    }

    /// The stock profile: 60 fps drogue, 12 fps main, main at 700 ft.
    private let profile = RecoveryProfile(drogueMps: 60.0 * 0.3048,       // 18.29 m/s
                                          mainMps: 12.0 * 0.3048,         //  3.66 m/s
                                          mainDeployAglM: 700.0 * 0.3048) // 213.4 m

    private var spy = SpySpeech()
    private var clock = Clock()

    private func makeAnnouncer(recovery: RecoveryProfile?) -> FlightAnnouncer {
        spy = SpySpeech()
        clock = Clock()
        let c = clock
        let a = FlightAnnouncer(speech: spy, now: { c.t },
                                unitSystem: { .metric }, persistEnabled: false,
                                recovery: { recovery })
        a.isEnabled = true
        spy.spoken.removeAll()   // drop the "Voice ready" confirmation
        return a
    }

    private let APO = 0x04

    private func frame(palt: Float?, rate: Float?, apo: Bool, maxAlt: Float? = nil) -> TelemetryData {
        var t = TelemetryData()
        t.state = "INFLIGHT"
        t.pressure_alt = palt
        t.altitude_rate = rate
        t.max_alt_m = maxAlt
        t.flight_status_bits = apo ? APO : 0
        t.data_status = .live
        return t
    }

    /// Feed the apogee frame and clear the log, leaving the flight descending.
    private func pastApogee(_ a: FlightAnnouncer, maxAlt: Float = 1000) {
        a.processTelemetry(frame(palt: maxAlt, rate: 0, apo: true, maxAlt: maxAlt))
        spy.spoken.removeAll()
    }

    /// Descend at `rate` (positive = falling) for `seconds`, integrating altitude.
    @discardableResult
    private func descend(_ a: FlightAnnouncer,
                         rate: Float,
                         from alt: Float,
                         seconds: TimeInterval,
                         step: TimeInterval = 0.5) -> Float {
        var altitude = alt
        var elapsed: TimeInterval = 0
        while elapsed < seconds {
            a.processTelemetry(frame(palt: altitude, rate: -rate, apo: true))
            clock.advance(step)
            elapsed += step
            altitude -= rate * Float(step)
        }
        return altitude
    }

    // MARK: - Cases

    func testTwoStageRecoveryAnnouncesDrogueThenMain() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        var alt = descend(a, rate: 30, from: 1000, seconds: 6)   // ballistic
        alt = descend(a, rate: 15, from: alt, seconds: 8)        // drogue
        XCTAssertTrue(spy.texts.contains("Drogue out."), "got \(spy.texts)")

        alt = descend(a, rate: 15, from: alt, seconds: 30)       // drift to main altitude
        descend(a, rate: 3.5, from: alt, seconds: 8)             // main
        XCTAssertTrue(spy.texts.contains("Main out."), "got \(spy.texts)")

        let drogue = spy.texts.firstIndex(of: "Drogue out.")!
        let main = spy.texts.firstIndex(of: "Main out.")!
        XCTAssertLessThan(drogue, main, "drogue must be called before main")
    }

    func testDeploymentInterruptsButVerdictDoesNot() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        let alt = descend(a, rate: 30, from: 1000, seconds: 6)
        descend(a, rate: 15, from: alt, seconds: 8)

        let i = spy.texts.firstIndex(of: "Drogue out.")!
        XCTAssertTrue(spy.spoken[i].interrupt, "the deployment callout must interrupt")
        let v = spy.texts.firstIndex(where: { $0.hasPrefix("Good chute") })!
        XCTAssertGreaterThan(v, i, "verdict should follow the deployment")
        XCTAssertFalse(spy.spoken[v].interrupt, "the verdict must not interrupt")
    }

    func testNominalRateEarnsGoodChute() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        let alt = descend(a, rate: 30, from: 1000, seconds: 6)
        descend(a, rate: 15, from: alt, seconds: 8)   // 15 m/s vs 18.3 expected
        XCTAssertTrue(spy.texts.contains { $0.hasPrefix("Good chute, descending") },
                      "a drogue at 15 m/s against an 18 m/s profile is nominal: \(spy.texts)")
    }

    func testBadRateIsStatedAsABareNumberNeverAnAlarm() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        var alt = descend(a, rate: 30, from: 1000, seconds: 6)
        alt = descend(a, rate: 15, from: alt, seconds: 8)
        spy.spoken.removeAll()

        alt = descend(a, rate: 15, from: alt, seconds: 30)
        descend(a, rate: 8, from: alt, seconds: 8)    // 8 m/s vs 3.7 expected — bad

        XCTAssertTrue(spy.texts.contains("Main out."), "got \(spy.texts)")
        XCTAssertTrue(spy.texts.contains("Descending 8 meters per second"),
                      "a bad main should be quoted as a plain rate: \(spy.texts)")
        XCTAssertFalse(spy.texts.contains { $0.contains("Good chute") },
                       "8 m/s under a 3.7 m/s main is not a good chute")
    }

    /// The failure this whole design guards against: descent rate collapses at
    /// touchdown exactly like a canopy, so a ballistic flight must not earn a
    /// deployment callout when it hits the ground.
    func testBallisticImpactIsNotAnnouncedAsADeployment() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a, maxAlt: 500)
        let alt = descend(a, rate: 40, from: 500, seconds: 11)   // straight in
        XCTAssertLessThan(alt, 70, "should be near the ground")
        descend(a, rate: 0, from: 8, seconds: 6)                 // stopped, on the deck

        XCTAssertFalse(spy.texts.contains { $0.contains("out.") },
                       "impact must not read as a deployment: \(spy.texts)")
    }

    func testSingleDeployBelowMainAltitudeIsCalledMain() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a, maxAlt: 250)
        let alt = descend(a, rate: 25, from: 250, seconds: 3)    // brief ballistic
        descend(a, rate: 4, from: alt, seconds: 8)               // straight to main

        XCTAssertTrue(spy.texts.contains("Main out."), "got \(spy.texts)")
        XCTAssertFalse(spy.texts.contains("Drogue out."), "there was no drogue")
    }

    func testNoProfileStillAnnouncesButCannotGrade() {
        let a = makeAnnouncer(recovery: nil)
        pastApogee(a)
        let alt = descend(a, rate: 30, from: 1000, seconds: 6)
        descend(a, rate: 15, from: alt, seconds: 8)

        XCTAssertTrue(spy.texts.contains("Chute out."), "got \(spy.texts)")
        XCTAssertFalse(spy.texts.contains { $0.contains("Good chute") },
                       "nothing to grade against without a profile")
    }

    func testBriefDipDoesNotFire() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        var alt = descend(a, rate: 30, from: 1000, seconds: 6)
        alt = descend(a, rate: 10, from: alt, seconds: 1)    // below the ratio, 1 s only
        descend(a, rate: 30, from: alt, seconds: 4)          // back up to speed

        XCTAssertFalse(spy.texts.contains { $0.contains("out.") },
                       "a dip shorter than the hold is not a canopy: \(spy.texts)")
    }

    func testBeforeApogeeNothingIsDetected() {
        let a = makeAnnouncer(recovery: profile)
        var alt: Float = 500
        for _ in 0..<40 {
            a.processTelemetry(frame(palt: alt, rate: -30, apo: false))
            clock.advance(0.5)
            alt -= 15
        }
        XCTAssertFalse(spy.texts.contains { $0.contains("out.") }, "got \(spy.texts)")
    }

    func testResetClearsDeploymentStateForTheNextFlight() {
        let a = makeAnnouncer(recovery: profile)
        pastApogee(a)
        var alt = descend(a, rate: 30, from: 1000, seconds: 6)
        descend(a, rate: 15, from: alt, seconds: 8)
        XCTAssertTrue(spy.texts.contains("Drogue out."))

        a.reset()
        spy.spoken.removeAll()

        // A second flight must be able to call its own drogue again.
        pastApogee(a)
        alt = descend(a, rate: 30, from: 1000, seconds: 6)
        descend(a, rate: 15, from: alt, seconds: 8)
        XCTAssertTrue(spy.texts.contains("Drogue out."), "reset should re-arm: \(spy.texts)")
    }
}
