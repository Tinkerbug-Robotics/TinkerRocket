import XCTest
import Combine
@testable import TinkerRocketApp

/// The predicted landing call: when the base station loses the rocket at
/// touchdown, before the landed frame gets through, the landing is called at
/// the predicted touchdown instead of minutes later.  The flight data is in
/// `TouchdownPredictorTests`.  Android twin: the predicted-landing cases in
/// `FlightAnnouncerTest`.
final class FlightAnnouncerPredictedLandingTests: XCTestCase {

    typealias SpySpeech = FlightAnnouncerPolicyTests.SpySpeech
    typealias Clock = FlightAnnouncerPolicyTests.Clock

    /// Captures what the announcer schedules; a test fires it by advancing
    /// the clock past it.
    final class ManualScheduler {
        var pending: [(at: Date, action: () -> Void, live: () -> Bool)] = []
        func schedule(_ at: Date, _ action: @escaping () -> Void) -> AnyCancellable {
            var cancelled = false
            pending.append((at, action, { !cancelled }))
            return AnyCancellable { cancelled = true }
        }
        /// Run every live action due by `now`.
        func fire(upTo now: Date) {
            let due = pending.filter { $0.at <= now && $0.live() }
            pending.removeAll { $0.at <= now }
            due.forEach { $0.action() }
        }
        var liveCount: Int { pending.filter { $0.live() }.count }
    }

    private var spy = SpySpeech()
    private var clock = Clock()
    private var scheduler = ManualScheduler()

    private func makeAnnouncer(enabled: Bool = true,
                               recovery: RecoveryProfile? = nil) -> FlightAnnouncer {
        spy = SpySpeech()
        clock = Clock()
        scheduler = ManualScheduler()
        let c = clock, s = scheduler
        let a = FlightAnnouncer(speech: spy, now: { c.t },
                                unitSystem: { .metric }, persistEnabled: false,
                                recovery: { recovery },
                                schedule: { s.schedule($0, $1) })
        if enabled {
            a.isEnabled = true
            spy.spoken.removeAll()   // drop the "Voice ready" confirmation
        }
        return a
    }

    private let APO = 0x04 | 0x400, APO_VOTE_CLEARED = 0x400, LANDED_F = 0x08

    private func frame(state: String = "INFLIGHT", palt: Float? = nil, rate: Float? = nil,
                       fs: Int = 0, lat: Double? = nil, lon: Double? = nil,
                       stale: Bool = false) -> TelemetryData {
        var t = TelemetryData()
        t.state = state
        t.pressure_alt = palt
        t.altitude_rate = rate
        t.flight_status_bits = fs
        t.latitude = lat
        t.longitude = lon
        t.data_status = stale ? .stale : .live
        return t
    }

    /// The pad, then the real last 20 s of 03 at the relay's spacing, the
    /// clock left at the flight computer's touchdown + `endAt`.
    private func flyRollCtrl03(_ a: FlightAnnouncer, lat: Double? = nil, lon: Double? = nil) {
        a.processTelemetry(frame(state: "PRELAUNCH", palt: 0, lat: 40.0, lon: -75.0))
        let start = clock.t.addingTimeInterval(30)
        for (t, alt, rate) in TouchdownPredictorTests.rollCtrl03 {
            clock.t = start.addingTimeInterval(t)
            a.processTelemetry(frame(palt: alt, rate: rate,
                                     fs: alt > 15 ? APO : APO_VOTE_CLEARED,
                                     lat: lat, lon: lon))
        }
        touchdownAt = start
    }
    private var touchdownAt = Date()

    /// Advance the clock in `step`s to `until` seconds after touchdown,
    /// firing anything due on the way (no frames: the link is down).
    private func wait(until s: TimeInterval, step: TimeInterval = 0.1) {
        while clock.t < touchdownAt.addingTimeInterval(s) {
            clock.advance(step)
            scheduler.fire(upTo: clock.t)
        }
    }

    private var spokenLandingCalls: [String] {
        spy.texts.filter { $0.hasPrefix("Predicted landing") || $0.hasPrefix("Landed") }
    }

    // MARK: - Tests

    /// The 2026-10-24 flight 03: the link died 0.5 s before touchdown.  The
    /// call comes 1.5 s after the last frame — 1.0 s after the real touchdown.
    func testLinkDiesAtTouchdown_callsTheLandingFromThePrediction() {
        let a = makeAnnouncer()
        flyRollCtrl03(a, lat: 40.001, lon: -75.0)

        wait(until: 0.9)
        XCTAssertTrue(spokenLandingCalls.isEmpty, "not before 1.5 s of silence")
        wait(until: 1.1)
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. 111 meters away"])
        XCTAssertTrue(spy.spoken.last!.interrupt, "a landing call interrupts, like the real one")
    }

    /// In the background the app may not run timers, but the base station's
    /// stale re-pushes keep arriving every 2 s, and each one checks.
    func testStaleRePushAlsoMakesTheCall() {
        let a = makeAnnouncer()
        flyRollCtrl03(a)
        clock.t = touchdownAt.addingTimeInterval(1.5)
        a.processTelemetry(frame(palt: 2, rate: -7, fs: APO_VOTE_CLEARED, stale: true))
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. "])
    }

    /// The landed frame that arrives minutes later, when someone walks into
    /// range, must not call the landing a second time.
    func testLateLandedFrame_doesNotCallAgain() {
        let a = makeAnnouncer()
        flyRollCtrl03(a)
        wait(until: 3)
        clock.t = touchdownAt.addingTimeInterval(140)
        a.processTelemetry(frame(state: "LANDED", palt: 0, rate: 0, fs: APO_VOTE_CLEARED | LANDED_F))
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. "])
    }

    /// While the link holds, the real landed frame makes the call and the
    /// prediction never speaks.
    func testLinkHolds_onlyTheRealLandingCall() {
        let a = makeAnnouncer()
        flyRollCtrl03(a)
        clock.t = touchdownAt.addingTimeInterval(0.0)
        a.processTelemetry(frame(palt: 0, rate: -2, fs: APO_VOTE_CLEARED))
        clock.t = touchdownAt.addingTimeInterval(0.5)
        a.processTelemetry(frame(palt: 0, rate: 0, fs: APO_VOTE_CLEARED | LANDED_F))
        wait(until: 10)
        XCTAssertEqual(spokenLandingCalls, ["Landed. "])
        XCTAssertEqual(scheduler.liveCount, 0, "the landed frame cancels the pending call")
    }

    /// A predicted call, then the link comes back with the rocket well off the
    /// ground (a dual-deploy main opened late, say): the prediction was early.
    /// The call is taken back, so the real landing still gets made.
    func testLinkReturnsWithTheRocketStillUp_realLandingIsStillCalled() {
        let a = makeAnnouncer()
        a.processTelemetry(frame(state: "PRELAUNCH", palt: 0))
        clock.advance(30)
        // Lost at 150 m falling at 20 m/s: predicted down 7.5 s later.
        for (alt, rate) in [(Float(180), Float(-20)), (170, -20), (160, -20), (150, -20)] {
            a.processTelemetry(frame(palt: alt, rate: rate, fs: APO))
            clock.advance(0.5)
        }
        touchdownAt = clock.t
        wait(until: 8)
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. "])

        // The link is back, still flying: 60 m and down at 5 m/s.
        for alt in stride(from: Float(60), through: 0, by: -2.5) {
            a.processTelemetry(frame(palt: alt, rate: -5, fs: alt > 15 ? APO : APO_VOTE_CLEARED))
            clock.advance(0.5)
            scheduler.fire(upTo: clock.t)
        }
        a.processTelemetry(frame(palt: 0, rate: 0, fs: APO_VOTE_CLEARED | LANDED_F))
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. ", "Landed. "])
    }

    func testVoiceOff_noPredictedCall() {
        let a = makeAnnouncer(enabled: false)
        flyRollCtrl03(a)
        wait(until: 5)
        XCTAssertTrue(spy.spoken.isEmpty)
    }

    /// The next flight starts clean: no call left over from the last one.
    func testPrelaunchResets() {
        let a = makeAnnouncer()
        flyRollCtrl03(a)
        a.processTelemetry(frame(state: "PRELAUNCH", palt: 0))
        wait(until: 5)
        XCTAssertTrue(spokenLandingCalls.isEmpty)
    }

    /// Dual deploy from the profile's pyro channels: lost at 400 m under
    /// drogue, the call waits for the main leg instead of coming 14-23 s
    /// early.
    func testDualDeployProfile_callWaitsForTheMainLeg() {
        let profile = RecoveryProfile(drogueMps: 18, mainMps: 6.5,
                                      mainDeployAglM: 213, pyroMainAglM: 243.84)
        let a = makeAnnouncer(recovery: profile)
        a.processTelemetry(frame(state: "PRELAUNCH", palt: 0))
        clock.advance(30)
        for alt in stride(from: Float(424), through: 400, by: -8) {
            a.processTelemetry(frame(palt: alt, rate: -16, fs: APO))
            clock.advance(0.5)
        }
        clock.advance(-0.5)
        touchdownAt = clock.t
        let expected = (400 - 243.84) / 16 + 243.84 / 6.5   // 47.3 s
        wait(until: expected - 0.5, step: 0.25)
        XCTAssertTrue(spokenLandingCalls.isEmpty)
        wait(until: expected + 0.5, step: 0.25)
        XCTAssertEqual(spokenLandingCalls, ["Predicted landing. "])
    }
}
