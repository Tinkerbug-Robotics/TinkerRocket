import XCTest
import Combine
@testable import TinkerRocketApp

/// Voice follows the computer the dashboard shows (`VoiceRocketPolicy`), not
/// only the base station's radio focus.
///
/// 2026-11-21, BARC: the 65 mm rocket (rocket 8) flew through the base
/// station first. A flown rocket keeps the focus pin when it goes quiet, so
/// the pin was still on rocket 8 when Rolly Polly V flew with two computers,
/// rocket 1 in the nose and rocket 7 in the avionics bay. Both were on screen
/// with Follow buttons, and the flight was silent: voice spoke only the
/// focused rocket, and that was rocket 8.
final class VoiceFollowsViewedRocketTests: XCTestCase {

    // MARK: - Policy

    private typealias Candidate = VoiceRocketPolicy.Candidate

    private func r(_ rid: UInt8, _ state: String = "PRELAUNCH", silentFor: TimeInterval = 0.5,
                   hidden: Bool = false, landed: Bool = false,
                   simulated: Bool = false) -> Candidate {
        Candidate(rocketID: rid, hidden: hidden, silentFor: silentFor, state: state,
                  landed: landed, simulated: simulated)
    }

    private func choose(focus: UInt8?, focusHidden: Bool = false, fresh: Bool = false,
                        current: UInt8? = nil, _ rockets: [Candidate]) -> UInt8? {
        VoiceRocketPolicy.choose(focus: focus, focusHidden: focusHidden,
                                 focusPinFresh: fresh, current: current, rockets: rockets)
    }

    func testFollowedRocketOnScreen_KeepsVoice() {
        XCTAssertEqual(choose(focus: 7, [r(1), r(7)]), 7)
    }

    /// THE regression: a pin on a rocket that has no section on screen.
    func testPinOnARocketNotOnScreen_VoiceFollowsTheComputerShown() {
        XCTAssertEqual(choose(focus: 8, [r(1), r(7)]), 1)
    }

    func testFreshPin_HoldsVoiceUntilItsRocketIsHeard() {
        XCTAssertEqual(choose(focus: 8, fresh: true, [r(1)]), 8,
                       "a reconnect re-seeds the pin before its rocket has transmitted")
    }

    func testFreshPin_OnAHiddenRocket_DoesNotHold() {
        XCTAssertEqual(choose(focus: 8, focusHidden: true, fresh: true, [r(1)]), 1)
    }

    func testHiddenFollowedRocket_VoiceMovesToTheOneShown() {
        XCTAssertEqual(choose(focus: 1, focusHidden: true, [r(1, hidden: true), r(7)]), 7)
    }

    /// The same failure with the base-station link never dropped: rocket 8's
    /// section is still up (its last frame said INFLIGHT, the link died
    /// before LANDED) when the next rocket launches.
    func testQuietFollowedRocket_AnotherFlying_TheFlightSpeaks() {
        XCTAssertEqual(choose(focus: 8, [r(1, "INFLIGHT"), r(7, "INFLIGHT"),
                                         r(8, "INFLIGHT", silentFor: 5400)]), 1)
    }

    func testQuietFollowedRocket_NothingFlying_KeepsVoice() {
        // The recovery walk: the landed rocket keeps its screen and its voice.
        XCTAssertEqual(choose(focus: 8, [r(1), r(8, "LANDED", silentFor: 60, landed: true)]), 8)
    }

    func testFollowedRocketOnThePad_AnotherFlying_KeepsVoice() {
        XCTAssertEqual(choose(focus: 1, [r(1), r(2, "INFLIGHT")]), 1,
                       "the user follows this one; Follow moves voice to the other")
    }

    func testTwoComputersFlying_VoiceStaysWhereItIs() {
        XCTAssertEqual(choose(focus: 8, current: 7,
                              [r(1, "INFLIGHT"), r(7, "INFLIGHT"), r(8, silentFor: 600)]), 7)
    }

    func testSimulatedFlight_DoesNotClaimVoice() {
        XCTAssertEqual(choose(focus: 8, [r(1, "INFLIGHT", simulated: true),
                                         r(8, "LANDED", landed: true)]), 8)
    }

    func testRocketHeardLongAgo_LosesToOneBeingHeard() {
        XCTAssertEqual(choose(focus: 9, [r(1, silentFor: 600), r(7)]), 7)
    }

    func testEverythingHidden_FallsBackToTheFocus() {
        XCTAssertEqual(choose(focus: 1, focusHidden: true, [r(1, hidden: true)]), 1)
    }

    func testNothingHeardNothingPinned_NoVoiceRocket() {
        XCTAssertNil(choose(focus: nil, []))
    }

    // MARK: - Base-station dispatch

    final class SpyAnnouncer: TelemetryAnnouncer {
        var received: [TelemetryData] = []
        var handovers: [TelemetryData?] = []
        func processTelemetry(_ telemetry: TelemetryData) { received.append(telemetry) }
        func reset() {}
        func followStream(baseline: TelemetryData?) { handovers.append(baseline) }
    }

    private var defaultsSuite: String!

    override func setUp() {
        super.setUp()
        defaultsSuite = "VoiceFollowsViewedRocketTests.\(UUID().uuidString)"
    }

    override func tearDown() {
        UserDefaults().removePersistentDomain(forName: defaultsSuite)
        super.tearDown()
    }

    /// A fleet that cannot write the app's real device registry.
    private func makeFleet() -> BLEFleet {
        BLEFleet(knownDevices: KnownDeviceStore(defaults: UserDefaults(suiteName: defaultsSuite)!))
    }

    private func makeBaseStation(in fleet: BLEFleet) -> BLEDevice {
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.fleet = fleet
        return bs
    }

    private let LAUNCH = 0x01, ALT_APO = 0x04, LANDED = 0x08

    private func relayed(_ rid: Int, _ state: String, fs: Int = 0, palt: Double = 0,
                         mspd: Double = 0, rate: Double = 0) -> Data {
        """
        {"rid":\(rid),"run":"R\(rid)","st":"\(state)","fs":\(fs),"palt":\(palt),\
        "mspd":\(mspd),"arate":\(rate),"malt":600,"lat":39.4681,"lon":-75.2927}
        """.data(using: .utf8)!
    }

    /// Rocket 8 flew and was recovered; the base station rebooted, the fleet
    /// re-seeded the pin on rocket 8 and the grace window ran out. What was
    /// true when Rolly Polly V reached the pad.
    private func wedgedOnRocket8(_ fleet: BLEFleet, _ bs: BLEDevice) {
        var flown = TelemetryData()
        flown.state = "INFLIGHT"
        fleet.noteFlightProgress(flown, key: RocketKey(networkID: bs.networkID, rocketID: 8))
        bs.focusRocketID = 8
        bs.focusPinGraceUntil = Date().addingTimeInterval(-1)
    }

    func testStalePinOnAFlownRocket_TheComputersOnScreenAreHeard() {
        let fleet = makeFleet()
        let bs = makeBaseStation(in: fleet)
        wedgedOnRocket8(fleet, bs)
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayed(1, "PRELAUNCH"))
        bs.parseTelemetryData(relayed(7, "PRELAUNCH"))
        bs.parseTelemetryData(relayed(1, "INFLIGHT", fs: LAUNCH, mspd: 30))
        bs.parseTelemetryData(relayed(7, "INFLIGHT", fs: LAUNCH, mspd: 30))

        XCTAssertEqual(bs.focusRocketID, 8, "the radio pin is not this change's business")
        XCTAssertEqual(spy.received.map(\.source_rocket_id), [1, 1],
                       "voice follows the first computer on screen, one stream only")
        XCTAssertEqual(bs.voiceRocketID, 1)

        // The marker reads it through the fleet's voice device.
        fleet.devices.append(bs)
        bs.isConnected = true
        XCTAssertEqual(fleet.voiceRocketKey, RocketKey(networkID: bs.networkID, rocketID: 1))
    }

    /// The same flight through the real announcer: every callout, once.
    func testStalePinOnAFlownRocket_TwoComputerFlight_IsCalledOut() {
        let fleet = makeFleet()
        let bs = makeBaseStation(in: fleet)
        wedgedOnRocket8(fleet, bs)
        let speech = FlightAnnouncerPolicyTests.SpySpeech()
        var now = Date(timeIntervalSince1970: 1_000_000)
        let announcer = FlightAnnouncer(speech: speech, now: { now }, unitSystem: { .metric },
                                        persistEnabled: false, recovery: { nil },
                                        schedule: { _, _ in AnyCancellable {} })
        announcer.isEnabled = true
        speech.spoken.removeAll()
        bs.flightAnnouncer = announcer

        // Both computers at 2 Hz, interleaved, as the base station relays them.
        func tick(_ make: (Int) -> Data) {
            bs.parseTelemetryData(make(1))
            bs.parseTelemetryData(make(7))
            now.addTimeInterval(0.5)
        }
        for _ in 0..<4 { tick { relayed($0, "PRELAUNCH") } }
        for speed in [30.0, 60, 90, 101] {
            tick { relayed($0, "INFLIGHT", fs: self.LAUNCH, palt: speed, mspd: speed, rate: speed) }
        }
        for i in 0..<16 {
            tick { relayed($0, "INFLIGHT", fs: self.LAUNCH, palt: 150 + Double(i) * 28,
                           mspd: 101, rate: 60 - Double(i) * 4) }
        }
        // Descent; the nose computer's apogee vote toggles, as it did on 11-21.
        for i in 0..<30 {
            tick { rid in
                let vote = rid == 7 || i % 4 != 1 ? self.ALT_APO : 0
                return self.relayed(rid, "INFLIGHT", fs: self.LAUNCH | vote,
                                    palt: 590 - Double(i) * 10, mspd: 101, rate: -20)
            }
        }
        for _ in 0..<3 {
            tick { relayed($0, "INFLIGHT", fs: self.LAUNCH | self.LANDED, palt: 0, mspd: 101) }
        }

        let said = speech.texts
        XCTAssertEqual(said.filter { $0.hasPrefix("Burnout") }.count, 1, "\(said)")
        XCTAssertEqual(said.filter { $0.hasPrefix("Apogee") }.count, 1, "\(said)")
        XCTAssertEqual(said.filter { $0.hasPrefix("Landed") }.count, 1, "\(said)")
    }

    func testHidingTheFollowedComputer_MovesVoiceWithAHandover() {
        let fleet = makeFleet()
        let bs = makeBaseStation(in: fleet)
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayed(1, "INFLIGHT", fs: LAUNCH | ALT_APO, palt: 500))
        bs.parseTelemetryData(relayed(7, "INFLIGHT", fs: LAUNCH | ALT_APO, palt: 490))
        XCTAssertEqual(bs.focusRocketID, 1)
        XCTAssertEqual(spy.received.map(\.source_rocket_id), [1])

        fleet.hiddenRocketKeys.insert(RocketKey(networkID: bs.networkID, rocketID: 1))
        bs.parseTelemetryData(relayed(1, "INFLIGHT", fs: LAUNCH | ALT_APO, palt: 480))
        bs.parseTelemetryData(relayed(7, "INFLIGHT", fs: LAUNCH | ALT_APO, palt: 470))

        XCTAssertEqual(bs.focusRocketID, 1, "hiding never touches the radio")
        XCTAssertEqual(spy.received.map(\.source_rocket_id), [1, 7])
        XCTAssertEqual(spy.handovers.count, 1)
        XCTAssertEqual(spy.handovers.first??.pressure_alt, 490,
                       "the baseline is rocket 7's own previous frame")
        XCTAssertEqual(spy.handovers.first??.past_apogee, true)
    }

    func testFollow_MovesVoice() {
        let fleet = makeFleet()
        let bs = makeBaseStation(in: fleet)
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayed(1, "PRELAUNCH"))
        bs.parseTelemetryData(relayed(7, "PRELAUNCH"))
        fleet.setFocus(baseStation: bs, rocketID: 7)
        bs.parseTelemetryData(relayed(1, "PRELAUNCH"))
        bs.parseTelemetryData(relayed(7, "PRELAUNCH"))

        XCTAssertEqual(spy.received.map(\.source_rocket_id), [1, 7])
        XCTAssertEqual(spy.handovers.count, 1)
    }

    func testRoutingTheSameAnnouncerAgain_IsNotAHandover() {
        let fleet = makeFleet()
        let bs = makeBaseStation(in: fleet)
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy
        bs.parseTelemetryData(relayed(1, "PRELAUNCH"))
        bs.flightAnnouncer = spy
        XCTAssertEqual(bs.voiceStreamRocketID, 1)
        bs.flightAnnouncer = SpyAnnouncer()
        XCTAssertNil(bs.voiceStreamRocketID, "a different announcer starts with no stream")
    }

    // MARK: - Announcer hand-over

    private func frame(_ state: String, fs: Int = 0, palt: Float = 0, mspd: Float = 0,
                       lat: Double? = nil, lon: Double? = nil) -> TelemetryData {
        var t = TelemetryData()
        t.state = state
        t.flight_status_bits = fs
        t.pressure_alt = palt
        t.max_speed_mps = mspd
        t.max_alt_m = 600
        t.latitude = lat
        t.longitude = lon
        return t
    }

    private func makeAnnouncer() -> (FlightAnnouncer, FlightAnnouncerPolicyTests.SpySpeech) {
        let speech = FlightAnnouncerPolicyTests.SpySpeech()
        let a = FlightAnnouncer(speech: speech, now: { Date(timeIntervalSince1970: 1_000_000) },
                                unitSystem: { .metric }, persistEnabled: false,
                                recovery: { nil }, schedule: { _, _ in AnyCancellable {} })
        a.isEnabled = true
        speech.spoken.removeAll()
        return (a, speech)
    }

    private let PAST_APOGEE = 0x04 | TelemetryData.pastApogeeBit

    /// A rocket that flew earlier must not leave apogee and landing marked as
    /// said for the next one. The next one's frames start PRELAUNCH like its
    /// baseline, so no PRELAUNCH transition would clear them.
    func testHandover_TheNextRocketsFlightIsCalledOut() {
        let (a, speech) = makeAnnouncer()
        a.processTelemetry(frame("PRELAUNCH"))
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH | PAST_APOGEE))
        a.processTelemetry(frame("LANDED", fs: LAUNCH | PAST_APOGEE | LANDED))
        XCTAssertEqual(speech.texts.filter { $0.hasPrefix("Apogee") }.count, 1)

        speech.spoken.removeAll()
        let pad = frame("PRELAUNCH", lat: 39.4681, lon: -75.2927)
        a.followStream(baseline: pad)
        a.processTelemetry(pad)
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH, palt: 100))
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH | PAST_APOGEE, palt: 600))
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH | PAST_APOGEE | LANDED, palt: 0,
                                 lat: 39.4681, lon: -75.2927))

        XCTAssertEqual(speech.texts.filter { $0.hasPrefix("Apogee") }.count, 1, "\(speech.texts)")
        XCTAssertEqual(speech.texts.filter { $0.hasPrefix("Landed") }.count, 1, "\(speech.texts)")
    }

    func testHandoverPastApogee_ATogglingVoteIsNoFreshApogee() {
        let (a, speech) = makeAnnouncer()
        a.followStream(baseline: frame("INFLIGHT", fs: LAUNCH | PAST_APOGEE, palt: 400))
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH | TelemetryData.pastApogeeBit, palt: 390))
        a.processTelemetry(frame("INFLIGHT", fs: LAUNCH | PAST_APOGEE, palt: 380))
        XCTAssertFalse(speech.texts.contains { $0.hasPrefix("Apogee") }, "\(speech.texts)")
    }

    func testHandoverLanded_IsNoFreshLanding() {
        let (a, speech) = makeAnnouncer()
        a.followStream(baseline: frame("LANDED", fs: LAUNCH | PAST_APOGEE | LANDED))
        a.processTelemetry(frame("LANDED", fs: LAUNCH | PAST_APOGEE | LANDED))
        XCTAssertTrue(speech.texts.isEmpty, "\(speech.texts)")
    }
}
