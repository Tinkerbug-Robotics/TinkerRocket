import XCTest
@testable import TinkerRocketApp

/// #968: `alt_apo` / `vel_apo` are LIVE VOTES from a leaky counter whose baro
/// test requires `alt_est > 15.0f`, so both clear below ~15 m AGL.  Measured on
/// the 2026-08-27 sim flight: `alt_apogee` dropped at 130.96 s and stayed down
/// through the end of the log at 138.84 s — the last ~8 seconds of the flight.
///
/// Consumers that mean "this flight is past apogee" therefore flipped back to
/// "pre-apogee" right before landing.  That is what let a stale burnout be
/// announced (#964), and the same clearing caused #235.  `past_apogee` is the
/// latch the wire cannot carry.
final class PastApogeeLatchTests: XCTestCase {

    private func makeBaseStation() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-B-Test")
    }

    private func makeRocket() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-R-Test")
    }

    private let ALT_APO = 0x04, VEL_APO = 0x02, LANDED = 0x08

    private func directJSON(state: String = "INFLIGHT", fs: Int) -> Data {
        """
        {"st":"\(state)","fs":\(fs),"palt":120.5}
        """.data(using: .utf8)!
    }

    private func relayedJSON(rocketID: Int, state: String = "INFLIGHT", fs: Int) -> Data {
        """
        {"rid":\(rocketID),"st":"\(state)","fs":\(fs),"palt":120.5}
        """.data(using: .utf8)!
    }

    // MARK: - The regression

    /// The exact shape of the sim flight: the vote rises at apogee, then clears
    /// under ~15 m AGL.  The latch must survive that.
    func testLatchHoldsWhenTheLiveVoteClearsNearLanding() {
        let r = makeRocket()
        r.parseTelemetryData(directJSON(fs: ALT_APO))
        XCTAssertTrue(r.telemetry.past_apogee, "latch should rise with the vote")

        // Below ~15 m AGL the FC's counter decays and the vote clears.
        r.parseTelemetryData(directJSON(fs: 0))
        XCTAssertFalse(r.telemetry.alt_apo, "the live vote really does clear")
        XCTAssertTrue(r.telemetry.past_apogee,
                      "past_apogee must NOT follow the vote down (#968)")
    }

    /// `vel_apo` leads `alt_apo` by a few hundred ms in the record, so either
    /// vote arms the latch.
    func testVelocityVoteAlsoArmsTheLatch() {
        let r = makeRocket()
        r.parseTelemetryData(directJSON(fs: VEL_APO))
        XCTAssertTrue(r.telemetry.past_apogee)
    }

    func testNotLatchedBeforeApogee() {
        let r = makeRocket()
        r.parseTelemetryData(directJSON(fs: 0))
        XCTAssertFalse(r.telemetry.past_apogee)
    }

    // MARK: - Reset

    /// A second flight on the same connection must start clean, matching
    /// `FlightAnnouncer.resetFlightState()`.
    func testLatchClearsOnPrelaunchForTheNextFlight() {
        let r = makeRocket()
        r.parseTelemetryData(directJSON(fs: ALT_APO))
        XCTAssertTrue(r.telemetry.past_apogee)

        r.parseTelemetryData(directJSON(state: "PRELAUNCH", fs: 0))
        XCTAssertFalse(r.telemetry.past_apogee, "PRELAUNCH re-arms for a new flight")

        r.parseTelemetryData(directJSON(fs: 0))
        XCTAssertFalse(r.telemetry.past_apogee, "and it stays clear until the next apogee")
    }

    // MARK: - Two rockets (#390)

    /// A base station can relay two flights at once.  One rocket reaching
    /// apogee must not mark the other as past apogee.
    func testRelayedRocketsLatchIndependently() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 7, fs: ALT_APO))
        bs.parseTelemetryData(relayedJSON(rocketID: 9, fs: 0))

        let seven = bs.remoteRockets.first { $0.rocketID == 7 }
        let nine  = bs.remoteRockets.first { $0.rocketID == 9 }
        XCTAssertEqual(bs.remoteRockets.count, 2, "both rockets should be tracked")
        XCTAssertTrue(seven?.telemetry.past_apogee ?? false,
                      "rocket 7 passed apogee")
        XCTAssertFalse(nine?.telemetry.past_apogee ?? true,
                       "rocket 9 has not — latches must not cross (#390)")
    }
}
