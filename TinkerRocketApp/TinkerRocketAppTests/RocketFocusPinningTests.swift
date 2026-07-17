import XCTest
@testable import TinkerRocketApp

/// #390: a base-station link pins its mirrored telemetry / announcer /
/// fix latch to ONE focused rocket (sticky first-heard). Before this,
/// every relayed frame from ANY rocket overwrote `device.telemetry` and
/// fed the announcer, so two powered rockets flip-flopped the dashboard
/// and interleaved voice callouts at packet rate.
final class RocketFocusPinningTests: XCTestCase {

    final class SpyAnnouncer: TelemetryAnnouncer {
        var received: [TelemetryData] = []
        func processTelemetry(_ telemetry: TelemetryData) {
            received.append(telemetry)
        }
        func reset() {}
    }

    private func makeBaseStation() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-B-Test")
    }

    private func relayedJSON(rocketID: Int, state: String = "READY",
                             alt: Double = 0) -> Data {
        """
        {"rid":\(rocketID),"run":"R\(rocketID)","st":"\(state)","palt":\(alt)}
        """.data(using: .utf8)!
    }

    // MARK: - Sticky first-heard focus

    func testFirstRelayedRocket_BecomesFocus() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        XCTAssertEqual(bs.focusRocketID, 1)
    }

    func testSecondRocket_DoesNotStealFocus() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        bs.parseTelemetryData(relayedJSON(rocketID: 2))
        bs.parseTelemetryData(relayedJSON(rocketID: 2))
        XCTAssertEqual(bs.focusRocketID, 1,
                       "Focus is sticky — the old last-heard behavior is the bug")
    }

    // MARK: - Telemetry mirror pinning

    func testInterleavedRockets_MirrorFollowsFocusOnly() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1, state: "READY", alt: 10))
        bs.parseTelemetryData(relayedJSON(rocketID: 2, state: "INFLIGHT", alt: 500))

        XCTAssertEqual(bs.telemetry.source_rocket_id, 1,
                       "A non-focused rocket's frame must not overwrite the mirror")
        XCTAssertEqual(bs.telemetry.state, "READY")

        // Both rockets are still tracked for the roster.
        XCTAssertEqual(bs.remoteRockets.count, 2)
        XCTAssertEqual(
            bs.remoteRockets.first { $0.rocketID == 2 }?.telemetry.state,
            "INFLIGHT",
            "The non-focused rocket's stream lives on its RemoteRocket")
    }

    // MARK: - Announcer pinning

    func testInterleavedRockets_AnnouncerHearsFocusOnly() {
        let bs = makeBaseStation()
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayedJSON(rocketID: 7, state: "PRELAUNCH"))
        bs.parseTelemetryData(relayedJSON(rocketID: 9, state: "INFLIGHT"))
        bs.parseTelemetryData(relayedJSON(rocketID: 7, state: "INFLIGHT"))

        XCTAssertEqual(spy.received.map { $0.source_rocket_id }, [7, 7],
                       "Callouts must not interleave two rockets (#390)")
        XCTAssertEqual(spy.received.map(\.state), ["PRELAUNCH", "INFLIGHT"])
    }

    // MARK: - User focus switch

    func testSetFocus_RepinsMirrorImmediately() {
        let fleet = BLEFleet()
        let bs = makeBaseStation()
        bs.fleet = fleet

        bs.parseTelemetryData(relayedJSON(rocketID: 1, state: "READY", alt: 10))
        bs.parseTelemetryData(relayedJSON(rocketID: 2, state: "INFLIGHT", alt: 500))
        XCTAssertEqual(bs.telemetry.source_rocket_id, 1)

        fleet.setFocus(baseStation: bs, rocketID: 2)

        XCTAssertEqual(bs.focusRocketID, 2)
        XCTAssertEqual(bs.telemetry.source_rocket_id, 2,
                       "Switch re-mirrors the cached stream without waiting for a frame")
        XCTAssertEqual(bs.telemetry.state, "INFLIGHT")

        // And subsequent frames follow the new focus.
        bs.parseTelemetryData(relayedJSON(rocketID: 1, state: "READY", alt: 11))
        XCTAssertEqual(bs.telemetry.source_rocket_id, 2)
        bs.parseTelemetryData(relayedJSON(rocketID: 2, state: "LANDED", alt: 3))
        XCTAssertEqual(bs.telemetry.state, "LANDED")
    }

    // MARK: - Focused-relay staleness overlay

    func testEffectiveDataStatus_OverlaysFocusedRocketAge() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        XCTAssertEqual(bs.effectiveDataStatus, .live)

        // Focused rocket last heard 10 s ago (simulate by refreshing the
        // overlay with a future "now") — the frame-carried status is still
        // .live because no newer frame arrived to correct it.
        bs.refreshFocusedRelayFreshness(now: Date().addingTimeInterval(10))
        XCTAssertEqual(bs.effectiveDataStatus, .stale,
                       "App-computed staleness must overlay the frozen frame status")
        XCTAssertGreaterThan(bs.effectiveDataAgeMs, 9000)
    }

    func testEffectiveDataStatus_FreshFocus_StaysLive() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        bs.refreshFocusedRelayFreshness(now: Date().addingTimeInterval(1))
        XCTAssertEqual(bs.effectiveDataStatus, .live)
    }
}
