import XCTest
@testable import TinkerRocketApp

/// Regression tests for #138: voice callouts must fire when telemetry arrives
/// through a base-station LoRa relay, not just on direct rocket BLE.
/// The original bug was an early `return` in the BS branch of
/// `parseTelemetryData` that skipped the announcer entirely.
final class FlightAnnouncerDispatchTests: XCTestCase {

    /// Captures `processTelemetry` calls without touching AVFoundation, so the
    /// test can run on any host without an audio session.
    final class SpyAnnouncer: TelemetryAnnouncer {
        var received: [TelemetryData] = []
        var resetCount = 0
        func processTelemetry(_ telemetry: TelemetryData) {
            received.append(telemetry)
        }
        func reset() {
            resetCount += 1
        }
    }

    // MARK: - Helpers

    /// Build a base-station BLEDevice with no peripheral.  The "TR-B-" prefix
    /// causes `BLEDeviceType.from(name:)` to classify it as a base station.
    private func makeBaseStation() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-B-Test")
    }

    private func makeRocket() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-R-Test")
    }

    /// JSON keys mirror TelemetryData.CodingKeys ("rid", "st", "aapo", etc.).
    private func relayedJSON(rocketID: Int, state: String = "INFLIGHT") -> Data {
        """
        {"rid":\(rocketID),"run":"Atlas","st":"\(state)","palt":120.5,"mspd":85.0}
        """.data(using: .utf8)!
    }

    private func directJSON(state: String = "INFLIGHT") -> Data {
        """
        {"st":"\(state)","palt":120.5,"mspd":85.0}
        """.data(using: .utf8)!
    }

    // MARK: - Tests

    /// Core regression: relayed telemetry must reach the announcer.
    /// Before the fix this asserted 0 calls (the dispatch returned early).
    func testRelayedTelemetry_TriggersAnnouncer() {
        let bs = makeBaseStation()
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayedJSON(rocketID: 7))

        XCTAssertEqual(spy.received.count, 1,
                       "Relayed telemetry should invoke the announcer (#138)")
        XCTAssertEqual(spy.received.first?.source_rocket_id, 7)
        XCTAssertEqual(spy.received.first?.state, "INFLIGHT")
        XCTAssertEqual(bs.remoteRockets.count, 1,
                       "Relayed rocket should also be tracked in remoteRockets")
    }

    /// A full PRELAUNCH→INFLIGHT→LANDED sequence over the relay path should
    /// produce one announcer call per frame, so edge-detection in the
    /// announcer sees the transitions.
    func testRelayedFlightSequence_AllFramesReachAnnouncer() {
        let bs = makeBaseStation()
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(relayedJSON(rocketID: 7, state: "PRELAUNCH"))
        bs.parseTelemetryData(relayedJSON(rocketID: 7, state: "INFLIGHT"))
        bs.parseTelemetryData(relayedJSON(rocketID: 7, state: "LANDED"))

        XCTAssertEqual(spy.received.count, 3)
        XCTAssertEqual(spy.received.map(\.state),
                       ["PRELAUNCH", "INFLIGHT", "LANDED"])
    }

    /// Direct-rocket path must keep working — guards against a fix that
    /// breaks the case the bug report compared against.
    func testDirectRocketTelemetry_TriggersAnnouncer() {
        let rocket = makeRocket()
        let spy = SpyAnnouncer()
        rocket.flightAnnouncer = spy

        rocket.parseTelemetryData(directJSON())

        XCTAssertEqual(spy.received.count, 1)
        XCTAssertNil(spy.received.first?.source_rocket_id)
    }

    /// A BS receiving a frame with no `rid` (e.g. its own heartbeat) must
    /// still route through the announcer via the non-relay branch.
    func testBaseStation_SelfTelemetry_TriggersAnnouncer() {
        let bs = makeBaseStation()
        let spy = SpyAnnouncer()
        bs.flightAnnouncer = spy

        bs.parseTelemetryData(directJSON())

        XCTAssertEqual(spy.received.count, 1)
        XCTAssertEqual(bs.remoteRockets.count, 0,
                       "Non-relayed frames should not populate remoteRockets")
    }
}
