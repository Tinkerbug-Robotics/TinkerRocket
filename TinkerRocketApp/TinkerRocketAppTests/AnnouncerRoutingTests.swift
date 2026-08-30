import XCTest
@testable import TinkerRocketApp

/// Regression tests for the callouts going silent mid-flight when the operator
/// switched to another app (2026-08-29 range session).
///
/// The announcer used to be routed onto a link from `DashboardView`'s
/// `.onChange(of:)` handlers.  SwiftUI does not run those while the app is
/// backgrounded, and every reconnect builds a NEW `BLEDevice` whose
/// `flightAnnouncer` starts nil — so a link drop while the operator was filming
/// the launch left the announcer wired to a dead object, and it never
/// recovered: once the reconnect ladder had refilled `devices` the value read
/// the same as before, so the `onChange` never fired on resume either.
///
/// Routing lives on `BLEFleet` now.  Every test here drives it the way the
/// radio does — device state changes, no view anywhere — which is exactly the
/// path that was broken.
final class AnnouncerRoutingTests: XCTestCase {

    /// Stands in for `FlightAnnouncer` without touching AVFoundation.
    final class SpyAnnouncer: TelemetryAnnouncer {
        var received: [TelemetryData] = []
        func processTelemetry(_ telemetry: TelemetryData) { received.append(telemetry) }
        func reset() {}
    }

    // MARK: - Helpers

    /// "TR-R-" / "TR-B-" prefixes drive `BLEDeviceType.from(name:)`.
    private func makeRocket() -> BLEDevice { BLEDevice(peripheral: nil, name: "TR-R-Test") }
    private func makeBaseStation() -> BLEDevice { BLEDevice(peripheral: nil, name: "TR-B-Test") }

    /// What `BLEFleet.adopt` does, minus CoreBluetooth.  Order matters: the
    /// device must be in the roster before `isConnected` flips, because that
    /// setter is what re-routes.
    private func join(_ device: BLEDevice, to fleet: BLEFleet, connected: Bool = true) {
        device.fleet = fleet
        fleet.devices.append(device)
        device.isConnected = connected
    }

    private func relayedJSON(rocketID: Int) -> Data {
        """
        {"rid":\(rocketID),"run":"Atlas","st":"INFLIGHT","palt":120.5,"mspd":85.0}
        """.data(using: .utf8)!
    }

    // MARK: - Tests

    /// THE regression.  A direct rocket link always drops at launch — the
    /// rocket flies out of BLE range — and voice has to move to the base
    /// station with nothing on screen to notice.
    func testDirectRocketDropping_HandsVoiceToTheBaseStation() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let rocket = makeRocket()
        join(bs, to: fleet)
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        XCTAssertTrue(rocket.flightAnnouncer === spy,
                      "The direct rocket link speaks while it is up")
        XCTAssertNil(bs.flightAnnouncer)

        rocket.isConnected = false

        XCTAssertTrue(bs.flightAnnouncer === spy,
                      "Voice must fall back to the base station with no view update")
        XCTAssertNil(rocket.flightAnnouncer)
    }

    /// The symptom itself, end to end: after the drop, relayed frames must
    /// still reach the announcer.  This is what was silent in the field.
    func testAfterTheDrop_RelayedTelemetryStillReachesTheAnnouncer() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let rocket = makeRocket()
        join(bs, to: fleet)
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        rocket.isConnected = false
        bs.parseTelemetryData(relayedJSON(rocketID: 7))

        XCTAssertEqual(spy.received.count, 1,
                       "Callouts must survive the link handover, not just the link")
    }

    /// `BLEFleet.didConnect` builds a fresh `BLEDevice` on every reconnect
    /// (#375), so the announcer reference on the old object is worthless.
    func testReconnect_RoutesToTheNewDeviceObject() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let rocket = makeRocket()
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy
        XCTAssertTrue(rocket.flightAnnouncer === spy)

        // Drop, then the ladder reconnects — a different object entirely.
        rocket.isConnected = false
        fleet.devices.removeAll { $0 === rocket }
        let reconnected = makeRocket()
        join(reconnected, to: fleet)

        XCTAssertTrue(reconnected.flightAnnouncer === spy,
                      "The replacement object must be routed, not the dead one")
    }

    /// A device joins as `.unknown` and only resolves its role when the
    /// config_identity readback lands ~1 s later (#375) — after the roster
    /// change has already been handled.  Without an edge on `deviceType`, a
    /// rocket reconnecting mid-flight never takes voice back off the relay.
    func testTypeResolvingAfterJoin_TakesVoiceOffTheBaseStation() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        join(bs, to: fleet)
        fleet.flightAnnouncer = spy
        XCTAssertTrue(bs.flightAnnouncer === spy)

        // Renamed rocket: no TR- prefix, so the name parse yields .unknown.
        let renamed = BLEDevice(peripheral: nil, name: "Atlas")
        XCTAssertEqual(renamed.deviceType, .unknown)
        join(renamed, to: fleet)
        XCTAssertTrue(bs.flightAnnouncer === spy,
                      "An unresolved link must not take voice off the relay")

        renamed.deviceType = .rocket    // config_identity "dt":"R"

        XCTAssertTrue(renamed.flightAnnouncer === spy,
                      "A direct rocket link takes voice back once its role resolves")
        XCTAssertNil(bs.flightAnnouncer)
    }

    /// Only one link may hold the announcer — two would interleave callouts
    /// from two rockets.
    func testExactlyOneLinkHoldsTheAnnouncer() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let first = makeRocket()
        let second = makeRocket()
        join(bs, to: fleet)
        join(first, to: fleet)
        join(second, to: fleet)
        fleet.flightAnnouncer = spy

        let holders = fleet.devices.filter { $0.flightAnnouncer != nil }
        XCTAssertEqual(holders.count, 1)
        XCTAssertTrue(holders.first === first, "The first connected direct rocket wins")
    }
}
