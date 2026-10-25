import XCTest
@testable import TinkerRocketApp

/// Regression tests for the callouts going silent mid-flight when the operator
/// switched to another app (2026-08-29 range session), and for the callouts
/// following the direct rocket link instead of the base station.
///
/// The announcer used to be routed onto a link from `DashboardView`'s
/// `.onChange(of:)` handlers.  SwiftUI does not run those while the app is
/// backgrounded, and every reconnect builds a NEW `BLEDevice` whose
/// `flightAnnouncer` starts nil — so a link drop while the operator was filming
/// the launch left the announcer wired to a dead object, and it never
/// recovered: once the reconnect ladder had refilled `devices` the value read
/// the same as before, so the `onChange` never fired on resume either.
///
/// Routing lives on `BLEFleet` now, and it routes to the foreground base
/// station only.  A direct rocket link used to be preferred, which tied voice
/// to a link that always leaves range at launch.  Every test here drives the
/// routing the way the radio does — device state changes, no view anywhere.
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

    private func directJSON() -> Data {
        """
        {"st":"INFLIGHT","palt":120.5,"mspd":85.0}
        """.data(using: .utf8)!
    }

    // MARK: - Tests

    /// THE regression.  With the rocket paired directly on the pad as well as
    /// the base station, voice must stay on the base station.
    func testDirectRocketConnected_VoiceStaysOnTheBaseStation() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let rocket = makeRocket()
        join(bs, to: fleet)
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        XCTAssertTrue(bs.flightAnnouncer === spy,
                      "The base station speaks even while a direct rocket link is up")
        XCTAssertNil(rocket.flightAnnouncer)
    }

    /// Connect order must not matter: a rocket paired first and a base station
    /// joining later still ends with the base station holding voice.
    func testBaseStationJoiningAfterTheRocket_TakesVoice() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let rocket = makeRocket()
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy
        XCTAssertNil(rocket.flightAnnouncer, "A direct rocket link never speaks")

        let bs = makeBaseStation()
        join(bs, to: fleet)

        XCTAssertTrue(bs.flightAnnouncer === spy)
        XCTAssertNil(rocket.flightAnnouncer)
    }

    /// No base station, no callouts — a rocket on its own does not speak.
    func testRocketOnly_NobodySpeaks() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let rocket = makeRocket()
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        XCTAssertNil(fleet.voiceDevice)
        XCTAssertNil(rocket.flightAnnouncer)

        rocket.parseTelemetryData(directJSON())
        XCTAssertTrue(spy.received.isEmpty)
    }

    /// The direct link always drops at launch (the rocket flies out of BLE
    /// range), and it flaps back in whenever the rocket is briefly close.
    /// Neither edge may move voice off the base station.
    func testDirectRocketDroppingAndReconnecting_NeverMovesVoice() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let rocket = makeRocket()
        join(bs, to: fleet)
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        rocket.isConnected = false
        XCTAssertTrue(bs.flightAnnouncer === spy)

        fleet.devices.removeAll { $0 === rocket }
        let reconnected = makeRocket()
        join(reconnected, to: fleet)

        XCTAssertTrue(bs.flightAnnouncer === spy,
                      "A rocket reconnecting mid-flight must not take voice")
        XCTAssertNil(reconnected.flightAnnouncer)
    }

    /// The symptom end to end: with a direct link up, only the relayed frames
    /// reach the announcer, never the direct link's own.
    func testOnlyRelayedTelemetryReachesTheAnnouncer() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let rocket = makeRocket()
        join(bs, to: fleet)
        join(rocket, to: fleet)
        fleet.flightAnnouncer = spy

        rocket.parseTelemetryData(directJSON())
        bs.parseTelemetryData(relayedJSON(rocketID: 7))

        XCTAssertEqual(spy.received.count, 1)
        XCTAssertEqual(spy.received.first?.source_rocket_id, 7)
    }

    /// `BLEFleet.didConnect` builds a fresh `BLEDevice` on every reconnect
    /// (#375), so the announcer reference on the old object is worthless.
    func testBaseStationReconnect_RoutesToTheNewDeviceObject() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        join(bs, to: fleet)
        fleet.flightAnnouncer = spy
        XCTAssertTrue(bs.flightAnnouncer === spy)

        // Drop, then the ladder reconnects — a different object entirely.
        bs.isConnected = false
        XCTAssertNil(bs.flightAnnouncer)
        fleet.devices.removeAll { $0 === bs }
        let reconnected = makeBaseStation()
        join(reconnected, to: fleet)

        XCTAssertTrue(reconnected.flightAnnouncer === spy,
                      "The replacement object must be routed, not the dead one")
    }

    /// A device joins as `.unknown` and only resolves its role when the
    /// config_identity readback lands ~1 s later (#375).  A renamed rocket
    /// resolving its role must leave voice where it is.
    func testRocketTypeResolvingAfterJoin_LeavesVoiceOnTheBaseStation() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        join(bs, to: fleet)
        fleet.flightAnnouncer = spy

        // Renamed rocket: no TR- prefix, so the name parse yields .unknown.
        let renamed = BLEDevice(peripheral: nil, name: "Atlas")
        XCTAssertEqual(renamed.deviceType, .unknown)
        join(renamed, to: fleet)
        renamed.deviceType = .rocket    // config_identity "dt":"R"

        XCTAssertTrue(bs.flightAnnouncer === spy)
        XCTAssertNil(renamed.flightAnnouncer)
    }

    /// A renamed base station joins as `.unknown` too; voice arrives once its
    /// role resolves, with no view update.
    func testBaseStationTypeResolvingAfterJoin_TakesVoice() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let renamed = BLEDevice(peripheral: nil, name: "Hilltop")
        XCTAssertEqual(renamed.deviceType, .unknown)
        join(renamed, to: fleet)
        fleet.flightAnnouncer = spy
        XCTAssertNil(renamed.flightAnnouncer, "An unresolved link does not speak")

        renamed.deviceType = .baseStation    // config_identity "dt":"B"

        XCTAssertTrue(renamed.flightAnnouncer === spy)
    }

    /// Only one link may hold the announcer — two would interleave callouts
    /// from two rockets.
    func testExactlyOneLinkHoldsTheAnnouncer() {
        let fleet = BLEFleet()
        let spy = SpyAnnouncer()
        let bs = makeBaseStation()
        let first = makeRocket()
        let second = makeRocket()
        join(first, to: fleet)
        join(bs, to: fleet)
        join(second, to: fleet)
        fleet.flightAnnouncer = spy

        let holders = fleet.devices.filter { $0.flightAnnouncer != nil }
        XCTAssertEqual(holders.count, 1)
        XCTAssertTrue(holders.first === bs, "The base station is the only link that speaks")
    }
}
