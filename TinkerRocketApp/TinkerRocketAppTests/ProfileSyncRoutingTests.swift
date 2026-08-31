import XCTest
@testable import TinkerRocketApp

/// The profile syncer's half of the #989 defect: its binding to the live link
/// was also decided in `DashboardView.attachActiveDevice()`, so a reconnect
/// while the app was backgrounded left it attached to a dead `BLEDevice`.
///
/// Quieter than the voice bug — an unsynced rocket shows `.awaitingSync`
/// rather than going silent — but the same defect, and #375 exists precisely
/// because a rocket that never synced reached the pad.
///
/// Every test drives the binding the way the radio does: device state changes,
/// no view anywhere.
final class ProfileSyncRoutingTests: XCTestCase {

    /// Records what the fleet hands to the syncer, without building a real
    /// `ActiveRocketSyncer` and `RocketProfileStore`.
    final class BinderSpy {
        private(set) var bound: [BLEDevice?] = []
        var latest: BLEDevice?? { bound.last }
        func callback() -> (BLEDevice?) -> Void {
            { [weak self] device in self?.bound.append(device) }
        }
    }

    private func makeRocket() -> BLEDevice { BLEDevice(peripheral: nil, name: "TR-R-Test") }
    private func makeBaseStation() -> BLEDevice { BLEDevice(peripheral: nil, name: "TR-B-Test") }

    /// What `BLEFleet.adopt` does, minus CoreBluetooth.  Roster first, then
    /// the connect edge — that setter is what re-routes.
    private func join(_ device: BLEDevice, to fleet: BLEFleet, connected: Bool = true) {
        device.fleet = fleet
        fleet.devices.append(device)
        device.isConnected = connected
    }

    // MARK: - Tests

    /// THE regression: the link drops at launch and the syncer must let go,
    /// with nothing on screen to notice.
    func testDirectRocketDropping_UnbindsTheSyncer() {
        let fleet = BLEFleet()
        let spy = BinderSpy()
        let rocket = makeRocket()
        join(rocket, to: fleet)
        fleet.onDirectRocketChange = spy.callback()

        XCTAssertTrue(spy.latest ?? nil === rocket, "Binds the direct link while it is up")

        rocket.isConnected = false

        XCTAssertNil(spy.latest ?? nil,
                     "Must unbind on the drop, with no view update to trigger it")
    }

    /// `BLEFleet.didConnect` builds a fresh object on every reconnect (#375),
    /// and `attach` deliberately falls through to a full re-attach for one.
    func testReconnect_BindsTheNewDeviceObject() {
        let fleet = BLEFleet()
        let spy = BinderSpy()
        let rocket = makeRocket()
        join(rocket, to: fleet)
        fleet.onDirectRocketChange = spy.callback()

        rocket.isConnected = false
        fleet.devices.removeAll { $0 === rocket }
        let reconnected = makeRocket()
        join(reconnected, to: fleet)

        XCTAssertTrue(spy.latest ?? nil === reconnected,
                      "The replacement object must be bound, not the dead one")
    }

    /// A device joins as `.unknown` and only resolves when config_identity
    /// lands ~1 s later.  #375's "different door": a rocket first mis-read as
    /// a base station keeps profile sync silently disabled all session.
    func testTypeResolvingAfterJoin_BindsOnceTheRoleIsKnown() {
        let fleet = BLEFleet()
        let spy = BinderSpy()
        let renamed = BLEDevice(peripheral: nil, name: "Atlas")   // no TR- prefix
        XCTAssertEqual(renamed.deviceType, .unknown)
        join(renamed, to: fleet)
        fleet.onDirectRocketChange = spy.callback()

        XCTAssertNil(spy.latest ?? nil, "An unresolved role does not bind")

        renamed.deviceType = .rocket    // config_identity "dt":"R"

        XCTAssertTrue(spy.latest ?? nil === renamed,
                      "Profile sync must start once the role resolves")
    }

    /// Profiles only push over a direct link — a base-station relay is a
    /// read-only display and never receives one.
    func testBaseStationOnly_NeverBinds() {
        let fleet = BLEFleet()
        let spy = BinderSpy()
        let bs = makeBaseStation()
        join(bs, to: fleet)
        fleet.onDirectRocketChange = spy.callback()

        XCTAssertNil(spy.latest ?? nil)
        XCTAssertNil(fleet.directRocket)
        XCTAssertTrue(fleet.voiceDevice === bs,
                      "Voice still follows the relay even though sync does not")
    }
}
