import XCTest
@testable import TinkerRocketApp

/// 2026-10-16, a V9 updated over BLE: the out computer flashed fine, but then
/// "Flash Flight Computer" stayed grayed out with a matching FC image picked,
/// until the operator left Settings and came back in.
///
/// The firmware screen held the BLEDevice that Settings was opened with. The
/// OC reboots after its own flash, and BLEFleet replaces the BLEDevice on every
/// disconnect/reconnect, so the held one went dead: `isConnected` false for
/// good, which is one of the Flash button's disable terms. The screen now runs
/// on `BLEFleet.liveDevice(for:)`. These tests drive the fleet the way the radio
/// does, no view anywhere.
@MainActor
final class FirmwareUpdateLiveDeviceTests: XCTestCase {

    /// A BLEDevice standing in for one on a real link. A CBPeripheral can't be
    /// built in a test, so the peripheral identity is injected instead.
    final class LinkedDevice: BLEDevice {
        private let linkID: UUID
        init(_ linkID: UUID, name: String = "TR-R-Test") {
            self.linkID = linkID
            super.init(peripheral: nil, name: name)
        }
        override var peripheralID: UUID? { linkID }
    }

    /// What `BLEFleet.didConnect` does, minus CoreBluetooth.
    private func connect(_ device: BLEDevice, to fleet: BLEFleet) {
        fleet.devices.append(device)
        device.isConnected = true
    }

    /// What `BLEFleet.didDisconnectPeripheral` does: the instance goes dead and
    /// leaves the roster. The reconnect, if any, builds a new one.
    private func drop(_ device: BLEDevice, from fleet: BLEFleet) {
        device.isConnected = false
        fleet.devices.removeAll { $0 === device }
    }

    // MARK: - Tests

    /// THE regression. Open the screen on the OC, let its flash reboot it, and
    /// the instance the screen runs on must be the rebuilt, connected one.
    func testAfterTheOcReboots_theScreenRunsOnTheRebuiltInstance() {
        let fleet = BLEFleet()
        let oc = UUID()
        let opened = LinkedDevice(oc)
        connect(opened, to: fleet)
        XCTAssertTrue(fleet.liveDevice(for: opened) === opened)

        // The reboot. Until the OC is back there is nothing live to follow,
        // and the opened instance, now disconnected, is what the screen shows.
        drop(opened, from: fleet)
        XCTAssertTrue(fleet.liveDevice(for: opened) === opened)
        XCTAssertFalse(fleet.liveDevice(for: opened).isConnected)

        let rebuilt = LinkedDevice(oc)
        connect(rebuilt, to: fleet)

        let live = fleet.liveDevice(for: opened)
        XCTAssertTrue(live === rebuilt, "The screen must follow the reconnect, not keep the dead copy")
        XCTAssertTrue(live.isConnected,
                      "The Flash button is disabled while this reads false — the reported symptom")
        XCTAssertFalse(opened.isConnected, "The opened instance never comes back")
    }

    /// The identity rows and the FC image verdict read the device too. They
    /// must come off the rebuilt instance, which carries what the OC published
    /// after the reboot, not the values frozen in the dead one.
    func testIdentityIsReadOffTheRebuiltInstance() {
        let fleet = BLEFleet()
        let oc = UUID()
        let opened = LinkedDevice(oc)
        connect(opened, to: fleet)
        opened.firmwareVersion = "oc-before"
        opened.fcFirmwareVersion = "fc-before"

        drop(opened, from: fleet)
        let rebuilt = LinkedDevice(oc)
        connect(rebuilt, to: fleet)
        rebuilt.firmwareVersion = "oc-after"
        rebuilt.fcFirmwareVersion = "fc-relayed"
        rebuilt.fcBoardRev = "v9"

        let live = fleet.liveDevice(for: opened)
        XCTAssertEqual(live.firmwareVersion, "oc-after")
        XCTAssertEqual(live.fcFirmwareVersion, "fc-relayed")
        XCTAssertEqual(live.fcBoardRev, "v9")
    }

    /// The other half of the screen: the OTA session, and the "Updated
    /// successfully" result it holds, must be the same object on both sides of
    /// the swap — that is why it lives on the fleet. And it must flash through
    /// the instance the screen gates on, so the button and the flash agree.
    func testTheSessionAndItsResultCarryAcrossTheSwap() {
        let fleet = BLEFleet()
        let oc = UUID()
        let opened = LinkedDevice(oc)
        connect(opened, to: fleet)
        let session = fleet.otaSession(for: opened)
        XCTAssertTrue(session.device === opened)

        drop(opened, from: fleet)
        XCTAssertNil(session.device, "Between the drop and the reconnect there is no link")
        let rebuilt = LinkedDevice(oc)
        connect(rebuilt, to: fleet)

        let live = fleet.liveDevice(for: opened)
        XCTAssertTrue(fleet.otaSession(for: live) === session,
                      "A new session here would wipe the OC flash's result off the screen")
        XCTAssertTrue(session.device === live, "The flash goes out over the link the button reads")
    }

    /// A base station and a rocket connected together: each screen follows its
    /// own peripheral, and a dropped one is never stood in for by the other.
    func testAnotherPeripheralIsNeverPickedUp() {
        let fleet = BLEFleet()
        let bs = LinkedDevice(UUID(), name: "TR-B-Test")
        let oc = LinkedDevice(UUID())
        connect(bs, to: fleet)
        connect(oc, to: fleet)
        XCTAssertTrue(fleet.liveDevice(for: bs) === bs)
        XCTAssertTrue(fleet.liveDevice(for: oc) === oc)

        drop(oc, from: fleet)
        XCTAssertTrue(fleet.liveDevice(for: oc) === oc,
                      "Still connected, the base station must not stand in for the dropped rocket")
        XCTAssertFalse(fleet.liveDevice(for: oc).isConnected)
    }

    /// The virtual rocket has no peripheral, so there is nothing to resolve by.
    /// Its one instance lives as long as the demo; it resolves to itself.
    func testAPeripheralLessDeviceResolvesToItself() {
        let fleet = BLEFleet()
        let virtual = BLEDevice(peripheral: nil, name: "TR-B-Virtual")
        let other = BLEDevice(peripheral: nil, name: "TR-R-Test")
        connect(other, to: fleet)
        XCTAssertTrue(fleet.liveDevice(for: virtual) === virtual)
        connect(virtual, to: fleet)
        XCTAssertTrue(fleet.liveDevice(for: virtual) === virtual,
                      "Two nil identities must not match each other")
    }
}
