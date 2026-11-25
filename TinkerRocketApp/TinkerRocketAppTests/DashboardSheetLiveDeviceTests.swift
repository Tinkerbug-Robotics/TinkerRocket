import XCTest
@testable import TinkerRocketApp

/// The dashboard's sheets after a reconnect. The firmware screen was the first
/// to be fixed (FirmwareUpdateLiveDeviceTests); the sheets themselves had the
/// same defect. Each one was handed the BLEDevice it was opened with
/// (`DashboardSheet`'s payload), and BLEFleet replaces that object on every
/// disconnect/reconnect, so after any reconnect an open sheet read a dead copy:
/// Settings kept the out computer's old firmware version with "Update
/// firmware…" disabled after an OC flash, and everything gated on
/// `isConnected` went nowhere until the sheet was closed and opened again.
///
/// The sheets now run on `DashboardSheet.live(in:)`. These tests drive the
/// fleet the way the radio does, no view anywhere.
@MainActor
final class DashboardSheetLiveDeviceTests: XCTestCase {

    /// A BLEDevice on a real link, by peripheral identity (a CBPeripheral
    /// can't be built in a test).
    private typealias LinkedDevice = FirmwareUpdateLiveDeviceTests.LinkedDevice

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

    /// Every sheet that talks to a device, as the dashboard opens it.
    private let deviceSheets: [(name: String, open: (BLEDevice) -> DashboardSheet)] = [
        ("settings", DashboardSheet.settings),
        ("simulator", DashboardSheet.simulator),
        ("servoTest", DashboardSheet.servoTest),
        ("frequencyScan", DashboardSheet.frequencyScan),
        ("preflight", DashboardSheet.preflight),
    ]

    private func device(of sheet: DashboardSheet) -> BLEDevice? {
        switch sheet {
        case .settings(let d), .simulator(let d), .servoTest(let d),
             .frequencyScan(let d), .preflight(let d):
            return d
        case .driftCast:
            return nil
        }
    }

    // MARK: - Tests

    /// THE regression, for every sheet: open it, let the link drop and come
    /// back, and the sheet must run on the rebuilt, connected instance.
    func testEverySheetFollowsTheReconnect() {
        for (name, open) in deviceSheets {
            let fleet = BLEFleet()
            let link = UUID()
            let opened = LinkedDevice(link)
            connect(opened, to: fleet)
            let sheet = open(opened)
            XCTAssertTrue(device(of: sheet.live(in: fleet)) === opened, name)

            drop(opened, from: fleet)
            let rebuilt = LinkedDevice(link)
            connect(rebuilt, to: fleet)

            let live = sheet.live(in: fleet)
            XCTAssertTrue(device(of: live) === rebuilt,
                          "\(name): the sheet must follow the reconnect, not keep the dead copy")
            XCTAssertEqual(device(of: live)?.isConnected, true, name)
            XCTAssertFalse(opened.isConnected, "\(name): the opened instance never comes back")
        }
    }

    /// The sheet is identified by the link, not the instance. The dashboard
    /// keeps the opened case in `activeSheet` and resolves it on every render,
    /// so the id can't change on a swap -- and if a case were ever rebuilt from
    /// the live device, it must still name the same sheet, or SwiftUI would
    /// dismiss it and present it again.
    func testTheSheetIdNamesTheLinkNotTheInstance() {
        let link = UUID()
        let opened = LinkedDevice(link)
        let rebuilt = LinkedDevice(link)
        let other = LinkedDevice(UUID())
        for (name, open) in deviceSheets {
            XCTAssertEqual(open(opened).id, open(rebuilt).id, name)
            XCTAssertNotEqual(open(opened).id, open(other).id, name)
        }
        XCTAssertNotEqual(DashboardSheet.settings(opened).id, DashboardSheet.servoTest(opened).id)

        let fleet = BLEFleet()
        connect(opened, to: fleet)
        drop(opened, from: fleet)
        connect(rebuilt, to: fleet)
        let sheet = DashboardSheet.settings(opened)
        XCTAssertEqual(sheet.live(in: fleet).id, sheet.id)
    }

    /// The reported case. After an out-computer flash the OC reboots; Settings
    /// must read the version the OC publishes after the reboot, and enable
    /// "Update firmware…" (disabled while `isConnected` reads false) for the
    /// flight-computer flash that follows.
    func testSettingsAfterAnOcFlash_readsTheRebootedOc() {
        let fleet = BLEFleet()
        let oc = UUID()
        let opened = LinkedDevice(oc)
        connect(opened, to: fleet)
        opened.firmwareVersion = "oc-before"
        let sheet = DashboardSheet.settings(opened)

        drop(opened, from: fleet)
        let rebuilt = LinkedDevice(oc)
        connect(rebuilt, to: fleet)
        rebuilt.firmwareVersion = "oc-after"

        guard case .settings(let live) = sheet.live(in: fleet) else {
            return XCTFail("Settings must stay Settings")
        }
        XCTAssertEqual(live.firmwareVersion, "oc-after", "The Version row")
        XCTAssertTrue(live.isConnected, "\"Update firmware…\" and every config push gate on this")
        XCTAssertEqual(opened.firmwareVersion, "oc-before", "What the sheet showed before")
    }

    /// Between the drop and the reconnect there is no live instance. The
    /// opened one, disconnected, is the honest answer: its buttons disable.
    func testBetweenTheDropAndTheReconnect_theSheetShowsTheLinkDown() {
        for (name, open) in deviceSheets {
            let fleet = BLEFleet()
            let opened = LinkedDevice(UUID())
            connect(opened, to: fleet)
            let sheet = open(opened)

            drop(opened, from: fleet)
            let live = sheet.live(in: fleet)
            XCTAssertTrue(device(of: live) === opened, name)
            XCTAssertEqual(device(of: live)?.isConnected, false, name)
        }
    }

    /// A base station and a rocket connected together: a sheet opened on the
    /// rocket never lands on the base station while the rocket is down, and
    /// follows the rocket when it is back.
    func testAnotherPeripheralIsNeverPickedUp() {
        let fleet = BLEFleet()
        let bs = LinkedDevice(UUID(), name: "TR-B-Test")
        let rocketLink = UUID()
        let rocket = LinkedDevice(rocketLink)
        connect(bs, to: fleet)
        connect(rocket, to: fleet)
        let onRocket = DashboardSheet.servoTest(rocket)
        let onBS = DashboardSheet.frequencyScan(bs)

        drop(rocket, from: fleet)
        XCTAssertTrue(device(of: onRocket.live(in: fleet)) === rocket,
                      "The base station must not stand in for the dropped rocket")
        XCTAssertTrue(device(of: onBS.live(in: fleet)) === bs)

        let rebuilt = LinkedDevice(rocketLink)
        connect(rebuilt, to: fleet)
        XCTAssertTrue(device(of: onRocket.live(in: fleet)) === rebuilt)
        XCTAssertTrue(device(of: onBS.live(in: fleet)) === bs)
    }

    /// Drift Cast carries no device (it reads the active one itself).
    func testDriftCastStaysDriftCast() {
        let fleet = BLEFleet()
        guard case .driftCast = DashboardSheet.driftCast.live(in: fleet) else {
            return XCTFail("Drift Cast must stay Drift Cast")
        }
        XCTAssertEqual(DashboardSheet.driftCast.live(in: fleet).id, "driftCast")
    }
}
