//
//  MultiRocketWireTests.swift
//  TinkerRocketAppTests
//
//  Three rockets, one base station — what the wire says: the firmware-stamp
//  comparison, the transmit-slot readback and command, and the three-rocket
//  demo.
//

import XCTest
@testable import TinkerRocketApp

final class MultiRocketWireTests: XCTestCase {

    // MARK: - Firmware stamps

    func testGitSha_isThePartBeforeDirtyBoardBenchAndDate() {
        XCTAssertEqual(FirmwareStamp.gitSha(of: "ee4fe786+20261019-1830"), "ee4fe786")
        XCTAssertEqual(FirmwareStamp.gitSha(of: "ee4fe786-dirty+20261019-1830"), "ee4fe786")
        XCTAssertEqual(FirmwareStamp.gitSha(of: "ee4fe786-dirty-v9-bench+20261019-1830"), "ee4fe786")
        XCTAssertEqual(FirmwareStamp.gitSha(of: "ee4fe786-m1+20261019-1830"), "ee4fe786")
        XCTAssertNil(FirmwareStamp.gitSha(of: ""))
        XCTAssertNil(FirmwareStamp.gitSha(of: "unknown"))
        XCTAssertNil(FirmwareStamp.gitSha(of: nil))
    }

    func testMismatchLine_onlyWhenCommitsDiffer() {
        let same: [FirmwareStamp.Unit] = [
            .init(label: "Base 1", version: "ee4fe786-v9+20261019-1830"),
            .init(label: "Booster", version: "ee4fe786-dirty-v9-bench+20261019-1900"),
            .init(label: "Dart", version: nil),
        ]
        XCTAssertNil(FirmwareStamp.mismatchLine(same), "dirty, board and bench suffixes are not a mismatch")

        let differ = same + [.init(label: "Pad Rocket", version: "ab12cd34-v7+20261001-0900")]
        XCTAssertEqual(FirmwareStamp.mismatchLine(differ),
                       "Firmware differs — ee4fe786: Base 1, Booster · ab12cd34: Pad Rocket")
    }

    func testIdentityLine_boardFirmwareAndProvenance() {
        var rec = KnownDevice(unitID: "aaa")
        XCTAssertNil(FirmwareStamp.identityLine(for: rec), "nothing known, nothing shown")

        let now = Date()
        rec.ocFirmware = "ee4fe786-v9+20261019-1830"
        rec.fcFirmware = "ee4fe786-v9+20261019-1830"
        rec.ocBoard = "v9"
        rec.fcBoard = "v9"
        rec.firmwareSeen = now.addingTimeInterval(-2 * 86_400)
        let line = FirmwareStamp.identityLine(for: rec, now: now)
        XCTAssertEqual(line, "Board v9 · fw ee4fe786 · last seen over Bluetooth 2 days ago")

        rec.fcFirmware = "ab12cd34-v9+20261001-0900"
        rec.ocBoard = nil   // unprovisioned OC: fall back to the image's board
        XCTAssertTrue(FirmwareStamp.identityLine(for: rec, now: now)?
            .hasPrefix("Board v9 · OC fw ee4fe786 / FC fw ab12cd34") ?? false)
    }

    // MARK: - Transmit slots

    func testLtsReadback_hydratesTheBaseStationConfig() {
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.parseTelemetryData(#"{"type":"config","lf":915.0,"lsf":8,"lhd":true,"lts":true}"#
            .data(using: .utf8))
        XCTAssertEqual(bs.rocketConfig?.loraTxSlots, true)
        bs.parseTelemetryData(#"{"type":"config","lf":915.0,"lsf":8,"lhd":true,"lts":false}"#
            .data(using: .utf8))
        XCTAssertEqual(bs.rocketConfig?.loraTxSlots, false)
        bs.parseTelemetryData(#"{"type":"config","lf":915.0,"lsf":8,"lhd":true}"#.data(using: .utf8))
        XCTAssertNil(bs.rocketConfig?.loraTxSlots, "older firmware: unknown, so never sent the command")
    }

    func testSlotCommand_isDesiredStateWithALocalEcho() {
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.parseTelemetryData(#"{"type":"config","lsf":8,"lts":true}"#.data(using: .utf8))
        var sent: [(UInt8, [UInt8])] = []
        bs.commandTap = { sent.append(($0, [UInt8]($1))) }
        bs.sendLoRaTxSlots(false)
        bs.sendLoRaTxSlots(true)
        XCTAssertEqual(sent.map { $0.0 }, [82, 82], "BLE_BS_CMD_SET_TX_SLOTS — not 70, which is OTA_BEGIN")
        XCTAssertEqual(sent.map { $0.1 }, [[0], [1]])
        XCTAssertEqual(bs.rocketConfig?.loraTxSlots, true)
    }

    // MARK: - Demo: three rockets, wired into the fleet

    private func waitUntil(_ what: String, timeout: TimeInterval = 5,
                           _ predicate: () -> Bool) async throws {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if predicate() { return }
            try await Task.sleep(nanoseconds: 10_000_000)
        }
        XCTFail("timed out waiting for \(what)")
    }

    @MainActor
    func testVirtualBaseStation_threeRocketsThreeFixes_focusAndSlots() async throws {
        let suite = "MultiRocketWireTests-\(UUID().uuidString)"
        let defaults = UserDefaults(suiteName: suite)!
        defer { defaults.removePersistentDomain(forName: suite) }
        let fleet = BLEFleet(knownDevices: KnownDeviceStore(defaults: defaults))
        fleet.startVirtualRocket()
        defer { fleet.disconnectAll() }
        let bs = try XCTUnwrap(fleet.devices.first)

        try await waitUntil("three relayed rockets") { bs.remoteRockets.count == 3 }
        XCTAssertEqual(bs.focusRocketID, 1, "the first rocket heard is the focus")
        XCTAssertTrue(bs.fleet === fleet, "adopt() wired the back-reference")

        try await waitUntil("a latched fix per rocket") {
            Set(fleet.lastValidRocketFixes.keys).isSuperset(of: [
                RocketKey(networkID: 7, rocketID: 1),
                RocketKey(networkID: 7, rocketID: 2),
                RocketKey(networkID: 7, rocketID: 3)])
        }
        let pins = MapView.rocketPins(for: fleet.dashboardRockets, fixes: fleet.lastValidRocketFixes,
                                      focused: RocketKey(networkID: 7, rocketID: 1)) { _ in nil }
        XCTAssertEqual(pins.map(\.title).sorted(), ["Booster", "Dart", "Pad Rocket"])
        XCTAssertEqual(pins.filter(\.focused).map(\.title), ["Booster"])

        // The virtual base station answers the slot switch like the firmware.
        XCTAssertEqual(bs.rocketConfig?.loraTxSlots, true)
        bs.sendLoRaTxSlots(false)
        XCTAssertEqual(bs.rocketConfig?.loraTxSlots, false)
    }
}
