//
//  MultiRocketRegistryTests.swift
//  TinkerRocketAppTests
//
//  Three rockets, one base station — the registry side: next free rocket ID,
//  the "Same ID" badge, the board and firmware a
//  record keeps for a rocket later heard only over LoRa, and the profile a
//  relayed rocket's pyro cards and servo limits read.
//

import XCTest
@testable import TinkerRocketApp

final class MultiRocketRegistryTests: XCTestCase {

    private var suiteName: String!
    private var defaults: UserDefaults!
    private var tempDir: URL!

    override func setUpWithError() throws {
        suiteName = "MultiRocketRegistryTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("MultiRocketRegistryTests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        defaults.removePersistentDomain(forName: suiteName)
        try? FileManager.default.removeItem(at: tempDir)
    }

    private func makeStore() -> KnownDeviceStore { KnownDeviceStore(defaults: defaults) }
    private func makeFleet() -> BLEFleet { BLEFleet(knownDevices: makeStore()) }

    private func addRocket(_ store: KnownDeviceStore, _ unitID: String, name: String,
                           nid: UInt8 = 7, rid: UInt8) {
        store.deviceDidReportIdentity(unitID: unitID, name: name, deviceType: .rocket,
                                      networkID: nid, rocketID: rid, pusher: nil)
    }

    // MARK: - Next free rocket ID

    func testRocketIDsInUse_otherRocketsOnTheSameNetworkOnly() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        addRocket(store, "bbb", name: "Dart", rid: 2)
        addRocket(store, "ccc", name: "Elsewhere", nid: 9, rid: 3)
        store.deviceDidReportIdentity(unitID: "bs1", name: "Base", deviceType: .baseStation,
                                      networkID: 7, rocketID: 0, pusher: nil)

        XCTAssertEqual(store.rocketIDsInUse(networkID: 7, excluding: "new"), [1, 2])
        XCTAssertEqual(store.rocketIDsInUse(networkID: 7, excluding: "aaa"), [2],
                       "the rocket being set up does not block its own ID")
    }

    func testRocketIDsInUse_countsAQueuedIDChange() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        store.setRocketID(4, for: "aaa", pusher: nil)   // offline: queued
        XCTAssertEqual(store.rocketIDsInUse(networkID: 7, excluding: "new"), [4],
                       "the target ID is the one that will collide")
    }

    func testNextFreeRocketID_lowestFromOne_includingWhatTheFleetHears() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        addRocket(store, "bbb", name: "Dart", rid: 3)
        XCTAssertEqual(store.nextFreeRocketID(networkID: 7, excluding: "new"), 2)
        XCTAssertEqual(store.nextFreeRocketID(networkID: 7, excluding: "new", alsoInUse: [2]), 4)
        XCTAssertEqual(store.nextFreeRocketID(networkID: 8, excluding: "new"), 1)
    }

    func testProvisioningRocketID_keepsAFreeReportedID_elseNextFree() {
        XCTAssertEqual(KnownDeviceStore.provisioningRocketID(reported: 5, taken: [1, 2]), 5)
        XCTAssertEqual(KnownDeviceStore.provisioningRocketID(reported: 1, taken: [1, 2]), 3,
                       "a factory-default ID 1 already in use must not be pre-filled")
        XCTAssertEqual(KnownDeviceStore.provisioningRocketID(reported: 0, taken: [1]), 2)
        XCTAssertEqual(KnownDeviceStore.provisioningRocketID(reported: 255, taken: []), 1)
    }

    func testFleetRocketIDsInUse_includesRocketsHeardOnlyThroughTheBaseStation() {
        let fleet = makeFleet()
        let store = fleet.knownDevices
        addRocket(store, "aaa", name: "Booster", rid: 1)

        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.fleet = fleet
        bs.isConnected = true
        bs.networkID = 7
        fleet.devices = [bs]
        bs.parseTelemetryData(#"{"rid":3,"run":"Stranger","st":"READY"}"#.data(using: .utf8))

        XCTAssertEqual(fleet.rocketIDsInUse(networkID: 7, excludingUnitID: "new"), [1, 3],
                       "a rocket with no registry record still holds the ID it transmits under")
        XCTAssertEqual(KnownDeviceStore.provisioningRocketID(
            reported: 1, taken: fleet.rocketIDsInUse(networkID: 7, excludingUnitID: "new")), 2)
    }

    // MARK: - Badges

    func testConflicts_sameIDOnly() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        addRocket(store, "bbb", name: "Twin", rid: 1)
        addRocket(store, "ccc", name: "Fourth", rid: 4)     // asks for rid 1's slot: not a conflict
        addRocket(store, "ddd", name: "Dart", rid: 2)
        addRocket(store, "eee", name: "Far", nid: 9, rid: 1) // other network

        let c = store.rocketIDConflicts(for: "aaa")
        XCTAssertEqual(c.sameID.map(\.unitID), ["bbb"])
        XCTAssertTrue(store.rocketIDConflicts(for: "ccc").sameID.isEmpty,
                      "the base station gives a rocket whose slot is taken a free one")
        XCTAssertTrue(store.rocketIDConflicts(for: "ddd").sameID.isEmpty)

        let badges = DeviceManagerView.conflictBadges(c)
        XCTAssertEqual(badges.map(\.text), ["Same ID as Twin"])
        XCTAssertEqual(badges.map(\.severe), [true])
    }

    func testConflicts_threeRocketsOnOneToThree_haveNone() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        addRocket(store, "bbb", name: "Pad Rocket", rid: 2)
        addRocket(store, "ccc", name: "Dart", rid: 3)
        for id in ["aaa", "bbb", "ccc"] {
            XCTAssertTrue(DeviceManagerView.conflictBadges(store.rocketIDConflicts(for: id)).isEmpty)
        }
    }

    // MARK: - Board + firmware per rocket

    func testFirmwareAndBoards_recordedFromBothReadbacks_andPersisted() {
        let fleet = makeFleet()
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        rocket.fleet = fleet

        // fc_identity can arrive before config_identity names the record.
        rocket.parseTelemetryData(#"""
            {"type":"fc_identity","fc_fw":"ab12cd34-v9+20261019-1200","fc_board":"v9","oc_board":"v9"}
            """#.data(using: .utf8))
        rocket.parseTelemetryData(#"""
            {"type":"config_identity","uid":"aaa","un":"Booster","nid":7,"rid":1,"dt":"R",
             "fw":"ee4fe786-dirty-v9+20261019-1830"}
            """#.data(using: .utf8))

        let rec = fleet.knownDevices.device(for: "aaa")
        XCTAssertEqual(rec?.ocFirmware, "ee4fe786-dirty-v9+20261019-1830")
        XCTAssertEqual(rec?.fcFirmware, "ab12cd34-v9+20261019-1200")
        XCTAssertEqual(rec?.fcBoard, "v9")
        XCTAssertEqual(rec?.ocBoard, "v9")
        XCTAssertNotNil(rec?.firmwareSeen)

        // An OC that cannot reach the FC keeps the last good FC version.
        rocket.parseTelemetryData(#"{"type":"fc_identity","fc_fw":"unknown"}"#.data(using: .utf8))
        XCTAssertEqual(fleet.knownDevices.device(for: "aaa")?.fcFirmware, "ab12cd34-v9+20261019-1200")
    }

    func testKnownDevice_decodesARecordWrittenBeforeTheFirmwareFields() throws {
        let old = #"[{"unitID":"aaa","name":"Booster","deviceType":"R","networkID":7,"rocketID":1,"provisioned":true}]"#
        defaults.set(old.data(using: .utf8), forKey: "knownDevices.v1")
        let store = makeStore()
        let rec = try XCTUnwrap(store.device(for: "aaa"))
        XCTAssertEqual(rec.rocketID, 1)
        XCTAssertNil(rec.ocFirmware)
        XCTAssertNil(rec.fcBoard)
    }

    func testRocketLookup_byReportedIdentity_nameBreaksATie() {
        let store = makeStore()
        addRocket(store, "aaa", name: "Booster", rid: 1)
        addRocket(store, "bbb", name: "Twin", rid: 1)
        addRocket(store, "ccc", name: "Dart", rid: 3)

        XCTAssertEqual(store.rocket(networkID: 7, rocketID: 3)?.unitID, "ccc")
        XCTAssertNil(store.rocket(networkID: 7, rocketID: 1), "two claimants and no name: no guess")
        XCTAssertEqual(store.rocket(networkID: 7, rocketID: 1, name: "Twin")?.unitID, "bbb")
        XCTAssertNil(store.rocket(networkID: 7, rocketID: 2))
        XCTAssertNil(store.rocket(networkID: 9, rocketID: 3))
    }

    // MARK: - Profile per rocket

    func testProfileLookup_relayedRocketGetsItsOwnBoundProfile_neverTheActiveOne() {
        let known = makeStore()
        addRocket(known, "aaa", name: "Booster", rid: 1)
        addRocket(known, "bbb", name: "Dart", rid: 2)
        addRocket(known, "ccc", name: "Unbound", rid: 3)

        let profiles = RocketProfileStore(directory: tempDir, defaults: defaults)
        let booster = profiles.add(name: "Booster")
        let dart = profiles.add(name: "Dart")
        let other = profiles.add(name: "Last direct link")
        profiles.bind(booster.id, toUnitID: "aaa")
        profiles.bind(dart.id, toUnitID: "bbb")
        profiles.setActive(other.id)

        XCTAssertEqual(profiles.profile(networkID: 7, rocketID: 2, knownDevices: known)?.id, dart.id)
        XCTAssertEqual(profiles.profile(networkID: 7, rocketID: 1, knownDevices: known)?.id, booster.id)
        XCTAssertNil(profiles.profile(networkID: 7, rocketID: 3, knownDevices: known),
                     "a board with no profile has none — not the active one")
        XCTAssertNil(profiles.profile(networkID: 7, rocketID: 4, knownDevices: known),
                     "a rocket never on a direct link has no record, so no profile")
    }

    func testProfileLookup_twoClaimantsResolveToTheMostRecentlyUpdated() {
        var older = RocketProfile.makeDefault(name: "Older")
        older.lastUsedUnitID = "aaa"
        older.updatedAt = Date(timeIntervalSince1970: 1_000)
        var newer = RocketProfile.makeDefault(name: "Newer")
        newer.lastUsedUnitID = "aaa"
        newer.updatedAt = Date(timeIntervalSince1970: 2_000)

        XCTAssertEqual(RocketProfileStore.profile(boundToUnitID: "aaa", in: [newer, older])?.name, "Newer")
        XCTAssertEqual(RocketProfileStore.profile(boundToUnitID: "aaa", in: [older, newer])?.name, "Newer")
        XCTAssertNil(RocketProfileStore.profile(boundToUnitID: "", in: [older, newer]))
    }

    func testUnboundRelayServoReason_isTheOperatorFacingCopy() {
        XCTAssertEqual(ServoTestView.unboundRelayReason,
                       "Connect to this rocket over Bluetooth once to bind its profile")
    }
}
