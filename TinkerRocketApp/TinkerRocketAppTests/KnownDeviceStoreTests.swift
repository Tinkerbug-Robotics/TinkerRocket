//
//  KnownDeviceStoreTests.swift
//  TinkerRocketAppTests
//
//  Registry semantics for the "My Devices" screen: readback upsert, the
//  offline pending-edit queue and its push-on-next-connect behavior, the
//  provisioning gate, and the legacy knownDeviceIDs migration.
//

import XCTest
@testable import TinkerRocketApp

/// Records pushes instead of writing to a CBPeripheral.
private final class PusherSpy: DeviceIdentityPusher {
    var names: [String] = []
    var nids: [UInt8] = []
    var rids: [UInt8] = []
    func sendSetUnitName(_ name: String) { names.append(name) }
    func sendSetNetworkID(_ nid: UInt8) { nids.append(nid) }
    func sendSetRocketID(_ rid: UInt8) { rids.append(rid) }
}

final class KnownDeviceStoreTests: XCTestCase {

    private var suiteName: String!
    private var defaults: UserDefaults!

    override func setUpWithError() throws {
        suiteName = "KnownDeviceTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
    }

    override func tearDownWithError() throws {
        defaults.removePersistentDomain(forName: suiteName)
    }

    private func makeStore() -> KnownDeviceStore {
        KnownDeviceStore(defaults: defaults)
    }

    /// Simulate a config_identity readback for a rocket.
    private func report(_ store: KnownDeviceStore, unitID: String = "a1b2c3d4",
                        name: String = "Atlas", type: BLEDeviceType = .rocket,
                        nid: UInt8 = 42, rid: UInt8 = 3,
                        pusher: DeviceIdentityPusher? = nil) {
        store.deviceDidReportIdentity(unitID: unitID, name: name, deviceType: type,
                                      networkID: nid, rocketID: rid, pusher: pusher)
    }

    // MARK: - Readback intake

    func testReadbackCreatesUnprovisionedRecord() {
        let store = makeStore()
        report(store)

        let rec = try! XCTUnwrap(store.device(for: "a1b2c3d4"))
        XCTAssertEqual(rec.name, "Atlas")
        XCTAssertEqual(rec.deviceType, .rocket)
        XCTAssertEqual(rec.networkID, 42)
        XCTAssertEqual(rec.rocketID, 3)
        XCTAssertNotNil(rec.lastSeen)
        // A readback alone must NOT mark the device provisioned — that would
        // stop the first-connect sheet from ever appearing.
        XCTAssertFalse(rec.provisioned)
        XCTAssertFalse(store.isProvisioned("a1b2c3d4"))
    }

    func testReadbackRefreshesExistingRecord() {
        let store = makeStore()
        report(store, name: "Atlas", nid: 42)
        report(store, name: "Renamed On Other Phone", nid: 77)

        let rec = store.device(for: "a1b2c3d4")
        XCTAssertEqual(store.devices.count, 1)
        XCTAssertEqual(rec?.name, "Renamed On Other Phone")
        XCTAssertEqual(rec?.networkID, 77)
    }

    func testEmptyUnitIDIsIgnored() {
        let store = makeStore()
        report(store, unitID: "")
        XCTAssertTrue(store.devices.isEmpty)
        // Empty id reads as provisioned so the sheet can't pop pre-readback.
        XCTAssertTrue(store.isProvisioned(""))
    }

    func testUnknownTypeReadbackKeepsLearnedType() {
        // A device never changes species — a readback without "dt" (pre-dt
        // firmware) must not downgrade a learned type to unknown.
        let store = makeStore()
        report(store, type: .rocket)
        report(store, type: .unknown)
        XCTAssertEqual(store.device(for: "a1b2c3d4")?.deviceType, .rocket)
    }

    // MARK: - Provisioning gate

    func testMarkProvisioned() {
        let store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        XCTAssertTrue(store.isProvisioned("a1b2c3d4"))
        // Survives a reload.
        XCTAssertTrue(makeStore().isProvisioned("a1b2c3d4"))
    }

    func testForgetRemovesAndReprompts() {
        let store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        store.forget("a1b2c3d4")
        XCTAssertNil(store.device(for: "a1b2c3d4"))
        // Next connect treats it as brand new.
        XCTAssertFalse(store.isProvisioned("a1b2c3d4"))
        XCTAssertFalse(makeStore().isProvisioned("a1b2c3d4"))
    }

    // MARK: - Connected edits (push immediately)

    func testConnectedRenamePushesAndUpdates() {
        let store = makeStore()
        report(store)
        let spy = PusherSpy()

        store.setName("  Ares 2  ", for: "a1b2c3d4", pusher: spy)

        XCTAssertEqual(spy.names, ["Ares 2"])   // trimmed
        let rec = store.device(for: "a1b2c3d4")
        XCTAssertEqual(rec?.name, "Ares 2")
        XCTAssertNil(rec?.pendingName)
    }

    func testConnectedNetworkAndRocketIDPush() {
        let store = makeStore()
        report(store)
        let spy = PusherSpy()

        store.setNetworkID(99, for: "a1b2c3d4", pusher: spy)
        store.setRocketID(7, for: "a1b2c3d4", pusher: spy)

        XCTAssertEqual(spy.nids, [99])
        XCTAssertEqual(spy.rids, [7])
        let rec = store.device(for: "a1b2c3d4")
        XCTAssertEqual(rec?.networkID, 99)
        XCTAssertEqual(rec?.rocketID, 7)
        XCTAssertFalse(rec?.hasPendingChanges ?? true)
    }

    func testInvalidValuesRejected() {
        let store = makeStore()
        report(store)
        let spy = PusherSpy()

        store.setName("   ", for: "a1b2c3d4", pusher: spy)
        store.setNetworkID(0, for: "a1b2c3d4", pusher: spy)   // 0 = unset sentinel
        store.setRocketID(0, for: "a1b2c3d4", pusher: spy)    // firmware rejects
        store.setRocketID(255, for: "a1b2c3d4", pusher: spy)  // firmware rejects

        XCTAssertTrue(spy.names.isEmpty)
        XCTAssertTrue(spy.nids.isEmpty)
        XCTAssertTrue(spy.rids.isEmpty)
        XCTAssertEqual(store.device(for: "a1b2c3d4")?.name, "Atlas")
    }

    func testNameClampedTo20Bytes() {
        let store = makeStore()
        report(store)
        let spy = PusherSpy()
        store.setName(String(repeating: "x", count: 30), for: "a1b2c3d4", pusher: spy)
        XCTAssertEqual(spy.names, [String(repeating: "x", count: 20)])

        // The firmware limit is 20 UTF-8 BYTES, not characters — it rejects
        // longer payloads silently (no write, no echo), so the clamp must
        // count bytes and never split a character.  🚀 is 4 bytes.
        store.setName(String(repeating: "🚀", count: 10), for: "a1b2c3d4", pusher: spy)
        XCTAssertEqual(spy.names.last, String(repeating: "🚀", count: 5))
        XCTAssertLessThanOrEqual(spy.names.last!.utf8.count, 20)
    }

    // MARK: - Offline edits (queue, then apply on next readback)

    func testOfflineEditsQueueAsPending() {
        let store = makeStore()
        report(store)

        store.setName("Ares 2", for: "a1b2c3d4", pusher: nil)
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)
        store.setRocketID(7, for: "a1b2c3d4", pusher: nil)

        let rec = try! XCTUnwrap(store.device(for: "a1b2c3d4"))
        // Current values untouched; targets queued.
        XCTAssertEqual(rec.name, "Atlas")
        XCTAssertEqual(rec.pendingName, "Ares 2")
        XCTAssertEqual(rec.pendingNetworkID, 99)
        XCTAssertEqual(rec.pendingRocketID, 7)
        XCTAssertTrue(rec.hasPendingChanges)
        // Display prefers the target state.
        XCTAssertEqual(rec.effectiveName, "Ares 2")
        XCTAssertEqual(rec.effectiveNetworkID, 99)
        XCTAssertEqual(rec.effectiveRocketID, 7)
    }

    func testClearPendingDropsQueueOnly() {
        let store = makeStore()
        report(store)
        store.setName("Ares 2", for: "a1b2c3d4", pusher: nil)
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)

        store.clearPending(for: "a1b2c3d4")

        let rec = try! XCTUnwrap(store.device(for: "a1b2c3d4"))
        XCTAssertFalse(rec.hasPendingChanges)
        XCTAssertEqual(rec.name, "Atlas")       // current values untouched
        XCTAssertEqual(rec.networkID, 42)
    }

    func testOfflineEditMatchingCurrentClearsPending() {
        let store = makeStore()
        report(store, name: "Atlas", nid: 42)
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)
        // User changes their mind back to the device's actual value.
        store.setNetworkID(42, for: "a1b2c3d4", pusher: nil)
        XCTAssertFalse(store.device(for: "a1b2c3d4")?.hasPendingChanges ?? true)
    }

    func testPendingAppliedOnNextReadback() {
        let store = makeStore()
        report(store)
        store.setName("Ares 2", for: "a1b2c3d4", pusher: nil)
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)

        // Device reconnects and reports its (old) identity.
        let spy = PusherSpy()
        report(store, name: "Atlas", nid: 42, pusher: spy)

        XCTAssertEqual(spy.names, ["Ares 2"])
        XCTAssertEqual(spy.nids, [99])
        XCTAssertTrue(spy.rids.isEmpty)   // nothing queued for rid
        let rec = try! XCTUnwrap(store.device(for: "a1b2c3d4"))
        XCTAssertFalse(rec.hasPendingChanges)
        XCTAssertEqual(rec.name, "Ares 2")       // optimistic until the echo
        XCTAssertEqual(rec.networkID, 99)
    }

    func testPendingMatchingReadbackClearsWithoutPush() {
        // The device already has the queued value (e.g. set from another
        // phone) — don't push a redundant write, just clear the queue.
        let store = makeStore()
        report(store, nid: 42)
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)

        let spy = PusherSpy()
        report(store, nid: 99, pusher: spy)

        XCTAssertTrue(spy.nids.isEmpty)
        let rec = try! XCTUnwrap(store.device(for: "a1b2c3d4"))
        XCTAssertFalse(rec.hasPendingChanges)
        XCTAssertEqual(rec.networkID, 99)
    }

    func testEchoReadbackAfterPushIsPlainUpsert() {
        // After a pending push, the firmware echoes config_identity with the
        // new values.  Pending was cleared at push time, so the echo must not
        // re-push (no loop) and must land the confirmed values.
        let store = makeStore()
        report(store)
        store.setName("Ares 2", for: "a1b2c3d4", pusher: nil)

        let spy = PusherSpy()
        report(store, name: "Atlas", pusher: spy)          // reconnect → push
        report(store, name: "Ares 2", pusher: spy)         // firmware echo

        XCTAssertEqual(spy.names, ["Ares 2"])              // exactly one push
        XCTAssertEqual(store.device(for: "a1b2c3d4")?.name, "Ares 2")
    }

    // MARK: - Persistence + migration

    func testPersistenceRoundTrip() {
        let store = makeStore()
        report(store)
        store.markProvisioned("a1b2c3d4")
        store.setNetworkID(99, for: "a1b2c3d4", pusher: nil)

        let reloaded = makeStore()
        let rec = try! XCTUnwrap(reloaded.device(for: "a1b2c3d4"))
        XCTAssertEqual(rec.name, "Atlas")
        XCTAssertEqual(rec.deviceType, .rocket)
        XCTAssertTrue(rec.provisioned)
        XCTAssertEqual(rec.pendingNetworkID, 99)   // queue survives relaunch
    }

    func testLegacyKnownDeviceIDsMigration() {
        defaults.set(["aaaa1111", "bbbb2222"], forKey: "knownDeviceIDs")

        let store = makeStore()

        // Legacy-known devices must not re-trigger the provisioning sheet.
        XCTAssertTrue(store.isProvisioned("aaaa1111"))
        XCTAssertTrue(store.isProvisioned("bbbb2222"))
        // Names/types are unknown until the next readback.
        XCTAssertEqual(store.device(for: "aaaa1111")?.name, "")
        // Source key is consumed so forget() can't be undone by re-migration.
        XCTAssertNil(defaults.stringArray(forKey: "knownDeviceIDs"))

        store.forget("aaaa1111")
        XCTAssertFalse(makeStore().isProvisioned("aaaa1111"))
    }

    func testMigrationMergesWithExistingRecords() {
        // Migration must not clobber a record that already exists (e.g. the
        // readback landed before the legacy array was folded in).
        let store = makeStore()
        report(store)
        defaults.set(["a1b2c3d4"], forKey: "knownDeviceIDs")

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.devices.count, 1)
        XCTAssertEqual(reloaded.device(for: "a1b2c3d4")?.name, "Atlas")
        // The legacy array only ever held provisioned devices — merging must
        // carry that over to the surviving record.
        XCTAssertTrue(reloaded.isProvisioned("a1b2c3d4"))
    }

    // MARK: - Scan-time type recovery (renamed devices lose the TR- prefix)

    func testDeviceTypeForAdvertisedName() {
        // The firmware advertises the raw unit name, so a device renamed to
        // "Atlas" no longer carries the TR-R- prefix the name heuristic
        // needs — the registry is the only thing that still knows its type.
        let store = makeStore()
        report(store, unitID: "r1", name: "Atlas", type: .rocket)
        report(store, unitID: "bs1", name: "Pad Station", type: .baseStation)

        XCTAssertEqual(store.deviceType(forAdvertisedName: "Atlas"), .rocket)
        XCTAssertEqual(store.deviceType(forAdvertisedName: "Pad Station"), .baseStation)
        XCTAssertNil(store.deviceType(forAdvertisedName: "TR-R-1a2b"))  // not in registry
        XCTAssertNil(store.deviceType(forAdvertisedName: ""))
    }

    func testDeviceTypeForAdvertisedName_AmbiguousOrUnknownGivesNil() {
        let store = makeStore()
        // Two devices sharing a name: no unambiguous answer.
        report(store, unitID: "r1", name: "Atlas", type: .rocket)
        report(store, unitID: "bs1", name: "Atlas", type: .baseStation)
        XCTAssertNil(store.deviceType(forAdvertisedName: "Atlas"))

        // A record whose own type is unknown can't type a scan result.
        report(store, unitID: "x1", name: "Mystery", type: .unknown)
        XCTAssertNil(store.deviceType(forAdvertisedName: "Mystery"))
    }

    // MARK: - Ordering

    func testRocketsSortBeforeBaseStations() {
        let store = makeStore()
        report(store, unitID: "bs1", name: "Pad Station", type: .baseStation)
        report(store, unitID: "r2", name: "Zephyr", type: .rocket)
        report(store, unitID: "r1", name: "Atlas", type: .rocket)

        XCTAssertEqual(store.devices.map { $0.name },
                       ["Atlas", "Zephyr", "Pad Station"])
    }
}
