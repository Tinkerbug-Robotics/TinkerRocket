//
//  RocketProfileStoreTests.swift
//  TinkerRocketAppTests
//
//  CRUD, persistence, legacy migration, and Codable round-trip for the
//  per-rocket profile store (issue #132).
//

import XCTest
@testable import TinkerRocketApp

final class RocketProfileStoreTests: XCTestCase {

    private var tempDir: URL!
    private var suiteName: String!
    private var defaults: UserDefaults!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("ProfileTests-\(UUID().uuidString)", isDirectory: true)
        suiteName = "ProfileTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
        defaults.removePersistentDomain(forName: suiteName)
    }

    private func makeStore() -> RocketProfileStore {
        RocketProfileStore(directory: tempDir, defaults: defaults)
    }

    // MARK: - CRUD

    func testAddCreatesProfileAndPersists() {
        let store = makeStore()
        XCTAssertTrue(store.profiles.isEmpty)

        let p = store.add(name: "Big Bertha")
        XCTAssertEqual(store.profiles.count, 1)
        XCTAssertEqual(p.name, "Big Bertha")
        // add() must not auto-activate.
        XCTAssertNil(store.activeId)

        // A fresh store over the same dir reloads it.
        let reloaded = makeStore()
        XCTAssertEqual(reloaded.profiles.count, 1)
        XCTAssertEqual(reloaded.profiles.first?.name, "Big Bertha")
    }

    func testAddDedupesNames() {
        let store = makeStore()
        store.add(name: "Arcas")
        store.add(name: "Arcas")
        store.add(name: "Arcas")
        let names = store.profiles.map { $0.name }.sorted()
        XCTAssertEqual(names, ["Arcas", "Arcas 2", "Arcas 3"])
    }

    func testDuplicateDropsMagCalAndRenames() {
        let store = makeStore()
        let src = store.add(name: "Source")
        store.update(src.id) {
            $0.pidKp = 0.42
            $0.magCal = MagCalData(offsetX: 1, offsetY: 2, offsetZ: 3,
                                   fieldR_uT: 48, residualUT: 2,
                                   calibratedOnUnitID: "ABC123",
                                   calibratedAt: Date())
            $0.sensorCal = SensorCalData(gyroX: 4, gyroY: 5, gyroZ: 6,
                                         hgX: 0.1, hgY: 0.2, hgZ: 0.3,
                                         calibratedOnUnitID: "ABC123",
                                         calibratedAt: Date())
        }

        let copy = store.duplicate(src.id)
        XCTAssertNotNil(copy)
        XCTAssertEqual(copy?.name, "Source copy")
        XCTAssertEqual(copy?.pidKp, 0.42)          // tuning copied
        XCTAssertNil(copy?.magCal)                  // cal NOT copied (board-specific)
        XCTAssertNil(copy?.sensorCal)               // sensor cal NOT copied either
        XCTAssertNil(copy?.lastUsedUnitID)
        XCTAssertNotEqual(copy?.id, src.id)
        XCTAssertEqual(store.profiles.count, 2)
    }

    func testRenamePersists() {
        let store = makeStore()
        let p = store.add(name: "Old")
        store.rename(p.id, to: "New")
        XCTAssertEqual(store.profiles.first?.name, "New")
        XCTAssertEqual(makeStore().profiles.first?.name, "New")
    }

    func testDeleteRemovesFileAndClearsActive() {
        let store = makeStore()
        let a = store.add(name: "A")
        store.setActive(a.id)
        XCTAssertEqual(store.activeId, a.id)

        store.delete(a.id)
        XCTAssertTrue(store.profiles.isEmpty)
        XCTAssertNil(store.activeId)
        // The on-disk file is gone too.
        XCTAssertTrue(makeStore().profiles.isEmpty)
    }

    func testDeleteActiveFallsBackToAnotherProfile() {
        let store = makeStore()
        let a = store.add(name: "A")
        let b = store.add(name: "B")
        store.setActive(a.id)
        store.delete(a.id)
        // Active falls back to a remaining profile rather than going nil.
        XCTAssertEqual(store.activeId, b.id)
    }

    func testSetActivePersistsAcrossReload() {
        let store = makeStore()
        store.add(name: "A")
        let b = store.add(name: "B")
        store.setActive(b.id)

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.activeId, b.id)
        XCTAssertEqual(reloaded.activeProfile?.name, "B")
    }

    func testUpdateBumpsTimestamp() {
        let store = makeStore()
        let p = store.add(name: "A")
        let before = p.updatedAt
        // Ensure a measurable delta.
        Thread.sleep(forTimeInterval: 0.01)
        store.update(p.id) { $0.cameraType = 1 }
        let after = store.profiles.first!.updatedAt
        XCTAssertGreaterThan(after, before)
        XCTAssertEqual(store.profiles.first?.cameraType, 1)
    }

    // MARK: - Resilience

    func testCorruptFileSkippedOnLoad() throws {
        let store = makeStore()
        store.add(name: "Good")
        // Drop a garbage .json next to the real one.
        let junk = tempDir.appendingPathComponent("\(UUID().uuidString).json")
        try Data("not json".utf8).write(to: junk)

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.profiles.count, 1)
        XCTAssertEqual(reloaded.profiles.first?.name, "Good")
    }

    func testDanglingActiveIdReconciled() {
        let store = makeStore()
        let a = store.add(name: "A")
        store.setActive(a.id)
        store.delete(a.id)
        // Manually point active at a now-nonexistent id, then reload.
        defaults.set(UUID().uuidString, forKey: "activeRocketProfileId")
        let reloaded = makeStore()
        XCTAssertNil(reloaded.activeId)
    }

    // MARK: - Codable round-trip

    func testProfileCodableRoundTrip() throws {
        var p = RocketProfile.makeDefault(name: "RT")
        p.notes = "4in glass, J motors"
        p.pidKp = 0.123
        p.servoBias2 = -17
        p.cameraType = 1
        p.rollWaypoints = [
            RollWaypoint(timeSeconds: 0, angleDeg: 0, mode: .nullRate),
            RollWaypoint(timeSeconds: 2.5, angleDeg: 90, mode: .angle),
        ]
        p.pyro2Enabled = true
        p.pyro2TriggerValue = 213
        p.magCal = MagCalData(offsetX: -5, offsetY: 6, offsetZ: 7,
                              fieldR_uT: 49.5, residualUT: 1.5,
                              calibratedOnUnitID: "DEAD::BEEF",
                              calibratedAt: Date(timeIntervalSince1970: 1_700_000_000))
        p.sensorCal = SensorCalData(gyroX: -8, gyroY: 9, gyroZ: -10,
                                    hgX: 0.05, hgY: -0.06, hgZ: 9.81,
                                    calibratedOnUnitID: "DEAD::BEEF",
                                    calibratedAt: Date(timeIntervalSince1970: 1_700_000_500))

        let data = try JSONEncoder().encode(p)
        let back = try JSONDecoder().decode(RocketProfile.self, from: data)
        XCTAssertEqual(p, back)
    }

    // MARK: - Migration

    private func seedLegacyDefaults() {
        defaults.set(true, forKey: "rocketSoundsEnabled")
        defaults.set(false, forKey: "servoControlEnabled")
        defaults.set(1, forKey: "cameraType")
        defaults.set(true, forKey: "useAngleControl")
        defaults.set(150.0, forKey: "rollDelayMs")
        defaults.set(90.0, forKey: "servoBias1")
        defaults.set(0.111, forKey: "pidKp")
        defaults.set(0.222, forKey: "pidKi")
        let roll = "[[\"0.0\",\"0\",\"1\"],[\"2.0\",\"45\",\"0\"]]"
        defaults.set(roll, forKey: "rollProfileJSON")
    }

    func testMigrationCreatesDefaultFromLegacyKeys() {
        seedLegacyDefaults()
        let store = makeStore()

        XCTAssertEqual(store.profiles.count, 1)
        let p = try! XCTUnwrap(store.activeProfile)
        XCTAssertEqual(p.name, "Default")
        XCTAssertTrue(p.soundsEnabled)
        XCTAssertFalse(p.servoControlEnabled)
        XCTAssertEqual(p.cameraType, 1)
        XCTAssertTrue(p.useAngleControl)
        XCTAssertEqual(p.rollDelayMs, 150)
        XCTAssertEqual(p.servoBias1, 90)
        XCTAssertEqual(p.pidKp, 0.111, accuracy: 1e-5)
        XCTAssertEqual(p.pidKi, 0.222, accuracy: 1e-5)
        XCTAssertEqual(p.rollWaypoints.count, 2)
        XCTAssertEqual(p.rollWaypoints[0].mode, .nullRate)
        XCTAssertEqual(p.rollWaypoints[1].angleDeg, 45)
    }

    func testMigrationRunsOnlyOnce() {
        seedLegacyDefaults()
        _ = makeStore()                 // first init migrates
        // Tamper: add another legacy key + re-init. Should NOT re-migrate.
        defaults.set(99.0, forKey: "servoBias2")
        let second = makeStore()
        XCTAssertEqual(second.profiles.count, 1)
    }

    func testNoMigrationOnFreshInstall() {
        // No legacy keys seeded.
        let store = makeStore()
        XCTAssertTrue(store.profiles.isEmpty)
        XCTAssertNil(store.activeId)
        XCTAssertTrue(defaults.bool(forKey: "rocketProfilesMigratedV1"))
    }
}
