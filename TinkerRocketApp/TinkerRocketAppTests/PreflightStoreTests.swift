//
//  PreflightStoreTests.swift
//  TinkerRocketAppTests
//
//  Persistence + CRUD for the pre-flight checklist store: master template
//  edits, per-rocket diffs, run state, and the skip-corrupt reload rule.
//

import XCTest
@testable import TinkerRocketApp

final class PreflightStoreTests: XCTestCase {

    private var tempDir: URL!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("PreflightTests-\(UUID().uuidString)", isDirectory: true)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    private func makeStore() -> PreflightStore {
        PreflightStore(directory: tempDir)
    }

    // MARK: - Master

    func testMasterAddEditDeletePersists() {
        let store = makeStore()
        let item = store.addMasterItem(PreflightItem(title: "Wadding"))
        store.updateMasterItem(item.id) { $0.title = "Recovery wadding" }

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.master.items.map(\.title), ["Recovery wadding"])

        reloaded.deleteMasterItem(item.id)
        XCTAssertTrue(makeStore().master.items.isEmpty)
    }

    func testMasterMovePersistsOrder() {
        let store = makeStore()
        _ = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        _ = store.addMasterItem(PreflightItem(title: "C"))
        store.moveMasterItems(fromOffsets: IndexSet(integer: 0), toOffset: 3)
        XCTAssertEqual(makeStore().master.items.map(\.title), ["B", "C", "A"])
    }

    func testDeleteMasterItemScrubsRocketDiffs() {
        let store = makeStore()
        let item = store.addMasterItem(PreflightItem(title: "A"))
        let rocket = UUID()
        store.setMasterItem(item.id, enabled: false, for: rocket)
        store.setChecked(item.id, checked: true, for: rocket)

        store.deleteMasterItem(item.id)
        let config = makeStore().config(for: rocket)
        XCTAssertEqual(config?.disabledMasterIds, [])
        XCTAssertEqual(config?.checked, [:])
    }

    // MARK: - Per-rocket config

    func testRocketConfigDiffAndEffectiveList() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        let rocket = UUID()

        store.setMasterItem(a.id, enabled: false, for: rocket)
        store.addExtraItem(PreflightItem(title: "Rail buttons"), for: rocket)

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.effectiveItems(for: rocket).map(\.title),
                       ["B", "Rail buttons"])
        // Re-enabling clears the exclusion.
        reloaded.setMasterItem(a.id, enabled: true, for: rocket)
        XCTAssertEqual(reloaded.effectiveItems(for: rocket).map(\.title),
                       ["A", "B", "Rail buttons"])
        // An untouched rocket just sees the master.
        XCTAssertEqual(reloaded.effectiveItems(for: UUID()).map(\.title), ["A", "B"])
    }

    func testDisablingMasterItemDropsItsCheck() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        let rocket = UUID()
        store.setChecked(a.id, checked: true, for: rocket)
        XCTAssertTrue(store.isChecked(a.id, for: rocket))

        store.setMasterItem(a.id, enabled: false, for: rocket)
        // Re-including later must come back UNCHECKED — the old check is
        // stale evidence.
        store.setMasterItem(a.id, enabled: true, for: rocket)
        XCTAssertFalse(store.isChecked(a.id, for: rocket))
    }

    func testExtraItemDeleteDropsItsCheck() {
        let store = makeStore()
        let rocket = UUID()
        let extra = store.addExtraItem(PreflightItem(title: "X"), for: rocket)
        store.setChecked(extra.id, checked: true, for: rocket)
        store.deleteExtraItem(extra.id, for: rocket)
        XCTAssertEqual(makeStore().config(for: rocket)?.checked, [:])
    }

    func testDeleteConfigRemovesFile() {
        let store = makeStore()
        let rocket = UUID()
        store.addExtraItem(PreflightItem(title: "X"), for: rocket)
        XCTAssertNotNil(makeStore().config(for: rocket))

        store.deleteConfig(for: rocket)
        XCTAssertNil(store.config(for: rocket))
        XCTAssertNil(makeStore().config(for: rocket))
    }

    // MARK: - Per-rocket ordering

    func testMoveEffectiveItemsInterleavesExtrasAndPersists() {
        let store = makeStore()
        _ = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        let rocket = UUID()
        store.addExtraItem(PreflightItem(title: "Extra"), for: rocket)

        // [A, B, Extra] → move Extra between the master steps.
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 2), toOffset: 1,
                                 for: rocket)
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title),
                       ["A", "Extra", "B"])
        XCTAssertEqual(makeStore().effectiveItems(for: rocket).map(\.title),
                       ["A", "Extra", "B"])
        // Other rockets keep the default order.
        XCTAssertEqual(store.effectiveItems(for: UUID()).map(\.title), ["A", "B"])
    }

    func testNewMasterStepAppendsAfterACustomOrder() {
        let store = makeStore()
        _ = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        let rocket = UUID()
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 0), toOffset: 2,
                                 for: rocket)   // [B, A]

        _ = store.addMasterItem(PreflightItem(title: "C"))
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title),
                       ["B", "A", "C"])
    }

    func testExcludedStepKeepsItsSlotWhenReIncluded() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        _ = store.addMasterItem(PreflightItem(title: "C"))
        let rocket = UUID()
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 0), toOffset: 3,
                                 for: rocket)   // [B, C, A]

        store.setMasterItem(a.id, enabled: false, for: rocket)
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title), ["B", "C"])
        // The order still remembers A's slot — re-including restores it.
        store.setMasterItem(a.id, enabled: true, for: rocket)
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title),
                       ["B", "C", "A"])
    }

    /// The review-found hazard: a move must NOT erase an excluded step's slot.
    func testReorderWhileExcludedKeepsTheRememberedSlot() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        _ = store.addMasterItem(PreflightItem(title: "C"))
        let rocket = UUID()
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 2), toOffset: 1,
                                 for: rocket)   // [A, C, B]

        store.setMasterItem(a.id, enabled: false, for: rocket)
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title), ["C", "B"])
        // Reorder WHILE A is excluded — A's first-place slot must survive.
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 0), toOffset: 2,
                                 for: rocket)   // visible [B, C]
        store.setMasterItem(a.id, enabled: true, for: rocket)
        XCTAssertEqual(makeStore().effectiveItems(for: rocket).map(\.title),
                       ["A", "B", "C"])
    }

    /// First-ever move must remember master positions of already-excluded steps.
    func testFirstMoveRemembersExcludedMasterPositions() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        _ = store.addMasterItem(PreflightItem(title: "C"))
        let rocket = UUID()

        store.setMasterItem(a.id, enabled: false, for: rocket)
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 1), toOffset: 0,
                                 for: rocket)   // visible [C, B]
        store.setMasterItem(a.id, enabled: true, for: rocket)
        // A comes back at its master position (first), not appended last.
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title),
                       ["A", "C", "B"])
    }

    func testDeleteScrubsOrderedIds() {
        let store = makeStore()
        let a = store.addMasterItem(PreflightItem(title: "A"))
        _ = store.addMasterItem(PreflightItem(title: "B"))
        let rocket = UUID()
        let extra = store.addExtraItem(PreflightItem(title: "Extra"), for: rocket)
        store.moveEffectiveItems(fromOffsets: IndexSet(integer: 2), toOffset: 0,
                                 for: rocket)   // [Extra, A, B]

        store.deleteMasterItem(a.id)
        store.deleteExtraItem(extra.id, for: rocket)
        XCTAssertEqual(store.config(for: rocket)?.orderedIds.count, 1)
        XCTAssertEqual(store.effectiveItems(for: rocket).map(\.title), ["B"])
    }

    // MARK: - Run state

    func testCheckedRoundTripAndReset() {
        let store = makeStore()
        let item = store.addMasterItem(PreflightItem(title: "A"))
        let rocket = UUID()

        store.setChecked(item.id, checked: true, for: rocket)
        XCTAssertTrue(makeStore().isChecked(item.id, for: rocket))

        store.setChecked(item.id, checked: false, for: rocket)
        XCTAssertFalse(makeStore().isChecked(item.id, for: rocket))

        store.setChecked(item.id, checked: true, for: rocket)
        store.resetRun(for: rocket)
        XCTAssertFalse(makeStore().isChecked(item.id, for: rocket))
    }

    // MARK: - Robustness

    func testCorruptConfigFileLosesOneRocketNotTheSet() throws {
        let store = makeStore()
        let rocketA = UUID(), rocketB = UUID()
        store.addExtraItem(PreflightItem(title: "A"), for: rocketA)
        store.addExtraItem(PreflightItem(title: "B"), for: rocketB)

        // Corrupt rocket A's file on disk.
        let fileA = tempDir.appendingPathComponent("\(rocketA.uuidString).json")
        try "not json {".data(using: .utf8)!.write(to: fileA)

        let reloaded = makeStore()
        XCTAssertNil(reloaded.config(for: rocketA))
        XCTAssertEqual(reloaded.config(for: rocketB)?.extraItems.map(\.title), ["B"])
    }

    func testCorruptMasterFileYieldsEmptyMasterButKeepsConfigs() throws {
        let store = makeStore()
        let rocket = UUID()
        store.addMasterItem(PreflightItem(title: "A"))
        store.addExtraItem(PreflightItem(title: "X"), for: rocket)

        try "garbage".data(using: .utf8)!
            .write(to: tempDir.appendingPathComponent("master.json"))

        let reloaded = makeStore()
        XCTAssertTrue(reloaded.master.items.isEmpty)
        XCTAssertEqual(reloaded.config(for: rocket)?.extraItems.map(\.title), ["X"])
    }

    // MARK: - Lenient dates (Android PreflightCodec.dateMs parity)

    /// A hand-checked step is the fact the pad cares about.  One unreadable
    /// timestamp used to throw out of [String: Date], and load()'s `continue`
    /// then reverted the whole airframe to the untouched master list.
    func testMalformedCheckedTimestampKeepsTheTick() throws {
        let store = makeStore()
        let rocket = UUID()
        let kept = store.addMasterItem(PreflightItem(title: "A"))
        let bad = store.addMasterItem(PreflightItem(title: "B"))
        store.addExtraItem(PreflightItem(title: "X"), for: rocket)
        store.setChecked(kept.id, checked: true, for: rocket)
        store.setChecked(bad.id, checked: true, for: rocket)

        try rewriteConfig(rocket) { json in
            json.replacingOccurrences(
                of: "\"\(bad.id.uuidString)\" : ",
                with: "\"\(bad.id.uuidString)\" : \"not-a-date\", \"_x\" : ")
        }

        let reloaded = makeStore()
        // Both ticks survive, and so does the rest of the rocket's diff.
        XCTAssertTrue(reloaded.isChecked(kept.id, for: rocket))
        XCTAssertTrue(reloaded.isChecked(bad.id, for: rocket))
        XCTAssertEqual(reloaded.config(for: rocket)?.extraItems.map(\.title), ["X"])
    }

    /// Wrong type for the whole key: empty checked state, but the rocket's
    /// exclusions and extras still load.
    func testWrongTypedCheckedKeyYieldsNoChecksButKeepsTheDiff() throws {
        let store = makeStore()
        let rocket = UUID()
        let item = store.addMasterItem(PreflightItem(title: "A"))
        store.addExtraItem(PreflightItem(title: "X"), for: rocket)
        store.setChecked(item.id, checked: true, for: rocket)

        try rewriteConfig(rocket) { json in
            let open = json.range(of: "\"checked\" : {")!
            let close = json.range(of: "}", range: open.upperBound..<json.endIndex)!
            return json.replacingCharacters(
                in: open.lowerBound..<close.upperBound, with: "\"checked\" : []")
        }

        let reloaded = makeStore()
        XCTAssertFalse(reloaded.isChecked(item.id, for: rocket))
        XCTAssertEqual(reloaded.config(for: rocket)?.extraItems.map(\.title), ["X"])
    }

    /// updatedAt is the other strict Date decode: unreadable on a CONFIG cost
    /// that rocket its whole diff, and on the MASTER cost every rocket its
    /// template.
    func testMalformedUpdatedAtKeepsBothFiles() throws {
        let store = makeStore()
        let rocket = UUID()
        let item = store.addMasterItem(PreflightItem(title: "A"))
        store.addExtraItem(PreflightItem(title: "X"), for: rocket)
        store.setChecked(item.id, checked: true, for: rocket)

        try rewriteConfig(rocket) { $0.replacingOccurrences(
            of: "\"updatedAt\" : ", with: "\"updatedAt\" : \"nope\", \"_x\" : ") }
        let masterURL = tempDir.appendingPathComponent("master.json")
        try String(contentsOf: masterURL, encoding: .utf8)
            .replacingOccurrences(of: "\"updatedAt\" : ",
                                  with: "\"updatedAt\" : \"nope\", \"_x\" : ")
            .data(using: .utf8)!.write(to: masterURL)

        let reloaded = makeStore()
        XCTAssertEqual(reloaded.master.items.map(\.title), ["A"])
        XCTAssertTrue(reloaded.isChecked(item.id, for: rocket))
        XCTAssertEqual(reloaded.config(for: rocket)?.extraItems.map(\.title), ["X"])
    }

    /// Rewrite a rocket's config file on disk.  Asserts the edit actually
    /// changed something — a no-op rewrite would make these tests pass for
    /// the wrong reason.
    private func rewriteConfig(_ profileId: UUID,
                               _ edit: (String) -> String) throws {
        let url = tempDir.appendingPathComponent("\(profileId.uuidString).json")
        let before = try String(contentsOf: url, encoding: .utf8)
        let after = edit(before)
        XCTAssertNotEqual(before, after, "config rewrite was a no-op")
        try after.data(using: .utf8)!.write(to: url)
    }
}
