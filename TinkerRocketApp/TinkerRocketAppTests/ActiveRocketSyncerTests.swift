//
//  ActiveRocketSyncerTests.swift
//  TinkerRocketAppTests
//
//  Pure decision logic for the connect-time profile sync (issue #132):
//  mag-cal push/warn/read and the soft profile suggestion.
//

import XCTest
@testable import TinkerRocketApp

final class ActiveRocketSyncerTests: XCTestCase {

    private func cal(on unitID: String) -> MagCalData {
        MagCalData(offsetX: 1, offsetY: 2, offsetZ: 3,
                   fieldR_uT: 48, residualUT: 2,
                   calibratedOnUnitID: unitID, calibratedAt: Date())
    }

    // MARK: - Mag cal action

    func testNoProfileCalAsksRocket() {
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: nil,
                                                         deviceUnitID: "BOARD1")
        XCTAssertEqual(action, .readRocket)
    }

    func testMatchingBoardPushesCal() {
        let c = cal(on: "BOARD1")
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: c,
                                                         deviceUnitID: "BOARD1")
        XCTAssertEqual(action, .push(c))
    }

    func testDifferentBoardWarns() {
        let c = cal(on: "BOARD1")
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: c,
                                                         deviceUnitID: "BOARD2")
        XCTAssertEqual(action, .warnMismatch(savedOn: "BOARD1", current: "BOARD2"))
    }

    // MARK: - Suggestion

    func testSuggestionMatchesLastUsedBoard() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        var b = RocketProfile.makeDefault(name: "B"); b.lastUsedUnitID = "BOARD2"
        let s = ActiveRocketSyncer.suggestedProfile(in: [a, b], active: nil, unitID: "BOARD2")
        XCTAssertEqual(s, b.id)
    }

    func testNoSuggestionWhenActiveAlreadyMatches() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        // Active profile already the one flown on this board → nothing to suggest.
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: a.id, unitID: "BOARD1")
        XCTAssertNil(s)
    }

    func testNoSuggestionForUnknownBoard() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: nil, unitID: "NEVER_SEEN")
        XCTAssertNil(s)
    }

    func testNoSuggestionForEmptyUnitID() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = ""
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: nil, unitID: "")
        XCTAssertNil(s)
    }

    // MARK: - MagCalData snapshot from a status frame

    func testMagCalDataFromStatusTagsBoard() {
        let status = MagCalStatus(
            subType: .applied, coverageBins: 26, sampleCount: 1000,
            instantaneousFieldUT: 48, offsetX: -4, offsetY: 5, offsetZ: -6,
            fieldR_uT: 49.5, residualUT: 1.2, rejectCode: .ok, coverageMask: 0,
            liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0)
        let data = MagCalData(status: status, unitID: "BOARD9")
        XCTAssertEqual(data.offsetX, -4)
        XCTAssertEqual(data.offsetY, 5)
        XCTAssertEqual(data.offsetZ, -6)
        XCTAssertEqual(data.fieldR_uT, 49.5, accuracy: 1e-4)
        XCTAssertEqual(data.residualUT, 1.2, accuracy: 1e-4)
        XCTAssertEqual(data.calibratedOnUnitID, "BOARD9")
    }
}
