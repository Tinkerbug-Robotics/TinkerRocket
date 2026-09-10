import XCTest
@testable import TinkerRocketApp

/// #773 step 4. Android twin: `FirmwareManifestTest.kt`, case for case.
///
/// The golden fixture is the SAME FILE the Android suite reads — located from
/// `#filePath` rather than copied — so the two platforms cannot drift from each
/// other or from what `image_info.py` actually emits.
final class FirmwareManifestTests: XCTestCase {

    private func img(_ project: String, _ board: String?,
                     file: String? = nil, sha: String = String(repeating: "a", count: 64),
                     size: Int = 644336) -> String {
        let f = file ?? "\(project).bin"
        let boardField = board.map { ",\"board\":\"\($0)\"" } ?? ""
        return """
        {"file":"\(f)","project":"\(project)","version":"3d1f5ca2-dirty+1",
         "chip_id":9,"chip":"ESP32-S3","idf_version":"v6.0.1",
         "build_date":"Sep  9 2026","size":\(size),"sha256":"\(sha)"\(boardField)}
        """
    }

    private func manifest(_ images: [String], version: Int = 1,
                          tag: String = "fw-v1.0.0") -> String {
        "{\"manifest_version\":\(version),\"tag\":\"\(tag)\",\"images\":[\(images.joined(separator: ","))]}"
    }

    func testParsesTheShapeImageInfoEmits() {
        let m = FirmwareManifest.parse(manifest([img("flight_computer", "v9"),
                                                 img("out_computer", "v9")]))
        XCTAssertNotNil(m)
        XCTAssertEqual(m?.tag, "fw-v1.0.0")
        XCTAssertEqual(m?.images.count, 2)
        XCTAssertEqual(m?.images[0].board, "v9")
        XCTAssertEqual(m?.images[0].sizeBytes, 644336)
    }

    func testABuildWithNoBoardSuffixParsesWithANilBoard() {
        let m = FirmwareManifest.parse(manifest([img("rocket_computer_mini", nil)]))
        XCTAssertNil(m?.images[0].board)
    }

    func testANewerManifestVersionIsRefusedRatherThanHalfRead() {
        XCTAssertNil(FirmwareManifest.parse(manifest([img("flight_computer", "v9")], version: 2)))
        XCTAssertNil(FirmwareManifest.parse(manifest([img("flight_computer", "v9")], version: 0)))
    }

    func testAnImageTheAppCouldNotVerifyIsDroppedNotShown() {
        let noSha = #"{"file":"x.bin","project":"flight_computer","size":100}"#
        let shortSha = #"{"file":"x.bin","project":"flight_computer","size":100,"sha256":"abc"}"#
        XCTAssertNil(FirmwareManifest.parse(manifest([noSha])))
        XCTAssertNil(FirmwareManifest.parse(manifest([shortSha])))
        // One bad row among good ones only drops that row.
        let m = FirmwareManifest.parse(manifest([img("flight_computer", "v9"), noSha]))
        XCTAssertEqual(m?.images.count, 1)
    }

    func testJunkIsNilNotACrash() {
        XCTAssertNil(FirmwareManifest.parse(""))
        XCTAssertNil(FirmwareManifest.parse("not json"))
        XCTAssertNil(FirmwareManifest.parse("[]"))
        XCTAssertNil(FirmwareManifest.parse(#"{"manifest_version":1}"#))
    }

    // MARK: - selection

    private var full: FirmwareManifest {
        FirmwareManifest.parse(manifest([
            img("flight_computer", "v7"), img("flight_computer", "v8"),
            img("flight_computer", "v9"), img("flight_computer", "m1"),
            img("out_computer", "v9"), img("base_station", "v3"),
            img("rocket_computer_mini", nil),
        ]))!
    }

    func testProjectIsTheHardFilter() {
        // The case #1310 exists for: base_station and out_computer are both
        // ESP32-S3 with byte-identical app slots.
        let oc = FirmwareCatalog.forUnit(full, expectedProject: EspImage.projectOC)
        XCTAssertEqual(oc.count, 1)
        XCTAssertEqual(oc.first?.project, "out_computer")
    }

    func testTheProvisionedBoardSortsFirstAndEverythingIsStillOffered() {
        let list = FirmwareCatalog.forUnit(full, expectedProject: EspImage.projectFC,
                                           provisionedBoard: "V8")
        XCTAssertEqual(list.first?.board, "v8")
        XCTAssertEqual(list.count, 4)
    }

    func testBestReturnsTheMatchingBoard() {
        XCTAssertEqual(FirmwareCatalog.best(full, expectedProject: EspImage.projectFC,
                                            provisionedBoard: "v9")?.board, "v9")
        XCTAssertEqual(FirmwareCatalog.best(full, expectedProject: EspImage.projectFC,
                                            provisionedBoard: "M1")?.board, "m1")
    }

    func testBestRefusesToGuessWhenTheBoardIsKnownAndNothingMatches() {
        XCTAssertNil(FirmwareCatalog.best(full, expectedProject: EspImage.projectFC,
                                          provisionedBoard: "v12"))
        XCTAssertFalse(FirmwareCatalog.forUnit(full, expectedProject: EspImage.projectFC,
                                               provisionedBoard: "v12").isEmpty)
    }

    func testAnUnknownBoardGetsNoDefaultWhenEveryImageIsBoardSpecific() {
        // This test used to assert the opposite. The bench disproved it on
        // 2026-09-10: against fw-v0.1.0 on a real V9 out computer, the guess
        // was the ROCKET-COMPUTER-MINI image, because with no board to match
        // the sort falls back to the board string and `m1` beats `v8` and `v9`
        // on spelling alone. It was presented with a tick, as the
        // recommendation.
        //
        // A wrong recommendation is worse than none: it is the one an operator
        // in a hurry takes. The warning at flash time is a backstop, not a
        // reason to point at the wrong file first — and it is only a warning.
        //
        // Boards provisioned before #773 step 2 report no revision, so this is
        // the ordinary state of existing hardware rather than an edge case.
        XCTAssertNil(FirmwareCatalog.best(full, expectedProject: EspImage.projectFC))
        XCTAssertNil(FirmwareCatalog.best(full, expectedProject: EspImage.projectFC,
                                          provisionedBoard: "  "))
        // ...and the list is still there, which is the whole point: refusing
        // to guess is not refusing to show.
        XCTAssertEqual(FirmwareCatalog.forUnit(full, expectedProject: EspImage.projectFC).count, 4)
    }

    func testAnUnknownBoardStillGetsASuffixlessImage() {
        // A project shipping exactly one build applies everywhere by
        // construction, so there is nothing to get wrong and no reason to
        // withhold it.
        XCTAssertEqual(FirmwareCatalog.best(full, expectedProject: EspImage.projectMini)?.project,
                       "rocket_computer_mini")
    }

    func testAVersionStringIsEnoughToIdentifyTheBoard() {
        // What the app had all along and was not using: the unit's running
        // firmware says which board it is.
        XCTAssertEqual(EspImage.boardSuffix(of: "e1a4bee4-v9+20260910-1112"), "v9")
        XCTAssertEqual(EspImage.boardSuffix(of: "f204e64-m1+20260910-1230"), "m1")
        XCTAssertNil(EspImage.boardSuffix(of: "f204e64+20260910-1230"))
        XCTAssertEqual(
            FirmwareCatalog.best(full, expectedProject: EspImage.projectFC,
                                 provisionedBoard: EspImage.boardSuffix(of: "e1a4bee4-v9+20260910-1112"))?.board,
            "v9")
    }

    func testASuffixlessImageAppliesToAnyBoard() {
        XCTAssertEqual(FirmwareCatalog.best(full, expectedProject: EspImage.projectMini,
                                            provisionedBoard: "v9")?.project,
                       "rocket_computer_mini")
    }

    func testNothingForThisUnitIsNilNotAnArbitraryImage() {
        XCTAssertNil(FirmwareCatalog.best(full, expectedProject: "radio_board",
                                          provisionedBoard: "v3"))
        XCTAssertTrue(FirmwareCatalog.forUnit(full, expectedProject: "radio_board").isEmpty)
    }

    func testTheGoldenManifestFromImageInfoParses() throws {
        // Not a hand-written fixture: produced by `image_info.py --manifest`
        // against images actually built from this tree, and the SAME file the
        // Android suite reads. It is the contract between the release workflow
        // (#1322) and this parser.
        let here = URL(fileURLWithPath: #filePath)
        let repo = here.deletingLastPathComponent()   // TinkerRocketAppTests
            .deletingLastPathComponent()              // TinkerRocketApp
            .deletingLastPathComponent()              // repo root
        let golden = repo
            .appendingPathComponent("TinkerRocketAndroid/core/protocol/src/test/resources")
            .appendingPathComponent("firmware_manifest_golden.json")
        let data = try Data(contentsOf: golden)
        let m = try XCTUnwrap(FirmwareManifest.parse(data))

        XCTAssertEqual(m.tag, "fw-v1.0.0")
        XCTAssertEqual(m.images.count, 3)

        let fc = try XCTUnwrap(m.images.first { $0.project == "flight_computer" })
        XCTAssertEqual(fc.board, "v9")
        XCTAssertEqual(fc.chip, "ESP32-P4")
        XCTAssertEqual(fc.chipId, 18)
        XCTAssertGreaterThan(fc.sizeBytes, 100_000)
        XCTAssertEqual(fc.sha256.count, 64)

        // The mini's single-MCU build carries no board suffix — the real case
        // the nil board exists for.
        XCTAssertNil(m.images.first { $0.project == "rocket_computer_mini" }?.board)

        XCTAssertEqual(FirmwareCatalog.best(m, expectedProject: EspImage.projectFC,
                                            provisionedBoard: "v9")?.board, "v9")
        XCTAssertNil(FirmwareCatalog.best(m, expectedProject: EspImage.projectBS,
                                          provisionedBoard: "v3"))
    }
}
