import XCTest
@testable import TinkerRocketApp

/// The store / dev edition rules (AppEdition.swift). Android twin:
/// `AppEditionTest.kt`, case for case. The tests build in Debug, the dev
/// edition, so every store rule is checked by passing the edition in.
final class AppEditionTests: XCTestCase {

    // MARK: - Board family

    func testRevisionLetterNamesTheFamily() {
        XCTAssertEqual(BoardFamily(revision: "M2"), .beetle)
        XCTAssertEqual(BoardFamily(revision: "M2 (Tinker-Beetle V2)"), .beetle)
        XCTAssertEqual(BoardFamily(revision: "m3"), .beetle)
        XCTAssertEqual(BoardFamily(revision: "V10"), .mantis)
        XCTAssertEqual(BoardFamily(revision: " v9 "), .mantis)
    }

    func testTextThatNamesNoBoardIsUnknown() {
        XCTAssertNil(BoardFamily(revision: nil))
        XCTAssertNil(BoardFamily(revision: ""))
        XCTAssertNil(BoardFamily(revision: "unknown"))
        XCTAssertNil(BoardFamily(revision: "M"))      // a letter with no revision
        XCTAssertNil(BoardFamily(revision: "Mantis"))
        XCTAssertNil(BoardFamily(revision: "B1"))     // not a family this app gates on
    }

    func testProvisionedBoardBeatsTheFirmwareBuild() {
        // A Beetle flashed with a V10 build still says what it is.
        XCTAssertEqual(BoardFamily.of(fcBoard: "M2", ocBoard: nil,
                                      fcFirmware: "3d1f5ca2-v10+20261120", ocFirmware: nil),
                       .beetle)
        XCTAssertEqual(BoardFamily.of(fcBoard: nil, ocBoard: "V10",
                                      fcFirmware: "", ocFirmware: "3d1f5ca2-m2+20261120"),
                       .mantis)
    }

    func testUnprovisionedBoardFallsBackToItsFirmwareBuild() {
        XCTAssertEqual(BoardFamily.of(fcBoard: nil, ocBoard: "",
                                      fcFirmware: "3d1f5ca2-m2+20261120", ocFirmware: ""),
                       .beetle)
        XCTAssertEqual(BoardFamily.of(fcBoard: nil, ocBoard: nil,
                                      fcFirmware: "", ocFirmware: "d7017c0-v10+20260827-1919"),
                       .mantis)
        XCTAssertNil(BoardFamily.of(fcBoard: nil, ocBoard: nil,
                                    fcFirmware: "unknown", ocFirmware: "3d1f5ca2-dirty"))
    }

    // MARK: - Features

    func testStoreEditionHasNoFinOrCameraScreensForAnyBoard() {
        for board in [BoardFamily.beetle, .mantis, nil] {
            let f = AppFeatures(edition: .store, board: board)
            XCTAssertFalse(f.finControl, "\(String(describing: board))")
            XCTAssertFalse(f.camera, "\(String(describing: board))")
        }
    }

    func testDevEditionFollowsTheBoard() {
        XCTAssertFalse(AppFeatures(edition: .dev, board: .beetle).finControl)
        XCTAssertFalse(AppFeatures(edition: .dev, board: .beetle).camera)
        XCTAssertTrue(AppFeatures(edition: .dev, board: .mantis).finControl)
        XCTAssertTrue(AppFeatures(edition: .dev, board: .mantis).camera)
        // Cannot tell (nothing connected, a relay, an unprovisioned board):
        // the dev edition shows everything.
        XCTAssertTrue(AppFeatures(edition: .dev, board: nil).finControl)
        XCTAssertTrue(AppFeatures(edition: .dev, board: nil).camera)
    }

    func testStoreEditionSupportsTheBeetleOnly() {
        XCTAssertTrue(AppFeatures(edition: .store, board: .beetle).boardSupported)
        XCTAssertFalse(AppFeatures(edition: .store, board: .mantis).boardSupported)
        // A Beetle reads as unknown until its identity arrives: no warning yet.
        XCTAssertTrue(AppFeatures(edition: .store, board: nil).boardSupported)
        for board in [BoardFamily.beetle, .mantis, nil] {
            XCTAssertTrue(AppFeatures(edition: .dev, board: board).boardSupported)
        }
    }

    func testThisTestBuildIsTheDevEdition() {
        XCTAssertEqual(AppEdition.current, .dev)
    }

    // MARK: - Firmware catalog

    private func img(_ project: String, _ board: String?) -> String {
        let boardField = board.map { ",\"board\":\"\($0)\"" } ?? ""
        return """
        {"file":"\(project)-\(board ?? "any").bin","project":"\(project)",
         "version":"3d1f5ca2+1","chip_id":9,"chip":"ESP32-S3","size":644336,
         "sha256":"\(String(repeating: "a", count: 64))"\(boardField)}
        """
    }

    private var manifest: FirmwareManifest {
        let images = [img("flight_computer", "v10"), img("flight_computer", "m2"),
                      img("flight_computer", nil),
                      img("out_computer", "v10"), img("out_computer", "m2"),
                      img("base_station", "v3"), img("base_station", "v4"),
                      img("base_station", "v5")]
        return FirmwareManifest.parse(
            "{\"manifest_version\":1,\"tag\":\"fw-v1.0.0\",\"images\":[\(images.joined(separator: ","))]}")!
    }

    private func boards(_ project: String, _ edition: AppEdition,
                        provisioned: String? = nil) -> [String?] {
        FirmwareCatalog.forUnit(manifest, expectedProject: project,
                                provisionedBoard: provisioned, edition: edition).map(\.board)
    }

    func testStoreCatalogOffersOnlyBeetleRocketImages() {
        XCTAssertEqual(boards("flight_computer", .store, provisioned: "m2"), ["m2", nil])
        XCTAssertEqual(boards("out_computer", .store, provisioned: "m2"), ["m2"])
        // A Mantis connected to the store app is offered nothing of its own.
        XCTAssertEqual(boards("out_computer", .store, provisioned: "v10"), ["m2"])
        XCTAssertNil(FirmwareCatalog.best(manifest, expectedProject: "out_computer",
                                          provisionedBoard: "v10", edition: .store))
    }

    func testStoreCatalogOffersEveryBaseStation() {
        XCTAssertEqual(Set(boards("base_station", .store)), Set(["v3", "v4", "v5"]))
    }

    func testDevCatalogIsUnchanged() {
        XCTAssertEqual(boards("flight_computer", .dev, provisioned: "v10"), ["v10", nil, "m2"])
        XCTAssertEqual(Set(boards("out_computer", .dev)), Set(["m2", "v10"]))
    }
}
