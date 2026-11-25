import XCTest
@testable import TinkerRocketApp

/// #1658 step 3: the out computer names itself in the log (OC_IDENTITY_MSG,
/// 0x87), so a flight's `source` no longer depends on which phone downloaded
/// it. The converter fills the out computer's part from the log; the download
/// adds what only the app knows, and the log wins where both speak.
final class OcIdentityTests: XCTestCase {

    func testDecodesTheSharedFixture() throws {
        let side = WireFixtures.sidecar("logframes/ocidentity_68.bin")
        let id = try OcIdentityData(from: WireFixtures.data("logframes/ocidentity_68.bin"))
        XCTAssertEqual(Int64(id.time_us), side.i64("time_us"))
        XCTAssertEqual(Int(id.version), side.int("version"))
        XCTAssertEqual(Int(id.network_id), side.int("network_id"))
        XCTAssertEqual(Int(id.rocket_id), side.int("rocket_id"))
        XCTAssertEqual(Int(id.board_rev_code), side.int("board_rev_code"))
        XCTAssertEqual(id.unit_id, side["unit_id"] as? String)
        // The fixture's name fills all 20 bytes with no NUL; it still reads whole.
        XCTAssertEqual(id.unit_name, "Twenty-char unitname")
        XCTAssertEqual(id.unit_name, side["unit_name"] as? String)
        XCTAssertEqual(id.fw_version, side["fw_version"] as? String)
        XCTAssertEqual(id.boardRevName, "M1")
    }

    func testWrongSizeIsRefused() {
        XCTAssertThrowsError(try OcIdentityData(from: Data(count: 67)))
        XCTAssertThrowsError(try OcIdentityData(from: Data(count: 69)))
    }

    /// The converter reads the record out of the log on its own: no device.
    func testConverterFillsTheOutComputerPart() throws {
        let tmp = FileManager.default.temporaryDirectory
            .appendingPathComponent("oc_identity_\(UUID().uuidString).csv")
        defer { try? FileManager.default.removeItem(at: tmp) }
        let summary = try CSVGenerator().generateCSV(from: WireFixtures.url("csv/tiny_flight.bin"), to: tmp)
        let source = try XCTUnwrap(summary.source)
        XCTAssertEqual(source.unit_id, "9e8a1b2c")
        XCTAssertEqual(source.unit_name, "Twenty-char unitname")
        XCTAssertEqual(source.rocket_id, 2)
        XCTAssertEqual(source.oc_board, "M1")
        XCTAssertEqual(source.oc_fw, "41b2ab92-m1+20261022-1126")
        XCTAssertNil(source.rocket_profile, "only the app knows the profile")
        XCTAssertNil(source.downloaded_at)
    }

    /// The unit was renamed after the flight: the log keeps the name it flew
    /// under, and the app still adds the profile, the FC and when.
    func testTheLogWinsForWhatItKnows() {
        let log = FlightSource(unit_id: "9e8a1b2c", unit_name: "Beetle", rocket_id: 2,
                               oc_board: "M1", oc_fw: "41b2ab92")
        let live = FlightSource(unit_id: "9e8a1b2c", unit_name: "Beetle (renamed)",
                                rocket_profile: "RIM66", rocket_id: 3, fc_board: "M1",
                                oc_board: "M1 (asserted)", fc_fw: "41b2ab92", oc_fw: "ffffffff",
                                fc_gnss: "LC86G", downloaded_at: "2026-10-24T18:03:00Z")
        let m = FlightSource.merged(log: log, live: live)
        XCTAssertEqual(m?.unit_name, "Beetle")
        XCTAssertEqual(m?.rocket_id, 2)
        XCTAssertEqual(m?.oc_board, "M1")
        XCTAssertEqual(m?.oc_fw, "41b2ab92")
        XCTAssertEqual(m?.rocket_profile, "RIM66")
        XCTAssertEqual(m?.fc_gnss, "LC86G")
        XCTAssertEqual(m?.downloaded_at, "2026-10-24T18:03:00Z")
    }

    func testEitherSideAloneSurvives() {
        let live = FlightSource(unit_name: "Atlas")
        XCTAssertEqual(FlightSource.merged(log: nil, live: live), live)
        XCTAssertEqual(FlightSource.merged(log: live, live: nil), live)
        XCTAssertNil(FlightSource.merged(log: nil, live: nil))
    }
}
