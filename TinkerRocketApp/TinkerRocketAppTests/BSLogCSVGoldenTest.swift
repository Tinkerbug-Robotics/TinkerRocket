import XCTest
@testable import TinkerRocketApp

/// Swift base-station bin→CSV against the committed golden.
///
/// The golden's INPUT (tests_cpp/fixtures/wire/csv/bs_tiny.bin) is packed by
/// wire_fixture_gen using the REAL firmware LoRa packers, so this is not three
/// ports agreeing with each other — it is three ports agreeing with the
/// firmware. A round-trip test would happily pass while every implementation
/// shared the same wrong idea of the layout.
final class BSLogCSVGoldenTest: XCTestCase {

    private var goldenDir: URL {
        WireFixtures.root.deletingLastPathComponent().appendingPathComponent("csv_golden")
    }

    private func bytes(_ rel: String) throws -> [UInt8] {
        [UInt8](try Data(contentsOf: WireFixtures.url(rel)))
    }

    private func golden() throws -> String {
        try String(contentsOf: goldenDir.appendingPathComponent("bs_tiny.expected.csv"),
                   encoding: .utf8)
    }

    func testBaseStationCsvMatchesGoldenByteForByte() throws {
        let csv = try BSLogCSVGenerator.writeCSV(try bytes("csv/bs_tiny.bin"))
        let want = try golden()
        if csv != want {
            let a = csv.components(separatedBy: "\n")
            let b = want.components(separatedBy: "\n")
            for i in 0..<max(a.count, b.count) {
                let al = i < a.count ? a[i] : "<missing>"
                let bl = i < b.count ? b[i] : "<missing>"
                if al != bl {
                    XCTFail("CSV differs at line \(i + 1):\n generated: \(al)\n golden:    \(bl)")
                    return
                }
            }
        }
        XCTAssertEqual(csv, want)
    }

    func testHeaderIsFirmwareColumnsPlusFrame() {
        let cols = BSLogCSVGenerator.columns
        XCTAssertEqual(cols.count, 40, "39 firmware columns + frame")
        XCTAssertEqual(cols.first, "time_ms")
        XCTAssertEqual(cols.last, "frame",
                       "frame is APPENDED so name-keyed consumers are unaffected")
        XCTAssertEqual(cols[cols.count - 2], "rocket_id")
    }

    func testForwardFillCarriesSlowFieldsOntoFastRows() throws {
        let csv = try BSLogCSVGenerator.writeCSV(try bytes("csv/bs_tiny.bin"))
        let lines = csv.trimmingCharacters(in: .newlines).components(separatedBy: "\n")
        let cols = lines[0].components(separatedBy: ",")
        let rows = lines.dropFirst().map { $0.components(separatedBy: ",") }
            .filter { $0[1] != "EVENT" }

        let iCam = cols.firstIndex(of: "cam_a")!
        let iPalt = cols.firstIndex(of: "pressure_alt")!
        let iFrame = cols.firstIndex(of: "frame")!

        guard let firstSlow = rows.firstIndex(where: { $0[iFrame] == "slow" }), firstSlow > 0 else {
            return XCTFail("the fixture must contain a slow frame after a fast one")
        }
        XCTAssertGreaterThan(Double(rows[firstSlow][iPalt]) ?? 0, 0,
                             "a slow frame must not blank the position")

        guard let nextFast = rows[(firstSlow + 1)...].first(where: { $0[iFrame] == "fast" }) else {
            return XCTFail("expected a fast frame after the slow one")
        }
        XCTAssertEqual(nextFast[iCam], "1.480",
                       "a fast frame must not blank the rail current from the slow frame before it")
    }

    func testUnknownRssiRendersAsNanNotZeroDbm() throws {
        let csv = try BSLogCSVGenerator.writeCSV(try bytes("csv/bs_tiny.bin"))
        let lines = csv.trimmingCharacters(in: .newlines).components(separatedBy: "\n")
        let iRssi = lines[0].components(separatedBy: ",").firstIndex(of: "rssi")!
        XCTAssertTrue(lines.dropFirst().contains { $0.components(separatedBy: ",")[iRssi] == "nan" },
                      "0 dBm is a legal reading; 'no reading' must not render as one")
    }

    func testEventsAreInterleavedByTime() throws {
        let csv = try BSLogCSVGenerator.writeCSV(try bytes("csv/bs_tiny.bin"))
        let lines = csv.trimmingCharacters(in: .newlines).components(separatedBy: "\n").dropFirst()
        let times = lines.compactMap { Int($0.components(separatedBy: ",")[0]) }
        XCTAssertEqual(times, times.sorted(), "rows and events must be in arrival order")
        XCTAssertTrue(lines.contains { $0.components(separatedBy: ",")[1] == "EVENT" })
    }

    func testRocketLogIsRefusedRatherThanParsedAsGarbage() throws {
        let rocket = try bytes("csv/tiny_flight.bin")
        XCTAssertThrowsError(try BSLogCSVGenerator.writeCSV(rocket))
    }

    func testDetectionIsByMagic() throws {
        XCTAssertTrue(BSLogCSVGenerator.isBaseStationLog(try bytes("csv/bs_tiny.bin")))
        XCTAssertFalse(BSLogCSVGenerator.isBaseStationLog(try bytes("csv/tiny_flight.bin")))
    }
}
