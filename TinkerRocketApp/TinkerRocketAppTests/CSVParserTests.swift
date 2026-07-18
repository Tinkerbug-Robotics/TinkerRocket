import XCTest
@testable import TinkerRocketApp

/// #236: `CSVParser.parse` trapped with "Range requires lowerBound <=
/// upperBound" on LoRa logs whose corrupted-telemetry rows have MORE comma
/// fields than the header (e.g. a garbled value like "166.65.5" splitting into
/// extra fields).  The padding loop `fields.count..<columnCount` inverted when
/// `fields.count > columnCount`.  Parsing must be crash-proof and column-aligned.
final class CSVParserTests: XCTestCase {

    private func parse(_ csv: String) throws -> FlightCSVData {
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("csvtest-\(UUID().uuidString).csv")
        try csv.write(to: url, atomically: true, encoding: .utf8)
        defer { try? FileManager.default.removeItem(at: url) }
        return try CSVParser.parse(url: url)
    }

    /// The exact #236 crash: a row with MORE fields than the header.
    func testParse_RowWithExtraFields_DoesNotTrap() throws {
        let data = try parse("a,b,c\n1,2,3\n4,5,6,7,8\n9,10,11\n")
        XCTAssertEqual(data.rowCount, 3)
        // Every column stays aligned (one value per row); extra fields ignored.
        XCTAssertEqual(data.columns["a"], [1.0, 4.0, 9.0])
        XCTAssertEqual(data.columns["b"], [2.0, 5.0, 10.0])
        XCTAssertEqual(data.columns["c"], [3.0, 6.0, 11.0])
    }

    /// Rows with FEWER fields than the header still pad with NaN.
    func testParse_RowWithMissingFields_PadsNaN() throws {
        let data = try parse("a,b,c\n1,2,3\n4,5\n")
        XCTAssertEqual(data.rowCount, 2)
        XCTAssertEqual(data.columns["a"], [1.0, 4.0])
        XCTAssertEqual(data.columns["b"], [2.0, 5.0])
        XCTAssertEqual(data.columns["c"]?[0], 3.0)
        XCTAssertEqual(data.columns["c"]?.count, 2)
        XCTAssertTrue(data.columns["c"]?[1].isNaN ?? false)   // padded
    }

    /// Mirrors the real LoRa shape: 36-col header with a 38-field garbled row.
    func testParse_LoRaShapedGarbledRow_DoesNotTrap() throws {
        let header  = (0..<36).map { "c\($0)" }.joined(separator: ",")
        let good    = (0..<36).map { "\($0)"  }.joined(separator: ",")
        let garbled = good + ",166.65.5,extra"   // 38 fields
        let data = try parse("\(header)\n\(good)\n\(garbled)\n")
        XCTAssertEqual(data.rowCount, 2)
        XCTAssertEqual(data.columns["c0"], [0.0, 0.0])
        XCTAssertEqual(data.columns["c35"]?.count, 2)        // all columns aligned
    }

    /// Exports from 2026-07-14…07-16 app builds have unquoted commas in the
    /// three #514 attitude names, so the header row carries three more tokens
    /// than every data row and all later columns load shifted. The parser must
    /// re-join those tokens into the current semicolon names so the values
    /// land under the right columns.
    func testParse_SplitAttitudeHeaderNames_AreRepairedAndAligned() throws {
        let header = "Quat q3,Roll (deg, body-Z azimuth),Pitch (deg, ZYX Euler),"
                   + "Yaw (deg, ZYX Euler),Roll Command (deg),Position Up (m)"
        let data = try parse("\(header)\n0.5,10,20,30,0.04,407.2\n")
        XCTAssertEqual(data.headers, [
            "Quat q3",
            "Roll (deg; body-Z azimuth)", "Pitch (deg; ZYX Euler)", "Yaw (deg; ZYX Euler)",
            "Roll Command (deg)", "Position Up (m)",
        ])
        XCTAssertEqual(data.columns["Quat q3"], [0.5])
        XCTAssertEqual(data.columns["Roll (deg; body-Z azimuth)"], [10.0])
        XCTAssertEqual(data.columns["Roll Command (deg)"], [0.04])
        XCTAssertEqual(data.columns["Position Up (m)"], [407.2])   // not shifted
    }

    /// Headers without the broken comma names pass through untouched — a
    /// lone "Roll (deg)" (pre-#514) or the current semicolon names must not
    /// trigger the repair.
    func testParse_CurrentAndLegacyHeaders_NotRewritten() throws {
        let data = try parse("Roll (deg; body-Z azimuth),Roll (deg),Yaw (deg)\n1,2,3\n")
        XCTAssertEqual(data.headers,
                       ["Roll (deg; body-Z azimuth)", "Roll (deg)", "Yaw (deg)"])
        XCTAssertEqual(data.columns["Roll (deg)"], [2.0])
    }
}
