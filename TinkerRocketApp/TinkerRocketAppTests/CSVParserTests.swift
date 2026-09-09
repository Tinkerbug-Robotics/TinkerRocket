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

// MARK: - #1081 streaming parse + preview ceiling (twins of CsvParserTest, #636)

final class CSVParserStreamingTests: XCTestCase {

    private func write(_ csv: String) throws -> URL {
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("stream-\(UUID().uuidString).csv")
        try csv.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    /// 1000 Hz source (1 ms apart), capped to 100 Hz -> every 10th row.
    /// "flag" latches partway through, like Launch/Apogee/Landed in a real
    /// log — the property that makes striding safe.
    func testMaxSampleHzThinsByTimeAndPreservesLatchedFlags() throws {
        let rows = 1000, latchAt = 400
        var csv = "Time (ms),v,flag\n"
        for i in 0..<rows { csv += "\(i),\(i * 2),\(i >= latchAt ? 1 : 0)\n" }
        let url = try write(csv)

        let full = try CSVParser.parse(url: url)
        let thin = try CSVParser.parse(url: url, maxSampleHz: 100.0)

        XCTAssertEqual(full.rowCount, rows)
        XCTAssertEqual(thin.rowCount, 100, "1000 rows at 1 ms, capped to 100 Hz")

        XCTAssertEqual(thin.columns["Time (ms)"]![0], 0.0)
        XCTAssertEqual(thin.columns["Time (ms)"]![1], 10.0)
        XCTAssertEqual(thin.columns["v"]![1], 20.0, "values stay aligned to their row")

        let flag = thin.columns["flag"]!
        XCTAssertTrue(flag.contains(1.0), "latched flag lost by decimation")
        XCTAssertEqual(flag.first, 0.0)
        XCTAssertEqual(flag.last, 1.0)
    }

    func testMaxSampleHzNilKeepsEveryRow() throws {
        let csv = "Time (ms),v\n" + (0..<50).map { "\($0),\($0)" }.joined(separator: "\n")
        let url = try write(csv)
        XCTAssertEqual(try CSVParser.parse(url: url, maxSampleHz: nil).rowCount, 50)
        XCTAssertEqual(try CSVParser.parse(url: url).rowCount, 50)
    }

    /// The streaming reader must produce exactly what the whole-file parser
    /// did: a line split across two 64 KB chunks is one row, not two.
    func testChunkBoundaryDoesNotSplitARow() throws {
        // Rows of ~40 bytes; 2000 of them straddle the 64 KB boundary many times.
        var csv = "Time (ms),a,b\n"
        for i in 0..<2000 { csv += "\(i),\(String(repeating: "7", count: 20)).5,\(i)\n" }
        let url = try write(csv)
        let d = try CSVParser.parse(url: url)
        XCTAssertEqual(d.rowCount, 2000)
        XCTAssertEqual(d.columns["b"]![1999], 1999.0)
        XCTAssertEqual(d.columns["a"]![1999], 77777777777777777777.5, accuracy: 1e5)
    }

    func testCRLFAndUnparseableTimeAreHandled() throws {
        let url = try write("Time (ms),v\r\n0,1\r\nabc,2\r\n5,3\r\n")
        // Unparseable time keeps the row; CRLF does not poison the last field.
        let thin = try CSVParser.parse(url: url, maxSampleHz: 100.0)
        XCTAssertEqual(thin.rowCount, 2, "row 0 kept, 'abc' kept, 5 ms dropped under a 10 ms interval")
        XCTAssertEqual(thin.columns["v"]![0], 1.0)
    }
}

