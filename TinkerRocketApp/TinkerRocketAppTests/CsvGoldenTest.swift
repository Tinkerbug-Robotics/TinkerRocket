import XCTest
@testable import TinkerRocketApp

/// The bin→CSV golden (android-port plan Phase 1 exit gate).
///
/// The emitter-generated synthetic flight (tests_cpp/fixtures/wire/csv/
/// tiny_flight.bin — deterministic, integer-derived) runs through the REAL
/// CSVGenerator; the output CSV + summary JSON are compared byte-for-byte
/// against committed goldens in tests_cpp/fixtures/csv_golden/.  The goldens
/// are APP output (this suite bootstraps them when absent), deliberately
/// outside fixtures/wire/ — the C++ freshness walk owns that tree.
///
/// The Kotlin CsvGenerator golden test consumes the SAME files: iOS behavior
/// is the reference, so any Kotlin divergence (rounding policy, forward-fill,
/// trim, column set) fails against bytes this suite proved iOS produces.
final class CsvGoldenTest: XCTestCase {

    private var goldenDir: URL {
        WireFixtures.root.deletingLastPathComponent().appendingPathComponent("csv_golden")
    }

    func testTinyFlightCsvAndSummaryMatchGoldens() throws {
        let bin = WireFixtures.url("csv/tiny_flight.bin")
        XCTAssertTrue(FileManager.default.fileExists(atPath: bin.path),
                      "regen-wire-fixtures first")

        let tmp = FileManager.default.temporaryDirectory
            .appendingPathComponent("tiny_flight_\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: tmp, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: tmp) }

        let csvOut = tmp.appendingPathComponent("tiny_flight.csv")
        let generator = CSVGenerator()
        let summary = try generator.generateCSV(from: bin, to: csvOut)
        let summaryOut = tmp.appendingPathComponent("tiny_flight.summary.json")
        try generator.writeSummary(summary, to: summaryOut)

        let csvGolden = goldenDir.appendingPathComponent("tiny_flight.expected.csv")
        let summaryGolden = goldenDir.appendingPathComponent("tiny_flight.expected.summary.json")

        // Bootstrap: first run writes the goldens and fails, forcing a human
        // review + commit; every later run compares byte-for-byte.
        guard FileManager.default.fileExists(atPath: csvGolden.path),
              FileManager.default.fileExists(atPath: summaryGolden.path) else {
            try FileManager.default.createDirectory(at: goldenDir, withIntermediateDirectories: true)
            try FileManager.default.copyItem(at: csvOut, to: csvGolden)
            try FileManager.default.copyItem(at: summaryOut, to: summaryGolden)
            XCTFail("goldens bootstrapped at \(goldenDir.path) — review the diff, commit, rerun")
            return
        }

        let generatedCsv = try String(contentsOf: csvOut, encoding: .utf8)
        let goldenCsv = try String(contentsOf: csvGolden, encoding: .utf8)
        if generatedCsv != goldenCsv {
            // Whole-file inequality is unreadable; find the first differing line.
            let g = generatedCsv.components(separatedBy: "\n")
            let e = goldenCsv.components(separatedBy: "\n")
            for i in 0..<max(g.count, e.count) {
                let gl = i < g.count ? g[i] : "<missing>"
                let el = i < e.count ? e[i] : "<missing>"
                if gl != el {
                    XCTFail("CSV differs at line \(i + 1):\n generated: \(gl)\n golden:    \(el)")
                    break
                }
            }
        }

        XCTAssertEqual(try String(contentsOf: summaryOut, encoding: .utf8),
                       try String(contentsOf: summaryGolden, encoding: .utf8),
                       "summary JSON drifted")

        // Structural sanity pins (also documents the fixture's shape).
        let lines = generatedCsv.components(separatedBy: "\n").filter { !$0.isEmpty }
        XCTAssertEqual(lines.count, 161 + 1, "expected 161 data rows + header")
        XCTAssertEqual(lines[0].components(separatedBy: ",").count, 62, "62 columns")
    }
}
