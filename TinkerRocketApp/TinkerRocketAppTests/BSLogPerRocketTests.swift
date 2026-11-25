//
//  BSLogPerRocketTests.swift
//  TinkerRocketAppTests
//
//  Three rockets, one base station: the base station keeps ONE log, every
//  frame carrying its rocket id, and the app renders it at download as the
//  combined CSV plus one CSV per rocket ("Rocket N" in Saved Flights).
//

import XCTest
@testable import TinkerRocketApp

final class BSLogPerRocketTests: XCTestCase {

    // MARK: - Fixtures

    private func singleRocketLog() throws -> [UInt8] {
        [UInt8](try Data(contentsOf: WireFixtures.url("csv/bs_tiny.bin")))
    }

    /// bs_tiny.bin (packed by the real firmware packers, one rocket) with its
    /// twelve LoRa records dealt round-robin to rockets 1, 2 and 3 by
    /// rewriting each frame's rocket-id byte. Record layout: preamble(4)
    /// type(1) len(1) payload(len) trailer(2); the LoRa frame starts 12 bytes
    /// into the payload and its rocket id is frame byte 1. The parser does not
    /// read the trailer, so leaving it stale changes nothing it decodes.
    private func threeRocketLog() throws -> [UInt8] {
        var b = try singleRocketLog()
        var i = 0, lora = 0
        while i + 6 <= b.count {
            guard b[i] == 0xAA, b[i + 1] == 0x55, b[i + 2] == 0xAA, b[i + 3] == 0x55 else {
                i += 1
                continue
            }
            let type = b[i + 4], len = Int(b[i + 5])
            if type == 0xFC, len > 13 {
                b[i + 6 + 12 + 1] = UInt8(lora % 3 + 1)
                lora += 1
            }
            i += 6 + len + 2
        }
        XCTAssertEqual(lora, 12, "the fixture's shape changed; recheck the rewrite")
        return b
    }

    private func lines(_ csv: String) -> [String] {
        csv.trimmingCharacters(in: .newlines).components(separatedBy: "\n")
    }

    private func rocketIDColumn() -> Int { BSLogCSVGenerator.columns.firstIndex(of: "rocket_id")! }

    // MARK: - Generator

    func testRocketIDs_ofTheThreeRocketFixture() throws {
        XCTAssertEqual(try BSLogCSVGenerator.rocketIDs(in: try threeRocketLog()), [1, 2, 3])
        XCTAssertEqual(try BSLogCSVGenerator.rocketIDs(in: try singleRocketLog()), [1])
    }

    /// The per-rocket file is exactly the combined file's rows for that rocket
    /// plus the base station's events — the forward-fill and seq gaps are
    /// already per rocket, so a rocket's rows do not change when its
    /// neighbours are taken out.
    func testEachRocketsCSV_isTheCombinedFilteredToIt() throws {
        let log = try threeRocketLog()
        let (combined, byRocket) = try BSLogCSVGenerator.writeCSVs(log)
        XCTAssertEqual(combined, try BSLogCSVGenerator.writeCSV(log))
        XCTAssertEqual(byRocket.map(\.rocketID), [1, 2, 3])

        let all = lines(combined)
        var dataRows = 0
        for (rid, csv) in byRocket {
            XCTAssertEqual(csv, try BSLogCSVGenerator.writeCSV(log, rocketID: rid))
            let want = [all[0]] + all.dropFirst().filter { line in
                let cells = line.components(separatedBy: ",")
                return cells[1] == "EVENT" || cells[rocketIDColumn()] == String(rid)
            }
            XCTAssertEqual(lines(csv), want, "rocket \(rid)")
            let rows = lines(csv).dropFirst().map { $0.components(separatedBy: ",") }
                .filter { $0[1] != "EVENT" }
            XCTAssertEqual(rows.count, 4, "twelve frames dealt three ways")
            XCTAssertTrue(rows.allSatisfy { $0[rocketIDColumn()] == String(rid) })
            dataRows += rows.count
        }
        let combinedRows = all.dropFirst().filter { !$0.contains(",EVENT,") }
        XCTAssertEqual(dataRows, combinedRows.count, "no row lost or duplicated")
    }

    func testEventsAppearInEveryRocketsCSV() throws {
        let (_, byRocket) = try BSLogCSVGenerator.writeCSVs(try threeRocketLog())
        for (rid, csv) in byRocket {
            XCTAssertEqual(lines(csv).filter { $0.contains(",EVENT,") }.count, 1, "rocket \(rid)")
        }
    }

    // MARK: - Names

    func testSplitNames_roundTrip_andKeepTheLogsDate() {
        let name = FileCache.bsRocketCSVName(combined: "lora_20261024_101500.csv", rocketID: 2)
        XCTAssertEqual(name, "lora_20261024_101500_rocket2.csv")
        XCTAssertEqual(FileCache.bsRocketSplit(of: name)?.combined, "lora_20261024_101500.csv")
        XCTAssertEqual(FileCache.bsRocketSplit(of: name)?.rocketID, 2)

        // The base station's same-second collision counter stays in the stem.
        let counted = FileCache.bsRocketSplit(of: "lora_20261024_101500_3_rocket12.csv")
        XCTAssertEqual(counted?.combined, "lora_20261024_101500_3.csv")
        XCTAssertEqual(counted?.rocketID, 12)

        for other in ["lora_20261024_101500.csv", "lora_20261024_101500_2.csv",
                      "flight_20261024_101500_rocket2.csv", "lora_20261024_101500_rocket2.bin",
                      "lora_20261024_101500_rocketx.csv"] {
            XCTAssertNil(FileCache.bsRocketSplit(of: other), other)
        }
    }

    // MARK: - Cache: write, list, delete

    private var tempDir: URL!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("BSLogPerRocketTests-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    /// The download path's order: the .bin first, then the renderings.
    private func cacheDownload(_ cache: FileCache, _ bytes: [UInt8], as name: String) throws {
        let src = tempDir.appendingPathComponent("download-\(name)")
        try Data(bytes).write(to: src)
        _ = try cache.cacheBinary(at: src, for: name)
        try cache.cacheBaseStationLogCSVs(bytes: bytes, binaryFilename: name)
    }

    func testThreeRocketLog_listsTheCombinedThenOneEntryPerRocket() throws {
        let cache = FileCache(documentsURL: tempDir)
        try cacheDownload(cache, try threeRocketLog(), as: "lora_20261024_101500.bin")

        let flights = cache.listCachedFlights()
        XCTAssertEqual(flights.map(\.name), ["lora_20261024_101500.csv",
                                             "lora_20261024_101500_rocket1.csv",
                                             "lora_20261024_101500_rocket2.csv",
                                             "lora_20261024_101500_rocket3.csv"])
        XCTAssertEqual(flights.map(\.bsLogRocketID), [nil, 1, 2, 3])
        XCTAssertTrue(flights.allSatisfy { $0.type == .loraLog })

        // Every entry keeps the log's date, and a split names its rocket.
        let date = try XCTUnwrap(flights[0].flightDate)
        XCTAssertTrue(flights.allSatisfy { $0.flightDate == date })
        XCTAssertTrue(flights[2].displayTitle.hasSuffix(" · Rocket 2"))
        XCTAssertFalse(flights[0].displayTitle.contains("Rocket"))

        // All share the one .bin, so each rides the Share sheet with it.
        let bin = try XCTUnwrap(flights[0].binaryURL)
        XCTAssertTrue(flights.allSatisfy { $0.binaryURL == bin })
    }

    func testSingleRocketLog_getsNoSplit() throws {
        let cache = FileCache(documentsURL: tempDir)
        try cacheDownload(cache, try singleRocketLog(), as: "lora_20261024_101500.bin")
        XCTAssertEqual(cache.listCachedFlights().map(\.name), ["lora_20261024_101500.csv"])
    }

    func testRedownload_dropsSplitsTheLogNoLongerHas() throws {
        let cache = FileCache(documentsURL: tempDir)
        try cacheDownload(cache, try threeRocketLog(), as: "lora_20261024_101500.bin")
        try cacheDownload(cache, try singleRocketLog(), as: "lora_20261024_101500.bin")
        XCTAssertEqual(cache.listCachedFlights().map(\.name), ["lora_20261024_101500.csv"])
    }

    func testDeletingOneRocket_keepsTheSharedLog() throws {
        let cache = FileCache(documentsURL: tempDir)
        try cacheDownload(cache, try threeRocketLog(), as: "lora_20261024_101500.bin")
        let split = try XCTUnwrap(cache.listCachedFlights().first { $0.bsLogRocketID == 2 })

        cache.deleteCachedFlight(split)

        let left = cache.listCachedFlights()
        XCTAssertEqual(left.compactMap(\.bsLogRocketID), [1, 3])
        XCTAssertNotNil(left.first { $0.bsLogRocketID == nil }, "the combined CSV stays")
        XCTAssertNotNil(cache.getCachedBinary(for: "lora_20261024_101500.bin"),
                        "the .bin belongs to every rocket in it")
    }

    func testDeletingTheCombinedLog_takesItsRocketsWithIt() throws {
        let cache = FileCache(documentsURL: tempDir)
        try cacheDownload(cache, try threeRocketLog(), as: "lora_20261024_101500.bin")
        try cacheDownload(cache, try threeRocketLog(), as: "lora_20261024_120000.bin")
        let combined = try XCTUnwrap(cache.listCachedFlights()
            .first { $0.name == "lora_20261024_101500.csv" })

        cache.deleteCachedFlight(combined)

        let left = cache.listCachedFlights().map(\.name)
        XCTAssertFalse(left.contains { $0.hasPrefix("lora_20261024_101500") })
        XCTAssertEqual(left.count, 4, "the other log and its three rockets are untouched")
        XCTAssertNil(cache.getCachedBinary(for: "lora_20261024_101500.bin"))
    }
}
