import XCTest
@testable import TinkerRocketApp

/// Two boards can both name a log `flight_3.bin` (a board without a phone time
/// sync numbers its own logs), and the cache used to key on that name alone:
/// the second download replaced the first board's flight, CSV and summary.
/// The cache name now carries the unit ID ("flight_3__a1cae150.bin").
final class FlightCacheNamesTests: XCTestCase {

    private var tempDir: URL!
    private var cache: FileCache!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("cache_names_\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
        cache = FileCache(documentsURL: tempDir)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    private func source(_ bytes: String) throws -> URL {
        let url = tempDir.appendingPathComponent(UUID().uuidString + ".bin")
        try Data(bytes.utf8).write(to: url)
        return url
    }

    private func put(_ text: String, _ rel: String) throws {
        try text.write(to: tempDir.appendingPathComponent(rel), atomically: true, encoding: .utf8)
    }

    func testRocketFlightsCarryTheUnitID() {
        XCTAssertEqual(FileCache.cacheName(for: "flight_3.bin", unitID: "a1cae150"), "flight_3__a1cae150.bin")
        XCTAssertEqual(FileCache.cacheName(for: "flight_20261024_161431.bin", unitID: "a1cae150"),
                       "flight_20261024_161431__a1cae150.bin")
        // already qualified: unchanged
        XCTAssertEqual(FileCache.cacheName(for: "flight_3__a1cae150.bin", unitID: "a1cae150"),
                       "flight_3__a1cae150.bin")
    }

    func testOtherNamesAndUnknownUnitsKeepTheBoardsName() {
        XCTAssertEqual(FileCache.cacheName(for: "lora_003.bin", unitID: "a1cae150"), "lora_003.bin")
        XCTAssertEqual(FileCache.cacheName(for: "flight_3.bin", unitID: ""), "flight_3.bin")
        // never a path component: anything but hex keeps the board's name
        XCTAssertEqual(FileCache.cacheName(for: "flight_3.bin", unitID: "../x"), "flight_3.bin")
    }

    func testTheUnitReadsBackOutOfTheName() {
        XCTAssertEqual(FileCache.unitID(inCacheName: "flight_3__a1cae150.csv"), "a1cae150")
        XCTAssertNil(FileCache.unitID(inCacheName: "flight_20261024_161431.csv"))
    }

    /// The bug: the same board filename from two units. Both copies survive.
    func testTwoBoardsSameNameBothKept() throws {
        let a = FileCache.cacheName(for: "flight_3.bin", unitID: "a1cae150")
        let b = FileCache.cacheName(for: "flight_3.bin", unitID: "9e8a1b2c")
        _ = try cache.cacheBinary(at: try source("board A's flight"), for: a)
        _ = try cache.cacheBinary(at: try source("board B's flight, longer"), for: b)
        let aURL = try XCTUnwrap(cache.getCachedBinary(for: a))
        let bURL = try XCTUnwrap(cache.getCachedBinary(for: b))
        XCTAssertEqual(try String(contentsOf: aURL, encoding: .utf8), "board A's flight")
        XCTAssertEqual(try String(contentsOf: bURL, encoding: .utf8), "board B's flight, longer")

        try put("t\n0\n", "CSVCache/flight_3__a1cae150.csv")
        try put("t\n0\n", "CSVCache/flight_3__9e8a1b2c.csv")
        let titles = cache.listCachedFlights().map(\.displayTitle).sorted()
        XCTAssertEqual(titles, ["flight_3 · 9e8a1b2c", "flight_3 · a1cae150"])
    }

    /// A copy cached before unit IDs (the board's own name) still counts as
    /// this unit's — unless its summary says another unit downloaded it.
    func testALegacyCopyResolvesUnlessItIsAnotherUnits() throws {
        _ = try cache.cacheBinary(at: try source("old"), for: "flight_3.bin")
        try put("t\n0\n", "CSVCache/flight_3.csv")
        XCTAssertEqual(cache.cachedName(for: "flight_3.bin", unitID: "a1cae150"), "flight_3.bin")

        try put(#"{ "source" : { "unit_id" : "9e8a1b2c" } }"#, "CSVCache/flight_3.json")
        XCTAssertEqual(cache.cachedName(for: "flight_3.bin", unitID: "9e8a1b2c"), "flight_3.bin")
        XCTAssertEqual(cache.cachedName(for: "flight_3.bin", unitID: "a1cae150"), "flight_3__a1cae150.bin",
                       "another unit's old copy is not this board's file")
    }

    func testAQualifiedCopyWinsOverALegacyOne() throws {
        _ = try cache.cacheBinary(at: try source("old"), for: "flight_3.bin")
        _ = try cache.cacheBinary(at: try source("new"), for: "flight_3__a1cae150.bin")
        XCTAssertEqual(cache.cachedName(for: "flight_3.bin", unitID: "a1cae150"), "flight_3__a1cae150.bin")
    }

    func testANewFlightResolvesToTheQualifiedName() {
        XCTAssertEqual(cache.cachedName(for: "flight_7.bin", unitID: "a1cae150"), "flight_7__a1cae150.bin")
        XCTAssertEqual(cache.cachedName(for: "lora_001.bin", unitID: "a1cae150"), "lora_001.bin")
    }
}
