import XCTest
@testable import TinkerRocketApp

/// #1658: a downloaded flight records which rocket and computer it came from.
///
/// The summary gains a `source` block (filled at download) and
/// `settings.board` (from the log). Both are read back on their own, so a
/// sidecar written before either existed still lists with its headline
/// numbers, and an old flight's board can still be read from its log.
final class FlightSourceTests: XCTestCase {

    private var tempDir: URL!

    override func setUpWithError() throws {
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent("flight_source_\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: tempDir, withIntermediateDirectories: true)
    }

    override func tearDownWithError() throws {
        try? FileManager.default.removeItem(at: tempDir)
    }

    private func encode<T: Encodable>(_ value: T) throws -> String {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        return String(decoding: try encoder.encode(value), as: UTF8.self)
    }

    func testUnknownFieldsAreLeftOutNotNulled() throws {
        let json = try encode(FlightSource(unit_id: "a1cae150", unit_name: "Atlas",
                                           rocket_id: 2, oc_board: "V5"))
        XCTAssertTrue(json.contains("\"unit_id\" : \"a1cae150\""))
        XCTAssertTrue(json.contains("\"rocket_id\" : 2"))
        XCTAssertTrue(json.contains("\"oc_board\" : \"V5\""))
        XCTAssertFalse(json.contains("null"), "an unknown field must be absent, not null")
        XCTAssertFalse(json.contains("rocket_profile"))
        XCTAssertFalse(json.contains("fc_fw"))
    }

    func testSourceSortsAfterSettingsAndRoundTrips() throws {
        var summary = FlightSummary(max_altitude_m: 912, max_speed_mps: 173,
                                    burnout_time_s: 1.3, apogee_time_s: 8.9, settings: nil)
        XCTAssertFalse(try encode(summary).contains("source"),
                       "a summary with no source writes no key")
        summary.source = FlightSource(unit_id: "a1cae150", rocket_profile: "Piercer Mini",
                                      fc_fw: "41b2ab92", downloaded_at: "2026-10-24T18:03:00Z")
        let json = try encode(summary)
        let keys = ["apogee_time_s", "burnout_time_s", "max_altitude_m", "max_speed_mps", "source"]
            .compactMap { json.range(of: "\"\($0)\"")?.lowerBound }
        XCTAssertEqual(keys, keys.sorted(), "sorted keys, as Android writes them")
        let back = try JSONDecoder().decode(FlightSummary.self, from: Data(json.utf8))
        XCTAssertEqual(back.source, summary.source)
    }

    /// The cached-flight list reads the source and the board beside the headline.
    func testCachedFlightCarriesSourceAndBoard() throws {
        let cache = FileCache(documentsURL: tempDir)
        let csv = tempDir.appendingPathComponent("CSVCache/flight_20261024_161431.csv")
        try "t\n0\n".write(to: csv, atomically: true, encoding: .utf8)
        let summary = """
        {
          "max_altitude_m" : 912.4,
          "settings" : { "board" : "M1", "fw_git_sha" : "41b2ab92" },
          "source" : { "unit_id" : "9e8a1b2c", "unit_name" : "Beetle", "rocket_profile" : "RIM66",
                       "oc_board" : "M1", "fc_gnss" : "LC86G" }
        }
        """
        try summary.write(to: tempDir.appendingPathComponent("CSVCache/flight_20261024_161431.json"),
                          atomically: true, encoding: .utf8)

        let flight = try XCTUnwrap(cache.listCachedFlights().first)
        XCTAssertEqual(flight.maxAltitudeM, 912.4)
        XCTAssertEqual(flight.source?.unit_name, "Beetle")
        XCTAssertEqual(flight.source?.fc_gnss, "LC86G")
        XCTAssertEqual(flight.fcBoard, "M1")
        XCTAssertEqual(flight.rocketLabel, "RIM66")
    }

    /// A sidecar from before #1658 (no source, no board) still lists, with
    /// its numbers; the list falls back to the unit name only when there is one.
    func testOldSidecarListsWithoutSource() throws {
        let cache = FileCache(documentsURL: tempDir)
        try "t\n0\n".write(to: tempDir.appendingPathComponent("CSVCache/flight_20260705_174532.csv"),
                           atomically: true, encoding: .utf8)
        try #"{ "max_altitude_m" : 311.0, "settings" : { "fw_git_sha" : "abc" } }"#
            .write(to: tempDir.appendingPathComponent("CSVCache/flight_20260705_174532.json"),
                   atomically: true, encoding: .utf8)

        let flight = try XCTUnwrap(cache.listCachedFlights().first)
        XCTAssertEqual(flight.maxAltitudeM, 311.0)
        XCTAssertNil(flight.source)
        XCTAssertNil(flight.fcBoard)
        XCTAssertNil(flight.rocketLabel)
    }

    /// The board an old summary lacks comes from the log's first settings frame.
    func testBoardFromLogReadsTheSettingsFrame() throws {
        let bin = WireFixtures.url("csv/tiny_flight.bin")
        XCTAssertEqual(FileCache.boardFromLog(bin), "V9")

        let frame = try XCTUnwrap(MessageParser().firstFrame(
            ofType: MessageType.flightSettings.rawValue, in: bin))
        XCTAssertEqual(frame.type, MessageType.flightSettings.rawValue)
        // The early stop returns the same frame the full parse finds first.
        let all = try MessageParser().parseLogFile(bin)
        XCTAssertEqual(all.first { $0.type == frame.type }?.payload, frame.payload)
    }

    func testBoardFromLogIsNilWithoutALog() {
        XCTAssertNil(FileCache.boardFromLog(tempDir.appendingPathComponent("missing.bin")))
    }
}
