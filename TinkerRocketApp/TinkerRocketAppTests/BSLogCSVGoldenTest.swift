import XCTest
@testable import TinkerRocketApp

/// Swift base-station bin→CSV against the committed golden.
///
/// The golden's INPUT (tests_cpp/fixtures/wire/csv/bs_tiny.bin) is packed by
/// wire_fixture_gen using the REAL firmware LoRa packers, so this is not three
/// ports agreeing with each other — it is three ports agreeing with the
/// firmware. A round-trip test would happily pass while every implementation
/// shared the same wrong idea of the layout.
///
/// bs_tiny.bin is a LoRa protocol v6 log (23-byte SLOW frames, status 1) and
/// bs_tiny_v5.bin the same log as a v5 rocket sent it (22-byte SLOW frames, no
/// status byte). Both render the ONE golden: the status byte adds no column.
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

    private func assertMatchesGolden(_ rel: String) throws {
        let csv = try BSLogCSVGenerator.writeCSV(try bytes(rel))
        let want = try golden()
        if csv != want {
            let a = csv.components(separatedBy: "\n")
            let b = want.components(separatedBy: "\n")
            for i in 0..<max(a.count, b.count) {
                let al = i < a.count ? a[i] : "<missing>"
                let bl = i < b.count ? b[i] : "<missing>"
                if al != bl {
                    XCTFail("\(rel): CSV differs at line \(i + 1):\n generated: \(al)\n golden:    \(bl)")
                    return
                }
            }
        }
        XCTAssertEqual(csv, want, rel)
    }

    /// The LoRa frame of every BS_LORA_RX_MSG record, in order, as (offset in
    /// `b`, length). Record layout: preamble(4) type(1) len(1) payload(len)
    /// trailer(2); the frame starts 12 bytes into the payload.
    private func loraFrames(_ b: [UInt8]) -> [(offset: Int, len: Int)] {
        var out: [(offset: Int, len: Int)] = []
        var i = 0
        while i + 6 <= b.count {
            guard b[i] == 0xAA, b[i + 1] == 0x55, b[i + 2] == 0xAA, b[i + 3] == 0x55 else {
                i += 1
                continue
            }
            let type = b[i + 4], len = Int(b[i + 5])
            if type == 0xFC, len > 12 { out.append((i + 6 + 12, len - 12)) }
            i += 6 + len + 2
        }
        return out
    }

    private let loRaFrameSlow = 0x1   // LORA_FRAME_SLOW, the ver_type low nibble

    func testBaseStationCsvMatchesGoldenByteForByte() throws {
        try assertMatchesGolden("csv/bs_tiny.bin")
    }

    func testV5RocketLogRendersTheSameGolden() throws {
        try assertMatchesGolden("csv/bs_tiny_v5.bin")
    }

    func testFixturesAreTheProtocolVersionsTheirSidecarsClaim() throws {
        for rel in ["csv/bs_tiny.bin", "csv/bs_tiny_v5.bin"] {
            let side = WireFixtures.sidecar(rel)
            let bin = try bytes(rel)
            let frames = loraFrames(bin)
            XCTAssertEqual(frames.count, side.int("rx_records"), rel)
            for (off, len) in frames {
                let verType = Int(bin[off + 5])
                XCTAssertEqual(verType >> 4, side.int("lora_proto"), "\(rel): version nibble")
                let want = (verType & 0x0F) == loRaFrameSlow ? side.int("slow_frame_len") : 55
                XCTAssertEqual(len, want, "\(rel): frame length")
            }
        }
    }

    func testV6StatusByteDecodesIntoTheRowAndV5ReadsAsNoBoard() throws {
        for rel in ["csv/bs_tiny.bin", "csv/bs_tiny_v5.bin"] {
            let side = WireFixtures.sidecar(rel)
            let (rows, _) = try BSLogCSVGenerator.parse(try bytes(rel))
            XCTAssertEqual(rows.filter { $0.frame == "fast" }.count, side.int("fast_frames"), rel)
            XCTAssertEqual(rows.filter { $0.frame == "slow" }.count, side.int("slow_frames"), rel)
            guard let firstSlow = rows.firstIndex(where: { $0.frame == "slow" }), firstSlow > 0 else {
                return XCTFail("\(rel): a fast frame must lead the slow one")
            }
            XCTAssertTrue(rows[..<firstSlow].allSatisfy { $0.gnssImu == 0 && $0.mount == 0 },
                          "\(rel): nothing before the first slow frame")
            // Set by the slow frame and forward-filled onto the fast rows after it.
            let want = side.int("gnss_imu")
            XCTAssertTrue(rows[firstSlow...].allSatisfy { $0.gnssImu == want },
                          "\(rel): gnss_imu \(want) from the slow frame on")
            // #1755: bits 2-3 of the same byte, each read apart from the other.
            let wantMount = side.int("mount")
            XCTAssertTrue(rows[firstSlow...].allSatisfy { $0.mount == wantMount },
                          "\(rel): mount \(wantMount) from the slow frame on")
        }
    }

    func testSlowFrameAtTheOtherVersionsLengthIsSkipped() throws {
        // v6 header on a 22-byte slow frame, and v5 header on a 23-byte one:
        // rewrite the version nibble of each slow frame in the other fixture.
        for (rel, wrongVersion) in [("csv/bs_tiny_v5.bin", 6), ("csv/bs_tiny.bin", 5)] {
            var bin = try bytes(rel)
            var rewritten = 0
            for (off, _) in loraFrames(bin) where (Int(bin[off + 5]) & 0x0F) == loRaFrameSlow {
                bin[off + 5] = UInt8((wrongVersion << 4) | (Int(bin[off + 5]) & 0x0F))
                rewritten += 1
            }
            XCTAssertEqual(rewritten, 2, "\(rel): the fixture's shape changed; recheck the rewrite")

            let (rows, events) = try BSLogCSVGenerator.parse(bin)
            XCTAssertEqual(rows.count, 10, "\(rel): only the ten fast frames survive")
            XCTAssertTrue(rows.allSatisfy { $0.frame == "fast" }, rel)
            // Nothing from a skipped frame leaks into the forward-fill.
            XCTAssertTrue(rows.allSatisfy { $0.camA == 0 && $0.maxAlt == 0 && $0.gnssImu == 0 && $0.mount == 0 }, rel)
            XCTAssertEqual(events.count, 1, rel)
            // The CSV still renders, header plus 10 rows plus the event.
            let csv = try BSLogCSVGenerator.writeCSV(bin)
            XCTAssertEqual(csv.trimmingCharacters(in: .newlines).components(separatedBy: "\n").count,
                           12, rel)
        }
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
