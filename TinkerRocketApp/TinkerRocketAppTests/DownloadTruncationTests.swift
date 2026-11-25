import XCTest
@testable import TinkerRocketApp

/// #832: a truncated download was saved, cached and reported complete.
///
/// The chunk header is `[offset u32 LE][len u16 LE][flags u8][data]`, flags
/// bit0 = EOF, bit1 = ABORT. #526 closed the explicit EOF|ABORT case; this is
/// the same truncation re-entering through a bare EOF — a notification dropped
/// AFTER the peripheral queued it, which is precisely the case the firmware's
/// redundant EOF exists to compensate for and which it cannot itself detect.
///
/// Nothing covered `handleFileChunk` or `completeDownload` before this file.
final class DownloadTruncationTests: XCTestCase {

    private func chunk(offset: UInt32, payload: [UInt8],
                       eof: Bool = false, abort: Bool = false) -> Data {
        var d = Data()
        d.append(contentsOf: withUnsafeBytes(of: offset.littleEndian, Array.init))
        d.append(contentsOf: withUnsafeBytes(of: UInt16(payload.count).littleEndian, Array.init))
        d.append((eof ? 0x01 : 0x00) | (abort ? 0x02 : 0x00))
        d.append(contentsOf: payload)
        return d
    }

    /// Drive a download through the real notification entry point.
    private func run(_ frames: [Data], expectedSize: UInt32) -> URL? {
        runDevice(frames, expectedSize: expectedSize).url
    }

    /// The same, handing back the device too, with its main-queue updates
    /// (downloadWarning) applied. `stall` ends the transfer the way the 3 s
    /// stall timer does instead of waiting for an EOF.
    private func runDevice(_ frames: [Data], expectedSize: UInt32,
                           stall: Bool = false) -> (url: URL?, device: BLEDevice) {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        d.files = [FileInfo(name: "flight.bin", size: expectedSize)]

        var result: URL?
        var called = false
        d.beginDownloadForTesting(filename: "flight.bin") { url in
            result = url; called = true
        }
        for f in frames { d.handleFileChunkForTesting(f) }
        if stall { d.completeDownloadFromStallForTesting() }
        XCTAssertTrue(called, "the completion handler must always be invoked")
        RunLoop.main.run(until: Date().addingTimeInterval(0.05))
        return (result, d)
    }

    override func tearDown() {
        UserDefaults.standard.removeObject(forKey: "FileCache.shortDownloads")
        super.tearDown()
    }

    /// Real hardware ends a transfer with DATA and the EOF flag in the SAME
    /// frame — measured on the bench 2026-08-22, the base station's last frame
    /// for an 18322-byte file is `offset=18190 len=132 eof=true`. The first
    /// cut of the EOF byte-count check compared `offset` alone and would have
    /// failed every one of those; every test here used an empty EOF payload,
    /// so none of them caught it.
    func testDataBearingEofSucceeds() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: Array(repeating: 0xBB, count: 32), eof: true),
        ], expectedSize: 132)
        XCTAssertNotNil(url, "an EOF frame carrying data must not be rejected")
    }

    /// The same shape, short against the listing, with the device's own EOF
    /// confirming it sent exactly what arrived. Out-computer firmware before
    /// the byte-exact stream did this whenever a file held bytes outside a
    /// whole log frame (the V5 nose cone, 2026-10-24: 6,901,811 listed,
    /// 6,901,781 sent), and rejecting it threw the whole flight away. Kept,
    /// with the shortfall said out loud.
    func testDataBearingEofShortAgainstListingIsKeptWithAWarning() {
        let r = runDevice([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: Array(repeating: 0xBB, count: 32), eof: true),
        ], expectedSize: 500)
        XCTAssertNotNil(r.url, "the device ended the transfer itself: keep its 132 bytes")
        if let url = r.url { XCTAssertEqual((try? Data(contentsOf: url))?.count, 132) }
        let warning = r.device.downloadWarning ?? ""
        XCTAssertTrue(warning.contains("132") && warning.contains("500"),
                      "the shortfall must be reported, not hidden: \(warning)")
        let rec = FileCache.shared.shortDownloadRecord(for: "flight.bin")
        XCTAssertEqual(rec?.listed, 500)
        XCTAssertEqual(rec?.received, 132)
    }

    /// No EOF: the stall timer ended it. We cannot know what the device meant
    /// to send, so short against the listing still fails.
    func testStallShortAgainstListingStillFails() {
        let r = runDevice([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
        ], expectedSize: 500, stall: true)
        XCTAssertNil(r.url, "a transfer that just went quiet is not complete")
        XCTAssertNil(r.device.downloadWarning)
        XCTAssertNil(FileCache.shared.shortDownloadRecord(for: "flight.bin"))
    }

    /// A download that matches its listing leaves no warning behind.
    func testFullDownloadHasNoWarning() {
        let r = runDevice([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: [], eof: true),
        ], expectedSize: 100)
        XCTAssertNotNil(r.url)
        XCTAssertNil(r.device.downloadWarning)
    }

    /// The row's "downloaded" test: the listed size, or exactly the short
    /// download the board confirmed for this same listing — never a file of
    /// any other size, and never a record left by a different listing (a new
    /// flight reusing the name).
    func testCachedSizeRuleHonoursOnlyAMatchingShortRecord() {
        XCTAssertTrue(FileCache.cachedSizeSatisfiesListing(cachedSize: 500, listedSize: 500, shortRecord: nil))
        XCTAssertFalse(FileCache.cachedSizeSatisfiesListing(cachedSize: 132, listedSize: 500, shortRecord: nil))
        XCTAssertTrue(FileCache.cachedSizeSatisfiesListing(cachedSize: 132, listedSize: 500,
                                                           shortRecord: (listed: 500, received: 132)))
        XCTAssertFalse(FileCache.cachedSizeSatisfiesListing(cachedSize: 131, listedSize: 500,
                                                            shortRecord: (listed: 500, received: 132)))
        XCTAssertFalse(FileCache.cachedSizeSatisfiesListing(cachedSize: 132, listedSize: 600,
                                                            shortRecord: (listed: 500, received: 132)))
    }

    /// A clean transfer still succeeds — the regression guard for the fix.
    func testCompleteTransferSucceeds() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: Array(repeating: 0xBB, count: 100)),
            chunk(offset: 200, payload: [], eof: true),
        ], expectedSize: 200)
        XCTAssertNotNil(url, "a contiguous, correctly-sized transfer must succeed")
    }

    /// The #832 defect: a chunk is dropped, EOF arrives with no abort bit.
    func testDroppedChunkIsDetectedByTheGap() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            // offset 100 never arrives
            chunk(offset: 200, payload: Array(repeating: 0xCC, count: 100)),
            chunk(offset: 300, payload: [], eof: true),
        ], expectedSize: 300)
        XCTAssertNil(url, "a gap in the offsets must fail the download, not splice it")
    }

    /// EOF carries the device's own bytes_sent. When it equals what arrived
    /// and every chunk was contiguous, a shortfall against the listing is the
    /// device's own (see testDataBearingEofShortAgainstListingIsKeptWithAWarning):
    /// kept. A mismatch between EOF and arrivals still fails, below.
    func testEofConfirmedShortfallIsKept() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: [], eof: true),   // device claims 100...
        ], expectedSize: 300)                              // ...listing says 300
        XCTAssertNotNil(url, "the device's EOF agrees with what arrived: keep it")
    }

    func testEofClaimingMoreThanArrivedFails() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 999, payload: [], eof: true),   // device sent 999, we have 100
        ], expectedSize: 999)
        XCTAssertNil(url, "EOF byte count must be checked against what arrived")
    }

    /// A frame shorter than its own length header used to be dropped in
    /// silence, leaving a hole indistinguishable from a clean transfer.
    func testTruncatedFrameFails() {
        var bad = chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100))
        bad = bad.prefix(50)                               // cut mid-payload
        let url = run([Data(bad), chunk(offset: 100, payload: [], eof: true)],
                      expectedSize: 100)
        XCTAssertNil(url, "a frame shorter than its length header must fail")
    }

    /// #526 must keep working.
    func testExplicitAbortStillFails() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: [], eof: true, abort: true),
        ], expectedSize: 300)
        XCTAssertNil(url)
    }
}

// MARK: - #1077 summary decode fallbacks

/// Sidecars written between 2026-05-22 and 07-27 predate `dynamic_rate` and
/// `profile_semantics`. The synthesized decoder threw keyNotFound on either,
/// and FileCache decoded the whole summary in one `try?` — so every one of
/// those flights showed no altitude, speed or apogee. The settings structs now
/// fall back to the documented pre-field values.
final class SummaryDecodeFallbackTests: XCTestCase {

    private func decode<T: Decodable>(_ type: T.Type, _ json: String) throws -> T {
        try JSONDecoder().decode(type, from: Data(json.utf8))
    }

    func testIMUSettingsWithoutDynamicRateDecodesAsFixedRate() throws {
        let s = try decode(IMUSettings.self, """
            {"gyro_fs_dps": 2000, "low_g_fs_g": 16, "high_g_fs_g": 400, "update_rate_hz": 960}
            """)
        XCTAssertFalse(s.dynamic_rate, "absent means the then-fixed build rate")
        XCTAssertEqual(s.update_rate_hz, 960)
        XCTAssertNil(s.mounting)
    }

    func testIMUSettingsWithDynamicRateIsHonoured() throws {
        let s = try decode(IMUSettings.self, """
            {"gyro_fs_dps": 2000, "low_g_fs_g": 16, "high_g_fs_g": 400, "dynamic_rate": true}
            """)
        XCTAssertTrue(s.dynamic_rate)
    }

    private let rollWithoutSemantics = """
        {"mode": "angle", "kp": 0.04, "ki": 0.0, "kd": 0.001, "d_lpf_hz": 20.0,
         "kp_angle": 2.0, "cmd_limit_min_deg": -10.0, "cmd_limit_max_deg": 10.0,
         "delay_ms": 500, "rate_cap_dps": 180.0, "roll_rate_set_point": 0.0,
         "guidance_enabled": false,
         "gain_schedule": {"enabled": false, "v_ref": 100.0, "v_min": 20.0, "scale_cap": 4.0},
         "profile": []}
        """

    func testRollControlSettingsWithoutProfileSemanticsDecodesAsStep() throws {
        let s = try decode(RollControlSettings.self, rollWithoutSemantics)
        XCTAssertEqual(s.profile_semantics, "step", "absent means the documented pre-v4 semantics")
        XCTAssertNil(s.min_speed_mps, "pre-speed-gate sidecars have no min_speed_mps")
        XCTAssertEqual(s.mode, "angle")
    }

    func testRollControlSettingsWithProfileSemanticsIsHonoured() throws {
        let json = rollWithoutSemantics.replacingOccurrences(of: "\"profile\": []", with: "\"profile\": [], \"profile_semantics\": \"ramp\"")
        let s = try decode(RollControlSettings.self, json)
        XCTAssertEqual(s.profile_semantics, "ramp")
    }
}
