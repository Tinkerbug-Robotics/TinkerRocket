import Compression
import XCTest
@testable import TinkerRocketApp

/// #1600 phase 5: the broadcast ionosphere a pad calibration sends — gzip as
/// NOAA serves it, today's file then yesterday's, and the copy the phone keeps
/// for a field with no signal. The network is a fake; the gzip is built here.
final class IonosphereStoreTests: XCTestCase {

    // 2026-10-13T12:00:00Z: GPS day 17082, the file of day-of-year 286.
    private let now = Date(timeIntervalSince1970: 1_791_892_800)
    private let todayURL = "https://geodesy.noaa.gov/corsdata/rinex/2026/286/brdc2860.26n.gz"
    private let yesterdayURL = "https://geodesy.noaa.gov/corsdata/rinex/2026/285/brdc2850.26n.gz"
    private let broadcast = [21, 0, -2, 0, 60, -1, -4, 3]

    // The header of NOAA's brdc2860.26n, as KlobucharTests has it.
    private let header = """
     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE
teqc  2019Feb25     CORS-ADM Account    20261013 10:45:02UTCPGM / RUN BY / DATE
    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA
    1.2288D+05 -1.6384D+04 -2.6214D+05  1.9661D+05          ION BETA
    0.000000000000D+00 3.552713678801D-15   405504     2440 DELTA-UTC: A0,A1,T,W
                                                            END OF HEADER
 1 26 10 13  0  0  0.0 3.508096560836D-04 1.136868377216D-12 0.000000000000D+00

"""

    private var headerWithoutIon: String {
        header.components(separatedBy: "\n")
            .filter { !$0.hasSuffix("ION ALPHA") && !$0.hasSuffix("ION BETA") }
            .joined(separator: "\n")
    }

    // MARK: - Building gzip here

    /// A gzip member: header with the optional fields asked for, the
    /// DEFLATE body, then CRC-32 and length. NOAA's carry FNAME.
    static func gzip(_ text: String, name: String? = "brdc2860.26n", extra: [UInt8]? = nil,
                     comment: String? = nil, headerCrc: Bool = false) -> Data {
        let raw = [UInt8](text.utf8)
        var flg: UInt8 = 0
        if headerCrc { flg |= 0x02 }
        if extra != nil { flg |= 0x04 }
        if name != nil { flg |= 0x08 }
        if comment != nil { flg |= 0x10 }
        var out: [UInt8] = [0x1F, 0x8B, 8, flg, 0, 0, 0, 0, 0, 3]
        if let extra {
            out += [UInt8(extra.count & 0xFF), UInt8(extra.count >> 8)] + extra
        }
        if let name { out += Array(name.utf8) + [0] }
        if let comment { out += Array(comment.utf8) + [0] }
        if headerCrc { out += [0xAB, 0xCD] }   // not checked by the reader
        out += deflate(raw)
        out += le32(crc32(raw)) + le32(UInt32(raw.count))
        return Data(out)
    }

    private static func deflate(_ bytes: [UInt8]) -> [UInt8] {
        var out = [UInt8](repeating: 0, count: bytes.count + 1024)
        let n = compression_encode_buffer(&out, out.count, bytes, bytes.count, nil, COMPRESSION_ZLIB)
        precondition(n > 0, "deflate failed")
        return Array(out[0..<n])
    }

    private static func crc32(_ bytes: [UInt8]) -> UInt32 {
        var c: UInt32 = 0xFFFF_FFFF
        for b in bytes {
            c ^= UInt32(b)
            for _ in 0..<8 { c = (c >> 1) ^ ((c & 1) != 0 ? 0xEDB8_8320 : 0) }
        }
        return ~c
    }

    private static func le32(_ v: UInt32) -> [UInt8] {
        (0..<4).map { UInt8((v >> (8 * $0)) & 0xFF) }
    }

    // MARK: - The fake network and the scratch cache

    /// Serves `files` by URL, after `delay`, and remembers what was asked.
    final class FakeNet: @unchecked Sendable {
        private let lock = NSLock()
        private var asked: [String] = []
        private let files: [String: Data]
        private let delay: TimeInterval

        init(_ files: [String: Data] = [:], delay: TimeInterval = 0) {
            self.files = files
            self.delay = delay
        }

        var requests: [String] { lock.withLock { asked } }

        var repository: IonosphereRepository {
            IonosphereRepository(fetch: { [self] url in
                lock.withLock { asked.append(url) }
                if delay > 0 {
                    try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
                    if Task.isCancelled { return nil }
                }
                return files[url]
            })
        }
    }

    private func scratchCache() -> (IonosphereCache, URL) {
        let dir = FileManager.default.temporaryDirectory
            .appendingPathComponent("iono-\(UUID().uuidString)")
        let file = dir.appendingPathComponent("broadcast.json")
        return (IonosphereCache(file: file), file)
    }

    @MainActor
    private func store(_ net: FakeNet, _ cache: IonosphereCache) -> IonosphereStore {
        let t = now
        return IonosphereStore(repository: net.repository, cache: cache, now: { t })
    }

    @MainActor
    private func waitUntil(_ timeout: TimeInterval = 5, _ done: () -> Bool) async throws {
        let deadline = Date().addingTimeInterval(timeout)
        while !done() {
            if Date() > deadline { return XCTFail("timed out") }
            try await Task.sleep(nanoseconds: 20_000_000)
        }
    }

    private func older() -> HeldIonosphere {
        HeldIonosphere(klobuchar: Klobuchar(raw: [1, 2, 3, 4, 5, 6, 7, 8], gpsDay: 17070)!,
                       fetchedAt: Date(timeIntervalSince1970: 1_790_900_000))
    }

    // MARK: - The policy

    func testTheTestClockIsTheDayTheUrlsName() {
        XCTAssertEqual(Klobuchar.gpsDay(now), 17082)
        XCTAssertEqual(Klobuchar.noaaUrl(now), todayURL)
        XCTAssertEqual(Klobuchar.noaaUrl(now.addingTimeInterval(-86_400)), yesterdayURL)
    }

    @MainActor
    func testTodaysFileIsReadKeptAndNotFetchedAgain() async {
        let (cache, _) = scratchCache()
        let net = FakeNet([todayURL: Self.gzip(header)])
        let s = store(net, cache)
        XCTAssertNil(s.held)
        XCTAssertFalse(s.isCurrent)

        let k = await s.setForPadCalibration()
        XCTAssertEqual(k?.raw, broadcast)
        XCTAssertEqual(k?.gpsDay, 17082)
        XCTAssertEqual(net.requests, [todayURL])
        XCTAssertEqual(s.held?.fetchedAt, now)
        XCTAssertTrue(s.isCurrent)

        // The next launch holds it without a network, and a current set goes
        // out without asking for it again.
        let offline = FakeNet()
        let next = store(offline, cache)
        XCTAssertEqual(next.held, s.held)
        let again = await next.setForPadCalibration()
        XCTAssertEqual(again, k)
        XCTAssertEqual(offline.requests, [])
        next.refreshIfStale()
        XCTAssertFalse(next.isRefreshing, "nothing to refresh")
    }

    @MainActor
    func testTodayMissingFallsBackToYesterdaysFileAndItsDay() async {
        let (cache, _) = scratchCache()
        let net = FakeNet([yesterdayURL: Self.gzip(header, name: "brdc2850.26n")])
        let s = store(net, cache)
        let k = await s.setForPadCalibration()
        XCTAssertEqual(k?.raw, broadcast)
        XCTAssertEqual(k?.gpsDay, 17081, "the file's own date, not today's")
        XCTAssertEqual(net.requests, [todayURL, yesterdayURL])
        XCTAssertFalse(s.isCurrent, "today's is still to be fetched")
    }

    @MainActor
    func testBothMissingSendsTheCachedSet() async {
        let (cache, _) = scratchCache()
        XCTAssertTrue(cache.store(older()))
        let net = FakeNet()
        let s = store(net, cache)
        XCTAssertEqual(s.held, older())
        let k = await s.setForPadCalibration()
        XCTAssertEqual(k, older().klobuchar)
        XCTAssertEqual(s.held, older(), "an old set beats none")
        XCTAssertEqual(net.requests, [todayURL, yesterdayURL])
    }

    @MainActor
    func testNoSetAnywhereSendsNone() async {
        let (cache, _) = scratchCache()
        let k = await store(FakeNet(), cache).setForPadCalibration()
        XCTAssertNil(k)
    }

    @MainActor
    func testAHeaderWithoutTheIonLinesStoresNothing() async {
        let (cache, file) = scratchCache()
        let net = FakeNet([todayURL: Self.gzip(headerWithoutIon),
                           yesterdayURL: Self.gzip(headerWithoutIon)])
        let s = store(net, cache)
        let k = await s.setForPadCalibration()
        XCTAssertNil(k)
        XCTAssertNil(s.held)
        XCTAssertEqual(net.requests, [todayURL, yesterdayURL])
        XCTAssertFalse(FileManager.default.fileExists(atPath: file.path), "nothing written")
    }

    @MainActor
    func testASlowNetworkIsWaitedOnFiveSecondsAtMostAndTheCacheSent() async throws {
        let (cache, _) = scratchCache()
        cache.store(older())
        let net = FakeNet([todayURL: Self.gzip(header)], delay: 1.0)
        let s = store(net, cache)
        let started = Date()
        let k = await s.setForPadCalibration(timeout: 0.2)
        XCTAssertEqual(k, older().klobuchar)
        XCTAssertLessThan(Date().timeIntervalSince(started), 0.9, "the calibration did not wait it out")
        XCTAssertEqual(IonosphereStore.calibrationRefreshTimeout, 5)
        // Not cut off: the fetch lands for the next calibration.
        try await waitUntil { s.held?.klobuchar.gpsDay == 17082 }
        XCTAssertEqual(net.requests, [todayURL])
    }

    @MainActor
    func testACalibrationWaitsOnTheRefreshAlreadyRunning() async {
        let (cache, _) = scratchCache()
        let net = FakeNet([todayURL: Self.gzip(header)], delay: 0.2)
        let s = store(net, cache)
        s.refreshIfStale()   // the app's start
        let k = await s.setForPadCalibration(timeout: 3)
        XCTAssertEqual(k?.gpsDay, 17082)
        XCTAssertEqual(net.requests, [todayURL], "one fetch, not two")
    }

    @MainActor
    func testTheBackgroundRefreshReturnsAtOnceAndLandsLater() async throws {
        let (cache, _) = scratchCache()
        let net = FakeNet([todayURL: Self.gzip(header)], delay: 0.1)
        let s = store(net, cache)
        s.refreshIfStale()
        s.refreshIfStale()   // one in flight is enough
        XCTAssertNil(s.held, "nothing waited for it")
        XCTAssertTrue(s.isRefreshing)
        try await waitUntil { s.held != nil }
        XCTAssertEqual(s.held?.klobuchar.gpsDay, 17082)
        try await waitUntil { !s.isRefreshing }
        XCTAssertEqual(net.requests, [todayURL])
        XCTAssertEqual(cache.load(), s.held, "and it is kept")
    }

    @MainActor
    func testAnOlderFileNeverReplacesANewerOne() async {
        let (cache, _) = scratchCache()
        let today = HeldIonosphere(klobuchar: Klobuchar(raw: broadcast, gpsDay: 17082)!,
                                   fetchedAt: now)
        cache.store(today)
        // Only yesterday's file answers this refresh.
        let s = store(FakeNet([yesterdayURL: Self.gzip(header)]), cache)
        await s.refresh()
        XCTAssertEqual(s.held, today)
        XCTAssertEqual(cache.load(), today)
    }

    func testTheCacheFileIsAndroidsAndADamagedOneReadsAsNone() throws {
        let (cache, file) = scratchCache()
        XCTAssertNil(cache.load(), "no file")
        XCTAssertTrue(cache.store(older()))
        XCTAssertEqual(cache.load(), older())
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: Data(contentsOf: file))
                                     as? [String: Any])
        XCTAssertEqual(Set(json.keys), ["raw", "gps_day", "fetched_at_ms"])
        XCTAssertEqual(json.int("gps_day"), 17070)
        XCTAssertEqual(json.i64("fetched_at_ms"), 1_790_900_000_000)

        try Data("{\"raw\":[1,2,3],\"gps_day\":17082,\"fetched_at_ms\":0}".utf8).write(to: file)
        XCTAssertNil(cache.load(), "five bytes short is not a set")
        try Data("not json".utf8).write(to: file)
        XCTAssertNil(cache.load())
    }

    // MARK: - The file itself

    func testTheParserTakesNoaasGzipOrPlainText() {
        XCTAssertEqual(IonosphereRepository.parse(Self.gzip(header), gpsDay: 17082)?.raw, broadcast)
        XCTAssertEqual(IonosphereRepository.parse(Data(header.utf8), gpsDay: 17082)?.raw, broadcast)
        XCTAssertNil(IonosphereRepository.parse(Data("<html>Not Found</html>".utf8), gpsDay: 17082))
    }

    func testGzipStepsOverEveryOptionalHeaderField() {
        let text = "1 26 10 13 the body\n"
        XCTAssertEqual(Gzip.decompress(Self.gzip(text)), Data(text.utf8))
        XCTAssertEqual(Gzip.decompress(Self.gzip(text, name: nil)), Data(text.utf8))
        XCTAssertEqual(Gzip.decompress(Self.gzip(text, name: "a.n", extra: [1, 2, 3],
                                                 comment: "made here", headerCrc: true)),
                       Data(text.utf8))
    }

    func testGzipRefusesWhatItCannotRead() {
        let good = [UInt8](Self.gzip(header))
        XCTAssertNil(Gzip.decompress(Data(header.utf8)), "not gzip")
        var reserved = good
        reserved[3] |= 0x20
        XCTAssertNil(Gzip.decompress(Data(reserved)), "a reserved flag")
        var method = good
        method[2] = 7
        XCTAssertNil(Gzip.decompress(Data(method)), "not deflate")
        XCTAssertNil(Gzip.decompress(Data(good.prefix(16))), "cut short in the name")
        XCTAssertNil(Gzip.decompress(Data()))
    }

    func testGzipStopsAtItsCap() {
        let big = String(repeating: "0.000000000000D+00 ", count: 1_000)   // 19 000 bytes
        XCTAssertEqual(Gzip.decompress(Self.gzip(big), maxBytes: 1_000), Data(big.utf8.prefix(1_000)))
        XCTAssertEqual(Gzip.decompress(Self.gzip(big), maxBytes: 19_000), Data(big.utf8))
        XCTAssertEqual(Gzip.defaultMaxBytes, 4 << 20)
    }

    func testAFileCutShortStillGivesItsHeader() {
        // A day's file: the header, then its ephemerides.
        let body = (0..<3_000).map { " \($0 % 32 + 1) 26 10 13  0  0  0.0 3.508096560836D-04\n" }.joined()
        let whole = [UInt8](Self.gzip(header + body))
        let cut = Data(whole.prefix(whole.count * 6 / 10))
        let inflated = Gzip.decompress(cut)
        XCTAssertNotNil(inflated)
        XCTAssertLessThan(inflated?.count ?? 0, (header + body).utf8.count)
        XCTAssertEqual(inflated, Data((header + body).utf8.prefix(inflated?.count ?? 0)),
                       "what came out is the file's start")
        XCTAssertEqual(IonosphereRepository.parse(cut, gpsDay: 17082)?.raw, broadcast)
    }
}
