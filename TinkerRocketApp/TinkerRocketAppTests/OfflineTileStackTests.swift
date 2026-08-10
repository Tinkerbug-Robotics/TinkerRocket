//
//  OfflineTileStackTests.swift
//  TinkerRocketAppTests
//
//  The offline tile stack had no tests on this side at all, while Android
//  pinned the same behaviour in five classes — including the one contract the
//  two apps MUST agree on byte-for-byte: the ArcGIS URL is ordered z/y/x and
//  the disk path is ordered z/x/y. A cache written by one app has to be
//  readable by the other, and a silent transposition would only surface as a
//  map that looks subtly wrong at a field with no signal.
//
//  Mirrors TinkerRocketAndroid/core/maps/src/test/.../TileMathAndCacheTest.kt
//  and DownloaderAndRegionTest.kt, case for case where the platforms share
//  behaviour.
//

import XCTest
import CoreLocation
@testable import TinkerRocketApp

private func tempDir() -> URL {
    let url = FileManager.default.temporaryDirectory
        .appendingPathComponent("tiletest-\(UUID().uuidString)", isDirectory: true)
    try? FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    return url
}

// MARK: - Tile math

final class TileMathTests: XCTestCase {

    func testSlippyCornersAtZoom4() {
        XCTAssertEqual(TileMath.tileX(lon: -179.9, z: 4), 0)
        XCTAssertEqual(TileMath.tileX(lon: 179.9, z: 4), 15)
        XCTAssertEqual(TileMath.tileY(lat: 85.0, z: 4), 0)
        XCTAssertEqual(TileMath.tileY(lat: -85.0, z: 4), 15)
    }

    /// Same region and golden count as the Android test, so the two tile
    /// enumerations cannot drift apart.
    func testRegionTilesMatchTheAndroidGolden() {
        let region = RegionSpec(
            center: CLLocationCoordinate2D(latitude: 39.97154, longitude: -74.93655),
            radiusMeters: 800, minZoom: 10, maxZoom: 14)

        XCTAssertEqual(TileMath.tileCount(for: region), 8)
        let tiles = TileMath.tiles(for: region)
        XCTAssertEqual(tiles.count, TileMath.tileCount(for: region))
        XCTAssertEqual(Set(tiles).count, tiles.count, "no duplicate tiles")

        for z in 10...14 {
            let atZ = tiles.filter { $0.z == z }
            XCTAssertFalse(atZ.isEmpty, "zoom \(z) missing")
            let cx = TileMath.tileX(lon: -74.93655, z: z)
            let cy = TileMath.tileY(lat: 39.97154, z: z)
            XCTAssertTrue(atZ.contains { $0.x == cx && $0.y == cy },
                          "center tile missing at z=\(z)")
        }
    }
}

// MARK: - Sources

final class TileSourceTests: XCTestCase {

    /// The save-area slider clamps to this. USGS ArcGIS stops at 16: z17/z18
    /// 404 every tile, and the downloader counts a failure — so an over-deep
    /// range would pad the estimate with tiles that can never arrive.
    func testCacheableSourcesDeclareTheDepthTheyServe() {
        for source in TileSource.allCases where source.isCacheable {
            XCTAssertEqual(source.maxZoom, 16, "\(source.rawValue) max zoom")
        }
    }

    /// THE cross-platform contract: the upstream URL is z/y/x. The disk path
    /// is z/x/y (asserted in OfflineTileCacheTests). Swapping either half
    /// silently serves the wrong tile.
    func testUpstreamTemplatesAreOrderedZYX() {
        for source in TileSource.allCases where source.isCacheable {
            let template = try! XCTUnwrap(source.urlTemplate)
            let z = try! XCTUnwrap(template.range(of: "{z}")?.lowerBound)
            let y = try! XCTUnwrap(template.range(of: "{y}")?.lowerBound)
            let x = try! XCTUnwrap(template.range(of: "{x}")?.lowerBound)
            XCTAssertTrue(z < y && y < x,
                          "\(source.rawValue) must be z/y/x, got \(template)")
        }
    }

    func testOnlyUSGSIsCacheable() {
        XCTAssertFalse(TileSource.appleStandard.isCacheable)
        XCTAssertFalse(TileSource.appleHybrid.isCacheable)
        XCTAssertTrue(TileSource.usgsImagery.isCacheable)
        XCTAssertTrue(TileSource.usgsImageryTopo.isCacheable)
        XCTAssertTrue(TileSource.usgsTopo.isCacheable)
        // A non-cacheable source has no template, so it can never be
        // region-downloaded — the downloader early-returns on nil.
        XCTAssertNil(TileSource.appleHybrid.urlTemplate)
    }
}

// MARK: - Cache

final class OfflineTileCacheTests: XCTestCase {

    func testDiskLayoutIsSourceZXY() {
        let root = tempDir()
        let cache = OfflineTileCache(root: root)
        cache.store(Data([1, 2, 3]), source: "usgsImagery", z: 12, x: 1195, y: 1551)

        // The disk path transposes x and y relative to the ArcGIS URL. This
        // exact path is what the Android app reads and writes.
        let expected = root
            .appendingPathComponent("usgsImagery/12/1195/1551.tile")
        XCTAssertTrue(FileManager.default.fileExists(atPath: expected.path),
                      "expected \(expected.path)")
        XCTAssertEqual(cache.tileData(source: "usgsImagery", z: 12, x: 1195, y: 1551),
                       Data([1, 2, 3]))
    }

    func testMissingTileIsNil() {
        let cache = OfflineTileCache(root: tempDir())
        XCTAssertNil(cache.tileData(source: "usgsImagery", z: 1, x: 2, y: 3))
    }

    func testByteCountAndRemoveArePerSource() {
        let cache = OfflineTileCache(root: tempDir())
        cache.store(Data(repeating: 0, count: 100), source: "a", z: 1, x: 1, y: 1)
        cache.store(Data(repeating: 0, count: 50), source: "b", z: 1, x: 1, y: 1)
        XCTAssertEqual(cache.byteCount(source: "a"), 100)
        XCTAssertEqual(cache.byteCount(source: "b"), 50)

        cache.removeTiles(source: "a", tiles: [TileXYZ(z: 1, x: 1, y: 1)])
        XCTAssertEqual(cache.byteCount(source: "a"), 0)
        XCTAssertEqual(cache.byteCount(source: "b"), 50, "another source is untouched")
    }
}

// MARK: - Region store

final class OfflineRegionStoreTests: XCTestCase {

    private func region(name: String = "Pad A", source: String = "usgsImagery",
                        radius: Double = 800) -> OfflineRegion {
        OfflineRegion(name: name, lat: 39.97154, lon: -74.93655,
                      radiusMeters: radius, minZoom: 10, maxZoom: 14,
                      source: source, tileCount: 8, bytes: 1234,
                      savedAt: Date(timeIntervalSince1970: 1_753_600_000))
    }

    func testManifestRoundTrips() {
        let dir = tempDir()
        let cache = OfflineTileCache(root: tempDir())
        let saved = region()
        OfflineRegionStore(directory: dir, cache: cache).add(saved)

        // A second store over the same directory must see it — this is the
        // schema Android reads too.
        let reloaded = OfflineRegionStore(directory: dir, cache: cache)
        XCTAssertEqual(reloaded.regions.count, 1)
        XCTAssertEqual(reloaded.regions.first?.id, saved.id)
        XCTAssertEqual(reloaded.regions.first?.name, "Pad A")
        XCTAssertEqual(reloaded.regions.first?.maxZoom, 14)
        XCTAssertEqual(reloaded.regions.first?.bytes, 1234)
    }

    func testCorruptManifestStartsEmpty() {
        let dir = tempDir()
        try! "{not json[".write(to: dir.appendingPathComponent("offline_regions.json"),
                                atomically: true, encoding: .utf8)
        XCTAssertTrue(OfflineRegionStore(directory: dir,
                                         cache: OfflineTileCache(root: tempDir())).regions.isEmpty)
    }

    func testDeleteReclaimsTilesAndManifestEntry() {
        let cache = OfflineTileCache(root: tempDir())
        let store = OfflineRegionStore(directory: tempDir(), cache: cache)
        let r = region()
        for t in TileMath.tiles(for: r.spec) {
            cache.store(Data([1]), source: r.source, z: t.z, x: t.x, y: t.y)
        }
        store.add(r)

        store.delete(r)
        XCTAssertTrue(store.regions.isEmpty)
        XCTAssertEqual(cache.byteCount(source: r.source), 0, "tiles must be reclaimed")
    }

    /// Two areas over the same site share tile paths. Deleting one used to
    /// take the shared tiles with it, leaving the survivor quietly incomplete
    /// — invisible until you are standing at the field needing the map.
    func testDeleteKeepsTilesAnOverlappingRegionStillNeeds() {
        let cache = OfflineTileCache(root: tempDir())
        let store = OfflineRegionStore(directory: tempDir(), cache: cache)
        let big = region(name: "Wide", radius: 3000)
        let small = region(name: "Tight", radius: 800)
        let shared = Set(TileMath.tiles(for: small.spec))
        XCTAssertTrue(Set(TileMath.tiles(for: big.spec)).isSuperset(of: shared),
                      "test premise: the small region is inside the big one")
        for t in Set(TileMath.tiles(for: big.spec)).union(shared) {
            cache.store(Data([1]), source: big.source, z: t.z, x: t.x, y: t.y)
        }
        store.add(big)
        store.add(small)

        store.delete(big)

        for t in shared {
            XCTAssertNotNil(cache.tileData(source: small.source, z: t.z, x: t.x, y: t.y),
                            "deleting the wide area punched a hole in the tight one at \(t)")
        }
        let onlyBig = TileMath.tiles(for: big.spec).filter { !shared.contains($0) }
        XCTAssertFalse(onlyBig.isEmpty, "test premise: the big region is strictly larger")
        for t in onlyBig {
            XCTAssertNil(cache.tileData(source: big.source, z: t.z, x: t.x, y: t.y),
                         "tiles no surviving region needs must be reclaimed")
        }
    }

    /// Sources are separate directories, so an identical z/x/y under another
    /// source is a different file and must not pin anything.
    func testDeleteIgnoresOverlapFromADifferentSource() {
        let cache = OfflineTileCache(root: tempDir())
        let store = OfflineRegionStore(directory: tempDir(), cache: cache)
        let mine = region(source: "usgsImagery")
        for t in TileMath.tiles(for: mine.spec) {
            cache.store(Data([1]), source: mine.source, z: t.z, x: t.x, y: t.y)
        }
        store.add(mine)
        store.add(region(name: "Topo", source: "usgsTopo"))

        store.delete(mine)
        XCTAssertEqual(cache.byteCount(source: mine.source), 0,
                       "another source must not pin these tiles")
    }
}

// MARK: - Downloader

/// Serves a fixed body (or 404s) for every tile request, so the downloader can
/// be exercised without a network.
private final class StubTileProtocol: URLProtocol {
    nonisolated(unsafe) static var body = Data([0xFF, 0xD8, 1, 2, 3, 4])
    nonisolated(unsafe) static var respond404 = false
    nonisolated(unsafe) static var hits = 0
    nonisolated(unsafe) static let lock = NSLock()

    static func reset() {
        lock.lock(); hits = 0; respond404 = false; lock.unlock()
    }

    override class func canInit(with request: URLRequest) -> Bool { true }
    override class func canonicalRequest(for request: URLRequest) -> URLRequest { request }

    override func startLoading() {
        Self.lock.lock(); Self.hits += 1; let fail = Self.respond404; Self.lock.unlock()
        let response = HTTPURLResponse(url: request.url!,
                                       statusCode: fail ? 404 : 200,
                                       httpVersion: nil, headerFields: nil)!
        client?.urlProtocol(self, didReceive: response, cacheStoragePolicy: .notAllowed)
        if !fail { client?.urlProtocol(self, didLoad: Self.body) }
        client?.urlProtocolDidFinishLoading(self)
    }

    override func stopLoading() {}
}

final class TileDownloaderTests: XCTestCase {

    private let region = RegionSpec(
        center: CLLocationCoordinate2D(latitude: 39.97154, longitude: -74.93655),
        radiusMeters: 800, minZoom: 10, maxZoom: 14)

    private func makeDownloader(cache: OfflineTileCache) -> TileDownloader {
        let config = URLSessionConfiguration.ephemeral
        config.protocolClasses = [StubTileProtocol.self]
        return TileDownloader(cache: cache, session: URLSession(configuration: config))
    }

    private func awaitTerminal(_ d: TileDownloader) {
        let deadline = Date().addingTimeInterval(15)
        while d.phase == .downloading || d.phase == .idle {
            RunLoop.current.run(until: Date().addingTimeInterval(0.02))
            if Date() > deadline { return XCTFail("downloader never reached a terminal phase") }
        }
    }

    override func setUp() { super.setUp(); StubTileProtocol.reset() }

    func testDownloadsRegionAndPersistsEveryTile() {
        let cache = OfflineTileCache(root: tempDir())
        let d = makeDownloader(cache: cache)
        var savedTiles = -1
        d.start(region: region, source: .usgsImagery) { tiles, _ in savedTiles = tiles }
        awaitTerminal(d)

        XCTAssertEqual(d.phase, .finished)
        XCTAssertEqual(d.total, 8)
        XCTAssertEqual(d.done, 8)
        XCTAssertEqual(savedTiles, 8, "the downloader hands totals to the caller")
        for t in TileMath.tiles(for: region) {
            XCTAssertEqual(cache.tileData(source: "usgsImagery", z: t.z, x: t.x, y: t.y),
                           StubTileProtocol.body, "missing \(t)")
        }
    }

    func testCachedTilesAreNotRefetched() {
        let cache = OfflineTileCache(root: tempDir())
        for t in TileMath.tiles(for: region) {
            cache.store(StubTileProtocol.body, source: "usgsImagery", z: t.z, x: t.x, y: t.y)
        }
        let d = makeDownloader(cache: cache)
        d.start(region: region, source: .usgsImagery)
        awaitTerminal(d)

        XCTAssertEqual(d.phase, .finished)
        XCTAssertEqual(d.done, 8)
        StubTileProtocol.lock.lock()
        let hits = StubTileProtocol.hits
        StubTileProtocol.lock.unlock()
        XCTAssertEqual(hits, 0, "cached tiles must not re-fetch")
    }

    /// The no-signal case the feature exists for. It used to reach .finished
    /// with 0 bytes and save a 0 MB area that looked real in the list.
    func testEveryTileMissingIsFailedAndSavesNothing() {
        StubTileProtocol.respond404 = true
        let cache = OfflineTileCache(root: tempDir())
        let d = makeDownloader(cache: cache)
        var recorded = false
        d.start(region: region, source: .usgsImagery) { _, _ in recorded = true }
        awaitTerminal(d)

        XCTAssertEqual(d.phase, .failed)
        XCTAssertEqual(d.done, 8, "progress still completes")
        XCTAssertEqual(d.failed, 8)
        XCTAssertEqual(d.bytes, 0)
        XCTAssertFalse(recorded, "a run that fetched nothing must not save a region")
        XCTAssertEqual(cache.byteCount(source: "usgsImagery"), 0, "404s must not be cached")
    }

    /// A few edge tiles 404ing is normal and must not block the save.
    func testPartialFailureStillFinishesAndCountsFailures() {
        let cache = OfflineTileCache(root: tempDir())
        let first = TileMath.tiles(for: region)[0]
        cache.store(StubTileProtocol.body, source: "usgsImagery",
                    z: first.z, x: first.x, y: first.y)
        StubTileProtocol.respond404 = true
        let d = makeDownloader(cache: cache)
        var recorded = false
        d.start(region: region, source: .usgsImagery) { _, _ in recorded = true }
        awaitTerminal(d)

        XCTAssertEqual(d.phase, .finished)
        XCTAssertEqual(d.failed, 7, "every uncached tile failed")
        XCTAssertTrue(recorded, "the region is still recorded")
    }

    func testUnknownSourceIsIgnored() {
        let d = makeDownloader(cache: OfflineTileCache(root: tempDir()))
        d.start(region: region, source: .appleHybrid)   // no template
        XCTAssertEqual(d.phase, .idle)
    }
}
