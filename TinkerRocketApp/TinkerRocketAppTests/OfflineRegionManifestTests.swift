import XCTest
import CoreLocation
@testable import TinkerRocketApp

/// #836 item 1 — a saved offline region recorded the radius, max zoom and
/// source that were set when the download FINISHED, not the ones that were
/// downloaded.
///
/// `SaveAreaView` hoisted only `regionName` and `center` into locals before
/// the escaping completion closure; `radiusKm * 1000`, `effectiveMaxZoom` and
/// `source` were read from `@State` *inside* it, minutes later. Meanwhile
/// `downloader.start(region: spec, source: source)` had snapshotted the real
/// geometry by value at tap time. Nothing disabled the source picker or the
/// sliders while the download ran — only the toolbar Cancel button — so the
/// user could freely move all three.
///
/// Set USGS Imagery+Topo / 5 km, tap Download, then drag Radius to 20 km and
/// switch to USGS Topo. The manifest records a 20 km `usgsTopo` region holding
/// the tile count and bytes of a 5 km `usgsImageryTopo` download:
///
///   - at the field with no signal it claims 20 km of coverage and draws the
///     hatched "not downloaded" placeholder everywhere past 5 km;
///   - `OfflineRegionStore.delete` calls `cache.removeTiles(source:)` with the
///     recorded source, so the wrong source deletes NOTHING while the storage
///     total drops by `bytes` — tiles left on disk that the total cannot see
///     and Delete can never reclaim, which is exactly the failure
///     TileDownloader says was fixed.
final class OfflineRegionManifestTests: XCTestCase {

    private let pad = CLLocationCoordinate2D(latitude: 40.1, longitude: -105.2)

    /// What was actually fetched.
    private var downloadedSpec: RegionSpec {
        RegionSpec(center: pad, radiusMeters: 5_000, minZoom: 10, maxZoom: 15)
    }

    /// THE regression: the manifest describes the download, and there is no
    /// second source of truth for it to drift from.
    func testManifestDescribesTheSpecThatWasDownloaded() {
        let saved = OfflineRegion(name: "Pad",
                                  spec: downloadedSpec,
                                  source: .usgsImageryTopo,
                                  tileCount: 2_000,
                                  bytes: 48_000_000,
                                  savedAt: Date(timeIntervalSince1970: 0))

        XCTAssertEqual(saved.radiusMeters, 5_000, "recorded a radius nobody downloaded")
        XCTAssertEqual(saved.maxZoom, 15)
        XCTAssertEqual(saved.minZoom, 10)
        XCTAssertEqual(saved.source, TileSource.usgsImageryTopo.rawValue,
                       "a wrong source makes the tiles unreclaimable")
        XCTAssertEqual(saved.lat, pad.latitude, accuracy: 1e-9)
        XCTAssertEqual(saved.lon, pad.longitude, accuracy: 1e-9)
        XCTAssertEqual(saved.tileCount, 2_000)
        XCTAssertEqual(saved.bytes, 48_000_000)
    }

    /// The round trip that matters for deletion: the source recorded in the
    /// manifest must resolve back to the TileSource whose directory holds the
    /// tiles.
    func testSourceRoundTripsForDelete() {
        for source in TileSource.allCases {
            let saved = OfflineRegion(name: "R", spec: downloadedSpec, source: source,
                                      tileCount: 1, bytes: 1, savedAt: Date())
            XCTAssertEqual(saved.tileSource, source,
                           "\(source) does not round-trip — delete would target the wrong directory")
        }
    }

    /// `spec` is how the rest of the app re-derives the region's geometry (the
    /// coverage overlay reads it), so it has to come back out unchanged.
    func testSpecRoundTrips() {
        let saved = OfflineRegion(name: "Pad", spec: downloadedSpec, source: .usgsTopo,
                                  tileCount: 1, bytes: 1, savedAt: Date())
        XCTAssertEqual(saved.spec.radiusMeters, downloadedSpec.radiusMeters)
        XCTAssertEqual(saved.spec.minZoom, downloadedSpec.minZoom)
        XCTAssertEqual(saved.spec.maxZoom, downloadedSpec.maxZoom)
        XCTAssertEqual(saved.spec.center.latitude, downloadedSpec.center.latitude, accuracy: 1e-9)
    }

    /// The exact scenario from the finding, stated as data: a manifest built
    /// from the download cannot pick up the values the user changed to while
    /// it ran.
    func testMidDownloadEditsCannotReachTheManifest() {
        // What the user left the sheet showing when the download finished.
        let editedSpec = RegionSpec(center: pad, radiusMeters: 20_000, minZoom: 10, maxZoom: 13)
        let editedSource = TileSource.usgsTopo

        let saved = OfflineRegion(name: "Pad",
                                  spec: downloadedSpec,          // what was fetched
                                  source: .usgsImageryTopo,
                                  tileCount: 2_000, bytes: 48_000_000,
                                  savedAt: Date())

        XCTAssertNotEqual(saved.radiusMeters, editedSpec.radiusMeters)
        XCTAssertNotEqual(saved.maxZoom, editedSpec.maxZoom)
        XCTAssertNotEqual(saved.source, editedSource.rawValue)
    }
}
