import XCTest
@testable import TinkerRocketApp

/// #838 item 1 — the saved-flight 3D trajectory fetched Esri imagery with no
/// cache, so it was blank offline while its twin was not.
///
/// `FlightTrajectoryView.fetchArcGISImagery` and
/// `DriftCastView.fetchArcGISImagery` were byte-for-byte identical. When the
/// app moved off Esri World Imagery, only the Drift Cast copy was migrated —
/// to the public-domain USGS endpoint and through `OfflineTileCache`'s blob
/// APIs, which the cache's own section header advertises as being for
/// "non-tile images, e.g. the 3D ground texture".
///
/// So at a launch field with no signal: Flight Logs ▸ a downloaded flight ▸
/// View Trajectory ▸ 3D left the ground a grey placeholder with a wireframe
/// grid and no terrain reference, while Drift Cast for the same site rendered
/// its cached texture. With signal, every entry re-downloaded a 1024×1024 PNG.
///
/// Esri is not a licensing option here either: `TileSource` records that its
/// free terms restrict offline caching, "which is the point of this feature".
///
/// There is now ONE copy, so the next migration cannot leave a view behind.
final class SceneGroundTextureTests: XCTestCase {

    private let pad = (lat: 40.1, lon: -105.2)

    /// THE regression: the endpoint must be the public-domain USGS one.
    func testUsesTheUsgsEndpointNotEsri() throws {
        let req = try XCTUnwrap(SceneGroundTexture.request(
            refLat: pad.lat, refLon: pad.lon, extent: 500))
        let url = req.url.absoluteString

        XCTAssertTrue(url.hasPrefix(
            "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export"),
            "not the USGS endpoint: \(url)")
        XCTAssertFalse(url.contains("arcgisonline.com"),
                       "Esri World Imagery cannot be cached offline under its free terms")
        XCTAssertFalse(url.contains("World_Imagery"))
    }

    /// The same scene must produce the same key, or the cache never hits and
    /// the view re-downloads on every entry.
    func testTheCacheKeyIsStableForTheSameScene() throws {
        let a = try XCTUnwrap(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 500))
        let b = try XCTUnwrap(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 500))
        XCTAssertEqual(a.cacheKey, b.cacheKey)
    }

    /// The key is derived from the bbox alone, so the Drift Cast view and the
    /// flight trajectory at one site SHARE a texture instead of each fetching.
    func testTwoViewsOfOneSiteShareTheKey() throws {
        // Both call the same entry point with the same geometry — that is the
        // point of there being one implementation.
        let fromTrajectory = try XCTUnwrap(
            SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 800))
        let fromDriftCast = try XCTUnwrap(
            SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 800))
        XCTAssertEqual(fromTrajectory.cacheKey, fromDriftCast.cacheKey)
    }

    /// A different site must not collide onto a cached texture of another one.
    func testDifferentSitesGetDifferentKeys() throws {
        let a = try XCTUnwrap(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 500))
        let b = try XCTUnwrap(SceneGroundTexture.request(refLat: 39.0, refLon: -104.0, extent: 500))
        XCTAssertNotEqual(a.cacheKey, b.cacheKey)
    }

    /// Zooming out is a different texture, not the same one stretched.
    func testDifferentExtentsGetDifferentKeys() throws {
        let near = try XCTUnwrap(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 200))
        let far  = try XCTUnwrap(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: 2000))
        XCTAssertNotEqual(near.cacheKey, far.cacheKey)
    }

    /// The bbox brackets the pad, and grows with extent.
    func testTheBboxIsCentredOnThePad() throws {
        let req = try XCTUnwrap(SceneGroundTexture.request(
            refLat: pad.lat, refLon: pad.lon, extent: 500))
        let bbox = req.cacheKey.replacingOccurrences(of: "3d_", with: "")
            .split(separator: ",").compactMap { Double($0) }
        XCTAssertEqual(bbox.count, 4)
        let (west, south, east, north) = (bbox[0], bbox[1], bbox[2], bbox[3])

        XCTAssertLessThan(west, pad.lon);  XCTAssertGreaterThan(east, pad.lon)
        XCTAssertLessThan(south, pad.lat); XCTAssertGreaterThan(north, pad.lat)
        XCTAssertEqual((west + east) / 2, pad.lon, accuracy: 1e-9)
        XCTAssertEqual((south + north) / 2, pad.lat, accuracy: 1e-9)
    }

    /// A non-finite scene (an empty or corrupt track) must not build a URL
    /// with "nan" in it and cache a failure under that key.
    func testNonFiniteInputsProduceNoRequest() {
        XCTAssertNil(SceneGroundTexture.request(refLat: .nan, refLon: pad.lon, extent: 500))
        XCTAssertNil(SceneGroundTexture.request(refLat: pad.lat, refLon: .nan, extent: 500))
        XCTAssertNil(SceneGroundTexture.request(refLat: pad.lat, refLon: pad.lon, extent: .infinity))
    }

    /// The request the server actually needs.
    func testRequestParameters() throws {
        let url = try XCTUnwrap(SceneGroundTexture.request(
            refLat: pad.lat, refLon: pad.lon, extent: 500)).url.absoluteString
        for param in ["bboxSR=4326", "imageSR=4326", "size=1024,1024", "format=png", "f=image"] {
            XCTAssertTrue(url.contains(param), "missing \(param)")
        }
    }
}
