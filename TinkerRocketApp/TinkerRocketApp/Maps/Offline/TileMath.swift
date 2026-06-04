//
//  TileMath.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 2 (see docs/plans/offline-maps.md).
//
//  Slippy-map tile math: convert a region (center + radius + zoom range) into
//  the set of (z,x,y) tiles to download, and count them for the pre-download
//  size estimate.
//

import Foundation
import CoreLocation

/// A single web-mercator tile coordinate.
struct TileXYZ: Hashable {
    let z: Int
    let x: Int
    let y: Int
}

/// What to download: a circle (center + radius) over a span of zoom levels.
struct RegionSpec {
    var center: CLLocationCoordinate2D
    var radiusMeters: Double
    var minZoom: Int = 10
    var maxZoom: Int

    /// Lat/lon bounding box of the circle (small-angle approximation — fine for
    /// the few-km radii used here).
    var bbox: (latMin: Double, latMax: Double, lonMin: Double, lonMax: Double) {
        let dLat = radiusMeters / 111_320.0
        let cosLat = max(0.01, cos(center.latitude * .pi / 180))
        let dLon = radiusMeters / (111_320.0 * cosLat)
        return (center.latitude - dLat, center.latitude + dLat,
                center.longitude - dLon, center.longitude + dLon)
    }
}

enum TileMath {
    static func tileX(lon: Double, z: Int) -> Int {
        let n = Double(1 << z)
        return Int(floor((lon + 180.0) / 360.0 * n))
    }

    static func tileY(lat: Double, z: Int) -> Int {
        let n = Double(1 << z)
        let r = lat * .pi / 180.0
        return Int(floor((1.0 - asinh(tan(r)) / .pi) / 2.0 * n))
    }

    private static func clamp(_ v: Int, _ lo: Int, _ hi: Int) -> Int {
        min(max(v, lo), hi)
    }

    /// Inclusive tile rectangle for a bbox at one zoom level.
    private static func rect(_ b: (latMin: Double, latMax: Double, lonMin: Double, lonMax: Double),
                             z: Int) -> (xMin: Int, xMax: Int, yMin: Int, yMax: Int) {
        let n = 1 << z
        // North (latMax) maps to the smaller y; south (latMin) to the larger y.
        return (clamp(tileX(lon: b.lonMin, z: z), 0, n - 1),
                clamp(tileX(lon: b.lonMax, z: z), 0, n - 1),
                clamp(tileY(lat: b.latMax, z: z), 0, n - 1),
                clamp(tileY(lat: b.latMin, z: z), 0, n - 1))
    }

    /// Total tile count for the region (for the size estimate; no allocation).
    static func tileCount(for region: RegionSpec) -> Int {
        let b = region.bbox
        var count = 0
        for z in min(region.minZoom, region.maxZoom)...region.maxZoom {
            let r = rect(b, z: z)
            count += (r.xMax - r.xMin + 1) * (r.yMax - r.yMin + 1)
        }
        return count
    }

    /// The full list of tiles to download for the region.
    static func tiles(for region: RegionSpec) -> [TileXYZ] {
        let b = region.bbox
        var out: [TileXYZ] = []
        for z in min(region.minZoom, region.maxZoom)...region.maxZoom {
            let r = rect(b, z: z)
            for x in r.xMin...r.xMax {
                for y in r.yMin...r.yMax {
                    out.append(TileXYZ(z: z, x: x, y: y))
                }
            }
        }
        return out
    }
}
