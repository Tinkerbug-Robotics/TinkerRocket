//
//  TileSource.swift
//  TinkerRocketApp
//
//  Offline maps — see docs/plans/offline-maps.md.
//
//  Selectable map imagery sources, shared by the Rocket Map and (later) Drift
//  Cast. Apple cases use the built-in MapKit basemap, which CANNOT be cached for
//  offline use (no public API; against MapKit terms) — they stay as the global
//  online option. The USGS cases are public-domain tiles drawn via a
//  basemap-replacing MKTileOverlay and are the only sources we cache/download
//  for offline use. (Esri World Imagery looked great online but its free terms
//  restrict offline caching, which is the point of this feature — see the
//  international-imagery follow-up issue.)
//

import MapKit

enum TileSource: String, CaseIterable, Identifiable {
    case appleStandard
    case appleHybrid
    case usgsImagery
    case usgsImageryTopo
    case usgsTopo

    var id: String { rawValue }

    var displayName: String {
        switch self {
        case .appleStandard:   return "Apple Standard"
        case .appleHybrid:     return "Apple Hybrid"
        case .usgsImagery:     return "USGS Imagery"
        case .usgsImageryTopo: return "USGS Imagery + Topo"
        case .usgsTopo:        return "USGS Topo"
        }
    }

    /// SF Symbol for the source menu.
    var symbol: String {
        switch self {
        case .appleStandard:   return "map"
        case .appleHybrid:     return "globe.americas.fill"
        case .usgsImagery:     return "globe.desk.fill"
        case .usgsImageryTopo: return "mountain.2.fill"
        case .usgsTopo:        return "map.fill"
        }
    }

    /// Apple basemap cases render natively; USGS cases draw via MKTileOverlay.
    var isAppleBasemap: Bool { self == .appleStandard || self == .appleHybrid }

    /// Whether this source can be cached / downloaded for offline use.
    /// USGS (public domain) yes; Apple no.
    var isCacheable: Bool { !isAppleBasemap }

    /// Deepest zoom this source actually serves.  The USGS ArcGIS basemaps
    /// stop at 16 — asking for 17/18 returns 404 for every tile in the band,
    /// which the downloader counts as done with 0 bytes, so an over-deep
    /// request silently inflates the estimate and downloads nothing.
    /// Matches Android's per-source `TileSource.maxZoom`.
    var maxZoom: Int {
        isAppleBasemap ? 19 : 16
    }

    /// MKMapType for the Apple cases. For USGS cases the value is irrelevant
    /// (the overlay replaces the basemap) but `.standard` keeps Apple from
    /// fetching its own imagery underneath.
    var appleMapType: MKMapType {
        self == .appleHybrid ? .hybrid : .standard
    }

    /// `{x}`/`{y}`/`{z}` URL template for USGS cases, nil for the Apple basemap.
    /// ArcGIS REST tile endpoints are ordered `z/y/x`.
    var urlTemplate: String? {
        switch self {
        case .appleStandard, .appleHybrid:
            return nil
        case .usgsImagery:
            return "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{z}/{y}/{x}"
        case .usgsImageryTopo:
            return "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryTopo/MapServer/tile/{z}/{y}/{x}"
        case .usgsTopo:
            return "https://basemap.nationalmap.gov/arcgis/rest/services/USGSTopo/MapServer/tile/{z}/{y}/{x}"
        }
    }

    /// Short attribution shown while the source is active (provider terms).
    var attribution: String? {
        isAppleBasemap ? nil : "USGS · The National Map"
    }
}
