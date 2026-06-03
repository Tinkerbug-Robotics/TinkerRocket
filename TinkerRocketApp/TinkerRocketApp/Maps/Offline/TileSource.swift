//
//  TileSource.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 0 (see docs/plans/offline-maps.md).
//
//  Defines the selectable map imagery sources. Apple cases use the built-in
//  MapKit basemap, which CANNOT be cached for offline use. The tile cases point
//  at cache-permissive providers (USGS public domain, Esri) and are drawn via an
//  MKTileOverlay that replaces the basemap — these are the candidates for the
//  eventual offline download. Trial 0 has no caching yet; this just lets us A/B
//  the imagery on-device, online, before building the cache/download machinery.
//

import MapKit

enum TileSource: String, CaseIterable, Identifiable {
    case appleStandard
    case appleHybrid
    case usgsImagery
    case usgsImageryTopo
    case usgsTopo
    case esriImagery

    var id: String { rawValue }

    var displayName: String {
        switch self {
        case .appleStandard:   return "Apple Standard"
        case .appleHybrid:     return "Apple Hybrid"
        case .usgsImagery:     return "USGS Imagery"
        case .usgsImageryTopo: return "USGS Imagery + Topo"
        case .usgsTopo:        return "USGS Topo"
        case .esriImagery:     return "Esri World Imagery"
        }
    }

    /// SF Symbol for the source menu.
    var symbol: String {
        switch self {
        case .appleStandard:   return "map"
        case .appleHybrid:     return "globe.americas.fill"
        case .usgsImagery, .esriImagery: return "globe.desk.fill"
        case .usgsImageryTopo: return "mountain.2.fill"
        case .usgsTopo:        return "map.fill"
        }
    }

    /// Apple basemap cases render natively; tile cases draw via MKTileOverlay.
    var isAppleBasemap: Bool { self == .appleStandard || self == .appleHybrid }

    /// MKMapType for the Apple cases. For tile cases the value is irrelevant
    /// (the overlay replaces the basemap) but `.standard` keeps Apple from
    /// fetching imagery underneath.
    var appleMapType: MKMapType {
        self == .appleHybrid ? .hybrid : .standard
    }

    /// `{x}`/`{y}`/`{z}` URL template for tile cases, nil for the Apple basemap.
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
        case .esriImagery:
            return "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        }
    }

    /// Short attribution shown while the source is active (provider terms).
    var attribution: String? {
        switch self {
        case .appleStandard, .appleHybrid: return nil
        case .usgsImagery, .usgsImageryTopo, .usgsTopo: return "USGS · The National Map"
        case .esriImagery: return "Esri, Maxar, Earthstar Geographics"
        }
    }

    /// Builds the basemap-replacing tile overlay for tile cases (nil for Apple).
    /// Trial 0: no on-disk caching yet — MKTileOverlay fetches straight from the
    /// provider. The read-through cache lands in Trial 1.
    func makeOverlay() -> MKTileOverlay? {
        guard let template = urlTemplate else { return nil }
        let overlay = MKTileOverlay(urlTemplate: template)
        overlay.canReplaceMapContent = true
        overlay.minimumZ = 0
        overlay.maximumZ = 19
        return overlay
    }
}
