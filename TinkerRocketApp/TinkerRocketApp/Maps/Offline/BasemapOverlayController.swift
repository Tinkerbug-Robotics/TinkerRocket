//
//  BasemapOverlayController.swift
//  TinkerRocketApp
//
//  Offline maps (see docs/plans/offline-maps.md).
//
//  Shared helper that keeps an MKMapView's basemap-replacing tile overlay in
//  sync with the selected TileSource — swapping only when the source actually
//  changes, so panning and per-tick data updates don't tear down and reload
//  tiles. Used by both the Rocket Map and Drift Cast map coordinators so the two
//  views stay aligned.
//

import MapKit

final class BasemapOverlayController {
    private var currentSource: TileSource?
    private(set) var overlay: MKTileOverlay?

    func apply(_ source: TileSource, to mapView: MKMapView) {
        guard currentSource != source else { return }
        currentSource = source

        if let existing = overlay {
            mapView.removeOverlay(existing)
            overlay = nil
        }
        // Apple basemap cases render natively (no overlay); USGS cases draw a
        // cached, basemap-replacing overlay above the labels.
        if source.isCacheable {
            let tile = CachingTileOverlay(source: source)
            mapView.addOverlay(tile, level: .aboveLabels)
            overlay = tile
        }
    }
}
