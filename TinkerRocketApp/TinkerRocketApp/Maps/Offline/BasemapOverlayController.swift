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

    /// The app builds with SWIFT_DEFAULT_ACTOR_ISOLATION = MainActor against
    /// an iOS 16 deployment target, so a main-actor class gets an ISOLATED
    /// deinit whose executor hop comes from the back-deploy shim compiled
    /// into the binary — and that shim double-frees on iOS 26.2:
    ///
    ///   POINTER_BEING_FREED_WAS_NOT_ALLOCATED
    ///     swift_task_deinitOnExecutorMainActorBackDeploy
    ///     BasemapOverlayController.__deallocating_deinit
    ///     @objc RocketMapView.Coordinator.__ivar_destroyer
    ///
    /// This is the third time this trap has been hit (PR #734 OfflineTileCache
    /// / OfflineRegionStore / TileDownloader, PR #817 DeploymentWatcher), and
    /// it is invisible on a modern simulator — ios-tests.yml sorts devices and
    /// takes the last, which is the SE on 26.2. The class stays main-actor
    /// because apply(_:to:) mutates an MKMapView; only the deinit opts out.
    nonisolated deinit {}

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
