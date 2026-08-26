//
//  OfflineRegion.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 2 (see docs/plans/offline-maps.md).
//
//  A saved offline region and a persisted store of them. Lets the user see what
//  they've downloaded, how much space it uses, and delete it to reclaim space.
//  Manifest is JSON under Application Support.
//

import Foundation
import CoreLocation
import Combine

struct OfflineRegion: Identifiable, Codable, Equatable {
    var id: UUID = UUID()
    var name: String
    var lat: Double
    var lon: Double
    var radiusMeters: Double
    var minZoom: Int
    var maxZoom: Int
    var source: String          // TileSource.rawValue
    var tileCount: Int
    var bytes: Int64
    var savedAt: Date

    var center: CLLocationCoordinate2D { .init(latitude: lat, longitude: lon) }

    var spec: RegionSpec {
        RegionSpec(center: center, radiusMeters: radiusMeters,
                   minZoom: minZoom, maxZoom: maxZoom)
    }

    var tileSource: TileSource? { TileSource(rawValue: source) }
}

extension OfflineRegion {
    /// Manifest entry for a completed download, derived from the SAME
    /// `RegionSpec` and `TileSource` that were handed to the downloader
    /// (#836 item 1).
    ///
    /// SaveAreaView used to build this by reading `radiusKm`, `maxZoom` and
    /// `source` out of `@State` inside the escaping completion closure — which
    /// runs when the download FINISHES, minutes later — while
    /// `downloader.start` had snapshotted the real geometry by value at tap
    /// time. Nothing disabled the source picker or the sliders during the run,
    /// so a user who adjusted them mid-download got a manifest describing a
    /// region nobody had fetched.
    ///
    /// Two things went wrong from there: the saved area claimed coverage it
    /// did not have and drew the hatched "not downloaded" placeholder out to
    /// its false radius; and `OfflineRegionStore.delete` calls
    /// `cache.removeTiles(source:)` with the recorded source, so a wrong
    /// source deleted nothing while the storage total dropped by `bytes` —
    /// leaving tiles on disk that the total could not see and Delete could
    /// never reclaim, which is exactly the failure TileDownloader says was
    /// fixed.
    ///
    /// Taking the spec means the manifest and the download cannot describe
    /// different regions: there is only one value.
    ///
    /// Use THIS initialiser for a finished download — the memberwise one
    /// takes radius, zoom and source separately and so can express a
    /// manifest that does not match what was fetched.
    init(name: String, spec: RegionSpec, source: TileSource,
         tileCount: Int, bytes: Int64, savedAt: Date) {
        self.init(name: name,
                  lat: spec.center.latitude,
                  lon: spec.center.longitude,
                  radiusMeters: spec.radiusMeters,
                  minZoom: spec.minZoom,
                  maxZoom: spec.maxZoom,
                  source: source.rawValue,
                  tileCount: tileCount,
                  bytes: bytes,
                  savedAt: savedAt)
    }
}


final class OfflineRegionStore: ObservableObject {
    static let shared = OfflineRegionStore()

    /// This one stays main-actor isolated — it drives SwiftUI — so it only
    /// opts its deinit out. See OfflineTileCache for why an isolated deinit
    /// aborts the process on iOS 26.2 at this deployment target.
    nonisolated deinit {}

    @Published private(set) var regions: [OfflineRegion] = []

    private let url: URL
    private let fm = FileManager.default
    private let cache: OfflineTileCache

    /// Test seam: manifest directory and tile cache injected, so a test
    /// never touches the real ones (Android's store takes both the same way).
    init(directory: URL, cache: OfflineTileCache = .shared) {
        self.cache = cache
        try? FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        url = directory.appendingPathComponent("offline_regions.json")
        load()
    }

    convenience init() {
        let base = FileManager.default
            .urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        self.init(directory: base)
    }

    var totalBytes: Int64 { regions.reduce(0) { $0 + $1.bytes } }

    func add(_ region: OfflineRegion) {
        regions.removeAll { $0.id == region.id }
        regions.insert(region, at: 0)
        save()
    }

    /// Remove a region from the manifest and delete the tiles no SURVIVING
    /// region still needs.
    ///
    /// Deleting by geometry alone punched holes in overlapping regions: two
    /// areas over the same launch site share tile paths, so dropping one took
    /// the shared tiles with it and left the other quietly incomplete. The old
    /// comment shrugged that a lost tile "will simply re-download next time
    /// it's viewed online" — true at home, and exactly wrong at a field with
    /// no signal, where these regions are the entire point.
    ///
    /// Only same-source regions can share tiles (the source is a directory in
    /// the path), so the keep-set is built from those alone. Tiles cached
    /// passively by panning the map are claimed by no region and are still
    /// removed if they fall inside — they cost one re-fetch online, and
    /// tracking them would mean refcounting the whole cache.
    func delete(_ region: OfflineRegion) {
        let survivors = regions.filter { $0.id != region.id }
        let keep = Set(survivors
            .filter { $0.source == region.source }
            .flatMap { TileMath.tiles(for: $0.spec) })
        let doomed = TileMath.tiles(for: region.spec).filter { !keep.contains($0) }
        cache.removeTiles(source: region.source, tiles: doomed)
        regions = survivors
        save()
    }

    // MARK: - Persistence

    private func load() {
        guard let data = try? Data(contentsOf: url),
              let decoded = try? JSONDecoder().decode([OfflineRegion].self, from: data) else { return }
        regions = decoded
    }

    private func save() {
        guard let data = try? JSONEncoder().encode(regions) else { return }
        try? data.write(to: url, options: .atomic)
    }
}
