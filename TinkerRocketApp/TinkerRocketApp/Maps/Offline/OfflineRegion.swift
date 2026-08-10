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

final class OfflineRegionStore: ObservableObject {
    static let shared = OfflineRegionStore()

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
