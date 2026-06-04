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

    init() {
        let base = fm.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        try? fm.createDirectory(at: base, withIntermediateDirectories: true)
        url = base.appendingPathComponent("offline_regions.json")
        load()
    }

    var totalBytes: Int64 { regions.reduce(0) { $0 + $1.bytes } }

    func add(_ region: OfflineRegion) {
        regions.removeAll { $0.id == region.id }
        regions.insert(region, at: 0)
        save()
    }

    /// Remove a region from the manifest and delete its tiles from disk.
    /// (Overlapping regions are rare here; a shared tile that gets removed will
    /// simply re-download next time it's viewed online.)
    func delete(_ region: OfflineRegion) {
        OfflineTileCache.shared.removeTiles(source: region.source,
                                            tiles: TileMath.tiles(for: region.spec))
        regions.removeAll { $0.id == region.id }
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
