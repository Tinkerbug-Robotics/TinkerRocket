//
//  OfflineTileCache.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 1 (see docs/plans/offline-maps.md).
//
//  On-disk read-through cache for map tiles. Tiles live under Application
//  Support — NOT Caches, which iOS purges under storage pressure, which would
//  silently delete the field data the user pre-downloaded. Keyed per source so
//  switching basemaps never mixes imagery.
//
//  Trial 1 fills this passively (tiles you view online get stored, then render
//  offline). The explicit region downloader lands in Trial 2.
//

import Foundation

final class OfflineTileCache {
    static let shared = OfflineTileCache()

    private let root: URL
    private let fm = FileManager.default

    private init() {
        let base = fm.urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        root = base.appendingPathComponent("OfflineTiles", isDirectory: true)
        try? fm.createDirectory(at: root, withIntermediateDirectories: true)
        excludeFromBackup(root)
    }

    /// Cached bytes for a tile, or nil if not stored.
    func tileData(source: String, z: Int, x: Int, y: Int) -> Data? {
        try? Data(contentsOf: fileURL(source: source, z: z, x: x, y: y))
    }

    /// Persist tile bytes atomically, creating intermediate directories.
    func store(_ data: Data, source: String, z: Int, x: Int, y: Int) {
        let url = fileURL(source: source, z: z, x: x, y: y)
        try? fm.createDirectory(at: url.deletingLastPathComponent(),
                                withIntermediateDirectories: true)
        try? data.write(to: url, options: .atomic)
    }

    /// Bytes on disk for a source (for the future region manager / debug).
    func byteCount(source: String) -> Int64 {
        let dir = root.appendingPathComponent(source, isDirectory: true)
        guard let e = fm.enumerator(at: dir, includingPropertiesForKeys: [.fileSizeKey]) else { return 0 }
        var total: Int64 = 0
        for case let url as URL in e {
            total += Int64((try? url.resourceValues(forKeys: [.fileSizeKey]))?.fileSize ?? 0)
        }
        return total
    }

    /// Delete the given tiles for a source from disk (used when a saved region
    /// is removed). Best-effort.
    func removeTiles(source: String, tiles: [TileXYZ]) {
        for t in tiles {
            try? fm.removeItem(at: fileURL(source: source, z: t.z, x: t.x, y: t.y))
        }
    }

    // MARK: - Paths

    private func fileURL(source: String, z: Int, x: Int, y: Int) -> URL {
        root.appendingPathComponent(source, isDirectory: true)
            .appendingPathComponent("\(z)", isDirectory: true)
            .appendingPathComponent("\(x)", isDirectory: true)
            .appendingPathComponent("\(y).tile", isDirectory: false)
    }

    private func excludeFromBackup(_ url: URL) {
        var u = url
        var values = URLResourceValues()
        values.isExcludedFromBackup = true
        try? u.setResourceValues(values)
    }
}
