//
//  TileDownloader.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 2 (see docs/plans/offline-maps.md).
//
//  Downloads every tile in a RegionSpec to the OfflineTileCache with bounded
//  concurrency, publishing progress for the UI. Resumable (already-cached tiles
//  are counted instantly, not re-fetched) and cancelable. GCD-based to keep the
//  concurrency bound simple and predictable.
//

import Foundation
import Combine

/// Rough bytes per imagery tile — only used for the pre-download estimate.
let kEstimatedTileBytes: Int64 = 25_000

final class TileDownloader: ObservableObject {
    enum Phase: Equatable { case idle, downloading, finished, cancelled }

    @Published private(set) var phase: Phase = .idle
    @Published private(set) var done = 0
    @Published private(set) var total = 0
    @Published private(set) var bytes: Int64 = 0

    private var cancelFlag = false
    private let session = URLSession(configuration: .default)
    private let maxConcurrent = 5

    var isRunning: Bool { phase == .downloading }

    func start(region: RegionSpec, source: TileSource) {
        guard let template = source.urlTemplate, !isRunning else { return }
        let tiles = TileMath.tiles(for: region)
        let key = source.rawValue

        cancelFlag = false
        phase = .downloading
        total = tiles.count
        done = 0
        bytes = 0

        DispatchQueue.global(qos: .utility).async { [weak self] in
            guard let self = self else { return }
            let sem = DispatchSemaphore(value: self.maxConcurrent)
            let group = DispatchGroup()

            for tile in tiles {
                if self.cancelFlag { break }
                sem.wait()
                group.enter()
                self.fetchOne(tile, key: key, template: template) { byteCount in
                    self.report(byteCount)
                    sem.signal()
                    group.leave()
                }
            }
            group.wait()

            DispatchQueue.main.async {
                self.phase = self.cancelFlag ? .cancelled : .finished
            }
        }
    }

    func cancel() { cancelFlag = true }

    func reset() {
        guard !isRunning else { return }
        phase = .idle; done = 0; total = 0; bytes = 0
    }

    // MARK: - Internals

    private func report(_ byteCount: Int) {
        DispatchQueue.main.async {
            self.done += 1
            self.bytes += Int64(byteCount)
        }
    }

    /// Fetch (or read from cache) one tile, then call completion with its byte
    /// size. Completion is always called exactly once.
    private func fetchOne(_ t: TileXYZ, key: String, template: String,
                          completion: @escaping (Int) -> Void) {
        if let data = OfflineTileCache.shared.tileData(source: key, z: t.z, x: t.x, y: t.y) {
            completion(data.count)
            return
        }
        let urlStr = template
            .replacingOccurrences(of: "{z}", with: "\(t.z)")
            .replacingOccurrences(of: "{x}", with: "\(t.x)")
            .replacingOccurrences(of: "{y}", with: "\(t.y)")
        guard let url = URL(string: urlStr) else { completion(0); return }

        var req = URLRequest(url: url)
        req.setValue("TinkerRocketApp/1.0 (offline map cache)", forHTTPHeaderField: "User-Agent")
        session.dataTask(with: req) { data, response, _ in
            if let data = data,
               let http = response as? HTTPURLResponse, http.statusCode == 200 {
                OfflineTileCache.shared.store(data, source: key, z: t.z, x: t.x, y: t.y)
                completion(data.count)
            } else {
                completion(0)  // missing/4xx tile — count it so progress completes
            }
        }.resume()
    }
}
