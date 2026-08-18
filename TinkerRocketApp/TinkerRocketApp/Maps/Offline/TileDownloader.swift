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

    /// Main-actor isolated (it drives SwiftUI) but with a nonisolated deinit —
    /// see OfflineTileCache for why an isolated deinit aborts the process on
    /// iOS 26.2 at this deployment target.
    nonisolated deinit {}

    /// `.failed` means NOTHING was retrieved — every tile errored or 404'd
    /// and none was already cached. It is a distinct outcome from `.finished`
    /// because a run with no connectivity used to reach `.finished` with 0
    /// bytes and save a 0 MB region: a saved area that looks real in the
    /// list, and is discovered empty at the field with no signal, which is
    /// the one place it was meant to work.
    enum Phase: Equatable { case idle, downloading, finished, cancelled, failed }

    @Published private(set) var phase: Phase = .idle
    @Published private(set) var done = 0
    @Published private(set) var total = 0
    @Published private(set) var bytes: Int64 = 0

    /// Tiles that errored or answered non-200. Surfaced so a partly-covered
    /// area can say so instead of looking complete.
    @Published private(set) var failed = 0

    private var cancelFlag = false
    private let session: URLSession
    private let cache: OfflineTileCache
    private let maxConcurrent = 5

    /// Test seam: cache and session injected so a test can point at a scratch
    /// directory and a stubbed upstream (Android's downloader takes both the
    /// same way). Production uses the defaults.
    init(cache: OfflineTileCache = .shared,
         session: URLSession = URLSession(configuration: .default)) {
        self.cache = cache
        self.session = session
    }

    /// Tiles THIS run wrote — cache hits are excluded, because `fetchOne`
    /// returns early on them. It is the undo log for a cancel: see `start`.
    private var created: [TileXYZ] = []
    private let createdLock = NSLock()
    /// Off-main tally of `failed`, so the terminal decision doesn't have to
    /// hop to the main queue to read the @Published mirror.
    private var failedCount = 0

    var isRunning: Bool { phase == .downloading }

    /// Download `region` and, on success, hand the totals to `onFinished` so
    /// the caller can record it.
    ///
    /// Both terminal outcomes are settled HERE rather than by whoever is
    /// watching `phase`, because the work outlives the sheet that started it.
    /// When the view owned them, dismissing mid-run left the download to
    /// finish into a manifest nobody wrote — a whole region of tiles on disk
    /// that the storage total could not see and Delete could never reclaim.
    ///
    /// A cancel deletes exactly what this run created, which is a precise
    /// undo: tiles already on disk are never re-fetched, so an overlapping
    /// saved region keeps every tile it had.
    func start(region: RegionSpec,
               source: TileSource,
               onFinished: ((Int, Int64) -> Void)? = nil) {
        guard let template = source.urlTemplate, !isRunning else { return }
        let tiles = TileMath.tiles(for: region)
        let key = source.rawValue

        cancelFlag = false
        createdLock.lock(); created.removeAll(); failedCount = 0; createdLock.unlock()
        phase = .downloading
        total = tiles.count
        done = 0
        bytes = 0
        failed = 0

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

            // cancel() is a flag, not task cancellation, so this cleanup is
            // guaranteed to run.
            self.createdLock.lock()
            let mine = self.created
            self.created.removeAll()
            self.createdLock.unlock()

            // Read the off-main tally, not the @Published mirror, so the
            // deletes stay off the main thread — a large cancelled run is
            // thousands of file removals.
            self.createdLock.lock()
            let failures = self.failedCount
            self.createdLock.unlock()

            // Nothing new was retrieved and something failed, so this run
            // accomplished nothing and there is no area to record.
            //
            // Not `failures == tiles.count`: that misses the case this was
            // found in on the Android bench. The preview map caches tiles
            // through the same cache, so with the network down 13 of 2991
            // tiles were already on disk, `failures` came in 13 short of the
            // total, and a 13 km area was saved holding 0.2% of itself.
            //
            // Not zero bytes either — a cache hit reports its size, so those
            // 13 tiles counted as ~200 KB. `mine` holds exactly the tiles this
            // run pulled off the network, which is the question being asked.
            // Empty with no failures is the honest opposite — every tile was
            // already cached — and still saves.
            let nothingArrived = mine.isEmpty && failures > 0
            if self.cancelFlag || nothingArrived {
                self.cache.removeTiles(source: key, tiles: mine)
            }
            DispatchQueue.main.async {
                if self.cancelFlag {
                    self.phase = .cancelled
                } else if nothingArrived {
                    self.phase = .failed
                } else {
                    onFinished?(self.total, self.bytes)
                    self.phase = .finished
                }
            }
        }
    }

    func cancel() { cancelFlag = true }

    func reset() {
        guard !isRunning else { return }
        phase = .idle; done = 0; total = 0; bytes = 0; failed = 0
    }

    // MARK: - Internals

    /// - Parameter byteCount: bytes retrieved, or nil when the tile failed.
    private func report(_ byteCount: Int?) {
        if byteCount == nil {
            createdLock.lock(); failedCount += 1; createdLock.unlock()
        }
        DispatchQueue.main.async {
            self.done += 1
            if let byteCount { self.bytes += Int64(byteCount) } else { self.failed += 1 }
        }
    }

    /// Fetch (or read from cache) one tile, then call completion with its byte
    /// size — or nil when it errored or answered non-200, which is distinct
    /// from a zero-byte body because "nothing came back" is exactly what a
    /// failed run needs to count. Completion is always called exactly once,
    /// so progress still completes either way.
    private func fetchOne(_ t: TileXYZ, key: String, template: String,
                          completion: @escaping (Int?) -> Void) {
        if let data = cache.tileData(source: key, z: t.z, x: t.x, y: t.y) {
            completion(data.count)
            return
        }
        let urlStr = template
            .replacingOccurrences(of: "{z}", with: "\(t.z)")
            .replacingOccurrences(of: "{x}", with: "\(t.x)")
            .replacingOccurrences(of: "{y}", with: "\(t.y)")
        guard let url = URL(string: urlStr) else { completion(nil); return }

        var req = URLRequest(url: url)
        req.setValue("TinkerRocketApp/1.0 (offline map cache)", forHTTPHeaderField: "User-Agent")
        session.dataTask(with: req) { data, response, _ in
            if let data = data,
               let http = response as? HTTPURLResponse, http.statusCode == 200 {
                self.cache.store(data, source: key, z: t.z, x: t.x, y: t.y)
                self.createdLock.lock()
                self.created.append(t)
                self.createdLock.unlock()
                completion(data.count)
            } else {
                completion(nil)  // unreachable/4xx — counted as failed
            }
        }.resume()
    }
}
