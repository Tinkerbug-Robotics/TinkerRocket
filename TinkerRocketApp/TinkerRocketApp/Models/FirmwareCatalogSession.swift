//
//  FirmwareCatalogSession.swift
//  TinkerRocketApp
//
//  Finding a published firmware image for the unit in front of you, and
//  getting its bytes (#773 step 4c). Port of core/session
//  FirmwareCatalogSession.kt — keep the two in step.
//
//  This is the sequencing only — which request follows which, and what each
//  outcome means to an operator. Deciding WHICH images fit is
//  FirmwareCatalog's job and proving the bytes are FirmwareRepository's, so
//  neither is repeated here. The view renders `state` and nothing else.
//
//  It ends at .downloaded. Handing the bytes to the OTA flow is the view's
//  business, and it goes through exactly the same path as a file the operator
//  picked by hand — including EspImage.check, which reads the downloaded
//  image's own header rather than trusting the manifest that described it.
//

import Combine
import Foundation

@MainActor
final class FirmwareCatalogSession: ObservableObject {

    enum State: Equatable {
        case idle
        /// Talking to GitHub. One API call, then one asset fetch.
        case checking
        /// A release was found and `images` are the ones for this unit, best
        /// first. `best` is nil when the unit's board is known and nothing in
        /// the release matches it — the list is still offered, because
        /// refusing to guess is not the same as refusing to show.
        /// `boardKnown` is whether the unit's board revision could be
        /// established at all — provisioned, or read out of its running
        /// version. When it could not, a nil `best` means "nobody knows what
        /// this is", which is a different thing to tell an operator than
        /// "this release has nothing for your board".
        /// `offline` is true when this came from the cache because the
        /// network could not be reached. The images are still real and still
        /// verified; the catalog may simply be older than what has since been
        /// published, and the operator should be told which they are looking
        /// at. `held` is the SHAs already on the phone, for a "have this"
        /// marker.
        case ready(release: FirmwareRelease, images: [FirmwareImage],
                   best: FirmwareImage?, alreadyRunning: Bool, boardKnown: Bool,
                   offline: Bool, held: Set<String>)
        /// Fetching `index` of `total` for offline use.
        case prefetching(image: FirmwareImage, index: Int, total: Int)
        /// Terminal for `prefetch`: how many landed, and what did not.
        case prefetched(release: FirmwareRelease, stored: Int,
                        failed: [String], bytesHeld: Int64)
        case downloading(image: FirmwareImage)
        /// Bytes that matched the manifest's size and SHA-256. Terminal.
        case downloaded(release: FirmwareRelease, image: FirmwareImage, bytes: Data)
        case failed(reason: String)
    }

    @Published private(set) var state: State = .idle

    private let repository: FirmwareRepository
    /// Where downloaded firmware is kept, or nil for a session that never
    /// stores anything. With a cache, a phone that fetched at home can still
    /// list and flash at a field with no signal — which #773's acceptance line
    /// asks for and is the ordinary condition at a launch site, not an edge.
    private let cache: FirmwareCache?
    private var task: Task<Void, Never>?
    private var release: FirmwareRelease?

    init(repository: FirmwareRepository = FirmwareRepository(
            fetch: FirmwareRepository.urlSessionFetch),
         cache: FirmwareCache? = FirmwareCache(directory: FirmwareCache.defaultDirectory())) {
        self.repository = repository
        self.cache = cache
    }

    /// Look for a release carrying an image for this unit.
    ///
    /// `expectedProject` is the hard filter, exactly as when a file is picked
    /// by hand — it is the only field separating a base station from an out
    /// computer, both ESP32-S3 with byte-identical app slots.
    func check(expectedProject: String, provisionedBoard: String?,
               runningVersion: String? = nil) {
        task?.cancel()
        state = .checking
        task = Task { [repository, cache] in
            let fetched = await repository.latestManifest()
            if Task.isCancelled { return }
            // No signal is the ordinary condition at a launch site, so it is a
            // fallback rather than a failure: anything cached is still real and
            // still verified on the way out.
            let offline = fetched == nil
            var pair: (FirmwareRelease, FirmwareManifest)?
            if let f = fetched {
                cache?.putCatalog(f)
                pair = (f.release, f.manifest)
            } else {
                pair = cache?.catalog()
            }
            guard let (rel, manifest) = pair else {
                // Deliberately not "no firmware available": the overwhelmingly
                // likely cause at a launch site is no signal, and telling an
                // operator their firmware is missing when their phone simply
                // cannot reach GitHub sends them looking in the wrong place.
                state = .failed(reason: "Could not reach the firmware releases, and "
                    + "nothing has been downloaded on this phone yet. Check the "
                    + "connection — or fetch a release before leaving for the field.")
                return
            }
            release = rel
            // The board the unit is provisioned with, or failing that the one
            // its running firmware claims to be. EspImage.check has always
            // used exactly this fallback; the catalog was never given it, so
            // on a board provisioned before #773 step 2 — which is every board
            // in the field — it ranked with nothing to rank on. See
            // FirmwareCatalog.best for what that produced on the bench.
            let trimmed = provisionedBoard?.trimmingCharacters(in: .whitespaces)
            let effectiveBoard = (trimmed?.isEmpty == false ? trimmed : nil)
                ?? EspImage.boardSuffix(of: runningVersion ?? "")

            let images = FirmwareCatalog.forUnit(manifest, expectedProject: expectedProject,
                                                 provisionedBoard: effectiveBoard)
            guard !images.isEmpty else {
                state = .failed(reason: "Release \(rel.tag) carries no \(expectedProject) image.")
                return
            }
            let best = FirmwareCatalog.best(manifest, expectedProject: expectedProject,
                                            provisionedBoard: effectiveBoard)
            state = .ready(release: rel, images: images, best: best,
                           alreadyRunning: best != nil && runningVersion != nil
                               && best?.version == runningVersion,
                           boardKnown: effectiveBoard != nil,
                           offline: offline,
                           held: cache?.heldShas() ?? [])
        }
    }

    /// Download one image and prove it against the manifest before showing it.
    func download(_ image: FirmwareImage) {
        guard let rel = release else {
            state = .failed(reason: "No release selected — check for updates first.")
            return
        }
        task?.cancel()
        // A cached hit is instantaneous and needs no network — the whole point
        // of having fetched at home. `cache.image` re-hashes before handing
        // anything back, so this is not a shortcut past verification.
        if let cached = cache?.image(image) {
            state = .downloaded(release: rel, image: image, bytes: cached)
            return
        }
        state = .downloading(image: image)
        task = Task { [repository, cache] in
            let got = await repository.download(release: rel, image: image)
            if Task.isCancelled { return }
            switch got {
            case .ok(let bytes):
                cache?.putImage(image, bytes)
                state = .downloaded(release: rel, image: image, bytes: bytes)
            // The split matters to whoever is standing at the pad: one is "try
            // again on better signal", the other is "do not flash this".
            // Neither hands any bytes back.
            case .unreachable(let why):
                state = .failed(reason: "Download did not finish: \(why)")
            case .corrupt(let why):
                state = .failed(reason: "Downloaded image does not match the release: \(why)")
            }
        }
    }

    /// Fetch several images and keep them, without flashing anything — the
    /// "do this at home" action.
    ///
    /// Reports progress as `.prefetching` and ends on `.prefetched` with what
    /// it managed. Deliberately does NOT fail the whole run on one bad image:
    /// getting three of four onto the phone before leaving is worth more than
    /// an all-or-nothing refusal.
    func prefetch(_ images: [FirmwareImage]) {
        guard let rel = release else {
            state = .failed(reason: "No release selected — check for updates first.")
            return
        }
        guard let store = cache else {
            state = .failed(reason: "This device has nowhere to keep downloads.")
            return
        }
        task?.cancel()
        task = Task { [repository] in
            var done = 0
            var failed: [String] = []
            for (i, img) in images.enumerated() {
                if Task.isCancelled { return }
                state = .prefetching(image: img, index: i + 1, total: images.count)
                if store.hasImage(img), store.image(img) != nil { done += 1; continue }
                switch await repository.download(release: rel, image: img) {
                case .ok(let bytes): store.putImage(img, bytes); done += 1
                case .unreachable: failed.append(img.file)
                case .corrupt: failed.append(img.file)
                }
            }
            // Superseded images age out here rather than filling the phone one
            // release at a time. Nothing dropped is precious; it can be
            // fetched again with a signal.
            store.prune(keep: Set(images.map { $0.sha256 }))
            state = .prefetched(release: rel, stored: done, failed: failed,
                                bytesHeld: store.bytesHeld())
        }
    }

    func reset() {
        task?.cancel()
        task = nil
        release = nil
        state = .idle
    }
}
