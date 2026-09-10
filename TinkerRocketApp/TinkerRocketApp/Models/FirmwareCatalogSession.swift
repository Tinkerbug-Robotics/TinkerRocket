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
        case ready(release: FirmwareRelease, images: [FirmwareImage],
                   best: FirmwareImage?, alreadyRunning: Bool)
        case downloading(image: FirmwareImage)
        /// Bytes that matched the manifest's size and SHA-256. Terminal.
        case downloaded(release: FirmwareRelease, image: FirmwareImage, bytes: Data)
        case failed(reason: String)
    }

    @Published private(set) var state: State = .idle

    private let repository: FirmwareRepository
    private var task: Task<Void, Never>?
    private var release: FirmwareRelease?

    init(repository: FirmwareRepository = FirmwareRepository(
        fetch: FirmwareRepository.urlSessionFetch)) {
        self.repository = repository
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
        task = Task { [repository] in
            guard let found = await repository.latestManifest() else {
                // Deliberately not "no firmware available": the overwhelmingly
                // likely cause at a launch site is no signal, and telling an
                // operator their firmware is missing when their phone simply
                // cannot reach GitHub sends them looking in the wrong place.
                if !Task.isCancelled {
                    state = .failed(reason: "Could not reach the firmware releases. "
                        + "Check the phone's connection — a published release cannot "
                        + "be seen without one.")
                }
                return
            }
            if Task.isCancelled { return }
            let (rel, manifest) = found
            release = rel
            let images = FirmwareCatalog.forUnit(manifest, expectedProject: expectedProject,
                                                 provisionedBoard: provisionedBoard)
            guard !images.isEmpty else {
                state = .failed(reason: "Release \(rel.tag) carries no \(expectedProject) image.")
                return
            }
            let best = FirmwareCatalog.best(manifest, expectedProject: expectedProject,
                                            provisionedBoard: provisionedBoard)
            state = .ready(release: rel, images: images, best: best,
                           alreadyRunning: best != nil && runningVersion != nil
                               && best?.version == runningVersion)
        }
    }

    /// Download one image and prove it against the manifest before showing it.
    func download(_ image: FirmwareImage) {
        guard let rel = release else {
            state = .failed(reason: "No release selected — check for updates first.")
            return
        }
        task?.cancel()
        state = .downloading(image: image)
        task = Task { [repository] in
            let got = await repository.download(release: rel, image: image)
            if Task.isCancelled { return }
            switch got {
            case .ok(let bytes):
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

    func reset() {
        task?.cancel()
        task = nil
        release = nil
        state = .idle
    }
}
