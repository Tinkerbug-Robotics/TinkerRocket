//
//  FirmwareCache.swift
//  TinkerRocketApp
//
//  Firmware kept on the phone so a launch site with no signal is survivable
//  (#773). Port of core/session FirmwareCache.kt — keep the two in step.
//
//  WHAT IS STORED, AND WHY THE MANIFEST TOO. Images alone are not enough: with
//  no network the app cannot LIST anything, so it cannot offer what it already
//  holds. The catalog — the release and the manifest exactly as fetched — is
//  cached alongside, and an offline check falls back to it.
//
//  CONTENT-ADDRESSED. An image is stored under its own SHA-256, which is the
//  name the manifest gives it. Two releases sharing an unchanged image share
//  one file, a re-download of something already held is free, and there is no
//  separate index to fall out of step with the directory.
//
//  VERIFIED ON READ, ALWAYS. The file name is a claim about the contents, not
//  proof of them — a half-written file from a killed app has the right name and
//  the wrong bytes. Every read re-hashes and discards anything that does not
//  match. Cheap next to a flash, and the alternative is flashing a rocket from
//  a file nobody checked.
//

import CryptoKit
import Foundation

nonisolated struct FirmwareCache {

    private let dir: URL
    private let sha256: @Sendable (Data) -> String

    init(directory: URL,
         sha256: @escaping @Sendable (Data) -> String = FirmwareRepository.cryptoSHA256) {
        self.dir = directory
        self.sha256 = sha256
    }

    /// Internal Application Support, not Caches: iOS reclaims Caches under
    /// pressure, and firmware fetched at home specifically to survive a field
    /// with no signal is the last thing that should evaporate on the drive
    /// there. Excluded from backup — it is re-downloadable, and 7-8 MB of
    /// firmware has no business in someone's iCloud backup.
    static func defaultDirectory() -> URL {
        let base = FileManager.default
            .urls(for: .applicationSupportDirectory, in: .userDomainMask).first!
        var url = base.appendingPathComponent("Firmware")
        try? FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
        var values = URLResourceValues()
        values.isExcludedFromBackup = true
        try? url.setResourceValues(values)
        return url
    }

    private var imagesDir: URL { dir.appendingPathComponent("images") }
    private var releaseFile: URL { dir.appendingPathComponent("release.json") }
    private var manifestFile: URL { dir.appendingPathComponent("manifest.json") }

    // MARK: - the catalog

    /// Remember a fetched catalog so a later offline check can still list it.
    @discardableResult
    func putCatalog(_ catalog: FetchedCatalog) -> Bool {
        do {
            try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
            // Manifest first: a reader requires BOTH, so a crash between the
            // two leaves a stale-but-consistent catalog rather than a torn one.
            try Data(catalog.manifestJSON.utf8).write(to: manifestFile, options: .atomic)
            try Data(catalog.release.toListingJSON().utf8).write(to: releaseFile, options: .atomic)
            return true
        } catch {
            return false
        }
    }

    /// The last catalog stored, read back through the same parsers the network
    /// path uses so there is one codec rather than two that can disagree.
    func catalog() -> (FirmwareRelease, FirmwareManifest)? {
        guard let releaseData = try? Data(contentsOf: releaseFile),
              let manifestData = try? Data(contentsOf: manifestFile),
              let release = FirmwareReleaseLocator
                  .firmwareReleases(releaseData, includePrereleases: true).first,
              let manifest = FirmwareManifest.parse(manifestData)
        else { return nil }
        return (release, manifest)
    }

    // MARK: - the images

    private func fileFor(_ image: FirmwareImage) -> URL {
        imagesDir.appendingPathComponent(image.sha256)
    }

    /// Is this image held? A cheap check for a badge in a list — existence and
    /// length only, no hashing. `image(_:)` is what proves it before use.
    func hasImage(_ image: FirmwareImage) -> Bool {
        guard let attrs = try? FileManager.default
            .attributesOfItem(atPath: fileFor(image).path),
              let size = attrs[.size] as? Int else { return false }
        return Int64(size) == image.sizeBytes
    }

    /// Store bytes that have already been proven against the manifest.
    @discardableResult
    func putImage(_ image: FirmwareImage, _ bytes: Data) -> Bool {
        do {
            try FileManager.default.createDirectory(at: imagesDir,
                                                    withIntermediateDirectories: true)
            // .atomic writes beside and renames, so a killed app cannot leave a
            // half-file under a name that claims to be a whole one.
            try bytes.write(to: fileFor(image), options: .atomic)
            return true
        } catch {
            return false
        }
    }

    /// The cached bytes for `image`, or nil when they are absent or wrong.
    ///
    /// Anything that fails the check is DELETED rather than left to be found
    /// again: a file that did not match once will not match later, and leaving
    /// it invites a second attempt to trust it.
    func image(_ image: FirmwareImage) -> Data? {
        let f = fileFor(image)
        guard let bytes = try? Data(contentsOf: f) else { return nil }
        guard Int64(bytes.count) == image.sizeBytes,
              sha256(bytes).lowercased() == image.sha256 else {
            try? FileManager.default.removeItem(at: f)
            return nil
        }
        return bytes
    }

    /// Every SHA currently held, for a "what do I have offline" view.
    func heldShas() -> Set<String> {
        let names = (try? FileManager.default.contentsOfDirectory(atPath: imagesDir.path)) ?? []
        return Set(names.filter { !$0.hasSuffix(".part") })
    }

    /// Drop everything not in `keep`.
    ///
    /// Called with the current release's SHAs, so superseded images age out
    /// instead of filling the phone one release at a time. Nothing here is
    /// precious — anything dropped can be fetched again with a signal.
    func prune(keep: Set<String>) {
        let names = (try? FileManager.default.contentsOfDirectory(atPath: imagesDir.path)) ?? []
        for n in names where !keep.contains(n) {
            try? FileManager.default.removeItem(at: imagesDir.appendingPathComponent(n))
        }
    }

    func bytesHeld() -> Int64 {
        let names = (try? FileManager.default.contentsOfDirectory(atPath: imagesDir.path)) ?? []
        return names.reduce(Int64(0)) { acc, n in
            let p = imagesDir.appendingPathComponent(n).path
            let size = (try? FileManager.default.attributesOfItem(atPath: p))?[.size] as? Int
            return acc + Int64(size ?? 0)
        }
    }
}
