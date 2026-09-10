//
//  FirmwareRelease.swift
//  TinkerRocketApp
//
//  Finding the firmware release on GitHub, and fetching from it (#773 step 4).
//  Port of core/protocol FirmwareRelease.kt — keep the two in step.
//
//  WHERE THE BINARIES LIVE: GitHub Releases on this repository, which is
//  public, so an anonymous client can read both the API and the assets — no
//  token, no account. Verified 2026-09-10 against a real release asset.
//
//  WHY NOT `/releases/latest`: that endpoint returns the newest release of ANY
//  kind, and this repo also tags board and Android releases. At the time of
//  writing it answers `rocket-computer-mini-v1.0.1` — a gerber zip. Firmware
//  has to be found by tag prefix, which is what this does.
//
//  RATE LIMIT: listing releases is ONE unauthenticated API call, and the
//  anonymous budget is 60/hour per IP. Asset downloads do not touch
//  api.github.com at all — they come from the releases/download host — so
//  downloading eleven images costs one API call, not twelve.
//

import CryptoKit
import Foundation

nonisolated struct FirmwareRelease: Equatable {
    let tag: String
    let isPrerelease: Bool
    /// Asset name to download URL.
    let assets: [String: String]

    static let manifestAsset = "manifest.json"

    var manifestURL: String? { assets[FirmwareRelease.manifestAsset] }

    func url(for image: FirmwareImage) -> String? { assets[image.file] }

    /// This release in the shape `FirmwareReleaseLocator.firmwareReleases`
    /// parses — a one-element `/releases` response.
    ///
    /// A cache writes this and reads it back through the same parser the
    /// network path uses, so there is one codec for release JSON rather than
    /// two that can disagree. Round-tripped by test.
    func toListingJSON() -> String {
        let assetsJSON = assets.map { name, url in
            "{\"name\":\(Self.quote(name)),\"browser_download_url\":\(Self.quote(url))}"
        }.joined(separator: ",")
        return "[{\"tag_name\":\(Self.quote(tag)),\"prerelease\":\(isPrerelease),"
            + "\"assets\":[\(assetsJSON)]}]"
    }

    private static func quote(_ s: String) -> String {
        var out = "\""
        for c in s.unicodeScalars {
            switch c {
            case "\"": out += "\\\""
            case "\\": out += "\\\\"
            case "\n": out += "\\n"
            case "\r": out += "\\r"
            case "\t": out += "\\t"
            default:
                if c.value < 0x20 { out += String(format: "\\u%04x", c.value) }
                else { out.unicodeScalars.append(c) }
            }
        }
        return out + "\""
    }
}

nonisolated enum FirmwareReleaseLocator {
    /// Tag scheme from `.github/workflows/firmware-release.yml`.
    static let tagPrefix = "fw-v"

    static let releasesURL =
        "https://api.github.com/repos/Tinkerbug-Robotics/TinkerRocket/releases?per_page=30"

    /// Every firmware release in a `/releases` response, newest first.
    ///
    /// Ordered by the version in the tag rather than by the API's own
    /// ordering. The API sorts by creation date, which is usually the same
    /// thing and occasionally is not — a re-cut tag, or a release edited
    /// later, moves in that ordering and would silently become "newest".
    ///
    /// A release with no `manifest.json` asset is skipped: it is either from
    /// before the manifest existed or a failed publish, and either way there
    /// is nothing the app can act on.
    static func firmwareReleases(_ data: Data,
                                 includePrereleases: Bool = false) -> [FirmwareRelease] {
        guard let arr = (try? JSONSerialization.jsonObject(with: data)) as? [[String: Any]]
        else { return [] }

        let found: [FirmwareRelease] = arr.compactMap { o in
            guard let tag = o["tag_name"] as? String, tag.hasPrefix(tagPrefix)
            else { return nil }
            let pre = (o["prerelease"] as? Bool) ?? false
            if pre && !includePrereleases { return nil }
            var assets: [String: String] = [:]
            for a in (o["assets"] as? [[String: Any]]) ?? [] {
                guard let name = a["name"] as? String, !name.isEmpty,
                      let url = a["browser_download_url"] as? String, !url.isEmpty
                else { continue }
                assets[name] = url
            }
            guard assets[FirmwareRelease.manifestAsset] != nil else { return nil }
            return FirmwareRelease(tag: tag, isPrerelease: pre, assets: assets)
        }
        return found.sorted { compareKeys(versionKey($0.tag), versionKey($1.tag)) > 0 }
    }

    static func firmwareReleases(_ text: String,
                                 includePrereleases: Bool = false) -> [FirmwareRelease] {
        firmwareReleases(Data(text.utf8), includePrereleases: includePrereleases)
    }

    static func newest(_ data: Data, includePrereleases: Bool = false) -> FirmwareRelease? {
        firmwareReleases(data, includePrereleases: includePrereleases).first
    }

    static func newest(_ text: String, includePrereleases: Bool = false) -> FirmwareRelease? {
        newest(Data(text.utf8), includePrereleases: includePrereleases)
    }

    /// Lexicographic compare of two equal-length version keys.
    static func compareKeys(_ a: [Int], _ b: [Int]) -> Int {
        for i in a.indices where a[i] != b[i] { return a[i] < b[i] ? -1 : 1 }
        return 0
    }

    /// Comparable key from a `fw-v1.2.3` tag. Non-numeric tails sort below an
    /// otherwise-equal release, so `fw-v1.0.0` beats `fw-v1.0.0-rc1`.
    static func versionKey(_ tag: String) -> [Int] {
        let body = String(tag.dropFirst(tag.hasPrefix(tagPrefix) ? tagPrefix.count : 0))
        let core = body.prefix { $0.isNumber || $0 == "." }
        let parts = core.split(separator: ".", omittingEmptySubsequences: false)
            .compactMap { Int($0) }
        let padded = Array((parts + [0, 0, 0]).prefix(3))
        // A suffix ("-rc1", "-dryrun") ranks below the bare version.
        return padded + [body.count > core.count ? 0 : 1]
    }
}

nonisolated struct FetchedCatalog {
    let release: FirmwareRelease
    let manifest: FirmwareManifest
    /// Exactly the bytes `manifest` was parsed from, so a cache stores what it
    /// actually verified rather than a re-serialization that could differ.
    let manifestJSON: String
}

/// What a download attempt produced.
nonisolated enum FirmwareFetch: Equatable {
    case ok(Data)
    /// The transport failed — no bytes, or the host said no.
    case unreachable(String)
    /// Bytes arrived and are NOT what the manifest said they would be. Never
    /// flashed, never cached: a truncated download and a tampered file look
    /// identical here, and neither belongs on a flight computer.
    case corrupt(String)
}

/// Fetching, with the network injected so the policy is testable without one.
nonisolated struct FirmwareRepository {
    /// Returns the body, or nil when the request failed.
    let fetch: @Sendable (String) async -> Data?
    /// Lowercase hex SHA-256. Injected only so tests can force a mismatch.
    let sha256: @Sendable (Data) -> String

    init(fetch: @escaping @Sendable (String) async -> Data?,
         sha256: @escaping @Sendable (Data) -> String = FirmwareRepository.cryptoSHA256) {
        self.fetch = fetch
        self.sha256 = sha256
    }

    static let cryptoSHA256: @Sendable (Data) -> String = { data in
        SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
    }

    /// The default: URLSession, no credentials of any kind.
    static let urlSessionFetch: @Sendable (String) async -> Data? = { urlString in
        guard let url = URL(string: urlString) else { return nil }
        var req = URLRequest(url: url)
        req.timeoutInterval = 30
        // Asking for the v3 media type keeps the response shape pinned even if
        // GitHub's default changes; no Authorization header — the repo is
        // public and an anonymous read is what a field phone can always do.
        req.setValue("application/vnd.github+json", forHTTPHeaderField: "Accept")
        guard let (data, resp) = try? await URLSession.shared.data(for: req),
              let http = resp as? HTTPURLResponse, (200..<300).contains(http.statusCode)
        else { return nil }
        return data
    }

    func latestManifest(includePrereleases: Bool = false) async -> FetchedCatalog? {
        guard let listing = await fetch(FirmwareReleaseLocator.releasesURL),
              let release = FirmwareReleaseLocator.newest(
                  listing, includePrereleases: includePrereleases),
              let url = release.manifestURL,
              let body = await fetch(url),
              let manifest = FirmwareManifest.parse(body)
        else { return nil }
        return FetchedCatalog(release: release, manifest: manifest,
                              manifestJSON: String(decoding: body, as: UTF8.self))
    }

    /// Download one image and prove it is the one the manifest described.
    ///
    /// Size is checked before the hash purely so a truncated download reports
    /// as truncated rather than as a hash mismatch — same refusal, clearer
    /// reason.
    func download(release: FirmwareRelease, image: FirmwareImage) async -> FirmwareFetch {
        guard let url = release.url(for: image) else {
            return .unreachable("\(image.file) is not in release \(release.tag)")
        }
        guard let bytes = await fetch(url) else {
            return .unreachable("could not download \(image.file)")
        }
        guard Int64(bytes.count) == image.sizeBytes else {
            return .corrupt(
                "\(image.file) is \(bytes.count) bytes, manifest says \(image.sizeBytes)")
        }
        guard sha256(bytes).lowercased() == image.sha256 else {
            return .corrupt("\(image.file) checksum does not match the manifest")
        }
        return .ok(bytes)
    }
}
