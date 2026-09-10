//
//  FirmwareManifest.swift
//  TinkerRocketApp
//
//  The firmware manifest published beside the images on an `fw-v*` release
//  (#773 step 4; the release itself is #773 step 3). Port of core/protocol
//  FirmwareManifest.kt — keep the two in step.
//
//  Written by `tinkerrocket-idf/tools/image_info.py`, which builds it by
//  READING each image's esp_app_desc_t rather than from the build matrix that
//  produced it. So every field here is the image's own account of itself, and
//  the app can check a downloaded file against it byte for byte.
//

import Foundation

nonisolated struct FirmwareImage: Equatable {
    let file: String
    let project: String
    let version: String
    let chipId: Int
    let chip: String
    let sizeBytes: Int64
    let sha256: String
    /// Board revision the image asserts, or nil when the build carries none.
    let board: String?
    let idfVersion: String
    let buildDate: String

    /// A human line for a catalog row: what it is and when it was built.
    var summary: String {
        var s = project
        if let b = board { s += " · \(b)" }
        s += " · \(chip)"
        if !buildDate.isEmpty { s += " · \(buildDate)" }
        return s
    }
}

nonisolated struct FirmwareManifest: Equatable {
    let manifestVersion: Int
    let tag: String
    let images: [FirmwareImage]

    /// The version this app understands. A newer manifest is refused rather
    /// than half-read: a field it does not know could be the one that says an
    /// image is unsafe for this unit.
    static let supportedVersion = 1

    /// Parse a manifest, or nil if it is not one we can act on.
    ///
    /// Deliberately strict about the things that decide whether a file may be
    /// flashed — an image with no project, no sha or no size is dropped rather
    /// than shown, because a catalog row the app cannot verify is worse than a
    /// missing one.
    static func parse(_ data: Data) -> FirmwareManifest? {
        guard let root = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
              let version = num(root["manifest_version"]).map({ Int($0) }),
              version == supportedVersion,
              let tag = str(root["tag"]),
              let rawImages = root["images"] as? [[String: Any]]
        else { return nil }

        let images: [FirmwareImage] = rawImages.compactMap { o in
            guard let file = str(o["file"]),
                  let project = str(o["project"]),
                  let sha = str(o["sha256"])?.lowercased(), sha.count == 64,
                  let size = num(o["size"]), size > 0
            else { return nil }
            return FirmwareImage(
                file: file,
                project: project,
                version: str(o["version"]) ?? "",
                chipId: num(o["chip_id"]).map { Int($0) } ?? -1,
                chip: str(o["chip"]) ?? "",
                sizeBytes: size,
                sha256: sha,
                board: str(o["board"])?.lowercased(),
                idfVersion: str(o["idf_version"]) ?? "",
                buildDate: str(o["build_date"]) ?? ""
            )
        }
        return images.isEmpty ? nil : FirmwareManifest(manifestVersion: version,
                                                       tag: tag, images: images)
    }

    static func parse(_ text: String) -> FirmwareManifest? { parse(Data(text.utf8)) }

    private static func str(_ v: Any?) -> String? {
        guard let s = v as? String, !s.isEmpty else { return nil }
        return s
    }
    private static func num(_ v: Any?) -> Int64? {
        if let n = v as? NSNumber { return n.int64Value }
        if let s = v as? String, let d = Double(s) { return Int64(d) }
        return nil
    }
}

/// Which images in a manifest belong on the unit in front of you.
///
/// The hard filter is `project`, exactly as in `EspImage.check` — it is the
/// only field that separates a base station from an out computer, both
/// ESP32-S3 with byte-identical app slots.
///
/// Board is a RANKING, not a filter:
///   1. an image whose board matches what this board says it is
///   2. an image with no board suffix at all, which applies everywhere
///   3. everything else for the same project
///
/// A non-matching board is offered LAST rather than hidden, because hiding it
/// would leave an operator with a board provisioned wrongly, or not at all,
/// unable to flash anything — and the flash button still refuses a wrong
/// project, which is the case that actually matters.
nonisolated enum FirmwareCatalog {

    static func forUnit(_ manifest: FirmwareManifest,
                        expectedProject: String,
                        provisionedBoard: String? = nil) -> [FirmwareImage] {
        let want = provisionedBoard?
            .trimmingCharacters(in: .whitespaces).lowercased()
            .nilIfEmptyManifest
        func rank(_ i: FirmwareImage) -> Int {
            if let w = want, i.board == w { return 0 }
            if i.board == nil { return 1 }
            return 2
        }
        return manifest.images
            .filter { $0.project == expectedProject }
            .sorted { a, b in
                let (ra, rb) = (rank(a), rank(b))
                if ra != rb { return ra < rb }
                if (a.board ?? "") != (b.board ?? "") { return (a.board ?? "") < (b.board ?? "") }
                return a.file < b.file
            }
    }

    /// The single image to offer by default, or nil when nothing in the
    /// manifest is for this unit at all.
    ///
    /// Returns nil rather than a guess when the board is known and no image
    /// matches it: offering the wrong revision as the default is how a wrong
    /// flash happens, and the full list is still there for a deliberate choice.
    static func best(_ manifest: FirmwareManifest,
                     expectedProject: String,
                     provisionedBoard: String? = nil) -> FirmwareImage? {
        let want = provisionedBoard?
            .trimmingCharacters(in: .whitespaces).lowercased()
            .nilIfEmptyManifest
        let candidates = forUnit(manifest, expectedProject: expectedProject,
                                 provisionedBoard: want)
        guard let head = candidates.first else { return nil }
        guard let w = want else { return head }
        return (head.board == w || head.board == nil) ? head : nil
    }
}

nonisolated private extension String {
    var nilIfEmptyManifest: String? { isEmpty ? nil : self }
}
