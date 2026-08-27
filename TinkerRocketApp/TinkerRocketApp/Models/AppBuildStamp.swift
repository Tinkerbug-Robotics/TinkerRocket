//
//  AppBuildStamp.swift
//  TinkerRocketApp
//
//  #974: which commit built this app.
//
//  The firmware stamps its commit into every flight record (`fw_git_sha`), so
//  any board traces back to its source.  The app could not: CFBundleShortVersionString
//  is a static "1.0" that has not moved in months, which made the app build the
//  one component of a flight nobody could identify afterwards.
//
//  The sha is written into the bundle at BUILD time by the "Stamp git sha into
//  the bundle" script phase, as a plain resource rather than an Info.plist key —
//  `ProcessInfoPlistFile` runs AFTER custom script phases, so a stamp written
//  into the plist is silently overwritten.  A committed Swift constant would not
//  work either: it can only ever name the commit BEFORE the one being built.
//

import Foundation

enum AppBuildStamp {

    /// Commit that built this bundle, e.g. `d7017c0` — or `d7017c0-dirty` when
    /// the tree had uncommitted changes, which means the binary corresponds to
    /// NO commit and should not be reported as if it did.
    static let gitSha: String = {
        guard let url = Bundle.main.url(forResource: "build-stamp", withExtension: "txt"),
              let raw = try? String(contentsOf: url, encoding: .utf8) else {
            // Older bundle, or a build that predates the stamp phase.  "unknown"
            // is honest; a fabricated value would be worse than none.
            return "unknown"
        }
        let trimmed = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        return trimmed.isEmpty ? "unknown" : trimmed
    }()

    /// Marketing version as shipped, for the row beside the firmware version.
    static let shortVersion: String =
        Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "?"

    /// `1.0 (d7017c0)` — what Settings shows.
    static var description: String { "\(shortVersion) (\(gitSha))" }
}
