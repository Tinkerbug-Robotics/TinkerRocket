//
//  FleetFirmware.swift
//  TinkerRocketApp
//
//  Three rockets, one base station: which board and firmware each rocket on
//  the dashboard runs, and whether the units disagree on the commit. A rocket
//  heard only through the base station never reports either over LoRa, so its
//  values come from the known-device registry — what the rocket said the last
//  time it was on a direct Bluetooth link.
//

import Foundation

enum FirmwareStamp {

    /// The commit part of a version stamp. Stamps read
    /// `<sha>[-dirty]<board suffix>[bench suffix]+<build date>`, for example
    /// `ee4fe786-dirty-v9-bench+20261019-1830`; only the sha says which source
    /// the image came from, so that is all the mismatch check compares. nil
    /// for an empty stamp or the out computer's "unknown" (FC unreachable).
    static func gitSha(of version: String?) -> String? {
        guard let version else { return nil }
        let trimmed = version.trimmingCharacters(in: .whitespaces)
        guard !trimmed.isEmpty, trimmed.lowercased() != "unknown" else { return nil }
        let sha = trimmed.prefix { $0 != "-" && $0 != "+" }
        return sha.isEmpty ? nil : String(sha)
    }

    /// One unit's firmware for the mismatch check: who, and the stamp.
    struct Unit: Equatable {
        let label: String
        let version: String?
    }

    /// One line naming each commit and the units on it, or nil when every
    /// unit that reported a version is on the same commit (or fewer than two
    /// reported). Grouped by commit in first-seen order, so the base station
    /// (listed first by the caller) leads:
    /// "Firmware differs — ee4fe786: Base 1, Booster · ab12cd34: Dart".
    static func mismatchLine(_ units: [Unit]) -> String? {
        var order: [String] = []
        var labels: [String: [String]] = [:]
        for unit in units {
            guard let sha = gitSha(of: unit.version) else { continue }
            if labels[sha] == nil { order.append(sha) }
            if !(labels[sha]?.contains(unit.label) ?? false) {
                labels[sha, default: []].append(unit.label)
            }
        }
        guard order.count > 1 else { return nil }
        let groups = order.map { "\($0): \(labels[$0]!.joined(separator: ", "))" }
        return "Firmware differs — " + groups.joined(separator: " · ")
    }

    /// Board + firmware for a relayed rocket's section header, from its
    /// registry record, marked as Bluetooth-era knowledge:
    /// "Board v9 · fw ee4fe786 · last seen over Bluetooth 2 days ago".
    /// nil when the record holds neither. The board is the provisioned one,
    /// falling back to the image's own board suffix.
    static func identityLine(for rec: KnownDevice, now: Date = Date()) -> String? {
        var parts: [String] = []

        let oc = rec.ocBoard ?? rec.ocFirmware.flatMap(EspImage.boardSuffix(of:))
        let fc = rec.fcBoard ?? rec.fcFirmware.flatMap(EspImage.boardSuffix(of:))
        switch (oc, fc) {
        case let (o?, f?) where o.caseInsensitiveCompare(f) != .orderedSame:
            parts.append("OC \(o) / FC \(f)")
        case let (o?, _):
            parts.append("Board \(o)")
        case let (nil, f?):
            parts.append("Board \(f)")
        case (nil, nil):
            break
        }

        let ocSha = gitSha(of: rec.ocFirmware)
        let fcSha = gitSha(of: rec.fcFirmware)
        switch (ocSha, fcSha) {
        case let (o?, f?) where o != f:
            parts.append("OC fw \(o) / FC fw \(f)")
        case let (o?, _):
            parts.append("fw \(o)")
        case let (nil, f?):
            parts.append("FC fw \(f)")
        case (nil, nil):
            break
        }

        guard !parts.isEmpty else { return nil }
        if let seen = rec.firmwareSeen ?? rec.lastSeen {
            parts.append("last seen over Bluetooth \(relative(seen, now: now))")
        } else {
            parts.append("last seen over Bluetooth")
        }
        return parts.joined(separator: " · ")
    }

    private static func relative(_ date: Date, now: Date) -> String {
        let f = RelativeDateTimeFormatter()
        f.unitsStyle = .full
        f.locale = Locale(identifier: "en_US")
        return f.localizedString(for: date, relativeTo: now)
    }
}
