//
//  EspImage.swift
//  TinkerRocketApp
//
//  ESP-IDF application-image header, and whether a picked .bin belongs on the
//  unit the user is about to flash (#773). Port of core/protocol EspImage.kt —
//  keep the two in step.
//
//  Today neither app looks at the image at all: the file is chosen by hand, and
//  TR_OTA on the far end validates only size and SHA-256. The out computer and
//  the base station are both ESP32-S3 with byte-identical app slots, so a
//  base-station image pushed to an out computer is accepted by both ends and
//  only rollback catches it, and only if the wrong image fails to boot.
//
//  The image already carries the answer: ESP-IDF writes an esp_app_desc_t at a
//  fixed offset holding the CMake project() name, so "is this the right program
//  for this box" is a string comparison. No manifest, no firmware change.
//
//  Layout (esp_image_format.h), verified against real built images:
//    0   esp_image_header_t, 24 bytes; [0] magic 0xE9, [12...13] chip_id (LE),
//        [15...16] min_chip_rev_full, [17...18] max_chip_rev_full
//    24  esp_image_segment_header_t, 8 bytes
//    32  esp_app_desc_t: magic 0xABCD5432, +16 version[32], +48 project_name[32],
//        +80 time[16], +96 date[16], +112 idf_ver[32]
//

import Foundation

nonisolated struct EspAppImage: Equatable {
    let chipId: Int
    let minChipRevFull: Int
    let maxChipRevFull: Int
    let version: String
    let projectName: String
    let buildTime: String
    let buildDate: String
    let idfVersion: String

    static let chipNames: [Int: String] = [
        0x0000: "ESP32", 0x0002: "ESP32-S2", 0x0005: "ESP32-C3",
        0x0009: "ESP32-S3", 0x000C: "ESP32-C2", 0x000D: "ESP32-C6",
        0x0010: "ESP32-H2", 0x0012: "ESP32-P4",
    ]

    var chipName: String {
        EspAppImage.chipNames[chipId] ?? String(format: "chip 0x%04X", chipId)
    }

    /// Board revision the image asserts, from the `-v9` in a version string
    /// like `537dc3ff-dirty-v9+20260909-1835`. Nil when the build carries no
    /// suffix (the mini's single-MCU project does not).
    ///
    /// This is what the image CLAIMS, not what the board is — a wrongly flashed
    /// board reports the wrong revision forever. Good enough to warn on, never
    /// to decide on.
    var boardSuffix: String? { EspImage.boardSuffix(of: version) }
}

nonisolated enum EspImageVerdict: Equatable {
    case ok(EspAppImage)
    /// Flashable, but say this first.
    case warn(EspAppImage, String)
    /// Do not flash. The reason is written for the operator, not the log.
    case refuse(EspAppImage?, String)

    var image: EspAppImage? {
        switch self {
        case .ok(let i): return i
        case .warn(let i, _): return i
        case .refuse(let i, _): return i
        }
    }
    var isRefusal: Bool { if case .refuse = self { return true }; return false }
}

nonisolated private extension String {
    var nilIfEmpty: String? { isEmpty ? nil : self }
}

nonisolated enum EspImage {
    static let projectFC = "flight_computer"
    static let projectOC = "out_computer"
    static let projectBS = "base_station"
    static let projectMini = "rocket_computer_mini"

    private static let imageMagic: UInt8 = 0xE9
    private static let appDescMagic: UInt32 = 0xABCD_5432
    private static let appDescOffset = 32
    private static let minLength = 32 + 256

    /// Shared with `image_info.py`'s `board_of` and Android's `BOARD_SUFFIX_RE`.
    ///
    /// The letter is NOT always `v`. Each project's CMakeLists stamps its own
    /// TR_BOARD_SUFFIX, and three shapes are in use: `-v7/-v8/-v9` (flight
    /// computer, out computer, base station), `-m1` (the flight and out
    /// computer builds for the rocket-computer-mini), and `-b1` (the mini's
    /// own single-MCU project, when TR_MINI_BOARD is set).
    ///
    /// This matched only `-v` until 2026-09-10, so every `-m1` image parsed as
    /// "no board" — read by the catalog as "applies everywhere" when it is the
    /// one image that applies to exactly one board.
    static let boardSuffixPattern = "-[vmb][0-9]+([+\\-]|$)"

    static func boardSuffix(of version: String) -> String? {
        guard let r = version.range(of: EspImage.boardSuffixPattern,
                                    options: [.regularExpression, .caseInsensitive])
        else { return nil }
        var s = String(version[r]).dropFirst()             // drop the leading "-"
        if let last = s.last, last == "+" || last == "-" { s = s.dropLast() }
        return s.lowercased()
    }

    private static func u16(_ b: [UInt8], _ o: Int) -> Int {
        Int(b[o]) | (Int(b[o + 1]) << 8)
    }
    private static func u32(_ b: [UInt8], _ o: Int) -> UInt32 {
        UInt32(b[o]) | (UInt32(b[o + 1]) << 8) | (UInt32(b[o + 2]) << 16) | (UInt32(b[o + 3]) << 24)
    }
    /// NUL-terminated fixed-width ASCII field; anything unprintable ends it.
    private static func str(_ b: [UInt8], _ o: Int, _ len: Int) -> String {
        var out = ""
        for i in o..<min(o + len, b.count) {
            let c = b[i]
            if c == 0 || c < 0x20 || c > 0x7E { break }
            out.append(Character(UnicodeScalar(c)))
        }
        return out
    }

    /// Parse an ESP-IDF app image, or nil if this is not one.
    static func parse(_ data: Data) -> EspAppImage? {
        guard data.count >= minLength else { return nil }
        let b = [UInt8](data.prefix(minLength))
        guard b[0] == imageMagic else { return nil }
        guard u32(b, appDescOffset) == appDescMagic else { return nil }
        return EspAppImage(
            chipId: u16(b, 12),
            minChipRevFull: u16(b, 15),
            maxChipRevFull: u16(b, 17),
            version: str(b, appDescOffset + 16, 32),
            projectName: str(b, appDescOffset + 48, 32),
            buildTime: str(b, appDescOffset + 80, 16),
            buildDate: str(b, appDescOffset + 96, 16),
            idfVersion: str(b, appDescOffset + 112, 32)
        )
    }

    /// Decide whether `data` may be flashed to a unit running `expectedProject`.
    ///
    /// `expectedChipId` and `runningVersion` are advisory: the flight computer
    /// is an ESP32-P4 on the V9 board and an ESP32-S3 on the mini, so a chip
    /// mismatch warns rather than refuses, and the running version is the box's
    /// own claim about itself.
    /// #773 step 2: `provisionedBoard` is what the BOARD says it is, read from
    /// its own NVS and untouched by an OTA. When present it WINS over
    /// `runningVersion`, because the running version is the image's claim and a
    /// wrongly flashed board repeats that wrong claim forever. The fallback is
    /// kept for a board that has never been provisioned; the warning says which
    /// source it used, so a fallback comparison is never read as authoritative.
    static func check(_ data: Data,
                      expectedProject: String,
                      expectedChipId: Int? = nil,
                      runningVersion: String? = nil,
                      provisionedBoard: String? = nil) -> EspImageVerdict {
        guard let img = parse(data) else {
            return .refuse(nil, "This file is not an ESP-IDF firmware image. "
                              + "Pick the .bin produced by the build, not a .zip, "
                              + "an .elf or a log.")
        }
        if img.projectName != expectedProject {
            let what = img.projectName.isEmpty ? "an unnamed program"
                                               : "\"\(img.projectName)\""
            return .refuse(img, "This image is \(what), but you are updating "
                              + "\"\(expectedProject)\". Flashing it would leave "
                              + "that unit running the wrong program.")
        }
        var warnings: [String] = []
        if let want = expectedChipId, img.chipId != want {
            let wantName = EspAppImage.chipNames[want] ?? String(format: "chip 0x%04X", want)
            warnings.append("built for \(img.chipName), but this unit is normally \(wantName)")
        }
        let provisioned = provisionedBoard?
            .trimmingCharacters(in: .whitespaces).lowercased()
            .nilIfEmpty
        let fromVersion = runningVersion.flatMap(boardSuffix(of:))
        if let picked = img.boardSuffix {
            if let p = provisioned, p != picked {
                warnings.append("built for board \(picked), but this board is provisioned as \(p)")
            } else if provisioned == nil, let v = fromVersion, v != picked {
                warnings.append("built for board \(picked), but this unit's firmware "
                              + "reports \(v) (board not provisioned, so this is the "
                              + "image's own claim)")
            }
        }
        return warnings.isEmpty ? .ok(img) : .warn(img, warnings.joined(separator: "; "))
    }
}
