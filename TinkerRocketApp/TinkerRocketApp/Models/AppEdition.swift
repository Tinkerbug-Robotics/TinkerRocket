//
//  AppEdition.swift
//  TinkerRocketApp
//
//  Which app this build is, and what it shows for the rocket it talks to.
//  Android twin: AppEdition.kt — keep the two in step.
//
//  One codebase, two editions (docs/app-editions.md):
//
//    store  The App Store app, TinkerbugRobotics.TinkerRocket, "Tinker Rocket".
//           Supports the Tinker-Beetle only, the first flight computer
//           released. Built by the Store configuration (the "TinkerRocketApp
//           Store" scheme), which sets TR_STORE.
//    dev    Every board. Debug and Release, TinkerbugRobotics.TinkerRocketApp,
//           "Tinker Rocket Dev": the app already on the bench phones, so its
//           flights and settings stay where they are.
//
//  A feature here follows what the BOARD has, not just the edition. The board
//  files say it: board_m2.h has "no servos -- no fin control" and no camera
//  channel; board_v10.h has three servo outputs and a GoPro/RunCam port. So the
//  dev edition hides the fin and camera screens too while a Beetle is the
//  rocket in front of it, and shows them when it cannot tell.
//
//  Every rule is a pure function of (edition, board) so the store's rules are
//  unit-tested from the dev build the tests run in.
//

import Foundation

nonisolated enum AppEdition: Equatable {
    case store
    case dev

    static let current: AppEdition = {
        #if TR_STORE
        return .store
        #else
        return .dev
        #endif
    }()
}

/// The family a board revision names, from its letter
/// (board_identity::revCodeToString): "M2 (Tinker-Beetle V2)", "M3", "V10".
nonisolated enum BoardFamily: Equatable {
    /// The M family: the Tinker-Beetle (M2 = Beetle V2, M3 = Beetle V3).
    case beetle
    /// The V family: the Mantis (V10, V11) and the V boards before it (V9).
    case mantis

    /// nil when the text names no family this app knows: empty, unprovisioned,
    /// or a letter without a revision number after it.
    init?(revision: String?) {
        guard let r = revision?.trimmingCharacters(in: .whitespaces).uppercased(),
              let letter = r.first,
              r.dropFirst().first?.isNumber == true
        else { return nil }
        switch letter {
        case "M": self = .beetle
        case "V": self = .mantis
        default: return nil
        }
    }

    /// The rocket's family from everything the link says about it, best
    /// source first: what each board was PROVISIONED as (#773 step 2), then
    /// the board its running firmware was built for — every board provisioned
    /// before #773 reports no revision, so that fallback is the ordinary case.
    static func of(fcBoard: String?, ocBoard: String?,
                   fcFirmware: String?, ocFirmware: String?) -> BoardFamily? {
        BoardFamily(revision: fcBoard)
            ?? BoardFamily(revision: ocBoard)
            ?? BoardFamily(revision: fcFirmware.flatMap(EspImage.boardSuffix(of:)))
            ?? BoardFamily(revision: ocFirmware.flatMap(EspImage.boardSuffix(of:)))
    }
}

/// What this build shows for the rocket in front of it.
nonisolated struct AppFeatures: Equatable {
    let edition: AppEdition
    /// nil: nothing connected, a base station relay, or a board that has not
    /// said what it is.
    let board: BoardFamily?

    init(edition: AppEdition = .current, board: BoardFamily?) {
        self.edition = edition
        self.board = board
    }

    /// The fin servos and everything that only reaches the fins through them:
    /// Settings' Control tab (servo control, PID gains, servo calibration,
    /// control mode, roll control, guidance, fin layout), Servo Test, Ground
    /// Test and Drift Cast's "Send to Unit" (an aim point guidance flies).
    /// The Beetle has no servo outputs.
    var finControl: Bool {
        switch edition {
        case .store: return false
        case .dev:   return board != .beetle
        }
    }

    /// The camera (GoPro / RunCam): Settings' Camera tab, the Start Camera
    /// control, the Status card's Camera badge, the camera in the rocket's
    /// summary line, and the pre-flight "camera recording" check.
    /// The Beetle has no camera port.
    var camera: Bool {
        switch edition {
        case .store: return false
        case .dev:   return board != .beetle
        }
    }

    /// Whether this edition supports the rocket at all. The store edition
    /// supports the Beetle only; an unknown board gets the benefit of the
    /// doubt, because a Beetle reads as unknown until its identity arrives.
    var boardSupported: Bool {
        guard edition == .store, let board else { return true }
        return board == .beetle
    }

    /// Whether to offer a release image in the firmware catalog. The store
    /// edition offers rocket images for the Beetle only (plus images built
    /// for every board, which carry no board suffix); base-station images are
    /// all offered, in both editions.
    static func offersFirmware(_ image: FirmwareImage,
                               edition: AppEdition = .current) -> Bool {
        guard edition == .store, image.project != EspImage.projectBS,
              let board = image.board
        else { return true }
        return BoardFamily(revision: board) == .beetle
    }
}

extension BLEDevice {
    /// The rocket's board family, from what this link has heard. nil on a
    /// base station: its own firmware carries a board suffix too ("-v5" on a
    /// Tinker-Base V2), which names a base-station board, not a rocket's.
    var boardFamily: BoardFamily? {
        guard !isBaseStation else { return nil }
        return BoardFamily.of(fcBoard: fcBoardRev, ocBoard: ocBoardRev,
                              fcFirmware: fcFirmwareVersion, ocFirmware: firmwareVersion)
    }

    /// What this build shows for this rocket.
    var features: AppFeatures { AppFeatures(board: boardFamily) }
}
