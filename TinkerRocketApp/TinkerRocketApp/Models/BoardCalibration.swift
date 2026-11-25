//
//  BoardCalibration.swift
//  TinkerRocketApp
//
//  #1629: what one flight computer holds for calibration, as last read from
//  the board itself — never from a rocket profile, which describes what the
//  app saved, not what the board flies with. Android twin: BoardCalibration.kt
//  (core/session); the two must say the same things.
//

import Foundation

/// Each answer is nil until the board has said it: "not read yet" must never
/// render as "not calibrated". The flight computer answers three read-only
/// commands from its own storage, and republishes the same frames after every
/// calibration it stores, so the record tracks the board:
/// - `mag`: BLE 61 → 0xCA, APPLIED = a hard-iron calibration is stored
///   (NVS "mag_cal");
/// - `gyro`: BLE 63 → 0xCB, `valid` = a pad calibration's gyro bias is stored
///   (NVS "cal");
/// - `accel` and `highGAligned`: BLE 77 → 0xD1, the six-face offset is stored
///   (NVS "acc_cal") and the high-g's six-face alignment is stored ("hg_cal").
///
/// The high-g alignment is a refinement: the high-g flies on its bias without
/// one, so it is shown in Settings but never counted as "not calibrated".
struct BoardCalibration: Codable, Equatable {
    var mag: Bool?
    var accel: Bool?
    var gyro: Bool?
    var highGAligned: Bool?
    /// When the last answer was folded in, or nil when none.
    var readAt: Date?

    /// The calibrations a flight needs, in the order they are named.
    enum Item: String, CaseIterable {
        case magnetometer, accelerometer, gyro
    }

    /// The needed calibrations the board said it does not have.
    var missing: [Item] {
        var out: [Item] = []
        if mag == false { out.append(.magnetometer) }
        if accel == false { out.append(.accelerometer) }
        if gyro == false { out.append(.gyro) }
        return out
    }

    /// Every needed calibration is confirmed present.
    var fullyCalibrated: Bool { mag == true && accel == true && gyro == true }

    /// Nothing has been read from this board yet.
    var nothingRead: Bool { mag == nil && accel == nil && gyro == nil && highGAligned == nil }

    /// The dashboard line, or nil when nothing is known to be missing.
    var notCalibratedLine: String? {
        missing.isEmpty ? nil : "Not calibrated: " + missing.map(\.rawValue).joined(separator: ", ")
    }

    /// One line for My Devices.
    var summary: String {
        if let line = notCalibratedLine { return line }
        if fullyCalibrated { return "Calibrated" }
        if nothingRead { return "Calibration not read yet" }
        return "Calibration not fully read"
    }

    /// Fold a newer reading in: each reply answers one or two of the fields,
    /// so only the fields it answers change, and the time is the newer one.
    func merged(_ newer: BoardCalibration) -> BoardCalibration {
        BoardCalibration(mag: newer.mag ?? mag,
                         accel: newer.accel ?? accel,
                         gyro: newer.gyro ?? gyro,
                         highGAligned: newer.highGAligned ?? highGAligned,
                         readAt: newer.readAt ?? readAt)
    }

    /// A Settings row's value for a needed calibration.
    static func rowText(_ value: Bool?) -> String {
        switch value {
        case true?:  return "Calibrated"
        case false?: return "Not calibrated"
        case nil:    return "Not read yet"
        }
    }

    /// The Settings row's value for the high-g alignment.
    static func highGRowText(_ value: Bool?) -> String {
        switch value {
        case true?:  return "Aligned"
        case false?: return "Not aligned (flies on its bias)"
        case nil:    return "Not read yet"
        }
    }
}
