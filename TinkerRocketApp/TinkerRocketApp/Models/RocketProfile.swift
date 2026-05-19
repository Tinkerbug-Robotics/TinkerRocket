//
//  RocketProfile.swift
//  TinkerRocketApp
//
//  Per-rocket settings profile (issue #132).  Each airframe gets its own
//  profile of gains / biases / roll profile / camera / pyro / mag cal.
//  Profiles live in the app (not on the flight computer) so the same set
//  of settings can be applied to whichever physical computer is flying
//  that airframe.  The app is source-of-truth: on connect the active
//  profile is pushed to the rocket (see ActiveRocketSyncer).
//
//  Field defaults mirror RocketConfig (BLETypes.swift) and the firmware
//  factory defaults, so a freshly-created profile matches an out-of-the-box
//  rocket.  Keep them in sync if the firmware defaults change.
//

import Foundation

/// Segment mode for a roll-profile waypoint.  Matches the firmware enum
/// (ROLL_SEG_* in RocketComputerTypes.h): the mode applies to the segment
/// *starting* at this waypoint.
enum RollSegmentMode: UInt8, Codable, CaseIterable {
    case angle    = 0   // interpolate target roll angle to the next waypoint
    case nullRate = 1   // hold zero roll rate; the angle field is ignored
}

/// One roll-profile waypoint.  `id` gives SwiftUI stable identity in the
/// editor; it is persisted but otherwise carries no wire meaning.
struct RollWaypoint: Codable, Equatable, Identifiable {
    var id: UUID = UUID()
    var timeSeconds: Float
    var angleDeg: Float
    var mode: RollSegmentMode = .angle
}

/// Saved magnetometer hard-iron calibration, captured from a cal run and
/// re-pushed to the rocket on connect.  Tagged with the hardware ID it was
/// captured on: cal varies by physical airframe *and* by the specific
/// flight-computer board, so applying a cal taken on a different board would
/// make heading drift.  The syncer compares `calibratedOnUnitID` to the
/// connected device and warns (instead of pushing) on a mismatch.
struct MagCalData: Codable, Equatable {
    var offsetX: Int16        // hard-iron offset, raw IIS2MDC LSB (0.15 µT/LSB)
    var offsetY: Int16
    var offsetZ: Int16
    var fieldR_uT: Float      // fitted Earth-field magnitude when accepted
    var residualUT: Float     // fit RMS residual when accepted
    var calibratedOnUnitID: String   // hardware ID of the board cal ran on
    var calibratedAt: Date
}

struct RocketProfile: Codable, Equatable, Identifiable {
    // MARK: Identity / meta
    var id: UUID = UUID()
    var name: String
    var notes: String = ""
    var createdAt: Date = Date()
    var updatedAt: Date = Date()
    /// Hardware ID of the last flight computer this profile was applied to.
    /// Drives a soft pre-select so connecting a known rocket lands on the
    /// profile last flown on it.  The user always confirms — no auto-apply
    /// without a selected active profile.
    var lastUsedUnitID: String? = nil

    // MARK: Rocket toggles
    var soundsEnabled: Bool = false
    var servoControlEnabled: Bool = true
    var gainScheduleEnabled: Bool = true
    var useAngleControl: Bool = false
    var rollDelayMs: UInt16 = 0
    var guidanceEnabled: Bool = false
    var cameraType: UInt8 = 2          // 0=None, 1=GoPro, 2=RunCam

    // MARK: Servo
    var servoBias1: Int16 = 85
    var servoBias2: Int16 = 0
    var servoBias3: Int16 = 0
    var servoBias4: Int16 = 0
    var servoHz: Int16 = 333
    var servoMinUs: Int16 = 1250
    var servoMaxUs: Int16 = 1750

    // MARK: PID
    var pidKp: Float = 0.08
    var pidKi: Float = 0.005
    var pidKd: Float = 0.003
    var pidMinCmd: Float = -10.0
    var pidMaxCmd: Float = 10.0

    // MARK: Roll profile
    var rollWaypoints: [RollWaypoint] = []

    // MARK: Pyro (per-airframe; auto-pushed on connect — arming stays a
    // separate explicit action, never driven by the profile).
    var pyro1Enabled: Bool = false
    var pyro1TriggerMode: UInt8 = 0
    var pyro1TriggerValue: Float = 1.0
    var pyro2Enabled: Bool = false
    var pyro2TriggerMode: UInt8 = 0
    var pyro2TriggerValue: Float = 100.0

    // MARK: Mag cal (per physical airframe)
    var magCal: MagCalData? = nil

    /// A profile with all firmware factory defaults and the given name.
    static func makeDefault(name: String) -> RocketProfile {
        RocketProfile(name: name)
    }
}
