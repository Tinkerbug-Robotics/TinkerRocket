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

/// Saved on-pad sensor calibration (issue #132): gyro zero-rate bias (raw
/// LSB) + high-g accel bias (m/s²).  Like mag cal it's board-specific, so
/// it's tagged with the hardware ID it was captured on and only re-applied to
/// a matching board.  The low-g accelerometer is the cal reference and isn't
/// itself corrected, so there's nothing to store for it.
struct SensorCalData: Codable, Equatable {
    var gyroX: Int16
    var gyroY: Int16
    var gyroZ: Int16
    var hgX: Float            // high-g accel bias, m/s²
    var hgY: Float
    var hgZ: Float
    var calibratedOnUnitID: String
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
    var rateCapDps: Float = 60          // outer-loop angle→rate cap (deg/s)
    var kpAngle: Float = 2.0            // outer angle-loop P-gain (cascaded angle control)
    var guidanceEnabled: Bool = false
    var cameraType: UInt8 = 2          // 0=None, 1=GoPro, 2=RunCam
    /// IMU mounting orientation: 0xFF = auto (pad-gravity detect), 0..23 =
    /// manual board→rocket code. Manual fixes the roll clocking the control
    /// surfaces need — required for roll-controlled/guided flights with an
    /// off-axis board; optional for non-controlled flights.
    var imuOrientSetting: UInt8 = 0xFF

    // MARK: Servo
    var servoBias1: Int16 = 85
    var servoBias2: Int16 = 0
    var servoBias3: Int16 = 0
    var servoBias4: Int16 = 0
    var servoHz: Int16 = 333
    // #267: full-travel range. 1000us = fin -60deg, 2000us = +60deg (the servo's
    // mechanical limit). MUST match config.h SERVO_MIN_US/MAX_US + the firmware
    // setFinCalibration endpoints, or the commanded-vs-physical fin angle is wrong
    // (e.g. 1250/1750 would only reach +/-30deg at a +/-60deg command).
    var servoMinUs: Int16 = 1000
    var servoMaxUs: Int16 = 2000

    // MARK: PID
    // Defaults match the FC factory config (#267): kp=0.12 reproduces the flown
    // physical roll authority on the corrected 1:1 fin-angle scale; ki is a small
    // bench-tunable seed (SIL roll plant unvalidated). MinCmd/MaxCmd are the max
    // command deflection (+/-20deg), well inside the +/-60deg mechanical limit.
    // MUST stay in sync with config.h KP/KI/MIN_CMD/MAX_CMD -- this profile is
    // pushed on connect (ActiveRocketSyncer) and OVERRIDES the firmware defaults.
    var pidKp: Float = 0.12
    var pidKi: Float = 0.01
    var pidKd: Float = 0.0
    var pidMinCmd: Float = -20.0
    var pidMaxCmd: Float = 20.0
    var integralSepThreshold: Float = 40   // PID integral-separation anti-windup threshold (deg/s); 0 disables

    // MARK: Roll profile
    var rollWaypoints: [RollWaypoint] = []

    // MARK: Pyro (per-airframe; auto-pushed on connect — arming stays a
    // separate explicit action, never driven by the profile). New PCB
    // has 4 channels sharing a single ARM FET.
    var pyro1Enabled: Bool = false
    var pyro1TriggerMode: UInt8 = 0
    var pyro1TriggerValue: Float = 1.0
    var pyro2Enabled: Bool = false
    var pyro2TriggerMode: UInt8 = 0
    var pyro2TriggerValue: Float = 100.0
    var pyro3Enabled: Bool = false
    var pyro3TriggerMode: UInt8 = 0
    var pyro3TriggerValue: Float = 0.0
    var pyro4Enabled: Bool = false
    var pyro4TriggerMode: UInt8 = 0
    var pyro4TriggerValue: Float = 0.0

    // MARK: Recovery (issue #156) — descent profile for landing-point
    // prediction and drift-cast.  Stored per-airframe so different rockets
    // can have different chute setups; DriftCastView reads these (with a
    // global @AppStorage fallback when no profile is selected).
    /// Drogue / no-chute descent rate, fps.  Used above mainDeployAltAglFt.
    var drogueRateFps: Double = 60.0
    /// Main chute descent rate, fps.  Used below mainDeployAltAglFt.
    var mainRateFps: Double = 12.0
    /// Altitude AGL at which the main pyro fires (drogue→main transition).
    var mainDeployAltAglFt: Double = 700.0
    /// Quadratic drag coefficient k (1/m) for the coast-to-apogee ballistic
    /// propagation.  Default 5e-4 ≈ terminal velocity ~140 m/s, in the
    /// ballpark for typical 54–65 mm airframes.  Operator can refine in
    /// the rocket-edit UI.
    var ballisticDragK: Double = 5e-4

    // MARK: Calibration (per physical airframe / board)
    var magCal: MagCalData? = nil
    var sensorCal: SensorCalData? = nil

    /// A profile with all firmware factory defaults and the given name.
    static func makeDefault(name: String) -> RocketProfile {
        RocketProfile(name: name)
    }
}

// MARK: - Backward-compatible decoding
//
// The synthesized Decodable conformance treats missing JSON keys as a hard
// decode failure even when the property has a default — so adding any new
// field without a custom decoder would silently drop existing saved
// profiles in RocketProfileStore.load() (it does `try? decoder.decode(...)`
// and skips on nil).  This extension uses decodeIfPresent ?? default for
// every field so old profiles upgrade gracefully and future additions only
// need a single line here.
extension RocketProfile {
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let defaults = RocketProfile(name: "")

        id = try c.decodeIfPresent(UUID.self, forKey: .id) ?? UUID()
        name = try c.decode(String.self, forKey: .name)
        notes = try c.decodeIfPresent(String.self, forKey: .notes) ?? defaults.notes
        createdAt = try c.decodeIfPresent(Date.self, forKey: .createdAt) ?? defaults.createdAt
        updatedAt = try c.decodeIfPresent(Date.self, forKey: .updatedAt) ?? defaults.updatedAt
        lastUsedUnitID = try c.decodeIfPresent(String.self, forKey: .lastUsedUnitID)

        soundsEnabled = try c.decodeIfPresent(Bool.self, forKey: .soundsEnabled) ?? defaults.soundsEnabled
        servoControlEnabled = try c.decodeIfPresent(Bool.self, forKey: .servoControlEnabled) ?? defaults.servoControlEnabled
        gainScheduleEnabled = try c.decodeIfPresent(Bool.self, forKey: .gainScheduleEnabled) ?? defaults.gainScheduleEnabled
        useAngleControl = try c.decodeIfPresent(Bool.self, forKey: .useAngleControl) ?? defaults.useAngleControl
        rollDelayMs = try c.decodeIfPresent(UInt16.self, forKey: .rollDelayMs) ?? defaults.rollDelayMs
        rateCapDps = try c.decodeIfPresent(Float.self, forKey: .rateCapDps) ?? defaults.rateCapDps
        kpAngle = try c.decodeIfPresent(Float.self, forKey: .kpAngle) ?? defaults.kpAngle
        guidanceEnabled = try c.decodeIfPresent(Bool.self, forKey: .guidanceEnabled) ?? defaults.guidanceEnabled
        cameraType = try c.decodeIfPresent(UInt8.self, forKey: .cameraType) ?? defaults.cameraType
        imuOrientSetting = try c.decodeIfPresent(UInt8.self, forKey: .imuOrientSetting) ?? defaults.imuOrientSetting

        servoBias1 = try c.decodeIfPresent(Int16.self, forKey: .servoBias1) ?? defaults.servoBias1
        servoBias2 = try c.decodeIfPresent(Int16.self, forKey: .servoBias2) ?? defaults.servoBias2
        servoBias3 = try c.decodeIfPresent(Int16.self, forKey: .servoBias3) ?? defaults.servoBias3
        servoBias4 = try c.decodeIfPresent(Int16.self, forKey: .servoBias4) ?? defaults.servoBias4
        servoHz = try c.decodeIfPresent(Int16.self, forKey: .servoHz) ?? defaults.servoHz
        servoMinUs = try c.decodeIfPresent(Int16.self, forKey: .servoMinUs) ?? defaults.servoMinUs
        servoMaxUs = try c.decodeIfPresent(Int16.self, forKey: .servoMaxUs) ?? defaults.servoMaxUs

        pidKp = try c.decodeIfPresent(Float.self, forKey: .pidKp) ?? defaults.pidKp
        pidKi = try c.decodeIfPresent(Float.self, forKey: .pidKi) ?? defaults.pidKi
        pidKd = try c.decodeIfPresent(Float.self, forKey: .pidKd) ?? defaults.pidKd
        pidMinCmd = try c.decodeIfPresent(Float.self, forKey: .pidMinCmd) ?? defaults.pidMinCmd
        pidMaxCmd = try c.decodeIfPresent(Float.self, forKey: .pidMaxCmd) ?? defaults.pidMaxCmd
        integralSepThreshold = try c.decodeIfPresent(Float.self, forKey: .integralSepThreshold) ?? defaults.integralSepThreshold

        rollWaypoints = try c.decodeIfPresent([RollWaypoint].self, forKey: .rollWaypoints) ?? defaults.rollWaypoints

        pyro1Enabled = try c.decodeIfPresent(Bool.self, forKey: .pyro1Enabled) ?? defaults.pyro1Enabled
        pyro1TriggerMode = try c.decodeIfPresent(UInt8.self, forKey: .pyro1TriggerMode) ?? defaults.pyro1TriggerMode
        pyro1TriggerValue = try c.decodeIfPresent(Float.self, forKey: .pyro1TriggerValue) ?? defaults.pyro1TriggerValue
        pyro2Enabled = try c.decodeIfPresent(Bool.self, forKey: .pyro2Enabled) ?? defaults.pyro2Enabled
        pyro2TriggerMode = try c.decodeIfPresent(UInt8.self, forKey: .pyro2TriggerMode) ?? defaults.pyro2TriggerMode
        pyro2TriggerValue = try c.decodeIfPresent(Float.self, forKey: .pyro2TriggerValue) ?? defaults.pyro2TriggerValue
        pyro3Enabled = try c.decodeIfPresent(Bool.self, forKey: .pyro3Enabled) ?? defaults.pyro3Enabled
        pyro3TriggerMode = try c.decodeIfPresent(UInt8.self, forKey: .pyro3TriggerMode) ?? defaults.pyro3TriggerMode
        pyro3TriggerValue = try c.decodeIfPresent(Float.self, forKey: .pyro3TriggerValue) ?? defaults.pyro3TriggerValue
        pyro4Enabled = try c.decodeIfPresent(Bool.self, forKey: .pyro4Enabled) ?? defaults.pyro4Enabled
        pyro4TriggerMode = try c.decodeIfPresent(UInt8.self, forKey: .pyro4TriggerMode) ?? defaults.pyro4TriggerMode
        pyro4TriggerValue = try c.decodeIfPresent(Float.self, forKey: .pyro4TriggerValue) ?? defaults.pyro4TriggerValue

        drogueRateFps = try c.decodeIfPresent(Double.self, forKey: .drogueRateFps) ?? defaults.drogueRateFps
        mainRateFps = try c.decodeIfPresent(Double.self, forKey: .mainRateFps) ?? defaults.mainRateFps
        mainDeployAltAglFt = try c.decodeIfPresent(Double.self, forKey: .mainDeployAltAglFt) ?? defaults.mainDeployAltAglFt
        ballisticDragK = try c.decodeIfPresent(Double.self, forKey: .ballisticDragK) ?? defaults.ballisticDragK

        magCal = try c.decodeIfPresent(MagCalData.self, forKey: .magCal)
        sensorCal = try c.decodeIfPresent(SensorCalData.self, forKey: .sensorCal)
    }
}
