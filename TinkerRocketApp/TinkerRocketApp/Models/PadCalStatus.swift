//
//  PadCalStatus.swift
//  TinkerRocketApp
//
//  Decoded form of the FC's PadCalStatusData frame (#1600 phase 5): what a pad
//  calibration is doing — the gyro/accel window, the GNSS pad survey and the
//  ionosphere the raw-GNSS filter uses. Arrives on the file_ops characteristic
//  behind a 0xD0 discriminator, once a second while the rocket is on the
//  ground (calibrating or not) and at each change, so a rocket that has sent
//  one runs firmware that knows command 75. The gyro/accel result itself
//  still comes as the 0xCB SensorCalStatus frame when the window closes.
//  Wire layout mirrors PadCalStatusData in RocketComputerTypes.h — keep both
//  in sync.
//
//  Wire (40 bytes, little-endian, packed):
//    [0..3]   u32 time_us
//    [4]      u8  cal_state       PAD_CAL_*: idle, running, done, rejected, refused
//    [5]      u8  cal_progress    % of the gyro/accel window gone (100 once closed)
//    [6..7]   u16 cal_gravity_mg  |gravity| the window measured (0: none yet)
//    [8]      u8  sats_tracked    GPS, Galileo, BeiDou with a pseudorange, last epoch
//    [9]      u8  sats_no_eph       of those, no broadcast ephemeris yet
//    [10]     u8  sats_settling     locked under 50 s, not yet surveyed
//    [11]     u8  sats_corrected    with a pad correction ready
//    [12..13] u16 corr_rms_cm     the corrections' spread
//    [14]     u8  iono_source     PAD_IONO_*: none, broadcast, pad survey
//    [15]     u8  flags           PCS_*
//    [16..17] u16 iono_gps_day    the stored broadcast set's file date (0: none)
//    [18..19] i16 iono_age_days   its age by the receiver's date (PCS_IONO_AGE)
//    [20..21] u16 pad_fixes       fixes in the pad average
//    [22..23] u16 pad_secs        the span they cover
//    [24..25] u16 pad_scatter_cm  their 3-D RMS about the average
//    [26..29] i32 pad_lat_e7
//    [30..33] i32 pad_lon_e7
//    [34..37] i32 pad_alt_mm      ellipsoidal
//    [38..39] u16 pos_sigma_cm    the raw-GNSS filter's 3-D sigma (0: not running)
//

import Foundation

nonisolated struct PadCalStatus: Equatable {

    /// PAD_CAL_*.
    enum CalState: UInt8 {
        case idle     = 0   // no pad calibration since boot
        case running  = 1   // the gyro/accel window is open
        case done     = 2   // it closed and the new offsets are in use
        case rejected = 3   // it closed implausible (gravity); the previous offsets kept
        case refused  = 4   // not run: launched, landed, a sim, or no IMU
    }

    /// PAD_IONO_*: what the raw-GNSS filter takes the ionosphere out with.
    enum IonoSource: UInt8 {
        case none      = 0  // neither: Klobuchar's night-time 5 ns only
        case broadcast = 1  // the broadcast parameters the app sent
        case padSurvey = 2  // the pad survey's per-satellite corrections (the backup)
    }

    // PCS_* flag bits.
    static let flagRawGnss: UInt8     = 0x01  // this board runs the raw-GNSS filter (LC86G)
    static let flagRunning: UInt8     = 0x02  //   and it is running
    static let flagIonoAge: UInt8     = 0x04  // iono_age_days is known
    static let flagPad: UInt8         = 0x08  // the pad average has fixes (pad_* are set)
    static let flagPadFrozen: UInt8   = 0x10  // launched: pad average and corrections frozen
    static let flagPadMismatch: UInt8 = 0x20  // raw measurements put the pad > 30 m from the
                                              // receiver's averaged fix: pad corrections off

    static let payloadLength = 40

    let timeUs: UInt32
    /// PAD_CAL_* as sent; `state` is nil for one newer than this app.
    let calState: UInt8
    let calProgress: UInt8
    let calGravityMg: UInt16
    let satsTracked: UInt8
    let satsNoEph: UInt8
    let satsSettling: UInt8
    let satsCorrected: UInt8
    let corrRmsCm: UInt16
    /// PAD_IONO_* as sent; `iono` is nil for one newer than this app.
    let ionoSource: UInt8
    let flags: UInt8
    let ionoGpsDay: UInt16
    let ionoAgeDays: Int16
    let padFixes: UInt16
    let padSecs: UInt16
    let padScatterCm: UInt16
    let padLatE7: Int32
    let padLonE7: Int32
    let padAltMm: Int32
    let posSigmaCm: UInt16

    var state: CalState? { CalState(rawValue: calState) }
    var iono: IonoSource? { IonoSource(rawValue: ionoSource) }

    /// The GNSS block applies only to a board that runs the raw-GNSS filter.
    var runsRawGnss: Bool { flags & Self.flagRawGnss != 0 }
    var filterRunning: Bool { flags & Self.flagRunning != 0 }
    var ionoAgeKnown: Bool { flags & Self.flagIonoAge != 0 }
    var hasPadAverage: Bool { flags & Self.flagPad != 0 }
    var padFrozen: Bool { flags & Self.flagPadFrozen != 0 }
    var padMismatch: Bool { flags & Self.flagPadMismatch != 0 }

    /// Decode the payload *after* the 0xD0 discriminator byte.
    static func decode(_ bytes: [UInt8]) -> PadCalStatus? {
        guard bytes.count >= payloadLength else { return nil }
        func u16(_ o: Int) -> UInt16 { UInt16(bytes[o]) | UInt16(bytes[o + 1]) << 8 }
        func u32(_ o: Int) -> UInt32 { UInt32(u16(o)) | UInt32(u16(o + 2)) << 16 }
        return PadCalStatus(
            timeUs: u32(0),
            calState: bytes[4], calProgress: bytes[5],
            calGravityMg: u16(6),
            satsTracked: bytes[8], satsNoEph: bytes[9],
            satsSettling: bytes[10], satsCorrected: bytes[11],
            corrRmsCm: u16(12),
            ionoSource: bytes[14], flags: bytes[15],
            ionoGpsDay: u16(16),
            ionoAgeDays: Int16(bitPattern: u16(18)),
            padFixes: u16(20),
            padSecs: u16(22),
            padScatterCm: u16(24),
            padLatE7: Int32(bitPattern: u32(26)),
            padLonE7: Int32(bitPattern: u32(30)),
            padAltMm: Int32(bitPattern: u32(34)),
            posSigmaCm: u16(38))
    }
}

// MARK: - What the pad calibration screen says
//
// Worded as Android's PadCalReadout words it, so the two apps tell an operator
// the same thing.

nonisolated extension PadCalStatus {

    /// A label and its value, as the screen lists them.
    struct Row: Equatable, Identifiable {
        let label: String
        let value: String
        var id: String { label }
    }

    static let padMismatchText =
        "Receiver's fix disagrees with its raw measurements — pad corrections off"

    /// The gyro/accel window, in words. While it runs the screen draws a
    /// progress bar under this.
    var gyroAccelText: String {
        switch state {
        case .idle?:
            return "No pad calibration since the rocket powered up"
        case .running?:
            return "Calibrating, \(calProgress) % — keep the rocket still"
        case .done?:
            return "Gyro & accel calibrated"
                + (calGravityMg > 0 ? " (\(Self.gravity(calGravityMg, decimals: 3)))" : "")
        case .rejected?:
            // A window that timed out or saw no samples is rejected with no
            // gravity at all.
            return "Rejected: "
                + (calGravityMg > 0 ? "gravity \(Self.gravity(calGravityMg, decimals: 2)) — " : "")
                + "keep the rocket still and try again; the previous calibration is kept"
        case .refused?:
            return "Not run: the rocket is not on the pad (or a sim is running)"
        case nil:
            return "Unknown state \(calState) — update the app"
        }
    }

    /// The check the 12 s flow made after a calibration (#1059), on the
    /// window's own gravity rather than one telemetry sample: a rocket sitting
    /// still reads 1 g, and the FC keeps anything within ±20 %, so a `done`
    /// more than 3 % off is still worth a warning.
    var gravityWarningText: String? {
        guard state == .done, calGravityMg > 0 else { return nil }
        let errorPct = abs(Double(calGravityMg) - 1000) / 10
        guard errorPct > 3 else { return nil }
        return String(format: "Gravity read %.1f %% off 1 g. ", errorPct)
            + "Consider running a bench calibration before flight."
    }

    /// The GNSS block, top to bottom (the mismatch warning goes above it).
    /// Meaningful only when `runsRawGnss`.
    var gnssRows: [Row] {
        var rows = [Row(label: "Satellites tracked", value: "\(satsTracked)")]
        if satsNoEph > 0 {
            rows.append(Row(label: "Waiting for ephemeris", value: "\(satsNoEph)"))
        }
        rows.append(Row(label: "Settling", value: "\(satsSettling)"))
        rows.append(Row(label: "Pad corrections", value: satsCorrected > 0
            ? "\(satsCorrected) (spread \(Self.metres(corrRmsCm)))" : "0"))
        rows.append(Row(label: "Ionosphere", value: ionosphereText))
        rows.append(Row(label: "Pad position", value: padPositionText))
        rows.append(Row(label: "Raw-GNSS filter", value: filterRunning
            ? "running" + (posSigmaCm > 0 ? ", position sigma \(Self.metres(posSigmaCm))" : "")
            : "not running yet"))
        return rows
    }

    /// What the raw-GNSS filter takes the ionosphere out with.
    var ionosphereText: String {
        switch iono {
        case .broadcast?:
            return "broadcast set"
                + (ionoGpsDay > 0 ? " of \(Self.day(Int(ionoGpsDay)))" : "")
                + (ionoAgeKnown ? ", " + Self.age(Int(ionoAgeDays)) : "")
        case .padSurvey?:
            return "pad survey (backup)"
        case .none?:
            return "none yet"
        case nil:
            return "source \(ionoSource) — update the app"
        }
    }

    var padPositionText: String {
        guard hasPadAverage else { return "no fixes yet" }
        return "\(padFixes) fixes over \(Self.minutesSeconds(padSecs)), "
            + "scatter \(Self.metres(padScatterCm))"
            + (padFrozen ? ", frozen at launch" : "")
    }

    /// "1.003 g" from milli-g.
    static func gravity(_ mg: UInt16, decimals: Int) -> String {
        String(format: "%.\(decimals)f g", Double(mg) / 1000)
    }

    /// "0.9 m" from centimetres.
    static func metres(_ cm: UInt16) -> String {
        String(format: "%.1f m", Double(cm) / 100)
    }

    /// "4:00" from seconds; the minutes run past 59 rather than wrap.
    static func minutesSeconds(_ s: UInt16) -> String {
        String(format: "%ld:%02ld", Int(s) / 60, Int(s) % 60)
    }

    /// A GPS day as its calendar date — in UTC, the date the file is named for.
    static func day(_ gpsDay: Int) -> String {
        let f = DateFormatter()
        f.dateStyle = .medium
        f.timeStyle = .none
        f.timeZone = TimeZone(identifier: "UTC")
        return f.string(from: Klobuchar.date(gpsDay))
    }

    /// The stored set's age by the receiver's date, which can sit a day
    /// behind the phone's around midnight UTC: a set dated "ahead" of it is as
    /// fresh as one can be.
    static func age(_ days: Int) -> String {
        switch days {
        case ...0: return "0 days old"
        case 1: return "1 day old"
        default: return "\(days) days old"
        }
    }
}
