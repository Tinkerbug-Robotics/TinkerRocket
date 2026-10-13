//
//  Klobuchar.swift
//  TinkerRocketApp
//
//  The GPS broadcast ionosphere (Klobuchar) parameters a pad calibration
//  carries to the flight computer (#1600 phase 5). Port of core/protocol
//  Klobuchar.kt — keep the two in step.
//
//  The raw-GNSS filter takes the ionosphere out of each pseudorange with the
//  eight numbers GPS broadcasts. The LC86G does not pass them on (RTCM has no
//  message for them), so the app reads them from the day's broadcast
//  navigation file — NOAA's CORS archive, `brdcDDD0.YYn.gz`, public and
//  without a login, updated through the day — and keeps the last set it read,
//  so a launch site with no signal still has one.
//
//  On the wire they go as GPS broadcasts them: eight signed bytes, alpha0..3
//  then beta0..3, at the scales of IS-GPS-200 20.3.3.5.1.7, then the file's
//  date as a GPS day (days since 1980-01-06), u16 little-endian. The flight
//  computer judges their age against the receiver's own date.
//
//  Swift has no LocalDate, so a date here is an instant and its UTC calendar
//  date is what counts: `gpsDay` of any moment of 2026-10-13 UTC is 17082, and
//  `date(_:)` is that day's midnight UTC. Show one in UTC, or a phone west of
//  Greenwich prints the day before.
//

import Foundation

nonisolated struct Klobuchar: Equatable {
    /// alpha0..3, beta0..3 in broadcast units, each -128...127.
    let raw: [Int]
    /// The navigation file's date: days since 1980-01-06.
    let gpsDay: Int

    /// Nil unless eight signed bytes and a GPS day (Kotlin's `require`s).
    init?(raw: [Int], gpsDay: Int) {
        guard raw.count == 8, raw.allSatisfy({ (-128...127).contains($0) }),
              (1...0xFFFF).contains(gpsDay) else { return nil }
        self.raw = raw
        self.gpsDay = gpsDay
    }

    /// alpha0..3 (s, s/semicircle, ...) then beta0..3 (s, s/semicircle, ...).
    func values() -> [Double] {
        raw.enumerated().map { k, b in Double(b) * Self.scales[k] }
    }

    /// `[raw i8 x8][gps_day u16 LE]`.
    func payload() -> Data {
        var out = Data(count: Self.payloadLength)
        for k in 0..<8 { out[k] = UInt8(bitPattern: Int8(raw[k])) }
        out[8] = UInt8(gpsDay & 0xFF)
        out[9] = UInt8((gpsDay >> 8) & 0xFF)
        return out
    }

    static let payloadLength = 10

    /// 2^-30, 2^-27, 2^-24, 2^-24, 2^11, 2^14, 2^16, 2^16.
    static let scales: [Double] = [
        9.313225746154785e-10, 7.450580596923828e-09, 5.960464477539063e-08, 5.960464477539063e-08,
        2048.0, 16384.0, 65536.0, 65536.0,
    ]

    /// 1980-01-06T00:00:00Z in seconds since 1970. Unix time has no leap
    /// seconds, so every UTC day is exactly 86 400 of them.
    private static let gpsEpoch: TimeInterval = 315_964_800
    private static let secondsPerDay: TimeInterval = 86_400

    /// Days since 1980-01-06 of the UTC calendar date `utcDate` falls on.
    static func gpsDay(_ utcDate: Date) -> Int {
        Int(((utcDate.timeIntervalSince1970 - gpsEpoch) / secondsPerDay).rounded(.down))
    }

    /// The calendar date of a GPS day: its midnight UTC.
    static func date(_ gpsDay: Int) -> Date {
        Date(timeIntervalSince1970: gpsEpoch + Double(gpsDay) * secondsPerDay)
    }

    /// NOAA CORS's GPS broadcast navigation file for the UTC date `utcDate`
    /// falls on.
    static func noaaUrl(_ utcDate: Date) -> String {
        var utc = Calendar(identifier: .gregorian)
        utc.timeZone = TimeZone(identifier: "UTC")!
        let year = utc.component(.year, from: utcDate)
        let doy = utc.ordinality(of: .day, in: .year, for: utcDate) ?? 1
        return String(format: "https://geodesy.noaa.gov/corsdata/rinex/%04ld/%03ld/brdc%03ld0.%02ldn.gz",
                      year, doy, doy, year % 100)
    }

    /// The parameters from a navigation file's header: RINEX 2's
    /// `ION ALPHA` / `ION BETA`, or RINEX 3's `GPSA` / `GPSB` lines of
    /// `IONOSPHERIC CORR`. Nil without both, or if a value is not a whole
    /// number of its broadcast unit (the file prints five digits, so a
    /// genuine one is within a few thousandths of one).
    ///
    /// Reads line by line and stops at `END OF HEADER`, as Kotlin's lazy
    /// `lineSequence()` does: the body is a day of ephemerides.
    static func fromRinexHeader(_ text: String, gpsDay: Int) -> Klobuchar? {
        var alpha: [Double]?
        var beta: [Double]?
        var rest = Substring(text)
        while !rest.isEmpty {
            // "\r\n" is one Character, so CRLF files split as LF ones do.
            let end = rest.firstIndex(where: \.isNewline) ?? rest.endIndex
            let line = rest[..<end]
            rest = end < rest.endIndex ? rest[rest.index(after: end)...] : ""
            let chars = Array(line)
            if chars.count < 61 { continue }
            let label = String(chars[60...]).trimmingCharacters(in: .whitespaces)
            // An `if`, not a case below: `break` in a Swift switch leaves the
            // switch, not the loop.
            if label == "END OF HEADER" { break }
            switch label {
            case "ION ALPHA": alpha = fields(chars, 2)
            case "ION BETA": beta = fields(chars, 2)
            case "IONOSPHERIC CORR" where line.hasPrefix("GPSA"): alpha = fields(chars, 5)
            case "IONOSPHERIC CORR" where line.hasPrefix("GPSB"): beta = fields(chars, 5)
            default: break
            }
        }
        guard let a = alpha, let b = beta else { return nil }
        let values = a + b
        var raw: [Int] = []
        raw.reserveCapacity(8)
        for k in 0..<8 {
            let x = values[k] / scales[k]
            let r = x.rounded()
            // isFinite first: Int(_:) traps on a NaN or an infinity, where
            // Kotlin's roundToInt() throws or saturates. Neither is broadcast.
            guard x.isFinite, abs(x - r) <= 0.05, r >= -128, r <= 127 else { return nil }
            raw.append(Int(r))
        }
        guard (1...0xFFFF).contains(gpsDay) else { return nil }
        return Klobuchar(raw: raw, gpsDay: gpsDay)
    }

    /// Four D12.4 fields from column `start`; nil if any is unreadable.
    private static func fields(_ line: [Character], _ start: Int) -> [Double]? {
        var out: [Double] = []
        out.reserveCapacity(4)
        for k in 0..<4 {
            let from = start + 12 * k
            let to = from + 12
            if to > line.count { return nil }
            let v = String(line[from..<to]).trimmingCharacters(in: .whitespaces)
                .replacingOccurrences(of: "D", with: "E")
                .replacingOccurrences(of: "d", with: "E")
            guard let d = Double(v) else { return nil }
            out.append(d)
        }
        return out
    }
}
