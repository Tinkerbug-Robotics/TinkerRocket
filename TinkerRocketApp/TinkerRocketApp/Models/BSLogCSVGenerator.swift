//
//  BSLogCSVGenerator.swift
//  Base-station binary log → CSV (#850).
//
//  The base station used to write this CSV itself, one row per packet,
//  formatting every float on the MCU. Since #850 it logs the bytes it received
//  — framed exactly like the rocket computer's log — and the CSV is generated
//  here.
//
//  THE FORWARD-FILL is the load-bearing part. A log holds two interleaved frame
//  types: a 55-byte FAST frame five slots out of six and a 22-byte SLOW frame
//  on the sixth. Each carries a subset, so a row is built by merging the frame
//  into a running accumulator per rocket, exactly as the base station does in
//  RAM. Without it a slow frame would blank the position and a fast frame would
//  blank the battery, and the CSV would alternate between half-empty rows.
//
//  `ecefToGeodetic` and `eulerFromQuat` are deliberately literal transcriptions
//  of TR_Coordinates::ecefToGeodetic and SensorConverter::eulerFromQuat rather
//  than tidier equivalents, so the Python, Kotlin and Swift renderings cannot
//  drift in the last decimal. All three are pinned against the same golden,
//  whose input is packed by the REAL firmware packers
//  (tests_cpp/fixtures/wire/csv/bs_tiny.bin).
//
//  Android twin: `BsLogCsvGenerator.kt`.
//

import Foundation

nonisolated enum BSLogCSVError: Error, LocalizedError {
    case notABaseStationLog
    case unsupportedVersion(Int)

    var errorDescription: String? {
        switch self {
        case .notABaseStationLog:
            return "not a base-station binary log (missing TRBSLOG magic)"
        case .unsupportedVersion(let v):
            return "base-station log format v\(v), this build speaks v\(BSLogCSVGenerator.formatVersion)"
        }
    }
}

nonisolated final class BSLogCSVGenerator {

    static let magic = Array("TRBSLOG".utf8)
    static let formatVersion = 1

    private static let bsLoRaRxMsg: UInt8 = 0xFC
    private static let bsEventMsg: UInt8 = 0xFD

    private static let sizeOfLoRaFast = 55
    private static let sizeOfLoRaSlow = 22
    private static let loRaProtoVersion = 5
    private static let loRaFrameSlow = 0x1

    private static let loRaHdrLen = 7
    private static let bsRxHdrLen = 12
    private static let bsEvtHdrLen = 9

    /// INT16_MIN: the radio reported no reading. Distinct from a genuine 0 dBm.
    private static let rssiUnknown = -32768

    private static let preamble: [UInt8] = [0xAA, 0x55, 0xAA, 0x55]

    private static let wgs84A = 6378137.0
    private static let wgs84F = 1.0 / 298.257223563
    private static let wgs84E2 = 2 * wgs84F - wgs84F * wgs84F

    private static let stateNames = [
        "INIT", "READY", "PRELAUNCH", "INFLIGHT", "LANDED", "MAG_CAL", "6", "7",
    ]

    /// The firmware's 39 columns verbatim, with `frame` appended. Appending is
    /// this repo's convention for column additions (it is how rocket_id and
    /// cam_a/servo_a arrived), so anything keying columns by name is unaffected
    /// while the fast/slow information that only exists post-#850 survives.
    static let columns: [String] = [
        "time_ms", "state", "num_sats", "pdop", "lat", "lon", "alt_m", "h_acc",
        "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
        "pressure_alt", "alt_rate", "max_alt", "max_speed",
        "voltage", "current", "soc", "cam_a", "servo_a",
        "roll", "pitch", "yaw", "speed",
        "launch", "vel_apo", "alt_apo", "landed", "rssi", "snr",
        "next_ch", "rx_freq_mhz", "seq", "gap", "event", "rocket_id", "frame",
    ]

    /// True when `bytes` opens with the base-station log magic.
    static func isBaseStationLog(_ bytes: [UInt8]) -> Bool {
        guard bytes.count >= magic.count else { return false }
        return Array(bytes[0..<magic.count]) == magic
    }

    // MARK: - decoded record

    struct Row {
        var timeMs: UInt32 = 0
        var rssi = Double.nan
        var snr = Double.nan
        var rxFreqMhz = 0.0
        var frame = ""

        var networkId = 0, rocketId = 0, nextCh = 255, seq = 0
        var state = ""
        var launch = 0, velApo = 0, altApo = 0, landed = 0

        var numSats = 0, pdop = 0.0, hAcc = 0.0
        var ecefX = 0.0, ecefY = 0.0, ecefZ = 0.0
        var accX = 0.0, accY = 0.0, accZ = 0.0
        var gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0
        var q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0
        var hasQuat = false
        var pressureAlt = 0.0, altRate = 0.0, speed = 0.0

        var maxAlt = 0.0, maxSpeed = 0.0
        var voltage = 0.0, current = 0.0, soc = 0.0
        var camA = 0.0, servoA = 0.0

        var gap = -1
    }

    struct Event {
        let timeMs: UInt32
        let rxFreqMhz: Double
        let text: String
    }

    // MARK: - little-endian readers

    private static func u8(_ b: [UInt8], _ o: Int) -> Int { Int(b[o]) }
    private static func i8(_ b: [UInt8], _ o: Int) -> Int { Int(Int8(bitPattern: b[o])) }
    private static func u16(_ b: [UInt8], _ o: Int) -> Int { Int(b[o]) | (Int(b[o + 1]) << 8) }
    private static func i16(_ b: [UInt8], _ o: Int) -> Int { Int(Int16(truncatingIfNeeded: u16(b, o))) }
    private static func u32(_ b: [UInt8], _ o: Int) -> UInt32 {
        UInt32(b[o]) | (UInt32(b[o + 1]) << 8) | (UInt32(b[o + 2]) << 16) | (UInt32(b[o + 3]) << 24)
    }

    /// Signed little-endian 24-bit, matching i24le_t on the wire.
    private static func i24(_ b: [UInt8], _ o: Int) -> Int {
        let v = Int(b[o]) | (Int(b[o + 1]) << 8) | (Int(b[o + 2]) << 16)
        return (v & 0x800000) != 0 ? v - 0x1000000 : v
    }

    private static func decodeVoltage(_ u: Int) -> Double { 2.0 + (Double(u) / 255.0) * 8.0 }

    // MARK: - frame decode

    /// Returns ver_type; writes the fields BOTH frames carry.
    private static func unpackHeader(_ f: [UInt8], _ r: inout Row) -> Int {
        r.networkId = u8(f, 0)
        r.rocketId = u8(f, 1)
        r.nextCh = u8(f, 2)
        r.seq = u16(f, 3)
        let verType = u8(f, 5)
        let flags = u8(f, 6)
        r.launch = (flags & 0x01) != 0 ? 1 : 0
        r.velApo = (flags & 0x02) != 0 ? 1 : 0
        r.altApo = (flags & 0x04) != 0 ? 1 : 0
        r.landed = (flags & 0x08) != 0 ? 1 : 0
        r.state = stateNames[(flags >> 4) & 0x07]
        return verType
    }

    /// Writes ONLY what the fast frame carries — never clears the rest.
    private static func unpackFast(_ f: [UInt8], _ r: inout Row) {
        let o = loRaHdrLen
        r.numSats = u8(f, o) & 0x3F
        r.pdop = Double(u8(f, o + 1))
        r.hAcc = Double(u8(f, o + 2))
        r.ecefX = Double(i24(f, o + 3))
        r.ecefY = Double(i24(f, o + 6))
        r.ecefZ = Double(i24(f, o + 9))
        r.accX = Double(i16(f, o + 12)) / 10.0
        r.accY = Double(i16(f, o + 14)) / 10.0
        r.accZ = Double(i16(f, o + 16)) / 10.0
        r.gyroX = Double(i16(f, o + 18)) / 10.0
        r.gyroY = Double(i16(f, o + 20)) / 10.0
        r.gyroZ = Double(i16(f, o + 22)) / 10.0
        r.q0 = Double(i16(f, o + 24)) / 10000.0
        r.q1 = Double(i16(f, o + 26)) / 10000.0
        r.q2 = Double(i16(f, o + 28)) / 10000.0
        r.q3 = Double(i16(f, o + 30)) / 10000.0
        r.hasQuat = true
        r.pressureAlt = Double(i24(f, o + 32))
        r.altRate = Double(i16(f, o + 35))
        let ve = Double(i16(f, o + 37)) / 10.0
        let vn = Double(i16(f, o + 39)) / 10.0
        let vu = Double(i16(f, o + 41)) / 10.0
        // Derived on the ground since #191; both inputs ride THIS frame, so the
        // derivation is self-consistent rather than mixing fresh with stale.
        r.speed = (ve * ve + vn * vn + vu * vu).squareRoot()
    }

    /// Writes ONLY what the slow frame carries.
    private static func unpackSlow(_ f: [UInt8], _ r: inout Row) {
        let o = loRaHdrLen
        r.maxAlt = Double(i24(f, o))
        r.maxSpeed = Double(i16(f, o + 3))
        r.voltage = decodeVoltage(u8(f, o + 7))
        r.current = Double(i16(f, o + 8))
        r.soc = Double(i8(f, o + 10))
        r.camA = Double(u16(f, o + 11)) / 1000.0
        r.servoA = Double(u16(f, o + 13)) / 1000.0
    }

    // MARK: - geodesy / attitude

    static func ecefToGeodetic(_ x: Double, _ y: Double, _ z: Double) -> (Double, Double, Double) {
        let lon = atan2(y, x)
        let p = (x * x + y * y).squareRoot()
        var lat = atan2(z, p * (1 - wgs84E2))
        var alt = 0.0
        while true {
            let prev = lat
            let n = wgs84A / (1 - wgs84E2 * sin(lat) * sin(lat)).squareRoot()
            alt = p / cos(lat) - n
            lat = atan2(z + n * wgs84E2 * sin(lat), p)
            if abs(lat - prev) <= 1e-10 { break }
        }
        return (lat * 180.0 / Double.pi, lon * 180.0 / Double.pi, alt)
    }

    static func eulerFromQuat(_ qw: Double, _ qx: Double, _ qy: Double,
                              _ qz: Double) -> (Double, Double, Double) {
        // Roll — azimuth of body Z in the NED horizontal plane (gimbal-lock-free)
        let zN = 2.0 * (qx * qz + qw * qy)
        let zE = 2.0 * (qy * qz - qw * qx)
        let roll = -atan2(zE, zN) * 180.0 / Double.pi
        let sinp = 2.0 * (qw * qy - qz * qx)
        let pitch = abs(sinp) >= 1.0 ? (90.0 * (sinp < 0 ? -1.0 : 1.0))
                                     : asin(sinp) * 180.0 / Double.pi
        let yaw = atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / Double.pi
        return (roll, pitch, yaw)
    }

    // MARK: - format

    /// printf("%.Nf") — which is exactly what Swift's String(format:) is.
    private static func fmt(_ v: Double, _ decimals: Int) -> String {
        if v.isNaN { return "nan" }
        return String(format: "%.\(decimals)f", v)
    }

    // MARK: - parse

    private static func indexOfPreamble(_ b: [UInt8], from: Int) -> Int? {
        var i = from
        while i + 4 <= b.count {
            if b[i] == preamble[0] && b[i + 1] == preamble[1]
                && b[i + 2] == preamble[2] && b[i + 3] == preamble[3] {
                return i
            }
            i += 1
        }
        return nil
    }

    static func parse(_ bytes: [UInt8]) throws -> ([Row], [Event]) {
        guard isBaseStationLog(bytes) else { throw BSLogCSVError.notABaseStationLog }
        let version = bytes.count > magic.count ? Int(bytes[magic.count]) : 0
        guard version == formatVersion else { throw BSLogCSVError.unsupportedVersion(version) }

        var rows: [Row] = []
        var events: [Event] = []
        var accum: [Int: Row] = [:]
        var lastSeq: [Int: Int] = [:]

        var i = 0
        while let p = indexOfPreamble(bytes, from: i) {
            var j = p + 4
            if j + 3 >= bytes.count { break }
            let mtype = bytes[j]
            let len = Int(bytes[j + 1])
            j += 2
            if j + len + 2 > bytes.count { break }
            let payload = Array(bytes[j..<(j + len)])
            i = j + len + 2

            if mtype == bsEventMsg {
                if payload.count < bsEvtHdrLen { continue }
                let tMs = u32(payload, 0)
                let freq = u32(payload, 4)
                let tlen = u8(payload, 8)
                let end = min(bsEvtHdrLen + tlen, payload.count)
                let text = String(decoding: payload[bsEvtHdrLen..<end], as: UTF8.self)
                events.append(Event(timeMs: tMs, rxFreqMhz: Double(freq) / 1e6, text: text))
                continue
            }
            if mtype != bsLoRaRxMsg || payload.count <= bsRxHdrLen { continue }

            let tMs = u32(payload, 0)
            let rssiX10 = i16(payload, 4)
            let snrX10 = i16(payload, 6)
            let freqHz = u32(payload, 8)
            let frame = Array(payload[bsRxHdrLen...])
            if frame.count != sizeOfLoRaFast && frame.count != sizeOfLoRaSlow { continue }

            var probe = Row()
            let verType = unpackHeader(frame, &probe)
            if (verType >> 4) != loRaProtoVersion { continue }
            let ftype = verType & 0x0F

            // Forward-fill: start from this rocket's last known state and let
            // the frame overwrite only what it actually contains.
            var row = accum[probe.rocketId] ?? Row()
            _ = unpackHeader(frame, &row)
            if ftype == loRaFrameSlow { unpackSlow(frame, &row) } else { unpackFast(frame, &row) }

            row.timeMs = tMs
            row.rssi = rssiX10 == rssiUnknown ? Double.nan : Double(rssiX10) / 10.0
            row.snr = snrX10 == rssiUnknown ? Double.nan : Double(snrX10) / 10.0
            row.rxFreqMhz = Double(freqHz) / 1e6
            row.frame = ftype == loRaFrameSlow ? "slow" : "fast"

            if let prev = lastSeq[row.rocketId] {
                row.gap = (row.seq - prev - 1) & 0xFFFF
            } else {
                row.gap = -1
            }
            lastSeq[row.rocketId] = row.seq

            accum[row.rocketId] = row
            rows.append(row)
        }
        return (rows, events)
    }

    // MARK: - render

    /// Binary base-station log → CSV text.
    static func writeCSV(_ bytes: [UInt8]) throws -> String {
        let (rows, events) = try parse(bytes)
        var out = columns.joined(separator: ",") + "\n"

        // Merge events in by time so one pass sees telemetry and events in
        // arrival order — the property the firmware's padded EVENT rows gave.
        enum Item { case row(Row), event(Event) }
        var merged: [(UInt32, Item)] = rows.map { ($0.timeMs, .row($0)) }
        merged += events.map { ($0.timeMs, .event($0)) }
        merged.sort { $0.0 < $1.0 }

        for (_, item) in merged {
            switch item {
            case .event(let ev):
                var cells = [String](repeating: "", count: columns.count)
                cells[0] = String(ev.timeMs)
                cells[1] = "EVENT"
                cells[columns.firstIndex(of: "rx_freq_mhz")!] = fmt(ev.rxFreqMhz, 3)
                cells[columns.firstIndex(of: "event")!] = ev.text
                out += cells.joined(separator: ",") + "\n"

            case .row(let r):
                // lat/lon only where the rocket claims a fix — nonzero ECEF
                // with num_sats == 0 is a stale register read (#95) and would
                // render a valid-looking position for an invalid fix.
                var lat = Double.nan, lon = Double.nan, alt = Double.nan
                if r.numSats > 0 && (r.ecefX != 0 || r.ecefY != 0 || r.ecefZ != 0) {
                    (lat, lon, alt) = ecefToGeodetic(r.ecefX, r.ecefY, r.ecefZ)
                }
                var roll = 0.0, pitch = 0.0, yaw = 0.0
                if r.hasQuat {
                    (roll, pitch, yaw) = eulerFromQuat(r.q0, r.q1, r.q2, r.q3)
                }

                out += [
                    String(r.timeMs), r.state, String(r.numSats), fmt(r.pdop, 1),
                    fmt(lat, 7), fmt(lon, 7), fmt(alt, 1), fmt(r.hAcc, 1),
                    fmt(r.accX, 2), fmt(r.accY, 2), fmt(r.accZ, 2),
                    fmt(r.gyroX, 1), fmt(r.gyroY, 1), fmt(r.gyroZ, 1),
                    fmt(r.pressureAlt, 1), fmt(r.altRate, 1),
                    fmt(r.maxAlt, 1), fmt(r.maxSpeed, 1),
                    fmt(r.voltage, 2), fmt(r.current, 0), fmt(r.soc, 1),
                    fmt(r.camA, 3), fmt(r.servoA, 3),
                    fmt(roll, 1), fmt(pitch, 1), fmt(yaw, 1), fmt(r.speed, 1),
                    String(r.launch), String(r.velApo), String(r.altApo), String(r.landed),
                    fmt(r.rssi, 0), fmt(r.snr, 1),
                    String(r.nextCh), fmt(r.rxFreqMhz, 3),
                    String(r.seq), String(r.gap),
                    "", String(r.rocketId), r.frame,
                ].joined(separator: ",") + "\n"
            }
        }
        return out
    }
}
