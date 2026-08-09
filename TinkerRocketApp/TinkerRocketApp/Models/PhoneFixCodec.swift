//
//  PhoneFixCodec.swift
//  TinkerRocketApp
//
//  Wire encoding for BLE_BS_CMD_SET_PHONE_FIX (47) — the phone's GPS fix,
//  pushed to the base station so its CSV records where the base station was.
//
//  The base station has no GNSS: every lat/lon column in its log is the
//  ROCKET's relayed position. The app computes and shows the range between
//  the two ends from this fix and then discards it, so the one number a range
//  test is about was never written down. The 2026-08-08 Kaua'i test could
//  only be analysed because the operator remembered the site afterwards.
//
//  Kept as a free function with no BLE or CoreLocation-manager dependency so
//  the byte layout is unit-testable without a radio or a device.
//

import Foundation
import CoreLocation

nonisolated enum PhoneFixCodec {

    // The command number itself is deliberately NOT held here. Senders pass
    // the literal 47, because tools/check_ble_command_ids.py sweeps Swift for
    // literal send(Raw)Command numbers to prove every one has a firmware
    // handler and matches the Kotlin table — a named constant is invisible to
    // that sweep, and silently opting out of the parity gate is worse than
    // repeating one number. Firmware side: BLE_BS_CMD_SET_PHONE_FIX.

    /// Payload length the firmware requires before it will parse.
    static let payloadLength = 11

    /// Horizontal accuracy sentinel. A phone fix worse than 255 m is useless
    /// for range, and clamping to the sentinel keeps it distinguishable from
    /// a real reading instead of wrapping into a plausible one.
    static let accuracyUnknown: UInt8 = 255

    /// `[lat_e7 i32][lon_e7 i32][alt_m i16][h_acc_m u8]`, little-endian.
    ///
    /// 1e-7 degrees matches GNSSData's convention and resolves ~1 cm — far
    /// finer than any phone fix, so the encoding never limits accuracy.
    /// Every field clamps rather than wraps: a wrapped coordinate would read
    /// as a valid position somewhere else on Earth.
    static func encode(_ coord: CLLocationCoordinate2D,
                       altitude: Double?,
                       accuracy: Double?) -> Data {
        let latE7 = Int32(clamping: Int64((coord.latitude  * 1e7).rounded()))
        let lonE7 = Int32(clamping: Int64((coord.longitude * 1e7).rounded()))
        let altM  = Int16(clamping: Int64((altitude ?? 0).rounded()))
        let accM: UInt8 = {
            guard let a = accuracy, a >= 0 else { return accuracyUnknown }
            return UInt8(clamping: Int64(a.rounded()))
        }()

        var data = Data(capacity: payloadLength)
        withUnsafeBytes(of: latE7.littleEndian) { data.append(contentsOf: $0) }
        withUnsafeBytes(of: lonE7.littleEndian) { data.append(contentsOf: $0) }
        withUnsafeBytes(of: altM.littleEndian)  { data.append(contentsOf: $0) }
        data.append(accM)
        return data
    }
}
