//
//  AccelCalStatus.swift
//  TinkerRocketApp
//
//  Decoded form of the FC's AccelCalStatusData frame (#1629): the low-g
//  accelerometer's six-face hand calibration -- the offset stored on the
//  board and the last face window. Arrives on the file_ops characteristic
//  behind a 0xD1 discriminator, in answer to commands 76 (apply), 77 (read)
//  and 78 (measure a face), and once more when a face window closes. Wire
//  layout mirrors AccelCalStatusData in RocketComputerTypes.h -- keep both in
//  sync. The six-face screen (AccelCalibrationView) is the only reader; the
//  offset is a board property, so it is in no rocket profile and no
//  connect-time sync.
//
//  Wire (40 bytes, little-endian, packed):
//    [0]      u8  flags            ACCF_*: bit 0 stored (in NVS), bit 1 applied
//                                  (the converter takes it off; never in a sim)
//    [1]      u8  face_state       ACC_FACE_*: idle, measuring, done, failed
//    [2..13]  f32 offset_mps2[3]   the stored offset, board frame (0 when none)
//    [14..25] f32 face_mean_mps2[3] the last face window's mean low-g reading,
//                                  board frame, before any offset
//    [26..37] f32 face_sd_mps2[3]  each axis's sample SD over that window
//    [38..39] u16 face_n           its samples
//

import Foundation

nonisolated struct AccelCalStatus: Equatable {

    /// ACC_FACE_*: the last face window.
    enum FaceState: UInt8 {
        case idle      = 0  // no window since boot
        case measuring = 1  // one is open (3 s)
        case done      = 2  // it closed still and at one g
        case failed    = 3  // it moved (an axis SD > 0.05 m/s²) or was not one g
                            // (|mean| more than 0.5 m/s² off); also a refusal,
                            // with face_n 0: in flight, after a flight, in a
                            // sim or during a magnetometer calibration
    }

    // ACCF_* flag bits.
    static let flagStored: UInt8  = 0x01   // an offset is in NVS
    static let flagApplied: UInt8 = 0x02   //   and the converter takes it off (not in sim)

    static let payloadLength = 40

    let flags: UInt8
    /// ACC_FACE_* as sent; `state` is nil for one newer than this app.
    let faceState: UInt8
    /// The stored offset, board frame x, y, z, m/s².
    let offset: [Float]
    /// The last face window's mean, board frame x, y, z, m/s², before any offset.
    let faceMean: [Float]
    /// Each axis's sample SD over that window, m/s².
    let faceSD: [Float]
    let faceN: UInt16

    var state: FaceState? { FaceState(rawValue: faceState) }
    var isStored: Bool { flags & Self.flagStored != 0 }
    var isApplied: Bool { flags & Self.flagApplied != 0 }

    /// Decode the payload *after* the 0xD1 discriminator byte.
    static func decode(_ bytes: [UInt8]) -> AccelCalStatus? {
        guard bytes.count >= payloadLength else { return nil }
        func u16(_ o: Int) -> UInt16 { UInt16(bytes[o]) | UInt16(bytes[o + 1]) << 8 }
        func u32(_ o: Int) -> UInt32 { UInt32(u16(o)) | UInt32(u16(o + 2)) << 16 }
        func f32x3(_ o: Int) -> [Float] { (0..<3).map { Float(bitPattern: u32(o + 4 * $0)) } }
        return AccelCalStatus(flags: bytes[0], faceState: bytes[1],
                              offset: f32x3(2), faceMean: f32x3(14), faceSD: f32x3(26),
                              faceN: u16(38))
    }
}

/// AccelCalApplyData, the command-76 payload: the offset to store and apply,
/// board frame, m/s², as three little-endian Float32 (12 bytes). All zero
/// clears it. The FC refuses any |component| > 1.0 m/s² (and in flight).
nonisolated enum AccelCalApply {
    static let payloadLength = 12

    static func payload(x: Float, y: Float, z: Float) -> Data {
        var data = Data(capacity: payloadLength)
        for v in [x, y, z] {
            withUnsafeBytes(of: v.bitPattern.littleEndian) { data.append(contentsOf: $0) }
        }
        return data
    }
}

// MARK: - What the six-face screen says

nonisolated extension AccelCalStatus {

    /// "+4.20 mg" from m/s²: milli-g against standard gravity.
    static func mg(_ mps2: Double) -> String {
        String(format: "%+.2f mg", mps2 / AccelSixFace.g0 * 1000)
    }

    /// "X +4.20 mg, Y -2.78 mg, Z +1.59 mg", or "no offset stored".
    var offsetsText: String {
        guard isStored else { return "no offset stored" }
        return (0..<3).map { "\(AccelSixFace.axisNames[$0]) \(Self.mg(Double(offset[$0])))" }
            .joined(separator: ", ")
    }

    /// What the flight computer does with the stored offset.
    var useText: String {
        switch (isStored, isApplied) {
        case (true, true):   return "In use by the flight computer"
        case (true, false):  return "Stored, but not in use while a sim runs"
        case (false, true):  return "In use, but not stored on the board"
        case (false, false): return "No offset stored"
        }
    }
}
