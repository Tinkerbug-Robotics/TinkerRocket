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
//  A flight computer that knows the high-g sends AccelCalHighGStatus after
//  those 40 bytes (114 in all), relayed whole by an out computer that knows
//  it too; a 40-byte frame is an older one's (`highG` nil):
//    [40..51] f32 face_hg_mean_mps2[3] the face window's mean high-g reading,
//                                  board frame, before any correction
//    [52..63] f32 face_hg_sd_mps2[3]
//    [64]     u8  hg_flags          HGF_*: bit 0 an alignment stored, bit 1
//                                  applied (never in a sim), bit 2 a bias stored
//    [65]     u8  hg_bias_src       HG_BIAS_SRC_*: what wrote the bias in use
//    [66..101] f32 hg_matrix[9]     the stored alignment M, row-major
//                                  (identity when none)
//    [102..113] f32 hg_bias_mps2[3] the bias in use, b
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
    static let highGPayloadLength = 114

    /// The high-g part of the frame (AccelCalHighGStatus): its corrected
    /// reading is high = M * h - b - offset, board frame.
    nonisolated struct HighG: Equatable {
        /// HG_BIAS_SRC_*.
        enum BiasSource: UInt8 {
            case none = 0       // none stored, or written before this was recorded
            case padCal = 1     // the pad calibration (or a saved one the app pushed back)
            case sixFaces = 2   // the six-face fit
        }

        static let flagStored: UInt8  = 0x01   // an alignment is in NVS
        static let flagApplied: UInt8 = 0x02   //   and the converter applies it (not in sim)
        static let flagBias: UInt8    = 0x04   // a high-g bias is in NVS

        /// The face window's mean high-g reading, board frame x, y, z, m/s².
        let faceMean: [Float]
        let faceSD: [Float]
        let flags: UInt8
        /// HG_BIAS_SRC_* as sent; `source` is nil for one newer than this app.
        let biasSource: UInt8
        /// M, row-major.
        let matrix: [Float]
        /// b, board frame x, y, z, m/s².
        let bias: [Float]

        var isStored: Bool { flags & Self.flagStored != 0 }
        var isApplied: Bool { flags & Self.flagApplied != 0 }
        var hasBias: Bool { flags & Self.flagBias != 0 }
        var source: BiasSource? { BiasSource(rawValue: biasSource) }
    }

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
    /// nil: the frame carries no high-g -- an older flight computer, or an
    /// older out computer relaying only the first 40 bytes.
    var highG: HighG? = nil

    var state: FaceState? { FaceState(rawValue: faceState) }
    var isStored: Bool { flags & Self.flagStored != 0 }
    var isApplied: Bool { flags & Self.flagApplied != 0 }

    /// Decode the payload *after* the 0xD1 discriminator byte.
    static func decode(_ bytes: [UInt8]) -> AccelCalStatus? {
        guard bytes.count >= payloadLength else { return nil }
        func u16(_ o: Int) -> UInt16 { UInt16(bytes[o]) | UInt16(bytes[o + 1]) << 8 }
        func u32(_ o: Int) -> UInt32 { UInt32(u16(o)) | UInt32(u16(o + 2)) << 16 }
        func f32s(_ o: Int, _ n: Int) -> [Float] { (0..<n).map { Float(bitPattern: u32(o + 4 * $0)) } }
        func f32x3(_ o: Int) -> [Float] { f32s(o, 3) }
        var highG: HighG?
        if bytes.count >= highGPayloadLength {
            highG = HighG(faceMean: f32x3(40), faceSD: f32x3(52), flags: bytes[64], biasSource: bytes[65],
                          matrix: f32s(66, 9), bias: f32x3(102))
        }
        return AccelCalStatus(flags: bytes[0], faceState: bytes[1],
                              offset: f32x3(2), faceMean: f32x3(14), faceSD: f32x3(26),
                              faceN: u16(38), highG: highG)
    }
}

/// HighGCalApplyData, the command-80 payload (49 bytes): op (1 store, 0
/// clear), the alignment M row-major and the bias b, little-endian Float32.
/// A clear sets M back to identity and keeps the bias. The FC refuses a
/// scale outside 0.97...1.03, a cross-axis term past 0.03 and a bias past
/// 11 m/s² (and anything in flight).
nonisolated enum HighGCalApply {
    static let payloadLength = 49
    static let opClear: UInt8 = 0
    static let opStore: UInt8 = 1
    static let identity: [Float] = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    static func payload(op: UInt8, matrix: [Float] = identity, bias: [Float] = [0, 0, 0]) -> Data {
        var data = Data(capacity: payloadLength)
        data.append(op)
        for v in matrix + bias {
            withUnsafeBytes(of: v.bitPattern.littleEndian) { data.append(contentsOf: $0) }
        }
        return data
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
