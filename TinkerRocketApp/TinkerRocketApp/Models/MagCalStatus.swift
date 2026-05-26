//
//  MagCalStatus.swift
//  TinkerRocketApp
//
//  Decoded form of the FC's MagCalStatusData binary frame (issue #96).
//  The wire frame is 26 bytes, little-endian, prefixed with a 0xCA byte
//  on the BLE file_ops characteristic (sibling of the 0xAA scan-results
//  prefix).  Field layout is mirrored from
//  tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h
//  — keep both in sync if the wire format ever changes.
//
//  Older firmware ships a 22-byte payload (no coverage_mask trailer);
//  the decoder accepts both and defaults coverageMask to 0 on old FC
//  builds so the UI degrades to timer-based prompts.
//

import Foundation

enum MagCalSubType: UInt8 {
    case idle      = 0
    case sampling  = 1
    case review    = 2
    case applied   = 3
    case aborted   = 4
    // #206 — between Accept and final NVS persist.  The chip's OFFSET
    // regs are already programmed with the new fit and the FC is
    // sampling for ~5 s while the user rotates the rocket, to verify
    // the corrected |B| stays in a tight band.  iOS shows a
    // "Verifying… rotate slowly" screen with the live |B|.
    case verifying = 5
}

enum MagCalRejectCode: UInt8 {
    case ok               = 0
    case rTooLow          = 1   // fitted R < 20 µT
    case rTooHigh         = 2   // fitted R > 80 µT
    case highResidual     = 3   // RMS residual above threshold (poor sphere fit)
    case lowCoverage      = 4   // < min populated wedges (insufficient tumble coverage)
    // #206 — post-accept verify pass saw the corrected |B| wander
    // outside the [20, 70] µT band, spread by more than 25 µT across
    // the rotation, or didn't gather enough samples / rotation.  The
    // frame's instantaneousFieldUT carries the worst observed |B|.
    case verifyFailed     = 5
}

struct MagCalStatus: Equatable {
    /// Sub-state of the cal flow.  Drives the iOS UI: SAMPLING shows a
    /// progress bar, REVIEW shows the fit + Accept/Retry buttons, APPLIED
    /// shows a one-shot success banner, ABORTED dismisses to settings.
    let subType: MagCalSubType

    /// Number of distinct directional wedges populated, out of 26 (3³ - 1
    /// — the (0,0,0) center cell is unreachable for a unit vector so we
    /// only count the 26 reachable wedges).  Hits ~26 on a clean tumble.
    let coverageBins: UInt8

    /// Total raw samples accumulated this run, capped at MAX_SAMPLES.
    let sampleCount: UInt16

    /// Magnitude of the most recent raw sample, in µT (post-IIS2MDC OFFSET
    /// chip subtract during SAMPLING — i.e. this is the *uncalibrated*
    /// reading the cal flow is trying to fix).  0 before the first sample.
    let instantaneousFieldUT: Float

    /// Fitted hard-iron offset in raw IIS2MDC LSB units (0.15 µT/LSB).
    /// Zero in SAMPLING/IDLE/ABORTED, populated in REVIEW/APPLIED.
    let offsetX: Int16
    let offsetY: Int16
    let offsetZ: Int16

    /// Fitted Earth-field magnitude in µT.  0 if no fit yet.  Should land
    /// in [20, 80] µT for the fit to pass the R-band sanity gate.
    let fieldR_uT: Float

    /// RMS residual of samples from the fitted sphere, in µT.  Smaller is
    /// better; values > MAG_CAL_MAX_RESIDUAL_UT (~8 µT) usually mean
    /// soft-iron distortion or a moving interferer during the tumble.
    let residualUT: Float

    /// 0 = fit accepted, non-zero = fit failed a gate; the iOS UI shows
    /// the right error and offers Retry instead of Accept.
    let rejectCode: MagCalRejectCode

    /// Bitmap of populated 3³ wedges; bit i corresponds to wedge i in
    /// directionWedge() (firmware-side TR_MagCalibrator).  Drives the
    /// per-direction progress grid and the gated orientation-prompt
    /// cycle.  Bit 13 is the unreachable centre cell and is always 0.
    /// 0 on old firmware builds that ship the 22-byte payload — UI
    /// then falls back to a simple timer cycle.
    let coverageMask: UInt32

    /// Live raw mag-vector components in µT (most recent sample).
    /// Lets the iOS UI render real-time direction feedback so the
    /// user can verify which axis is currently dominant and adjust
    /// the rocket to match the target orientation.  All zero on old
    /// firmware (< 32-byte payload).
    let liveX_uT: Float
    let liveY_uT: Float
    let liveZ_uT: Float

    /// Convenience: human-readable explanation for a non-zero rejectCode.
    var rejectMessage: String {
        switch rejectCode {
        case .ok:            return "Looks good"
        case .rTooLow:       return "Field magnitude too low — interference?"
        case .rTooHigh:      return "Field magnitude too high — magnet nearby?"
        case .highResidual:  return "Sphere fit poor — likely soft-iron distortion"
        case .lowCoverage:   return "Insufficient orientation coverage — keep tumbling"
        case .verifyFailed:
            // The frame stamps the worst observed |B| into instantaneousFieldUT
            // so we can show the user the actual number that tripped the gate.
            return String(format: "Verify failed — corrected |B| reached %.1f µT (need 20–70 µT, tight spread)",
                          instantaneousFieldUT)
        }
    }

    /// Convenience: progress fraction in [0, 1] for the SAMPLING UI.
    /// Combines the sample-count cap and the coverage gate so the bar
    /// only fills once both are healthy.
    func samplingProgress(targetSamples: UInt16, minCoverage: UInt8) -> Double {
        let s = min(Double(sampleCount) / Double(targetSamples), 1.0)
        let c = min(Double(coverageBins) / Double(minCoverage), 1.0)
        return min(s, c)
    }

    /// Decode the wire payload (bytes *after* the 0xCA discriminator).
    /// Accepts both the original 22-byte layout and the 26-byte layout
    /// that trails a uint32 coverage_mask — old firmware → coverageMask=0
    /// and UI falls back to a timer-only prompt cycle.  Returns nil if
    /// the buffer is too short to be the original payload.
    static func decode(_ bytes: [UInt8]) -> MagCalStatus? {
        guard bytes.count >= 22 else { return nil }

        // Helpers — Data slicing is alignment-safe even at odd offsets.
        func u16(_ offset: Int) -> UInt16 {
            return bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: offset, as: UInt16.self)
            }
        }
        func u32(_ offset: Int) -> UInt32 {
            return bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: offset, as: UInt32.self)
            }
        }
        func i16(_ offset: Int) -> Int16 {
            return bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: offset, as: Int16.self)
            }
        }

        // bytes[0..3]   = uint32 time_us (unused on iOS — we use phone clock for UI)
        let subTypeRaw    = bytes[4]
        let coverage      = bytes[5]
        let sampleCount   = u16(6)
        let instUTx10     = u16(8)
        let offX          = i16(10)
        let offY          = i16(12)
        let offZ          = i16(14)
        let RUTx10        = u16(16)
        let resUTx10      = u16(18)
        let rejectRaw     = bytes[20]
        // bytes[21]      = _pad
        // bytes[22..25]  = uint32 coverage_mask  (new in 26-byte payload)
        // bytes[26..31]  = int16 inst_x/y/z LSB   (new in 32-byte payload)
        let coverageMask: UInt32 = (bytes.count >= 26) ? u32(22) : 0
        let liveX_lsb: Int16 = (bytes.count >= 32) ? i16(26) : 0
        let liveY_lsb: Int16 = (bytes.count >= 32) ? i16(28) : 0
        let liveZ_lsb: Int16 = (bytes.count >= 32) ? i16(30) : 0

        // IIS2MDC sensitivity is 0.15 µT/LSB (datasheet 9.13).  Mirrors
        // the firmware-side IIS2MDC_LSB_TO_uT constant.
        let LSB_TO_uT: Float = 0.15

        let sub    = MagCalSubType(rawValue: subTypeRaw) ?? .idle
        let reject = MagCalRejectCode(rawValue: rejectRaw) ?? .ok

        return MagCalStatus(
            subType: sub,
            coverageBins: coverage,
            sampleCount: sampleCount,
            instantaneousFieldUT: Float(instUTx10) / 10.0,
            offsetX: offX,
            offsetY: offY,
            offsetZ: offZ,
            fieldR_uT: Float(RUTx10) / 10.0,
            residualUT: Float(resUTx10) / 10.0,
            rejectCode: reject,
            coverageMask: coverageMask,
            liveX_uT: Float(liveX_lsb) * LSB_TO_uT,
            liveY_uT: Float(liveY_lsb) * LSB_TO_uT,
            liveZ_uT: Float(liveZ_lsb) * LSB_TO_uT
        )
    }
}

// Six cardinal-axis "wedge caps" that the UI uses to drive the gated
// prompt cycle (issue #96 follow-up).  These are the firmware
// directionWedge() indices for unit vectors near ±X / ±Y / ±Z — see the
// 3×3×3 encoding in TR_MagCalibrator.cpp (bx*9 + by*3 + bz with each
// component in {0,1,2} for {<-T, [-T,T], >T}).
//
// Body-frame convention (empirical, verified by the user on the flight
// computer): nose-up populates the +X wedge.  Sides and faces follow
// from the right-hand rule around that — if any axis turns out to be
// flipped on the rocket, swap the +Y/+Z assignments here without
// touching the firmware.
enum MagCalAxis: CaseIterable {
    case noseUp, noseDown
    case rightSide, leftSide
    case frontFace, backFace

    /// Bit index inside MagCalStatus.coverageMask.
    var wedgeBit: UInt32 {
        switch self {
        // bx, by, bz ∈ {0,1,2} → index = bx*9 + by*3 + bz
        case .noseUp:    return 2 * 9 + 1 * 3 + 1  // (+,0,0) = 22  — user-confirmed: Up = +X
        case .noseDown:  return 0 * 9 + 1 * 3 + 1  // (-,0,0) =  4
        case .rightSide: return 1 * 9 + 2 * 3 + 1  // (0,+,0) = 16
        case .leftSide:  return 1 * 9 + 0 * 3 + 1  // (0,-,0) = 10
        case .frontFace: return 1 * 9 + 1 * 3 + 2  // (0,0,+) = 14
        case .backFace:  return 1 * 9 + 1 * 3 + 0  // (0,0,-) = 12
        }
    }

    /// Big arrow glyph for the prompt card.
    var icon: String {
        switch self {
        case .noseUp:    return "arrow.up"
        case .noseDown:  return "arrow.down"
        case .rightSide: return "arrow.right"
        case .leftSide:  return "arrow.left"
        case .frontFace: return "arrow.up.right"
        case .backFace:  return "arrow.down.left"
        }
    }

    /// Imperative direction for the prompt card.
    var prompt: String {
        switch self {
        case .noseUp:    return "Point the nose UP"
        case .noseDown:  return "Point the nose DOWN"
        case .rightSide: return "Lay it on its RIGHT side"
        case .leftSide:  return "Lay it on its LEFT side"
        case .frontFace: return "Tilt the FRONT face up"
        case .backFace:  return "Tilt the BACK face up"
        }
    }

    /// Short label for the per-direction status grid.
    var shortLabel: String {
        switch self {
        case .noseUp:    return "Up"
        case .noseDown:  return "Down"
        case .rightSide: return "Right"
        case .leftSide:  return "Left"
        case .frontFace: return "Front"
        case .backFace:  return "Back"
        }
    }

    /// Which body-frame axis (X/Y/Z) this orientation targets.
    var component: MagCalComponent {
        switch self {
        case .noseUp, .noseDown:     return .x
        case .rightSide, .leftSide:  return .y
        case .frontFace, .backFace:  return .z
        }
    }

    /// Sign the target component should have when correctly oriented.
    /// e.g. nose-up → +X (sign +1); nose-down → -X (sign -1).
    var sign: Float {
        switch self {
        case .noseUp, .rightSide, .frontFace: return  1.0
        case .noseDown, .leftSide, .backFace: return -1.0
        }
    }
}

/// Sensor-frame component axis labels for the direction-feedback bars.
enum MagCalComponent: String, CaseIterable {
    case x = "X"
    case y = "Y"
    case z = "Z"

    /// Pull the live µT reading for this component out of a status frame.
    func value(in status: MagCalStatus) -> Float {
        switch self {
        case .x: return status.liveX_uT
        case .y: return status.liveY_uT
        case .z: return status.liveZ_uT
        }
    }
}

extension MagCalStatus {
    /// True iff the wedge for `axis` has been populated in this run.
    func isAxisCovered(_ axis: MagCalAxis) -> Bool {
        return (coverageMask & (UInt32(1) << axis.wedgeBit)) != 0
    }
}

// Match firmware-side gates (RocketComputerTypes.h) so the UI doesn't
// drift from the FC's accept/reject decision.  The FC is the source of
// truth via rejectCode — these constants are just for the progress UI.
enum MagCalConstants {
    static let maxSamples: UInt16 = 2700   // 27 accel-wedges × 100 slots each
    static let minSamples: UInt16 = 500
    static let minCoverageBins: UInt8 = 18
}
