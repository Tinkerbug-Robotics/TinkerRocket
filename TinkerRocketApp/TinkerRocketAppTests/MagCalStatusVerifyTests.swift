import XCTest
@testable import TinkerRocketApp

/// #206 — locks down the wire-format mapping for the new
/// MagCalSubType.verifying / MagCalRejectCode.verifyFailed values,
/// plus the rejectMessage format that surfaces the worst observed |B|.
final class MagCalStatusVerifyTests: XCTestCase {

    /// Build a synthetic frame with the verifying sub-type set on a fresh
    /// post-accept window — checks decode() preserves the new enum case.
    func testDecodeVerifyingSubType() {
        var bytes = [UInt8](repeating: 0, count: 32)
        // bytes[0..3] = time_us (any)
        bytes[4] = MagCalSubType.verifying.rawValue   // sub_type
        bytes[5] = 6                                  // coverage_bins
        // sample_count = 250 → 0xFA 0x00
        bytes[6] = 0xFA; bytes[7] = 0x00
        // inst_field_uT_x10 = 555 (55.5 µT) → 0x2B 0x02
        bytes[8] = 0x2B; bytes[9] = 0x02
        // offset_x/y/z / R / residual / reject all zero during VERIFYING
        guard let s = MagCalStatus.decode(bytes) else {
            return XCTFail("decode returned nil for 32-byte verifying frame")
        }
        XCTAssertEqual(s.subType, .verifying)
        XCTAssertEqual(s.sampleCount, 250)
        XCTAssertEqual(s.coverageBins, 6)
        XCTAssertEqual(s.instantaneousFieldUT, 55.5, accuracy: 0.01)
        XCTAssertEqual(s.rejectCode, .ok)
    }

    /// Verify-too-high frame: FC stamps the observed max |B| into
    /// inst_field_uT_x10 and the message names the 70 µT cap.
    func testDecodeVerifyTooHighFrame() {
        var bytes = [UInt8](repeating: 0, count: 32)
        bytes[4] = MagCalSubType.review.rawValue
        bytes[5] = 6
        bytes[6] = 0xC8; bytes[7] = 0x00          // sample_count = 200
        // inst_field_uT_x10 = 890 → 0x7A 0x03  (worst observed 89.0 µT — Eagle Claw)
        bytes[8] = 0x7A; bytes[9] = 0x03
        bytes[20] = MagCalRejectCode.verifyTooHigh.rawValue

        guard let s = MagCalStatus.decode(bytes) else {
            return XCTFail("decode returned nil for verify-too-high frame")
        }
        XCTAssertEqual(s.subType, .review)
        XCTAssertEqual(s.rejectCode, .verifyTooHigh)
        XCTAssertEqual(s.instantaneousFieldUT, 89.0, accuracy: 0.01)
        XCTAssertTrue(s.rejectMessage.contains("89.0"),
                      "rejectMessage should quote worst |B|, got: \(s.rejectMessage)")
        XCTAssertTrue(s.rejectMessage.contains("70"),
                      "rejectMessage should call out the 70 µT cap, got: \(s.rejectMessage)")
    }

    /// Verify-low-coverage frame: the user didn't rotate enough during
    /// verify.  Message should explain that, not quote a |B| value.
    func testDecodeVerifyLowCoverageFrame() {
        var bytes = [UInt8](repeating: 0, count: 32)
        bytes[4] = MagCalSubType.review.rawValue
        bytes[20] = MagCalRejectCode.verifyLowCoverage.rawValue

        guard let s = MagCalStatus.decode(bytes) else {
            return XCTFail("decode returned nil")
        }
        XCTAssertEqual(s.rejectCode, .verifyLowCoverage)
        XCTAssertTrue(s.rejectMessage.lowercased().contains("rotat") ||
                      s.rejectMessage.lowercased().contains("orientation"),
                      "rejectMessage should tell the user to rotate more, got: \(s.rejectMessage)")
    }

    /// Round-trip: every defined MagCalRejectCode value has a non-empty
    /// rejectMessage — guards against the switch losing exhaustiveness
    /// when a new case is added.
    func testAllRejectCodesHaveMessages() {
        let allCodes: [MagCalRejectCode] = [.ok, .rTooLow, .rTooHigh,
                                            .highResidual, .lowCoverage,
                                            .verifyFailed,
                                            .verifyTooHigh, .verifyTooLow,
                                            .verifyRangeWide,
                                            .verifyLowCoverage, .verifyFewSamples]
        for code in allCodes {
            let s = MagCalStatus(
                subType: .review,
                coverageBins: 0,
                sampleCount: 0,
                instantaneousFieldUT: 50.0,
                offsetX: 0, offsetY: 0, offsetZ: 0,
                fieldR_uT: 50.0,
                residualUT: 0.0,
                rejectCode: code,
                coverageMask: 0,
                liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0,
                partialMask: 0)
            XCTAssertFalse(s.rejectMessage.isEmpty,
                           "Missing rejectMessage for \(code)")
        }
    }
}
