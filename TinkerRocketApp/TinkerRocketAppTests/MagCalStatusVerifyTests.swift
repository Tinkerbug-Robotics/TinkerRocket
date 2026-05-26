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

    /// On verify failure the FC publishes sub_type=REVIEW with
    /// reject_code=verifyFailed and stamps the worst |B| into
    /// inst_field_uT_x10 so the message can quote it.
    func testDecodeVerifyFailedFrame() {
        var bytes = [UInt8](repeating: 0, count: 32)
        bytes[4] = MagCalSubType.review.rawValue
        bytes[5] = 6
        bytes[6] = 0xC8; bytes[7] = 0x00          // sample_count = 200
        // inst_field_uT_x10 = 890 → 0x7A 0x03  (worst observed 89.0 µT — Eagle Claw)
        bytes[8] = 0x7A; bytes[9] = 0x03
        // bytes 10..15 offset = 0 (FC restored prior cal already)
        // bytes 16..17 field_R = 0
        // bytes 18..19 residual = 0
        bytes[20] = MagCalRejectCode.verifyFailed.rawValue

        guard let s = MagCalStatus.decode(bytes) else {
            return XCTFail("decode returned nil for verify-failed frame")
        }
        XCTAssertEqual(s.subType, .review)
        XCTAssertEqual(s.rejectCode, .verifyFailed)
        XCTAssertEqual(s.instantaneousFieldUT, 89.0, accuracy: 0.01)
        XCTAssertTrue(s.rejectMessage.contains("89.0"),
                      "rejectMessage should quote worst |B|, got: \(s.rejectMessage)")
        XCTAssertTrue(s.rejectMessage.contains("20") && s.rejectMessage.contains("70"),
                      "rejectMessage should call out the [20, 70] µT trust band, got: \(s.rejectMessage)")
    }

    /// Round-trip: every defined MagCalRejectCode value has a non-empty
    /// rejectMessage — guards against the switch losing exhaustiveness
    /// when a new case is added.
    func testAllRejectCodesHaveMessages() {
        let allCodes: [MagCalRejectCode] = [.ok, .rTooLow, .rTooHigh,
                                            .highResidual, .lowCoverage,
                                            .verifyFailed]
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
                liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0)
            XCTAssertFalse(s.rejectMessage.isEmpty,
                           "Missing rejectMessage for \(code)")
        }
    }
}
