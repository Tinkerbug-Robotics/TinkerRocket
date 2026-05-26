import XCTest
@testable import TinkerRocketApp

/// Locks down the hard-iron center-magnitude math and the warning-bucket
/// thresholds added for issue #207.  The thresholds (0.3 caution, 0.5 high)
/// were picked from the 2026-05-17 field flights — Eagle Claw 0.84 and
/// RIM-66 0.67 — so we hard-code those scenarios as fixtures here.
final class MagCalStatusTests: XCTestCase {

    /// Build a synthetic REVIEW-state status with hand-set offset / R fields.
    /// Other fields are irrelevant to the warning math.
    private func makeStatus(offsetXYZ_lsb: (Int16, Int16, Int16),
                            R_uT: Float) -> MagCalStatus {
        return MagCalStatus(
            subType: .review,
            coverageBins: 26,
            sampleCount: 2000,
            instantaneousFieldUT: R_uT,
            offsetX: offsetXYZ_lsb.0,
            offsetY: offsetXYZ_lsb.1,
            offsetZ: offsetXYZ_lsb.2,
            fieldR_uT: R_uT,
            residualUT: 2.0,
            rejectCode: .ok,
            coverageMask: 0,
            liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0
        )
    }

    // 0.15 µT/LSB on the IIS2MDC; sqrt(100² + 100² + 100²) * 0.15 ≈ 25.98 µT.
    func testCenterMagnitudeBasic() {
        let s = makeStatus(offsetXYZ_lsb: (100, 100, 100), R_uT: 50.0)
        XCTAssertEqual(s.centerMagnitudeUT, 25.98, accuracy: 0.01)
        XCTAssertEqual(s.centerToRRatio, 25.98 / 50.0, accuracy: 0.001)
    }

    func testCenterMagnitudeZero() {
        let s = makeStatus(offsetXYZ_lsb: (0, 0, 0), R_uT: 50.0)
        XCTAssertEqual(s.centerMagnitudeUT, 0.0)
        XCTAssertEqual(s.centerToRRatio, 0.0)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// R == 0 (no fit yet) must not divide-by-zero — ratio defined as 0.
    func testCenterToRRatioZeroR() {
        let s = makeStatus(offsetXYZ_lsb: (100, 0, 0), R_uT: 0.0)
        XCTAssertGreaterThan(s.centerMagnitudeUT, 0)
        XCTAssertEqual(s.centerToRRatio, 0.0)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// 2026-05-17 Eagle Claw — recovered hard-iron (-17, +31, +20) µT body
    /// frame against R = 48.7 µT → ratio ~0.84 → should land in .high.
    /// Convert µT back to LSB: divide by 0.15.
    func testEagleClawWarningHigh() {
        let toLsb: (Float) -> Int16 = { Int16(($0 / 0.15).rounded()) }
        let s = makeStatus(
            offsetXYZ_lsb: (toLsb(-17.2), toLsb(30.8), toLsb(20.4)),
            R_uT: 48.7)
        XCTAssertEqual(s.centerMagnitudeUT, 41.0, accuracy: 0.5)
        XCTAssertEqual(s.centerToRRatio, 0.84, accuracy: 0.02)
        XCTAssertEqual(s.centerWarning, .high)
        XCTAssertNotNil(s.centerWarning.helpText)
    }

    /// 2026-05-17 RIM-66 — recovered (-9, +11, +30) µT vs R = 49.2 µT → 0.67
    /// → should also land in .high (above the 0.5 threshold).
    func testRIM66WarningHigh() {
        let toLsb: (Float) -> Int16 = { Int16(($0 / 0.15).rounded()) }
        let s = makeStatus(
            offsetXYZ_lsb: (toLsb(-8.8), toLsb(11.2), toLsb(29.5)),
            R_uT: 49.2)
        XCTAssertEqual(s.centerToRRatio, 0.67, accuracy: 0.02)
        XCTAssertEqual(s.centerWarning, .high)
    }

    /// Threshold transitions around the 0.30 (ok→caution) and 0.50
    /// (caution→high) boundaries.  Use R=100 µT and explicit LSB values
    /// so the 0.15 µT/LSB quantization doesn't trip the test — at 0.15
    /// µT/LSB the smallest representable |c| step is 0.15 µT (= 0.0015
    /// ratio at R=100), so we pick offsets that land cleanly either side
    /// of each threshold.
    func testWarningThresholds() {
        // 199 lsb × 0.15 = 29.85 µT → 0.2985 ratio → ok (under 0.30).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (199, 0, 0), R_uT: 100.0).centerWarning, .ok)
        // 200 lsb × 0.15 = 30.00 µT → 0.300 ratio → caution (>= 0.30).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (200, 0, 0), R_uT: 100.0).centerWarning, .caution)
        // 333 lsb × 0.15 = 49.95 µT → 0.4995 ratio → still caution.
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (333, 0, 0), R_uT: 100.0).centerWarning, .caution)
        // 334 lsb × 0.15 = 50.10 µT → 0.501 ratio → high (>= 0.50).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (334, 0, 0), R_uT: 100.0).centerWarning, .high)
    }

    /// .ok should have no help text; non-.ok should have non-empty help.
    func testHelpTextPresence() {
        XCTAssertNil(MagCalStatus.CenterWarning.ok.helpText)
        XCTAssertFalse(MagCalStatus.CenterWarning.caution.helpText?.isEmpty ?? true)
        XCTAssertFalse(MagCalStatus.CenterWarning.high.helpText?.isEmpty ?? true)
    }
}
