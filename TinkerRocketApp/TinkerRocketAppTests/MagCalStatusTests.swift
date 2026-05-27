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
    /// 2026-05-17 Eagle Claw historical residual was ~41 µT — well
    /// under the new 2500 µT "interesting" threshold.  Should land .ok
    /// because the new metric is absolute |c|, not |c|/R; small
    /// residuals stay quiet regardless of R.
    func testEagleClawHistoricalResidualOk() {
        let toLsb: (Float) -> Int16 = { Int16(($0 / 0.15).rounded()) }
        let s = makeStatus(
            offsetXYZ_lsb: (toLsb(-17.2), toLsb(30.8), toLsb(20.4)),
            R_uT: 48.7)
        XCTAssertEqual(s.centerMagnitudeUT, 41.0, accuracy: 0.5)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// Fresh new-PCB IIS2MDC fit: ~1700 µT board hard-iron.  Expected
    /// to be normal (.ok) — this is the typical board residual that
    /// the cal is supposed to absorb.
    func testFreshNewPCBOk() {
        // 1700 µT / 0.15 = 11333 LSB; pure-X for simplicity.
        let s = makeStatus(offsetXYZ_lsb: (11333, 0, 0), R_uT: 48.0)
        XCTAssertEqual(s.centerMagnitudeUT, 1700.0, accuracy: 1.0)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// Threshold transitions around the 2500 µT (ok→caution) and 4000
    /// µT (caution→high) boundaries.  Use explicit LSB values so the
    /// 0.15 µT/LSB quantization doesn't trip the test.
    func testWarningThresholds() {
        // 16666 lsb × 0.15 = 2499.9 µT → ok (under 2500).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (16666, 0, 0), R_uT: 50.0).centerWarning, .ok)
        // 16667 lsb × 0.15 = 2500.05 µT → caution (>= 2500).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (16667, 0, 0), R_uT: 50.0).centerWarning, .caution)
        // 26666 lsb × 0.15 = 3999.9 µT → still caution.
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (26666, 0, 0), R_uT: 50.0).centerWarning, .caution)
        // 26667 lsb × 0.15 = 4000.05 µT → high (>= 4000).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (26667, 0, 0), R_uT: 50.0).centerWarning, .high)
    }

}
