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
            liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0,
            partialMask: 0
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

    /// 2026-05-17 Eagle Claw historical residual was ~41 µT — well
    /// under the chip-limit warning threshold.  Should land .ok.
    func testEagleClawHistoricalResidualOk() {
        let toLsb: (Float) -> Int16 = { Int16(($0 / 0.15).rounded()) }
        let s = makeStatus(
            offsetXYZ_lsb: (toLsb(-17.2), toLsb(30.8), toLsb(20.4)),
            R_uT: 48.7)
        XCTAssertEqual(s.centerMagnitudeUT, 41.0, accuracy: 0.5)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// Fresh new-PCB IIS2MDC fit: ~1700 µT board hard-iron — typical,
    /// nowhere near the chip's ±4915 µT OFFSET register limit, so the
    /// warning stays quiet (.ok).
    func testFreshNewPCBOk() {
        // 1700 µT / 0.15 = 11333 LSB; pure-X for simplicity.
        let s = makeStatus(offsetXYZ_lsb: (11333, 0, 0), R_uT: 48.0)
        XCTAssertEqual(s.centerMagnitudeUT, 1700.0, accuracy: 1.0)
        XCTAssertEqual(s.centerWarning, .ok)
    }

    /// Threshold transition at 4500 µT — the only failure mode for
    /// |c| alone is approaching the chip's ±4915 µT OFFSET register
    /// range.  Below the threshold, every plausible PCB hard-iron
    /// reading stays green.
    func testWarningThresholds() {
        // 29999 lsb × 0.15 = 4499.85 µT → ok (just under).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (29999, 0, 0), R_uT: 50.0).centerWarning, .ok)
        // 30000 lsb × 0.15 = 4500.00 µT → high (== threshold).
        XCTAssertEqual(makeStatus(offsetXYZ_lsb: (30000, 0, 0), R_uT: 50.0).centerWarning, .high)
    }

}
