import XCTest
@testable import TinkerRocketApp

/// #236: the LoRa-data plot crashed with "Range requires lowerBound <=
/// upperBound" — an inverted `ClosedRange` reaching Swift Charts from a gesture
/// or near-degenerate domain.  Every chart domain and zoom window now goes
/// through `FlightChartView.safeDomain`, which can never produce an inverted
/// (or non-finite, or zero-width) range.
final class FlightChartViewTests: XCTestCase {

    func testSafeDomain_NormalRangeUnchanged() {
        let d = FlightChartView.safeDomain(0, 100)
        XCTAssertEqual(d.lowerBound, 0)
        XCTAssertEqual(d.upperBound, 100)
    }

    /// The exact #236 trap: lower > upper.  Must not crash; comes back ordered.
    func testSafeDomain_ReversedIsCorrected() {
        let d = FlightChartView.safeDomain(100, 0)
        XCTAssertEqual(d.lowerBound, 0)
        XCTAssertEqual(d.upperBound, 100)
    }

    func testSafeDomain_ZeroWidthGetsMinWidth() {
        let d = FlightChartView.safeDomain(5, 5)
        XCTAssertEqual(d.lowerBound, 5)
        XCTAssertGreaterThan(d.upperBound, d.lowerBound)   // never zero-width
    }

    func testSafeDomain_NonFiniteFallsBack() {
        for bad in [Double.nan, .infinity, -.infinity] {
            XCTAssertEqual(FlightChartView.safeDomain(bad, 10), 0...1)
            XCTAssertEqual(FlightChartView.safeDomain(0, bad), 0...1)
        }
    }

    /// Sweep adversarial (a, b) pairs — the invariant that matters is that the
    /// result is always a valid, finite, non-empty ClosedRange (never traps).
    func testSafeDomain_AlwaysValidRange() {
        let vals: [Double] = [-1e9, -1, 0, 1e-12, 1, 100, 1e9]
        for a in vals {
            for b in vals {
                let d = FlightChartView.safeDomain(a, b)
                XCTAssertLessThanOrEqual(d.lowerBound, d.upperBound)
                XCTAssertTrue(d.lowerBound.isFinite && d.upperBound.isFinite)
            }
        }
    }
}
