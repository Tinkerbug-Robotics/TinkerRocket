import XCTest
@testable import TinkerRocketApp

/// #382 display mapping: READY and PRELAUNCH are both "on the pad"; the app
/// shows one PRELAUNCH label whose COLOR carries the distinction (acquiring
/// orange, ready green). The text badge was removed 2026-07-31 — "EKF Ready"
/// beside an amber EKF health dot read as a contradiction; the sensor dots
/// own live quality. Wire strings are untouched — only display.
final class PadStateDisplayTests: XCTestCase {

    func testBothPadStatesDisplayAsPrelaunch() {
        XCTAssertEqual(RocketStateView.displayLabel(for: "READY"), "PRELAUNCH")
        XCTAssertEqual(RocketStateView.displayLabel(for: "PRELAUNCH"), "PRELAUNCH")
    }

    func testNonPadStatesUnchanged() {
        for s in ["INIT", "INFLIGHT", "COMPLETE", "MAG_CAL", "LANDED", "UNKNOWN"] {
            XCTAssertEqual(RocketStateView.displayLabel(for: s), s)
        }
    }

    func testPadStatesKeepDistinctColors() {
        // The banner color is now the ONLY carrier of the acquiring-vs-ready
        // distinction — pin that the two pad states never collapse to one.
        XCTAssertNotEqual(RocketStateView(state: "READY").stateColor,
                          RocketStateView(state: "PRELAUNCH").stateColor)
    }
}
