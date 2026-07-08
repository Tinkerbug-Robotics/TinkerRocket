import XCTest
@testable import TinkerRocketApp

/// #382 display mapping: READY and PRELAUNCH are both "on the pad"; the app
/// shows one PRELAUNCH label with a readiness badge instead of two states
/// whose names read backwards. Wire strings are untouched — only display.
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

    func testBadgeSemantics() {
        let ready = RocketStateView.padBadge(for: "PRELAUNCH")
        XCTAssertEqual(ready?.0, "EKF Ready")
        XCTAssertEqual(ready?.1, true)

        let acquiring = RocketStateView.padBadge(for: "READY")
        XCTAssertEqual(acquiring?.1, false)

        XCTAssertNil(RocketStateView.padBadge(for: "INFLIGHT"))
        XCTAssertNil(RocketStateView.padBadge(for: "COMPLETE"))
    }
}
