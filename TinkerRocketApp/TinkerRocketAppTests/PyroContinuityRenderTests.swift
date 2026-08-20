import XCTest
@testable import TinkerRocketApp

/// Regression tests for #828.
///
/// The Settings ▸ Pyro row computed its own two-state Bool
/// (`telemetry.pyroCont(channel:) && data_status == .live`) and drew a filled
/// red dot + "NO CONT" whenever it was false. That is exactly the collapse
/// `BLEDevice.pyroContinuity` exists to prevent: a channel that has simply
/// never been measured rendered as a confident MEASURED open — read by an
/// operator as a dead igniter or a spent charge, on a channel that is in fact
/// live. On the same frame the dashboard tile showed a hollow grey
/// "NOT TESTED" for that channel, so the two surfaces disagreed.
///
/// These pin the shared accessor both surfaces now go through. The view layer
/// is not unit-testable here, so the contract is enforced at its source.
final class PyroContinuityRenderTests: XCTestCase {

    private func rocket(health: Int, ps: Int = 0, ds: Int? = nil) -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        var json = #"{"st":"READY","h":\#(health),"ps":\#(ps)"#
        if let ds { json += #","ds":\#(ds)"# }
        json += "}"
        d.parseTelemetryData(json.data(using: .utf8)!)
        return d
    }

    /// The #828 defect, stated directly: ch1 measured OK means the firmware
    /// reports measured bits, so ch3's .na is a real "never tested this
    /// session" — and must never render as an open circuit.
    func testNeverMeasuredChannelIsUntestedNotOpen() {
        let d = rocket(health: 1 << 24)                  // ch1 = .ok, ch3 = .na
        XCTAssertEqual(d.pyroContinuity(channel: 3), .untested,
                       "a never-measured channel must not read as a MEASURED open")
        XCTAssertNotEqual(d.pyroContinuity(channel: 3), .open)
    }

    func testMeasuredOpenIsOpen() {
        let d = rocket(health: 3 << 28)                  // ch3 = .bad
        XCTAssertEqual(d.pyroContinuity(channel: 3), .open,
                       "red is reserved for this case")
    }

    func testMeasuredContinuityIsPresent() {
        let d = rocket(health: 1 << 24)
        XCTAssertEqual(d.pyroContinuity(channel: 1), .present)
    }

    /// The old row ANDed in `data_status == .live` and fell to red. #297
    /// fail-safe is "no trustworthy reading", which is .noData (hollow grey),
    /// not a measured open.
    func testStaleFrameIsNoDataNotOpen() {
        // ds raw 1 = .stale (see TelemetryData.DataStatus).
        let d = rocket(health: 1 << 24, ds: 1)
        XCTAssertEqual(d.pyroContinuity(channel: 1), .noData,
                       "a stale frame yields no trustworthy reading")
        XCTAssertNotEqual(d.pyroContinuity(channel: 1), .open)
    }

    func testDisconnectedIsNoData() {
        let d = rocket(health: 1 << 24)
        d.isConnected = false
        XCTAssertEqual(d.pyroContinuity(channel: 1), .noData)
    }

    /// Firmware predating SH_PYRO_MEAS reports no measured bits at all, so the
    /// raw cont bit is the only source and the two-state collapse is
    /// unavoidable there. Pinned so the fallback is not "improved" into
    /// reporting .untested forever against older rockets.
    func testLegacyFirmwareFallsBackToTheRawContBit() {
        // No measured bits (h carries only a non-pyro sensor), ps bit set for
        // ch1 continuity. ps layout: b0 = armed, then (cont, fired) per channel.
        let contCh1 = 1 << 1
        let d = rocket(health: 1 << 2, ps: contCh1)
        XCTAssertNil(d.telemetry.pyroMeasuredContinuity(channel: 1),
                     "test premise: this frame has no measured bits")
        XCTAssertEqual(d.pyroContinuity(channel: 1), .present)
        XCTAssertEqual(d.pyroContinuity(channel: 2), .open,
                       "legacy path genuinely cannot tell open from untested")
    }
}
