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
        XCTAssertEqual(d.pyroContinuity(channel: 1), .present,
                       "a SET cont bit is unambiguous: measured, and continuous")
        // Corrected 2026-08-22. This used to assert .open, pinning the idea
        // that the legacy path "cannot tell open from untested" — but the FC
        // only ever sets the bit as `cont_known && cont_state`, so a clear bit
        // is precisely that ambiguity and must not be rendered as red.
        XCTAssertEqual(d.pyroContinuity(channel: 2), .untested,
                       "a CLEAR cont bit is ambiguous — never red")
    }
}

// MARK: - #831: freshness on the direct path

extension PyroContinuityRenderTests {

    /// A link that has never delivered a frame is not a reading. Before this,
    /// `effectiveDataStatus` was a pass-through on a direct link and the
    /// rocket only ever sends "ds" when NOT live — so the #297 guard reduced
    /// to `isConnected` and this returned a verdict from zeroed telemetry.
    func testConnectedButNoFrameYetIsNoData() {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        XCTAssertNil(d.lastTelemetryAt, "test premise: no frame has arrived")
        XCTAssertEqual(d.pyroContinuity(channel: 1), .noData,
                       "a connected link with no frames must not report continuity")
    }

    /// The #831 hold-over, from the app's side: frames stop while the link
    /// still counts as connected, and the last good reading must not stand.
    func testSilentLinkGoesNoDataOnceStale() {
        let d = rocket(health: 1 << 24)
        XCTAssertEqual(d.pyroContinuity(channel: 1), .present,
                       "premise: it reads green while the frame is fresh")

        let seen = d.lastTelemetryAt!
        d.nowProvider = { seen.addingTimeInterval(
            Double(BLEDevice.telemetryStaleThresholdMs) / 1000 + 1) }

        XCTAssertEqual(d.pyroContinuity(channel: 1), .noData,
                       "a held-over green is exactly what #297 exists to prevent")
    }

    func testFreshFrameJustInsideTheWindowStillReads() {
        let d = rocket(health: 1 << 24)
        let seen = d.lastTelemetryAt!
        d.nowProvider = { seen.addingTimeInterval(
            Double(BLEDevice.telemetryStaleThresholdMs) / 1000 - 0.5) }
        XCTAssertEqual(d.pyroContinuity(channel: 1), .present)
    }

    /// The OC now ages its own FC snapshot and sends STALE, which the existing
    /// guard already honours — this pins that the two layers agree.
    func testRocketReportedStaleIsNoDataEvenWithAFreshFrame() {
        let d = rocket(health: 1 << 24, ds: 1)   // ds 1 = .stale
        XCTAssertNotNil(d.lastTelemetryAt, "premise: the BLE frame itself is fresh")
        XCTAssertEqual(d.pyroContinuity(channel: 1), .noData)
    }

    /// SYNCING is what the OC sends before the FC has ever spoken. It must not
    /// fall through to the raw-cont-bit path and render a confident red.
    func testRocketReportedSyncingIsNoDataNotOpen() {
        let d = rocket(health: 0, ds: 2)         // ds 2 = .syncing
        XCTAssertEqual(d.pyroContinuity(channel: 3), .noData)
        XCTAssertNotEqual(d.pyroContinuity(channel: 3), .open)
    }
}

// MARK: - #831: no evidence is not a measured open

extension PyroContinuityRenderTests {

    /// The OC's low-power frames (FC rail off) carry no pyro_status and no
    /// sensor_health at all. A clear cont bit there is absence of evidence,
    /// not a measured open — red is reserved for a measurement.
    func testRocketPoweredOffIsUntestedNotOpen() {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        d.parseTelemetryData(#"{"st":"OFF"}"#.data(using: .utf8)!)
        XCTAssertFalse(d.telemetry.hasSensorHealth, "premise: an all-zero scorecard")
        XCTAssertEqual(d.pyroContinuity(channel: 1), .untested)
        XCTAssertNotEqual(d.pyroContinuity(channel: 1), .open,
                          "a powered-off rocket has not measured an open circuit")
    }

    /// A populated scorecard does not make a clear cont bit a measurement.
    /// This asserted .open until 2026-08-22; the bench showed that is exactly
    /// the state a rocket sits in before its first continuity test.
    func testPopulatedScorecardStillDoesNotInventAnOpen() {
        let d = rocket(health: 1 << 2, ps: 0)     // a non-pyro sensor reporting
        XCTAssertTrue(d.telemetry.hasSensorHealth)
        XCTAssertNil(d.telemetry.pyroMeasuredContinuity(channel: 2))
        XCTAssertEqual(d.pyroContinuity(channel: 2), .untested)
    }

    /// The exact frame the bench OC sent on 2026-08-22 with nothing yet
    /// measured — sensor_health populated by the other sensors, all four
    /// SH_PYRO_MEAS fields NA. Every channel rendered a confident red before
    /// this fix. This is the state EVERY session starts in.
    func testBenchFrameWithNothingMeasuredIsUntestedOnEveryChannel() {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Bench")
        d.isConnected = true
        d.parseTelemetryData(#"{"st":"READY","fs":16,"h":1092981}"#.data(using: .utf8)!)

        XCTAssertNil(d.telemetry.pyroMeasuredContinuity(channel: 1),
                     "premise: nothing measured, so this returns nil for every channel")
        XCTAssertTrue(d.telemetry.hasSensorHealth,
                      "premise: the scorecard IS populated by the other sensors")
        for ch in 1...4 {
            XCTAssertEqual(d.pyroContinuity(channel: ch), .untested,
                           "ch\(ch): never measured must never render as a measured open")
        }
    }
}
