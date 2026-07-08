import XCTest
@testable import TinkerRocketApp

/// Regression tests for #377: a freshly-(re)connected BLEDevice starts with a
/// zeroed TelemetryData, which reads as pwr_pin_on == false — indistinguishable
/// from a genuinely powered-off rocket. In that window the dashboard used to
/// show the "Power On" button, and cmd 8 is a blind toggle: one tap powered an
/// already-on rocket OFF (then the 180 s poweringOn spinner masked it).
/// `hasReceivedTelemetry` is the "power state is confirmed" gate the UI now
/// holds on.
final class PowerStateGateTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-R-Test")
    }

    /// fs bit 0x10 is pwr_pin_on (see TelemetryData.pwr_pin_on).
    private func frame(fs: Int) -> Data {
        """
        {"st":"READY","fs":\(fs)}
        """.data(using: .utf8)!
    }

    func testFreshDevice_PowerStateUnknown() {
        let rocket = makeRocket()
        XCTAssertFalse(rocket.hasReceivedTelemetry,
                       "no frame yet — power state must read as unknown, not off")
        XCTAssertFalse(rocket.telemetry.pwr_pin_on,
                       "zeroed default telemetry (the #377 trap this flag guards)")
    }

    func testFirstFrame_ConfirmsPowerOn() {
        let rocket = makeRocket()
        rocket.parseTelemetryData(frame(fs: 0x10))
        XCTAssertTrue(rocket.hasReceivedTelemetry)
        XCTAssertTrue(rocket.telemetry.pwr_pin_on,
                      "an already-on rocket is confirmed on by its first frame")
    }

    func testFirstFrame_ConfirmsPowerOff() {
        let rocket = makeRocket()
        rocket.parseTelemetryData(frame(fs: 0))
        XCTAssertTrue(rocket.hasReceivedTelemetry,
                      "a genuinely-off rocket still sends OC frames — state becomes KNOWN off")
        XCTAssertFalse(rocket.telemetry.pwr_pin_on)
    }

    func testRelayedFrame_AlsoConfirms() {
        // The BS relay path decodes through the same entry point; the gate
        // must flip there too.
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.parseTelemetryData("""
        {"rid":7,"run":"Atlas","st":"READY","fs":16}
        """.data(using: .utf8)!)
        XCTAssertTrue(bs.hasReceivedTelemetry)
    }

    func testDisconnect_ResetsToUnknown() {
        let rocket = makeRocket()
        rocket.parseTelemetryData(frame(fs: 0x10))
        XCTAssertTrue(rocket.hasReceivedTelemetry)
        rocket.onDisconnect()
        XCTAssertFalse(rocket.hasReceivedTelemetry,
                       "next session's power state is unknown until its first frame")
    }

    func testGarbageFrame_DoesNotConfirm() {
        let rocket = makeRocket()
        rocket.parseTelemetryData("not json".data(using: .utf8)!)
        XCTAssertFalse(rocket.hasReceivedTelemetry,
                       "a frame that fails to decode confirms nothing")
    }
}
