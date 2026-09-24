import XCTest
@testable import TinkerRocketApp

/// #1485: the 8k logging rates ("8k Dynamic" and a fixed 7680 Hz) need a
/// board whose flight computer reads the IMU from its FIFO. The rocket says
/// how fast it can go in the `config` frame's `"irmax"`; the settings screen
/// offers the 8k choices only where that covers them.
final class ImuRateCapabilityTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    func testIrmax_IsParsedWhenPresent() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":1,"irmax":7680}"#)
        XCTAssertEqual(rocket.rocketConfig?.imuRateMaxHz, 7680)
        XCTAssertEqual(rocket.rocketConfig?.imuRateHz, RocketProfile.imuRateDynamic8k)
    }

    func testIrmax_AbsentOnOlderFirmwareReadsAsNil() {
        // Firmware from before the 8k rates: the screen treats nil as 3840.
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":0}"#)
        XCTAssertNotNil(rocket.rocketConfig)
        XCTAssertNil(rocket.rocketConfig?.imuRateMaxHz)
    }

    func testChoices_MatchTheFirmwareSettings() {
        // RocketComputerTypes.h: IMU_RATE_DYNAMIC 0, IMU_RATE_DYNAMIC_8K 1,
        // IMU_RATE_OPTIONS_HZ {960, 1920, 3840, 7680}; imuRatePeakHz() is a
        // dynamic mode's boost rate and a fixed rate's own.
        let choices = RocketProfile.imuRateChoices
        XCTAssertEqual(choices.map(\.setting), [0, 1, 960, 1920, 3840, 7680])
        XCTAssertEqual(choices.map(\.peakHz), [3840, 7680, 960, 1920, 3840, 7680])
        XCTAssertEqual(Set(choices.map(\.label)).count, choices.count, "labels must be distinct")
        XCTAssertEqual(RocketProfile.imuRateDynamic, 0)
        XCTAssertEqual(RocketProfile.imuRateDynamic8k, 1)
    }

    func testChoices_ABaselineRocketSeesNoEightKRates() {
        let shown = RocketProfile.imuRateChoices.filter {
            $0.peakHz <= RocketProfile.imuRateBaselineMaxHz
        }
        XCTAssertEqual(shown.map(\.label), ["4k Dynamic", "1k", "2k", "4k"])
    }
}
