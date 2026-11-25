import XCTest
@testable import TinkerRocketApp

/// The GNSS log setting (cmd 79): with a Septentrio mosaic-G5, either all of
/// its data in the log and the IMU at most 4k ("Detailed", the default), or
/// its position and once-a-second raw measurements with the 8k rates open
/// ("Reduced"). The out computer reports the setting as the config frame's
/// "glog" and moves "irmax" with it, which the rate menu already follows.
final class GnssLogSettingTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    func testTheValuesMatchTheFirmware() {
        // RocketComputerTypes.h: GNSS_LOG_DETAILED 0, GNSS_LOG_REDUCED 1.
        XCTAssertEqual(RocketProfile.gnssLogDetailed, 0)
        XCTAssertEqual(RocketProfile.gnssLogReduced, 1)
        XCTAssertEqual(RocketProfile.makeDefault(name: "x").gnssLogSetting, RocketProfile.gnssLogDetailed)
    }

    func testGlogIsParsed() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":1,"irmax":7680,"glog":1}"#)
        XCTAssertEqual(rocket.rocketConfig?.gnssLogSetting, RocketProfile.gnssLogReduced)
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":0}"#)
        XCTAssertEqual(rocket.rocketConfig?.gnssLogSetting, RocketProfile.gnssLogDetailed)
    }

    func testOlderFirmwareAndUnknownValuesReadAsNotReported() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840}"#)
        XCTAssertNotNil(rocket.rocketConfig)
        XCTAssertNil(rocket.rocketConfig?.gnssLogSetting)
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":7}"#)
        XCTAssertNil(rocket.rocketConfig?.gnssLogSetting)
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":-1}"#)
        XCTAssertNil(rocket.rocketConfig?.gnssLogSetting)
    }

    func testTheReducedLogOpensTheEightKRates() {
        // What the OC sends after cmd 79 with a mosaic-G5: the readback again,
        // with the board's own limit, and the kept 8k setting flying again.
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":0}"#)
        feed(rocket, #"{"type":"config","irate":1,"irmax":7680,"glog":1}"#)
        XCTAssertEqual(rocket.rocketConfig?.imuRateMaxHz, 7680)
        XCTAssertEqual(rocket.rocketConfig?.imuRateHz, RocketProfile.imuRateDynamic8k)
        let shown = RocketProfile.imuRateChoices.filter { $0.peakHz <= 7680 }
        XCTAssertEqual(shown.map(\.label), ["4k Dynamic", "8k Dynamic", "1k", "2k", "4k", "8k"])
    }

    func testTheProfileKeepsIt() throws {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.gnssLogSetting = RocketProfile.gnssLogReduced
        let data = try JSONEncoder().encode(p)
        let back = try JSONDecoder().decode(RocketProfile.self, from: data)
        XCTAssertEqual(back.gnssLogSetting, RocketProfile.gnssLogReduced)
    }

    func testAProfileFromBeforeTheSettingIsDetailed() throws {
        let legacy = #"{"name":"Old","imuRateHz":0}"#.data(using: .utf8)!
        XCTAssertEqual(try JSONDecoder().decode(RocketProfile.self, from: legacy).gnssLogSetting,
                       RocketProfile.gnssLogDetailed)
        let corrupt = #"{"name":"Odd","gnssLogSetting":9}"#.data(using: .utf8)!
        XCTAssertEqual(try JSONDecoder().decode(RocketProfile.self, from: corrupt).gnssLogSetting,
                       RocketProfile.gnssLogDetailed)
    }

    func testTheRocketsSettingIsAdopted() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        var cfg = RocketConfig()
        cfg.gnssLogSetting = RocketProfile.gnssLogReduced
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertTrue(changed.contains(ActiveRocketSyncer.groupGnssLog))
        XCTAssertEqual(p.gnssLogSetting, RocketProfile.gnssLogReduced)
    }

    func testARocketThatDoesNotReportItLeavesTheProfileAlone() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.gnssLogSetting = RocketProfile.gnssLogReduced
        let changed = ActiveRocketSyncer.adopt(&p, from: RocketConfig())
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupGnssLog))
        XCTAssertEqual(p.gnssLogSetting, RocketProfile.gnssLogReduced)
    }

    func testSendingItIsCommand79WithOneByte() {
        let rocket = makeRocket()
        var sent: [(UInt8, Data)] = []
        rocket.commandTap = { sent.append(($0, $1)) }
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":0}"#)
        rocket.sendGnssLogConfig(RocketProfile.gnssLogReduced)
        XCTAssertEqual(sent.count, 1)
        XCTAssertEqual(sent.first?.0, 79)
        XCTAssertEqual(sent.first?.1, Data([1]))
        // The cached readback follows at once, like the other config senders.
        XCTAssertEqual(rocket.rocketConfig?.gnssLogSetting, RocketProfile.gnssLogReduced)
    }
}
