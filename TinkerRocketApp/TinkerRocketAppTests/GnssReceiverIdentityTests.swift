import XCTest
@testable import TinkerRocketApp

/// The V9 finds its GNSS receiver at boot (the u-blox SAM-M10Q carrier or a
/// Septentrio mosaic-G5 on J1), and the out computer relays which one the
/// flight computer took as fc_identity's "fc_gnss", so the pad shows it.
/// With the mosaic the rocket also lowers "irmax" to 3840, which the
/// settings screen already follows.
final class GnssReceiverIdentityTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    func testTheReceiverIsParsed() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"fc_identity","fc_fw":"abc-v9+20261017","fc_board":"V9","oc_board":"V9","fc_gnss":"mosaic-G5"}"#)
        XCTAssertEqual(rocket.fcGnss, "mosaic-G5")
        XCTAssertEqual(rocket.fcBoardRev, "V9")
    }

    func testNoReceiverClearsIt() {
        // The FC came back GNSS-absent: "" is a real answer.
        let rocket = makeRocket()
        feed(rocket, #"{"type":"fc_identity","fc_fw":"abc","fc_board":"V9","oc_board":"V9","fc_gnss":"u-blox"}"#)
        feed(rocket, #"{"type":"fc_identity","fc_fw":"abc","fc_board":"V9","oc_board":"V9","fc_gnss":""}"#)
        XCTAssertNil(rocket.fcGnss)
    }

    func testAnOlderOutComputerLeavesItAlone() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"fc_identity","fc_fw":"abc","fc_board":"V9","oc_board":"V9","fc_gnss":"u-blox"}"#)
        feed(rocket, #"{"type":"fc_identity","fc_fw":"abc","fc_board":"V9","oc_board":"V9"}"#)
        XCTAssertEqual(rocket.fcGnss, "u-blox")
        let fresh = makeRocket()
        feed(fresh, #"{"type":"fc_identity","fc_fw":"abc","fc_board":"V9","oc_board":"V9"}"#)
        XCTAssertNil(fresh.fcGnss)
    }

    func testTheMosaicsLimitReachesTheRateMenu() {
        // What the OC sends once the FC names the mosaic: 8k Dynamic flies as
        // 4k Dynamic, and only the 4k choices fit.
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":1,"irmax":7680}"#)
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840}"#)
        XCTAssertEqual(rocket.rocketConfig?.imuRateMaxHz, 3840)
        XCTAssertEqual(rocket.rocketConfig?.imuRateHz, RocketProfile.imuRateDynamic)
        let shown = RocketProfile.imuRateChoices.filter { $0.peakHz <= 3840 }
        XCTAssertEqual(shown.map(\.label), ["4k Dynamic", "1k", "2k", "4k"])
    }
}
