import XCTest
@testable import TinkerRocketApp

/// #1231: the `config_pyro` readback says where its values came from, and the
/// optimistic write mirror (#1078) yields to the flight computer's own echo.
final class PyroReadbackProvenanceTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        BLEDevice(peripheral: nil, name: "TR-R-Test")
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    func testProvenance_ParsesSourceAndStoredFlag() {
        let rocket = makeRocket()
        feed(rocket, """
            {"type":"config_pyro","p1e":true,"p1m":0,"p1v":1.0,"src":"fc","fnv":false}
            """)
        XCTAssertEqual(rocket.rocketConfig?.pyroSource, .flightComputer)
        XCTAssertEqual(rocket.rocketConfig?.pyroStoredOnFlightComputer, false,
                       "the FC is reporting its never-configured default")
        XCTAssertEqual(rocket.rocketConfig?.pyroIsFlightComputerSourced, true)

        // A `config` rebuild carries provenance over with the pyro fields.
        feed(rocket, #"{"type":"config","shz":333}"#)
        XCTAssertEqual(rocket.rocketConfig?.pyroSource, .flightComputer)
        XCTAssertEqual(rocket.rocketConfig?.pyroStoredOnFlightComputer, false)

        // The OC's own cache: "fnv" has no meaning there.
        feed(rocket, #"{"type":"config_pyro","p1e":true,"src":"oc","fnv":true}"#)
        XCTAssertEqual(rocket.rocketConfig?.pyroSource, .outComputerCache)
        XCTAssertNil(rocket.rocketConfig?.pyroStoredOnFlightComputer)

        // An out computer that predates the key.
        feed(rocket, #"{"type":"config_pyro","p1e":true}"#)
        XCTAssertEqual(rocket.rocketConfig?.pyroSource, .unknown)
        XCTAssertNil(rocket.rocketConfig?.pyroStoredOnFlightComputer)
    }

    func testWriteMirror_YieldsToFlightComputerEcho() {
        let rocket = makeRocket()
        let channels: [(enabled: Bool, mode: UInt8, value: Float)] =
            [(true, 1, 150), (false, 0, 1), (false, 0, 0), (false, 0, 0)]

        // FC-sourced and the rail is on: the FC's report will echo the write,
        // so the tiles must NOT jump ahead of it.
        feed(rocket, #"{"st":"READY","fs":16}"#)   // 0x10 = pwr_pin_on
        feed(rocket, #"{"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0,"src":"fc","fnv":true}"#)
        rocket.sendPyroConfig(channels: channels)
        XCTAssertEqual(rocket.rocketConfig?.pyro1Enabled, false,
                       "no optimistic mirror: the tiles wait for the FC's echo")

        // The OC's cache: no echo is coming, so the #1078 mirror still applies.
        feed(rocket, #"{"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0,"src":"oc"}"#)
        rocket.sendPyroConfig(channels: channels)
        XCTAssertEqual(rocket.rocketConfig?.pyro1Enabled, true)

        // FC-sourced but the rail has since gone off: nothing can echo.
        feed(rocket, #"{"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0,"src":"fc","fnv":true}"#)
        feed(rocket, #"{"st":"READY","fs":0}"#)
        rocket.sendPyroConfig(channels: channels)
        XCTAssertEqual(rocket.rocketConfig?.pyro1Enabled, true)

        // Pre-#1231 out computer: mirror, as before.
        feed(rocket, #"{"st":"READY","fs":16}"#)
        feed(rocket, #"{"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0}"#)
        rocket.sendPyroConfig(channels: channels)
        XCTAssertEqual(rocket.rocketConfig?.pyro1Enabled, true)
    }
}
