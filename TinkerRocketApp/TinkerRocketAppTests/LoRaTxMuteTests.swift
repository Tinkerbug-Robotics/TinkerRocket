import XCTest
@testable import TinkerRocketApp

/// "LoRa off" — the rocket's transmit mute, as the app sees it.
///
/// The whole feature rests on one three-state value: `RocketConfig.loraTxDisabled`
/// is `true` (muted), `false` (on the air), or `nil` (firmware predates the
/// setting).  Collapsing `nil` into `false` is the failure mode worth a test:
/// it would make the dashboard advisory and the settings switch report a radio
/// state the rocket never claimed, on exactly the older firmware that cannot
/// be commanded either way.
final class LoRaTxMuteTests: XCTestCase {

    private func config(_ json: String) -> RocketConfig? {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.parseTelemetryData(json.data(using: .utf8))
        return d.rocketConfig
    }

    func testLtxdTrueDecodesAsMuted() {
        XCTAssertEqual(config(#"{"type":"config","ltxd":true}"#)?.loraTxDisabled, true)
    }

    func testLtxdFalseDecodesAsTransmitting() {
        XCTAssertEqual(config(#"{"type":"config","ltxd":false}"#)?.loraTxDisabled, false)
    }

    func testLtxdAbsentStaysNilNotFalse() {
        // A config readback from firmware without the feature.  nil is the
        // only honest answer — the app must not claim the radio is on.
        let cfg = config(#"{"type":"config","lf":915.0,"lsf":8}"#)
        XCTAssertNotNil(cfg)
        XCTAssertNil(cfg?.loraTxDisabled)
    }

    func testAdvisoryShowsOnlyForAPositiveMute() {
        // The dashboard line is gated on `== true`, so both other states must
        // leave it hidden.  Written against the same expression the view uses.
        XCTAssertTrue(config(#"{"type":"config","ltxd":true}"#)?.loraTxDisabled == true)
        XCTAssertFalse(config(#"{"type":"config","ltxd":false}"#)?.loraTxDisabled == true)
        XCTAssertFalse(config(#"{"type":"config"}"#)?.loraTxDisabled == true)
    }

    func testMuteIsRebuiltFromEachReadbackNotCarriedOver() {
        // A `config` frame rebuilds RocketConfig from defaults (iOS decode
        // rule), so a later readback WITHOUT the key must not keep reporting a
        // stale mute — that would survive a firmware downgrade as a phantom.
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.parseTelemetryData(#"{"type":"config","ltxd":true}"#.data(using: .utf8))
        XCTAssertEqual(d.rocketConfig?.loraTxDisabled, true)
        d.parseTelemetryData(#"{"type":"config","lsf":8}"#.data(using: .utf8))
        XCTAssertNil(d.rocketConfig?.loraTxDisabled)
    }
}
