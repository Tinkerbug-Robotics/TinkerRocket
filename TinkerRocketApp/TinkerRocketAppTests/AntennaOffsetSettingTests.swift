import XCTest
@testable import TinkerRocketApp

/// The GNSS antenna offset (cmd 81): from the flight computer's IMU to the
/// GNSS antenna, in millimeters along the board's own axes, or Unknown (the
/// default). Sent as `[known u8][x_mm i16][y_mm i16][z_mm i16]`; the out
/// computer reports it as the config frame's "antk"/"antx"/"anty"/"antz",
/// all four absent on firmware without the setting.
final class AntennaOffsetSettingTests: XCTestCase {

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    private func measured(_ x: Int16, _ y: Int16, _ z: Int16) -> AntennaOffset {
        AntennaOffset(known: true, xMm: x, yMm: y, zMm: z)
    }

    // MARK: Defaults

    func testANewProfileIsUnknown() {
        let p = RocketProfile.makeDefault(name: "x")
        XCTAssertFalse(p.antennaOffsetKnown)
        XCTAssertEqual(p.antennaXMm, 0)
        XCTAssertEqual(p.antennaYMm, 0)
        XCTAssertEqual(p.antennaZMm, 0)
        XCTAssertEqual(p.antennaOffset, AntennaOffset.unknown)
    }

    func testUnknownAlwaysStoresZeros() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        XCTAssertTrue(p.antennaOffsetKnown)
        p.antennaOffset = AntennaOffset(known: false, xMm: 7, yMm: 8, zMm: 9)
        XCTAssertFalse(p.antennaOffsetKnown)
        XCTAssertEqual(p.antennaXMm, 0)
        XCTAssertEqual(p.antennaYMm, 0)
        XCTAssertEqual(p.antennaZMm, 0)
    }

    // MARK: The command

    func testSendingItIsCommand80WithSevenLittleEndianBytes() {
        let rocket = makeRocket()
        var sent: [(UInt8, Data)] = []
        rocket.commandTap = { sent.append(($0, $1)) }
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0}"#)
        rocket.sendAntennaOffsetConfig(measured(312, -25, 4))
        XCTAssertEqual(sent.count, 1)
        XCTAssertEqual(sent.first?.0, 81)
        // 312 = 0x0138, -25 = 0xFFE7, 4 = 0x0004.
        let expected = Data([0x01, 0x38, 0x01, 0xE7, 0xFF, 0x04, 0x00])
        XCTAssertEqual(sent.first?.1, expected)
        // The cached readback follows at once, like the other config senders.
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(312, -25, 4))
    }

    func testTheLimitsEncode() {
        let low = measured(-2000, 2000, 0).payload
        XCTAssertEqual(low, Data([0x01, 0x30, 0xF8, 0xD0, 0x07, 0x00, 0x00]))
    }

    func testUnknownGoesOutAsZeros() {
        let rocket = makeRocket()
        var sent: [(UInt8, Data)] = []
        rocket.commandTap = { sent.append(($0, $1)) }
        rocket.sendAntennaOffsetConfig(.unknown)
        XCTAssertEqual(sent.first?.0, 81)
        XCTAssertEqual(sent.first?.1, Data([0, 0, 0, 0, 0, 0, 0]))
    }

    func testAnOffsetTheFirmwareWouldRefuseIsNeverSent() {
        let rocket = makeRocket()
        var sent: [(UInt8, Data)] = []
        rocket.commandTap = { sent.append(($0, $1)) }
        rocket.sendAntennaOffsetConfig(measured(2001, 0, 0))
        rocket.sendAntennaOffsetConfig(measured(0, -2001, 0))
        XCTAssertTrue(sent.isEmpty)
    }

    func testSendingToFirmwareWithoutItDoesNotPretendItHasIt() {
        let rocket = makeRocket()
        rocket.commandTap = { _, _ in }
        feed(rocket, #"{"type":"config","irate":0}"#)
        rocket.sendAntennaOffsetConfig(measured(10, 20, 30))
        XCTAssertNotNil(rocket.rocketConfig)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
    }

    // MARK: Typed fields

    func testTypedMillimeters() {
        XCTAssertEqual(AntennaOffset.parseMm("312"), 312)
        XCTAssertEqual(AntennaOffset.parseMm("-25"), -25)
        XCTAssertEqual(AntennaOffset.parseMm(" 4 "), 4)
        XCTAssertEqual(AntennaOffset.parseMm("2000"), 2000)
        XCTAssertEqual(AntennaOffset.parseMm("-2000"), -2000)
        XCTAssertNil(AntennaOffset.parseMm("2001"))
        XCTAssertNil(AntennaOffset.parseMm("-2001"))
        XCTAssertNil(AntennaOffset.parseMm("12.5"))
        XCTAssertNil(AntennaOffset.parseMm(""))
        XCTAssertNil(AntennaOffset.parseMm("abc"))
        XCTAssertNil(AntennaOffset.parseMm("99999999999999999999"))
    }

    // MARK: The readback

    func testTheReadbackIsParsed() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":1,"antx":312,"anty":-25,"antz":4}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(312, -25, 4))
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, AntennaOffset.unknown)
    }

    func testOlderFirmwareReadsAsNotReported() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","irate":0,"irmax":3840,"glog":0}"#)
        XCTAssertNotNil(rocket.rocketConfig)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
    }

    func testAMissingKeyReadsAsNotReported() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":1,"antx":312,"anty":-25}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antx":312,"anty":-25,"antz":4}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
    }

    func testValuesTheFirmwareWouldRefuseReadAsNotReported() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":2,"antx":0,"anty":0,"antz":0}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antk":-1,"antx":0,"anty":0,"antz":0}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antk":1,"antx":2001,"anty":0,"antz":0}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antk":1,"antx":0,"anty":0,"antz":-2001}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antk":1,"antx":"12","anty":0,"antz":0}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        feed(rocket, #"{"type":"config","antk":1,"antx":1.5,"anty":0,"antz":0}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
    }

    func testTheLimitsLand() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":1,"antx":-2000,"anty":2000,"antz":0}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(-2000, 2000, 0))
    }

    func testAnUnknownReportCarriesZeros() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":0,"antx":5,"anty":6,"antz":7}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, AntennaOffset.unknown)
    }

    // MARK: The profile

    func testTheProfileKeepsIt() throws {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let data = try JSONEncoder().encode(p)
        let back = try JSONDecoder().decode(RocketProfile.self, from: data)
        XCTAssertEqual(back.antennaOffset, measured(312, -25, 4))
        XCTAssertEqual(back.antennaXMm, 312)
        XCTAssertEqual(back.antennaYMm, -25)
        XCTAssertEqual(back.antennaZMm, 4)
    }

    func testTheExportCarriesIt() throws {
        // #297: the config export is the encoded profile.
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let data = try JSONEncoder().encode(p)
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        XCTAssertEqual(json["antennaOffsetKnown"] as? Bool, true)
        XCTAssertEqual(json["antennaXMm"] as? Int, 312)
        XCTAssertEqual(json["antennaYMm"] as? Int, -25)
        XCTAssertEqual(json["antennaZMm"] as? Int, 4)
    }

    func testAProfileFromBeforeTheSettingIsUnknown() throws {
        let legacy = #"{"name":"Old","imuRateHz":0}"#.data(using: .utf8)!
        let p = try JSONDecoder().decode(RocketProfile.self, from: legacy)
        XCTAssertEqual(p.antennaOffset, AntennaOffset.unknown)
    }

    func testACorruptStoredOffsetIsUnknownAndKeepsTheProfile() throws {
        let cases = [
            #"{"name":"Odd","antennaOffsetKnown":true,"antennaXMm":5000,"antennaYMm":0,"antennaZMm":0}"#,
            #"{"name":"Odd","antennaOffsetKnown":true,"antennaXMm":99999,"antennaYMm":0,"antennaZMm":0}"#,
            #"{"name":"Odd","antennaOffsetKnown":true,"antennaXMm":1,"antennaYMm":2}"#,
            #"{"name":"Odd","antennaOffsetKnown":"yes","antennaXMm":1,"antennaYMm":2,"antennaZMm":3}"#,
            #"{"name":"Odd","antennaOffsetKnown":false,"antennaXMm":1,"antennaYMm":2,"antennaZMm":3}"#,
        ]
        for text in cases {
            let p = try JSONDecoder().decode(RocketProfile.self, from: text.data(using: .utf8)!)
            XCTAssertEqual(p.name, "Odd")
            XCTAssertEqual(p.antennaOffset, AntennaOffset.unknown, text)
            XCTAssertEqual(p.antennaXMm, 0, text)
        }
    }

    // MARK: The syncer

    func testTheRocketsOffsetIsAdopted() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        var cfg = RocketConfig()
        cfg.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertTrue(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, measured(312, -25, 4))
    }

    func testTheRocketsUnknownIsAdopted() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        var cfg = RocketConfig()
        cfg.antennaOffset = .unknown
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertTrue(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, AntennaOffset.unknown)
    }

    func testAMatchingOffsetIsNotAChange() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        var cfg = RocketConfig()
        cfg.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    func testARocketThatDoesNotReportItLeavesTheProfileAlone() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: RocketConfig())
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, measured(312, -25, 4))
    }
}
