import XCTest
@testable import TinkerRocketApp

/// The GNSS antenna offset (cmd 81): from the flight computer's IMU to the
/// GNSS antenna, in millimeters along the board's own axes, or Unknown (the
/// default). Sent as `[known u8][x_mm i16][y_mm i16][z_mm i16]`; the out
/// computer reports it as the config frame's "antk"/"antx"/"anty"/"antz",
/// all four absent on firmware without the setting.
final class AntennaOffsetSettingTests: XCTestCase {

    private var tempDir: URL!
    private var defaults: UserDefaults!

    override func setUp() {
        super.setUp()
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        defaults = UserDefaults(suiteName: "antenna-offset-tests-\(UUID().uuidString)")
    }

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

    /// "antsrc": "fc" when the four keys are the flight computer's own
    /// report, "oc" when they are the out computer's copy of the last cmd 81
    /// it relayed (the rail off, or a flight computer too old to report it).
    /// Only "fc" counts; absent is an out computer that predates the key,
    /// which can only hold its copy.
    func testTheReadbackSaysWhoseOffsetItIs() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","antk":1,"antx":312,"anty":-25,"antz":4,"antsrc":"fc"}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(312, -25, 4))
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, true)

        feed(rocket, #"{"type":"config","antk":1,"antx":312,"anty":-25,"antz":4,"antsrc":"oc"}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(312, -25, 4))
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)

        // A fresh readback says it again: the source never carries over.
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":"fc"}"#)
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, AntennaOffset.unknown)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)

        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":"FC"}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":1}"#)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)
        // No offset, nothing to vouch for.
        feed(rocket, #"{"type":"config","antsrc":"fc"}"#)
        XCTAssertNil(rocket.rocketConfig?.antennaOffset)
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)
    }

    /// The cached readback follows a send at once, but as what was sent, not
    /// as the flight computer's report: the screen says so until the next
    /// readback does.
    func testASentOffsetIsNotTheFlightComputersReportYet() {
        let rocket = makeRocket()
        rocket.commandTap = { _, _ in }
        feed(rocket, #"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":"fc"}"#)
        rocket.sendAntennaOffsetConfig(measured(312, -25, 4))
        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(312, -25, 4))
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, false)
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

    /// A readback carrying `offset` as the flight computer's own report.
    private func fcReport(_ offset: AntennaOffset) -> RocketConfig {
        var cfg = RocketConfig()
        cfg.antennaOffset = offset
        cfg.antennaOffsetFromFlightComputer = true
        return cfg
    }

    func testTheRocketsOffsetIsAdopted() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        let changed = ActiveRocketSyncer.adopt(&p, from: fcReport(measured(312, -25, 4)))
        XCTAssertTrue(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, measured(312, -25, 4))
    }

    func testTheRocketsUnknownIsAdopted() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: fcReport(.unknown))
        XCTAssertTrue(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, AntennaOffset.unknown)
    }

    func testAMatchingOffsetIsNotAChange() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: fcReport(measured(312, -25, 4)))
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    /// Only the flight computer's own report ("antsrc":"fc"). The out
    /// computer's copy of the last cmd 81 it relayed — the rail off, or a
    /// flight computer too old to report it — is not adopted, whatever it
    /// holds.
    func testTheOutComputersCopyIsNotAdopted() {
        for copy in [AntennaOffset.unknown, measured(10, 20, 30)] {
            var p = RocketProfile.makeDefault(name: "Goblin")
            p.antennaOffset = measured(312, -25, 4)
            var cfg = RocketConfig()
            cfg.antennaOffset = copy
            cfg.antennaOffsetFromFlightComputer = false
            let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
            XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset), "\(copy)")
            XCTAssertEqual(p.antennaOffset, measured(312, -25, 4), "\(copy)")
        }
    }

    /// The same through the readback parser: what "oc" or no "antsrc" brings
    /// in is left out of the profile, what "fc" brings in is taken.
    func testOnlyAnFcReadbackIsAdopted() throws {
        for (frame, adopted) in [
            (#"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":"oc"}"#, false),
            (#"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0}"#, false),
            (#"{"type":"config","antk":0,"antx":0,"anty":0,"antz":0,"antsrc":"fc"}"#, true),
        ] {
            let rocket = makeRocket()
            feed(rocket, frame)
            var p = RocketProfile.makeDefault(name: "Goblin")
            p.antennaOffset = measured(312, -25, 4)
            let cfg = try XCTUnwrap(rocket.rocketConfig)
            let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
            XCTAssertEqual(changed.contains(ActiveRocketSyncer.groupAntennaOffset), adopted, frame)
            XCTAssertEqual(p.antennaOffset, adopted ? .unknown : measured(312, -25, 4), frame)
        }
    }

    func testARocketThatDoesNotReportItLeavesTheProfileAlone() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(312, -25, 4)
        let changed = ActiveRocketSyncer.adopt(&p, from: RocketConfig())
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, measured(312, -25, 4))
    }

    /// takeAntenna false: the extras re-adopt's call once the late antenna
    /// adopt owns it.
    func testWithoutTheAntennaTheProfileKeepsItsOffsetAndItIsNotNamed() {
        var p = RocketProfile.makeDefault(name: "Goblin")
        p.antennaOffset = measured(50, 60, 70)
        let changed = ActiveRocketSyncer.adopt(&p, from: fcReport(measured(10, 20, 30)), takeAntenna: false)
        XCTAssertFalse(changed.contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(p.antennaOffset, measured(50, 60, 70))
    }

    // MARK: The syncer, end to end

    private static let identityFrame =
        #"{"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,"rid":1,"dt":"R","fw":"test"}"#

    private static func configFrame(_ antennaKeys: String) -> String {
        #"{"type":"config","# + antennaKeys + "}"
    }

    private func settleMainQueue(_ turns: Int = 5) {
        for _ in 0..<turns {
            let exp = expectation(description: "main-queue turn")
            DispatchQueue.main.async { exp.fulfill() }
            wait(for: [exp], timeout: 1)
        }
    }

    /// The board's own profile holding `offset`, and the rocket connected
    /// with a readback carrying `antennaKeys`: the connect-time sync is done.
    private func connect(profile offset: AntennaOffset, antennaKeys: String)
        -> (ActiveRocketSyncer, RocketProfileStore, BLEDevice) {
        let syncer = ActiveRocketSyncer()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "boardA"; $0.antennaOffset = offset }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(antennaKeys))
        feed(rocket, Self.identityFrame)
        settleMainQueue()
        return (syncer, store, rocket)
    }

    private func adoptedGroups(_ syncer: ActiveRocketSyncer) -> [String] {
        if case .adopted(let groups) = syncer.syncState { return groups }
        return []
    }

    /// Connected with the rail off: only the out computer's copy is on offer,
    /// so the profile keeps its own; when the flight computer's report lands
    /// (the out computer sends the readback again) its offset is adopted, and
    /// said so — the camera's late adopt.
    func testRailOffAtConnect_TheFlightComputersOffsetIsAdoptedWhenItArrives() {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(312, -25, 4))
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))

        feed(rocket, Self.configFrame(#""antk":1,"antx":100,"anty":0,"antz":-50,"antsrc":"fc""#))
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(100, 0, -50),
                       "the flight computer's own offset is the rocket's")
        XCTAssertTrue(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    /// The same, with the flight computer agreeing with the profile: nothing to say.
    func testRailOffAtConnect_AFlightComputerThatAgreesIsNoAdoption() {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)
        let before = syncer.syncState

        feed(rocket, Self.configFrame(#""antk":1,"antx":312,"anty":-25,"antz":4,"antsrc":"fc""#))
        settleMainQueue()

        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, true)
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(312, -25, 4))
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(syncer.syncState, before)
    }

    /// Switching profiles disarms it: the profile now active was not the one
    /// reconciled at connect, and was just sent to the rocket as it stands.
    func testRailOffAtConnect_AProfileSwitchDisarmsTheLateAdopt() throws {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)
        let first = try XCTUnwrap(store.activeId)

        let second = store.add(name: "Backup")
        store.setActive(second.id)
        settleMainQueue()
        feed(rocket, Self.configFrame(#""antk":1,"antx":100,"anty":0,"antz":-50,"antsrc":"fc""#))
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.antennaOffset, AntennaOffset.unknown)
        XCTAssertEqual(store.profiles.first { $0.id == first }?.antennaOffset, measured(312, -25, 4))
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    /// The user sets the offset after it armed: with the rail off the cmd 81
    /// waits in the out computer, and the flight computer's boot report still
    /// carries its old value as its own.  That report is not the rocket
    /// disagreeing with the phone: the user's value stands, nothing is said,
    /// and the one-shot is spent.
    func testRailOffAtConnect_AnOffsetTheUserSetSinceIsNotOverwrittenByTheLateReport() throws {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)
        let before = syncer.syncState

        // The settings screen's commit: the profile, then cmd 81.
        let id = try XCTUnwrap(store.activeId)
        var sent: [(UInt8, Data)] = []
        rocket.commandTap = { sent.append(($0, $1)) }
        store.update(id) { $0.antennaOffset = measured(50, 60, 70) }
        rocket.sendAntennaOffsetConfig(measured(50, 60, 70))
        settleMainQueue()
        XCTAssertEqual(sent.map { $0.0 }, [81])
        XCTAssertEqual(sent.first?.1, measured(50, 60, 70).payload)

        feed(rocket, Self.configFrame(#""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"fc""#))
        settleMainQueue()

        XCTAssertEqual(rocket.rocketConfig?.antennaOffset, measured(10, 20, 30))
        XCTAssertEqual(rocket.rocketConfig?.antennaOffsetFromFlightComputer, true)
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(50, 60, 70),
                       "the user's offset stands")
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
        XCTAssertEqual(syncer.syncState, before)

        // Spent: a later report of the flight computer's is not adopted either.
        feed(rocket, Self.configFrame(#""antk":1,"antx":100,"anty":0,"antz":-50,"antsrc":"fc""#))
        settleMainQueue()
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(50, 60, 70))
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    /// The three #915 config-report frames the out computer relays after the
    /// config frame, each one a copy of the current readback plus its group.
    private func feedConfigReport(_ rocket: BLEDevice, servoBias2: Int) {
        feed(rocket, #"{"type":"config_servo","sb2":"# + "\(servoBias2)"
             + #","sb3":0,"sb4":0,"fmn":-60.00,"fmx":60.00,"faz":[0.0,90.0,180.0,270.0],"frv":0,"frrv":0}"#)
        feed(rocket, #"{"type":"config_guid","gng":3.00,"gma":30.0,"gaf":0.50,"gmf":15.0,"gms":30.0,"gcd":0,"gtm":0,"gte":0.0,"gtn":0.0,"gta":0.0,"gkp":0.00,"gkd":0.00,"glw":0}"#)
        feed(rocket, #"{"type":"config_roll","n":0,"wp":[]}"#)
    }

    /// The same user edit, with the report groups also missing at connect
    /// (the out computer's normal boot state): the extras re-adopt is armed
    /// beside the antenna's.  When the rail comes up the flight computer's
    /// boot report (its old offset, as "fc") lands, then the report groups,
    /// which carry it along; the extras re-adopt takes those groups but not
    /// the antenna, which belongs to the late antenna adopt.  The user's
    /// value stands.
    func testRailOffAtConnect_TheExtrasReAdoptDoesNotOverwriteAnOffsetTheUserSetSince() throws {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)
        XCTAssertEqual(rocket.rocketConfig?.configReportGroupsMissing, true, "no report groups at connect")
        XCTAssertEqual(store.activeProfile?.servoBias2, 0)

        let id = try XCTUnwrap(store.activeId)
        store.update(id) { $0.antennaOffset = measured(50, 60, 70) }
        rocket.sendAntennaOffsetConfig(measured(50, 60, 70))
        settleMainQueue()

        feed(rocket, Self.configFrame(#""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"fc""#))
        settleMainQueue()
        feedConfigReport(rocket, servoBias2: 40)
        settleMainQueue()

        let cfg = try XCTUnwrap(rocket.rocketConfig)
        XCTAssertFalse(cfg.configReportGroupsMissing)
        XCTAssertEqual(cfg.antennaOffset, measured(10, 20, 30))
        XCTAssertTrue(cfg.antennaOffsetFromFlightComputer)
        XCTAssertEqual(store.activeProfile?.servoBias2, 40, "the extras re-adopt ran")
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(50, 60, 70),
                       "the user's offset stands")
        XCTAssertFalse(adoptedGroups(syncer).contains(ActiveRocketSyncer.groupAntennaOffset))
    }

    /// The extras re-adopt still takes the other groups, and adds what it
    /// changed to what was already said: here the late antenna adopt's
    /// "GNSS antenna offset", which it used to replace.
    func testRailOffAtConnect_TheExtrasReAdoptTakesTheOtherGroupsAndAddsToTheAdoptedSet() {
        let (syncer, store, rocket) = connect(
            profile: measured(312, -25, 4),
            antennaKeys: #""antk":1,"antx":10,"anty":20,"antz":30,"antsrc":"oc""#)

        feed(rocket, Self.configFrame(#""antk":1,"antx":100,"anty":0,"antz":-50,"antsrc":"fc""#))
        settleMainQueue()
        let said = adoptedGroups(syncer)
        XCTAssertTrue(said.contains(ActiveRocketSyncer.groupAntennaOffset))

        feedConfigReport(rocket, servoBias2: 40)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.servoBias2, 40)
        XCTAssertEqual(store.activeProfile?.antennaOffset, measured(100, 0, -50))
        let groups = adoptedGroups(syncer)
        XCTAssertEqual(Array(groups.prefix(said.count)), said, "what was said stands: \(groups)")
        XCTAssertTrue(groups.contains(ActiveRocketSyncer.groupServoTrim24), "\(groups)")
        XCTAssertEqual(Set(groups).count, groups.count, "\(groups)")
    }
}
