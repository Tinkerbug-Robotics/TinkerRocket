import XCTest
@testable import TinkerRocketApp

/// #1775: the app's copy of the rocket's pyro trigger check (firmware
/// PyroConfigGate.h), the pyro editors' value parse, and the send hold-back.
final class PyroTriggerRuleTests: XCTestCase {

    // MARK: - The rule

    func testDelay_ZeroTo120SecondsIsAccepted() {
        for v: Float in [0, -0.0, 0.5, 1.5, 100, 120] {
            XCTAssertTrue(PyroTriggerRule.accepts(mode: 0, value: v), "\(v)")
        }
        for v: Float in [-0.001, -1, 120.001, 121, 1500, .infinity, .nan] {
            XCTAssertFalse(PyroTriggerRule.accepts(mode: 0, value: v), "\(v)")
        }
    }

    func testAltitude_30MetersOrMoreIsAccepted() {
        for v: Float in [30, 30.48, 150, 3000] {
            XCTAssertTrue(PyroTriggerRule.accepts(mode: 1, value: v), "\(v)")
        }
        for v: Float in [29.99, 29.87, 0, -0.0, -5, .nan, -.infinity, .infinity] {
            XCTAssertFalse(PyroTriggerRule.accepts(mode: 1, value: v), "\(v)")
        }
    }

    func testOtherModesAreRefused() {
        for mode: UInt8 in [2, 3, 255] {
            XCTAssertFalse(PyroTriggerRule.accepts(mode: mode, value: 1), "\(mode)")
        }
    }

    func testFirstRefused_NamesTheFirstFailingEnabledChannel() {
        XCTAssertNil(PyroTriggerRule.firstRefused(
            [(true, 0, 0), (true, 1, 150), (false, 1, 0), (false, 0, 0)]))
        XCTAssertEqual(PyroTriggerRule.firstRefused(
            [(true, 0, 0), (false, 1, 0), (true, 1, 10), (true, 2, 1)]), 3)
        XCTAssertEqual(PyroTriggerRule.notSentText(channel: 3),
                       "Pyro settings not sent: channel 3's trigger is outside what the rocket "
                       + "accepts, so it kept its own. Everything else was sent.")
    }

    func testAnyRefused_ChecksOnlyEnabledChannels() {
        XCTAssertFalse(PyroTriggerRule.anyRefused(
            [(true, 0, 0), (true, 1, 150), (false, 1, 0), (false, 7, .nan)]))
        XCTAssertTrue(PyroTriggerRule.anyRefused(
            [(true, 0, 0), (true, 1, 0), (false, 0, 0), (false, 0, 0)]))
    }

    func testTheProfileDefaultsPassEnabled() {
        let p = RocketProfile(name: "Test")
        XCTAssertFalse(PyroTriggerRule.anyRefused([
            (true, p.pyro1TriggerMode, p.pyro1TriggerValue),
            (true, p.pyro2TriggerMode, p.pyro2TriggerValue),
            (true, p.pyro3TriggerMode, p.pyro3TriggerValue),
            (true, p.pyro4TriggerMode, p.pyro4TriggerValue),
        ]))
    }

    // MARK: - The range text

    func testRangeText_TheAltitudeShownIsItselfAccepted() {
        XCTAssertEqual(PyroTriggerRule.rangeText(mode: 0, unitSystem: .metric),
                       "The rocket accepts 0 to 120 s.")
        XCTAssertEqual(PyroTriggerRule.rangeText(mode: 1, unitSystem: .metric),
                       "The rocket accepts 30 m or more.")
        // 30 m is 98.4 ft: 98 ft is under the floor, so the text says 99.
        XCTAssertEqual(PyroTriggerRule.rangeText(mode: 1, unitSystem: .imperial),
                       "The rocket accepts 99 ft or more.")
        XCTAssertTrue(PyroTriggerRule.accepts(
            mode: 1, value: Float(UnitFormatter.displayToMeters(99, system: .imperial))))
        XCTAssertFalse(PyroTriggerRule.accepts(
            mode: 1, value: Float(UnitFormatter.displayToMeters(98, system: .imperial))))
        XCTAssertEqual(PyroTriggerRule.rangeText(mode: 2, unitSystem: .metric), "Choose a trigger.")
    }

    // MARK: - The editors' parse

    private func send(_ text: String, mode: UInt8 = 1, unit: UnitSystem = .metric,
                      loadedText: String = "150", loadedMode: UInt8 = 1,
                      loadedValue: Float = 150) -> Float? {
        PyroTriggerRule.valueToSend(text: text, mode: mode, unitSystem: unit,
                                    loadedText: loadedText, loadedMode: loadedMode,
                                    loadedValue: loadedValue)
    }

    func testValueToSend_AClearedOrUnparseableFieldIsNotSentAsZero() {
        // The dashboard sheet sent `Float(s) ?? 0` for these.
        XCTAssertNil(send(""))
        XCTAssertNil(send("  "))
        XCTAssertNil(send("abc"))
        XCTAssertNil(send("nan"))
        XCTAssertNil(send("inf"))
        XCTAssertNil(send("1e40"))  // overflows to infinity
    }

    func testValueToSend_ACommaIsTheDecimalPoint() {
        // A comma-decimal locale's decimal pad types "1,5"; Float() does not parse it.
        XCTAssertEqual(send("1,5", mode: 0, loadedText: "1.0", loadedMode: 0, loadedValue: 1), 1.5)
        XCTAssertEqual(send("152,5"), 152.5)
        XCTAssertNil(send("1,000.5"))
    }

    func testValueToSend_UnchangedTextKeepsTheStoredValue() {
        // 30 m shows as "98" ft; parsed back it would be 29.87 m, under the floor.
        XCTAssertEqual(send("98", unit: .imperial, loadedText: "98", loadedValue: 30), 30)
        // and 152.4 m (500 ft) shows as "152" m; parsed back it would move.
        XCTAssertEqual(send("152", loadedText: "152", loadedValue: 152.4), 152.4)
    }

    func testValueToSend_ChangedTextOrModeIsParsed() {
        XCTAssertEqual(send("200"), 200)
        XCTAssertEqual(send(" 200 "), 200)
        XCTAssertEqual(send("100", unit: .imperial)!, 30.48, accuracy: 0.001)
        // the same text in another mode is a new entry: channel 3's "0.0"
        // switched to altitude
        XCTAssertEqual(send("0.0", mode: 1, loadedText: "0.0", loadedMode: 0, loadedValue: 0), 0)
        XCTAssertFalse(PyroTriggerRule.accepts(mode: 1, value: 0))
        // a delay is not converted
        XCTAssertEqual(send("1.5", mode: 0, unit: .imperial, loadedText: "1.0", loadedMode: 0,
                            loadedValue: 1), 1.5)
    }

    // MARK: - The send hold-back

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    func testSendPyroConfig_HoldsBackARefusedConfigAndDoesNotMirrorIt() {
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        // The OC's cache: the case where a sent config is mirrored at once.
        feed(rocket, #"{"type":"config_pyro","p2e":true,"p2m":1,"p2v":150.0,"src":"oc"}"#)
        let refused: [(enabled: Bool, mode: UInt8, value: Float)] =
            [(true, 0, 0), (true, 1, 0), (false, 0, 0), (false, 0, 0)]
        XCTAssertFalse(rocket.sendPyroConfig(channels: refused))
        XCTAssertEqual(rocket.rocketConfig?.pyro2TriggerValue, 150,
                       "a held config never reaches the tiles")

        let good: [(enabled: Bool, mode: UInt8, value: Float)] =
            [(true, 0, 0), (true, 1, 200), (false, 0, 0), (false, 0, 0)]
        XCTAssertTrue(rocket.sendPyroConfig(channels: good))
        XCTAssertEqual(rocket.rocketConfig?.pyro2TriggerValue, 200)
    }

    // MARK: - The whole-profile push

    private func settleMainQueue(_ turns: Int = 5) {
        for _ in 0..<turns {
            let exp = expectation(description: "main-queue turn")
            DispatchQueue.main.async { exp.fulfill() }
            wait(for: [exp], timeout: 1)
        }
    }

    /// The pattern match keeps SyncState's main-actor Equatable out of this
    /// nonisolated test (as AntennaOffsetSettingTests' adoptedGroups does).
    private func failedText(_ syncer: ActiveRocketSyncer) -> String? {
        if case .failed(let text) = syncer.syncState { return text }
        return nil
    }

    /// A profile holding a trigger the rocket refuses (a channel left mid-edit,
    /// or one adopted from a record saved before the check): the push sends
    /// everything but the pyro config and says so, rather than "Profile matches
    /// this rocket".
    func testPushProfile_HoldsBackARefusedPyroConfigAndSaysSo() {
        let tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        let defaults = UserDefaults(suiteName: "pyro-trigger-tests-\(UUID().uuidString)")!
        let syncer = ActiveRocketSyncer()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let mine = store.add(name: "Atlas")
        store.update(mine.id) {
            $0.lastUsedUnitID = "boardA"
            $0.pyro2Enabled = true; $0.pyro2TriggerMode = 1; $0.pyro2TriggerValue = 0
        }
        store.setActive(mine.id)
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        rocket.isConnected = true
        syncer.attach(device: rocket, store: store)
        feed(rocket, #"{"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,"rid":1,"dt":"R","fw":"test"}"#)
        settleMainQueue()
        // The connect adopted the rocket's (empty) pyro report; put the edit back.
        store.update(mine.id) { $0.pyro2Enabled = true; $0.pyro2TriggerMode = 1; $0.pyro2TriggerValue = 0 }

        var sent: [UInt8] = []
        rocket.commandTap = { cmd, _ in sent.append(cmd) }
        syncer.pushProfileToRocket()
        settleMainQueue()
        XCTAssertFalse(sent.contains(34), "no pyro config the rocket would refuse")
        XCTAssertFalse(sent.isEmpty, "everything else still goes")
        XCTAssertEqual(failedText(syncer), PyroTriggerRule.notSentText(channel: 2))

        store.update(mine.id) { $0.pyro2TriggerValue = 150 }
        sent = []
        syncer.pushProfileToRocket()
        let badge = expectation(description: "the synced badge's 0.8 s hold")
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.2) { badge.fulfill() }
        wait(for: [badge], timeout: 5)
        XCTAssertTrue(sent.contains(34))
        if case .synced = syncer.syncState {} else { XCTFail("not synced: \(syncer.syncState)") }
    }
}
