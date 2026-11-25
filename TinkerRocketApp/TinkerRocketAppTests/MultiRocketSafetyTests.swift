//
//  MultiRocketSafetyTests.swift
//  TinkerRocketAppTests
//
//  Three rockets, one base station — the parts that decide what a pyro
//  command reaches and which rocket the screen follows: two rockets under one
//  rocket ID (the base station's "dup", and the app's own name-alternation
//  check), the pyro refusal that follows from it, and the flown latch that
//  keeps the focus heal off a rocket that has flown.
//

import XCTest
@testable import TinkerRocketApp

final class MultiRocketSafetyTests: XCTestCase {

    private var suiteName: String!
    private var defaults: UserDefaults!

    override func setUpWithError() throws {
        suiteName = "MultiRocketSafetyTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
    }

    override func tearDownWithError() throws {
        defaults.removePersistentDomain(forName: suiteName)
    }

    private func makeFleet() -> BLEFleet {
        BLEFleet(knownDevices: KnownDeviceStore(defaults: defaults))
    }

    private func makeBaseStation(fleet: BLEFleet? = nil) -> BLEDevice {
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.fleet = fleet
        return bs
    }

    private func frame(rid: Int, name: String? = nil, state: String = "READY",
                       extra: String = "") -> Data {
        var json = #"{"rid":\#(rid),"st":"\#(state)""#
        if let name { json += #","run":"\#(name)""# }
        if !extra.isEmpty { json += "," + extra }
        json += "}"
        return json.data(using: .utf8)!
    }

    private func telemetry(_ json: String) throws -> TelemetryData {
        try JSONDecoder().decode(TelemetryData.self, from: json.data(using: .utf8)!)
    }

    // MARK: - "dup" from the base station

    func testDupKey_decodes_andAbsentMeansClear() throws {
        XCTAssertTrue(try telemetry(#"{"rid":2,"dup":1}"#).duplicateIDSuspected)
        XCTAssertFalse(try telemetry(#"{"rid":2}"#).duplicateIDSuspected)
        XCTAssertEqual(try telemetry(#"{"rid":2,"dup":"1"}"#).duplicate_id, 1,
                       "tolerant like every integer key (#571)")
    }

    func testDupKey_flagsTheRocketOnTheBaseStation() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(frame(rid: 2, name: "Dart", extra: #""dup":1"#))
        XCTAssertTrue(bs.isDuplicateIDSuspected(rocketID: 2))
        bs.parseTelemetryData(frame(rid: 2, name: "Dart"))
        XCTAssertFalse(bs.isDuplicateIDSuspected(rocketID: 2),
                       "the base station omits dup once it clears")
    }

    // MARK: - Name alternation, the app's own check

    func testAlternatingNames_twoChangesInsideTheWindow_flag() {
        let t0 = Date()
        let r = RemoteRocket(baseStationDeviceID: UUID(), rocketID: 1, unitName: "Booster")
        r.updateTelemetry(TelemetryData(), unitName: "Twin", now: t0)
        XCTAssertFalse(r.duplicateIDSuspected(now: t0), "one change is a rename")
        r.updateTelemetry(TelemetryData(), unitName: "Booster", now: t0.addingTimeInterval(1))
        XCTAssertTrue(r.duplicateIDSuspected(now: t0.addingTimeInterval(1)))
    }

    func testRename_isNotADuplicate() {
        let t0 = Date()
        let r = RemoteRocket(baseStationDeviceID: UUID(), rocketID: 1, unitName: "Old name")
        for i in 0..<20 {
            r.updateTelemetry(TelemetryData(), unitName: "New name", now: t0.addingTimeInterval(Double(i) * 0.5))
        }
        XCTAssertFalse(r.duplicateIDSuspected(now: t0.addingTimeInterval(10)))
    }

    func testMissingName_neverCountsAsAChange() {
        let t0 = Date()
        let r = RemoteRocket(baseStationDeviceID: UUID(), rocketID: 1, unitName: "Booster")
        // Trimmed frames and frames before the beacon carry no "run".
        for (i, name) in [nil, "Booster", nil, "", "Booster", nil].enumerated() {
            r.updateTelemetry(TelemetryData(), unitName: name, now: t0.addingTimeInterval(Double(i)))
        }
        XCTAssertFalse(r.duplicateIDSuspected(now: t0.addingTimeInterval(6)))

        // And a gap between two names is still one change, not two.
        r.updateTelemetry(TelemetryData(), unitName: nil, now: t0.addingTimeInterval(7))
        r.updateTelemetry(TelemetryData(), unitName: "Renamed", now: t0.addingTimeInterval(8))
        XCTAssertFalse(r.duplicateIDSuspected(now: t0.addingTimeInterval(8)))
    }

    func testChangesFurtherApartThanTheWindow_doNotFlag() {
        let t0 = Date()
        let r = RemoteRocket(baseStationDeviceID: UUID(), rocketID: 1, unitName: "A")
        r.updateTelemetry(TelemetryData(), unitName: "B", now: t0)
        r.updateTelemetry(TelemetryData(), unitName: "A", now: t0.addingTimeInterval(11))
        XCTAssertFalse(r.duplicateIDSuspected(now: t0.addingTimeInterval(11)))
    }

    func testFlag_holdsThirtySecondsAfterTheLastAlternation() {
        let t0 = Date()
        let r = RemoteRocket(baseStationDeviceID: UUID(), rocketID: 1, unitName: "A")
        r.updateTelemetry(TelemetryData(), unitName: "B", now: t0)
        r.updateTelemetry(TelemetryData(), unitName: "A", now: t0.addingTimeInterval(0.5))
        XCTAssertTrue(r.duplicateIDSuspected(now: t0.addingTimeInterval(20)))
        XCTAssertFalse(r.duplicateIDSuspected(now: t0.addingTimeInterval(31)))
    }

    func testAlternationThroughTheRealParsePath() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        bs.parseTelemetryData(frame(rid: 1, name: "Twin"))
        bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        XCTAssertTrue(bs.isDuplicateIDSuspected(rocketID: 1))
        XCTAssertEqual(bs.remoteRockets.count, 1, "one rid, one roster entry — that is the problem")
    }

    func testSectionHeaderWarning_namesTheRocketID() {
        let bs = makeBaseStation()
        bs.isConnected = true
        bs.parseTelemetryData(frame(rid: 2, name: "Dart", extra: #""dup":1"#))
        let subject = RocketRoster.build(devices: [bs]).first!
        let warning = RocketSectionHeader.duplicateIDWarning(for: subject)
        XCTAssertNotNil(warning)
        XCTAssertTrue(warning?.contains("rocket ID 2") ?? false)
        XCTAssertTrue(RocketSectionHeader.needsHeaderWhenSolo(subject, flown: false),
                      "the warning must show even when this is the only section")
    }

    // MARK: - Pyro refusal

    private func connectedBaseStationFocusedOnRocket1(flagged: Bool) -> (BLEDevice, () -> [(UInt8, Data)]) {
        let bs = makeBaseStation()
        bs.isConnected = true
        if flagged {
            bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
            bs.parseTelemetryData(frame(rid: 1, name: "Twin"))
            bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        } else {
            bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        }
        XCTAssertEqual(bs.focusRocketID, 1)
        // Tapped only now: a first-heard rocket can start the auto channel
        // pick, which is not what these tests are about.
        var sent: [(UInt8, Data)] = []
        bs.commandTap = { sent.append(($0, $1)) }
        return (bs, { sent })
    }

    func testPyroPath_readyForAFreshUnflaggedRocket() {
        let (bs, sent) = connectedBaseStationFocusedOnRocket1(flagged: false)
        XCTAssertTrue(bs.pyroCommandPathReady)
        XCTAssertFalse(bs.focusedRocketIDShared)
        bs.sendPyroFire(channel: 2)
        bs.sendPyroContTest(channel: 3)
        XCTAssertEqual(sent().map { $0.0 }, [50, 50])
        XCTAssertEqual(sent().map { [UInt8]($0.1) }, [[1, 36, 2], [1, 35, 3]])
    }

    func testPyroPath_refusesAFlaggedRocketID() {
        let (bs, sent) = connectedBaseStationFocusedOnRocket1(flagged: true)
        XCTAssertFalse(bs.pyroCommandPathReady, "the T-0 guard must see the shared ID")
        XCTAssertTrue(bs.focusedRocketIDShared, "PyroTestView's FIRE gate reads this")
        bs.sendPyroFire(channel: 2)
        bs.sendPyroContTest(channel: 3)
        XCTAssertTrue(sent().isEmpty, "no pyro uplink may name an ID two rockets answer to")
    }

    func testPyroPath_refusesWhenTheBaseStationSaysDup() {
        let bs = makeBaseStation()
        bs.isConnected = true
        bs.parseTelemetryData(frame(rid: 1, name: "Booster", extra: #""dup":1"#))
        var sent: [UInt8] = []
        bs.commandTap = { cmd, _ in sent.append(cmd) }
        XCTAssertFalse(bs.pyroCommandPathReady)
        bs.sendPyroFire(channel: 1)
        XCTAssertTrue(sent.isEmpty)
    }

    // MARK: - Flown latch

    func testFlownLatch_setOnLiveInflightLandedOrLaunchFlag() throws {
        let fleet = makeFleet()
        let k = { RocketKey(networkID: 7, rocketID: $0) }
        fleet.noteFlightProgress(try telemetry(#"{"st":"READY"}"#), key: k(1))
        XCTAssertTrue(fleet.flownRockets.isEmpty)
        fleet.noteFlightProgress(try telemetry(#"{"st":"INFLIGHT"}"#), key: k(1))
        fleet.noteFlightProgress(try telemetry(#"{"st":"LANDED"}"#), key: k(2))
        fleet.noteFlightProgress(try telemetry(#"{"st":"PRELAUNCH","fs":1}"#), key: k(3))
        XCTAssertEqual(fleet.flownRockets, [k(1), k(2), k(3)])
    }

    func testFlownLatch_ignoresStaleFramesAndSimulatedFlights() throws {
        let fleet = makeFleet()
        let key = RocketKey(networkID: 7, rocketID: 1)
        fleet.noteFlightProgress(try telemetry(#"{"st":"INFLIGHT","ds":1}"#), key: key)
        fleet.noteFlightProgress(try telemetry(#"{"st":"INFLIGHT","fs":257}"#), key: key)  // sim bit
        XCTAssertTrue(fleet.flownRockets.isEmpty)
    }

    func testFlownLatch_survivesTheBaseStationBeingRebuilt() {
        let fleet = makeFleet()
        let first = makeBaseStation(fleet: fleet)
        first.parseTelemetryData(frame(rid: 1, name: "Booster", state: "INFLIGHT"))
        // A reconnect builds a new BLEDevice with an empty roster.
        let second = makeBaseStation(fleet: fleet)
        XCTAssertTrue(second.remoteRockets.isEmpty)
        XCTAssertTrue(fleet.flownRockets.contains(RocketKey(networkID: 0, rocketID: 1)))
    }

    // MARK: - Focus heal (BLEDevice: 15 s silence moves the pin)

    /// Rocket 1 pinned, then silent past focusPinStaleAfter with the grace
    /// window spent: the clock is backdated rather than waited on.
    private func silencePinnedRocket(_ bs: BLEDevice, rid: UInt8 = 1) {
        let remote = bs.remoteRockets.first { $0.rocketID == rid }!
        remote.lastSeen = Date().addingTimeInterval(-(BLEDevice.focusPinStaleAfter + 5))
        bs.focusPinGraceUntil = Date().addingTimeInterval(-1)
    }

    func testHeal_movesFocusOffASilentRocketThatNeverFlew() {
        let fleet = makeFleet()
        let bs = makeBaseStation(fleet: fleet)
        bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        bs.parseTelemetryData(frame(rid: 2, name: "Dart"))
        XCTAssertEqual(bs.focusRocketID, 1)

        silencePinnedRocket(bs)
        bs.parseTelemetryData(frame(rid: 2, name: "Dart"))
        XCTAssertEqual(bs.focusRocketID, 2, "a dangling pin heals to the rocket on the air")
    }

    func testHeal_respectsTheGraceWindow() {
        let fleet = makeFleet()
        let bs = makeBaseStation(fleet: fleet)
        bs.parseTelemetryData(frame(rid: 1, name: "Booster"))
        let remote = bs.remoteRockets.first { $0.rocketID == 1 }!
        remote.lastSeen = Date().addingTimeInterval(-60)
        // focusPinGraceUntil is still in the future from the first latch.
        bs.parseTelemetryData(frame(rid: 2, name: "Dart"))
        XCTAssertEqual(bs.focusRocketID, 1)
    }

    func testHeal_leavesAPinnedRocketThatHasFlownAlone() {
        let fleet = makeFleet()
        let bs = makeBaseStation(fleet: fleet)
        bs.parseTelemetryData(frame(rid: 1, name: "Booster", state: "INFLIGHT",
                                    extra: #""lat":34.6572,"lon":-118.2015,"nsat":9"#))
        bs.parseTelemetryData(frame(rid: 2, name: "Dart"))
        XCTAssertEqual(bs.focusRocketID, 1)
        XCTAssertTrue(fleet.flownRockets.contains(RocketKey(networkID: 0, rocketID: 1)))

        silencePinnedRocket(bs)
        for _ in 0..<5 { bs.parseTelemetryData(frame(rid: 2, name: "Dart")) }

        XCTAssertEqual(bs.focusRocketID, 1, "the rocket that flew keeps the screen when it goes quiet")
        XCTAssertNotNil(bs.lastValidRocketFix, "and its last fix stays latched for the walk")
        XCTAssertEqual(bs.telemetry.source_rocket_id, 1, "Dart's frames do not take over the mirror")

        // Its section says lost — even alone on screen.
        bs.isConnected = true
        let remote = bs.remoteRockets.first { $0.rocketID == 1 }!
        remote.lastSeen = Date().addingTimeInterval(-(RocketFreshness.lostAfter + 5))
        let subject = RocketRoster.build(devices: [bs]).first { $0.key?.rocketID == 1 }!
        guard case .lost = subject.freshness() else { return XCTFail("expected lost") }
        XCTAssertTrue(RocketSectionHeader.needsHeaderWhenSolo(subject, flown: true))

        // Follow still moves it on purpose.
        fleet.setFocus(baseStation: bs, rocketID: 2)
        XCTAssertEqual(bs.focusRocketID, 2)
    }
}
