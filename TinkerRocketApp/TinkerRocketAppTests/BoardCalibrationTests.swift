//
//  BoardCalibrationTests.swift
//  TinkerRocketAppTests
//
//  #1629: what a flight computer holds for calibration, per board — the
//  wording (BoardCalibration) and the device that reads it on connect and
//  folds the board's answers in, then into the registry. Android twin:
//  BoardCalibrationTest.kt.
//

import XCTest
@testable import TinkerRocketApp

final class BoardCalibrationTests: XCTestCase {

    private var suiteName: String!
    private var defaults: UserDefaults!
    /// A device holds its fleet weakly; the app's fleet lives for the whole
    /// session, so the tests keep theirs alive too.
    private var fleets: [BLEFleet] = []

    override func setUpWithError() throws {
        suiteName = "BoardCalibrationTests-\(UUID().uuidString)"
        defaults = UserDefaults(suiteName: suiteName)
    }

    override func tearDownWithError() throws {
        defaults.removePersistentDomain(forName: suiteName)
    }

    // MARK: - The record and its wording

    func testNotReadIsNeverNotCalibrated() {
        let none = BoardCalibration()
        XCTAssertTrue(none.missing.isEmpty)
        XCTAssertNil(none.notCalibratedLine)
        XCTAssertEqual(none.summary, "Calibration not read yet")
        XCTAssertEqual(BoardCalibration.rowText(nil), "Not read yet")
    }

    func testMissingNamesEachSensorInOrder() {
        let c = BoardCalibration(mag: false, accel: false, gyro: false, highGAligned: false)
        XCTAssertEqual(c.notCalibratedLine, "Not calibrated: magnetometer, accelerometer, gyro")
        XCTAssertEqual(c.summary, c.notCalibratedLine)
        XCTAssertEqual(BoardCalibration(mag: true, accel: false, gyro: true).notCalibratedLine,
                       "Not calibrated: accelerometer")
    }

    func testTheHighGAlignmentIsNeverMissing() {
        let c = BoardCalibration(mag: true, accel: true, gyro: true, highGAligned: false)
        XCTAssertNil(c.notCalibratedLine)
        XCTAssertTrue(c.fullyCalibrated)
        XCTAssertEqual(c.summary, "Calibrated")
        XCTAssertEqual(BoardCalibration.highGRowText(false), "Not aligned (flies on its bias)")
    }

    func testPartlyReadSaysSo() {
        XCTAssertEqual(BoardCalibration(mag: true, accel: true).summary, "Calibration not fully read")
    }

    func testAReadingChangesOnlyWhatItAnswers() {
        let t1 = Date(timeIntervalSince1970: 1), t2 = Date(timeIntervalSince1970: 2)
        let old = BoardCalibration(mag: true, accel: false, gyro: true, highGAligned: false, readAt: t1)
        let now = old.merged(BoardCalibration(accel: true, highGAligned: true, readAt: t2))
        XCTAssertEqual(now, BoardCalibration(mag: true, accel: true, gyro: true, highGAligned: true, readAt: t2))
    }

    // MARK: - The device

    private func device(dt: String, store: KnownDeviceStore? = nil,
                        clock: @escaping () -> Date = { Date() }) -> (BLEDevice, () -> [UInt8], BLEFleet) {
        let fleet = BLEFleet(knownDevices: store ?? KnownDeviceStore(defaults: defaults))
        fleets.append(fleet)
        let d = BLEDevice(peripheral: nil, name: "TR-test")
        d.fleet = fleet
        d.nowProvider = clock
        var sent: [UInt8] = []
        d.commandTap = { cmd, _ in sent.append(cmd) }
        d.parseTelemetryData(Data(
            #"{"type":"config_identity","uid":"a1b2c3d4","un":"Atlas","nid":5,"rid":1,"dt":"\#(dt)"}"#.utf8))
        return (d, { sent.filter { [61, 63, 77].contains($0) } }, fleet)
    }

    /// 0xCA: MagCalStatusData, sub_type at payload byte 4.
    private func mag(_ d: BLEDevice, subType: UInt8) {
        var b = [UInt8](repeating: 0, count: 1 + 36)
        b[0] = 0xCA; b[1 + 4] = subType
        d.handleFileOpsFrame(Data(b))
    }

    /// 0xCB: SensorCalStatusData, valid first.
    private func sensor(_ d: BLEDevice, valid: Bool) {
        var b = [UInt8](repeating: 0, count: 1 + 19)
        b[0] = 0xCB; b[1] = valid ? 1 : 0
        d.handleFileOpsFrame(Data(b))
    }

    /// 0xD1: AccelCalStatusData flags first; the high-g tail's flags 24 bytes in.
    private func accel(_ d: BLEDevice, stored: Bool, hgStored: Bool) {
        var b = [UInt8](repeating: 0, count: 1 + 40 + 74)
        b[0] = 0xD1; b[1] = stored ? 0x03 : 0
        b[1 + 40 + 24] = hgStored ? 0x07 : 0x04
        d.handleFileOpsFrame(Data(b))
    }

    func testARocketIsReadOnConnect_aBaseStationIsNot() {
        let (_, rocketReads, _) = device(dt: "R")
        XCTAssertEqual(rocketReads(), [61, 63, 77], "one read of each, once the role is known")
        let (_, bsReads, _) = device(dt: "B")
        XCTAssertTrue(bsReads().isEmpty, "a base station has nothing to calibrate")
    }

    func testTheBoardsAnswersFillTheRecord() {
        let (d, _, _) = device(dt: "R")
        mag(d, subType: 3)                // APPLIED
        sensor(d, valid: false)
        accel(d, stored: true, hgStored: false)
        let c = d.calibration
        XCTAssertEqual(c.mag, true)
        XCTAssertEqual(c.gyro, false)
        XCTAssertEqual(c.accel, true)
        XCTAssertEqual(c.highGAligned, false)
        XCTAssertEqual(c.notCalibratedLine, "Not calibrated: gyro")
        XCTAssertNotNil(c.readAt)
    }

    func testAnIdleMagFrameIsAnAnswerOnlyRightAfterTheRead() {
        var now = Date(timeIntervalSince1970: 1_000)
        let (d, _, _) = device(dt: "R", clock: { now })
        mag(d, subType: 0)                // IDLE, answering the connect read
        XCTAssertEqual(d.calibration.mag, false)

        mag(d, subType: 3)                // a calibration stored
        XCTAssertEqual(d.calibration.mag, true)

        now = now.addingTimeInterval(BLEDevice.calReadWindow + 1)
        mag(d, subType: 0)                // a session ending, long after any read
        XCTAssertEqual(d.calibration.mag, true, "an IDLE nobody asked for says nothing")
    }

    func testRailOnAsksAgainForWhatIsStillUnanswered() {
        let (d, reads, _) = device(dt: "R")
        XCTAssertEqual(reads().count, 3)
        d.parseTelemetryData(Data(#"{"st":"READY","fs":16}"#.utf8))
        XCTAssertEqual(reads().count, 6, "nothing came back before the rail came up")

        mag(d, subType: 3); sensor(d, valid: true); accel(d, stored: true, hgStored: true)
        d.parseTelemetryData(Data(#"{"st":"READY","fs":0}"#.utf8))
        d.parseTelemetryData(Data(#"{"st":"READY","fs":16}"#.utf8))
        XCTAssertEqual(reads().count, 6, "all answered: no need to ask again")
        XCTAssertTrue(d.calibration.fullyCalibrated)
    }

    func testEachAnswerIsKeptOnTheBoardsRecord() throws {
        let store = KnownDeviceStore(defaults: defaults)
        let t = Date(timeIntervalSinceReferenceDate: 800_000_000)
        let (d, _, _) = device(dt: "R", store: store, clock: { t })
        mag(d, subType: 3)
        accel(d, stored: false, hgStored: false)

        let rec = try XCTUnwrap(store.device(for: "a1b2c3d4"))
        XCTAssertEqual(rec.calibration, BoardCalibration(mag: true, accel: false, highGAligned: false, readAt: t))

        // And it survives a restart of the app.
        let reloaded = KnownDeviceStore(defaults: defaults).device(for: "a1b2c3d4")?.calibration
        XCTAssertEqual(reloaded, rec.calibration)
    }

    func testARelayedRocketsLineReadsItsBoardsRecord() {
        let store = KnownDeviceStore(defaults: defaults)
        store.deviceDidReportIdentity(unitID: "aaa", name: "Booster", deviceType: .rocket,
                                      networkID: 5, rocketID: 1, pusher: nil)
        store.deviceDidReportCalibration(unitID: "aaa",
                                         reading: BoardCalibration(mag: false, accel: true, gyro: true))
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        bs.isConnected = true
        bs.parseTelemetryData(Data(
            #"{"type":"config_identity","uid":"bs","un":"Base","nid":5,"dt":"B"}"#.utf8))
        bs.parseTelemetryData(Data(#"{"rid":1,"run":"Booster","st":"READY"}"#.utf8))
        let subject = RocketRoster.build(devices: [bs]).first!
        XCTAssertEqual(RegistryCalibrationLine.text(for: subject, in: store),
                       "Not calibrated: magnetometer (as of its last Bluetooth link)")
    }
}
