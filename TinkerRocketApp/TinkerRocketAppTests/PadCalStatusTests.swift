import XCTest
@testable import TinkerRocketApp

/// #1600 phase 5: the 0xD0 pad-calibration status against the golden the C++
/// emitter wrote from the real PadCalStatusData, command 75 byte for byte, the
/// command-21 fallback for firmware that predates both, and what the pad
/// calibration screen says.
final class PadCalStatusTests: XCTestCase {

    private let statusFixture = "fileops/padcal_0xD0_40.bin"
    private let commandFixture = "commands/cmd75_padcal_10.bin"

    /// The golden's payload, after the 0xD0.
    private var goldenPayload: [UInt8] {
        Array([UInt8](WireFixtures.data(statusFixture)).dropFirst())
    }

    /// The golden with some payload bytes replaced.
    private func patched(_ edits: [Int: UInt8]) -> PadCalStatus {
        var p = goldenPayload
        for (i, b) in edits { p[i] = b }
        return PadCalStatus.decode(p)!
    }

    // MARK: - 0xD0 decode

    func testDecodesEveryFieldOfTheGolden() throws {
        let side = WireFixtures.sidecar(statusFixture)
        let frame = [UInt8](WireFixtures.data(statusFixture))
        XCTAssertEqual(frame.count, 41)
        XCTAssertEqual(frame.first, 0xD0)
        let s = try XCTUnwrap(PadCalStatus.decode(Array(frame.dropFirst())))
        XCTAssertEqual(Int64(s.timeUs), side.i64("time_us"))
        XCTAssertEqual(Int(s.calState), side.int("cal_state"))
        XCTAssertEqual(Int(s.calProgress), side.int("cal_progress"))
        XCTAssertEqual(Int(s.calGravityMg), side.int("cal_gravity_mg"))
        XCTAssertEqual(Int(s.satsTracked), side.int("sats_tracked"))
        XCTAssertEqual(Int(s.satsNoEph), side.int("sats_no_eph"))
        XCTAssertEqual(Int(s.satsSettling), side.int("sats_settling"))
        XCTAssertEqual(Int(s.satsCorrected), side.int("sats_corrected"))
        XCTAssertEqual(Int(s.corrRmsCm), side.int("corr_rms_cm"))
        XCTAssertEqual(Int(s.ionoSource), side.int("iono_source"))
        XCTAssertEqual(Int(s.flags), side.int("flags"))
        XCTAssertEqual(Int(s.ionoGpsDay), side.int("iono_gps_day"))
        XCTAssertEqual(Int(s.ionoAgeDays), side.int("iono_age_days"))   // -1: signed
        XCTAssertEqual(Int(s.padFixes), side.int("pad_fixes"))
        XCTAssertEqual(Int(s.padSecs), side.int("pad_secs"))
        XCTAssertEqual(Int(s.padScatterCm), side.int("pad_scatter_cm"))
        XCTAssertEqual(Int(s.padLatE7), side.int("pad_lat_e7"))
        XCTAssertEqual(Int(s.padLonE7), side.int("pad_lon_e7"))          // negative
        XCTAssertEqual(Int(s.padAltMm), side.int("pad_alt_mm"))          // negative
        XCTAssertEqual(Int(s.posSigmaCm), side.int("pos_sigma_cm"))

        // What the screen reads off them.
        XCTAssertEqual(s.state, .running)
        XCTAssertEqual(s.iono, .broadcast)
        // flags 0x0F: the filter's board, running, age known, pad average.
        XCTAssertTrue(s.runsRawGnss)
        XCTAssertTrue(s.filterRunning)
        XCTAssertTrue(s.ionoAgeKnown)
        XCTAssertTrue(s.hasPadAverage)
        XCTAssertFalse(s.padFrozen)
        XCTAssertFalse(s.padMismatch)
    }

    func testEachFlagBitReadsAlone() {
        let bits: [(UInt8, (PadCalStatus) -> Bool)] = [
            (0x01, \.runsRawGnss), (0x02, \.filterRunning), (0x04, \.ionoAgeKnown),
            (0x08, \.hasPadAverage), (0x10, \.padFrozen), (0x20, \.padMismatch),
        ]
        for (bit, read) in bits {
            let s = patched([15: bit])
            XCTAssertTrue(read(s), "flags 0x\(String(bit, radix: 16))")
            XCTAssertEqual(bits.filter { $0.1(s) }.count, 1,
                           "flags 0x\(String(bit, radix: 16)) reads as that bit alone")
        }
    }

    func testAShortPayloadIsNotDecoded() {
        XCTAssertNil(PadCalStatus.decode(Array(goldenPayload.prefix(39))))
        XCTAssertNotNil(PadCalStatus.decode(goldenPayload))
    }

    func testAnUnknownStateOrSourceIsKeptRawNotGuessed() {
        let s = patched([4: 9, 14: 7])
        XCTAssertNil(s.state)
        XCTAssertNil(s.iono)
        XCTAssertEqual(s.calState, 9)
        XCTAssertEqual(s.ionoSource, 7)
    }

    @MainActor
    func testTheDeviceTakesA41ByteFrameAndDropsAShorterOne() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        let frame = WireFixtures.data(statusFixture)
        device.handleFileOpsFrame(frame.prefix(40))
        XCTAssertNil(device.padCalStatus, "frame length < 41: no decode")
        device.handleFileOpsFrame(frame)
        XCTAssertEqual(device.padCalStatus, PadCalStatus.decode(goldenPayload))
    }

    @MainActor
    func testADisconnectForgetsTheStatus() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        device.isConnected = true
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        XCTAssertNotNil(device.padCalStatus)
        device.onDisconnect()
        XCTAssertNil(device.padCalStatus, "whether 0xD0 came is asked per connection")
    }

    // MARK: - Command 75

    @MainActor
    func testCommand75IsByteExactAgainstTheGolden() throws {
        let side = WireFixtures.sidecar(commandFixture)
        let iono = try XCTUnwrap(Klobuchar(raw: side.ints("iono"),
                                           gpsDay: side.int("iono_gps_day")))
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var wire = Data()
        // What sendRawCommand writes on the command characteristic.
        device.commandTap = { cmd, payload in
            wire.append(cmd)
            wire.append(payload)
        }
        device.startPadCalibration(iono)
        XCTAssertEqual(wire, WireFixtures.data(commandFixture))
    }

    @MainActor
    func testWithoutAnIonosphereThePayloadIsTenZeros() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var sent: [(UInt8, Data)] = []
        device.commandTap = { sent.append(($0, $1)) }
        device.startPadCalibration(nil)
        XCTAssertEqual(sent.map { $0.0 }, [75])
        XCTAssertEqual(sent.first?.1, Data(count: 10), "gps_day 0: the rocket keeps what it has")
    }

    // MARK: - The command-21 fallback

    /// A connected rocket link with no peripheral: the tap is the wire.
    @MainActor
    private func rocket() -> (BLEDevice, () -> [UInt8]) {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        device.isConnected = true
        var cmds: [UInt8] = []
        device.commandTap = { cmd, _ in cmds.append(cmd) }
        return (device, { cmds })
    }

    @MainActor
    func testTheFallbackWaitsThreeSeconds() {
        XCTAssertEqual(BLEDevice.padCalLegacyFallbackDelay, 3.0)
    }

    @MainActor
    func testNo0xD0WithinTheWindowSendsCommand21AndRunsTheOldFlow() async throws {
        let (device, sent) = rocket()
        var legacy = false
        device.startPadCalibration(nil, legacyAfter: 0.05) { legacy = true }
        XCTAssertEqual(sent(), [75])
        XCTAssertFalse(legacy)
        try await Task.sleep(nanoseconds: 300_000_000)
        XCTAssertEqual(sent(), [75, 21])
        XCTAssertTrue(legacy)
    }

    @MainActor
    func testARocketThatHasSent0xD0GetsNoFallback() async throws {
        let (device, sent) = rocket()
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        var legacy = false
        device.startPadCalibration(nil, legacyAfter: 0.05) { legacy = true }
        try await Task.sleep(nanoseconds: 300_000_000)
        XCTAssertEqual(sent(), [75])
        XCTAssertFalse(legacy)
    }

    @MainActor
    func testA0xD0InsideTheWindowCancelsTheFallback() async throws {
        let (device, sent) = rocket()
        var legacy = false
        device.startPadCalibration(nil, legacyAfter: 0.2) { legacy = true }
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        try await Task.sleep(nanoseconds: 500_000_000)
        XCTAssertEqual(sent(), [75])
        XCTAssertFalse(legacy)
    }

    @MainActor
    func testADisconnectDisarmsTheFallback() async throws {
        let (device, sent) = rocket()
        var legacy = false
        device.startPadCalibration(nil, legacyAfter: 0.05) { legacy = true }
        device.onDisconnect()
        try await Task.sleep(nanoseconds: 300_000_000)
        XCTAssertEqual(sent(), [75])
        XCTAssertFalse(legacy)
    }

    @MainActor
    func testOnlyTheLatestStartFallsBack() async throws {
        let (device, sent) = rocket()
        var legacyRuns = 0
        device.startPadCalibration(nil, legacyAfter: 0.05) { legacyRuns += 1 }
        device.startPadCalibration(nil, legacyAfter: 0.05) { legacyRuns += 1 }
        try await Task.sleep(nanoseconds: 300_000_000)
        XCTAssertEqual(sent(), [75, 75, 21])
        XCTAssertEqual(legacyRuns, 1)
    }

    // MARK: - What the screen says (worded as Android's PadCalReadout)

    func testTheGoldenAsTheScreenShowsIt() {
        let s = PadCalStatus.decode(goldenPayload)!
        XCTAssertEqual(s.gyroAccelText, "Calibrating, 42 % — keep the rocket still")
        XCTAssertEqual(s.gnssRows.map(\.label),
                       ["Satellites tracked", "Waiting for ephemeris", "Settling", "Pad corrections",
                        "Ionosphere", "Pad position", "Raw-GNSS filter"])
        let rows = Dictionary(uniqueKeysWithValues: s.gnssRows.map { ($0.label, $0.value) })
        XCTAssertEqual(rows["Satellites tracked"], "14")
        XCTAssertEqual(rows["Waiting for ephemeris"], "2")
        XCTAssertEqual(rows["Settling"], "3")
        XCTAssertEqual(rows["Pad corrections"], "9 (spread 0.9 m)")
        // iono_age_days -1: the receiver's date a day behind the file's
        XCTAssertEqual(rows["Ionosphere"], "broadcast set of \(PadCalStatus.day(17082)), 0 days old")
        XCTAssertEqual(rows["Pad position"], "1500 fixes over 4:00, scatter 1.8 m")
        XCTAssertEqual(rows["Raw-GNSS filter"], "running, position sigma 2.1 m")
    }

    func testTheWindowsOutcomes() {
        // 1003 mg = 0x03EB, the golden's
        XCTAssertEqual(patched([4: 2]).gyroAccelText, "Gyro & accel calibrated (1.003 g)")
        // 870 mg = 0x0366
        XCTAssertEqual(patched([4: 3, 6: 0x66, 7: 0x03]).gyroAccelText,
                       "Rejected: gravity 0.87 g — keep the rocket still and try again; the previous calibration is kept")
        // a window that timed out has no gravity to report
        XCTAssertEqual(patched([4: 3, 6: 0, 7: 0]).gyroAccelText,
                       "Rejected: keep the rocket still and try again; the previous calibration is kept")
        XCTAssertEqual(patched([4: 4]).gyroAccelText,
                       "Not run: the rocket is not on the pad (or a sim is running)")
        XCTAssertEqual(patched([4: 0]).gyroAccelText, "No pad calibration since the rocket powered up")
    }

    func testAWindowMoreThan3PercentOffGravityIsWarnedAbout() {
        // #1059's check, on the window's own measure: the FC keeps ±20 %.
        XCTAssertNil(patched([4: 2]).gravityWarningText, "1.003 g is fine")
        // 1050 mg = 0x041A
        XCTAssertEqual(patched([4: 2, 6: 0x1A, 7: 0x04]).gravityWarningText,
                       "Gravity read 5.0 % off 1 g. Consider running a bench calibration before flight.")
        // 969 mg = 0x03C9: 3.1 % low
        XCTAssertNotNil(patched([4: 2, 6: 0xC9, 7: 0x03]).gravityWarningText)
        // only a calibration the rocket kept; a rejection says so itself
        XCTAssertNil(patched([4: 3, 6: 0x1A, 7: 0x04]).gravityWarningText)
        XCTAssertNil(patched([4: 1, 6: 0x1A, 7: 0x04]).gravityWarningText)
    }

    func testTheIonosphereLine() {
        XCTAssertEqual(patched([14: 2]).ionosphereText, "pad survey (backup)")
        XCTAssertEqual(patched([14: 0]).ionosphereText, "none yet")
        // PCS_PAD_MISMATCH: the receiver's fix and its raw measurements
        // disagree, so the survey's corrections are off and the source reads
        // none; the warning line above the block says why.
        let mismatch = patched([14: 0, 15: 0x0F | 0x20])
        XCTAssertTrue(mismatch.padMismatch)
        XCTAssertEqual(mismatch.ionosphereText, "none yet")
        XCTAssertEqual(PadCalStatus.padMismatchText,
                       "Receiver's fix disagrees with its raw measurements — pad corrections off")
        // the age only once the receiver has given its date
        XCTAssertEqual(patched([15: 0x0F & ~0x04]).ionosphereText,
                       "broadcast set of \(PadCalStatus.day(17082))")
        XCTAssertEqual(patched([16: 0, 17: 0, 15: 0x0F & ~0x04]).ionosphereText, "broadcast set")
        XCTAssertEqual(PadCalStatus.age(-1), "0 days old")
        XCTAssertEqual(PadCalStatus.age(0), "0 days old")
        XCTAssertEqual(PadCalStatus.age(1), "1 day old")
        XCTAssertEqual(PadCalStatus.age(12), "12 days old")
    }

    func testTheSurveyRowsAsTheyFill() {
        let fresh = patched([9: 0, 11: 0, 15: 0x01])   // ephemeris in; no corrections, fixes or filter
        let rows = Dictionary(uniqueKeysWithValues: fresh.gnssRows.map { ($0.label, $0.value) })
        XCTAssertNil(rows["Waiting for ephemeris"], "shown only while some are missing")
        XCTAssertEqual(rows["Pad corrections"], "0")
        XCTAssertEqual(rows["Pad position"], "no fixes yet")
        XCTAssertEqual(rows["Raw-GNSS filter"], "not running yet")
        XCTAssertEqual(patched([15: 0x0F | 0x10]).padPositionText,
                       "1500 fixes over 4:00, scatter 1.8 m, frozen at launch")
        XCTAssertEqual(PadCalStatus.minutesSeconds(3725), "62:05")
    }

    func testThePhonesOwnLine() {
        XCTAssertEqual(IonosphereStore.phoneText(nil, now: Date()),
                       "none — the rocket will use its pad survey")
        let k = Klobuchar(raw: [21, 0, -2, 0, 60, -1, -4, 3], gpsDay: 17082)!
        let at = Date()
        let text = IonosphereStore.phoneText(HeldIonosphere(klobuchar: k, fetchedAt: at), now: at)
        XCTAssertTrue(text.hasPrefix("set of \(PadCalStatus.day(17082)) (fetched "), text)
    }
}
