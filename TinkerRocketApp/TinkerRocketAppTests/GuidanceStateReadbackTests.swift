import XCTest
@testable import TinkerRocketApp

/// GTV Blue, 2026-11-01: the app showed station-keeping ON and the flight
/// computer flew with it OFF.  Every screen showed the profile; the only
/// value the out computer reported ("ge") was its own copy; and sending a
/// guidance config wrote "on" into the app's view of the rocket whether or
/// not the flight computer ever applied it.  The flight computer's own on/off
/// now rides config_guid ("gen"), and the app compares against that alone.
final class GuidanceStateReadbackTests: XCTestCase {

    private var tempDir: URL!
    private var defaults: UserDefaults!

    override func setUp() {
        super.setUp()
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        defaults = UserDefaults(suiteName: "guidance-readback-tests-\(UUID().uuidString)")
    }

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func feed(_ rocket: BLEDevice, _ json: String) {
        rocket.parseTelemetryData(json.data(using: .utf8))
    }

    private func settleMainQueue(_ turns: Int = 5) {
        for _ in 0..<turns {
            let exp = expectation(description: "main-queue turn")
            DispatchQueue.main.async { exp.fulfill() }
            wait(for: [exp], timeout: 1)
        }
    }

    /// Everything matches RocketProfile's defaults except what a test passes.
    private static func configFrame(ge: Bool) -> String {
        #"{"type":"config","sb1":0,"shz":333,"smn":1000,"smx":2000,"kp":0.1200,"ki":0.0100,"kd":0.0000,"pmn":-20.0,"pmx":20.0,"sen":true,"gs":true,"ac":false,"rdly":0,"rcap":60.0,"kpang":2.00,"iwind":200.0,"ge":\#(ge),"camt":2,"irate":0}"#
    }
    private static func guidFrame(law: Int = 0, genTail: String) -> String {
        let p = RocketProfile.makeDefault(name: "x")
        return #"{"type":"config_guid","gng":\#(p.pnNavGain),"gma":\#(p.pnMaxAccel),"gaf":\#(p.pnAccelToFin),"gmf":\#(p.pnMaxFinDeg),"gms":\#(p.pnMinSpeed),"gcd":\#(p.pnCoastDelayMs),"gtm":\#(p.pnTargetMode),"gte":\#(p.pnTargetE),"gtn":\#(p.pnTargetN),"gta":\#(p.pnTargetAltM),"gkp":\#(p.pnKpPos),"gkd":\#(p.pnKdVel),"glw":\#(law)\#(genTail)}"#
    }
    private static let identityFrame =
        #"{"type":"config_identity","uid":"boardA","un":"GTV","nid":5,"rid":1,"dt":"R","fw":"test"}"#
    // The other two #915 report frames: the extras re-adopt waits on all three.
    private static let servoFrame =
        #"{"type":"config_servo","sb2":0,"sb3":0,"sb4":0,"fmn":-60.0,"fmx":60.0,"faz":[0.0,90.0,180.0,270.0],"frv":0,"frrv":0}"#
    private static let rollFrame = #"{"type":"config_roll","n":0,"wp":[]}"#

    private func extras(enabled: Bool?, law: UInt8 = 0) -> RocketGuidanceExtras {
        let p = RocketProfile.makeDefault(name: "x")
        return RocketGuidanceExtras(
            navGain: p.pnNavGain, maxAccel: p.pnMaxAccel,
            accelToFin: p.pnAccelToFin, maxFinDeg: p.pnMaxFinDeg,
            minSpeed: p.pnMinSpeed, coastDelayMs: p.pnCoastDelayMs,
            targetMode: p.pnTargetMode, targetE: p.pnTargetE, targetN: p.pnTargetN,
            targetAltM: p.pnTargetAltM, kpPos: p.pnKpPos, kdVel: p.pnKdVel,
            guidanceLaw: law, enabled: enabled)
    }

    private func profile(guidance: Bool, law: UInt8 = 0) -> RocketProfile {
        var p = RocketProfile.makeDefault(name: "GTV")
        p.guidanceEnabled = guidance
        p.pnGuidanceLaw = law
        return p
    }

    private func config(fc: Bool?, law: UInt8 = 0, ge: Bool = true) -> RocketConfig {
        var c = RocketConfig()
        c.guidanceEnabled = ge
        c.guidanceExtras = extras(enabled: fc, law: law)
        return c
    }

    private func check(_ p: RocketProfile?, _ c: RocketConfig?,
                       connected: Bool = true, baseStation: Bool = false) -> GuidanceStateCheck {
        GuidanceStateCheck.evaluate(profile: p, config: c,
                                    isConnected: connected, isBaseStation: baseStation)
    }

    // MARK: - Parsing

    func testConfigGuid_GenIsTheFlightComputersOnOff() {
        let rocket = makeRocket()
        feed(rocket, Self.guidFrame(genTail: #","gen":false"#))
        XCTAssertEqual(rocket.rocketConfig?.guidanceEnabledOnFlightComputer, false)
        feed(rocket, Self.guidFrame(genTail: #","gen":true"#))
        XCTAssertEqual(rocket.rocketConfig?.guidanceEnabledOnFlightComputer, true)
    }

    func testConfigGuid_WithoutGenIsUnvouched() {
        // Rail off, or an out computer that predates the key.
        let rocket = makeRocket()
        feed(rocket, Self.guidFrame(genTail: ""))
        XCTAssertNotNil(rocket.rocketConfig?.guidanceExtras)
        XCTAssertNil(rocket.rocketConfig?.guidanceEnabledOnFlightComputer)
    }

    func testTheConfigFramesGeIsNotTheFlightComputers() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame(ge: true))
        XCTAssertEqual(rocket.rocketConfig?.guidanceEnabled, true)
        XCTAssertNil(rocket.rocketConfig?.guidanceEnabledOnFlightComputer)
    }

    // MARK: - Sending is not applying

    func testSendingAGuidanceConfigDoesNotMarkItApplied() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame(ge: false))
        feed(rocket, Self.guidFrame(genTail: #","gen":false"#))
        var sent: UInt8?
        rocket.commandTap = { cmd, _ in sent = cmd }
        let p = RocketProfile.makeDefault(name: "x")
        rocket.sendGuidanceConfig(enabled: true, navGain: p.pnNavGain, maxAccel: p.pnMaxAccel,
                                  accelToFin: p.pnAccelToFin, maxFinDeg: p.pnMaxFinDeg,
                                  minSpeed: p.pnMinSpeed, coastDelayMs: p.pnCoastDelayMs,
                                  targetMode: p.pnTargetMode, targetE: p.pnTargetE,
                                  targetN: p.pnTargetN, targetAlt: p.pnTargetAltM,
                                  kpPos: p.pnKpPos, kdVel: p.pnKdVel, guidanceLaw: 1)
        XCTAssertEqual(sent, 65)
        XCTAssertEqual(rocket.rocketConfig?.guidanceEnabled, false,
                       "a send that never arrived must not read as applied")
        XCTAssertEqual(rocket.rocketConfig?.guidanceEnabledOnFlightComputer, false)
    }

    // MARK: - The check

    func testGtvBlue_ProfileOnRocketOff() {
        let c = check(profile(guidance: true, law: 1), config(fc: false, law: 1))
        XCTAssertEqual(c, .rocketOff)
        XCTAssertTrue(c.isMismatch)
        XCTAssertEqual(c.message, "Rocket guidance OFF \u{2014} profile says ON")
    }

    func testProfileOffRocketOn() {
        let c = check(profile(guidance: false), config(fc: true))
        XCTAssertEqual(c, .rocketOn)
        XCTAssertTrue(c.isMismatch)
    }

    func testAgreementSaysNothing() {
        XCTAssertEqual(check(profile(guidance: true, law: 1), config(fc: true, law: 1)), .agrees)
        XCTAssertEqual(check(profile(guidance: false), config(fc: false)), .agrees)
        XCTAssertNil(GuidanceStateCheck.agrees.message)
    }

    func testBothOnDifferentLaws() {
        let c = check(profile(guidance: true, law: 1), config(fc: true, law: 0))
        XCTAssertEqual(c, .lawDiffers(rocket: 0, profile: 1))
        XCTAssertTrue(c.isMismatch)
        XCTAssertEqual(c.message, "Rocket flies PN \u{2014} profile says station-keep")
    }

    func testAGuidanceProfileWithoutTheFlightComputersWordIsUnverified() {
        let p = profile(guidance: true)
        for (c, why) in [
            (check(p, config(fc: true), connected: false), "not connected"),
            (check(p, config(fc: true), baseStation: true), "connect to the rocket directly to check"),
            (check(p, nil), "rocket settings not read yet"),
            (check(p, RocketConfig()), "flight computer not reporting (power it on)"),
            (check(p, config(fc: nil)), "this rocket's firmware doesn't report it"),
        ] {
            XCTAssertEqual(c, .unverified(why))
            XCTAssertFalse(c.isMismatch)
        }
    }

    func testTheOutComputersGeNeverVouches() {
        // "ge" true with no "gen": the stale-NVS case.  Unverified, not agrees.
        XCTAssertEqual(check(profile(guidance: true), config(fc: nil, ge: true)),
                       .unverified("this rocket's firmware doesn't report it"))
    }

    func testARollProfileWithNothingToCompareIsQuiet() {
        let p = profile(guidance: false)
        XCTAssertEqual(check(p, nil), .notApplicable)
        XCTAssertEqual(check(p, config(fc: nil)), .notApplicable)
        XCTAssertEqual(check(p, config(fc: true), baseStation: true), .notApplicable)
        XCTAssertNil(GuidanceStateCheck.notApplicable.message)
        XCTAssertEqual(check(nil, config(fc: true)), .notApplicable)
    }

    // MARK: - Adoption

    func testAdoptionTakesOnlyTheFlightComputersOnOff() {
        var p = profile(guidance: false)
        var c = config(fc: nil, ge: true)
        XCTAssertFalse(ActiveRocketSyncer.adopt(&p, from: c)
                           .contains(ActiveRocketSyncer.groupGuidanceEnable))
        XCTAssertFalse(p.guidanceEnabled, "the out computer's copy is not adopted")

        c.guidanceExtras?.enabled = true
        XCTAssertTrue(ActiveRocketSyncer.adopt(&p, from: c)
                          .contains(ActiveRocketSyncer.groupGuidanceEnable))
        XCTAssertTrue(p.guidanceEnabled, "the flight computer's own word wins (#915)")
    }

    // MARK: - Preflight

    func testSettingsStepBlocksOnAMismatch() {
        var ctx = PreflightAutoContext(isConnected: true, hasTelemetry: true)
        ctx.syncState = .synced
        ctx.guidance = .rocketOff
        XCTAssertEqual(PreflightChecklist.autoStatus(.settingsSynced, in: ctx),
                       .pending("Rocket guidance OFF \u{2014} profile says ON"))
        ctx.guidance = .agrees
        XCTAssertEqual(PreflightChecklist.autoStatus(.settingsSynced, in: ctx), .satisfied)
        ctx.guidance = .unverified("this rocket's firmware doesn't report it")
        XCTAssertEqual(PreflightChecklist.autoStatus(.settingsSynced, in: ctx), .satisfied,
                       "older out computers never say; the header carries that line")
    }

    // MARK: - End to end

    /// GTV Blue with the fix: the profile says station-keep, the out
    /// computer's NVS says "ge":true, the flight computer holds it off.  The
    /// rocket wins on connect, so the profile now shows it off; and when the
    /// user turns it back on and that send is lost, the check says so.
    func testGtvBlue_ConnectThenALostSend() {
        let syncer = ActiveRocketSyncer()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let mine = store.add(name: "GTV")
        store.update(mine.id) {
            $0.lastUsedUnitID = "boardA"; $0.guidanceEnabled = true; $0.pnGuidanceLaw = 1
        }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(ge: true))
        feed(rocket, Self.identityFrame)
        settleMainQueue()
        XCTAssertEqual(store.activeProfile?.guidanceEnabled, true,
                       "nothing from the flight computer yet: the profile is left alone")
        XCTAssertEqual(check(store.activeProfile, rocket.rocketConfig),
                       .unverified("flight computer not reporting (power it on)"))

        feed(rocket, Self.servoFrame)
        feed(rocket, Self.guidFrame(law: 1, genTail: #","gen":false"#))
        feed(rocket, Self.rollFrame)
        settleMainQueue()
        XCTAssertEqual(store.activeProfile?.guidanceEnabled, false, "the flight computer's off is adopted")

        store.update(mine.id) { $0.guidanceEnabled = true }   // the user re-enables; the send is lost
        XCTAssertEqual(check(store.activeProfile, rocket.rocketConfig), .rocketOff)
    }
}
