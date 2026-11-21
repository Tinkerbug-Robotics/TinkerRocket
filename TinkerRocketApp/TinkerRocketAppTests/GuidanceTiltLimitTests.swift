import XCTest
@testable import TinkerRocketApp

/// The guidance tilt limits: the off-vertical angle above which the flight
/// computer latches guidance off for the rest of the flight, before and after
/// burnout.  Once compile-time constants, now cmd 65's tail (offsets 45/49)
/// and read back in their own config_tilt frame.
final class GuidanceTiltLimitTests: XCTestCase {

    private var tempDir: URL!
    private var defaults: UserDefaults!

    override func setUp() {
        super.setUp()
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        defaults = UserDefaults(suiteName: "guidance-tilt-tests-\(UUID().uuidString)")
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

    // The readback frames, matching RocketProfile's defaults.
    private static let configFrame =
        #"{"type":"config","sb1":0,"shz":333,"smn":1000,"smx":2000,"kp":0.1200,"ki":0.0100,"kd":0.0000,"pmn":-20.0,"pmx":20.0,"sen":true,"gs":true,"ac":false,"rdly":0,"rcap":60.0,"kpang":2.00,"iwind":200.0,"ge":false,"camt":2,"irate":0}"#
    private static let identityFrame =
        #"{"type":"config_identity","uid":"boardA","un":"GTV","nid":5,"rid":1,"dt":"R","fw":"test"}"#
    private static let servoFrame =
        #"{"type":"config_servo","sb2":0,"sb3":0,"sb4":0,"fmn":-60.0,"fmx":60.0,"faz":[0.0,90.0,180.0,270.0],"frv":0,"frrv":0}"#
    private static var guidFrame: String {
        let p = RocketProfile.makeDefault(name: "x")
        return #"{"type":"config_guid","gng":\#(p.pnNavGain),"gma":\#(p.pnMaxAccel),"gaf":\#(p.pnAccelToFin),"gmf":\#(p.pnMaxFinDeg),"gms":\#(p.pnMinSpeed),"gcd":\#(p.pnCoastDelayMs),"gtm":\#(p.pnTargetMode),"gte":\#(p.pnTargetE),"gtn":\#(p.pnTargetN),"gta":\#(p.pnTargetAltM),"gkp":\#(p.pnKpPos),"gkd":\#(p.pnKdVel),"glw":0,"gen":false}"#
    }
    private static let rollFrame = #"{"type":"config_roll","n":0,"wp":[]}"#

    // MARK: - cmd 65 on the wire

    /// Byte-for-byte against the C++ emitter's GuidanceConfigFrame.
    func testCmd65MatchesTheGoldenFixture() {
        let rel = "commands/cmd65_guidance_53.bin"
        let side = WireFixtures.sidecar(rel)
        let rocket = makeRocket()
        var writes: [Data] = []
        rocket.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        rocket.sendGuidanceConfig(
            enabled: side.int("enable") != 0,
            navGain: side.f32("nav_gain"), maxAccel: side.f32("max_accel_mps2"),
            accelToFin: side.f32("accel_to_fin_deg"), maxFinDeg: side.f32("max_fin_deg"),
            minSpeed: side.f32("min_speed_mps"),
            coastDelayMs: UInt16(side.int("coast_delay_ms")),
            targetMode: UInt8(side.int("target_mode")),
            targetE: side.f32("target_e_m"), targetN: side.f32("target_n_m"),
            targetAlt: side.f32("target_alt_m"),
            kpPos: side.f32("kp_pos_per_s2"), kdVel: side.f32("kd_vel_per_s"),
            guidanceLaw: UInt8(side.int("guidance_law")),
            tiltLimitBoostDeg: side.f32("tilt_limit_boost_deg"),
            tiltLimitCoastDeg: side.f32("tilt_limit_coast_deg"))
        XCTAssertEqual(writes, [WireFixtures.data(rel)])
    }

    func testTheTiltLimitsAreAppendedAt45And49() {
        let rocket = makeRocket()
        var sent: Data?
        rocket.commandTap = { _, payload in sent = payload }
        let p = RocketProfile.makeDefault(name: "x")
        rocket.sendGuidanceConfig(enabled: false, navGain: p.pnNavGain, maxAccel: p.pnMaxAccel,
                                  accelToFin: p.pnAccelToFin, maxFinDeg: p.pnMaxFinDeg,
                                  minSpeed: p.pnMinSpeed, coastDelayMs: p.pnCoastDelayMs,
                                  targetMode: p.pnTargetMode, targetE: p.pnTargetE,
                                  targetN: p.pnTargetN, targetAlt: p.pnTargetAltM,
                                  kpPos: p.pnKpPos, kdVel: p.pnKdVel, guidanceLaw: 1,
                                  tiltLimitBoostDeg: 12.5, tiltLimitCoastDeg: 40)
        guard let bytes = sent.map({ [UInt8]($0) }) else { return XCTFail("nothing sent") }
        XCTAssertEqual(bytes.count, 53)
        XCTAssertEqual(bytes[44], 1, "guidance_law stays the last byte of the 45-byte block")
        func f32(_ at: Int) -> Float {
            bytes.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: at, as: Float.self) }
        }
        XCTAssertEqual(f32(45), 12.5)
        XCTAssertEqual(f32(49), 40)
    }

    // MARK: - config_tilt readback

    func testConfigTiltIsParsed() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config_tilt","gtlb":15.0,"gtlc":20.0}"#)
        XCTAssertEqual(rocket.rocketConfig?.guidanceTilt,
                       RocketGuidanceTilt(boostDeg: 15, coastDeg: 20))
        feed(rocket, #"{"type":"config_tilt","gtlb":25.0,"gtlc":32.5}"#)
        XCTAssertEqual(rocket.rocketConfig?.guidanceTilt,
                       RocketGuidanceTilt(boostDeg: 25, coastDeg: 32.5))
    }

    func testConfigTiltNeedsBothKeys() {
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config_tilt","gtlb":15.0}"#)
        XCTAssertNil(rocket.rocketConfig?.guidanceTilt)
        feed(rocket, #"{"type":"config_tilt","gtlc":20.0}"#)
        XCTAssertNil(rocket.rocketConfig?.guidanceTilt)
    }

    /// Older firmware never sends config_tilt: config_guid still parses and
    /// the tilt limits stay unknown.
    func testConfigGuidWithoutConfigTiltLeavesTheLimitsUnknown() {
        let rocket = makeRocket()
        feed(rocket, Self.guidFrame)
        XCTAssertNotNil(rocket.rocketConfig?.guidanceExtras)
        XCTAssertNil(rocket.rocketConfig?.guidanceTilt)
    }

    /// A re-sent `config` frame rebuilds RocketConfig; the limits must survive it.
    func testAConfigRebuildKeepsTheLimits() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame)
        feed(rocket, #"{"type":"config_tilt","gtlb":25.0,"gtlc":32.5}"#)
        feed(rocket, Self.configFrame)
        XCTAssertEqual(rocket.rocketConfig?.guidanceTilt,
                       RocketGuidanceTilt(boostDeg: 25, coastDeg: 32.5))
    }

    // MARK: - Profile

    func testDefaultsEqualTheFirmwares() {
        let p = RocketProfile.makeDefault(name: "x")
        XCTAssertEqual(p.pnTiltLimitBoostDeg, 15)   // config::GUIDANCE_TILT_LIMIT_BOOST_DEG
        XCTAssertEqual(p.pnTiltLimitCoastDeg, 20)   // config::GUIDANCE_TILT_LIMIT_COAST_DEG
    }

    func testAProfileFromBeforeTheSettingLoadsWithTheDefaults() throws {
        let old = #"{"name":"Old","pnKpPos":0.8,"pnKdVel":1.5,"pnGuidanceLaw":1}"#.data(using: .utf8)!
        let p = try JSONDecoder().decode(RocketProfile.self, from: old)
        XCTAssertEqual(p.pnTiltLimitBoostDeg, 15)
        XCTAssertEqual(p.pnTiltLimitCoastDeg, 20)
        XCTAssertEqual(p.pnGuidanceLaw, 1)
    }

    func testTheLimitsRoundTrip() throws {
        var p = RocketProfile.makeDefault(name: "x")
        p.pnTiltLimitBoostDeg = 12.5
        p.pnTiltLimitCoastDeg = 45
        let back = try JSONDecoder().decode(RocketProfile.self, from: JSONEncoder().encode(p))
        XCTAssertEqual(back.pnTiltLimitBoostDeg, 12.5)
        XCTAssertEqual(back.pnTiltLimitCoastDeg, 45)
    }

    // MARK: - Connect burst, end to end

    /// The out computer sends config_tilt between config_guid and config_roll,
    /// so the extras re-adopt (which waits on config_roll) takes it.
    private func connect(sendTilt: String?) -> RocketProfile? {
        let syncer = ActiveRocketSyncer()
        let store = RocketProfileStore(directory: tempDir, defaults: defaults)
        let mine = store.add(name: "GTV")
        store.update(mine.id) {
            $0.lastUsedUnitID = "boardA"
            $0.pnTiltLimitBoostDeg = 10; $0.pnTiltLimitCoastDeg = 30
        }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame)
        feed(rocket, Self.identityFrame)
        settleMainQueue()
        feed(rocket, Self.servoFrame)
        feed(rocket, Self.guidFrame)
        if let sendTilt { feed(rocket, sendTilt) }
        feed(rocket, Self.rollFrame)
        settleMainQueue()
        return store.activeProfile
    }

    func testTheRocketsLimitsAreAdoptedOnConnect() {
        let p = connect(sendTilt: #"{"type":"config_tilt","gtlb":25.0,"gtlc":32.5}"#)
        XCTAssertEqual(p?.pnTiltLimitBoostDeg, 25)
        XCTAssertEqual(p?.pnTiltLimitCoastDeg, 32.5)
    }

    func testOlderFirmwareLeavesTheProfilesLimitsAlone() {
        let p = connect(sendTilt: nil)
        XCTAssertEqual(p?.pnTiltLimitBoostDeg, 10)
        XCTAssertEqual(p?.pnTiltLimitCoastDeg, 30)
    }
}
