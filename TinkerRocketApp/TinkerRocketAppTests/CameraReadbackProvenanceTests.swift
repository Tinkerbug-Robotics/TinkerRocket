import XCTest
@testable import TinkerRocketApp

/// #1472: the camera type the app shows — and adopts into the profile — must
/// be the one the FLIGHT computer will drive.  The `config` frame's `"camt"`
/// was the out computer's copy of the last cmd 33 it relayed; on the V9 bench
/// (2026-09-22) the FC was in GoPro mode while that copy said RunCam, the app
/// adopted RunCam on connect, and the next "Send all" would have switched the
/// FC.  The camera type now rides `config_pyro` with its own `"camsrc"`.
final class CameraReadbackProvenanceTests: XCTestCase {

    private var tempDir: URL!
    private var defaults: UserDefaults!

    override func setUp() {
        super.setUp()
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        defaults = UserDefaults(suiteName: "camera-readback-tests-\(UUID().uuidString)")
    }

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    private func makeStore() -> RocketProfileStore {
        RocketProfileStore(directory: tempDir, defaults: defaults)
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

    // The connect-time frames, in the order the out computer queues them.
    // Every value matches RocketProfile's defaults except the camera, so the
    // camera is the only thing adoption can change.
    private static func configFrame(camt: Int) -> String {
        #"{"type":"config","sb1":0,"shz":333,"smn":1000,"smx":2000,"kp":0.1200,"ki":0.0100,"kd":0.0000,"pmn":-20.0,"pmx":20.0,"sen":true,"gs":true,"ac":false,"rdly":0,"rcap":60.0,"kpang":2.00,"iwind":200.0,"ge":false,"camt":\#(camt),"irate":0}"#
    }
    private static func pyroFrame(camTail: String) -> String {
        #"{"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0,"p2e":false,"p2m":0,"p2v":100.0,"p3e":false,"p3m":0,"p3v":0.0,"p4e":false,"p4m":0,"p4v":0.0,"src":"fc","fnv":true\#(camTail)}"#
    }
    private static let identityFrame =
        #"{"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,"rid":1,"dt":"R","fw":"test"}"#

    // MARK: - Parsing

    func testProvenance_ParsesTheCameraSourceFromConfigPyro() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame(camt: 2))
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .configFrame,
                       "the config frame's camt says nothing about where it came from")

        feed(rocket, Self.pyroFrame(camTail: #","camt":1,"camsrc":"fc","camfnv":true"#))
        XCTAssertEqual(rocket.rocketConfig?.cameraType, 1)
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .flightComputer)

        feed(rocket, Self.pyroFrame(camTail: #","camt":2,"camsrc":"oc""#))
        XCTAssertEqual(rocket.rocketConfig?.cameraType, 2)
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .outComputerCache)
    }

    func testProvenance_AConfigRebuildKeepsTheSourcedCameraType() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame(camt: 1))
        feed(rocket, Self.pyroFrame(camTail: #","camt":1,"camsrc":"fc","camfnv":true"#))
        // A re-sent `config` frame (cmd 20) whose camt disagrees must not
        // overwrite the value config_pyro vouched for, nor its source.
        feed(rocket, Self.configFrame(camt: 2))
        XCTAssertEqual(rocket.rocketConfig?.cameraType, 1)
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .flightComputer)
    }

    func testProvenance_AnOutComputerThatPredatesTheKeyStaysUnverified() {
        let rocket = makeRocket()
        feed(rocket, Self.configFrame(camt: 2))
        feed(rocket, Self.pyroFrame(camTail: ""))   // no camt / camsrc at all
        XCTAssertEqual(rocket.rocketConfig?.cameraType, 2)
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .configFrame)
        XCTAssertTrue(rocket.rocketConfig?.unreportedGroups.contains("Camera") ?? false)

        // A camt without a recognised source is ignored, not trusted.
        feed(rocket, Self.pyroFrame(camTail: #","camt":1,"camsrc":"xx""#))
        XCTAssertEqual(rocket.rocketConfig?.cameraType, 2)
        XCTAssertEqual(rocket.rocketConfig?.cameraSource, .configFrame)
    }

    func testProvenance_ARocketWithNoCameraClaimsNothing() {
        // The mini has no camera: its config frame carries no camt.
        let rocket = makeRocket()
        feed(rocket, #"{"type":"config","shz":333}"#)
        XCTAssertNil(rocket.rocketConfig?.cameraSource)
        XCTAssertFalse(rocket.rocketConfig?.unreportedGroups.contains("Camera") ?? true,
                       "a board without a camera must not be told it can't verify one")
    }

    // MARK: - The precedence rule

    func testOnlyTheFlightComputersCameraTypeIsAdopted() {
        var cfg = RocketConfig()
        cfg.cameraType = 2
        for source in [CameraTypeSource.configFrame, .outComputerCache] {
            var p = RocketProfile.makeDefault(name: "x")
            p.cameraType = 1
            cfg.cameraSource = source
            XCTAssertFalse(ActiveRocketSyncer.adopt(&p, from: cfg)
                               .contains(ActiveRocketSyncer.groupCamera),
                           "\(source): the OC's copy is not the rocket's camera")
            XCTAssertEqual(p.cameraType, 1)
        }
        var p = RocketProfile.makeDefault(name: "x")
        p.cameraType = 1
        cfg.cameraSource = .flightComputer
        XCTAssertTrue(ActiveRocketSyncer.adopt(&p, from: cfg)
                          .contains(ActiveRocketSyncer.groupCamera))
        XCTAssertEqual(p.cameraType, 2, "the flight computer's own type wins")
    }

    func testTheCameraDoesNotHoldBackTheReportGroups() {
        // configReportGroupsMissing is what the #915 re-adopt waits on; an
        // unverifiable camera must not keep it waiting forever.
        var cfg = RocketConfig()
        cfg.cameraSource = .configFrame
        cfg.servoExtras = RocketServoExtras(
            bias2: 0, bias3: 0, bias4: 0, finMinDeg: -60, finMaxDeg: 60,
            finAzimuths: [0, 90, 180, 270], finReverseMask: 0,
            finRollReverseMask: 0, soundsEnabled: true)
        cfg.guidanceExtras = RocketGuidanceExtras(
            navGain: 3, maxAccel: 30, accelToFin: 0.5, maxFinDeg: 15,
            minSpeed: 30, coastDelayMs: 0, targetMode: 0,
            targetE: 0, targetN: 0, targetAltM: 0,
            kpPos: 0, kdVel: 0, guidanceLaw: 0)
        cfg.rollWaypoints = []
        XCTAssertFalse(cfg.configReportGroupsMissing)
        XCTAssertEqual(cfg.unreportedGroups, ["Camera"])
    }

    // MARK: - End to end through the syncer

    /// The 2026-09-22 bench state with the fix in both boards: the FC is in
    /// GoPro mode, the OC's cache says RunCam, and the profile says GoPro.
    /// The readback now carries the FC's type, so nothing changes.
    func testBenchState_TheProfileKeepsTheFlightComputersGoPro() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "boardA"; $0.cameraType = 1 }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(camt: 1))   // the OC serves the FC's type now
        feed(rocket, Self.pyroFrame(camTail: #","camt":1,"camsrc":"fc","camfnv":true"#))
        feed(rocket, Self.identityFrame)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.cameraType, 1)
        XCTAssertFalse(syncer.unreportedGroups.contains("Camera"))
    }

    /// The same boards with an out computer that predates #1472: its camt is
    /// the RunCam cache.  The old rule adopted it — the bug.  Now the profile
    /// keeps GoPro and the app says it cannot verify the camera.
    func testPreFixOutComputer_ItsCachedCameraIsNotAdopted() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "boardA"; $0.cameraType = 1 }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(camt: 2))
        feed(rocket, Self.pyroFrame(camTail: ""))
        feed(rocket, Self.identityFrame)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.cameraType, 1,
                       "the OC's cached RunCam must not overwrite the profile")
        XCTAssertTrue(syncer.unreportedGroups.contains("Camera"))
        XCTAssertNotEqual(syncer.syncState, .adopted([ActiveRocketSyncer.groupCamera]))
    }

    /// Connected with the rail off: only the OC's cache is on offer, so the
    /// profile keeps its own.  When the rail comes up and the FC's report
    /// lands, the FC's type is adopted — and said so.
    func testRailOffAtConnect_TheFlightComputersTypeIsAdoptedWhenItArrives() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "boardA"; $0.cameraType = 1 }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(camt: 2))
        feed(rocket, Self.pyroFrame(camTail: #","camt":2,"camsrc":"oc""#))
        feed(rocket, Self.identityFrame)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.cameraType, 1)
        XCTAssertTrue(syncer.unreportedGroups.contains("Camera"))

        // Rail on: the FC boots, reports RunCam, the OC re-publishes.
        feed(rocket, Self.pyroFrame(camTail: #","camt":2,"camsrc":"fc","camfnv":true"#))
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.cameraType, 2,
                       "the flight computer's own type is the rocket's camera")
        XCTAssertFalse(syncer.unreportedGroups.contains("Camera"))
        XCTAssertEqual(syncer.syncState, .adopted([ActiveRocketSyncer.groupCamera]))
    }

    /// "Camera" can now sit in unreportedGroups for good (an OC that predates
    /// "camsrc"), so the #915 re-adopt must key on the report groups alone —
    /// otherwise the fin layout / guidance / roll groups would never be
    /// adopted from such a rocket.
    func testReportGroupsAreStillReAdopted_WhileTheCameraStaysUnverified() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "boardA" }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        feed(rocket, Self.configFrame(camt: 2))
        feed(rocket, Self.pyroFrame(camTail: ""))
        feed(rocket, Self.identityFrame)
        settleMainQueue()
        XCTAssertEqual(store.activeProfile?.servoBias2, 0)

        feed(rocket, #"{"type":"config_servo","sb2":40,"sb3":0,"sb4":0,"fmn":-60.00,"fmx":60.00,"faz":[0.0,90.0,180.0,270.0],"frv":0,"frrv":0,"snd":true}"#)
        feed(rocket, #"{"type":"config_guid","gng":3.00,"gma":30.0,"gaf":0.50,"gmf":15.0,"gms":30.0,"gcd":0,"gtm":0,"gte":0.0,"gtn":0.0,"gta":0.0,"gkp":0.00,"gkd":0.00,"glw":0}"#)
        feed(rocket, #"{"type":"config_roll","n":0,"wp":[]}"#)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.servoBias2, 40,
                       "the report groups are adopted even though the camera is unverified")
        XCTAssertEqual(syncer.unreportedGroups, ["Camera"])
    }
}
