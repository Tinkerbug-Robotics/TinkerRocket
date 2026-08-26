import XCTest
@testable import TinkerRocketApp

/// Regression tests for #375: the syncer used to attach only on the fleet's
/// 0→1 connection edge, so a rocket connecting after the base station (or an
/// auto-reconnect, which creates a NEW BLEDevice) silently got no profile
/// push, and its state rendered as .idle → EmptyView (no warning at all).
/// The dashboard now re-attaches on every device-list / active-device change,
/// which requires attach() to be (a) idempotent for the same device object and
/// (b) visibly non-idle (.awaitingSync) from the moment a rocket is attached.
final class ActiveRocketSyncerLifecycleTests: XCTestCase {

    private var tempDir: URL!
    private var defaults: UserDefaults!

    override func setUp() {
        super.setUp()
        tempDir = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        defaults = UserDefaults(suiteName: "syncer-lifecycle-tests-\(UUID().uuidString)")
    }

    private func makeStore() -> RocketProfileStore {
        RocketProfileStore(directory: tempDir, defaults: defaults)
    }

    private func makeRocket() -> BLEDevice {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        return d
    }

    /// Let queued main-queue work run: the syncer's readback trigger and the
    /// active-profile subscription both hop through `.receive(on: .main)`.
    private func settleMainQueue(_ turns: Int = 5) {
        for _ in 0..<turns {
            let exp = expectation(description: "main-queue turn")
            DispatchQueue.main.async { exp.fulfill() }
            wait(for: [exp], timeout: 1)
        }
    }

    /// Feed the connect-time readback the syncer waits on.
    private func reportConfig(_ d: BLEDevice, unitID: String,
                              mutate: (inout RocketConfig) -> Void = { _ in }) {
        var cfg = RocketConfig()
        mutate(&cfg)
        d.rocketConfig = cfg
        d.unitID = unitID
    }

    // MARK: - #915: the rocket keeps its own settings

    /// The headline case: the phone is holding a DIFFERENT airframe's profile
    /// and connects to a known rocket.  The board's own profile must become
    /// active and nothing may be written to the vehicle.
    func testConnectBindsTheBoardsProfileAndPushesNothing() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) { $0.lastUsedUnitID = "BOARD1" }
        let other = store.add(name: "Some other rocket")
        store.setActive(other.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        reportConfig(rocket, unitID: "BOARD1")
        settleMainQueue()

        XCTAssertEqual(store.activeId, mine.id,
                       "the board's own profile is the one that becomes active")
        XCTAssertNotEqual(syncer.syncState, .syncing,
                          "connecting must never write settings to the rocket (#915)")
    }

    /// A board the app has never seen must be adopted as its own profile —
    /// pushing whatever happened to be active is the failure #915 names.
    func testUnknownBoardIsAdoptedAsANewProfile() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let other = store.add(name: "Some other rocket")
        store.update(other.id) { $0.pidKp = 0.99 }
        store.setActive(other.id)

        let rocket = makeRocket()
        rocket.unitName = "Newcomer"
        syncer.attach(device: rocket, store: store)
        reportConfig(rocket, unitID: "BOARD9")
        settleMainQueue()

        XCTAssertNotEqual(store.activeId, other.id)
        XCTAssertEqual(store.activeProfile?.lastUsedUnitID, "BOARD9")
        XCTAssertEqual(syncer.createdProfileName, "Newcomer")
        XCTAssertNotEqual(syncer.syncState, .syncing)
        XCTAssertEqual(store.profiles.first { $0.id == other.id }?.pidKp, 0.99,
                       "the untouched profile keeps its own values")
    }

    /// The rocket's values win over the profile's, and the difference is
    /// reported rather than applied silently.
    func testDisagreementAdoptsTheRocketsValueAndSaysSo() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) {
            $0.lastUsedUnitID = "BOARD1"
            $0.cameraType = 0
        }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        reportConfig(rocket, unitID: "BOARD1") { $0.cameraType = 1 }
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.cameraType, 1, "the rocket's value wins")
        XCTAssertEqual(syncer.syncState,
                       .adopted([ActiveRocketSyncer.groupCamera]))
    }

    // MARK: - Binding is exclusive (#915 bench regression)

    /// Bench 2026-08-25: assign a second profile to a rocket, connect to a
    /// different rocket, come back — and the selection had reverted. Both
    /// profiles still claimed the board, and the lookup takes the first match
    /// in a list sorted by NAME, so the winner was decided alphabetically
    /// instead of by what the user chose.
    func testAssigningASecondProfileReleasesTheFirst() {
        let store = makeStore()
        let alpha = store.add(name: "Alpha")        // sorts FIRST by name
        let zulu  = store.add(name: "Zulu")
        store.bind(alpha.id, toUnitID: "BOARD1")
        store.bind(zulu.id,  toUnitID: "BOARD1")    // the user's later choice

        XCTAssertNil(store.profiles.first { $0.id == alpha.id }?.lastUsedUnitID,
                     "the earlier profile must release the board")
        XCTAssertEqual(store.profiles.first { $0.id == zulu.id }?.lastUsedUnitID, "BOARD1")
        XCTAssertEqual(store.profiles.filter { $0.lastUsedUnitID == "BOARD1" }.count, 1,
                       "a board is claimed by exactly one profile")
    }

    /// The symptom as reported: come back to the rocket and get the profile
    /// you actually chose, not the alphabetically-earlier one.
    func testReconnectBindsTheChosenProfileNotTheAlphabeticallyFirst() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let alpha = store.add(name: "Alpha")
        let zulu  = store.add(name: "Zulu")
        store.bind(alpha.id, toUnitID: "BOARD1")
        store.bind(zulu.id,  toUnitID: "BOARD1")    // chosen later, so it wins
        store.setActive(alpha.id)                   // simulate being elsewhere

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)
        reportConfig(rocket, unitID: "BOARD1")
        settleMainQueue()

        XCTAssertEqual(store.activeId, zulu.id,
                       "the board comes back on the profile the user put on it")
    }

    /// End-to-end through the real JSON parser, in the order the out computer
    /// actually queues the readback: config, config_pyro, config_identity,
    /// then imu_orient.  The orientation setting rides that LAST frame, so it
    /// lands after the syncer has already reconciled everything else — the
    /// case the direct-assignment tests above can't reach.
    func testLateOrientationFrameIsAdoptedToo() {
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let mine = store.add(name: "Atlas")
        store.update(mine.id) {
            $0.lastUsedUnitID = "boardA"
            $0.imuOrientSetting = 7        // manual, carried over from elsewhere
        }
        store.setActive(mine.id)

        let rocket = makeRocket()
        syncer.attach(device: rocket, store: store)

        func feed(_ json: String) {
            rocket.parseTelemetryData(json.data(using: .utf8))
        }
        // Values chosen to match the profile so only orientation differs.
        feed("""
            {"type":"config","sb1":0,"shz":333,"smn":1000,"smx":2000,            "kp":0.1200,"ki":0.0100,"kd":0.0000,"pmn":-20.0,"pmx":20.0,            "sen":true,"gs":true,"ac":false,"rdly":0,            "rcap":60.0,"kpang":2.00,"iwind":40.0,            "ge":false,"camt":2,"irate":0}
            """)
        feed("""
            {"type":"config_pyro","p1e":false,"p1m":0,"p1v":1.0,            "p2e":false,"p2m":0,"p2v":100.0,"p3e":false,"p3m":0,"p3v":0.0,            "p4e":false,"p4m":0,"p4v":0.0}
            """)
        feed("""
            {"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,            "rid":1,"dt":"R","fw":"test"}
            """)
        settleMainQueue()

        XCTAssertEqual(store.activeId, mine.id)
        XCTAssertNotEqual(syncer.syncState, .syncing,
                          "the readback alone must not trigger a write")

        // The orientation frame the OC queues last.
        feed("""
            {"type":"imu_orient","code":0,"mode":0,"name":"+X","set":255}
            """)
        settleMainQueue()

        XCTAssertEqual(store.activeProfile?.imuOrientSetting, 0xFF,
                       "this rocket is on pad auto-detect and stays that way")
        XCTAssertEqual(syncer.syncState,
                       .adopted([ActiveRocketSyncer.groupImuOrientation]))
    }

    func testAttachRocket_IsVisiblyAwaitingSync_NotSilentIdle() {
        let syncer = ActiveRocketSyncer()
        syncer.attach(device: makeRocket(), store: makeStore())
        XCTAssertEqual(syncer.syncState, .awaitingSync,
                       "connected-but-unsynced must be visible (#375), not .idle/EmptyView")
    }

    func testAttachBaseStation_StaysIdle() {
        let syncer = ActiveRocketSyncer()
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        syncer.attach(device: bs, store: makeStore())
        XCTAssertEqual(syncer.syncState, .idle,
                       "base station never receives a profile push")
    }

    func testReattachSameDevice_IsIdempotent() {
        // The dashboard re-attaches on every fleet.devices / activeDeviceID
        // change; a redundant call for the SAME device object must not tear
        // down subscriptions or bounce the state.
        let syncer = ActiveRocketSyncer()
        let rocket = makeRocket()
        let store = makeStore()

        syncer.attach(device: rocket, store: store)
        let stateAfterFirst = syncer.syncState
        syncer.attach(device: rocket, store: store)
        XCTAssertEqual(syncer.syncState, stateAfterFirst,
                       "same-pair re-attach must be a no-op")
    }

    func testAttachNewDeviceObject_ReattachesFresh() {
        // A reconnect creates a NEW BLEDevice — the exact #375 gap. Attaching
        // the new object must run a full re-attach (fresh .awaitingSync), not
        // be swallowed by the idempotence guard.
        let syncer = ActiveRocketSyncer()
        let store = makeStore()

        syncer.attach(device: makeRocket(), store: store)
        XCTAssertEqual(syncer.syncState, .awaitingSync)

        let reconnected = makeRocket()   // new object, same physical rocket
        syncer.attach(device: reconnected, store: store)
        XCTAssertEqual(syncer.syncState, .awaitingSync,
                       "new device object must get its own attach cycle")
    }

    func testDetach_ResetsToIdle() {
        let syncer = ActiveRocketSyncer()
        syncer.attach(device: makeRocket(), store: makeStore())
        syncer.detach()
        XCTAssertEqual(syncer.syncState, .idle)
    }

    // MARK: - Role flip after a late config_identity (renamed devices)

    func testRoleFlipToRocket_ReattachesInsteadOfStayingLatched() {
        // A rocket renamed "SUBSONIC" parses as .baseStation via the legacy
        // "BS" substring check, so the first attach idles it.  When the
        // config_identity readback corrects the type, a re-attach for the
        // SAME device object must break through the idempotence guard —
        // otherwise profile sync stays silently dead all session (#375's
        // failure mode through a different door).
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let device = BLEDevice(peripheral: nil, name: "SUBSONIC")
        device.isConnected = true
        device.deviceType = .baseStation   // as the name parse / a stale seed left it

        syncer.attach(device: device, store: store)
        XCTAssertEqual(syncer.syncState, .idle)

        device.deviceType = .rocket        // config_identity "dt" correction
        syncer.attach(device: device, store: store)
        XCTAssertEqual(syncer.syncState, .awaitingSync,
                       "role correction must re-run the rocket attach path")
    }

    func testRoleFlipToBaseStation_DropsRocketSubscriptions() {
        // Mirror image: a base station mis-seeded as a rocket must fall back
        // to the read-only .idle role once the readback says "B".
        let syncer = ActiveRocketSyncer()
        let store = makeStore()
        let device = makeRocket()

        syncer.attach(device: device, store: store)
        XCTAssertEqual(syncer.syncState, .awaitingSync)

        device.deviceType = .baseStation
        syncer.attach(device: device, store: store)
        XCTAssertEqual(syncer.syncState, .idle,
                       "a corrected base station must not keep a rocket sync pipeline")
    }
}
