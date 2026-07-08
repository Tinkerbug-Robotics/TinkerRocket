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
}
