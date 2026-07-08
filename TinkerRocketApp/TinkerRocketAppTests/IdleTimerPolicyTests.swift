import XCTest
@testable import TinkerRocketApp

/// #385: `onDisconnect` used to re-enable screen auto-lock unconditionally —
/// with a base station AND a rocket connected, either one dropping re-armed
/// sleep while telemetry still streamed on the other. The idle timer must only
/// re-arm when the LAST connected device goes away.
final class IdleTimerPolicyTests: XCTestCase {

    @MainActor
    func testDisconnect_WithAnotherDeviceStillConnected_KeepsScreenAwake() {
        let fleet = BLEFleet()
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Test")
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        bs.fleet = fleet
        rocket.fleet = fleet
        bs.isConnected = true
        rocket.isConnected = true
        fleet.devices = [bs, rocket]

        UIApplication.shared.isIdleTimerDisabled = true
        rocket.onDisconnect()
        XCTAssertTrue(UIApplication.shared.isIdleTimerDisabled,
                      "BS is still connected and streaming — screen must stay awake")
    }

    @MainActor
    func testDisconnect_LastDevice_ReenablesAutoLock() {
        let fleet = BLEFleet()
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        rocket.fleet = fleet
        rocket.isConnected = true
        fleet.devices = [rocket]

        UIApplication.shared.isIdleTimerDisabled = true
        rocket.onDisconnect()
        XCTAssertFalse(UIApplication.shared.isIdleTimerDisabled,
                       "last device out — auto-lock must re-arm")
    }

    @MainActor
    func testDisconnect_NoFleet_ReenablesAutoLock() {
        // A device without a fleet reference (defensive path) behaves as before.
        let rocket = BLEDevice(peripheral: nil, name: "TR-R-Test")
        rocket.isConnected = true

        UIApplication.shared.isIdleTimerDisabled = true
        rocket.onDisconnect()
        XCTAssertFalse(UIApplication.shared.isIdleTimerDisabled)
    }
}
