import XCTest
@testable import TinkerRocketApp

/// The Virtual Rocket's contract, at 1/100 time-scale (the OTASession test
/// pattern): idles in READY, flies exactly one flight per SIM_START through
/// the REAL parse/dispatch stack, honors SIM_STOP, and returns to READY —
/// never loops on its own.  Converged with Android's DemoMode behavior.
@MainActor
final class VirtualRocketTests: XCTestCase {

    private func waitUntil(_ what: String,
                           timeout: TimeInterval = 5.0,
                           _ predicate: () -> Bool) async throws {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if predicate() { return }
            try await Task.sleep(nanoseconds: 5_000_000)
        }
        XCTFail("timed out waiting for \(what)")
        struct TimedOut: Error {}
        throw TimedOut()
    }

    func testIdlesInReady_fliesOncePerStart_returnsToReady() async throws {
        let driver = VirtualRocketDriver()
        driver.timeScale = 1.0 / 100.0
        driver.start()
        defer { driver.stop() }
        let device = driver.device

        // Identity through the real demux; base-station type from the name.
        XCTAssertTrue(device.isConnected)
        XCTAssertEqual(device.deviceType, .baseStation)
        try await waitUntil("identity + READY idle") {
            device.unitName == "Virtual Base Station"
                && device.telemetry.state == "READY"
        }
        XCTAssertEqual(device.remoteRockets.count, 2, "Booster + Pad Rocket in the roster")

        // No self-starting: still READY after several scaled flight-lengths.
        try await Task.sleep(nanoseconds: 500_000_000)
        XCTAssertEqual(device.telemetry.state, "READY", "must not fly without SIM_START")

        // SIM_START exactly as the Simulation sheet sends it on a BS link:
        // the cmd-50 relay envelope targeting the focused rocket (#390).
        // (A bare cmd 6 also works — the tap unwraps both.)
        device.sendRelayCommand(targetRocketID: 1, innerCommand: 6)
        try await waitUntil("flight reaches the air") {
            device.telemetry.state == "INFLIGHT" || device.telemetry.state == "DESCENT"
        }
        try await waitUntil("landing") { device.telemetry.landed_flag }
        try await waitUntil("back to READY") { device.telemetry.state == "READY" }

        // And it STAYS there — one flight per start.
        try await Task.sleep(nanoseconds: 400_000_000)
        XCTAssertEqual(device.telemetry.state, "READY", "no endless loop")
    }

    func testSimStop_abortsMidFlight() async throws {
        let driver = VirtualRocketDriver()
        driver.timeScale = 1.0 / 100.0
        driver.start()
        defer { driver.stop() }
        let device = driver.device

        try await waitUntil("READY idle") { device.telemetry.state == "READY" }
        device.sendCommand(6)   // bare path stays supported
        try await waitUntil("airborne") { device.telemetry.state == "INFLIGHT" }

        device.sendRelayCommand(targetRocketID: 1, innerCommand: 7)   // SIM_STOP, relay-wrapped
        try await waitUntil("aborted to READY") { device.telemetry.state == "READY" }
        XCTAssertFalse(device.telemetry.landed_flag, "aborted flight never landed")
    }
}
