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

    /// Replays the EXACT Virtual Rocket frames through the real announcer
    /// policy — the regression for the 2026-07-30 voice-callout outage,
    /// where the script spoke a dialect of firmware the announcer's gates
    /// couldn't hear ("DESCENT" isn't a firmware state; `mspd` was never
    /// fed so burnout — and the ascent altitude cadence gated behind it —
    /// never fired; no PRELAUNCH edge, so the one-shot flags never reset
    /// and a SECOND flight was completely mute).  Two replays, identical
    /// callouts each.  Android twin: `VirtualFlightAnnouncerTest`.
    func testVirtualFlightScript_drivesFullCalloutSequence() throws {
        let spy = FlightAnnouncerPolicyTests.SpySpeech()
        let clock = FlightAnnouncerPolicyTests.Clock()
        let announcer = FlightAnnouncer(speech: spy, now: { clock.t },
                                        unitSystem: { .metric },
                                        persistEnabled: false)
        announcer.isEnabled = true
        spy.spoken.removeAll()   // drop the "Voice ready" confirmation

        let decoder = JSONDecoder()
        let idleReady = try decoder.decode(
            TelemetryData.self,
            from: Data(#"{"rid":1,"run":"Booster","st":"READY","fs":16,"palt":0.2}"#.utf8))

        for flight in 0..<2 {
            let startCount = spy.spoken.count
            for tick in 0..<VirtualRocketDriver.flightTicks {
                clock.t = clock.t.addingTimeInterval(VirtualRocketDriver.flightTickSeconds)
                let json = VirtualRocketDriver.flightFrameJSON(tick: tick)
                let frame = try decoder.decode(TelemetryData.self, from: Data(json.utf8))
                announcer.processTelemetry(frame)
            }
            let calls = spy.texts.dropFirst(startCount)

            // One-shots: exactly one each, in flight order.
            XCTAssertEqual(calls.filter { $0.hasPrefix("Burnout") }.count, 1, "flight \(flight): \(calls)")
            XCTAssertEqual(calls.filter { $0.hasPrefix("Apogee") }.count, 1, "flight \(flight): \(calls)")
            XCTAssertEqual(calls.filter { $0.hasPrefix("Landed") }.count, 1, "flight \(flight): \(calls)")
            XCTAssertTrue(calls.first?.hasPrefix("Burnout") ?? false,
                          "nothing may speak before burnout (flight \(flight)): \(calls)")
            XCTAssertTrue(calls.last?.hasPrefix("Landed") ?? false, "flight \(flight): \(calls)")

            // Apogee altitude reaches the callout (script max = 400 m).
            let apogee = calls.first { $0.hasPrefix("Apogee") } ?? ""
            XCTAssertTrue(apogee.contains("400"), apogee)

            // Descent cadence: first 5 s after apogee, then every 10 s — the
            // 16 s descent yields exactly two, both saying "descending".
            XCTAssertEqual(calls.filter { $0.contains("descending") }.count, 2,
                           "flight \(flight): \(calls)")

            // Ascent altitude cadence between burnout and apogee, climbing.
            let ordered = Array(calls)
            let apogeeIx = ordered.firstIndex { $0.hasPrefix("Apogee") } ?? 1
            let ascent = ordered.isEmpty ? [] : Array(ordered[1..<max(apogeeIx, 1)])
            XCTAssertFalse(ascent.isEmpty, "flight \(flight): \(calls)")
            XCTAssertTrue(ascent.allSatisfy { $0.contains("climbing") }, "flight \(flight): \(calls)")

            // The drift under canopy gives the landed callout a distance.
            XCTAssertTrue(calls.last?.contains("away") ?? false, "\(calls.last ?? "")")

            // Back to READY idle, then the next flight's PRELAUNCH edge must
            // reset the one-shots (the second-flight-mute regression).
            clock.advance(5)
            announcer.processTelemetry(idleReady)
        }
    }
}
