import XCTest
@testable import TinkerRocketApp

/// Which settings the sheet offers while the flight computer is off, booting,
/// stuck or in rescue mode.  Reported 2026-10-16: an FC parked on "Starting
/// sensors" put a full-sheet "Initializing…" overlay over Settings, Firmware
/// included, so the only practical fix for a bad FC image on an assembled
/// rocket, an over-the-air update, could not be reached.  Android twin:
/// SettingsAccessTest.kt.
final class SettingsAccessTests: XCTestCase {

    private func access(connected: Bool = true,
                        baseStation: Bool = false,
                        telemetry: Bool = true,
                        railOn: Bool = true,
                        state: String = "READY",
                        boot: Bool = false) -> SettingsAccess {
        SettingsAccess.evaluate(isConnected: connected,
                                isBaseStation: baseStation,
                                hasReceivedTelemetry: telemetry,
                                pwrPinOn: railOn,
                                state: state,
                                bootStepReported: boot)
    }

    // MARK: - The shared table

    /// tests_cpp/fixtures/app_behavior/settings_access.json, which the Android
    /// suite runs too, located as OTATimeoutsTests finds its fixture.
    private static let fixtureURL: URL = {
        var url = URL(fileURLWithPath: #filePath)
        url.deleteLastPathComponent()   // SettingsAccessTests.swift
        url.deleteLastPathComponent()   // TinkerRocketAppTests
        url.deleteLastPathComponent()   // TinkerRocketApp
        return url.appendingPathComponent("tests_cpp/fixtures/app_behavior/settings_access.json")
    }()

    private static func fixtureName(_ a: SettingsAccess) -> String {
        switch a {
        case .full:                     return "full"
        case .rocketOff:                return "rocketOff"
        case .flightComputerNotRunning: return "flightComputerNotRunning"
        }
    }

    func testMatchesTheSharedFixture() throws {
        let data = try Data(contentsOf: Self.fixtureURL)
        let json = try XCTUnwrap(try JSONSerialization.jsonObject(with: data) as? [String: Any])
        let cases = try XCTUnwrap(json["cases"] as? [[String: Any]])
        XCTAssertGreaterThanOrEqual(cases.count, 10, "the fixture lost its cases")
        for c in cases {
            let name = try XCTUnwrap(c["name"] as? String)
            let got = SettingsAccess.evaluate(
                isConnected: try XCTUnwrap(c["connected"] as? Bool),
                isBaseStation: try XCTUnwrap(c["baseStation"] as? Bool),
                hasReceivedTelemetry: try XCTUnwrap(c["telemetry"] as? Bool),
                pwrPinOn: try XCTUnwrap(c["railOn"] as? Bool),
                state: try XCTUnwrap(c["state"] as? String),
                bootStepReported: try XCTUnwrap(c["bootStep"] as? Bool))
            XCTAssertEqual(Self.fixtureName(got), try XCTUnwrap(c["expect"] as? String), name)
        }
    }

    // MARK: - The reported case

    func testStuckBootKeepsFirmwareReachable() {
        // "Starting sensors", forever: INITIALIZATION with a boot step.
        let a = access(state: "INITIALIZATION", boot: true)
        XCTAssertEqual(a, .flightComputerNotRunning)
        XCTAssertTrue(a.allowsOutComputerSettings, "Firmware lives with the out computer's settings")
        XCTAssertFalse(a.allowsFlightComputerSettings)
    }

    func testRescueModeIsNotRunning() {
        // A rescue-mode FC never sends NonSensorData, so the state stays at the
        // zeroed INITIALIZATION while "bs" says rescue.
        XCTAssertEqual(access(state: "INITIALIZATION", boot: true), .flightComputerNotRunning)
    }

    func testSilentFlightComputerIsNotRunning() {
        // Rail on, no boot step at all (an FC that dies before its links).
        XCTAssertEqual(access(state: "INITIALIZATION", boot: false), .flightComputerNotRunning)
    }

    func testABootStepAloneMeansNotRunning() {
        // "bs" rides only setup_fc() and rescue mode; trust it even if a stale
        // state string from the previous session says otherwise.
        XCTAssertEqual(access(state: "READY", boot: true), .flightComputerNotRunning)
    }

    // MARK: - Running

    func testRunningStatesGetEverything() {
        for s in ["READY", "PRELAUNCH", "INFLIGHT", "LANDED", "MAG_CAL"] {
            let a = access(state: s)
            XCTAssertEqual(a, .full, s)
            XCTAssertTrue(a.allowsFlightComputerSettings, s)
            XCTAssertTrue(a.allowsOutComputerSettings, s)
        }
    }

    // MARK: - Rail off

    func testRailOffWinsOverInitialization() {
        // A rail-off FC usually reads INITIALIZATION too. Rocket-off is the root
        // fact, and it used to be covered by the INITIALIZATION overlay as well,
        // which hid the out computer's own update behind it.
        let a = access(railOn: false, state: "INITIALIZATION")
        XCTAssertEqual(a, .rocketOff)
        XCTAssertTrue(a.allowsOutComputerSettings)
        XCTAssertFalse(a.allowsFlightComputerSettings)
    }

    func testRailOffWithAStaleBootStepIsStillOff() {
        // The OC keeps forwarding the last boot step if the rail drops mid-boot.
        XCTAssertEqual(access(railOn: false, state: "INITIALIZATION", boot: true), .rocketOff)
    }

    // MARK: - Scopes that must not lock anything (#377, BS)

    func testNoFrameYetLocksNothing() {
        // Before the first frame telemetry is all defaults: pwr_pin_on false and
        // state "UNKNOWN". That is not a rocket that is off.
        XCTAssertEqual(access(telemetry: false, railOn: false, state: "UNKNOWN"), .full)
    }

    func testBaseStationIsNeverGated() {
        // The relay's telemetry never carries pwr_pin_on; a gate here would
        // lock a base station's Settings for good.
        XCTAssertEqual(access(baseStation: true, railOn: false, state: "INITIALIZATION", boot: true),
                       .full)
    }

    func testOfflineEditingStaysOpen() {
        XCTAssertEqual(access(connected: false, railOn: false, state: "INITIALIZATION"), .full)
    }

    // MARK: - What the notice says

    func testNoticeNamesRescueMode() {
        let rescue = TelemetryData.FcBootProgress(step: 64, elapsedMs: 400, degradedBits: 0)
        XCTAssertEqual(FlightComputerNotRunningSection.title(rescue), "Flight computer in rescue mode")
        XCTAssertTrue(FlightComputerNotRunningSection.explanation(rescue).contains("power-cycle"))
        XCTAssertTrue(FlightComputerNotRunningSection.explanation(rescue).contains("Update the flight computer"))
    }

    func testNoticeForAStartingFlightComputerPointsAtTheUpdate() {
        let starting = TelemetryData.FcBootProgress(step: 2, elapsedMs: 199, degradedBits: 0)
        XCTAssertEqual(FlightComputerNotRunningSection.title(starting), "Flight computer starting")
        XCTAssertTrue(FlightComputerNotRunningSection.explanation(starting)
            .contains("whether the flight computer answered"))
    }

    func testNoticeForASilentFlightComputer() {
        XCTAssertEqual(FlightComputerNotRunningSection.title(nil), "Waiting for flight computer")
    }
}
