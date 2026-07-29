import XCTest
@testable import TinkerRocketApp

/// The OTA timing contract, checked two ways.
///
/// `testEveryRelayCrossingStageIsStretchedOnTheFCPath` is the one that matters:
/// it states the property the numbers have to satisfy rather than the numbers
/// themselves, so it catches a NEW stage added without an FC window as well as
/// the bug it was written for. Bench 2026-07-28 found `finish` flat at 15 s on
/// both paths — it expired while a live 591.7 kB FC flash was still running, so
/// the app declared failure on a transfer that then succeeded (#627). A plain
/// Swift↔Kotlin diff would NOT have caught that: both platforms said 15 s and
/// agreed with each other.
///
/// `testMatchesTheSharedFixture` is the anti-drift half — the Kotlin suite
/// asserts the same file, so changing one platform's table without the other
/// fails that platform's build.
final class OTATimeoutsTests: XCTestCase {

    /// tests_cpp/fixtures/app_behavior/, located the same way WireFixtures
    /// finds the wire corpus (#filePath — no bundle folder reference needed).
    private static let fixtureURL: URL = {
        var url = URL(fileURLWithPath: #filePath)
        url.deleteLastPathComponent()   // OTATimeoutsTests.swift
        url.deleteLastPathComponent()   // TinkerRocketAppTests
        url.deleteLastPathComponent()   // TinkerRocketApp
        return url.appendingPathComponent("tests_cpp/fixtures/app_behavior/ota_timeouts.json")
    }()

    func testEveryRelayCrossingStageIsStretchedOnTheFCPath() {
        for stage in OTAStage.allCases {
            let local = OTATimeouts.seconds(stage, targetIsFC: false)
            let fc = OTATimeouts.seconds(stage, targetIsFC: true)
            if stage.crossesRelay {
                XCTAssertGreaterThan(
                    fc, local,
                    "\(stage.rawValue) waits on the FC over the relay, so the FC "
                    + "window must exceed the local one (was fc=\(fc) local=\(local))")
            } else {
                XCTAssertEqual(
                    local, fc,
                    "\(stage.rawValue) is a local link event — the relay plays no "
                    + "part, so both paths must wait the same")
            }
        }
    }

    func testMatchesTheSharedFixture() throws {
        let data = try Data(contentsOf: Self.fixtureURL)
        let json = try XCTUnwrap(
            try JSONSerialization.jsonObject(with: data) as? [String: Any])
        let stages = try XCTUnwrap(json["stages"] as? [[String: Any]])

        XCTAssertEqual(stages.count, OTAStage.allCases.count,
                       "fixture lists \(stages.count) stages but Swift has "
                       + "\(OTAStage.allCases.count)")

        for entry in stages {
            let name = try XCTUnwrap(entry["name"] as? String)
            let stage = try XCTUnwrap(OTAStage(rawValue: name),
                                      "fixture stage '\(name)' has no Swift OTAStage")
            XCTAssertEqual(entry["crossesRelay"] as? Bool, stage.crossesRelay,
                           "\(name) crossesRelay disagrees with the fixture")
            XCTAssertEqual(try XCTUnwrap(entry["localMs"] as? Double) / 1000.0,
                           OTATimeouts.seconds(stage, targetIsFC: false),
                           "\(name) local window disagrees with the fixture")
            XCTAssertEqual(try XCTUnwrap(entry["fcMs"] as? Double) / 1000.0,
                           OTATimeouts.seconds(stage, targetIsFC: true),
                           "\(name) FC window disagrees with the fixture")
        }

        XCTAssertEqual(try XCTUnwrap(json["pollMs"] as? Double) / 1000.0,
                       OTATimeouts.pollSeconds)
        XCTAssertEqual(try XCTUnwrap(json["fcRelayMaxBytesPerSec"] as? Double),
                       OTATimeouts.fcRelayMaxBytesPerSec,
                       "#627 relay cap disagrees with the fixture")
    }

    func testFCRelayPacerHoldsTheCapAndCreditsElapsedTime() {
        let rate = OTATimeouts.fcRelayMaxBytesPerSec
        let oneSecond = Int(rate)

        // One second's worth of bytes, no time spent yet → wait the full second.
        XCTAssertEqual(
            OTATimeouts.fcRelayPaceDelay(bytesSent: oneSecond, elapsed: 0), 1.0,
            accuracy: 0.001)

        // Same bytes, but the writes already took 0.4 s — credit that rather
        // than adding to it, or the effective rate drifts under the cap.
        XCTAssertEqual(
            OTATimeouts.fcRelayPaceDelay(bytesSent: oneSecond, elapsed: 0.4), 0.6,
            accuracy: 0.001)

        // Already slower than the cap → never wait, never go backwards.
        XCTAssertEqual(
            OTATimeouts.fcRelayPaceDelay(bytesSent: oneSecond, elapsed: 1.0), 0)
        XCTAssertEqual(
            OTATimeouts.fcRelayPaceDelay(bytesSent: oneSecond, elapsed: 5.0), 0)

        // The rate enforced is the one measured working on the bench: iOS ran
        // the relay at 11.4 kB/s, Android at ~68 kB/s wedged the OC's mbuf pool.
        XCTAssertTrue((11_000...13_000).contains(rate),
                      "cap should sit at iOS's proven-good relay rate, was \(rate) B/s")
    }
}
