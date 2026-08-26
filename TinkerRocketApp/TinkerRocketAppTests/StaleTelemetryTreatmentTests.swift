import XCTest
@testable import TinkerRocketApp

/// #836 item 3 — a relayed rocket that had stopped transmitting rendered its
/// go/no-go and telemetry cards at FULL brightness, however old the last
/// packet was.
///
/// `RocketSubject.freshness()` already existed and already knew the answer,
/// but it drove exactly one thing: a 7-point colored dot on the rocket chip.
/// The focused/direct dashboard had the whole treatment — a status banner, a
/// 0.5 dim on every card — driven off `effectiveDataStatus`; the relayed
/// path never got wired up.
///
/// Dimming alone is not the whole fix. Every other card shows a MEASUREMENT,
/// which stays true of the moment it was taken. The health card shows a
/// RECOMMENDATION, and "Ready to fly" derived from minutes-old sensor bits is
/// an assertion about right now that nothing supports. It is held instead.
final class StaleTelemetryTreatmentTests: XCTestCase {

    // MARK: - Age text

    func testAgeText_SecondsUnderAMinuteThenMinutes() {
        XCTAssertEqual(RocketFreshness.ageText(0), "0 s")
        XCTAssertEqual(RocketFreshness.ageText(47), "47 s")
        XCTAssertEqual(RocketFreshness.ageText(59.9), "59 s")
        XCTAssertEqual(RocketFreshness.ageText(60), "1 min")
        XCTAssertEqual(RocketFreshness.ageText(3599), "59 min")
    }

    func testAgeText_NonFiniteDoesNotTrap() {
        // `.lost(lastSeen: nil)` yields a non-finite age, and Int(.infinity)
        // is a TRAP in Swift, not a large number.  A rocket in the roster
        // that has never been heard would have crashed the dashboard.
        XCTAssertEqual(RocketFreshness.ageText(.infinity), "unknown")
        XCTAssertEqual(RocketFreshness.ageText(.nan), "unknown")
        XCTAssertEqual(RocketFreshness.ageText(-1), "unknown")
    }

    // MARK: - Card dimming

    func testCardOpacity_MatchesTheFocusedDashboard() {
        // 0.5 is DashboardView's own staleOpacity.  Pinned so a rocket does
        // not change appearance when it gains or loses focus.
        XCTAssertEqual(RocketFreshness.live.cardOpacity, 1.0)
        XCTAssertEqual(RocketFreshness.stale(age: 10).cardOpacity, 0.5)
        XCTAssertEqual(RocketFreshness.lost(lastSeen: nil).cardOpacity, 0.35)
    }

    // MARK: - Age handed to the go/no-go card

    func testStaleAgeSec_NilOnlyWhileLive() {
        // nil is what tells the health card to assert a verdict, so it must
        // mean live and nothing else.
        XCTAssertNil(RocketFreshness.live.staleAgeSec())
        XCTAssertEqual(RocketFreshness.stale(age: 12).staleAgeSec(), 12)
    }

    func testStaleAgeSec_LostReportsAgeSinceLastSeen() {
        let now = Date()
        let seen = now.addingTimeInterval(-300)
        let age = try? XCTUnwrap(RocketFreshness.lost(lastSeen: seen).staleAgeSec(now: now))
        XCTAssertEqual(age ?? 0, 300, accuracy: 0.01)
    }

    func testStaleAgeSec_NeverHeardIsNonFiniteNotNil() {
        // Must NOT be nil: a rocket never heard from is the LEAST safe case
        // to let the card assert a verdict on.
        let age = RocketFreshness.lost(lastSeen: nil).staleAgeSec()
        XCTAssertNotNil(age)
        XCTAssertFalse(age!.isFinite)
    }

    // MARK: - The verdict itself

    func testVerdictAsserted_OnlyWhenLive() {
        XCTAssertFalse(HealthCardView.isHeld(staleAgeSec: nil))
        XCTAssertTrue(HealthCardView.isHeld(staleAgeSec: 4))
        XCTAssertTrue(HealthCardView.isHeld(staleAgeSec: .infinity))
    }

    func testReadyToFly_IsNotRestatedOnStaleData() {
        // THE regression: this is the exact string that was rendering in
        // green, at full brightness, on a rocket nobody had heard from.
        XCTAssertEqual(
            HealthCardView.readinessLabel(.ready, staleAgeSec: nil),
            "Ready to fly"
        )
        XCTAssertEqual(
            HealthCardView.readinessLabel(.ready, staleAgeSec: 47),
            "Held — data 47 s old"
        )
    }

    func testDoNotFly_IsAlsoHeld() {
        // Held in BOTH directions.  A stale "Do not fly" is equally
        // unsupported, and leaving red alone while greying green would make
        // the card quietly fail-safe in one direction and not the other —
        // which reads as a live verdict.
        XCTAssertEqual(
            HealthCardView.readinessLabel(.notReady, staleAgeSec: 90),
            "Held — data 1 min old"
        )
    }

    func testNeverHeard_SaysSoRatherThanShowingAnAge() {
        XCTAssertEqual(
            HealthCardView.readinessLabel(.ready, staleAgeSec: .infinity),
            "Held — no telemetry received"
        )
    }

    // MARK: - The advisory line

    func testAdvisoryRow_SilentWhileLive() {
        // The line renders nothing when it has nothing to say — the standing
        // rule for advisory surfaces on these dashboards.
        XCTAssertNil(StaleTelemetryAdvisoryRow(freshness: .live).messageForTest)
    }

    func testAdvisoryRow_WordsEachTier() {
        XCTAssertEqual(
            StaleTelemetryAdvisoryRow(freshness: .stale(age: 47)).messageForTest,
            "Telemetry 47 s old"
        )
        let now = Date()
        XCTAssertEqual(
            StaleTelemetryAdvisoryRow(freshness: .lost(lastSeen: now.addingTimeInterval(-300)),
                                      now: now).messageForTest,
            "Link lost — last heard 5 min ago"
        )
        XCTAssertEqual(
            StaleTelemetryAdvisoryRow(freshness: .lost(lastSeen: nil)).messageForTest,
            "No telemetry received"
        )
    }
}
