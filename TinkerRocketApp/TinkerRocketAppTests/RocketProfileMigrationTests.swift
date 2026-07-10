import XCTest
@testable import TinkerRocketApp

/// #449: the servo calibration is three datasheet numbers — the pulse endpoints
/// and the TOTAL fin travel swept between them. The endpoint angles are derived,
/// so a profile can no longer describe a servo that does not exist.
///
/// This supersedes the #378 migration these tests used to cover. #378 derived the
/// angles only when the JSON *omitted* them and deliberately left an explicit pair
/// alone — which is how `1250/1750 µs ↔ ±60°` (no physical servo: that span sweeps
/// 60° total on a standard arm) survived in the flying profile and made the FC
/// drive half the commanded deflection. The stale keys are now ignored outright.
final class RocketProfileMigrationTests: XCTestCase {

    private func decode(_ json: String) throws -> RocketProfile {
        try JSONDecoder().decode(RocketProfile.self, from: json.data(using: .utf8)!)
    }

    // MARK: - The standard-servo seed line

    func testStandardFinTravel_CanonicalSpans() {
        XCTAssertEqual(RocketProfile.standardFinTravelDeg(minUs: 1000, maxUs: 2000), 120.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.standardFinTravelDeg(minUs: 1250, maxUs: 1750), 60.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.standardFinTravelDeg(minUs: 1100, maxUs: 1900), 96.0, accuracy: 0.001)
    }

    // MARK: - Endpoints are derived, always symmetric about the travel

    func testEndpointAngles_AreHalfTheTravel() {
        var p = RocketProfile.makeDefault(name: "Fresh")
        XCTAssertEqual(p.finMinDeg, -60.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 60.0, accuracy: 0.001)

        p.finTravelDeg = 90.0            // e.g. a 90°-travel servo
        XCTAssertEqual(p.finMinDeg, -45.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 45.0, accuracy: 0.001)
    }

    func testFinDegForPulse_WalksTheCalibrationLine() {
        var p = RocketProfile.makeDefault(name: "Std")   // 1000/2000 µs, 120°
        XCTAssertEqual(p.finDeg(forPulse: 1000), -60.0, accuracy: 0.001)
        XCTAssertEqual(p.finDeg(forPulse: 1500), 0.0, accuracy: 0.001)
        XCTAssertEqual(p.finDeg(forPulse: 1750), 30.0, accuracy: 0.001)
        XCTAssertEqual(p.finDeg(forPulse: 2000), 60.0, accuracy: 0.001)
        XCTAssertEqual(p.finDegPerUs, 0.12, accuracy: 0.0001)

        // Narrowed endpoints on the same servo: the fin sweeps proportionally less.
        p.servoMinUs = 1250; p.servoMaxUs = 1750
        p.finTravelDeg = RocketProfile.standardFinTravelDeg(minUs: 1250, maxUs: 1750)
        XCTAssertEqual(p.finDeg(forPulse: 1250), -30.0, accuracy: 0.001)
        XCTAssertEqual(p.finDeg(forPulse: 1500), 0.0, accuracy: 0.001)
        XCTAssertEqual(p.finDeg(forPulse: 1750), 30.0, accuracy: 0.001)
        XCTAssertEqual(p.finDegPerUs, 0.12, accuracy: 0.0001, "same servo, same slope")
    }

    func testNonStandardServo_TravelIsHonoured() {
        // A 90°-travel servo driven across the full 1000–2000 µs span: the slope
        // is NOT the 0.12 °/µs standard line, and nothing in the model assumes it.
        var p = RocketProfile.makeDefault(name: "SG90-ish")
        p.finTravelDeg = 90.0
        XCTAssertEqual(p.finDegPerUs, 0.09, accuracy: 0.0001)
        XCTAssertEqual(p.finDeg(forPulse: 2000), 45.0, accuracy: 0.001)
    }

    func testZeroSpan_DoesNotDivideByZero() {
        var p = RocketProfile.makeDefault(name: "Degenerate")
        p.servoMinUs = 1500; p.servoMaxUs = 1500
        XCTAssertEqual(p.finDegPerUs, 0.0)
        XCTAssertEqual(p.finDeg(forPulse: 1800), 0.0)
        XCTAssertFalse(p.finDeg(forPulse: 1800).isNaN)
    }

    // MARK: - Decoding: stale angle keys are ignored (the #449 inversion)

    func testFlyingProfile_StaleExplicitAngles_AreIgnored() throws {
        // The exact pair every flight report carried: 1250/1750 µs declared ±60°.
        // Under #378 this was "explicit user data, untouched" and the FC flew a
        // 2×-off calibration. It must now resolve to the physical ±30°.
        let p = try decode(#"{"name":"RollyPoly","servoMinUs":1250,"servoMaxUs":1750,"finMinDeg":-60,"finMaxDeg":60}"#)
        XCTAssertEqual(p.servoMinUs, 1250)
        XCTAssertEqual(p.servoMaxUs, 1750)
        XCTAssertEqual(p.finTravelDeg, 60.0, accuracy: 0.001)
        XCTAssertEqual(p.finMinDeg, -30.0, accuracy: 0.001,
                       "1250 µs physically reaches −30°; the stored −60° was fiction")
        XCTAssertEqual(p.finMaxDeg, 30.0, accuracy: 0.001)
    }

    func testLegacyProfile_PulsesOnly_SeedsStandardTravel() throws {
        let p = try decode(#"{"name":"OldBird","servoMinUs":1250,"servoMaxUs":1750}"#)
        XCTAssertEqual(p.finMinDeg, -30.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 30.0, accuracy: 0.001)
    }

    func testLegacyProfile_CustomPulses_SeedProportionalTravel() throws {
        let p = try decode(#"{"name":"Custom","servoMinUs":1100,"servoMaxUs":1900}"#)
        XCTAssertEqual(p.finTravelDeg, 96.0, accuracy: 0.001)
        XCTAssertEqual(p.finMinDeg, -48.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 48.0, accuracy: 0.001)
    }

    func testProfileWithoutAnyServoKeys_KeepsFullTravelDefaults() throws {
        let p = try decode(#"{"name":"Fresh"}"#)
        XCTAssertEqual(p.servoMinUs, 1000)
        XCTAssertEqual(p.servoMaxUs, 2000)
        XCTAssertEqual(p.finTravelDeg, 120.0, accuracy: 0.001)
        XCTAssertEqual(p.finMinDeg, -60.0, accuracy: 0.001)
    }

    func testExplicitTravel_SurvivesDecode() throws {
        // Once a user states their servo's travel it is authoritative — the
        // standard-servo seed applies only when the key is absent.
        let p = try decode(#"{"name":"NineZero","servoMinUs":1000,"servoMaxUs":2000,"finTravelDeg":90}"#)
        XCTAssertEqual(p.finTravelDeg, 90.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 45.0, accuracy: 0.001)
    }

    // MARK: - Round trip

    func testRoundTrip_IsStable_AndDropsDerivedKeys() throws {
        let migrated = try decode(#"{"name":"OldBird","servoMinUs":1250,"servoMaxUs":1750,"finMinDeg":-60,"finMaxDeg":60}"#)
        let data = try JSONEncoder().encode(migrated)
        let json = String(data: data, encoding: .utf8)!
        XCTAssertFalse(json.contains("finMinDeg"), "derived angles must not be persisted")
        XCTAssertFalse(json.contains("finMaxDeg"))
        XCTAssertTrue(json.contains("finTravelDeg"))

        let again = try JSONDecoder().decode(RocketProfile.self, from: data)
        XCTAssertEqual(again.finTravelDeg, migrated.finTravelDeg, accuracy: 0.001)
        XCTAssertEqual(again.finMinDeg, -30.0, accuracy: 0.001)
        XCTAssertEqual(again.servoMinUs, 1250)
    }
}
