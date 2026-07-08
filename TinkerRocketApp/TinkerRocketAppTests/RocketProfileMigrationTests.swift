import XCTest
@testable import TinkerRocketApp

/// Regression tests for #378: pre-#267 profile JSON carries servo pulse limits
/// (old defaults 1250/1750 µs) but no finMinDeg/finMaxDeg keys. The decoder
/// used to backfill the missing angles with the new ±60° defaults, pairing
/// "1250–1750 µs ↔ ±60°" — a mapping whose commanded-vs-physical fin angle is
/// off ~2×, so an untouched old profile silently flew with about half the
/// roll/guidance authority the gains were retuned for. The migration now
/// derives missing angles from the pulses via the 1:1 servo-arm line
/// (1000 µs = −60°, 2000 µs = +60°), keeping the pair self-consistent without
/// changing the hardware's actual travel.
final class RocketProfileMigrationTests: XCTestCase {

    private func decode(_ json: String) throws -> RocketProfile {
        try JSONDecoder().decode(RocketProfile.self, from: json.data(using: .utf8)!)
    }

    // MARK: - The pulse→angle line itself

    func testFinDegForPulse_CanonicalLine() {
        XCTAssertEqual(RocketProfile.finDegForPulse(1000), -60.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.finDegForPulse(1500), 0.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.finDegForPulse(2000), 60.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.finDegForPulse(1250), -30.0, accuracy: 0.001)
        XCTAssertEqual(RocketProfile.finDegForPulse(1750), 30.0, accuracy: 0.001)
    }

    // MARK: - The #378 regression

    func testPre267Profile_OldDefaultPulses_GetConsistentAngles() throws {
        // The exact case from the issue: old defaults, no fin-angle keys.
        let p = try decode(#"{"name":"OldBird","servoMinUs":1250,"servoMaxUs":1750}"#)
        XCTAssertEqual(p.servoMinUs, 1250)
        XCTAssertEqual(p.servoMaxUs, 1750)
        XCTAssertEqual(p.finMinDeg, -30.0, accuracy: 0.001,
                       "1250 µs physically reaches −30°, not the ±60° default")
        XCTAssertEqual(p.finMaxDeg, 30.0, accuracy: 0.001)
    }

    func testPre267Profile_CustomPulses_GetProportionalAngles() throws {
        // A user-tuned old profile keeps its physical travel, correctly labeled.
        let p = try decode(#"{"name":"Custom","servoMinUs":1100,"servoMaxUs":1900}"#)
        XCTAssertEqual(p.finMinDeg, -48.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 48.0, accuracy: 0.001)
    }

    func testProfileWithoutAnyServoKeys_KeepsFullTravelDefaults() throws {
        // No pulses AND no angles: defaults are 1000/2000 ↔ ±60 — the formula
        // reproduces the defaults exactly, so nothing changes for new profiles.
        let p = try decode(#"{"name":"Fresh"}"#)
        XCTAssertEqual(p.servoMinUs, 1000)
        XCTAssertEqual(p.servoMaxUs, 2000)
        XCTAssertEqual(p.finMinDeg, -60.0, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 60.0, accuracy: 0.001)
    }

    func testPost267Profile_ExplicitAngles_Untouched() throws {
        // A profile that already carries angle keys is never rewritten — even
        // if the pair looks odd, explicit user data wins.
        let p = try decode(
            #"{"name":"Tuned","servoMinUs":1250,"servoMaxUs":1750,"finMinDeg":-25.5,"finMaxDeg":27.0}"#)
        XCTAssertEqual(p.finMinDeg, -25.5, accuracy: 0.001)
        XCTAssertEqual(p.finMaxDeg, 27.0, accuracy: 0.001)
    }

    func testMigratedProfile_RoundTripsStable() throws {
        // Once migrated and re-saved, the angles are explicit — decoding the
        // re-encoded JSON must not shift values again (idempotence).
        let migrated = try decode(#"{"name":"OldBird","servoMinUs":1250,"servoMaxUs":1750}"#)
        let reencoded = try JSONEncoder().encode(migrated)
        let again = try JSONDecoder().decode(RocketProfile.self, from: reencoded)
        XCTAssertEqual(again.finMinDeg, migrated.finMinDeg, accuracy: 0.001)
        XCTAssertEqual(again.finMaxDeg, migrated.finMaxDeg, accuracy: 0.001)
        XCTAssertEqual(again.servoMinUs, 1250)
    }
}
