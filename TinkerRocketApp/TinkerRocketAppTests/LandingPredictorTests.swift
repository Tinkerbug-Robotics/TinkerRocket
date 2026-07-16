//
//  LandingPredictorTests.swift
//  TinkerRocketAppTests
//
//  Smoke tests for the descent-branch drift-cast wrapper used by the live
//  predictor.  The underlying algorithm (layered descent + wind drift) is
//  validated in depth via the Python harness at
//  `Data_Analysis/landing_predictor.py` against the 2026-05-17 flights;
//  these tests just guard the Swift binding against accidental breakage.
//

import XCTest
import CoreLocation
@testable import TinkerRocketApp

final class LandingPredictorTests: XCTestCase {

    private let launchLat = 39.000
    private let launchLon = -76.105

    /// A representative wind layered ENE drift (FROM 250°, ~10 kts) like the
    /// 2026-05-17 Eagle Claw flight.
    private func sampleWind() -> WindProfile {
        WindProfile(
            layers: [
                WindLayer(altFt: 0,    speedKts: 6,  directionDeg: 250),
                WindLayer(altFt: 500,  speedKts: 8,  directionDeg: 250),
                WindLayer(altFt: 2000, speedKts: 10, directionDeg: 250),
            ],
            groundElevFt: 0, fetchTime: "", location: (launchLat, launchLon)
        )
    }

    func testDescentReturnsTrackEndingAtGround() {
        var profile = RocketProfile.makeDefault(name: "T")
        profile.mainDeployAltAglFt = 700
        profile.drogueRateFps = 60
        profile.mainRateFps = 12

        let track = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500,
            observedVerticalRateMps: -20,    // ~65 fps fall (drogue)
            profile: profile,
            wind: sampleWind()
        )
        XCTAssertGreaterThan(track.count, 2)
        XCTAssertEqual(track.last?.altAglFt ?? -1, 0, accuracy: 0.5,
                       "Track must end at ground level")
    }

    func testDescentDriftsDownwindFromFromDirection() {
        // FROM 250° means wind blows TO 70° (ENE).  Predicted landing must
        // sit east of the snapshot.
        var profile = RocketProfile.makeDefault(name: "T")
        let track = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1000,
            observedVerticalRateMps: -5,     // slow main descent
            profile: profile,
            wind: sampleWind()
        )
        guard let landing = track.last else {
            return XCTFail("Empty descent track")
        }
        XCTAssertGreaterThan(landing.lon, launchLon,
                             "Landing must be east of launch under FROM-250° wind")
    }

    func testObservedFallRateOverridesProfileRate() {
        // Observed rate is much slower than profile drogue rate -> longer
        // time aloft -> more lateral drift -> further from snapshot.
        var profile = RocketProfile.makeDefault(name: "T")
        profile.drogueRateFps = 60          // profile says fast
        let trackSlow = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500,
            observedVerticalRateMps: -3,    // actually descending very slowly
            profile: profile,
            wind: sampleWind()
        )
        let trackFast = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500,
            observedVerticalRateMps: -25,   // descending fast
            profile: profile,
            wind: sampleWind()
        )
        guard let slow = trackSlow.last, let fast = trackFast.last else {
            return XCTFail()
        }
        let driftSlow = abs(slow.lon - launchLon)
        let driftFast = abs(fast.lon - launchLon)
        XCTAssertGreaterThan(driftSlow, driftFast,
                             "Slower observed descent should produce more drift")
    }

    func testNoWindMeansNoDrift() {
        var profile = RocketProfile.makeDefault(name: "T")
        let track = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500,
            observedVerticalRateMps: -15,
            profile: profile,
            wind: nil
        )
        guard let landing = track.last else { return XCTFail() }
        XCTAssertEqual(landing.lat, launchLat, accuracy: 1e-6)
        XCTAssertEqual(landing.lon, launchLon, accuracy: 1e-6)
    }

    // MARK: - Uncertainty model (#191 item 2)

    func testUncertaintyIsFractionOfMeanWindTimesDescentTime() {
        // Constant 10 kts at every altitude, 100 s descent:
        // mean = 5.14444 m/s, radius = 0.2 × 5.14444 × 100.
        let wind = WindProfile(
            layers: [WindLayer(altFt: 0, speedKts: 10, directionDeg: 250)],
            groundElevFt: 0, fetchTime: "", location: (launchLat, launchLon)
        )
        let track = [
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 1000, timeS: 0),
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 0, timeS: 100),
        ]
        let u = landingUncertainty(track: track, wind: wind)
        XCTAssertEqual(u, 0.2 * 5.14444 * 100, accuracy: 0.1)
    }

    func testNoWindDataChargesAssumedWindBound() {
        // No wind profile → the cast applied zero drift, so the FULL
        // (unknown) drift is error: 2 m/s assumed × 100 s descent.
        let track = [
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 1000, timeS: 0),
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 0, timeS: 100),
        ]
        let u = landingUncertainty(track: track, wind: nil)
        XCTAssertEqual(u, 200.0, accuracy: 1e-9)
    }

    func testUncertaintyGrowsWithTimeAloft() {
        // Same wind, slower observed descent → longer time aloft → larger
        // uncertainty, mirroring the drift behavior itself.
        var profile = RocketProfile.makeDefault(name: "T")
        profile.drogueRateFps = 60
        let slow = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500, observedVerticalRateMps: -3,
            profile: profile, wind: sampleWind())
        let fast = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500, observedVerticalRateMps: -25,
            profile: profile, wind: sampleWind())
        XCTAssertGreaterThan(landingUncertainty(track: slow, wind: sampleWind()),
                             landingUncertainty(track: fast, wind: sampleWind()))
    }

    func testUncertaintyShrinksAsAltitudeFalls() {
        // The error budget is set by how much modeled descent REMAINS: the
        // same rocket re-predicted from lower altitude must show a smaller
        // radius — and a latched prediction keeps the radius from its
        // snapshot altitude (it does not grow with staleness).
        var profile = RocketProfile.makeDefault(name: "T")
        let high = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 1500, observedVerticalRateMps: -15,
            profile: profile, wind: sampleWind())
        let low = simulateDescentForLanding(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 400, observedVerticalRateMps: -15,
            profile: profile, wind: sampleWind())
        XCTAssertGreaterThan(landingUncertainty(track: high, wind: sampleWind()),
                             landingUncertainty(track: low, wind: sampleWind()))
    }
}
