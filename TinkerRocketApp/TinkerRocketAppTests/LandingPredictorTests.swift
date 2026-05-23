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
}
