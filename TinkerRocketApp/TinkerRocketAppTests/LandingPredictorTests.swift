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
        XCTAssertEqual(u, snapshotPositionErrorMeters + 0.2 * 5.14444 * 100,
                       accuracy: 0.1)
    }

    func testNoWindDataChargesAssumedWindBound() {
        // No wind profile → the cast applied zero drift, so the FULL
        // (unknown) drift is error: 2 m/s assumed × 100 s descent.
        let track = [
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 1000, timeS: 0),
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 0, timeS: 100),
        ]
        let u = landingUncertainty(track: track, wind: nil)
        XCTAssertEqual(u, snapshotPositionErrorMeters + 200.0, accuracy: 1e-9)
    }

    func testUncertaintyFloorsAtGnssErrorNotZero() {
        // A prediction computed essentially at the ground has no remaining
        // descent, so the wind term vanishes — but the rocket's own fix is
        // still only good to GNSS accuracy.  The circle must converge on
        // that floor rather than collapsing to a false 0 m.
        let atGround = [
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 0, timeS: 0),
            TrackPoint(lat: launchLat, lon: launchLon, altAglFt: 0, timeS: 0),
        ]
        XCTAssertEqual(landingUncertainty(track: atGround, wind: sampleWind()),
                       snapshotPositionErrorMeters, accuracy: 1e-9)
        XCTAssertEqual(landingUncertainty(track: atGround, wind: nil),
                       snapshotPositionErrorMeters, accuracy: 1e-9)
        // Degenerate/empty tracks floor too — never 0.
        XCTAssertEqual(landingUncertainty(track: [], wind: sampleWind()),
                       snapshotPositionErrorMeters, accuracy: 1e-9)
    }

    func testUncertaintyShrinksMonotonicallyTowardTheFloor() {
        // Re-predicting from progressively lower altitudes must narrow the
        // circle — the "shrinks as packets arrive" property — and never
        // dip below the floor.
        var profile = RocketProfile.makeDefault(name: "T")
        profile.drogueRateFps = 60
        let radii = [3000.0, 2000.0, 1000.0, 300.0, 50.0].map { alt in
            landingUncertainty(
                track: simulateDescentForLanding(
                    startLat: launchLat, startLon: launchLon,
                    currentAltAglFt: alt, observedVerticalRateMps: -15,
                    profile: profile, wind: sampleWind()),
                wind: sampleWind())
        }
        for (hi, lo) in zip(radii, radii.dropFirst()) {
            XCTAssertGreaterThan(hi, lo)
        }
        XCTAssertGreaterThan(radii.last!, snapshotPositionErrorMeters)
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

    // MARK: - Ascent ballistic (#191 item 1)

    func testAscentGravityOnlyMatchesClosedForm() {
        // k=0 pure ballistic: vu=50 m/s from 100 ft → apogee v²/2g above
        // the snapshot, time to apogee v/g.  dt=0.05 semi-implicit Euler
        // bias is ~½·v·dt ≈ 1.25 m — inside the 10 ft tolerance.
        let track = simulateAscentToApogee(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 100,
            velocityENUMps: (e: 0, n: 0, u: 50),
            dragK: 0)
        guard let apogee = track.last else { return XCTFail() }
        let expectedFt = 100 + mToFt(50.0 * 50.0 / (2.0 * 9.80665))
        XCTAssertEqual(apogee.altAglFt, expectedFt, accuracy: 10.0)
        XCTAssertEqual(apogee.timeS, 50.0 / 9.80665, accuracy: 0.1)
        // No horizontal velocity → no horizontal motion.
        XCTAssertEqual(apogee.lat, launchLat, accuracy: 1e-9)
        XCTAssertEqual(apogee.lon, launchLon, accuracy: 1e-9)
    }

    func testAscentDragLowersApogee() {
        // Closed form with quadratic drag: h = ln(1 + k·v²/g) / (2k)
        // ≈ 412 m vs 510 m ballistic at v=100, k=5e-4 — assert well apart.
        let vel = (e: 0.0, n: 0.0, u: 100.0)
        let ballistic = simulateAscentToApogee(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 0, velocityENUMps: vel, dragK: 0)
        let dragged = simulateAscentToApogee(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 0, velocityENUMps: vel, dragK: 5e-4)
        guard let b = ballistic.last, let d = dragged.last else { return XCTFail() }
        XCTAssertLessThan(d.altAglFt, b.altAglFt - 200.0)
    }

    func testAscentCarriesDownrange() {
        // Eastward velocity displaces the apogee east of the snapshot.
        let track = simulateAscentToApogee(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 500,
            velocityENUMps: (e: 30, n: 0, u: 60),
            dragK: 5e-4)
        guard let apogee = track.last else { return XCTFail() }
        XCTAssertGreaterThan(apogee.lon, launchLon)
        XCTAssertEqual(apogee.lat, launchLat, accuracy: 1e-6)
    }

    func testAscentThenDescentIsContinuousAndLands() {
        let cast = simulateAscentThenDescent(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 800,
            velocityENUMps: (e: 10, n: 5, u: 80),
            profile: RocketProfile.makeDefault(name: "T"),
            dragK: 5e-4, wind: sampleWind())
        guard let last = cast.track.last else { return XCTFail() }
        XCTAssertEqual(last.altAglFt, 0, accuracy: 0.5,
                       "Stitched track must end at ground level")
        for i in 1..<cast.track.count {
            XCTAssertGreaterThanOrEqual(cast.track[i].timeS,
                                        cast.track[i - 1].timeS,
                                        "Time must be continuous across the stitch")
        }
        let maxAlt = cast.track.map(\.altAglFt).max() ?? 0
        XCTAssertGreaterThan(maxAlt, 800, "Apogee must exceed the snapshot altitude")
        XCTAssertEqual(cast.descent.first?.altAglFt ?? -1, maxAlt, accuracy: 0.5,
                       "Descent segment must start at the apogee")
    }

    func testAscentDragSpreadPositiveWhenDragMatters() {
        let vel = (e: 40.0, n: 0.0, u: 100.0)
        let profile = RocketProfile.makeDefault(name: "T")
        let cast = simulateAscentThenDescent(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 500, velocityENUMps: vel,
            profile: profile, dragK: 5e-4, wind: sampleWind())
        let spread = ascentDragSpreadMeters(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 500, velocityENUMps: vel,
            profile: profile, dragK: 5e-4, wind: sampleWind(),
            nominalLanding: cast.track.last)
        XCTAssertGreaterThan(spread, 10.0,
                             "±50% drag-k on a 100 m/s coast must move the pin")

        // Gravity-only: no drag model to be wrong about → zero spread.
        let cast0 = simulateAscentThenDescent(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 500, velocityENUMps: vel,
            profile: profile, dragK: 0, wind: sampleWind())
        let spread0 = ascentDragSpreadMeters(
            startLat: launchLat, startLon: launchLon,
            currentAltAglFt: 500, velocityENUMps: vel,
            profile: profile, dragK: 0, wind: sampleWind(),
            nominalLanding: cast0.track.last)
        XCTAssertEqual(spread0, 0, accuracy: 0.001)
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
