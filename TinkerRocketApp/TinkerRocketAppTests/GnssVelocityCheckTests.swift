import XCTest
@testable import TinkerRocketApp

/// #552: `hacc` chooses how far back to difference GNSS positions.  These pin
/// the behaviour the four 2026-08-29 BARC L2 flights were measured against,
/// and mirror `GnssVelocityCheckTest.kt` case for case.
final class GnssVelocityCheckTests: XCTestCase {

    private let mPerDegLat = 111_320.0
    private let t0 = Date(timeIntervalSince1970: 1_000_000)

    /// A rocket moving due north at `velN` m/s, sampled every `stepMs`.
    private func track(n: Int, velN: Double, hAccM: Int?,
                       stepMs: Double = 500, truthVelN: Double? = nil,
                       lat0: Double = 40.0) -> [GnssVelocityCheck.Sample] {
        let truth = truthVelN ?? velN
        return (0..<n).map { i in
            let tS = Double(i) * stepMs / 1000.0
            // Position advances at the TRUE rate; velE/velN are what the
            // filter claims.  Equal by default, so they agree.
            return GnssVelocityCheck.Sample(
                t: t0.addingTimeInterval(tS),
                latDeg: lat0 + truth * tS / mPerDegLat,
                lonDeg: -105.0, hAccM: hAccM,
                velE: 0, velN: velN)
        }
    }

    func testAFilterThatAgreesWithItsOwnFixesReadsNearZero() throws {
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 10, velN: 30, hAccM: 0)))
        XCTAssertLessThan(r.disagreementMps, 0.5, "agreement should read ~0")
    }

    func testAFilterInventingFortyMetresPerSecondIsCaughtAtThatSize() throws {
        // Position says 5 m/s, the filter says 45 — the RIM-66 shape.
        let h = track(n: 10, velN: 45, hAccM: 0, truthVelN: 5)
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: h))
        XCTAssertEqual(r.disagreementMps, 40, accuracy: 1.0)
    }

    func testGoodFixesDifferenceAtTheFrameRate() throws {
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 10, velN: 30, hAccM: 0)))
        XCTAssertEqual(r.baselineS, 0.5, accuracy: 1e-9)
    }

    func testATwentyNineMetreFixLengthensTheBaselineInsteadOfTrustingIt() throws {
        // Rolly Polly V: four satellites, 29 m accuracy.  sqrt(2)*29/3 = 13.7 s
        // would be needed, so this clamps at maxBaselineS.
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 20, velN: 30, hAccM: 29)))
        XCTAssertEqual(r.baselineS, GnssVelocityCheck.maxBaselineS, accuracy: 1e-9)
    }

    func testTheBaselineGrowsWithHaccRatherThanJumping() throws {
        let baselines = try [0, 1, 3, 6, 12, 29].map { h in
            try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 30, velN: 30, hAccM: h))).baselineS
        }
        XCTAssertEqual(baselines, baselines.sorted(), "baseline must be monotone in hacc")
        XCTAssertLessThan(baselines.first!, baselines.last!)
    }

    func testTheLongerBaselineIsWhatCutsTheNoiseFloor() throws {
        // n = 2 is the only history that FORCES the frame interval: with more
        // samples the code already walks back as far as it is allowed to.
        let short = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 2, velN: 30, hAccM: 29)))
        let long = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 20, velN: 30, hAccM: 29)))
        XCTAssertEqual(short.baselineS, 0.5, accuracy: 1e-9)
        XCTAssertGreaterThan(short.noiseFloorMps, 50, "0.5 s on a 29 m fix is ~82 m/s")
        XCTAssertLessThan(long.noiseFloorMps, 12, "4 s should bring it under 12 m/s")
    }

    func testAnAbsentHaccKeepsTheOldShortBaseline() throws {
        // Legacy firmware and the mini.  Absent must not silently become a
        // LONG baseline either — that would lag every flight without the field.
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 10, velN: 30, hAccM: nil)))
        XCTAssertEqual(r.baselineS, 0.5, accuracy: 1e-9)
    }

    func testHaccZeroAndAbsentAgreeHereAndNeitherClaimsZeroNoise() throws {
        let absent = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 10, velN: 30, hAccM: nil)))
        let zero = try XCTUnwrap(GnssVelocityCheck.evaluate(history: track(n: 10, velN: 30, hAccM: 0)))
        XCTAssertEqual(absent.baselineS, zero.baselineS, accuracy: 1e-9)
        // 5 dp of wire precision is still 1.11 m of rounding; a claim of zero
        // noise would be arithmetic, not measurement.
        XCTAssertGreaterThan(zero.noiseFloorMps, 0.5, "quantization alone is ~0.9 m/s")
    }

    func testOneSampleCannotBeDifferenced() {
        XCTAssertNil(GnssVelocityCheck.evaluate(history: track(n: 1, velN: 30, hAccM: 0)))
        XCTAssertNil(GnssVelocityCheck.evaluate(history: []))
    }

    func testHistoryOlderThanTheWindowIsDropped() throws {
        var h: [GnssVelocityCheck.Sample] = []
        for i in 0..<200 {
            h = GnssVelocityCheck.trimmed(history: h, appending: GnssVelocityCheck.Sample(
                t: t0.addingTimeInterval(Double(i) * 0.5),
                latDeg: 40, lonDeg: -105, hAccM: 0, velE: 0, velN: 0))
        }
        let spanS = h.last!.t.timeIntervalSince(h.first!.t)
        XCTAssertLessThanOrEqual(spanS, GnssVelocityCheck.historyS + 0.5)
        XCTAssertGreaterThanOrEqual(spanS, GnssVelocityCheck.maxBaselineS,
                                    "must still cover the longest baseline")
    }

    func testABackwardsClockJumpLeavesNoStaleSampleToDifferenceAgainst() {
        var h: [GnssVelocityCheck.Sample] = []
        h = GnssVelocityCheck.trimmed(history: h, appending: GnssVelocityCheck.Sample(
            t: t0.addingTimeInterval(100), latDeg: 40, lonDeg: -105, hAccM: 0, velE: 0, velN: 0))
        h = GnssVelocityCheck.trimmed(history: h, appending: GnssVelocityCheck.Sample(
            t: t0.addingTimeInterval(1), latDeg: 40, lonDeg: -105, hAccM: 0, velE: 0, velN: 0))
        XCTAssertEqual(h.count, 1, "the future-stamped sample must not survive")
        XCTAssertNil(GnssVelocityCheck.evaluate(history: h))
    }

    func testTheFilterAverageIsComparedAgainstTheChordNotOneEndpoint() throws {
        // Needs the LONG baseline to be meaningful — over one frame interval
        // the "average" is just the two endpoints.  hacc 29 forces the 4 s
        // window, which here spans the whole ramp.
        //
        // A filter whose velocity ramps 0 -> 40 while the rocket truly moves at
        // the mean of that ramp is NOT disagreeing; comparing the chord against
        // the latest instantaneous value would read 20 m/s.
        let samples = (0..<9).map { i -> GnssVelocityCheck.Sample in
            let tS = Double(i) * 0.5
            return GnssVelocityCheck.Sample(
                t: t0.addingTimeInterval(tS),
                latDeg: 40 + 20 * tS / mPerDegLat,
                lonDeg: -105, hAccM: 29,
                velE: 0, velN: Double(i) * 5)     // 0..40, mean 20
        }
        let r = try XCTUnwrap(GnssVelocityCheck.evaluate(history: samples))
        XCTAssertEqual(r.baselineS, GnssVelocityCheck.maxBaselineS, accuracy: 1e-9)
        XCTAssertLessThan(abs(r.disagreementMps), 3.0, "a ramp about the true mean is agreement")
    }

    // MARK: - The other half: the disagreement has to reach the radius
    //
    // Everything above pins the ESTIMATOR.  These pin the WIRING, because a
    // spread term that silently returned 0 would satisfy every test above.

    private func nominalLanding() -> TrackPoint? {
        simulateAscentThenDescent(
            startLat: 40, startLon: -105, currentAltAglFt: 1000,
            velocityENUMps: (e: 20, n: 0, u: 100),
            profile: RocketProfile(name: "test"), dragK: 5e-4, wind: nil).track.last
    }

    private func spread(_ disagreementMps: Double) -> Double {
        ascentVelocitySpreadMeters(
            startLat: 40, startLon: -105, currentAltAglFt: 1000,
            velocityENUMps: (e: 20, n: 0, u: 100),
            profile: RocketProfile(name: "test"), dragK: 5e-4, wind: nil,
            nominalLanding: nominalLanding(), disagreementMps: disagreementMps)
    }

    func testNoDisagreementMeansNoVelocityTerm() {
        XCTAssertEqual(spread(0), 0, accuracy: 1e-9)
    }

    func testAFortyMetrePerSecondDisagreementMovesTheLandingPointUsefully() {
        // Assert the PHYSICS rather than a number: the spread per m/s of
        // disagreement is an effective lever arm in seconds, and that lever is
        // time-to-APOGEE, not time-to-ground — the predictor integrates the
        // rocket's own velocity only until vu <= 0 and then hands over to the
        // wind drift cast.  Climbing at 100 m/s that is 7-8 s, and these
        // flights run 19-40 s to the ground, so a regression that restored the
        // to-the-ground reading (the mistake made while diagnosing #552) would
        // blow the upper bound rather than hide.
        let leverS = spread(40) / 40.0
        XCTAssertGreaterThan(leverS, 4.0, "40 m/s should be clearly visible")
        XCTAssertLessThan(leverS, 12.0, "lever must be time-to-apogee, not to ground")
    }

    func testTheVelocityTermGrowsWithTheDisagreement() {
        let spreads = [0.0, 5.0, 15.0, 40.0].map { spread($0) }
        XCTAssertEqual(spreads, spreads.sorted(), "must be monotone in the disagreement")
        XCTAssertGreaterThan(spreads.last!, spreads.first!)
    }

    func testANonsenseDisagreementIsRefusedRatherThanPropagated() {
        for bad in [Double.nan, Double.infinity, -1.0] {
            XCTAssertEqual(spread(bad), 0, accuracy: 1e-9,
                           "disagreement \(bad) must not produce a radius")
        }
    }
}
