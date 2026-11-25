import XCTest
@testable import TinkerRocketApp

/// `TouchdownPredictor`: the model, and the last 20 s of three real flights
/// whose landing call never came (2026-10-24 BARC).  Android twin:
/// `TouchdownPredictorTest`.
final class TouchdownPredictorTests: XCTestCase {

    private let t0 = Date(timeIntervalSince1970: 1_000_000)

    /// The 2026-10-24 Rolly Polly V setup: main charge at 800 ft on descent.
    private let dualDeploy = RecoveryProfile(drogueMps: 18, mainMps: 6.5,
                                             mainDeployAglM: 213, pyroMainAglM: 243.84)
    /// A motor-ejection rocket whose recovery fields were left at the
    /// defaults: 700 ft main, but no pyro channel fires on altitude.
    private let singleDeploy = RecoveryProfile(drogueMps: 18.3, mainMps: 3.66,
                                               mainDeployAglM: 213)

    // MARK: - The model

    func testSingleRate_isHeightOverRate() {
        XCTAssertEqual(TouchdownPredictor.secondsToTouchdown(
            altAglM: 30, descentRateMps: 6, profile: nil), 5, accuracy: 1e-9)
    }

    func testAtOrBelowPadLevel_isNow() {
        XCTAssertEqual(TouchdownPredictor.secondsToTouchdown(
            altAglM: -12, descentRateMps: 6, profile: nil), 0)
        XCTAssertEqual(TouchdownPredictor.secondsToTouchdown(
            altAglM: 0, descentRateMps: 6, profile: nil), 0)
    }

    /// Under drogue above the main: the observed rate down to the main
    /// altitude, the profile's main rate below it.
    func testDualDeploy_underDrogue_addsTheMainLeg() {
        let s = TouchdownPredictor.secondsToTouchdown(
            altAglM: 400, descentRateMps: 16, profile: dualDeploy)
        XCTAssertEqual(s, (400 - 243.84) / 16 + 243.84 / 6.5, accuracy: 1e-9)
    }

    /// Below the main altitude but still falling at drogue speed: the main has
    /// fired and not yet opened (04 at 200 m), so the rest is at main rate.
    func testDualDeploy_belowMainAltitudeStillFast_usesTheMainRate() {
        let s = TouchdownPredictor.secondsToTouchdown(
            altAglM: 200, descentRateMps: 16, profile: dualDeploy)
        XCTAssertEqual(s, 200 / 6.5, accuracy: 1e-9)
    }

    func testDualDeploy_underMain_isHeightOverRate() {
        let s = TouchdownPredictor.secondsToTouchdown(
            altAglM: 100, descentRateMps: 6, profile: dualDeploy)
        XCTAssertEqual(s, 100 / 6.0, accuracy: 1e-9)
    }

    /// The recovery fields always hold a main altitude, so they cannot be
    /// what says a main is coming; without an altitude channel, a fast
    /// single-deploy rocket is not given one.
    func testNoAltitudeChannel_isHeightOverRateEvenWhenFast() {
        let s = TouchdownPredictor.secondsToTouchdown(
            altAglM: 400, descentRateMps: 9, profile: singleDeploy)
        XCTAssertEqual(s, 400 / 9.0, accuracy: 1e-9)
    }

    func testRate_isTheMeanOfTheLastThreeSeconds() {
        let p = TouchdownPredictor()
        p.observe(now: t0, altAglM: 100, altitudeRateMps: -20, profile: nil)          // drops out
        p.observe(now: t0 + 1, altAglM: 90, altitudeRateMps: -4, profile: nil)
        p.observe(now: t0 + 2, altAglM: 85, altitudeRateMps: -6, profile: nil)
        p.observe(now: t0 + 4, altAglM: 80, altitudeRateMps: -8, profile: nil)
        // Window at t0+4 holds the 4, 6 and 8 m/s frames: 6 m/s.
        XCTAssertEqual(p.predictedTouchdownAt!.timeIntervalSince(t0 + 4), 80 / 6.0, accuracy: 1e-6)
    }

    /// Just past apogee the rocket is not yet falling; height over a rate near
    /// zero would put the landing minutes out, so nothing is predicted.
    func testHoveringAtApogee_predictsNothing() {
        let p = TouchdownPredictor()
        p.observe(now: t0, altAglM: 300, altitudeRateMps: 0, profile: nil)
        p.observe(now: t0 + 0.5, altAglM: 300, altitudeRateMps: -1, profile: nil)
        XCTAssertNil(p.predictedTouchdownAt)
        XCTAssertNil(p.callDueAt)
    }

    func testCallDue_waitsForTheLinkToGoQuiet() {
        let p = TouchdownPredictor()
        p.observe(now: t0, altAglM: 2, altitudeRateMps: -8, profile: nil)
        // Touchdown is 0.25 s out, but the call waits for 1.5 s of silence.
        XCTAssertEqual(p.callDueAt!.timeIntervalSince(t0), 1.5, accuracy: 1e-6)

        // Higher up the touchdown itself is the later of the two.
        let high = TouchdownPredictor()
        high.observe(now: t0, altAglM: 120, altitudeRateMps: -6, profile: nil)
        XCTAssertEqual(high.callDueAt!.timeIntervalSince(t0), 20, accuracy: 1e-6)
    }

    /// `RecoveryProfile(RocketProfile)` reads the main altitude off the pyro
    /// channels.
    func testRecoveryProfile_mainAltitudeFromThePyroChannels() {
        var p = RocketProfile(name: "Rolly Polly V")
        XCTAssertNil(RecoveryProfile(p).pyroMainAglM, "no channel enabled")
        p.pyro1Enabled = true; p.pyro1TriggerMode = 0; p.pyro1TriggerValue = 1        // apogee + 1 s
        p.pyro2Enabled = true; p.pyro2TriggerMode = 1; p.pyro2TriggerValue = 243.84   // 800 ft
        p.pyro3Enabled = false; p.pyro3TriggerMode = 1; p.pyro3TriggerValue = 400     // off
        XCTAssertEqual(try XCTUnwrap(RecoveryProfile(p).pyroMainAglM), 243.84, accuracy: 1e-3)
    }

    // MARK: - Real flights

    /// Live LoRa frames from the base-station logs: seconds from the flight
    /// computer's own touchdown, pressure altitude (m), altitude rate (m/s).
    /// In all three the link died at the ground and the landed frame arrived
    /// minutes later.
    static let goblin01: [(Double, Float, Float)] = [(-19.92, 331, -23), (-19.42, 325, -14), (-18.96, 325, -14), (-17.92, 299, -15), (-17.42, 282, -35), (-16.92, 273, -22), (-16.42, 263, -29), (-15.97, 263, -29), (-15.42, 243, -22), (-14.92, 235, -22), (-14.42, 227, -15), (-13.92, 215, -25), (-13.42, 206, -22), (-12.96, 206, -22), (-12.42, 188, -13), (-11.92, 187, 4), (-11.42, 166, -29), (-10.92, 159, -14), (-10.42, 150, -15), (-9.96, 150, -15), (-8.92, 117, -15), (-8.42, 101, -42), (-7.92, 98, -7), (-7.42, 94, -7), (-6.97, 94, -7), (-6.42, 85, -11), (-5.92, 79, -9), (-5.42, 72, -15), (-4.92, 68, -9), (-4.42, 61, -13), (-3.97, 61, -13), (-3.42, 48, -14), (-2.92, 42, -13), (-2.42, 34, -14), (-1.92, 26, -19), (-1.42, 19, -14), (-0.96, 19, -14), (-0.42, 4, -18)]
    static let rollCtrl03: [(Double, Float, Float)] = [(-19.50, 162, -6), (-19.00, 157, -10), (-18.50, 152, -10), (-17.99, 148, -8), (-17.50, 142, -12), (-17.04, 142, -12), (-16.50, 133, -10), (-15.99, 129, -6), (-15.50, 124, -10), (-15.00, 119, -8), (-14.50, 115, -9), (-14.04, 115, -9), (-13.50, 105, -9), (-13.00, 100, -9), (-12.50, 96, -8), (-12.00, 92, -9), (-11.50, 89, -8), (-11.04, 89, -8), (-10.50, 80, -7), (-10.00, 76, -8), (-9.50, 72, -6), (-9.00, 67, -9), (-8.50, 63, -8), (-8.04, 63, -8), (-7.50, 55, -9), (-7.00, 52, -8), (-6.50, 47, -10), (-6.00, 44, -6), (-5.50, 39, -10), (-5.04, 39, -10), (-4.50, 32, -6), (-4.00, 28, -5), (-3.50, 24, -8), (-3.00, 20, -7), (-2.50, 16, -6), (-2.04, 16, -6), (-1.50, 9, -6), (-1.00, 6, -7), (-0.50, 2, -7)]
    /// Landed below pad level: the frames reach -12 m still descending.
    static let piercer05: [(Double, Float, Float)] = [(-19.52, 100, -7), (-19.02, 98, -4), (-18.52, 95, -7), (-18.02, 92, -6), (-17.52, 89, -6), (-17.06, 89, -6), (-16.52, 83, -5), (-16.02, 81, -4), (-15.52, 78, -6), (-15.02, 75, -5), (-14.52, 72, -5), (-14.06, 72, -5), (-13.52, 67, -4), (-13.02, 65, -5), (-12.52, 62, -6), (-12.02, 59, -5), (-11.52, 57, -6), (-11.06, 57, -6), (-10.52, 52, -4), (-10.02, 49, -5), (-9.52, 47, -4), (-9.02, 44, -5), (-8.52, 41, -6), (-8.06, 41, -6), (-7.52, 35, -4), (-7.02, 32, -7), (-6.52, 29, -5), (-6.02, 27, -4), (-5.52, 24, -6), (-4.52, 19, -5), (-4.02, 16, -7), (-3.52, 14, -3), (-3.02, 10, -7), (-2.52, 8, -4), (-2.06, 8, -4), (-1.52, 4, -6), (-1.02, 1, -6), (-0.52, -2, -5), (-0.02, -4, -4), (0.48, -7, -5), (0.94, -7, -5), (1.48, -12, -6)]

    /// When the call comes on each, relative to the real touchdown.
    private func callTime(_ frames: [(Double, Float, Float)]) -> TimeInterval? {
        let p = TouchdownPredictor()
        for (t, alt, rate) in frames {
            p.observe(now: t0 + t, altAglM: alt, altitudeRateMps: rate, profile: nil)
        }
        return p.callDueAt.map { $0.timeIntervalSince(t0) }
    }

    func testRealFlights_callWithinSecondsOfTouchdown() throws {
        XCTAssertEqual(try XCTUnwrap(callTime(Self.goblin01)), 1.1, accuracy: 0.1)
        XCTAssertEqual(try XCTUnwrap(callTime(Self.rollCtrl03)), 1.0, accuracy: 0.1)
        XCTAssertEqual(try XCTUnwrap(callTime(Self.piercer05)), 3.0, accuracy: 0.1)
    }

    /// Had the link died higher up on 03, the prediction from that frame.
    func testRealFlight_cutHigher_predictsTheTouchdown() throws {
        for cut in [100.0, 60.0, 30.0] {
            let frames = Self.rollCtrl03.prefix { Double($0.1) >= cut }
            let p = TouchdownPredictor()
            for (t, alt, rate) in frames {
                p.observe(now: t0 + t, altAglM: alt, altitudeRateMps: rate, profile: nil)
            }
            let predicted = try XCTUnwrap(p.predictedTouchdownAt).timeIntervalSince(t0)
            XCTAssertEqual(predicted, 0, accuracy: 2.0, "cut at \(cut) m")
        }
    }
}
