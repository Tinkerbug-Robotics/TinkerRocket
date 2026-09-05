//
//  ActiveRocketSyncerTests.swift
//  TinkerRocketAppTests
//
//  Pure decision logic for the connect-time reconcile (issues #132, #915):
//  the rocket-wins adoption rule, mag-cal push/warn/read, and the soft
//  profile suggestion.
//

import XCTest
@testable import TinkerRocketApp

final class ActiveRocketSyncerTests: XCTestCase {

    private func cal(on unitID: String) -> MagCalData {
        MagCalData(offsetX: 1, offsetY: 2, offsetZ: 3,
                   fieldR_uT: 48, residualUT: 2,
                   calibratedOnUnitID: unitID, calibratedAt: Date())
    }

    // MARK: - Adoption: the rocket wins (#915)

    /// A readback whose values all match RocketProfile's factory defaults, so
    /// a test only has to state the field it cares about.
    private func matchingConfig() -> RocketConfig {
        let p = RocketProfile.makeDefault(name: "x")
        var c = RocketConfig()
        c.servoBias1 = p.servoBias1
        c.servoHz = p.servoHz
        c.servoMinUs = p.servoMinUs
        c.servoMaxUs = p.servoMaxUs
        c.pidKp = p.pidKp; c.pidKi = p.pidKi; c.pidKd = p.pidKd
        c.pidMinCmd = p.pidMinCmd; c.pidMaxCmd = p.pidMaxCmd
        c.servoEnabled = p.servoControlEnabled
        c.gainScheduleEnabled = p.gainScheduleEnabled
        c.useAngleControl = p.useAngleControl
        c.rollDelayMs = p.rollDelayMs
        c.rollMinSpeedMps = p.rollMinSpeedMps
        c.rateCapDps = p.rateCapDps
        c.kpAngle = p.kpAngle
        c.integralSepThreshold = p.integralSepThreshold
        c.rollGainsReported = true
        c.guidanceEnabled = p.guidanceEnabled
        c.cameraType = p.cameraType
        c.imuOrientSetting = p.imuOrientSetting
        c.imuRateHz = p.imuRateHz
        c.pyro1Enabled = p.pyro1Enabled
        c.pyro1TriggerMode = p.pyro1TriggerMode
        c.pyro1TriggerValue = p.pyro1TriggerValue
        c.pyro2Enabled = p.pyro2Enabled
        c.pyro2TriggerMode = p.pyro2TriggerMode
        c.pyro2TriggerValue = p.pyro2TriggerValue
        c.pyro3Enabled = p.pyro3Enabled
        c.pyro3TriggerMode = p.pyro3TriggerMode
        c.pyro3TriggerValue = p.pyro3TriggerValue
        c.pyro4Enabled = p.pyro4Enabled
        c.pyro4TriggerMode = p.pyro4TriggerMode
        c.pyro4TriggerValue = p.pyro4TriggerValue
        return c
    }

    func testAgreementReportsNothingChanged() {
        var p = RocketProfile.makeDefault(name: "Rolly Polly")
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: matchingConfig()), [])
    }

    func testRocketValueOverwritesTheProfile() {
        var p = RocketProfile.makeDefault(name: "Rolly Polly")
        p.pidKp = 0.30
        var cfg = matchingConfig()
        cfg.pidKp = 0.12
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertEqual(changed, [ActiveRocketSyncer.groupPidGains])
        XCTAssertEqual(p.pidKp, 0.12, "the rocket's gain wins, not the phone's")
    }

    /// The whole point of #915: the rocket the phone connected to must come
    /// away flying what it already had, and the profile must say so.
    func testAnotherAirframesOrientationDoesNotSurvive() {
        var p = RocketProfile.makeDefault(name: "Wrong airframe")
        p.imuOrientSetting = 7            // manual, from a different rocket
        var cfg = matchingConfig()
        cfg.imuOrientSetting = 0xFF       // this rocket is on pad auto-detect
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertEqual(changed, [ActiveRocketSyncer.groupImuOrientation])
        XCTAssertEqual(p.imuOrientSetting, 0xFF)
    }

    /// Wire rounding must not manufacture a diff on every single connect:
    /// kp crosses at 4 decimals, kpAngle at 2, pyro values at 1.
    func testWireRoundingIsNotADifference() {
        var p = RocketProfile.makeDefault(name: "x")
        p.pidKp = 0.120004            // rounds to 0.1200 on the wire
        p.kpAngle = 2.001             // rounds to 2.00
        p.pyro2TriggerValue = 100.04  // rounds to 100.0
        var cfg = matchingConfig()
        cfg.pidKp = 0.12
        cfg.kpAngle = 2.0
        cfg.pyro2TriggerValue = 100.0
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), [])
    }

    /// #253 sentinels mean "the firmware is on its own default", and
    /// RocketConfig then holds the APP's defaults — adopting those would
    /// overwrite a deliberately-tuned profile with a number nobody chose.
    func testUnreportedRollGainsAreLeftAlone() {
        var p = RocketProfile.makeDefault(name: "x")
        p.rateCapDps = 120
        p.kpAngle = 5
        var cfg = matchingConfig()
        cfg.rollGainsReported = false     // rocket sent the sentinels
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), [])
        XCTAssertEqual(p.rateCapDps, 120)
        XCTAssertEqual(p.kpAngle, 5)
    }

    /// Firmware too old to report orientation / IMU rate leaves those nil;
    /// the profile keeps its own rather than being reset to an invention.
    func testFieldsThisFirmwareNeverReportsAreKept() {
        var p = RocketProfile.makeDefault(name: "x")
        p.imuOrientSetting = 7
        p.imuRateHz = 1920
        var cfg = matchingConfig()
        cfg.imuOrientSetting = nil
        cfg.imuRateHz = nil
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), [])
        XCTAssertEqual(p.imuOrientSetting, 7)
        XCTAssertEqual(p.imuRateHz, 1920)
    }

    /// Groups the readback never covers must survive untouched — adoption
    /// may not silently reset what it cannot see.
    func testUnreportedGroupsSurviveAdoption() {
        var p = RocketProfile.makeDefault(name: "x")
        p.servoBias2 = 40
        p.finTravelDeg = 90
        p.finRingMode = 1
        p.soundsEnabled = true
        p.pnNavGain = 9
        p.rollWaypoints = [RollWaypoint(timeSeconds: 1, angleDeg: 90)]
        _ = ActiveRocketSyncer.adopt(&p, from: matchingConfig())
        XCTAssertEqual(p.servoBias2, 40)
        XCTAssertEqual(p.finTravelDeg, 90)
        XCTAssertEqual(p.finRingMode, 1)
        XCTAssertTrue(p.soundsEnabled)
        XCTAssertEqual(p.pnNavGain, 9)
        XCTAssertEqual(p.rollWaypoints.count, 1)
    }

    func testEachPyroChannelIsNamedSeparately() {
        var p = RocketProfile.makeDefault(name: "x")
        var cfg = matchingConfig()
        cfg.pyro1Enabled = !p.pyro1Enabled
        cfg.pyro3TriggerValue = p.pyro3TriggerValue + 50
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), ["Pyro 1", "Pyro 3"])
        XCTAssertEqual(p.pyro1Enabled, cfg.pyro1Enabled)
        XCTAssertEqual(p.pyro3TriggerValue, cfg.pyro3TriggerValue)
    }

    func testAdoptionIsIdempotent() {
        var p = RocketProfile.makeDefault(name: "x")
        p.cameraType = 0
        var cfg = matchingConfig()
        cfg.cameraType = 1
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg),
                       [ActiveRocketSyncer.groupCamera])
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), [],
                       "a second connect with the same rocket reports nothing")
    }

    // MARK: - The firmware config report (#915 follow-up)

    private func servoExtras() -> RocketServoExtras {
        RocketServoExtras(bias2: 0, bias3: 0, bias4: 0,
                          finMinDeg: -60, finMaxDeg: 60,
                          finAzimuths: [0, 90, 180, 270],
                          finReverseMask: 0, finRollReverseMask: 0,
                          soundsEnabled: false)
    }

    private func guidanceExtras() -> RocketGuidanceExtras {
        let p = RocketProfile.makeDefault(name: "x")
        return RocketGuidanceExtras(
            navGain: p.pnNavGain, maxAccel: p.pnMaxAccel,
            accelToFin: p.pnAccelToFin, maxFinDeg: p.pnMaxFinDeg,
            minSpeed: p.pnMinSpeed, coastDelayMs: p.pnCoastDelayMs,
            targetMode: p.pnTargetMode, targetE: p.pnTargetE, targetN: p.pnTargetN,
            targetAltM: p.pnTargetAltM, kpPos: p.pnKpPos, kdVel: p.pnKdVel,
            guidanceLaw: p.pnGuidanceLaw)
    }

    /// A rocket that reports everything and agrees about all of it.
    private func fullyReportingConfig() -> RocketConfig {
        var c = matchingConfig()
        c.servoExtras = servoExtras()
        c.guidanceExtras = guidanceExtras()
        c.rollWaypoints = []
        return c
    }

    func testFullReportWithNoDisagreementChangesNothing() {
        var p = RocketProfile.makeDefault(name: "x")
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: fullyReportingConfig()), [])
    }

    /// The group that motivated the firmware work: a fin layout from another
    /// airframe used to be invisible AND unverifiable.
    func testFinLayoutIsAdoptedFromTheRocket() {
        var p = RocketProfile.makeDefault(name: "x")
        p.finServoAtSlot = [4, 3, 2, 1]
        p.finRollReverse = [true, false, false, false]
        var cfg = fullyReportingConfig()
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertEqual(changed, [ActiveRocketSyncer.groupFinLayout])
        XCTAssertEqual(p.finServoAtSlot, [1, 2, 3, 4])
        XCTAssertEqual(p.finRollReverse, [false, false, false, false])
        XCTAssertEqual(p.finRingMode, 0)
        cfg.servoExtras?.finAzimuths = [45, 135, 225, 315]
        _ = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertEqual(p.finRingMode, 1, "the 45°-rotated ring reads as ×")
    }

    func testServoTrimAndFinTravelAndSoundsAreAdopted() {
        var p = RocketProfile.makeDefault(name: "x")
        p.servoBias3 = 55
        p.finTravelDeg = 90
        p.soundsEnabled = true
        var cfg = fullyReportingConfig()
        cfg.servoExtras?.soundsEnabled = false
        let changed = ActiveRocketSyncer.adopt(&p, from: cfg)
        XCTAssertEqual(changed, [ActiveRocketSyncer.groupServoTrim24,
                                 ActiveRocketSyncer.groupFinTravel,
                                 ActiveRocketSyncer.groupSounds])
        XCTAssertEqual(p.servoBias3, 0)
        XCTAssertEqual(p.finTravelDeg, 120, "travel is the reported span")
        XCTAssertFalse(p.soundsEnabled)
    }

    func testGuidanceParametersAreAdopted() {
        var p = RocketProfile.makeDefault(name: "x")
        p.pnNavGain = 9
        p.pnGuidanceLaw = 1
        let changed = ActiveRocketSyncer.adopt(&p, from: fullyReportingConfig())
        XCTAssertEqual(changed, [ActiveRocketSyncer.groupGuidanceParams])
        XCTAssertEqual(p.pnNavGain, 5.0)
        XCTAssertEqual(p.pnGuidanceLaw, 0)
    }

    /// "No waypoints" and "we can't see the waypoints" must not be the same
    /// thing: one clears the profile's roll profile, the other must not.
    func testEmptyWaypointListIsAnAnswerButNilIsNot() {
        var reported = RocketProfile.makeDefault(name: "x")
        reported.rollWaypoints = [RollWaypoint(timeSeconds: 1, angleDeg: 90)]
        var cfg = fullyReportingConfig()
        cfg.rollWaypoints = []
        XCTAssertEqual(ActiveRocketSyncer.adopt(&reported, from: cfg),
                       [ActiveRocketSyncer.groupRollProfile])
        XCTAssertTrue(reported.rollWaypoints.isEmpty, "the rocket flies rate-only")

        var unreported = RocketProfile.makeDefault(name: "x")
        unreported.rollWaypoints = [RollWaypoint(timeSeconds: 1, angleDeg: 90)]
        cfg.rollWaypoints = nil
        XCTAssertEqual(ActiveRocketSyncer.adopt(&unreported, from: cfg), [])
        XCTAssertEqual(unreported.rollWaypoints.count, 1,
                       "a rocket that can't report waypoints must not erase them")
    }

    func testWaypointsAreAdoptedFromTheRocket() {
        var p = RocketProfile.makeDefault(name: "x")
        var cfg = fullyReportingConfig()
        cfg.rollWaypoints = [ReportedRollWaypoint(timeSeconds: 2.0, angleDeg: 45.0),
                             ReportedRollWaypoint(timeSeconds: 5.0, angleDeg: 180.0)]
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg),
                       [ActiveRocketSyncer.groupRollProfile])
        XCTAssertEqual(p.rollWaypoints.map(\.timeSeconds), [2.0, 5.0])
        XCTAssertEqual(p.rollWaypoints.map(\.angleDeg), [45.0, 180.0])
    }

    /// Pre-report firmware, and the mini (no servo/fin/guidance hardware):
    /// the profile keeps its own and the app says it cannot check.
    func testAGroupTheRocketCannotReportIsNamedAndLeftAlone() {
        var p = RocketProfile.makeDefault(name: "x")
        p.servoBias2 = 40
        p.pnNavGain = 9
        let cfg = matchingConfig()   // no extras at all
        XCTAssertEqual(ActiveRocketSyncer.adopt(&p, from: cfg), [])
        XCTAssertEqual(p.servoBias2, 40)
        XCTAssertEqual(p.pnNavGain, 9)
        XCTAssertEqual(cfg.unreportedGroups,
                       ["Servo trim 2-4", "Fin travel", "Fin layout", "Sounds",
                        "Guidance parameters", "Roll profile"])
        XCTAssertEqual(fullyReportingConfig().unreportedGroups, [],
                       "a reporting rocket leaves nothing unverifiable")
    }

    func testAzimuthsThatDoNotDescribeFourSlotsAreRefused() {
        // Two servos claiming the same slot can't be inverted into a mapping;
        // guessing one would put the wrong servo on the wrong fin.
        XCTAssertNil(ActiveRocketSyncer.slotsFromAzimuths([0, 0, 180, 270]))
        XCTAssertNil(ActiveRocketSyncer.slotsFromAzimuths([0, 90, 180]))
        // Servo 1 sits at 270° (slot 3), servo 2 at 0° (slot 0), and so on —
        // so slot 0 holds servo 2, slot 1 servo 3, slot 2 servo 4, slot 3 servo 1.
        XCTAssertEqual(ActiveRocketSyncer.slotsFromAzimuths([270, 0, 90, 180]),
                       [2, 3, 4, 1])
    }

    // MARK: - Mag cal action

    func testNoProfileCalAsksRocket() {
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: nil,
                                                         deviceUnitID: "BOARD1")
        XCTAssertEqual(action, .readRocket)
    }

    func testMatchingBoardPushesCal() {
        let c = cal(on: "BOARD1")
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: c,
                                                         deviceUnitID: "BOARD1")
        XCTAssertEqual(action, .push(c))
    }

    func testDifferentBoardWarns() {
        let c = cal(on: "BOARD1")
        let action = ActiveRocketSyncer.magCalSyncAction(profileCal: c,
                                                         deviceUnitID: "BOARD2")
        XCTAssertEqual(action, .warnMismatch(savedOn: "BOARD1", current: "BOARD2"))
    }

    // MARK: - Sensor cal action

    private func sensorCal(on unitID: String) -> SensorCalData {
        SensorCalData(gyroX: 1, gyroY: 2, gyroZ: 3, hgX: 0.1, hgY: 0.2, hgZ: 0.3,
                      calibratedOnUnitID: unitID, calibratedAt: Date())
    }

    func testNoProfileSensorCalAsksRocket() {
        XCTAssertEqual(ActiveRocketSyncer.sensorCalSyncAction(profileCal: nil,
                                                              deviceUnitID: "B1"), .readRocket)
    }

    func testMatchingBoardPushesSensorCal() {
        let c = sensorCal(on: "B1")
        XCTAssertEqual(ActiveRocketSyncer.sensorCalSyncAction(profileCal: c,
                                                              deviceUnitID: "B1"), .push(c))
    }

    func testDifferentBoardWarnsSensorCal() {
        let c = sensorCal(on: "B1")
        XCTAssertEqual(ActiveRocketSyncer.sensorCalSyncAction(profileCal: c, deviceUnitID: "B2"),
                       .warnMismatch(savedOn: "B1", current: "B2"))
    }

    func testSensorCalDataFromStatusTagsBoard() {
        let status = SensorCalStatus(valid: true, gyroX: -3, gyroY: 4, gyroZ: -5,
                                     hgX: 0.11, hgY: -0.22, hgZ: 9.7)
        let data = SensorCalData(status: status, unitID: "B9")
        XCTAssertEqual(data.gyroX, -3)
        XCTAssertEqual(data.gyroZ, -5)
        XCTAssertEqual(data.hgZ, 9.7, accuracy: 1e-4)
        XCTAssertEqual(data.calibratedOnUnitID, "B9")
    }

    // MARK: - Suggestion

    func testSuggestionMatchesLastUsedBoard() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        var b = RocketProfile.makeDefault(name: "B"); b.lastUsedUnitID = "BOARD2"
        let s = ActiveRocketSyncer.suggestedProfile(in: [a, b], active: nil, unitID: "BOARD2")
        XCTAssertEqual(s, b.id)
    }

    func testNoSuggestionWhenActiveAlreadyMatches() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        // Active profile already the one flown on this board → nothing to suggest.
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: a.id, unitID: "BOARD1")
        XCTAssertNil(s)
    }

    func testNoSuggestionForUnknownBoard() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = "BOARD1"
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: nil, unitID: "NEVER_SEEN")
        XCTAssertNil(s)
    }

    func testNoSuggestionForEmptyUnitID() {
        var a = RocketProfile.makeDefault(name: "A"); a.lastUsedUnitID = ""
        let s = ActiveRocketSyncer.suggestedProfile(in: [a], active: nil, unitID: "")
        XCTAssertNil(s)
    }

    // MARK: - MagCalData snapshot from a status frame

    func testMagCalDataFromStatusTagsBoard() {
        let status = MagCalStatus(
            subType: .applied, coverageBins: 26, sampleCount: 1000,
            instantaneousFieldUT: 48, offsetX: -4, offsetY: 5, offsetZ: -6,
            fieldR_uT: 49.5, residualUT: 1.2, rejectCode: .ok, coverageMask: 0,
            liveX_uT: 0, liveY_uT: 0, liveZ_uT: 0, partialMask: 0)
        let data = MagCalData(status: status, unitID: "BOARD9")
        XCTAssertEqual(data.offsetX, -4)
        XCTAssertEqual(data.offsetY, 5)
        XCTAssertEqual(data.offsetZ, -6)
        XCTAssertEqual(data.fieldR_uT, 49.5, accuracy: 1e-4)
        XCTAssertEqual(data.residualUT, 1.2, accuracy: 1e-4)
        XCTAssertEqual(data.calibratedOnUnitID, "BOARD9")
    }
}
