import Combine
import XCTest
@testable import TinkerRocketApp

/// The gnss-sdr-dev board's IMU mounting: the 0xD2 status against the golden
/// the C++ emitter wrote from the real SdrAlignStatusData, command 83 byte for
/// byte, the poses matched to the six faces, and the board's part of the
/// six-face flow (BoardAlignFlow): the restart with the first face, the save
/// after the flight computer's, the faces asked for again when the flight
/// computer's offset moved, and the command answers.
final class SdrAlignStatusTests: XCTestCase {

    private typealias Face = AccelSixFace.Face
    private let statusFixture = "fileops/sdralign_0xD2_111.bin"
    private let commandFixture = "commands/cmd83_sdralign_2.bin"

    private var golden: SdrAlignStatus {
        SdrAlignStatus.decode(Array([UInt8](WireFixtures.data(statusFixture)).dropFirst()))!
    }

    // MARK: - The wire

    func testDecodesEveryFieldOfTheGolden() throws {
        let side = WireFixtures.sidecar(statusFixture)
        let frame = [UInt8](WireFixtures.data(statusFixture))
        XCTAssertEqual(frame.count, 112)
        XCTAssertEqual(frame.first, 0xD2)
        let s = try XCTUnwrap(SdrAlignStatus.decode(Array(frame.dropFirst())))
        XCTAssertEqual(Int(s.ver), side.int("ver"))
        XCTAssertEqual(Int(s.flags), side.int("flags"))
        XCTAssertEqual(Int(s.imu), side.int("imu"))
        XCTAssertEqual(Int(s.haveRaw), side.int("have"))
        XCTAssertEqual(Int(s.from), side.int("from"))
        XCTAssertEqual(Int(s.savedRaw), side.int("saved"))
        XCTAssertEqual(s.poses.count, side.int("n_pose"))
        XCTAssertEqual(Int(s.calAxes), side.int("cal_axes"))
        XCTAssertEqual(Int(s.lastRestRaw), side.int("last_rest"))
        XCTAssertEqual(Int(s.gen), side.int("gen"))
        XCTAssertEqual(Int(s.nRest), side.int("n_rest"))
        XCTAssertEqual(Int(s.nPair), side.int("n_pair"))
        XCTAssertEqual(Int(s.cmdId), side.int("cmd_id"))
        XCTAssertEqual(Int(s.cmdOp), side.int("cmd_op"))
        XCTAssertEqual(Int(s.cmdResRaw), side.int("cmd_res"))
        XCTAssertEqual(Int(s.fcCmdStateRaw), side.int("fc_cmd_state"))
        XCTAssertEqual(Int(s.fcCmdOp), side.int("fc_cmd_op"))
        XCTAssertEqual(Int(s.fcCmdId), side.int("fc_cmd_id"))
        XCTAssertEqual(Int(s.restsSent), side.int("rests_sent"))
        XCTAssertEqual(Int(s.restAgeDs), side.int("rest_age_ds"))
        XCTAssertEqual(Int(s.boardAgeDs), side.int("board_age_ds"))
        XCTAssertEqual(s.spreadDeg, side.f32("spread_deg"))
        XCTAssertEqual(s.residDeg, side.f32("resid_deg"))
        XCTAssertEqual(s.calRms, side.f32("cal_rms"))
        XCTAssertEqual(s.q, [side.f32("q0"), side.f32("q1"), side.f32("q2"), side.f32("q3")])
        XCTAssertEqual(s.scale, [side.f32("scale_x"), side.f32("scale_y"), side.f32("scale_z")])
        XCTAssertEqual(s.bias, [side.f32("bias_x"), side.f32("bias_y"), side.f32("bias_z")])
        XCTAssertEqual(s.poses[1].dir, [side.int("pose1_x"), side.int("pose1_y"), side.int("pose1_z")].map { Double($0) / 127 })
        XCTAssertEqual(s.poses[5].pairs, side.int("pose5_pairs"))
        XCTAssertEqual(s.movedDeg, side.f32("moved_deg"))

        // What the screen reads off them
        XCTAssertTrue(s.boardPresent)
        XCTAssertTrue(s.reportHeld)
        XCTAssertTrue(s.canPlaceRest)
        XCTAssertTrue(s.restsStamped)
        XCTAssertTrue(s.autosave)
        XCTAssertTrue(s.fcStill)
        XCTAssertEqual(s.have, .whole)
        XCTAssertEqual(s.saved, .same)
        XCTAssertEqual(s.lastRest, .paired)
        XCTAssertEqual(s.cmdResult, .done)
        XCTAssertEqual(s.fcCmdState, .echoed)
        XCTAssertTrue(s.isComplete)
        XCTAssertTrue(s.isGoodToSave)
        XCTAssertNil(s.poorFitText)
        XCTAssertNil(s.movedText, "from 1: this boot's rests")
        XCTAssertNil(s.blockedText)
        XCTAssertNil(s.lastRestText)
        XCTAssertEqual(s.mountingText, "Whole: 6 poses, 90° apart, the worst 0.14° off")
        XCTAssertEqual(s.savedText, "Saved on the board")
        XCTAssertEqual(s.rotationText, "Turned 90.0° from the flight computer's IMU")
        XCTAssertEqual(s.axisText(0), "scale +1.20 %, offset +15.60 mg")
        XCTAssertEqual(s.axisText(1), "scale -1.30 %, offset -27.63 mg")
        // and the test frames round-trip through the same layout
        XCTAssertEqual(s.wireFrame, WireFixtures.data(statusFixture))
    }

    func testAShortFrameIsNotDecoded() {
        let frame = [UInt8](WireFixtures.data(statusFixture))
        XCTAssertNil(SdrAlignStatus.decode(Array(frame[1..<111])))
    }

    @MainActor
    func testCommand83IsByteExactAgainstTheGolden() {
        let side = WireFixtures.sidecar(commandFixture)
        XCTAssertEqual(side.int("cmd"), 83)
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var writes: [Data] = []
        device.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        device.sendSdrAlignCommand(op: UInt8(side.int("op")), id: UInt8(side.int("id")))
        XCTAssertEqual(writes, [WireFixtures.data(commandFixture)])
    }

    @MainActor
    func testTheDevicePublishesEveryFrame() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var got: [SdrAlignStatus] = []
        let sub = device.sdrAlignFrames.sink { got.append($0) }
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        XCTAssertEqual(got.count, 2, "equal frames both arrive: the flow sequences them")
        XCTAssertEqual(device.sdrAlignStatus, golden)
        sub.cancel()
    }

    // MARK: - The faces

    func testEachPoseIsMatchedToItsFace() {
        let byFace = golden.pairsByFace
        XCTAssertEqual(byFace[Face(axis: 0, up: true)], 2)
        XCTAssertEqual(byFace[Face(axis: 0, up: false)], 3)
        XCTAssertEqual(byFace[Face(axis: 1, up: true)], 2)
        XCTAssertEqual(byFace[Face(axis: 1, up: false)], 1)
        XCTAssertEqual(byFace[Face(axis: 2, up: true)], 4)
        XCTAssertEqual(byFace[Face(axis: 2, up: false)], 255)
    }

    func testAPoseFarFromEveryFaceIsNoFaces() {
        // 30 deg off +Z: no face; 15 deg: +Z UP
        let off30 = SdrAlignStatus.Pose(dir: [sin(.pi / 6), 0, cos(.pi / 6)], pairs: 1)
        XCTAssertNil(SdrAlignStatus.face(of: off30))
        let off15 = SdrAlignStatus.Pose(dir: [sin(.pi / 12), 0, cos(.pi / 12)], pairs: 1)
        XCTAssertEqual(SdrAlignStatus.face(of: off15), Face(axis: 2, up: true))
    }

    func testWhyNoFaceCounts() {
        XCTAssertEqual(golden.with(flags: SdrAlignStatus.flagBoard).blockedText,
                       "The GNSS board is not reporting its mounting: is its firmware current?")
        XCTAssertEqual(golden.with(imu: 0).blockedText, "The GNSS board's IMU is off: its readings are not paired.")
        XCTAssertEqual(golden.with(flags: SdrAlignStatus.flagBoard | SdrAlignStatus.flagReport | SdrAlignStatus.flagLine).blockedText,
                       "The GNSS board cannot place the flight computer's readings yet: update its firmware, or give it the sky for a minute.")
        XCTAssertNil(golden.with(flags: SdrAlignStatus.flagBoard | SdrAlignStatus.flagReport | SdrAlignStatus.flagTime).blockedText,
                     "unstamped rests: the board places them at their arrival")
        XCTAssertEqual(golden.with(lastRest: .moving).lastRestText, "The board was moving: hold it still.")
    }

    func testTheRailPoseOffTheSavedMounting() {
        let onRail = golden.with(from: 3, movedDeg: 0.42)
        XCTAssertEqual(onRail.movedText, "On the saved mounting, 0.42° off it here")
        XCTAssertEqual(golden.with(from: 3, movedDeg: 2.5).movedText,
                       "The GNSS board sits 2.5° off its saved mounting: did it move since its faces were measured? Measure the six faces again.")
        XCTAssertNil(golden.with(from: 2, movedDeg: 2.5).movedText, "moved is from 3's")
    }

    func testAPoorFitIsNotSavedByItself() throws {
        XCTAssertFalse(golden.with(residDeg: 1.5).isGoodToSave)
        XCTAssertFalse(golden.with(calRms: 0.06).isGoodToSave)
        XCTAssertFalse(golden.with(poses: Array(golden.poses.prefix(5))).isGoodToSave)
        XCTAssertEqual(golden.with(residDeg: 1.5).poorFitText,
                       "Not saved by itself: 6 poses, the worst 1.50° off, the fit 0.012 m/s² rms. Measure the faces again, or save it as it is.")
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        let r = try XCTUnwrap(f.faceStarting(fresh: true, offsetInUse: [0, 0, 0]))
        let poor = golden.with(fcCmdState: .echoed, fcCmdId: r.id, fcCmdOp: r.op, residDeg: 1.5)
        _ = f.take(poor)
        XCTAssertNil(f.flightComputerSaved(offset: [0, 0, 0]), "the board's own rule's quality")
        let s = try XCTUnwrap(f.take(poor.with(residDeg: 0.4)), "measured better: saved")
        XCTAssertEqual(s.op, SdrAlignCommand.opSave)
    }

    // MARK: - The board's part of the flow

    /// The board on J1 with nothing yet, as it reports at the start.
    private var fresh: SdrAlignStatus {
        golden.with(have: .none, calAxes: 0, poses: [], fcCmdState: .idle, fcCmdId: 0, fcCmdOp: 0)
    }

    private func answer(_ c: BoardAlignFlow.Command, _ state: SdrAlignStatus.CommandState,
                        result: SdrAlignStatus.CommandResult = .done,
                        from s: SdrAlignStatus) -> SdrAlignStatus {
        s.with(fcCmdState: state, fcCmdId: c.id, fcCmdOp: c.op, cmdRes: result)
    }

    func testTheFirstFaceStartsTheBoardAfresh() throws {
        var f = BoardAlignFlow()
        XCTAssertNil(f.take(fresh))
        let c = try XCTUnwrap(f.faceStarting(fresh: true, offsetInUse: [0.01, 0, 0]))
        XCTAssertEqual(c.op, SdrAlignCommand.opRestart)
        XCTAssertEqual(c.id, 1)
        XCTAssertTrue(f.isBusy)
        XCTAssertNil(f.faceStarting(fresh: false, offsetInUse: [0, 0, 0]), "one restart for the set")
        XCTAssertNil(f.take(answer(c, .sending, from: fresh)))
        XCTAssertTrue(f.isBusy)
        XCTAssertNil(f.take(answer(c, .echoed, from: fresh)))
        XCTAssertFalse(f.isBusy)
        XCTAssertEqual(f.lastOutcome, .done(op: SdrAlignCommand.opRestart))
        XCTAssertNil(f.faceStarting(fresh: false, offsetInUse: [0, 0, 0]), "not again for the second face")
        XCTAssertEqual(f.offsetAtStart, [0.01, 0, 0])
    }

    func testNoBoardNoCommands() {
        var f = BoardAlignFlow()
        XCTAssertNil(f.faceStarting(fresh: true, offsetInUse: [0, 0, 0]), "no status: no board")
        _ = f.take(fresh.with(flags: 0))
        XCTAssertNil(f.faceStarting(fresh: true, offsetInUse: [0, 0, 0]), "a rocket without the board")
    }

    func testTheBoardSavesAfterTheFlightComputer() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        let r = try XCTUnwrap(f.faceStarting(fresh: true, offsetInUse: [0.04, -0.03, 0.01]))
        _ = f.take(answer(r, .echoed, from: fresh))
        // The faces go by; the board completes
        let complete = golden.with(saved: .differs, fcCmdState: .echoed, fcCmdId: r.id, fcCmdOp: r.op)
        XCTAssertNil(f.take(complete), "no save before the flight computer's")
        // The flight computer's offset saved, 1 mg off the one in use
        let s = try XCTUnwrap(f.flightComputerSaved(offset: [0.0410, -0.0300, 0.0095]))
        XCTAssertEqual(s.op, SdrAlignCommand.opSave)
        XCTAssertEqual(s.id, 2)
        _ = f.take(answer(s, .echoed, from: complete))
        XCTAssertEqual(f.lastOutcome, .done(op: SdrAlignCommand.opSave))
        XCTAssertEqual(f.lastOutcome?.text, "Saved on the GNSS board")
        XCTAssertNil(f.take(complete), "once for that save")
    }

    func testTheBoardSavesWhenItCompletesAfterTheFlightComputer() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        let r = try XCTUnwrap(f.faceStarting(fresh: true, offsetInUse: [0, 0, 0]))
        let five = golden.with(have: .whole, calAxes: 3, fcCmdState: .echoed, fcCmdId: r.id, fcCmdOp: r.op)
        _ = f.take(five)
        XCTAssertNil(f.flightComputerSaved(offset: [0.001, 0, 0]), "the board's z not calibrated yet")
        let s = try XCTUnwrap(f.take(five.with(calAxes: 7)), "the last pair came in: save")
        XCTAssertEqual(s.op, SdrAlignCommand.opSave)
    }

    func testAMovedOffsetAsksForTheFacesAgain() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        let r = try XCTUnwrap(f.faceStarting(fresh: true, offsetInUse: [0, 0, 0]))   // no offset in use yet
        let complete = golden.with(fcCmdState: .echoed, fcCmdId: r.id, fcCmdOp: r.op)
        _ = f.take(complete)
        // The flight computer's first calibration: 16 mg on Y
        XCTAssertNil(f.flightComputerSaved(offset: [-0.0155, -0.1569, 0.0088]))
        let again = try XCTUnwrap(f.again)
        XCTAssertEqual(again.move[1], -0.1569, accuracy: 1e-6)
        XCTAssertTrue(again.text.hasPrefix("The flight computer's calibration moved (X -1.58 mg, Y -16.00 mg, Z +0.90 mg)"))
        XCTAssertNil(f.take(complete), "nothing saved against the old offset")
        // The next face (the six already captured) starts the board afresh
        let r2 = try XCTUnwrap(f.faceStarting(fresh: false, offsetInUse: [-0.0155, -0.1569, 0.0088]))
        XCTAssertEqual(r2.op, SdrAlignCommand.opRestart)
        XCTAssertNil(f.again)
        _ = f.take(answer(r2, .echoed, from: fresh))
        // Measured again: a tiny move now, and the board's save goes
        _ = f.take(complete.with(fcCmdId: r2.id, fcCmdOp: r2.op))
        let s = try XCTUnwrap(f.flightComputerSaved(offset: [-0.0150, -0.1580, 0.0090]))
        XCTAssertEqual(s.op, SdrAlignCommand.opSave)
    }

    func testRefusedNoAnswerAndSilence() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        var c = try XCTUnwrap(f.request(SdrAlignCommand.opClear))
        _ = f.take(answer(c, .refused, from: fresh))
        XCTAssertEqual(f.lastOutcome, .refused(op: SdrAlignCommand.opClear))
        c = try XCTUnwrap(f.request(SdrAlignCommand.opSave))
        _ = f.take(answer(c, .noAnswer, from: fresh))
        XCTAssertEqual(f.lastOutcome, .noAnswer(op: SdrAlignCommand.opSave))
        XCTAssertEqual(f.lastOutcome?.text, "Save: the GNSS board did not answer")
        c = try XCTUnwrap(f.request(SdrAlignCommand.opSave))
        _ = f.take(answer(c, .echoed, result: .nothingToSave, from: fresh))
        XCTAssertEqual(f.lastOutcome, .notDone(op: SdrAlignCommand.opSave, .nothingToSave))
        c = try XCTUnwrap(f.request(SdrAlignCommand.opReport))
        let gen = f.generation
        f.timedOut(gen - 1)
        XCTAssertTrue(f.isBusy, "an older timer does nothing")
        f.timedOut(gen)
        XCTAssertEqual(f.lastOutcome, .noStatus(op: SdrAlignCommand.opReport))
        XCTAssertFalse(f.isBusy)
    }

    func testAnAnswerToAnotherIdIsNotThisOnes() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        let c = try XCTUnwrap(f.request(SdrAlignCommand.opSave))
        XCTAssertNil(f.request(SdrAlignCommand.opClear), "one at a time")
        _ = f.take(fresh.with(fcCmdState: .echoed, fcCmdId: c.id &+ 1, fcCmdOp: c.op))
        XCTAssertTrue(f.isBusy)
        _ = f.take(fresh.with(fcCmdState: .echoed, fcCmdId: c.id, fcCmdOp: SdrAlignCommand.opClear))
        XCTAssertTrue(f.isBusy, "same id, another op: an older answer")
    }

    func testIdsWrapPastZero() throws {
        var f = BoardAlignFlow()
        _ = f.take(fresh)
        var last: UInt8 = 0
        for _ in 0..<256 {
            let c = try XCTUnwrap(f.request(SdrAlignCommand.opReport))
            last = c.id
            XCTAssertNotEqual(c.id, 0)
            f.timedOut(f.generation)
        }
        XCTAssertEqual(last, 1)
    }
}

// MARK: - Test frames

extension SdrAlignStatus {

    /// A copy with some fields changed.
    func with(flags: UInt8? = nil, imu: UInt8? = nil, have: Have? = nil, from: UInt8? = nil, saved: Saved? = nil,
              calAxes: UInt8? = nil, lastRest: RestFate? = nil, poses: [Pose]? = nil,
              fcCmdState: CommandState? = nil, fcCmdId: UInt8? = nil, fcCmdOp: UInt8? = nil,
              cmdRes: CommandResult? = nil, residDeg: Float? = nil, calRms: Float? = nil,
              movedDeg: Float? = nil) -> SdrAlignStatus {
        SdrAlignStatus(ver: ver, flags: flags ?? self.flags, imu: imu ?? self.imu, haveRaw: have?.rawValue ?? haveRaw,
                       from: from ?? self.from, savedRaw: saved?.rawValue ?? savedRaw, calAxes: calAxes ?? self.calAxes,
                       lastRestRaw: lastRest?.rawValue ?? lastRestRaw, gen: gen, nRest: nRest, nPair: nPair,
                       cmdId: cmdId, cmdOp: cmdOp, cmdResRaw: cmdRes?.rawValue ?? cmdResRaw,
                       fcCmdStateRaw: fcCmdState?.rawValue ?? fcCmdStateRaw, fcCmdOp: fcCmdOp ?? self.fcCmdOp,
                       fcCmdId: fcCmdId ?? self.fcCmdId, restsSent: restsSent, restAgeDs: restAgeDs,
                       boardAgeDs: boardAgeDs, spreadDeg: spreadDeg, residDeg: residDeg ?? self.residDeg,
                       calRms: calRms ?? self.calRms, q: q, scale: scale, bias: bias, poses: poses ?? self.poses,
                       movedDeg: movedDeg ?? self.movedDeg)
    }

    /// The [0xD2][SdrAlignStatusData] frame for this status. Only drives the
    /// device in tests; the golden pins the layout (the round trip above).
    var wireFrame: Data {
        var d = Data([0xD2, ver, flags, imu, haveRaw, from, savedRaw, UInt8(poses.count), calAxes, lastRestRaw, gen,
                      nRest, nPair, cmdId, cmdOp, cmdResRaw, fcCmdStateRaw, fcCmdOp, fcCmdId, restsSent])
        withUnsafeBytes(of: restAgeDs.littleEndian) { d.append(contentsOf: $0) }
        withUnsafeBytes(of: boardAgeDs.littleEndian) { d.append(contentsOf: $0) }
        for v in [spreadDeg, residDeg, calRms] + q + scale + bias {
            withUnsafeBytes(of: v.bitPattern.littleEndian) { d.append(contentsOf: $0) }
        }
        for i in 0..<SdrAlignStatus.poses {
            for k in 0..<3 {
                let v = i < poses.count ? Int8((poses[i].dir[k] * 127).rounded()) : 0
                d.append(UInt8(bitPattern: v))
            }
        }
        for i in 0..<SdrAlignStatus.poses { d.append(i < poses.count ? UInt8(poses[i].pairs) : 0) }
        withUnsafeBytes(of: movedDeg.bitPattern.littleEndian) { d.append(contentsOf: $0) }
        return d
    }
}
