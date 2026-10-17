import Combine
import XCTest
@testable import TinkerRocketApp

/// #1629: the six-face accelerometer calibration's math and flow, which must
/// match tools/accel_cal_six_face.py: which face a reading is, the tilt check,
/// the solve and its limits, the rule that picks a face window's result out of
/// the 0xD1 frames, and the save / clear read-backs.
final class AccelSixFaceTests: XCTestCase {

    private typealias Face = AccelSixFace.Face
    private let g = AccelSixFace.g0
    private let mg = AccelSixFace.g0 / 1000

    private func face(_ name: String) -> Face {
        Face.allCases.first { $0.name == name }!
    }

    /// A still reading on `face`: gravity `tiltDeg` off the face's axis
    /// (towards `azimuthDeg`), through the accelerometer's scale and offset.
    private func reading(_ face: Face, tiltDeg: Double, azimuthDeg: Double = 30,
                         offset: [Double] = [0, 0, 0], scale: [Double] = [1, 1, 1]) -> [Double] {
        let t = tiltDeg * .pi / 180, a = azimuthDeg * .pi / 180
        var u = [0.0, 0.0, 0.0]
        u[face.axis] = (face.up ? 1 : -1) * cos(t)
        u[(face.axis + 1) % 3] = sin(t) * cos(a)
        u[(face.axis + 2) % 3] = sin(t) * sin(a)
        return (0..<3).map { scale[$0] * g * u[$0] + offset[$0] }
    }

    // MARK: - Faces

    func testTheSixFacesAndTheirNames() {
        XCTAssertEqual(Face.allCases.map(\.name),
                       ["+X UP", "+X DOWN", "+Y UP", "+Y DOWN", "+Z UP", "+Z DOWN"])
        XCTAssertEqual(Face.allCases.map(\.howTo),
                       ["Forward end up", "Forward end down", "Left side up", "Left side down",
                        "Component side up", "Component side down"])
    }

    func testEachFaceIsIdentifiedFromItsReading() {
        for (i, f) in Face.allCases.enumerated() {
            let tilt = 1.0 + Double(i)          // 1 to 6 degrees, a different way each time
            let (got, tiltDeg) = AccelSixFace.identify(reading(f, tiltDeg: tilt, azimuthDeg: 60 * Double(i)))
            XCTAssertEqual(got, f)
            XCTAssertEqual(tiltDeg, tilt, accuracy: 1e-9, f.name)
        }
        // The axis nearest vertical wins, up when gravity reads positive on it.
        XCTAssertEqual(AccelSixFace.identify([0.3, -9.7, 1.2]).face, face("+Y DOWN"))
        XCTAssertEqual(AccelSixFace.identify([-5.0, 0.2, 8.4]).face, face("+Z UP"))
        XCTAssertEqual(AccelSixFace.identify([0, 0, 0]).tiltDeg, 90, "no reading: never square")
    }

    // MARK: - The verdict on one window

    func testATiltPast10DegreesIsRejected() {
        let zUp = face("+Z UP")
        guard case .tilted(let f, let tilt) = AccelSixFace.verdict(
            for: .face(.done, mean: reading(zUp, tiltDeg: 12))) else {
            return XCTFail("12 degrees off vertical must be measured again")
        }
        XCTAssertEqual(f, zUp)
        XCTAssertEqual(tilt, 12, accuracy: 1e-4)
        guard case .accepted(let f2, let tilt2) = AccelSixFace.verdict(
            for: .face(.done, mean: reading(zUp, tiltDeg: 9.5))) else {
            return XCTFail("9.5 degrees is square enough")
        }
        XCTAssertEqual(f2, zUp)
        XCTAssertEqual(tilt2, 9.5, accuracy: 1e-4)
    }

    func testAFailedWindowIsNotAccepted() {
        // FAILED: it moved, or it is not one g -- whatever its mean says.
        XCTAssertEqual(AccelSixFace.verdict(for: .face(.failed, mean: reading(face("+X UP"), tiltDeg: 0))),
                       .moved)
    }

    // MARK: - The solve

    /// Six faces made by the model the reference script assumes: an offset of
    /// 30 / -46 / 12 mg and scales off 1, each face up to 4 degrees off its
    /// axis, rounded to float32 as the frames carry them. The expected numbers
    /// are tools/accel_cal_six_face.py's own which_face() and solve() on the
    /// same means (run with bleak stubbed out).
    private let scriptFaces: [(String, [Float], Double)] = [
        ("+X UP", [10.0965261, 0.139545158, 0.460231513], 2.7270751546085279),
        ("+X DOWN", [-9.51267433, -1.02799952, -0.0932400227], 6.1929290023937131),
        ("+Y UP", [0.625446022, 9.3201685, 0.20639275], 4.0421518534043726),
        ("+Y DOWN", [-0.0769924819, -10.2190304, 0.331880659], 1.9094882736091854),
        ("+Z UP", [0.208453536, -0.30333063, 9.93754387], 2.1210671418074938),
        ("+Z DOWN", [0.378646016, -0.436289996, -9.7033062], 3.4070840073919526),
    ]

    func testTheSolveMatchesTheReferenceScript() throws {
        var means: [Face: [Double]] = [:]
        for (name, mean, scriptTilt) in scriptFaces {
            let m = mean.map(Double.init)
            let (f, tilt) = AccelSixFace.identify(m)
            XCTAssertEqual(f.name, name)
            XCTAssertEqual(tilt, scriptTilt, accuracy: 1e-9, name)
            means[f] = m
        }
        let s = try XCTUnwrap(AccelSixFace.solve(means))
        let offset = [0.29192590713500977, -0.44943094253540039, 0.11711883544921875]
        let scale = [0.99979098252716769, 0.99622189409366335, 1.0014046625064967]
        for i in 0..<3 {
            XCTAssertEqual(s.offset[i], offset[i], accuracy: 1e-12)
            XCTAssertEqual(s.scale[i], scale[i], accuracy: 1e-12)
        }
        let residual: [String: Double] = [
            "+X UP": 0.023777735767447794, "+X DOWN": 0.019427366001863788,
            "+Y UP": 0.0060771384303279774, "+Y DOWN": 0.0092827245471482911,
            "+Z UP": 0.0014518694989469338, "+Z DOWN": 0.00039245638408047512,
        ]
        for f in Face.allCases {
            XCTAssertEqual(try XCTUnwrap(s.residual[f]), residual[f.name]!, accuracy: 1e-12, f.name)
        }

        // The known offset, within half a milli-g, even with the faces tilted.
        let truth = [30 * mg, -46 * mg, 12 * mg]
        for i in 0..<3 { XCTAssertEqual(s.offset[i], truth[i], accuracy: 0.5 * mg) }
        XCTAssertTrue(s.canSave)
        XCTAssertFalse(s.needsResidualNote, "2.4 mg at worst")
        XCTAssertNil(s.residualNoteText)
        XCTAssertNil(s.tooLargeText)
        XCTAssertEqual(s.scaleText(0), "-0.021 %")
        XCTAssertEqual(s.scaleText(2), "+0.140 %")
        // The script's struct.pack("<3f", *off) for these faces.
        XCTAssertEqual(AccelCalApply.payload(x: s.payload[0], y: s.payload[1], z: s.payload[2]),
                       Data([0x50, 0x77, 0x95, 0x3e, 0xd0, 0x1b, 0xe6, 0xbe, 0x00, 0xdc, 0xef, 0x3d]))
    }

    func testSixSquarishFacesRecoverAKnownOffset() throws {
        let truth = [30 * mg, -46 * mg, 12 * mg]
        let scale = [1.002, 0.997, 1.0015]
        // Each axis's two faces equally off vertical, up to 4 degrees: the
        // g cos(tilt) cancels in the sum.
        let tilts = [4.0, 4.0, 3.0, 3.0, 1.5, 1.5]
        var means: [Face: [Double]] = [:]
        for (i, f) in Face.allCases.enumerated() {
            means[f] = reading(f, tiltDeg: tilts[i], azimuthDeg: 45 + 70 * Double(i),
                               offset: truth, scale: scale).map { Double(Float($0)) }
        }
        let s = try XCTUnwrap(AccelSixFace.solve(means))
        for i in 0..<3 {
            XCTAssertEqual(s.offset[i], truth[i], accuracy: 0.5 * mg)
            XCTAssertEqual(s.scale[i], scale[i], accuracy: 0.003)
        }
        XCTAssertTrue(s.canSave)
    }

    func testFiveFacesDoNotSolve() {
        var means: [Face: [Double]] = [:]
        for f in Face.allCases.dropLast() { means[f] = reading(f, tiltDeg: 0) }
        XCTAssertNil(AccelSixFace.solve(means))
    }

    func testAnOffsetPast1IsNotSavable() throws {
        // 1.2 m/s^2 on X: a solve the flight computer would refuse.
        var means: [Face: [Double]] = [:]
        for f in Face.allCases { means[f] = reading(f, tiltDeg: 0, offset: [1.2, 0.3, 0]) }
        let s = try XCTUnwrap(AccelSixFace.solve(means))
        XCTAssertEqual(s.offset[0], 1.2, accuracy: 1e-9)
        XCTAssertEqual(s.tooLargeAxes, [0])
        XCTAssertFalse(s.canSave)
        XCTAssertEqual(s.tooLargeText,
                       "Offset past 1.0 m/s² on X — the flight computer would refuse it. "
                       + "Redo the faces: was one of them not still, or the wrong way up?")
        // At the limit is still savable, as the flight computer takes it.
        for f in Face.allCases { means[f] = reading(f, tiltDeg: 0, offset: [0.999, -0.999, 0]) }
        XCTAssertTrue(try XCTUnwrap(AccelSixFace.solve(means)).canSave)
    }

    func testAFaceFarOffOneGGetsANoteButCanStillBeSaved() throws {
        // +X UP 8 degrees off, +X DOWN square: no offset and scale fit both,
        // so the corrected +X UP reads about 10 mg off one g.
        var means: [Face: [Double]] = [:]
        for f in Face.allCases { means[f] = reading(f, tiltDeg: f.name == "+X UP" ? 8 : 0) }
        let s = try XCTUnwrap(AccelSixFace.solve(means))
        XCTAssertTrue(s.needsResidualNote)
        XCTAssertTrue(s.canSave)
        XCTAssertGreaterThan(s.worstResidual, 5 * mg)
        let note = try XCTUnwrap(s.residualNoteText)
        XCTAssertTrue(note.hasPrefix("A face is 9."), note)
        XCTAssertTrue(note.hasSuffix(" mg off one g after the correction (over 5). The offset is still good if the faces were squarish; the scale is not used."), note)
    }

    func testTheReadBackToleranceIs1e5() {
        let sent: [Float] = [0.0412, -0.0273, 0.0156]
        XCTAssertTrue(AccelSixFace.readBackMatches(.stored(sent), sent: sent))
        XCTAssertTrue(AccelSixFace.readBackMatches(.stored([0.0412 + 8e-6, -0.0273, 0.0156]), sent: sent))
        XCTAssertFalse(AccelSixFace.readBackMatches(.stored([0.0412, -0.0273 - 2e-5, 0.0156]), sent: sent))
        XCTAssertFalse(AccelSixFace.readBackMatches(.stored(nil), sent: sent), "not stored")
    }

    // MARK: - The sequencing rule

    func testAStaleResultBeforeMeasuringIsIgnored() {
        var m = AccelSixFace.FaceMeasurement()
        // A late answer to an earlier 77 carries the previous window's state.
        XCTAssertNil(m.take(.face(.done, mean: reading(face("+X UP"), tiltDeg: 1))))
        XCTAssertNil(m.take(.face(.failed, mean: reading(face("+X UP"), tiltDeg: 1))),
                     "a FAILED with samples is an old window, not a refusal")
        XCTAssertNil(m.take(.face(.idle, mean: [0, 0, 0], n: 0)))
        XCTAssertFalse(m.windowOpen)
        XCTAssertNil(m.take(.face(.measuring, mean: [0, 0, 0], n: 0)))
        XCTAssertTrue(m.windowOpen)
        let result = AccelCalStatus.face(.done, mean: reading(face("+Z DOWN"), tiltDeg: 2))
        XCTAssertEqual(m.take(result), .finished(result))
    }

    func testAFailedFrameWithNoSamplesInsteadOfMeasuringIsARefusal() {
        var m = AccelSixFace.FaceMeasurement()
        XCTAssertEqual(m.take(.face(.failed, mean: [0, 0, 0], n: 0)), .refused)
    }

    func testAFailedWindowAfterMeasuringIsItsResult() {
        var m = AccelSixFace.FaceMeasurement()
        _ = m.take(.face(.measuring, mean: [0, 0, 0], n: 0))
        let moved = AccelCalStatus.face(.failed, mean: reading(face("+Y UP"), tiltDeg: 0))
        XCTAssertEqual(m.take(moved), .finished(moved))
    }

    // MARK: - The flow

    /// Measure one face through the flow, frames as the flight computer sends
    /// them (an older answer first, which must not count).
    private func measure(_ flow: inout AccelCalFlow, _ mean: [Double],
                         state: AccelCalStatus.FaceState = .done) {
        XCTAssertEqual(flow.measure(), .measureFace)
        XCTAssertEqual(flow.timeout, 10)
        XCTAssertNil(flow.take(.face(.done, mean: [0.1, 0.2, 9.8])))
        XCTAssertTrue(flow.isMeasuring, "a stale DONE before MEASURING is not this face's")
        XCTAssertNil(flow.take(.face(.measuring, mean: [0, 0, 0], n: 0)))
        XCTAssertTrue(flow.windowOpen)
        XCTAssertNil(flow.take(.face(state, mean: mean)))
        XCTAssertFalse(flow.isBusy)
    }

    private func sixFaces(offset: [Double] = [0.02, -0.03, 0.01]) -> AccelCalFlow {
        var flow = AccelCalFlow()
        for f in Face.allCases.reversed() {   // any order
            measure(&flow, reading(f, tiltDeg: 2, offset: offset))
        }
        return flow
    }

    func testTheOpeningRead() {
        var flow = AccelCalFlow()
        XCTAssertEqual(flow.read(), .read)
        XCTAssertTrue(flow.isReading)
        XCTAssertEqual(flow.timeout, 6)
        XCTAssertNil(flow.readAnswered)
        XCTAssertNil(flow.take(.stored(nil)))
        XCTAssertEqual(flow.readAnswered, true)
        XCTAssertFalse(flow.isBusy)

        // No answer: older firmware, or a lost frame.
        XCTAssertEqual(flow.read(), .read)
        XCTAssertNil(flow.timedOut(flow.generation))
        XCTAssertEqual(flow.readAnswered, false)
        XCTAssertFalse(flow.isBusy)
    }

    func testOneRequestAtATime() {
        var flow = sixFaces()
        XCTAssertEqual(flow.read(), .read)
        XCTAssertNil(flow.measure())
        XCTAssertNil(flow.save())
        XCTAssertNil(flow.clear())
        XCTAssertNil(flow.read())
    }

    func testATimerForAnEarlierWaitDoesNothing() {
        var flow = AccelCalFlow()
        _ = flow.read()
        let readTimer = flow.generation
        _ = flow.take(.stored(nil))
        XCTAssertEqual(flow.measure(), .measureFace)
        XCTAssertNil(flow.timedOut(readTimer))
        XCTAssertTrue(flow.isMeasuring, "the read's timer must not end the face")
        XCTAssertNil(flow.lastFace)
    }

    func testAFaceIsCapturedAndReported() throws {
        var flow = AccelCalFlow()
        measure(&flow, reading(face("+Z UP"), tiltDeg: 1.5))
        let report = try XCTUnwrap(flow.lastFace)
        XCTAssertTrue(report.isAccepted)
        XCTAssertEqual(report.text, "+Z UP, 1.5° off vertical")
        XCTAssertEqual(report.detail, "(+0.222, +0.128, +9.803) m/s², SD max 1.2 mg, 2880 samples")
        XCTAssertEqual(Array(flow.captures.faces.keys), [face("+Z UP")])
        XCTAssertEqual(flow.captures.missing.map(\.name), ["+X UP", "+X DOWN", "+Y UP", "+Y DOWN", "+Z DOWN"])
        XCTAssertNil(flow.captures.solution)
        XCTAssertNil(flow.save(), "nothing to save before all six")
    }

    func testMeasuringAFaceAgainReplacesItAndSaysSo() throws {
        var flow = AccelCalFlow()
        measure(&flow, reading(face("+Y DOWN"), tiltDeg: 3))
        XCTAssertEqual(flow.lastFace?.replaced, false)
        let again = reading(face("+Y DOWN"), tiltDeg: 1)
        measure(&flow, again)
        let report = try XCTUnwrap(flow.lastFace)
        XCTAssertTrue(report.replaced)
        XCTAssertEqual(report.text, "+Y DOWN, 1.0° off vertical — replaced the earlier one")
        XCTAssertEqual(flow.captures.faces.count, 1)
        XCTAssertEqual(try XCTUnwrap(flow.captures.faces[face("+Y DOWN")]).tiltDeg, 1, accuracy: 1e-4)
    }

    func testARejectedFaceIsNotCaptured() {
        var flow = AccelCalFlow()
        measure(&flow, reading(face("+X DOWN"), tiltDeg: 14))
        XCTAssertEqual(flow.lastFace?.text, "+X DOWN is 14° off vertical — set it squarer and measure again")
        measure(&flow, reading(face("+X DOWN"), tiltDeg: 0), state: .failed)
        XCTAssertEqual(flow.lastFace?.text, "It moved, or it is not one g — hold it still and measure again")
        XCTAssertTrue(flow.captures.faces.isEmpty)
    }

    func testARefusalIsReported() {
        var flow = AccelCalFlow()
        XCTAssertEqual(flow.measure(), .measureFace)
        XCTAssertNil(flow.take(.face(.failed, mean: [0, 0, 0], n: 0)))
        XCTAssertFalse(flow.isBusy)
        XCTAssertEqual(flow.lastFace?.verdict, .refused)
        XCTAssertEqual(flow.lastFace?.text, "The flight computer refused: it measures no face in flight, "
                       + "after a flight, in a sim or during a magnetometer calibration")
        XCTAssertNil(flow.lastFace?.detail)
        XCTAssertTrue(flow.captures.faces.isEmpty)
    }

    func testNoAnswerToAFace() {
        var flow = AccelCalFlow()
        XCTAssertEqual(flow.measure(), .measureFace)
        XCTAssertNil(flow.timedOut(flow.generation))
        XCTAssertEqual(flow.lastFace?.verdict, .noAnswer)
        XCTAssertEqual(flow.lastFace?.text, "No answer from the flight computer — measure again")
        XCTAssertFalse(flow.isBusy)
    }

    func testSaveThenReadBack() throws {
        var flow = sixFaces()
        let solution = try XCTUnwrap(flow.captures.solution)
        let sent = solution.payload
        for i in 0..<3 { XCTAssertEqual(Double(sent[i]), [0.02, -0.03, 0.01][i], accuracy: 1e-6) }

        XCTAssertEqual(flow.save(), .apply(sent))
        XCTAssertEqual(flow.writing, .save(sent))
        XCTAssertEqual(flow.timeout, 8)
        // The apply's own answer, then 77 for the read-back.
        XCTAssertEqual(flow.take(.stored(sent)), .read)
        XCTAssertEqual(flow.timeout, 6)
        XCTAssertNil(flow.take(.stored(sent)))
        let w = try XCTUnwrap(flow.lastWrite)
        XCTAssertTrue(w.confirmed)
        XCTAssertEqual(w.text, "Saved and read back")
        XCTAssertFalse(flow.isBusy)
    }

    func testAReadBackThatDoesNotMatch() throws {
        var flow = sixFaces()
        let sent = try XCTUnwrap(flow.captures.solution).payload
        _ = flow.save()
        _ = flow.take(.stored(sent))
        _ = flow.take(.stored([sent[0] + 1e-4, sent[1], sent[2]]))
        XCTAssertEqual(flow.lastWrite?.confirmed, false)
        XCTAssertEqual(flow.lastWrite?.text, "The read-back does not match what was sent")

        // Refused in flight: no answer to the apply, and the old offset reads back.
        _ = flow.save()
        XCTAssertEqual(flow.timedOut(flow.generation), .read, "read it back anyway")
        _ = flow.take(.stored(nil))
        XCTAssertEqual(flow.lastWrite?.confirmed, false)
    }

    func testWithNoReadBackTheApplysOwnAnswerStands() throws {
        var flow = sixFaces()
        let sent = try XCTUnwrap(flow.captures.solution).payload
        _ = flow.save()
        XCTAssertEqual(flow.take(.stored(sent)), .read)
        XCTAssertNil(flow.timedOut(flow.generation))
        XCTAssertEqual(flow.lastWrite?.readBack, .stored(sent))
        XCTAssertEqual(flow.lastWrite?.confirmed, true)
    }

    func testNoAnswerAtAllThenReadItBack() throws {
        var flow = sixFaces()
        let sent = try XCTUnwrap(flow.captures.solution).payload
        _ = flow.save()
        XCTAssertEqual(flow.timedOut(flow.generation), .read)
        XCTAssertNil(flow.timedOut(flow.generation))
        XCTAssertNil(flow.lastWrite?.readBack)
        XCTAssertEqual(flow.lastWrite?.text, "No answer from the flight computer — read it back to check")

        XCTAssertEqual(flow.readBackAgain(), .read)
        XCTAssertNil(flow.take(.stored(sent)))
        XCTAssertEqual(flow.lastWrite?.text, "Saved and read back")
        XCTAssertNil(flow.readBackAgain(), "only after a write that got no answer")
    }

    func testAnOffsetPast1IsNotSent() throws {
        var flow = sixFaces(offset: [0, 1.2, 0])
        XCTAssertFalse(try XCTUnwrap(flow.captures.solution).canSave)
        XCTAssertNil(flow.save())
        XCTAssertFalse(flow.isBusy)
    }

    func testClearThenReadBack() throws {
        var flow = AccelCalFlow()
        XCTAssertEqual(flow.clear(), .apply([0, 0, 0]))
        XCTAssertEqual(flow.writing, .clear)
        XCTAssertEqual(flow.take(.stored(nil)), .read)
        XCTAssertNil(flow.take(.stored(nil)))
        XCTAssertEqual(flow.lastWrite?.confirmed, true)
        XCTAssertEqual(flow.lastWrite?.text, "Cleared: no offset stored")

        _ = flow.clear()
        _ = flow.take(.stored([0.01, 0, 0]))
        _ = flow.take(.stored([0.01, 0, 0]))
        XCTAssertEqual(flow.lastWrite?.confirmed, false)
        XCTAssertEqual(flow.lastWrite?.text, "The flight computer still has an offset stored")
    }

    func testStartOver() {
        var flow = sixFaces()
        XCTAssertTrue(flow.captures.isComplete)
        flow.startOver()
        XCTAssertTrue(flow.captures.faces.isEmpty)
        XCTAssertNil(flow.lastFace)
    }

    // MARK: - The flow on the device's wire

    /// What the screen does: the device's 0xD1 frames into the flow, the
    /// flow's commands out through the device.
    @MainActor
    func testSixFacesAndASaveOnTheWire() throws {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var flow = AccelCalFlow()
        var writes: [Data] = []
        func run(_ c: AccelCalFlow.Command?) {
            switch c {
            case .read?: device.sendAccelCalRead()
            case .measureFace?: device.sendAccelFaceMeasure()
            case .apply(let o)?: device.sendAccelCalApply(x: o[0], y: o[1], z: o[2])
            case nil: break
            }
        }
        device.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        let sub = device.accelCalFrames.sink { run(flow.take($0)) }
        defer { sub.cancel() }

        // The script's six faces, as the flight computer would send them.
        for (_, mean, _) in scriptFaces {
            run(flow.measure())
            device.handleFileOpsFrame(AccelCalStatus.face(.measuring, mean: [0, 0, 0], n: 0).wireFrame)
            device.handleFileOpsFrame(AccelCalStatus(flags: 0, faceState: 2, offset: [0, 0, 0], faceMean: mean,
                                                     faceSD: [0.01, 0.01, 0.01], faceN: 2880).wireFrame)
        }
        XCTAssertEqual(writes, Array(repeating: Data([78]), count: 6))
        XCTAssertTrue(flow.captures.isComplete)

        writes = []
        run(flow.save())
        let script = Data([0x50, 0x77, 0x95, 0x3e, 0xd0, 0x1b, 0xe6, 0xbe, 0x00, 0xdc, 0xef, 0x3d])
        XCTAssertEqual(writes, [Data([76]) + script], "the script's struct.pack(\"<3f\", *off)")
        let sent = try XCTUnwrap(flow.writing?.sent)
        device.handleFileOpsFrame(AccelCalStatus.stored(sent).wireFrame)   // the apply's answer
        XCTAssertEqual(writes.last, Data([77]))
        device.handleFileOpsFrame(AccelCalStatus.stored(sent).wireFrame)   // the read-back
        XCTAssertEqual(flow.lastWrite?.text, "Saved and read back")
        XCTAssertEqual(device.accelCalStatus?.offsetsText, "X +29.77 mg, Y -45.83 mg, Z +11.94 mg")
    }
}
