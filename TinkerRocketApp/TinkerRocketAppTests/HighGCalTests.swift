import Combine
import SwiftUI
import XCTest
@testable import TinkerRocketApp

/// The high-g accelerometer's six-face alignment to the low-g
/// (RocketComputerTypes.h, HG_CAL_*): the 0xD1 frame's high-g part and
/// command 80 against the goldens, the fit (the cases
/// tests/unit/test_accel_cal_six_face.py pins on the reference script), the
/// save and clear through the flow, and the rocket profile's bias after a save.
final class HighGCalTests: XCTestCase {

    private typealias Face = AccelSixFace.Face
    private let g = AccelSixFace.g0
    private let mg = AccelSixFace.g0 / 1000

    // MARK: - The wire

    func testDecodesTheHighGGolden() throws {
        let rel = "fileops/accelcal_0xD1_114.bin"
        let side = WireFixtures.sidecar(rel)
        let frame = [UInt8](WireFixtures.data(rel))
        XCTAssertEqual(frame.count, 115)
        let s = try XCTUnwrap(AccelCalStatus.decode(Array(frame.dropFirst())))
        XCTAssertEqual(Int(s.faceN), side.int("face_n"))
        let h = try XCTUnwrap(s.highG)
        XCTAssertEqual(h.faceMean, [side.f32("hg_face_mean_x"), side.f32("hg_face_mean_y"), side.f32("hg_face_mean_z")])
        XCTAssertEqual(h.faceSD, [side.f32("hg_face_sd_x"), side.f32("hg_face_sd_y"), side.f32("hg_face_sd_z")])
        XCTAssertEqual(Int(h.flags), side.int("hg_flags"))
        XCTAssertEqual(Int(h.biasSource), side.int("hg_bias_src"))
        let names = ["m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"]
        XCTAssertEqual(h.matrix, names.map { side.f32($0) })
        XCTAssertEqual(h.bias, [side.f32("hg_bias_x"), side.f32("hg_bias_y"), side.f32("hg_bias_z")])
        XCTAssertTrue(h.isStored)
        XCTAssertTrue(h.isApplied)
        XCTAssertTrue(h.hasBias)
        XCTAssertEqual(h.source, .sixFaces)
        XCTAssertEqual(h.alignmentText, "High-g alignment in use: X +0.810 %, Y -0.660 %, Z +1.220 %")
        XCTAssertEqual(h.biasText, "X -66.4 mg, Y +150.9 mg, Z -31.9 mg")
        XCTAssertEqual(h.sourceText, "from the six faces")
        // and the test frames round-trip through the same layout
        XCTAssertEqual(s.wireFrame, WireFixtures.data(rel))
    }

    func testA40ByteFrameCarriesNoHighG() throws {
        let frame = [UInt8](WireFixtures.data("fileops/accelcal_0xD1_40.bin"))
        let s = try XCTUnwrap(AccelCalStatus.decode(Array(frame.dropFirst())))
        XCTAssertNil(s.highG, "an older flight or out computer: no high-g")
        // a frame cut anywhere inside the high-g part is still read as the low-g alone
        let long = [UInt8](WireFixtures.data("fileops/accelcal_0xD1_114.bin"))
        XCTAssertNil(AccelCalStatus.decode(Array(long[1..<100]))?.highG)
        XCTAssertNotNil(AccelCalStatus.decode(Array(long[1..<100])))
    }

    @MainActor
    func testCommand80IsByteExactAgainstTheGolden() {
        let rel = "commands/cmd80_hgcal_49.bin"
        let side = WireFixtures.sidecar(rel)
        XCTAssertEqual(side.int("cmd"), 80)
        let names = ["m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"]
        let m = names.map { side.f32($0) }
        let b = [side.f32("bias_x"), side.f32("bias_y"), side.f32("bias_z")]
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var writes: [Data] = []
        device.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        device.sendHighGCalApply(op: HighGCalApply.opStore, matrix: m, bias: b)
        XCTAssertEqual(writes, [WireFixtures.data(rel)])
        // a clear: op 0, the identity and zeros
        writes = []
        device.sendHighGCalApply(op: HighGCalApply.opClear)
        XCTAssertEqual(writes.first?.count, 50)
        XCTAssertEqual(writes.first?[1], 0)
    }

    // MARK: - Synthetic boards

    // A board's two accelerometers as real ones differ: reading = S a + offset,
    // the high-g with a full 3x3 S. The fit must find l = M h - b with
    // M = S_lg S_hg^-1 and b = M c_hg - o_lg.
    private let sLG: [Double] = [1.0006, 0.9991, 1.0004]
    private let oLG: [Double] = [-0.0155, -0.1324, -0.0306]
    private let sHG: [Double] = [0.9921, -0.0047, 0.0030, 0.0029, 1.0068, -0.0054, -0.0016, 0.0051, 0.9880]
    private let cHG: [Double] = [0.98, -1.47, 0.62]
    /// One 3 s window at 3780 Hz on the V9 (the 2026-10-17 bench log): the
    /// low-g's and the high-g's sample SD, m/s² (1.9 and 57 mg).
    private let sdLG = 0.019, sdHG = 0.56
    private let nSamples = 11340
    /// The user's first run on the Beetle: each face's tilt off its axis, deg.
    private let userTilts: [String: Double] = ["+X UP": 9.1, "+X DOWN": 5.0, "+Y UP": 6.9, "+Y DOWN": 4.5,
                                               "+Z UP": 3.7, "+Z DOWN": 9.0]

    private func mul(_ m: [Double], _ v: [Double]) -> [Double] {
        (0..<3).map { i -> Double in
            let a: Double = m[3 * i] * v[0]
            let b: Double = m[3 * i + 1] * v[1]
            let c: Double = m[3 * i + 2] * v[2]
            return a + b + c
        }
    }

    private func inv3(_ a: [Double]) -> [Double] {
        let c00: Double = a[4] * a[8] - a[5] * a[7]
        let c01: Double = a[5] * a[6] - a[3] * a[8]
        let c02: Double = a[3] * a[7] - a[4] * a[6]
        let det: Double = a[0] * c00 + a[1] * c01 + a[2] * c02
        let r0: [Double] = [c00, a[2] * a[7] - a[1] * a[8], a[1] * a[5] - a[2] * a[4]]
        let r1: [Double] = [c01, a[0] * a[8] - a[2] * a[6], a[2] * a[3] - a[0] * a[5]]
        let r2: [Double] = [c02, a[1] * a[6] - a[0] * a[7], a[0] * a[4] - a[1] * a[3]]
        return (r0 + r1 + r2).map { $0 / det }
    }

    /// M and b the fit should find on this board.
    private func truth(sHG: [Double]? = nil, cHG: [Double]? = nil) -> (m: [Double], b: [Double]) {
        let si = inv3(sHG ?? self.sHG)
        var m = [Double](repeating: 0, count: 9)
        for i in 0..<3 {
            for j in 0..<3 { m[3 * i + j] = sLG[i] * si[3 * i + j] }
        }
        let mc = mul(m, cHG ?? self.cHG)
        return (m, (0..<3).map { mc[$0] - oLG[$0] })
    }

    private struct Gauss {
        var rng: SplitMix
        mutating func next(_ sd: Double) -> Double {
            let u1 = max(Double.random(in: 0..<1, using: &rng), 1e-300)
            let u2 = Double.random(in: 0..<1, using: &rng)
            return sd * (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
        }
    }

    /// Six faces' captures: gravity tilted off each face's axis (towards an
    /// azimuth that turns face by face), each channel's mean of it with
    /// optional Gaussian noise of SD / sqrt(n), and the high-g's even-order
    /// nonlinearity quad_i * a_i^2.
    private func board(tilts: [String: Double], noiseSeed: UInt64? = nil, sHG: [Double]? = nil, cHG: [Double]? = nil,
                       quad: [Double] = [0, 0, 0], withHighG: Bool = true) -> [Face: AccelSixFace.Capture] {
        var noise = noiseSeed.map { Gauss(rng: SplitMix(state: $0)) }
        var out: [Face: AccelSixFace.Capture] = [:]
        for (k, face) in Face.allCases.enumerated() {
            let t = (tilts[face.name] ?? 0) * .pi / 180
            let az = Double(30 + 50 * k) * .pi / 180
            var a = [0.0, 0.0, 0.0]
            a[face.axis] = (face.up ? 1 : -1) * g * cos(t)
            a[(face.axis + 1) % 3] = g * sin(t) * cos(az)
            a[(face.axis + 2) % 3] = g * sin(t) * sin(az)
            var l: [Double] = (0..<3).map { sLG[$0] * a[$0] + oLG[$0] }
            let sa = mul(sHG ?? self.sHG, a)
            var h: [Double] = (0..<3).map { i -> Double in
                let q: Double = quad[i] * a[i] * a[i]
                return sa[i] + (cHG ?? self.cHG)[i] + q
            }
            if noise != nil {
                let n = Double(nSamples).squareRoot()
                l = l.map { $0 + noise!.next(sdLG / n) }
                h = h.map { $0 + noise!.next(sdHG / n) }
            }
            let hg = withHighG ? AccelCalStatus.HighG(faceMean: h.map(Float.init), faceSD: [Float](repeating: Float(sdHG), count: 3),
                                                      flags: 0, biasSource: 0, matrix: HighGCalApply.identity,
                                                      bias: [0, 0, 0]) : nil
            let frame = AccelCalStatus(flags: 0, faceState: AccelCalStatus.FaceState.done.rawValue, offset: [0, 0, 0],
                                       faceMean: l.map(Float.init), faceSD: [Float](repeating: Float(sdLG), count: 3),
                                       faceN: UInt16(nSamples), highG: hg)
            out[face] = AccelSixFace.Capture(frame, tiltDeg: tilts[face.name] ?? 0)
        }
        return out
    }

    private func fit(_ faces: [Face: AccelSixFace.Capture]) throws -> AccelSixFace.HighGSolution {
        let lowG = try XCTUnwrap(AccelSixFace.solve(faces.mapValues(\.mean)))
        XCTAssertTrue(lowG.converged)
        guard case .solved(let s) = AccelSixFace.solveHighG(faces, lowG: lowG) else {
            XCTFail("no high-g solution")
            throw XCTSkip("unsolved")
        }
        return s
    }

    // MARK: - The fit

    func testTheAlignmentRecoversScaleCrossAxisAndOffsetWhateverTheTilts() throws {
        let want = truth()
        for tilts in [userTilts, [:]] {
            let s = try fit(board(tilts: tilts))
            XCTAssertEqual(s.problems, [])
            XCTAssertEqual(s.disagreeAxes, [])
            // the means went through Float32 on the wire: a few parts in 1e7
            for k in 0..<9 { XCTAssertEqual(s.matrix[k], want.m[k], accuracy: 2e-6) }
            for i in 0..<3 { XCTAssertEqual(s.bias[i], want.b[i], accuracy: 0.01 * mg) }
            XCTAssertLessThan(s.residualMax, 0.01 * mg)
            for i in 0..<3 {
                XCTAssertEqual(s.ownOffset[i], cHG[i], accuracy: 0.2 * mg, "the sphere fit has no cross-axis terms")
                XCTAssertEqual(s.impliedZero[i], cHG[i], accuracy: 0.01 * mg)
            }
            XCTAssertTrue(s.canSave)
        }
    }

    func testWithBenchNoiseTheErrorsMatchTheReportedSigma() throws {
        let want = truth()
        var zM: [Double] = []
        var zB: [Double] = []
        var falseAlarms = 0
        let trials = 200
        var last: AccelSixFace.HighGSolution?
        for trial in 0..<trials {
            let s = try fit(board(tilts: userTilts, noiseSeed: 1629 &+ UInt64(trial) &* 1_000_003))
            XCTAssertEqual(s.problems, [])
            for k in 0..<9 { zM.append((s.matrix[k] - want.m[k]) / s.sigmaMatrix[k]) }
            for i in 0..<3 { zB.append((s.bias[i] - want.b[i]) / s.sigmaBias[i]) }
            if !s.disagreeAxes.isEmpty { falseAlarms += 1 }
            last = s
        }
        func rms(_ z: [Double]) -> Double { (z.map { $0 * $0 }.reduce(0, +) / Double(z.count)).squareRoot() }
        XCTAssertGreaterThan(rms(zM), 0.3)
        XCTAssertLessThan(rms(zM), 1.1)
        XCTAssertGreaterThan(rms(zB), 0.3)
        XCTAssertLessThan(rms(zB), 1.1)
        XCTAssertLessThan(Double(falseAlarms) / Double(trials), 0.02)
        // what a 3 s face buys: the scale to a few hundredths of a percent, the offset to a few tenths of a mg
        let s = try XCTUnwrap(last)
        XCTAssertLessThan([0, 4, 8].map { s.sigmaMatrix[$0] }.max()!, 5e-4)
        XCTAssertLessThan(s.sigmaBias.max()!, 0.5 * mg)
    }

    func testADisagreeingHighGIsFlaggedAndTheLowGAlignmentStillSaved() throws {
        // an even-order nonlinearity of 4 mg at one g on the high-g's Z: its own
        // gravity fit takes it all as offset, the alignment a third of it
        let q = 4 * mg / (g * g)
        let s = try fit(board(tilts: userTilts, quad: [0, 0, q]))
        XCTAssertEqual(s.disagreeAxes, [2])
        XCTAssertGreaterThan(abs(s.diffOffset[2]), 2 * mg)
        XCTAssertEqual(s.problems, [], "the alignment to the low-g is what is saved")
        XCTAssertTrue(s.agreementText.hasPrefix("Its own gravity fit differs on Z by "))
        XCTAssertTrue(s.agreementText.hasSuffix("the alignment to the low-g, the bench's reference, is what gets saved."))
        // a worse one leaves a face further from the low-g than the limit
        let worse = try fit(board(tilts: userTilts, quad: [0, 0, 1.5 * q]))
        XCTAssertTrue(worse.problems.contains(.residual))
        XCTAssertFalse(worse.canSave)
    }

    func testTheHighGRefusals() throws {
        XCTAssertEqual(try fit(board(tilts: userTilts, sHG: [1.045, 0, 0, 0, 1, 0, 0, 0, 1])).problems, [.scale])
        XCTAssertEqual(try fit(board(tilts: userTilts, sHG: [1, 0.05, 0, 0, 1, 0, 0, 0, 1])).problems, [.cross])
        XCTAssertEqual(try fit(board(tilts: userTilts, cHG: [12, 0, 0])).problems, [.bias])
        let faces = board(tilts: userTilts, withHighG: false)
        let lowG = try XCTUnwrap(AccelSixFace.solve(faces.mapValues(\.mean)))
        XCTAssertEqual(AccelSixFace.solveHighG(faces, lowG: lowG), .noReading)
        XCTAssertNotNil(AccelSixFace.HighGResult.noReading.unavailableText)
    }

    func testTheScreensNumbers() throws {
        let s = try fit(board(tilts: [:]))
        let want = truth()
        XCTAssertEqual(s.scaleText(0), String(format: "%+.3f %% ± %.3f", (want.m[0] - 1) * 100, s.sigmaMatrix[0] * 100))
        XCTAssertEqual(s.offsetText(1), String(format: "%+.1f mg ± %.2f", want.b[1] / mg, s.sigmaBias[1] / mg))
        XCTAssertTrue(s.crossText.hasPrefix("Cross-axis: up to 0."))
        XCTAssertTrue(s.residualText.hasSuffix("(limit 3 mg)"))
        XCTAssertNil(s.problemText)
    }

    // MARK: - The flow

    /// The six faces through the flow: the sixth sends the save.
    private func capturesFlow(withHighG: Bool = true, quad: [Double] = [0, 0, 0]) -> AccelCalFlow {
        var flow = AccelCalFlow()
        for (face, c) in board(tilts: [:], quad: quad, withHighG: withHighG) {
            XCTAssertEqual(flow.measure(), .measureFace)
            _ = flow.take(.face(.measuring, mean: [0, 0, 0], n: 0))
            let hg = c.highGMean.map {
                AccelCalStatus.HighG(faceMean: $0.map(Float.init), faceSD: [0.56, 0.56, 0.56], flags: 0, biasSource: 0,
                                     matrix: HighGCalApply.identity, bias: [0, 0, 0])
            }
            let frame = AccelCalStatus(flags: 0, faceState: AccelCalStatus.FaceState.done.rawValue, offset: [0, 0, 0],
                                       faceMean: c.mean.map(Float.init), faceSD: [0.019, 0.019, 0.019],
                                       faceN: UInt16(nSamples), highG: hg)
            _ = flow.take(frame)
            XCTAssertEqual(flow.captures.faces[face]?.highGMean != nil, withHighG)
        }
        return flow
    }

    /// A status frame saying what is stored, low-g and high-g.
    private func stored(_ offset: [Float]?, matrix: [Float]?, bias: [Float] = [0, 0, 0],
                        source: UInt8 = 2) -> AccelCalStatus {
        var s = AccelCalStatus.stored(offset)
        s.highG = AccelCalStatus.HighG(faceMean: [0, 0, 0], faceSD: [0, 0, 0],
                                       flags: (matrix == nil ? 0 : 0x03) | 0x04, biasSource: source,
                                       matrix: matrix ?? HighGCalApply.identity, bias: bias)
        return s
    }

    func testASaveSendsTheOffsetThenTheHighGAndReadsBothBack() throws {
        var flow = capturesFlow()
        guard case .solved(let h)? = flow.captures.highG else { return XCTFail("no high-g fit") }
        let o = try XCTUnwrap(flow.captures.solution).payload
        XCTAssertEqual(flow.writing, .save(o), "the sixth face sent it")
        XCTAssertEqual(flow.take(stored(o, matrix: nil)), .read)
        // the low-g's read-back, then the high-g's write
        let w = AccelCalFlow.HighGWrite.save(matrix: h.payloadMatrix, bias: h.payloadBias)
        XCTAssertEqual(flow.take(stored(o, matrix: nil)), .applyHighG(w))
        XCTAssertEqual(flow.lastWrite?.confirmed, true)
        XCTAssertEqual(flow.writingHighG, w)
        XCTAssertEqual(flow.timeout, 8)
        XCTAssertEqual(flow.take(stored(o, matrix: h.payloadMatrix, bias: h.payloadBias)), .read)
        XCTAssertNil(flow.take(stored(o, matrix: h.payloadMatrix, bias: h.payloadBias)))
        XCTAssertFalse(flow.isBusy)
        XCTAssertEqual(flow.lastHighGWrite?.confirmed, true)
        XCTAssertEqual(flow.lastHighGWrite?.text, "High-g alignment saved and read back")

        // a read-back from the pad calibration's bias, not the faces': not this save
        _ = flow.save()
        _ = flow.take(stored(o, matrix: nil))
        _ = flow.take(stored(o, matrix: nil))
        _ = flow.take(stored(o, matrix: h.payloadMatrix, bias: h.payloadBias, source: 1))
        _ = flow.take(stored(o, matrix: h.payloadMatrix, bias: h.payloadBias, source: 1))
        XCTAssertEqual(flow.lastHighGWrite?.confirmed, false)
        XCTAssertEqual(flow.lastHighGWrite?.text, "The high-g read-back does not match what was sent")
    }

    func testAnOlderFlightComputerSavesTheOffsetAndSaysWhyNotTheHighG() throws {
        // its frames carry no high-g: the fit is unavailable and only 76 goes out
        var flow = capturesFlow(withHighG: false)
        XCTAssertEqual(flow.captures.highG, .noReading)
        let o = try XCTUnwrap(flow.captures.solution).payload
        XCTAssertEqual(flow.writing, .save(o), "the sixth face sent it")
        XCTAssertEqual(flow.take(.stored(o)), .read)
        XCTAssertNil(flow.take(.stored(o)))
        XCTAssertNil(flow.lastHighGWrite)
        XCTAssertFalse(flow.isBusy)

        // a new flight computer behind an old out computer: 80 gets no answer
        var flow2 = capturesFlow()
        _ = flow2.take(.stored(o))
        guard case .applyHighG? = flow2.take(.stored(o)) else { return XCTFail("the high-g goes out") }
        XCTAssertEqual(flow2.timedOut(flow2.generation), .read)
        XCTAssertNil(flow2.take(.stored(o)))   // the old out computer's 40 bytes
        XCTAssertEqual(flow2.lastHighGWrite?.confirmed, false)
        XCTAssertEqual(flow2.lastHighGWrite?.text,
                       "The answer has no high-g in it: update the flight computer and the out computer")
    }

    func testAHighGThatFailsItsChecksIsNotSentButTheOffsetIs() throws {
        var flow = capturesFlow(quad: [0, 0, 6 * mg / (g * g)])
        guard case .solved(let h)? = flow.captures.highG else { return XCTFail("no high-g fit") }
        XCTAssertFalse(h.canSave)
        let o = try XCTUnwrap(flow.captures.solution).payload
        XCTAssertEqual(flow.writing, .save(o), "the sixth face sent the offset")
        _ = flow.take(stored(o, matrix: nil))
        XCTAssertNil(flow.take(stored(o, matrix: nil)))
        XCTAssertNil(flow.lastHighGWrite)
    }

    func testAClearTakesTheHighGWithItWhenOneIsStored() {
        var flow = AccelCalFlow()
        XCTAssertEqual(flow.clear(highG: true), .apply([0, 0, 0]))
        _ = flow.take(stored(nil, matrix: [1.01, 0, 0, 0, 1, 0, 0, 0, 1]))
        XCTAssertEqual(flow.take(stored(nil, matrix: [1.01, 0, 0, 0, 1, 0, 0, 0, 1])), .applyHighG(.clear))
        XCTAssertEqual(flow.take(stored(nil, matrix: nil)), .read)
        XCTAssertNil(flow.take(stored(nil, matrix: nil)))
        XCTAssertEqual(flow.lastWrite?.confirmed, true)
        XCTAssertEqual(flow.lastHighGWrite?.confirmed, true)
        XCTAssertEqual(flow.lastHighGWrite?.text, "Cleared: no high-g alignment stored")
        // without one stored, 76 alone, as before
        XCTAssertEqual(flow.clear(), .apply([0, 0, 0]))
        _ = flow.take(.stored(nil))
        XCTAssertNil(flow.take(.stored(nil)))
        XCTAssertNil(flow.lastHighGWrite)
    }

    func testNoAnswerToTheHighGThenReadItBack() throws {
        var flow = capturesFlow()
        guard case .solved(let h)? = flow.captures.highG else { return XCTFail("no high-g fit") }
        let o = try XCTUnwrap(flow.captures.solution).payload
        _ = flow.take(stored(o, matrix: nil))
        _ = flow.take(stored(o, matrix: nil))
        XCTAssertEqual(flow.timedOut(flow.generation), .read)
        XCTAssertNil(flow.timedOut(flow.generation))
        XCTAssertNil(flow.lastHighGWrite?.readBack)
        XCTAssertEqual(flow.lastHighGWrite?.text, "No answer about the high-g from the flight computer — read it back to check")
        XCTAssertEqual(flow.readBackAgain(), .read)
        XCTAssertNil(flow.take(stored(o, matrix: h.payloadMatrix, bias: h.payloadBias)))
        XCTAssertEqual(flow.lastHighGWrite?.confirmed, true)
    }

    // MARK: - The rocket profile

    func testAConfirmedSaveGivesThisBoardsProfileTheNewBias() {
        let cal = SensorCalData(gyroX: 3, gyroY: -2, gyroZ: 1, hgX: -0.65, hgY: -0.57, hgZ: 0.91,
                                calibratedOnUnitID: "AA:BB", calibratedAt: Date(timeIntervalSince1970: 0))
        let updated = ActiveRocketSyncer.sensorCalAfterHighGSave(profileCal: cal, deviceUnitID: "AA:BB",
                                                                 bias: [-0.6512, 1.48, -0.3127])
        XCTAssertEqual(updated?.hgX, -0.6512)
        XCTAssertEqual(updated?.hgY, 1.48)
        XCTAssertEqual(updated?.hgZ, -0.3127)
        XCTAssertEqual(updated?.gyroX, 3, "the gyro's offsets stay the pad calibration's")
        XCTAssertEqual(updated?.calibratedAt, cal.calibratedAt)
        // another board's cal is never pushed, and no cal stays none
        XCTAssertNil(ActiveRocketSyncer.sensorCalAfterHighGSave(profileCal: cal, deviceUnitID: "CC:DD", bias: [0, 0, 0]))
        XCTAssertNil(ActiveRocketSyncer.sensorCalAfterHighGSave(profileCal: nil, deviceUnitID: "AA:BB", bias: [0, 0, 0]))
    }

    // MARK: - The screen, rendered (only with TR_SCREENSHOT_DIR set)

    /// The six-face screen with six captured faces and the save they sent,
    /// hosted and drawn to a PNG in $TR_SCREENSHOT_DIR (xcodebuild:
    /// TEST_RUNNER_TR_SCREENSHOT_DIR=...).
    @MainActor
    private func renderScreen(_ name: String, faces: [Face: AccelSixFace.Capture],
                              stored: AccelCalStatus) async throws {
        guard let dir = ProcessInfo.processInfo.environment["TR_SCREENSHOT_DIR"] else {
            throw XCTSkip("set TR_SCREENSHOT_DIR to render the screen")
        }
        var flow = AccelCalFlow()
        for (_, c) in faces {
            _ = flow.measure()
            _ = flow.take(.face(.measuring, mean: [0, 0, 0], n: 0))
            let hg = c.highGMean.map {
                AccelCalStatus.HighG(faceMean: $0.map(Float.init), faceSD: c.highGSD!.map(Float.init), flags: 0,
                                     biasSource: 0, matrix: HighGCalApply.identity, bias: [0, 0, 0])
            }
            _ = flow.take(AccelCalStatus(flags: 0, faceState: 2, offset: [0, 0, 0], faceMean: c.mean.map(Float.init),
                                         faceSD: c.sd.map(Float.init), faceN: UInt16(c.samples), highG: hg))
        }
        var shown = stored
        // The sixth face sent the save: answer it, and the high-g's after it.
        if let o = flow.writing?.sent {
            let lowStored = AccelCalStatus(flags: 3, faceState: 0, offset: o, faceMean: [0, 0, 0], faceSD: [0, 0, 0],
                                       faceN: 0, highG: stored.highG)
            _ = flow.take(lowStored)
            if case .applyHighG(let w)? = flow.take(lowStored) {
                shown = AccelCalStatus(flags: 3, faceState: 0, offset: o, faceMean: [0, 0, 0], faceSD: [0, 0, 0],
                                       faceN: 0, highG: AccelCalStatus.HighG(faceMean: [0, 0, 0], faceSD: [0, 0, 0],
                                                                             flags: 0x07, biasSource: 2,
                                                                             matrix: w.matrix, bias: w.bias))
                _ = flow.take(shown)
                _ = flow.take(shown)
            } else {
                shown = lowStored
            }
        }
        let device = BLEDevice(peripheral: nil, name: "RC V9")
        device.isConnected = true
        device.parseTelemetryData(#"{"st":"READY","fs":16,"lx":9.79,"ly":0.12,"lz":-0.05}"#.data(using: .utf8))
        device.handleFileOpsFrame(shown.wireFrame)
        let defaults = UserDefaults(suiteName: "HighGScreen")!
        defaults.removePersistentDomain(forName: "HighGScreen")
        let store = RocketProfileStore(directory: FileManager.default.temporaryDirectory
                                        .appendingPathComponent("hgscreen-\(UUID())"), defaults: defaults)
        let view = NavigationView { AccelCalibrationView(device: device, initialFlow: flow) }
            .navigationViewStyle(.stack)
            .environmentObject(store)
        let scene = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }.first
        let w = scene.map { UIWindow(windowScene: $0) } ?? UIWindow()
        w.frame = scene?.coordinateSpace.bounds ?? CGRect(x: 0, y: 0, width: 402, height: 874)
        w.rootViewController = UIHostingController(rootView: view)
        w.makeKeyAndVisible()
        try? await Task.sleep(nanoseconds: 1_500_000_000)
        device.handleFileOpsFrame(shown.wireFrame)   // the screen's opening read
        try? await Task.sleep(nanoseconds: 1_000_000_000)
        // A page at a time: the Form scrolled, a marker file written, and the
        // host's `simctl io screenshot` loop answering it with a done file.
        func views(_ v: UIView) -> [UIView] { [v] + v.subviews.flatMap(views) }
        let form = try XCTUnwrap(views(w).compactMap { $0 as? UICollectionView }.first)
        let page = form.bounds.height - form.adjustedContentInset.top - form.adjustedContentInset.bottom - 60
        var offset = -form.adjustedContentInset.top
        var n = 0
        while true {
            // the Form lays its cells out as they come into view: its height grows as it scrolls
            let bottom = form.contentSize.height + form.adjustedContentInset.bottom - form.bounds.height
            form.setContentOffset(CGPoint(x: 0, y: min(offset, max(bottom, -form.adjustedContentInset.top))),
                                  animated: false)
            try? await Task.sleep(nanoseconds: 800_000_000)
            let tag = "\(name)_\(n)"
            let ready = URL(fileURLWithPath: dir).appendingPathComponent("ready-\(tag)")
            let done = URL(fileURLWithPath: dir).appendingPathComponent("done-\(tag)")
            try Data().write(to: ready)
            let deadline = Date().addingTimeInterval(20)
            while !FileManager.default.fileExists(atPath: done.path), Date() < deadline {
                try? await Task.sleep(nanoseconds: 200_000_000)
            }
            n += 1
            let end = form.contentSize.height + form.adjustedContentInset.bottom - form.bounds.height
            if offset >= end || n >= 8 { break }
            offset += page
        }
        w.isHidden = true
    }

    @MainActor
    func testRenderTheScreenPassedAndSaved() async throws {
        try await renderScreen("ios_highg_passed_saved", faces: board(tilts: userTilts, noiseSeed: 11),
                               stored: stored(nil, matrix: nil, bias: [-0.65, -0.57, 0.91], source: 1))
    }

    @MainActor
    func testRenderTheScreenDisagreeing() async throws {
        try await renderScreen("ios_highg_disagree", faces: board(tilts: userTilts, noiseSeed: 12,
                                                                  quad: [0, 0, 3.5 * mg / (g * g)]),
                               stored: stored(nil, matrix: nil, bias: [-0.65, -0.57, 0.91], source: 1))
    }

    @MainActor
    func testRenderTheScreenOlderFirmware() async throws {
        try await renderScreen("ios_highg_older_firmware", faces: board(tilts: userTilts, noiseSeed: 13, withHighG: false),
                               stored: .stored([0.0412, -0.0273, 0.0156]))
    }

    // MARK: - On the device's wire

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
            case .applyHighG(let w)?: device.sendHighGCalApply(op: w.op, matrix: w.matrix, bias: w.bias)
            case nil: break
            }
        }
        device.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        let sub = device.accelCalFrames.sink { run(flow.take($0)) }
        defer { sub.cancel() }
        for (_, c) in board(tilts: userTilts) {
            run(flow.measure())
            device.handleFileOpsFrame(AccelCalStatus.face(.measuring, mean: [0, 0, 0], n: 0).wireFrame)
            let hg = AccelCalStatus.HighG(faceMean: c.highGMean!.map(Float.init), faceSD: [0.56, 0.56, 0.56],
                                          flags: 0, biasSource: 0, matrix: HighGCalApply.identity, bias: [0, 0, 0])
            device.handleFileOpsFrame(AccelCalStatus(flags: 0, faceState: 2, offset: [0, 0, 0],
                                                     faceMean: c.mean.map(Float.init), faceSD: [0.019, 0.019, 0.019],
                                                     faceN: 11340, highG: hg).wireFrame)
        }
        XCTAssertTrue(flow.captures.isComplete)
        guard case .solved(let h)? = flow.captures.highG else { return XCTFail("no high-g fit") }
        // the sixth face's frame sent the save by itself
        XCTAssertEqual(writes.prefix(7).map { $0[0] }, [78, 78, 78, 78, 78, 78, 76])
        writes.removeFirst(6)
        let o = try XCTUnwrap(flow.writing?.sent)
        device.handleFileOpsFrame(stored(o, matrix: nil).wireFrame)    // the apply's answer
        device.handleFileOpsFrame(stored(o, matrix: nil).wireFrame)    // its read-back -> 80
        let after = stored(o, matrix: h.payloadMatrix, bias: h.payloadBias)
        device.handleFileOpsFrame(after.wireFrame)                     // 80's answer
        device.handleFileOpsFrame(after.wireFrame)                     // its read-back
        XCTAssertEqual(writes.map { $0[0] }, [76, 77, 80, 77])
        XCTAssertEqual(writes[2], Data([80]) + HighGCalApply.payload(op: 1, matrix: h.payloadMatrix, bias: h.payloadBias))
        XCTAssertEqual(flow.lastWrite?.text, "Saved and read back")
        XCTAssertEqual(flow.lastHighGWrite?.text, "High-g alignment saved and read back")
        XCTAssertEqual(device.accelCalStatus?.highG?.isStored, true)
    }
}

/// SplitMix64 for the noise trials (the same on every run).
struct SplitMix: RandomNumberGenerator {
    var state: UInt64

    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
}
