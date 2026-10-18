//
//  AccelSixFace.swift
//  TinkerRocketApp
//
//  The low-g accelerometer's six-face hand calibration (#1629), without the
//  BLE and the timers: which face a reading is, the solve, the limits, the
//  rule that picks a face window's result out of the 0xD1 frames, and the
//  six-face screen's request/answer flow. AccelCalibrationView sends what
//  AccelCalFlow asks for and feeds it the frames and the timeouts. Also the
//  screen's live hint: which face the board is on now, from telemetry.
//
//  The reference for the flow is tools/accel_cal_six_face.py. On a still pad
//  the navigation filter levels on gravity, and an accelerometer offset there
//  looks exactly like a tilt. Turn each board axis straight up and then
//  straight down: gravity flips sign on that axis and the offset does not.
//  The per-axis average
//
//      offset_i = (mean with axis i up + mean with axis i down) / 2
//
//  cancels a face's tilt only when both faces of the axis are tilted alike,
//  so it only seeds the solve: one sphere fit, |k * (mean - offset)| = g for
//  all six faces at once with a scale per axis, which no tilt biases.
//  The board frame is +X forward, +Y left, +Z up out of the component side,
//  whatever the board's mounting in the rocket.
//
//  The same windows average the high-g accelerometer (a flight and out
//  computer that send it). Its correction is fitted to the low-g, the
//  accurate channel on the bench: the matrix M (scale and cross-axis) and
//  the offset b with
//
//      l = M * h - b   on every face (least squares)
//
//  l the raw low-g, h the raw high-g, so the high-g reads as the calibrated
//  low-g and takes over at the low-g's rail with no step (RocketComputerTypes.h,
//  HG_CAL_*). The high-g's own sphere fit is shown beside it as a check; when
//  the two disagree beyond their noise, the alignment to the low-g is what is
//  saved.
//

import Foundation

nonisolated enum AccelSixFace {

    /// Standard gravity, m/s²: the solve's g and the mg unit.
    static let g0 = 9.80665
    /// m/s²: the flight computer refuses a larger component
    /// (ACC_CAL_OFFSET_MAX_MPS2), so the screen will not save one.
    static let offsetMax = 1.0
    /// A face whose axis is further off vertical is measured again.
    static let tiltMaxDeg = 10.0
    /// m/s²: how close a read-back must be to what was sent.
    static let readBackTolerance = 1e-5

    static let axisNames = ["X", "Y", "Z"]

    // MARK: - Faces

    /// One of the six faces: a board axis pointing straight up or straight down.
    nonisolated struct Face: Hashable, Identifiable, CaseIterable, CustomStringConvertible {
        /// 0 = X, 1 = Y, 2 = Z.
        let axis: Int
        /// The axis points up: a still accelerometer reads +g along it.
        let up: Bool

        static let allCases: [Face] = (0..<3).flatMap { [Face(axis: $0, up: true), Face(axis: $0, up: false)] }

        /// "+X UP", as the reference script names it.
        var name: String { "+\(AccelSixFace.axisNames[axis]) \(up ? "UP" : "DOWN")" }
        var id: String { name }
        var description: String { name }

        /// The face in plain words.
        var howTo: String {
            let part = ["Forward end", "Left side", "Component side"][axis]
            return "\(part) \(up ? "up" : "down")"
        }

        /// `howTo` in lower case, inside a sentence: "left side up".
        var hint: String { howTo.lowercased() }

        /// How to set the board down on this face.
        var placement: String {
            switch (axis, up) {
            case (0, true):  return "stand it on its back end, forward end up."
            case (0, false): return "stand it on its forward end, forward end down."
            case (1, true):  return "lay it on its right side, left side up."
            case (1, false): return "lay it on its left side, left side down."
            case (2, true):  return "lay it flat, component side up."
            default:         return "lay it flat, component side down."
            }
        }
    }

    /// The face a still reading is: the axis nearest vertical (the largest
    /// |mean|), up if gravity reads positive on it, and how far that axis is
    /// off vertical.
    static func identify(_ mean: [Double]) -> (face: Face, tiltDeg: Double) {
        var axis = 0
        for i in 1..<3 where abs(mean[i]) > abs(mean[axis]) { axis = i }
        let norm = (mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]).squareRoot()
        let tilt = norm > 0 ? acos(min(1, abs(mean[axis]) / norm)) * 180 / .pi : 90
        return (Face(axis: axis, up: mean[axis] > 0), tilt)
    }

    // MARK: - Which face it is on now (a hint; a measurement names its own)

    /// The board→rocket mounting rotations: orientCodeToMatrix in
    /// TR_Orientation.cpp, ported exactly. code = nose_sel * 4 + clock (24
    /// proper rotations; nose_sel picks the board axis toward the nose: +X,
    /// -X, +Y, -Y, +Z, -Z), R = Rx(clock) * B[nose_sel], row-major,
    /// v_rocket = R * v_board.
    nonisolated enum Mounting {
        static let codeCount = 24

        /// clock = 0: the shortest arc taking the selected board axis to the
        /// nose (+X). All entries exact 0 / ±1.
        private static let noseBase: [[Int]] = [
            [1, 0, 0, 0, 1, 0, 0, 0, 1],      // +X: identity
            [-1, 0, 0, 0, -1, 0, 0, 0, 1],    // -X: 180° about board +Z
            [0, 1, 0, -1, 0, 0, 0, 0, 1],     // +Y: 90° about -Z
            [0, -1, 0, 1, 0, 0, 0, 0, 1],     // -Y: 90° about +Z
            [0, 0, 1, 0, 1, 0, -1, 0, 0],     // +Z: 90° about +Y
            [0, 0, -1, 0, 1, 0, 1, 0, 0],     // -Z: 90° about -Y
        ]
        /// A quarter-turn about rocket +X (CCW), after the nose alignment.
        private static let clockCos: [Int] = [1, 0, -1, 0]
        private static let clockSin: [Int] = [0, 1, 0, -1]

        /// R for `code`, row-major; nil outside 0...23. (The firmware takes
        /// such a code as identity; the app gives no hint instead.)
        static func matrix(_ code: Int) -> [Int]? {
            guard code >= 0, code < codeCount else { return nil }
            let b = noseBase[code / 4]
            let c = clockCos[code % 4]
            let s = clockSin[code % 4]
            // R = Rx(clock) * B with Rx = [[1,0,0],[0,c,-s],[0,s,c]]:
            // row 0 = B row 0, row 1 = c B row 1 - s B row 2, row 2 = s B row 1 + c B row 2.
            var r = [Int](repeating: 0, count: 9)
            for j in 0..<3 {
                r[j] = b[j]
                r[3 + j] = c * b[3 + j] - s * b[6 + j]
                r[6 + j] = s * b[3 + j] + c * b[6 + j]
            }
            return r
        }

        /// A rocket-frame vector back in the board frame: R^T * v.
        static func toBoard(_ rocket: [Double], code: Int) -> [Double]? {
            guard rocket.count == 3, let r = matrix(code) else { return nil }
            return (0..<3).map { j -> Double in
                let fromX: Double = Double(r[j]) * rocket[0]
                let fromY: Double = Double(r[3 + j]) * rocket[1]
                let fromZ: Double = Double(r[6 + j]) * rocket[2]
                return fromX + fromY + fromZ
            }
        }
    }

    /// m/s²: a reading this far off one g (0.1 g) is a board being moved.
    static let liveMovingTolerance = 0.98

    /// Degrees: under auto-exact, a still reading further than this from
    /// rocket +X is a board the flight computer has not re-read yet -- it
    /// re-reads after 2 s still (its ORIENT_ACCEPT_TOL_DEG), and until then
    /// the code beside the reading still names the face the board was on.
    static let exactSettledDeg = 5.0

    /// Which face the board is on now, from the newest telemetry frame.
    enum LiveFace: Equatable {
        /// No reading yet, a field missing, or no mounting it can be turned
        /// back with.
        case waiting
        /// More than 0.1 g off one g, or, under auto-exact, more than 5° off
        /// rocket +X: a board the flight computer has not re-read since it
        /// was moved.
        case moving
        /// More than 10° off vertical.
        case askew(Face, tiltDeg: Double)
        /// On the face; `tiltDeg` nil when the mounting cannot tell the tilt
        /// (auto-exact).
        case on(Face, tiltDeg: Double?)

        /// The checklist row to highlight.
        var face: Face? {
            switch self {
            case .on(let f, _), .askew(let f, _): return f
            case .waiting, .moving: return nil
            }
        }
    }

    /// The mounting a frame's lx/ly/lz are in, as far as the app can name it.
    struct LiveMounting: Equatable {
        /// orientCodeToMatrix's code.
        let code: Int
        /// The rotation in use is the code's, so the tilt reads true. In
        /// auto-exact the flight computer uses the exact pad-gravity rotation
        /// and reports the code of the board axis nearest up beside it:
        /// turning back with that code still names the face, but snaps up
        /// onto its axis, so the tilt reads about 0 and means nothing.
        let tiltKnown: Bool
    }

    /// The face from a rocket-frame low-g reading and the mounting it is in:
    /// turned back to the board frame (R^T), then named as a measurement is.
    /// Without a true tilt (auto-exact) it is never askew: the measurement
    /// judges the tilt. And there it names the face only once the reading has
    /// settled onto rocket +X.
    static func liveFace(rocket: [Double]?, mounting: LiveMounting?) -> LiveFace {
        guard let rocket, let mounting,
              let v = Mounting.toBoard(rocket, code: mounting.code) else { return .waiting }
        let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).squareRoot()
        if abs(norm - g0) > liveMovingTolerance { return .moving }
        let (face, tilt) = identify(v)
        guard mounting.tiltKnown else {
            let cosOffNose: Double = max(-1, min(1, rocket[0] / norm))
            let offNoseDeg: Double = acos(cosOffNose) * 180 / .pi
            return offNoseDeg > exactSettledDeg ? .moving : .on(face, tiltDeg: nil)
        }
        return tilt > tiltMaxDeg ? .askew(face, tiltDeg: tilt) : .on(face, tiltDeg: tilt)
    }

    /// The mounting a telemetry frame's lx/ly/lz are in. The frame's own
    /// "imo" (code = imo & 0x1F; 31, the relay's auto-exact, names none)
    /// when it carries one; the base station's relay does. The rocket's own
    /// link leaves imo out and reports the mounting in its "imu_orient"
    /// message instead, re-sent as the pad auto-detect re-racks: its code
    /// with a true tilt in default, manual and auto-snap mode, and without
    /// one in auto-exact. nil: no hint.
    static func liveMounting(imo: Int?, reportedCode: Int?,
                             reportedMode: IMUOrientationMode) -> LiveMounting? {
        if let imo { return LiveMounting(code: imo & 0x1F, tiltKnown: true) }
        guard let code = reportedCode else { return nil }
        switch reportedMode {
        case .defaultMounting, .manual, .autoSnap: return LiveMounting(code: code, tiltKnown: true)
        case .autoExact: return LiveMounting(code: code, tiltKnown: false)
        case .unknown: return nil
        }
    }

    /// The Measure button's label: the live face when the board sits square
    /// on one. Only a hint: the measurement names its face from its reading.
    static func measureLabel(live: LiveFace, allCaptured: Bool) -> String {
        if case .on(let face, _) = live { return "Measure \(face.name)" }
        return allCaptured ? "Measure a Face Again" : "Measure This Face"
    }

    // MARK: - One face window

    /// What a face window came to.
    enum Verdict: Equatable {
        /// Still, one g, and square enough: captured.
        case accepted(Face, tiltDeg: Double)
        /// FAILED: it moved, or it is not one g.
        case moved
        /// Still and one g, but the axis is more than 10° off vertical.
        case tilted(Face, tiltDeg: Double)
        /// The flight computer would not open a window: in flight, after a
        /// flight, in a sim or during a magnetometer calibration.
        case refused
        /// Nothing came back in time.
        case noAnswer
    }

    /// The verdict on a window's closing frame (DONE or FAILED).
    static func verdict(for frame: AccelCalStatus) -> Verdict {
        guard frame.state == .done else { return .moved }
        let (face, tilt) = identify(frame.faceMean.map(Double.init))
        return tilt <= tiltMaxDeg ? .accepted(face, tiltDeg: tilt) : .tilted(face, tiltDeg: tilt)
    }

    /// The sequencing rule for one command 78: only a frame that follows the
    /// app's own 78 counts. Frames are ignored until a MEASURING frame, and the
    /// next DONE or FAILED after it is the result. A FAILED frame with no
    /// samples arriving instead of MEASURING is a refusal. Anything else before
    /// MEASURING -- a late answer to an earlier 77, which carries the previous
    /// window's state -- is not this face's result.
    nonisolated struct FaceMeasurement: Equatable {
        enum Result: Equatable {
            /// The window's closing frame, DONE or FAILED.
            case finished(AccelCalStatus)
            case refused
        }

        /// A MEASURING frame has come: the window is open.
        private(set) var windowOpen = false

        mutating func take(_ frame: AccelCalStatus) -> Result? {
            switch frame.state {
            case .measuring?:
                windowOpen = true
                return nil
            case .done?, .failed?:
                if windowOpen { return .finished(frame) }
                if frame.state == .failed && frame.faceN == 0 { return .refused }
                return nil
            case .idle?, nil:
                return nil
            }
        }
    }

    // MARK: - The six faces

    /// A captured face: its window's numbers.
    nonisolated struct Capture: Equatable {
        let mean: [Double]
        let sd: [Double]
        let sdMax: Double
        let samples: Int
        let tiltDeg: Double
        /// The same window's high-g mean and SD (board frame, m/s²); nil when
        /// the frame carried none.
        let highGMean: [Double]?
        let highGSD: [Double]?

        init(_ frame: AccelCalStatus, tiltDeg: Double) {
            mean = frame.faceMean.map(Double.init)
            sd = frame.faceSD.map(Double.init)
            sdMax = sd.max() ?? 0
            samples = Int(frame.faceN)
            self.tiltDeg = tiltDeg
            highGMean = frame.highG.map { $0.faceMean.map(Double.init) }
            highGSD = frame.highG.map { $0.faceSD.map(Double.init) }
        }
    }

    /// The faces captured so far, in any order.
    nonisolated struct Captures: Equatable {
        private(set) var faces: [Face: Capture] = [:]

        var missing: [Face] { Face.allCases.filter { faces[$0] == nil } }
        var isComplete: Bool { missing.isEmpty }

        /// Keep `capture` for `face`. True when it replaced an earlier one.
        @discardableResult
        mutating func record(_ face: Face, _ capture: Capture) -> Bool {
            faces.updateValue(capture, forKey: face) != nil
        }

        /// The solve, once all six are in.
        var solution: Solution? {
            isComplete ? AccelSixFace.solve(faces.mapValues(\.mean)) : nil
        }

        /// The high-g's alignment to the low-g, once all six are in.
        var highG: HighGResult? {
            guard isComplete, let lowG = solution else { return nil }
            return AccelSixFace.solveHighG(faces, lowG: lowG)
        }
    }

    // MARK: - The solve: one sphere fit over all six faces

    /// Save only scales within this range (1 / k), as a check on the fit.
    static let scaleMin = 0.97
    static let scaleMax = 1.03
    /// Gauss-Newton: stop on a step below `fitStepTolerance`, give up after
    /// `fitMaxIterations`; converged only on the step, with every face then
    /// within `fitResidualTolerance` (m/s²) of one g.
    static let fitMaxIterations = 20
    static let fitStepTolerance = 1e-9
    static let fitResidualTolerance = 1e-6

    nonisolated struct Solution: Equatable {
        /// m/s², board frame x, y, z: what gets saved.
        let offset: [Double]
        /// Per axis, 1 / k: shown for information, never sent.
        let scale: [Double]
        /// The fit stopped on its step size with every face on the sphere.
        let converged: Bool
        /// Gauss-Newton steps taken.
        let iterations: Int

        /// The axes past 1.0 m/s², which the flight computer would refuse.
        var tooLargeAxes: [Int] { (0..<3).filter { abs(offset[$0]) > AccelSixFace.offsetMax } }
        /// The axes whose scale is outside 0.97...1.03.
        var scaleOutOfRangeAxes: [Int] {
            (0..<3).filter { !(scale[$0] >= AccelSixFace.scaleMin && scale[$0] <= AccelSixFace.scaleMax) }
        }
        var canSave: Bool { converged && tooLargeAxes.isEmpty && scaleOutOfRangeAxes.isEmpty }

        /// The command-76 values.
        var payload: [Float] { offset.map(Float.init) }
    }

    /// Solve the offset and scale from the six faces' means (nil until all
    /// six are there): one sphere fit, |k * (m - b)| = g for every face, over
    /// p = (b_x, b_y, b_z, k_x, k_y, k_z) -- the offset b and k = 1 / scale per
    /// axis. Six faces, six unknowns: it fits them exactly, so a face a few
    /// degrees off square does not bias the offset, as it does the per-axis
    /// average (up + down) / 2 when an axis's two faces are tilted unequally.
    /// That average seeds it, with k = 1. Gauss-Newton on the normal
    /// equations, double precision throughout.
    static func solve(_ means: [Face: [Double]], g: Double = g0) -> Solution? {
        var rows: [[Double]] = []
        for face in Face.allCases {
            guard let m = means[face], m.count == 3 else { return nil }
            rows.append(m)
        }
        var p: [Double] = [0, 0, 0, 1, 1, 1]
        for axis in 0..<3 {
            let up: Double = rows[2 * axis][axis]
            let down: Double = rows[2 * axis + 1][axis]
            p[axis] = 0.5 * (up + down)
        }
        var onStep = false
        var iterations = 0
        while iterations < fitMaxIterations {
            guard let fitted = fitResiduals(rows, p, g) else { break }
            let r = fitted.r
            let jac = fitted.jac
            iterations += 1
            // J^T J dp = -J^T r
            var a = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
            var rhs = [Double](repeating: 0, count: 6)
            for f in 0..<rows.count {
                for i in 0..<6 {
                    rhs[i] -= jac[f][i] * r[f]
                    for j in 0..<6 { a[i][j] += jac[f][i] * jac[f][j] }
                }
            }
            guard let dp = solveLinear(a, rhs) else { break }
            let next = (0..<6).map { p[$0] + dp[$0] }
            guard next.allSatisfy({ $0.isFinite }) else { break }
            p = next
            if dp.allSatisfy({ abs($0) < fitStepTolerance }) {
                onStep = true
                break
            }
        }
        let onSphere = fitResiduals(rows, p, g)?.r.allSatisfy { abs($0) < fitResidualTolerance } ?? false
        return Solution(offset: Array(p[0..<3]), scale: (3..<6).map { 1 / p[$0] },
                        converged: onStep && onSphere, iterations: iterations)
    }

    /// The fit's residuals |k * (m - b)| - g, one per face, and their
    /// Jacobian rows: dr/db_i = -k_i v_i / n, dr/dk_i = (m_i - b_i) v_i / n.
    /// nil when a face sits on the offset (n = 0) or anything is not finite.
    private static func fitResiduals(_ rows: [[Double]], _ p: [Double],
                                     _ g: Double) -> (r: [Double], jac: [[Double]])? {
        var r: [Double] = []
        var jac: [[Double]] = []
        for m in rows {
            let d: [Double] = (0..<3).map { m[$0] - p[$0] }
            let v: [Double] = (0..<3).map { p[3 + $0] * d[$0] }
            let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).squareRoot()
            guard n > 1e-9, n.isFinite else { return nil }
            r.append(n - g)
            let dB: [Double] = (0..<3).map { -p[3 + $0] * v[$0] / n }
            let dK: [Double] = (0..<3).map { d[$0] * v[$0] / n }
            jac.append(dB + dK)
        }
        return (r, jac)
    }

    /// Solve a x = b by Gaussian elimination with partial pivoting; nil when
    /// singular.
    private static func solveLinear(_ a: [[Double]], _ b: [Double]) -> [Double]? {
        let n = b.count
        var m = a
        var y = b
        for c in 0..<n {
            var pivot = c
            for row in (c + 1)..<n where abs(m[row][c]) > abs(m[pivot][c]) { pivot = row }
            guard abs(m[pivot][c]) > 1e-12 else { return nil }
            if pivot != c {
                m.swapAt(c, pivot)
                y.swapAt(c, pivot)
            }
            for row in (c + 1)..<n {
                let f = m[row][c] / m[c][c]
                for k in c..<n { m[row][k] -= f * m[c][k] }
                y[row] -= f * y[c]
            }
        }
        var x = [Double](repeating: 0, count: n)
        for row in stride(from: n - 1, through: 0, by: -1) {
            var s = y[row]
            for k in (row + 1)..<n { s -= m[row][k] * x[k] }
            x[row] = s / m[row][row]
        }
        return x
    }

    /// A save's read-back: stored, and each component within 1e-5 m/s² of
    /// what was sent.
    static func readBackMatches(_ status: AccelCalStatus, sent: [Float]) -> Bool {
        status.isStored && (0..<3).allSatisfy {
            abs(Double(status.offset[$0]) - Double(sent[$0])) < readBackTolerance
        }
    }

    // MARK: - The high-g, aligned to the low-g

    /// What the flight computer refuses (HG_CAL_SCALE_MIN...MAX,
    /// HG_CAL_CROSS_MAX, HG_CAL_BIAS_MAX_MPS2) and what the screen refuses on
    /// top: a face fitted worse than `highGResidualMax`, ~7x the residual of a
    /// still 3 s face.
    static let highGScaleMin = 0.97
    static let highGScaleMax = 1.03
    static let highGCrossMax = 0.03
    static let highGBiasMax = 11.0
    static let highGResidualMax = 0.03
    /// m/s²: each face mean's error beyond its samples' SD / sqrt(n) (the
    /// offset's drift while the board is handled, ~2 mg/°C, and slow noise).
    static let highGNoiseFloor = 0.002
    /// The floors under the two fits' agreement test, and its width.
    static let highGAgreeFloorOffset = 0.005
    static let highGAgreeFloorScale = 2e-4
    static let highGAgreeSigmas = 3.0
    /// How close a read-back's matrix must be to what was sent.
    static let matrixReadBackTolerance = 1e-6
    /// The off-diagonal entries of a row-major 3x3.
    static let crossIndices = [1, 2, 3, 5, 6, 7]

    nonisolated enum HighGResult: Equatable {
        /// The faces came without the high-g reading: an older flight or out
        /// computer.
        case noReading
        /// The fit could not be solved (the faces do not span three axes).
        case singular
        case solved(HighGSolution)
    }

    nonisolated struct HighGSolution: Equatable {
        enum Problem: Equatable {
            case residual, scale, cross, bias
        }

        /// M, row-major, and b (m/s², board frame): l = M h - b.
        let matrix: [Double]
        let bias: [Double]
        /// m/s², over the six faces' 18 equations.
        let residualRMS: Double
        let residualMax: Double
        /// 1-sigma of each M entry and of b, from the faces' noise.
        let sigmaMatrix: [Double]
        let sigmaBias: [Double]
        /// The high-g's own sphere fit, and the zero-g the alignment implies
        /// for it, M^-1 (b + the low-g's offset).
        let ownOffset: [Double]
        let ownScale: [Double]
        let impliedZero: [Double]
        /// Per axis: the alignment less the own fit (offset, m/s²; scale), the
        /// difference's 1-sigma, and the axes where it is past 3 sigma.
        let diffOffset: [Double]
        let diffScale: [Double]
        let sigmaDiffOffset: [Double]
        let sigmaDiffScale: [Double]
        let disagreeAxes: [Int]
        let problems: [Problem]

        /// Within the limits: saved with the offset. A disagreement is not a
        /// problem: the alignment to the low-g is saved either way.
        var canSave: Bool { problems.isEmpty }
        var payloadMatrix: [Float] { matrix.map(Float.init) }
        var payloadBias: [Float] { bias.map(Float.init) }
        /// The largest cross-axis term, as an angle, degrees.
        var crossDeg: Double {
            atan(AccelSixFace.crossIndices.map { abs(matrix[$0]) }.max() ?? 0) * 180 / .pi
        }
        var sigmaCrossDeg: Double {
            (AccelSixFace.crossIndices.map { sigmaMatrix[$0] }.max() ?? 0) * 180 / .pi
        }
    }

    /// The high-g's alignment to the low-g from the six captures, with the
    /// low-g's own fit (`lowG`, for the agreement check): one row of M (and b)
    /// at a time, [h_x h_y h_z -1] (M_i, b_i) = l_i over the six faces -- four
    /// unknowns, six equations. Its 1-sigma comes from the faces' noise (each
    /// mean's SD / sqrt(n), the high-g's through M, and the noise floor), not
    /// the residuals, two degrees of freedom a row being too few to judge by.
    /// tools/accel_cal_six_face.py's solve_high_g, number for number.
    static func solveHighG(_ faces: [Face: Capture], lowG: Solution, g: Double = g0) -> HighGResult {
        var l: [[Double]] = []
        var h: [[Double]] = []
        var sl: [[Double]] = []
        var sh: [[Double]] = []
        var n: [Double] = []
        for face in Face.allCases {
            guard let c = faces[face], let hm = c.highGMean, let hs = c.highGSD else { return .noReading }
            l.append(c.mean)
            h.append(hm)
            sl.append(c.sd)
            sh.append(hs)
            n.append(Double(max(c.samples, 1)))
        }
        let x: [[Double]] = h.map { $0 + [-1] }
        var xtx = [[Double]](repeating: [Double](repeating: 0, count: 4), count: 4)
        for f in 0..<6 {
            for i in 0..<4 {
                for j in 0..<4 { xtx[i][j] += x[f][i] * x[f][j] }
            }
        }
        guard let xtxInv = invert(xtx) else { return .singular }
        var m = [Double](repeating: 0, count: 9)
        var b = [Double](repeating: 0, count: 3)
        for i in 0..<3 {
            var rhs = [Double](repeating: 0, count: 4)
            for f in 0..<6 {
                for j in 0..<4 { rhs[j] += x[f][j] * l[f][i] }
            }
            for j in 0..<4 {
                var t: Double = 0
                for k in 0..<4 { t += xtxInv[j][k] * rhs[k] }
                if j < 3 { m[3 * i + j] = t } else { b[i] = t }
            }
        }
        guard (m + b).allSatisfy({ $0.isFinite }) else { return .singular }

        var sumSq: Double = 0
        var worst: Double = 0
        for f in 0..<6 {
            let mh = mul(m, h[f])
            for i in 0..<3 {
                let r: Double = mh[i] - b[i] - l[f][i]
                sumSq += r * r
                worst = max(worst, abs(r))
            }
        }
        let rms = (sumSq / 18).squareRoot()
        // each row's noise: the mean over the faces of every reading's variance in that row's equation
        var sigRow: [Double] = []
        for i in 0..<3 {
            var v: Double = 0
            for f in 0..<6 {
                var hv: Double = 0
                for j in 0..<3 {
                    let mj: Double = m[3 * i + j] * sh[f][j]
                    hv += mj * mj
                }
                let lv: Double = sl[f][i] * sl[f][i]
                v += (lv + hv) / n[f] + highGNoiseFloor * highGNoiseFloor
            }
            sigRow.append((v / 6).squareRoot())
        }
        var sigM: [Double] = []
        for i in 0..<3 {
            for j in 0..<3 { sigM.append(sigRow[i] * xtxInv[j][j].squareRoot()) }
        }
        let sigB: [Double] = (0..<3).map { sigRow[$0] * xtxInv[3][3].squareRoot() }

        // the high-g's own gravity fit, and what the alignment says it should have found
        var ownMeans: [Face: [Double]] = [:]
        for (i, face) in Face.allCases.enumerated() { ownMeans[face] = h[i] }
        guard let own = solve(ownMeans, g: g),
              let mInv = invert([Array(m[0..<3]), Array(m[3..<6]), Array(m[6..<9])]) else { return .singular }
        let shifted: [Double] = (0..<3).map { b[$0] + lowG.offset[$0] }
        let zero: [Double] = mul(mInv.flatMap { $0 }, shifted)
        var dOff: [Double] = []
        var dScl: [Double] = []
        var sdOff: [Double] = []
        var sdScl: [Double] = []
        var disagree: [Int] = []
        for i in 0..<3 {
            dOff.append(zero[i] - own.offset[i])
            dScl.append(m[4 * i] - lowG.scale[i] / own.scale[i])
            let up = Face(axis: i, up: true), down = Face(axis: i, up: false)
            let iu = Face.allCases.firstIndex(of: up)!, id = Face.allCases.firstIndex(of: down)!
            let ownVar: Double = sh[iu][i] * sh[iu][i] / n[iu] + sh[id][i] * sh[id][i] / n[id]
                + 2 * highGNoiseFloor * highGNoiseFloor
            let offVar: Double = ownVar / 4 + sigB[i] * sigB[i] + highGAgreeFloorOffset * highGAgreeFloorOffset
            let sclVar: Double = ownVar / (4 * g * g) + sigM[4 * i] * sigM[4 * i]
                + highGAgreeFloorScale * highGAgreeFloorScale
            sdOff.append(offVar.squareRoot())
            sdScl.append(sclVar.squareRoot())
            if !(abs(dOff[i]) <= highGAgreeSigmas * sdOff[i] && abs(dScl[i]) <= highGAgreeSigmas * sdScl[i]) {
                disagree.append(i)
            }
        }

        var problems: [HighGSolution.Problem] = []
        if worst > highGResidualMax { problems.append(.residual) }
        if (0..<3).contains(where: { !(m[4 * $0] >= highGScaleMin && m[4 * $0] <= highGScaleMax) }) {
            problems.append(.scale)
        }
        if crossIndices.contains(where: { abs(m[$0]) > highGCrossMax }) { problems.append(.cross) }
        if b.contains(where: { abs($0) > highGBiasMax }) { problems.append(.bias) }
        return .solved(HighGSolution(matrix: m, bias: b, residualRMS: rms, residualMax: worst,
                                     sigmaMatrix: sigM, sigmaBias: sigB, ownOffset: own.offset,
                                     ownScale: own.scale, impliedZero: zero, diffOffset: dOff, diffScale: dScl,
                                     sigmaDiffOffset: sdOff, sigmaDiffScale: sdScl, disagreeAxes: disagree,
                                     problems: problems))
    }

    /// m * v, m row-major 3x3.
    private static func mul(_ m: [Double], _ v: [Double]) -> [Double] {
        (0..<3).map { i -> Double in
            let a: Double = m[3 * i] * v[0]
            let b: Double = m[3 * i + 1] * v[1]
            let c: Double = m[3 * i + 2] * v[2]
            return a + b + c
        }
    }

    /// The inverse of a small square matrix, column by column; nil when singular.
    private static func invert(_ a: [[Double]]) -> [[Double]]? {
        let n = a.count
        var cols: [[Double]] = []
        for c in 0..<n {
            let e: [Double] = (0..<n).map { $0 == c ? 1 : 0 }
            guard let col = solveLinear(a, e) else { return nil }
            cols.append(col)
        }
        return (0..<n).map { r in (0..<n).map { cols[$0][r] } }
    }

    /// A high-g save's read-back: an alignment stored from the six faces, M
    /// within 1e-6 and b within 1e-5 m/s² of what was sent.
    static func highGReadBackMatches(_ status: AccelCalStatus, matrix: [Float], bias: [Float]) -> Bool {
        guard let h = status.highG, h.isStored, h.source == .sixFaces else { return false }
        let mOK = (0..<9).allSatisfy { abs(Double(h.matrix[$0]) - Double(matrix[$0])) < matrixReadBackTolerance }
        let bOK = (0..<3).allSatisfy { abs(Double(h.bias[$0]) - Double(bias[$0])) < readBackTolerance }
        return mOK && bOK
    }
}

// MARK: - The screen's flow

/// The six-face screen's requests and answers, one at a time: a read (77), a
/// face (78), a save or a clear (76, then 77 to read it back; and with the
/// high-g, 80 and 77 again after it). Pure: the view
/// sends the `Command` each step returns, arms a timer for `timeout` with the
/// `generation` it had, and hands back every 0xD1 frame (`take`) and every
/// timer that fires (`timedOut`).
nonisolated struct AccelCalFlow: Equatable {
    typealias Face = AccelSixFace.Face

    /// What the screen sends.
    enum Command: Equatable {
        case read                // 77
        case measureFace         // 78
        case apply([Float])      // 76: board-frame m/s², all zero clears
        case applyHighG(HighGWrite)  // 80
    }

    /// A write of the high-g's alignment.
    enum HighGWrite: Equatable {
        case save(matrix: [Float], bias: [Float])
        case clear

        var op: UInt8 {
            switch self {
            case .save: return HighGCalApply.opStore
            case .clear: return HighGCalApply.opClear
            }
        }
        var matrix: [Float] {
            if case .save(let m, _) = self { return m }
            return HighGCalApply.identity
        }
        var bias: [Float] {
            if case .save(_, let b) = self { return b }
            return [0, 0, 0]
        }
    }

    /// A write to the flight computer.
    enum Write: Equatable {
        case save([Float])
        case clear

        var sent: [Float] {
            switch self {
            case .save(let o): return o
            case .clear: return [0, 0, 0]
            }
        }
    }

    /// What the screen is waiting for.
    enum Wait: Equatable {
        case none
        /// 77 is out: the status.
        case reading
        /// 78 is out: the window's result.
        case measuring(AccelSixFace.FaceMeasurement)
        /// 76 is out: the flight computer's answer to the apply.
        case applying(Write)
        /// Then 77: the read-back (the apply's own answer is the fallback).
        case readingBack(Write, applyAnswer: AccelCalStatus?)
        /// After it, 80 is out: the answer to the high-g's write.
        case applyingHighG(HighGWrite)
        /// Then 77 again.
        case readingBackHighG(HighGWrite, applyAnswer: AccelCalStatus?)
    }

    /// The last face measurement, as the screen reports it.
    struct FaceReport: Equatable {
        let verdict: AccelSixFace.Verdict
        /// The window's closing frame; nil for a refusal or no answer.
        let frame: AccelCalStatus?
        /// It replaced an earlier capture of the same face.
        let replaced: Bool
    }

    /// The high-g's save or clear, and what its read-back said.
    struct HighGWriteReport: Equatable {
        let write: HighGWrite
        /// nil: no answer to the apply or the read-back.
        let readBack: AccelCalStatus?

        var confirmed: Bool {
            guard let b = readBack else { return false }
            switch write {
            case .save(let m, let bias): return AccelSixFace.highGReadBackMatches(b, matrix: m, bias: bias)
            case .clear: return b.highG.map { !$0.isStored } ?? false
            }
        }
    }

    /// A save or a clear, and what the read-back said.
    struct WriteReport: Equatable {
        let write: Write
        /// nil: no answer to the apply or the read-back.
        let readBack: AccelCalStatus?

        var confirmed: Bool {
            guard let b = readBack else { return false }
            switch write {
            case .save(let o): return AccelSixFace.readBackMatches(b, sent: o)
            case .clear: return !b.isStored
            }
        }
    }

    private(set) var wait: Wait = .none
    /// Bumped by every wait; a timer armed for an older one does nothing.
    private(set) var generation = 0
    /// The last read's answer: nil until one finishes, false for no answer.
    private(set) var readAnswered: Bool?
    private(set) var captures = AccelSixFace.Captures()
    private(set) var lastFace: FaceReport?
    private(set) var lastWrite: WriteReport?
    /// The high-g's part of the last save or clear; nil when it had none.
    private(set) var lastHighGWrite: HighGWriteReport?
    /// The high-g write that follows the low-g's, while that is out.
    private(set) var pendingHighG: HighGWrite?
    /// The face whose placement help shows under the checklist: the last row
    /// tapped, until a measurement starts.
    private(set) var placementFace: Face?

    var isBusy: Bool { wait != .none }
    var isReading: Bool { wait == .reading }
    var isMeasuring: Bool {
        if case .measuring = wait { return true }
        return false
    }
    /// The window is open: the flight computer is averaging.
    var windowOpen: Bool {
        if case .measuring(let m) = wait { return m.windowOpen }
        return false
    }
    /// The write in progress, if any.
    var writing: Write? {
        switch wait {
        case .applying(let w), .readingBack(let w, _): return w
        default: return nil
        }
    }
    /// The high-g write in progress, if any.
    var writingHighG: HighGWrite? {
        switch wait {
        case .applyingHighG(let w), .readingBackHighG(let w, _): return w
        default: return nil
        }
    }

    /// The current wait's timeout, s (the reference script's: 10 s for a face).
    var timeout: TimeInterval {
        switch wait {
        case .none: return 0
        case .reading, .readingBack, .readingBackHighG: return 6
        case .measuring: return 10
        case .applying, .applyingHighG: return 8
        }
    }

    // MARK: Requests (each nil while another is out)

    mutating func read() -> Command? {
        guard !isBusy else { return nil }
        readAnswered = nil
        begin(.reading)
        return .read
    }

    mutating func measure() -> Command? {
        guard !isBusy else { return nil }
        lastWrite = nil   // the faces are changing under it
        lastHighGWrite = nil
        placementFace = nil
        begin(.measuring(AccelSixFace.FaceMeasurement()))
        return .measureFace
    }

    /// A checklist row was tapped: show how to set the board on that face.
    /// The rows only explain; the measurement still names its own face.
    mutating func showPlacement(_ face: Face) {
        placementFace = face
    }

    /// Save the solved offset; nil unless all six faces are in and it is
    /// within the flight computer's limit. The high-g's alignment follows it
    /// when that is within its limits too.
    mutating func save() -> Command? {
        guard !isBusy, let s = captures.solution, s.canSave else { return nil }
        let o = s.payload
        lastWrite = nil
        lastHighGWrite = nil
        pendingHighG = nil
        if case .solved(let h)? = captures.highG, h.canSave {
            pendingHighG = .save(matrix: h.payloadMatrix, bias: h.payloadBias)
        }
        begin(.applying(.save(o)))
        return .apply(o)
    }

    /// Clear the stored offset, and the high-g's alignment after it when
    /// `highG` (one is stored: an older flight computer would not answer 80).
    mutating func clear(highG: Bool = false) -> Command? {
        guard !isBusy else { return nil }
        lastWrite = nil
        lastHighGWrite = nil
        pendingHighG = highG ? .clear : nil
        begin(.applying(.clear))
        return .apply(Write.clear.sent)
    }

    /// After a write that got no answer: read it back again.
    mutating func readBackAgain() -> Command? {
        guard !isBusy else { return nil }
        if let w = lastWrite, w.readBack == nil {
            lastWrite = nil
            begin(.readingBack(w.write, applyAnswer: nil))
            return .read
        }
        if let w = lastHighGWrite, w.readBack == nil {
            lastHighGWrite = nil
            begin(.readingBackHighG(w.write, applyAnswer: nil))
            return .read
        }
        return nil
    }

    /// Drop the captured faces.
    mutating func startOver() {
        guard !isBusy else { return }
        captures = AccelSixFace.Captures()
        lastFace = nil
        lastWrite = nil
        lastHighGWrite = nil
    }

    // MARK: Answers

    /// A 0xD1 frame. Returns the next command when the flow moves on.
    mutating func take(_ frame: AccelCalStatus) -> Command? {
        switch wait {
        case .none:
            return nil
        case .reading:
            readAnswered = true
            wait = .none
            return nil
        case .measuring(var m):
            guard let result = m.take(frame) else {
                wait = .measuring(m)
                return nil
            }
            wait = .none
            finish(result)
            return nil
        case .applying(let w):
            begin(.readingBack(w, applyAnswer: frame))
            return .read
        case .readingBack(let w, _):
            lastWrite = WriteReport(write: w, readBack: frame)
            return nextHighG()
        case .applyingHighG(let w):
            begin(.readingBackHighG(w, applyAnswer: frame))
            return .read
        case .readingBackHighG(let w, _):
            lastHighGWrite = HighGWriteReport(write: w, readBack: frame)
            wait = .none
            return nil
        }
    }

    /// The low-g's write is done: the high-g's, if one follows.
    private mutating func nextHighG() -> Command? {
        guard let w = pendingHighG else {
            wait = .none
            return nil
        }
        pendingHighG = nil
        begin(.applyingHighG(w))
        return .applyHighG(w)
    }

    /// A timer armed at `generation` fired.
    mutating func timedOut(_ generation: Int) -> Command? {
        guard generation == self.generation else { return nil }
        switch wait {
        case .none:
            return nil
        case .reading:
            readAnswered = false
            wait = .none
            return nil
        case .measuring:
            lastFace = FaceReport(verdict: .noAnswer, frame: nil, replaced: false)
            wait = .none
            return nil
        case .applying(let w):
            // Read it back anyway: the answer may be what was lost.
            begin(.readingBack(w, applyAnswer: nil))
            return .read
        case .readingBack(let w, let applyAnswer):
            lastWrite = WriteReport(write: w, readBack: applyAnswer)
            return nextHighG()
        case .applyingHighG(let w):
            begin(.readingBackHighG(w, applyAnswer: nil))
            return .read
        case .readingBackHighG(let w, let applyAnswer):
            lastHighGWrite = HighGWriteReport(write: w, readBack: applyAnswer)
            wait = .none
            return nil
        }
    }

    private mutating func begin(_ next: Wait) {
        wait = next
        generation += 1
    }

    private mutating func finish(_ result: AccelSixFace.FaceMeasurement.Result) {
        switch result {
        case .refused:
            lastFace = FaceReport(verdict: .refused, frame: nil, replaced: false)
        case .finished(let frame):
            let verdict = AccelSixFace.verdict(for: frame)
            var replaced = false
            if case .accepted(let face, let tilt) = verdict {
                replaced = captures.record(face, AccelSixFace.Capture(frame, tiltDeg: tilt))
            }
            lastFace = FaceReport(verdict: verdict, frame: frame, replaced: replaced)
        }
    }
}

// MARK: - What the screen says

nonisolated extension AccelCalFlow.FaceReport {

    /// The result line under the Measure button.
    var text: String {
        switch verdict {
        case .accepted(let face, let tilt):
            let tiltText = String(format: "%.1f", tilt)
            let again = replaced ? " — replaced the earlier one" : ""
            return "\(face.name), \(tiltText)° off vertical\(again)"
        case .moved:
            return "It moved, or it is not one g — hold it still and measure again"
        case .tilted(let face, let tilt):
            let tiltText = String(format: "%.0f", tilt)
            return "\(face.name) is \(tiltText)° off vertical — set it squarer and measure again"
        case .refused:
            return "The flight computer refused: it measures no face in flight, after a flight, "
                + "in a sim or during a magnetometer calibration"
        case .noAnswer:
            return "No answer from the flight computer — measure again"
        }
    }

    var isAccepted: Bool {
        if case .accepted = verdict { return true }
        return false
    }

    /// The window's numbers: "(−0.083, +0.115, +9.821) m/s², SD max 1.4 mg, 2880 samples".
    var detail: String? {
        guard let f = frame else { return nil }
        let m = f.faceMean.map(Double.init)
        let sdMaxMg = (f.faceSD.map(Double.init).max() ?? 0) / AccelSixFace.g0 * 1000
        let numbers = String(format: "(%+.3f, %+.3f, %+.3f) m/s², SD max %.1f mg",
                             m[0], m[1], m[2], sdMaxMg)
        return "\(numbers), \(f.faceN) samples"
    }
}

nonisolated extension AccelCalFlow.WriteReport {

    var text: String {
        switch (write, readBack) {
        case (_, nil):
            return "No answer from the flight computer — read it back to check"
        case (.save, _?):
            return confirmed ? "Saved and read back" : "The read-back does not match what was sent"
        case (.clear, _?):
            return confirmed ? "Cleared: no offset stored" : "The flight computer still has an offset stored"
        }
    }
}

nonisolated extension AccelCalFlow.HighGWriteReport {

    var text: String {
        switch (write, readBack) {
        case (_, nil):
            return "No answer about the high-g from the flight computer — read it back to check"
        case (_, let b?) where b.highG == nil:
            return "The answer has no high-g in it: update the flight computer and the out computer"
        case (.save, _?):
            return confirmed ? "High-g alignment saved and read back"
                : "The high-g read-back does not match what was sent"
        case (.clear, _?):
            return confirmed ? "Cleared: no high-g alignment stored"
                : "The flight computer still has a high-g alignment stored"
        }
    }
}

nonisolated extension AccelSixFace.HighGSolution {

    /// "+0.812 % ± 0.041": the scale's difference from 1 and its 1-sigma.
    func scaleText(_ axis: Int) -> String {
        String(format: "%+.3f %% ± %.3f", (matrix[4 * axis] - 1) * 100, sigmaMatrix[4 * axis] * 100)
    }

    /// "+66.4 mg ± 0.24": the offset against the low-g.
    func offsetText(_ axis: Int) -> String {
        String(format: "%+.1f mg ± %.2f", bias[axis] / AccelSixFace.g0 * 1000,
               sigmaBias[axis] / AccelSixFace.g0 * 1000)
    }

    var crossText: String {
        String(format: "Cross-axis: up to %.2f° (± %.3f°)", crossDeg, sigmaCrossDeg)
    }

    var residualText: String {
        String(format: "Fit to the low-g: %.2f mg rms, %.2f mg at most (limit %.0f mg)",
               residualRMS / AccelSixFace.g0 * 1000, residualMax / AccelSixFace.g0 * 1000,
               AccelSixFace.highGResidualMax / AccelSixFace.g0 * 1000)
    }

    /// The two fits' agreement: the high-g's own gravity fit against what the
    /// alignment to the low-g implies.
    var agreementText: String {
        guard let first = disagreeAxes.first else {
            return "Its own gravity fit agrees with the alignment to the low-g within their noise."
        }
        let axes = disagreeAxes.map { AccelSixFace.axisNames[$0] }.joined(separator: ", ")
        let off = abs(diffOffset[first]) / AccelSixFace.g0 * 1000
        let lim = AccelSixFace.highGAgreeSigmas * sigmaDiffOffset[first] / AccelSixFace.g0 * 1000
        let scl = abs(diffScale[first]) * 100
        let sLim = AccelSixFace.highGAgreeSigmas * sigmaDiffScale[first] * 100
        let numbers = String(format: "%.1f mg (3σ %.1f) and %.3f %% (3σ %.3f)", off, lim, scl, sLim)
        return "Its own gravity fit differs on \(axes) by \(numbers): the alignment to the low-g, the "
            + "bench's reference, is what gets saved."
    }

    var problemText: String? {
        guard let p = problems.first else { return nil }
        switch p {
        case .residual:
            return "A face fits the low-g worse than 3 mg — did it move, or warm up between faces? Measure the faces again."
        case .scale:
            return "A scale is outside 0.97–1.03."
        case .cross:
            return "A cross-axis term is past 0.03 (1.7°)."
        case .bias:
            return "An offset is past 11 m/s²."
        }
    }
}

nonisolated extension AccelSixFace.HighGResult {

    /// The line when there is nothing to show: no high-g with the faces, or
    /// no fit.
    var unavailableText: String? {
        switch self {
        case .noReading:
            return "The faces came without the high-g reading. Update the flight computer and the out "
                + "computer to calibrate the high-g; the low-g offset saves as before."
        case .singular:
            return "The high-g fit failed: are the six faces six different sides?"
        case .solved:
            return nil
        }
    }
}

nonisolated extension AccelCalStatus.HighG {

    /// "X +0.812 %, Y -0.664 %, Z +1.224 %": the stored alignment's scales.
    var scalesText: String {
        (0..<3).map { String(format: "\(AccelSixFace.axisNames[$0]) %+.3f %%", (Double(matrix[4 * $0]) - 1) * 100) }
            .joined(separator: ", ")
    }

    /// "X -66.4 mg, Y +150.9 mg, Z -31.9 mg": the bias in use.
    var biasText: String {
        (0..<3).map { String(format: "\(AccelSixFace.axisNames[$0]) %+.1f mg",
                             Double(bias[$0]) / AccelSixFace.g0 * 1000) }
            .joined(separator: ", ")
    }

    var sourceText: String {
        switch source {
        case .padCal?: return "measured by the pad calibration"
        case .sixFaces?: return "from the six faces"
        case .none?, nil: return "source not recorded"
        }
    }

    var alignmentText: String {
        switch (isStored, isApplied) {
        case (true, true): return "High-g alignment in use: \(scalesText)"
        case (true, false): return "High-g alignment stored, but not in use while a sim runs"
        case (false, _): return "No high-g alignment stored"
        }
    }
}

nonisolated extension AccelSixFace.Solution {

    /// "+0.123 %": the scale's difference from 1.
    func scaleText(_ axis: Int) -> String {
        String(format: "%+.3f %%", (scale[axis] - 1) * 100)
    }

    /// The line under the offsets.
    static let solvedTogetherText =
        "All six faces are solved together, so a face a few degrees off square does not bias the offset."

    static let redoText = "Redo the faces: was one of them not still, or the wrong way up?"

    /// Why the offset cannot be saved, if it cannot: past the flight
    /// computer's limit, or a fit that did not converge or has a scale outside
    /// 0.97...1.03.
    var cannotSaveText: String? {
        guard !canSave else { return nil }
        let axes = tooLargeAxes.map { AccelSixFace.axisNames[$0] }.joined(separator: ", ")
        guard converged, !axes.isEmpty else { return Self.redoText }
        return "Offset past 1.0 m/s² on \(axes) — the flight computer would refuse it. " + Self.redoText
    }
}

nonisolated extension AccelSixFace.LiveFace {

    /// The live line at the top of the checklist. The tilt is in whole
    /// degrees, and left out when the mounting cannot tell it.
    var text: String {
        switch self {
        case .on(let face, let tilt?):
            let deg = String(format: "%.0f", tilt)
            return "On now: \(face.name) (\(face.hint)), \(deg)° off vertical"
        case .on(let face, nil):
            return "On now: \(face.name) (\(face.hint))"
        case .askew(let face, let tilt):
            let deg = String(format: "%.0f", tilt)
            return "Set it squarer: \(face.name) is \(deg)° off vertical"
        case .moving:
            return "Hold it still"
        case .waiting:
            return "Waiting for the rocket's reading…"
        }
    }
}

nonisolated extension AccelSixFace.Face {

    /// The line a tapped checklist row shows under the checklist.
    var placementText: String { "To measure \(name): \(placement)" }
}

// MARK: - The live face on a device

extension BLEDevice {

    /// Which face the board is on now, from the newest telemetry frame: its
    /// lx/ly/lz (rocket frame) turned back through the mounting they are in.
    /// `.waiting` without a link, without a full reading, or without a
    /// mounting code (`liveMounting`); no tilt in auto-exact.
    var accelLiveFace: AccelSixFace.LiveFace {
        guard isConnected,
              let x = telemetry.low_g_x, let y = telemetry.low_g_y, let z = telemetry.low_g_z
        else { return .waiting }
        let rocket: [Double] = [Double(x), Double(y), Double(z)]
        let mounting = AccelSixFace.liveMounting(imo: telemetry.imu_orient_packed,
                                                 reportedCode: imuOrientationCode,
                                                 reportedMode: imuOrientationMode)
        return AccelSixFace.liveFace(rocket: rocket, mounting: mounting)
    }
}
