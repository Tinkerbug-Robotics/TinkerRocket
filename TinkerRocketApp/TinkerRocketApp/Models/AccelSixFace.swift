//
//  AccelSixFace.swift
//  TinkerRocketApp
//
//  The low-g accelerometer's six-face hand calibration (#1629), without the
//  BLE and the timers: which face a reading is, the solve, the limits, the
//  rule that picks a face window's result out of the 0xD1 frames, and the
//  six-face screen's request/answer flow. AccelCalibrationView sends what
//  AccelCalFlow asks for and feeds it the frames and the timeouts.
//
//  The reference is tools/accel_cal_six_face.py, which this follows step for
//  step. On a still pad the navigation filter levels on gravity, and an
//  accelerometer offset there looks exactly like a tilt. Turn each board axis
//  straight up and then straight down: gravity flips sign on that axis and the
//  offset does not, so
//
//      offset_i = (mean with axis i up + mean with axis i down) / 2
//
//  A face a few degrees off its axis reads g cos(tilt) both ways and cancels.
//  The board frame is +X forward, +Y left, +Z up out of the component side,
//  whatever the board's mounting in the rocket.
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
    /// mg: a face further than this off one g after the correction gets a note.
    static let residualNoteMg = 5.0
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
        let sdMax: Double
        let samples: Int
        let tiltDeg: Double

        init(_ frame: AccelCalStatus, tiltDeg: Double) {
            mean = frame.faceMean.map(Double.init)
            sdMax = frame.faceSD.map(Double.init).max() ?? 0
            samples = Int(frame.faceN)
            self.tiltDeg = tiltDeg
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
    }

    // MARK: - The solve

    nonisolated struct Solution: Equatable {
        /// m/s², board frame x, y, z: what gets saved.
        let offset: [Double]
        /// Per axis, (up − down) / 2g: shown for information, never sent.
        let scale: [Double]
        /// Each face after the correction, |(mean − offset) / scale| − g, m/s².
        let residual: [Face: Double]

        /// The axes past 1.0 m/s², which the flight computer would refuse.
        var tooLargeAxes: [Int] { (0..<3).filter { abs(offset[$0]) > AccelSixFace.offsetMax } }
        var canSave: Bool { tooLargeAxes.isEmpty }

        var worstResidual: Double { residual.values.map(abs).max() ?? 0 }
        /// A face more than 5 mg off one g: the offset is still good if the
        /// faces were squarish, so this only adds a note.
        var needsResidualNote: Bool { worstResidual > AccelSixFace.residualNoteMg / 1000 * AccelSixFace.g0 }

        /// The command-76 values.
        var payload: [Float] { offset.map(Float.init) }
    }

    /// Solve the offset and scale from the six faces' means (nil until all
    /// six are there).
    static func solve(_ means: [Face: [Double]], g: Double = g0) -> Solution? {
        var offset = [0.0, 0.0, 0.0], scale = [0.0, 0.0, 0.0]
        for axis in 0..<3 {
            guard let up = means[Face(axis: axis, up: true)],
                  let down = means[Face(axis: axis, up: false)] else { return nil }
            offset[axis] = 0.5 * (up[axis] + down[axis])
            scale[axis] = (up[axis] - down[axis]) / (2 * g)
        }
        var residual: [Face: Double] = [:]
        for (face, mean) in means {
            let c = (0..<3).map { (mean[$0] - offset[$0]) / scale[$0] }
            residual[face] = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).squareRoot() - g
        }
        return Solution(offset: offset, scale: scale, residual: residual)
    }

    /// A save's read-back: stored, and each component within 1e-5 m/s² of
    /// what was sent.
    static func readBackMatches(_ status: AccelCalStatus, sent: [Float]) -> Bool {
        status.isStored && (0..<3).allSatisfy {
            abs(Double(status.offset[$0]) - Double(sent[$0])) < readBackTolerance
        }
    }
}

// MARK: - The screen's flow

/// The six-face screen's requests and answers, one at a time: a read (77), a
/// face (78), a save or a clear (76, then 77 to read it back). Pure: the view
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
    }

    /// The last face measurement, as the screen reports it.
    struct FaceReport: Equatable {
        let verdict: AccelSixFace.Verdict
        /// The window's closing frame; nil for a refusal or no answer.
        let frame: AccelCalStatus?
        /// It replaced an earlier capture of the same face.
        let replaced: Bool
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

    /// The current wait's timeout, s (the reference script's: 10 s for a face).
    var timeout: TimeInterval {
        switch wait {
        case .none: return 0
        case .reading, .readingBack: return 6
        case .measuring: return 10
        case .applying: return 8
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
        begin(.measuring(AccelSixFace.FaceMeasurement()))
        return .measureFace
    }

    /// Save the solved offset; nil unless all six faces are in and it is
    /// within the flight computer's limit.
    mutating func save() -> Command? {
        guard !isBusy, let s = captures.solution, s.canSave else { return nil }
        let o = s.payload
        lastWrite = nil
        begin(.applying(.save(o)))
        return .apply(o)
    }

    mutating func clear() -> Command? {
        guard !isBusy else { return nil }
        lastWrite = nil
        begin(.applying(.clear))
        return .apply(Write.clear.sent)
    }

    /// After a write that got no answer: read it back again.
    mutating func readBackAgain() -> Command? {
        guard !isBusy, let w = lastWrite, w.readBack == nil else { return nil }
        lastWrite = nil
        begin(.readingBack(w.write, applyAnswer: nil))
        return .read
    }

    /// Drop the captured faces.
    mutating func startOver() {
        guard !isBusy else { return }
        captures = AccelSixFace.Captures()
        lastFace = nil
        lastWrite = nil
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
            wait = .none
            return nil
        }
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

nonisolated extension AccelSixFace.Solution {

    /// "+0.123 %": the scale's difference from 1.
    func scaleText(_ axis: Int) -> String {
        String(format: "%+.3f %%", (scale[axis] - 1) * 100)
    }

    /// Why the offset cannot be saved, if it cannot.
    var tooLargeText: String? {
        let axes = tooLargeAxes.map { AccelSixFace.axisNames[$0] }.joined(separator: ", ")
        guard !axes.isEmpty else { return nil }
        return "Offset past 1.0 m/s² on \(axes) — the flight computer would refuse it. "
            + "Redo the faces: was one of them not still, or the wrong way up?"
    }

    /// The note for a face far off one g after the correction.
    var residualNoteText: String? {
        guard needsResidualNote else { return nil }
        let worstMg = String(format: "%.1f", worstResidual / AccelSixFace.g0 * 1000)
        return "A face is \(worstMg) mg off one g after the correction (over 5). The offset "
            + "is still good if the faces were squarish; the scale is not used."
    }
}
