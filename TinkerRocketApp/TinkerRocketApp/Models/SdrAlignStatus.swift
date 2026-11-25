//
//  SdrAlignStatus.swift
//  TinkerRocketApp
//
//  The gnss-sdr-dev board's IMU on its flight board: how it is mounted on
//  the flight computer and its accelerometer's calibration. The board finds
//  both from the flight computer's rests (its mean reading over each still
//  second, which the board pairs with its own): two poses apart give the
//  whole rotation, the six faces every axis's scale and bias. It keeps the
//  result in its own storage and reports it; the flight computer relays the
//  report on the file_ops characteristic behind a 0xD2 discriminator, once a
//  second on the ground and at each change, its poses turned into the flight
//  computer's own IMU frame -- the six-face screen's board frame, so a pose
//  is matched to the face it was held on. Command 83 sends the board a save,
//  a clear, a restart or a report ([op, id]); the status echoes the id.
//  Wire layout mirrors SdrAlignStatusData in RocketComputerTypes.h -- keep
//  both in sync. The six-face screen (AccelCalibrationView) is the reader.
//
//  Wire (111 bytes, little-endian, packed):
//    [0]  u8 ver          SDR_ALIGN_STATUS_VER (1)
//    [1]  u8 flags        SDRA_*: bit 0 the board is on J1, 1 its report held
//                         (under 3 s old), 2 it can place a rest (GPS time,
//                         or the rest's arrival), 3 the flight computer's
//                         rests go stamped (its clock line; else unstamped),
//                         4 the board saves by itself this boot (a command
//                         turns that off), 5 it solved against the body, 6
//                         the flight computer was still over its last second
//    [2]  u8 imu          the board's IMU: 0 off, 1 watching, 2 aiding
//    [3]  u8 have         0 none, 1 tilt (one pose), 2 whole
//    [4]  u8 from         0 none, 1 this boot's rests, 2 the saved one,
//                         3 the saved one with this boot's rests
//    [5]  u8 saved        0 nothing saved, 1 saved = in use, 2 saved, differs
//    [6]  u8 n_pose       0-8
//    [7]  u8 cal_axes     bit j: the board's axis j calibrated
//    [8]  u8 last_rest    what the board did with the last rest (RestFate)
//    [9]  u8 gen          the board's solution's generation
//    [10] u8 n_rest       rests the board received, mod 256
//    [11] u8 n_pair       and paired, mod 256
//    [12] u8 cmd_id       the board's echo: the last command it acted on,
//    [13] u8 cmd_op       its op
//    [14] u8 cmd_res      and its result (CommandResult)
//    [15] u8 fc_cmd_state the flight computer's part (CommandState)
//    [16] u8 fc_cmd_op
//    [17] u8 fc_cmd_id    the app's id for it
//    [18] u8 rests_sent   the flight computer's rests sent, mod 256
//    [19..20] u16 rest_age_ds   since its last rest, 0.1 s (65535 none)
//    [21..22] u16 board_age_ds  since the board's last report, 0.1 s
//    [23..34] f32 spread_deg, resid_deg, cal_rms_mps2
//    [35..50] f32 q[4]    U10 to the flight computer's IMU, scalar first
//    [51..62] f32 scale[3]  the board's axes: read = s * true + b
//    [63..74] f32 bias_mps2[3]
//    [75..98] i8 pose_dir[8][3]  each pose's direction, 1/127
//    [99..106] u8 pose_pairs[8]
//    [107..110] f32 moved_deg  with the saved rotation and this boot's rests
//                         (from 3): the rail pose off the saved rotation
//

import Foundation

nonisolated struct SdrAlignStatus: Equatable {

    static let payloadLength = 111
    static let poses = 8

    // SDRA_* flag bits.
    static let flagBoard: UInt8    = 0x01
    static let flagReport: UInt8   = 0x02
    static let flagTime: UInt8     = 0x04
    static let flagLine: UInt8     = 0x08
    static let flagAutosave: UInt8 = 0x10
    static let flagBody: UInt8     = 0x20
    static let flagStill: UInt8    = 0x40

    enum Have: UInt8 { case none = 0, tilt = 1, whole = 2 }
    enum Saved: UInt8 { case none = 0, same = 1, differs = 2 }
    /// What the board did with the last rest.
    enum RestFate: UInt8 {
        case none = 0, paired = 1, moving = 2, short = 3, late = 4, notOneG = 5, noTime = 6, imuOff = 7
    }
    /// The board's answer to a command.
    enum CommandResult: UInt8 { case none = 0, done = 1, nothingToSave = 2, refused = 3, storageFailed = 4, unknownOp = 5 }
    /// The flight computer's part in the app's last command.
    enum CommandState: UInt8 { case idle = 0, sending = 1, echoed = 2, noAnswer = 3, refused = 4 }

    /// One pose the board holds: the flight computer's reading's direction
    /// on it (its IMU frame, unit length) and the rests paired into it.
    nonisolated struct Pose: Equatable {
        let dir: [Double]
        let pairs: Int
    }

    let ver: UInt8
    let flags: UInt8
    let imu: UInt8
    let haveRaw: UInt8
    let from: UInt8
    let savedRaw: UInt8
    let calAxes: UInt8
    let lastRestRaw: UInt8
    let gen: UInt8
    let nRest: UInt8
    let nPair: UInt8
    let cmdId: UInt8
    let cmdOp: UInt8
    let cmdResRaw: UInt8
    let fcCmdStateRaw: UInt8
    let fcCmdOp: UInt8
    let fcCmdId: UInt8
    let restsSent: UInt8
    let restAgeDs: UInt16
    let boardAgeDs: UInt16
    let spreadDeg: Float
    let residDeg: Float
    let calRms: Float
    let q: [Float]
    let scale: [Float]
    let bias: [Float]
    let poses: [Pose]
    /// From 3 only: the rail pose off the saved rotation, deg (5.1: that or more).
    let movedDeg: Float

    var boardPresent: Bool { flags & Self.flagBoard != 0 }
    var reportHeld: Bool { flags & Self.flagReport != 0 }
    /// The board can place a rest: GPS time, or the rest's arrival.
    var canPlaceRest: Bool { flags & Self.flagTime != 0 }
    /// The flight computer's rests go stamped (its clock line); else unstamped.
    var restsStamped: Bool { flags & Self.flagLine != 0 }
    var autosave: Bool { flags & Self.flagAutosave != 0 }
    var fcStill: Bool { flags & Self.flagStill != 0 }
    var have: Have { Have(rawValue: haveRaw) ?? .none }
    var saved: Saved { Saved(rawValue: savedRaw) ?? .none }
    var lastRest: RestFate? { RestFate(rawValue: lastRestRaw) }
    var cmdResult: CommandResult? { CommandResult(rawValue: cmdResRaw) }
    var fcCmdState: CommandState? { CommandState(rawValue: fcCmdStateRaw) }
    /// Every accelerometer axis of the board's is calibrated.
    var allAxesCalibrated: Bool { calAxes & 0x07 == 0x07 }
    func axisCalibrated(_ j: Int) -> Bool { calAxes & UInt8(1 << j) != 0 }

    /// Decode the payload *after* the 0xD2 discriminator byte.
    static func decode(_ bytes: [UInt8]) -> SdrAlignStatus? {
        guard bytes.count >= payloadLength else { return nil }
        func u16(_ o: Int) -> UInt16 { UInt16(bytes[o]) | UInt16(bytes[o + 1]) << 8 }
        func u32(_ o: Int) -> UInt32 { UInt32(u16(o)) | UInt32(u16(o + 2)) << 16 }
        func f32(_ o: Int) -> Float { Float(bitPattern: u32(o)) }
        func f32s(_ o: Int, _ n: Int) -> [Float] { (0..<n).map { f32(o + 4 * $0) } }
        let n = min(Int(bytes[6]), poses)
        let list: [Pose] = (0..<n).map { i in
            let d = (0..<3).map { Double(Int8(bitPattern: bytes[75 + 3 * i + $0])) / 127 }
            return Pose(dir: d, pairs: Int(bytes[99 + i]))
        }
        return SdrAlignStatus(ver: bytes[0], flags: bytes[1], imu: bytes[2], haveRaw: bytes[3], from: bytes[4],
                              savedRaw: bytes[5], calAxes: bytes[7], lastRestRaw: bytes[8], gen: bytes[9],
                              nRest: bytes[10], nPair: bytes[11], cmdId: bytes[12], cmdOp: bytes[13],
                              cmdResRaw: bytes[14], fcCmdStateRaw: bytes[15], fcCmdOp: bytes[16], fcCmdId: bytes[17],
                              restsSent: bytes[18], restAgeDs: u16(19), boardAgeDs: u16(21),
                              spreadDeg: f32(23), residDeg: f32(27), calRms: f32(31),
                              q: f32s(35, 4), scale: f32s(51, 3), bias: f32s(63, 3), poses: list,
                              movedDeg: f32(107))
    }
}

/// SdrAlignCmdData, the command-83 payload: [op, id].
nonisolated enum SdrAlignCommand {
    static let opSave: UInt8 = 1
    static let opClear: UInt8 = 2
    static let opRestart: UInt8 = 3
    static let opReport: UInt8 = 4

    static func payload(op: UInt8, id: UInt8) -> Data { Data([op, id]) }
}

// MARK: - The faces

nonisolated extension SdrAlignStatus {

    /// A pose further than this from a face's axis is no face's.
    static let faceToleranceDeg = 20.0

    /// The face a pose was held on: its reading's axis nearest vertical,
    /// within `faceToleranceDeg`.
    static func face(of pose: Pose) -> AccelSixFace.Face? {
        let (face, tilt) = AccelSixFace.identify(pose.dir)
        return tilt <= faceToleranceDeg ? face : nil
    }

    /// The rests the board paired on each face (faces with none left out).
    var pairsByFace: [AccelSixFace.Face: Int] {
        var out: [AccelSixFace.Face: Int] = [:]
        for p in poses {
            if let f = Self.face(of: p) { out[f, default: 0] += p.pairs }
        }
        return out
    }

    /// How far the board's IMU is turned from the flight computer's, deg.
    var rotationDeg: Double {
        let w = Double(min(1, max(-1, abs(q[0]))))
        return 2 * acos(w) * 180 / .pi
    }

    /// Whole, every axis calibrated: nothing more for the six faces to give.
    var isComplete: Bool { have == .whole && allAxesCalibrated }

    // The board's own auto-save rule's quality, which the app's save keeps
    // too: six poses at least, none more than 1 deg off the rotation, the
    // calibration's fit within 0.05 m/s² rms.
    static let savePosesMin = 6
    static let saveResidMaxDeg: Float = 1.0
    static let saveRmsMax: Float = 0.05

    /// Complete and good enough to save by itself.
    var isGoodToSave: Bool {
        isComplete && poses.count >= Self.savePosesMin && residDeg <= Self.saveResidMaxDeg && calRms <= Self.saveRmsMax
    }

    /// The rail pose this far off the saved rotation says the board moved
    /// since its faces were measured.
    static let movedWarnDeg: Float = 1.0

    /// In use: the saved rotation with this boot's rests (its tilt checked on
    /// the rail).
    var onSavedRotation: Bool { from == 3 }
}

// MARK: - The board's part of the six-face screen

/// The board's part of the six-face flow, without the BLE and the timers:
/// its commands one at a time and their answers, and when to save. The six
/// faces the flight computer measures are the board's poses too: it pairs
/// each rest the flight computer sends while the rocket is held still. The
/// first face measured starts the board afresh (restart: its saved mounting
/// stays until the new one is saved; the command also turns the board's own
/// auto-save off for that boot, so the app saves). Each time the flight
/// computer's offset is saved and read back, the board's mounting is saved
/// too, once it is whole with every axis calibrated and passes the board's
/// own auto-save quality (`isGoodToSave`; at once, or when the board's last
/// pair comes in after it) -- unless the flight computer's offset moved by more
/// than `offsetMoveMax` since the board began, when the board's calibration
/// was made against the old one: then nothing is saved, and the next face
/// starts the board afresh for the faces once more. Pure: the view sends the
/// `Command` each step returns, arms a timer for `timeout` with the
/// `generation` it had, and hands back every 0xD2 status (`take`) and every
/// timer that fires (`timedOut`).
nonisolated struct BoardAlignFlow: Equatable {

    /// m/s² on any axis (2 mg): a larger move of the flight computer's
    /// offset after the board began calls for the faces again.
    static let offsetMoveMax: Float = 0.02
    /// s: the flight computer tries the board for 3 s, then says so.
    static let timeout: TimeInterval = 6

    /// What the screen sends: command 83.
    struct Command: Equatable {
        let op: UInt8
        let id: UInt8
    }

    /// How the last command ended.
    enum Outcome: Equatable {
        case done(op: UInt8)
        /// The board answered but did not do it.
        case notDone(op: UInt8, SdrAlignStatus.CommandResult?)
        /// The flight computer would not send it: in flight, after a flight,
        /// during a magnetometer calibration, or no board on J1.
        case refused(op: UInt8)
        /// The flight computer sent it for 3 s and the board never echoed it.
        case noAnswer(op: UInt8)
        /// Nothing came back from the flight computer at all.
        case noStatus(op: UInt8)
    }

    /// Why the faces must be turned once more for the board.
    struct Again: Equatable {
        /// The flight computer's offset's move, m/s², per axis.
        let move: [Float]
    }

    private(set) var status: SdrAlignStatus?
    private(set) var pending: Command?
    private(set) var generation = 0
    private(set) var lastOutcome: Outcome?
    /// The board was started afresh for these faces (restart echoed or sent).
    private(set) var started = false
    /// The flight computer's offset in use when the board began.
    private(set) var offsetAtStart: [Float]?
    /// The faces must be turned again for the board.
    private(set) var again: Again?
    /// The app's last id: the next command takes the next one (1-255).
    private(set) var lastId: UInt8 = 0
    /// The flight computer's offset saved since the board's last save: the
    /// board's save is due once it is complete.
    private(set) var savedOffset: [Float]?

    var isBusy: Bool { pending != nil }
    /// The board is on J1: everything below applies.
    var boardPresent: Bool { status?.boardPresent == true }

    /// The board's state, for the screen.
    var have: SdrAlignStatus.Have { status?.have ?? .none }

    // MARK: Requests (each nil while another is out)

    mutating func request(_ op: UInt8) -> Command? {
        guard !isBusy else { return nil }
        lastId = lastId == 255 ? 1 : lastId + 1
        let c = Command(op: op, id: lastId)
        pending = c
        lastOutcome = nil
        generation += 1
        return c
    }

    /// A face is about to be measured. The first one of a fresh set (`fresh`:
    /// no face captured yet, or the faces asked for again) starts the board
    /// afresh, remembering the flight computer's offset in use then.
    mutating func faceStarting(fresh: Bool, offsetInUse: [Float]) -> Command? {
        guard boardPresent, fresh || !started else { return nil }
        guard let c = request(SdrAlignCommand.opRestart) else { return nil }
        started = true
        offsetAtStart = offsetInUse
        again = nil
        savedOffset = nil
        return c
    }

    /// The flight computer's offset was saved and read back: save the board's
    /// mounting when it is complete, or ask for the faces again.
    mutating func flightComputerSaved(offset: [Float]) -> Command? {
        guard boardPresent, started else { return nil }
        if let o0 = offsetAtStart {
            let move = zip(offset, o0).map { $0 - $1 }
            if move.contains(where: { abs($0) > Self.offsetMoveMax }) {
                again = Again(move: move)
                started = false   // the next face starts the board afresh
                savedOffset = nil
                return nil
            }
        }
        savedOffset = offset
        return saveIfDue()
    }

    /// The board's save, when the flight computer's is in and the board's
    /// mounting complete.
    private mutating func saveIfDue() -> Command? {
        guard savedOffset != nil, started, status?.isGoodToSave == true,
              let c = request(SdrAlignCommand.opSave) else { return nil }
        savedOffset = nil
        return c
    }

    // MARK: Answers

    /// A 0xD2 status. The pending command is answered when the status
    /// carries its id in a final state; then, or with none out, the board's
    /// save goes if it is due.
    mutating func take(_ s: SdrAlignStatus) -> Command? {
        status = s
        if let c = pending {
            guard s.fcCmdId == c.id, s.fcCmdOp == c.op, let st = s.fcCmdState else { return nil }
            switch st {
            case .idle, .sending:
                return nil
            case .echoed:
                lastOutcome = s.cmdResult == .done ? .done(op: c.op) : .notDone(op: c.op, s.cmdResult)
            case .noAnswer:
                lastOutcome = .noAnswer(op: c.op)
            case .refused:
                lastOutcome = .refused(op: c.op)
            }
            pending = nil
            generation += 1
        }
        return saveIfDue()
    }

    /// A timer armed at `generation` fired.
    mutating func timedOut(_ generation: Int) {
        guard generation == self.generation, let c = pending else { return }
        lastOutcome = .noStatus(op: c.op)
        pending = nil
        self.generation += 1
    }
}

// MARK: - What the screen says

nonisolated extension SdrAlignStatus {

    /// "Whole: 6 poses, 90° apart, the worst 0.14° off".
    var mountingText: String {
        switch have {
        case .none:
            return "No mounting yet"
        case .tilt:
            return "Tilt only: one pose so far. Turn it to another face for the whole rotation."
        case .whole:
            return String(format: "Whole: %d poses, %.0f° apart, the worst %.2f° off", poses.count,
                          Double(spreadDeg), Double(residDeg))
        }
    }

    /// "Turned 45.0° from the flight computer's IMU".
    var rotationText: String {
        String(format: "Turned %.1f° from the flight computer's IMU", rotationDeg)
    }

    /// One axis's calibration: "scale +1.20 %, offset +15.6 mg", or "not
    /// calibrated yet".
    func axisText(_ j: Int) -> String {
        guard axisCalibrated(j) else { return "not calibrated yet" }
        let scaleText = String(format: "scale %+.2f %%", (Double(scale[j]) - 1) * 100)
        return "\(scaleText), offset \(AccelCalStatus.mg(Double(bias[j])))"
    }

    /// What is saved on the board.
    var savedText: String {
        switch saved {
        case .none: return "Nothing saved on the board"
        case .same: return "Saved on the board"
        case .differs: return "The board has a saved mounting, but uses another now: not saved yet"
        }
    }

    /// What the board did with the last rest, when it says why a face is not
    /// counting; nil when it paired it (or none came).
    var lastRestText: String? {
        switch lastRest {
        case .paired?, .none?, nil: return nil
        case .moving?: return "The board was moving: hold it still."
        case .short?: return "The board was not still long enough: hold it still a little longer."
        case .late?: return "The flight computer's reading came too late for the board."
        case .notOneG?: return "The readings were not 1 g: was it moving?"
        case .noTime?: return "The GNSS board could not place that reading in time."
        case .imuOff?: return "The GNSS board's IMU is off."
        }
    }

    /// Why no face can count for the board now, if none can.
    var blockedText: String? {
        if !reportHeld { return "The GNSS board is not reporting its mounting: is its firmware current?" }
        if imu == 0 { return "The GNSS board's IMU is off: its readings are not paired." }
        if !canPlaceRest {
            return "The GNSS board cannot place the flight computer's readings yet: update its firmware, or give it the sky for a minute."
        }
        return nil
    }

    /// On the rail with a saved mounting: how far the board sits off it, and
    /// a warning past `movedWarnDeg`; nil otherwise.
    var movedText: String? {
        guard onSavedRotation else { return nil }
        if movedDeg > Self.movedWarnDeg {
            return String(format: "The GNSS board sits %.1f° off its saved mounting: did it move since its faces were measured? Measure the six faces again.", Double(movedDeg))
        }
        return String(format: "On the saved mounting, %.2f° off it here", Double(movedDeg))
    }

    /// Whole and calibrated, but not good enough for the save to go by itself.
    var poorFitText: String? {
        guard isComplete, !isGoodToSave else { return nil }
        return String(format: "Not saved by itself: %d poses, the worst %.2f° off, the fit %.3f m/s² rms. Measure the faces again, or save it as it is.",
                      poses.count, Double(residDeg), Double(calRms))
    }
}

nonisolated extension BoardAlignFlow.Outcome {
    private static func name(_ op: UInt8) -> String {
        switch op {
        case SdrAlignCommand.opSave: return "Save"
        case SdrAlignCommand.opClear: return "Clear"
        case SdrAlignCommand.opRestart: return "Start over"
        default: return "Report"
        }
    }

    var isGood: Bool {
        if case .done = self { return true }
        return false
    }

    var text: String {
        switch self {
        case .done(let op):
            switch op {
            case SdrAlignCommand.opSave: return "Saved on the GNSS board"
            case SdrAlignCommand.opClear: return "Cleared on the GNSS board"
            case SdrAlignCommand.opRestart: return "The GNSS board started its mounting afresh"
            default: return "The GNSS board reported"
            }
        case .notDone(let op, let r):
            switch r {
            case .nothingToSave?: return "\(Self.name(op)): the GNSS board had nothing to save"
            case .refused?: return "\(Self.name(op)): the GNSS board refused it (in flight, or its IMU off)"
            case .storageFailed?: return "\(Self.name(op)): the GNSS board could not write its storage"
            default: return "\(Self.name(op)): the GNSS board did not do it"
            }
        case .refused(let op):
            return "\(Self.name(op)): the flight computer would not send it (in flight, after a flight, or no GNSS board)"
        case .noAnswer(let op):
            return "\(Self.name(op)): the GNSS board did not answer"
        case .noStatus(let op):
            return "\(Self.name(op)): no answer from the flight computer: is its firmware current?"
        }
    }
}

nonisolated extension BoardAlignFlow.Again {
    /// "The flight computer's calibration moved by X +15.60 mg, ...".
    var text: String {
        let parts = (0..<3).map { "\(AccelSixFace.axisNames[$0]) \(AccelCalStatus.mg(Double(move[$0])))" }
        return "The flight computer's calibration moved (\(parts.joined(separator: ", "))), and the GNSS board calibrates against it: measure the six faces once more for the board."
    }
}
