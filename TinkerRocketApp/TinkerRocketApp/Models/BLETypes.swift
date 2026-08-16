//
//  BLETypes.swift
//  TinkerRocketApp
//
//  Shared types used by BLEFleet, BLEDevice, and views.
//  Extracted from BLEManager.swift.
//

import Foundation
import CoreBluetooth

/// Rail-off pyro-test refusal (0xCE on the file_ops characteristic): the OC
/// refused to queue a pyro test because the FC power rail is off — a held
/// cmd would deliver its fire/ARM pulse at the next power-on.
struct PyroTestRefusal: Equatable {
    let cmd: UInt8       // refused BLE cmd: 35 continuity / 36 fire
    let channel: UInt8   // 1..4
    let reason: UInt8    // 1 = FC power rail off
    /// Arrival stamp so consecutive refusals of the same channel still
    /// register as a change for SwiftUI onChange observers.
    let at: Date
}

struct FileInfo: Identifiable, Codable, Equatable {
    let name: String
    let size: UInt32
    var id: String { name }

    private static let utcDateFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyyMMddHHmmss"
        f.timeZone = TimeZone(identifier: "UTC")
        return f
    }()

    private static let localDisplayFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateStyle = .short
        f.timeStyle = .short
        f.timeZone = .current
        return f
    }()

    var flightDate: Date? {
        let prefixLen: Int
        if name.hasPrefix("flight_") && name.count >= 26 {
            prefixLen = 7
        } else if name.hasPrefix("lora_") && name.count >= 23 {
            prefixLen = 5
        } else {
            return nil
        }

        let dateStart = name.index(name.startIndex, offsetBy: prefixLen)
        let dateEnd = name.index(dateStart, offsetBy: 8)
        let timeStart = name.index(dateEnd, offsetBy: 1)
        let timeEnd = name.index(timeStart, offsetBy: 6)

        let dateStr = String(name[dateStart..<dateEnd])
        let timeStr = String(name[timeStart..<timeEnd])

        guard dateStr.count == 8, timeStr.count == 6,
              dateStr.allSatisfy(\.isNumber), timeStr.allSatisfy(\.isNumber) else { return nil }

        return Self.utcDateFormatter.date(from: dateStr + timeStr)
    }

    var displayTitle: String {
        guard let date = flightDate else { return name }
        return Self.localDisplayFormatter.string(from: date)
    }

    var timestampDisplay: String { displayTitle }
}

enum DownloadState {
    case notDownloaded
    case downloading
    case generatingCSV
    case completed
    case failed
}

struct DiscoveredDevice: Identifiable {
    let id: UUID
    let peripheral: CBPeripheral
    /// Mutable: the first advertising report for a peripheral often carries no
    /// local name, so the name is upgraded when a later report brings the real
    /// one (see BLEFleet.centralManager(_:didDiscover:...)).
    var name: String
    var rssi: Int

    /// Type recovered from the known-device registry at discovery time
    /// (matched by advertised name).  nil when the device isn't in the
    /// registry — e.g. a fresh app install, or a board renamed on another
    /// phone.
    var knownType: BLEDeviceType? = nil

    /// Registry hint first; the TR-R-/TR-B- name heuristic only works for
    /// factory-default names (the firmware advertises the raw unit name).
    var deviceType: BLEDeviceType {
        knownType ?? BLEDeviceType.from(name: name)
    }

    var isBaseStation: Bool {
        deviceType == .baseStation
    }

    var typeLabel: String {
        switch deviceType {
        case .rocket: return "Rocket"
        case .baseStation: return "Base Station"
        case .unknown: return "TinkerRocket device"
        }
    }
}

struct RocketConfig {
    var servoBias1: Int16 = 0   // #561: match RocketProfile/config.h (was 85 → ~10° servo-1 trim)
    var servoHz: Int16 = 333
    // #407: match RocketProfile (the source of truth) and config.h
    // SERVO_MIN_US/MAX_US = 1000/2000. The old 1250/1750 disagreed with the
    // profile default and, paired with the ±60° fin cal, under-deflected ~2×.
    var servoMinUs: Int16 = 1000
    var servoMaxUs: Int16 = 2000
    var pidKp: Float = 0.08
    var pidKi: Float = 0.005
    var pidKd: Float = 0.003
    var pidMinCmd: Float = -10.0
    var pidMaxCmd: Float = 10.0
    var servoEnabled: Bool = true
    var gainScheduleEnabled: Bool = true
    var useAngleControl: Bool = false
    var rollDelayMs: UInt16 = 0
    var rateCapDps: Float = 60
    var kpAngle: Float = 2.0               // outer angle-loop P-gain (cascaded angle control)
    var integralSepThreshold: Float = 40   // PID integral-separation anti-windup threshold (deg/s); 0 disables
    var guidanceEnabled: Bool = false
    var cameraType: UInt8 = 2
    var imuOrientSetting: UInt8? = nil   // 0xFF auto / 0..23 manual (nil = not reported)
    var imuRateHz: UInt16? = nil         // ISM6 logging rate readback (nil = not reported)
    var loraFreqMHz: Float? = nil
    var loraSF: UInt8? = nil
    var loraBwKHz: Float? = nil
    var loraCR: UInt8? = nil
    var loraTxPower: Int8? = nil
    var loraHopDisabled: Bool? = nil   // #106 fixed-frequency override (BS-controlled)
    // #150: firmware-computed hop dwell for the current modulation.
    // 0 = hopping not legally possible at this preset (GUI greys the
    // option); nil = device doesn't report it (pre-#150 firmware).
    var loraHopDwell: Int? = nil
    var pyro1Enabled: Bool = false
    var pyro1TriggerMode: UInt8 = 0
    var pyro1TriggerValue: Float = 1.0
    var pyro2Enabled: Bool = false
    var pyro2TriggerMode: UInt8 = 0
    var pyro2TriggerValue: Float = 100.0
    var pyro3Enabled: Bool = false
    var pyro3TriggerMode: UInt8 = 0
    var pyro3TriggerValue: Float = 0.0
    var pyro4Enabled: Bool = false
    var pyro4TriggerMode: UInt8 = 0
    var pyro4TriggerValue: Float = 0.0
}

// MARK: - Guidance-target echo (#435)

/// FC-authoritative guidance aim-point state, relayed by the OC as a small
/// `{"type":"guid_target",...}` JSON frame (OutStatusQueryData v5 tail).
/// Live state, NOT folded into `RocketConfig` — it changes with every
/// processed cmd 28 / cmd-65 apply, and the Drift-Cast send button gates its
/// success on it. `status` describes the CURRENT target; `lastRc` the result
/// of the LAST processed cmd-28 upload. They are separate fields so a
/// rejected upload does not erase the display of a still-active target.
/// Absent entirely (nil on BLEDevice) on pre-#435 firmware.
nonisolated struct GuidanceTargetEcho: Equatable {
    // tgt_status values — mirror GUID_TGT_* in RocketComputerTypes.h.
    static let statusNone = 0        // no aim point (overhead / cleared)
    static let statusGeoActive = 1   // cmd-28 geodetic point active
    static let statusEnActive = 2    // cmd-65 profile E/N point active

    /// Bumps on EVERY processed cmd 28 (accept or reject) and on any cmd-65
    /// apply that changes the echo content. The send flow captures a baseline
    /// before sending and treats any advance as "the FC processed something".
    let seq: Int
    let status: Int   // GUID_TGT_*
    let lastRc: Int   // GUID_RC_* (see GuidancePointRc)
    /// Receipt-time geodetic point as the FC stored it, f32-rounded on the
    /// wire (ulp ~0.4 m at mid-latitudes — verification precision, the FC
    /// keeps the f64 originals for navigation).
    let lat: Double
    let lon: Double
    let altM: Int     // stored/echoed only — station-keep ignores altitude

    /// Parse the OC's compact readback frame. Defensive per the MTU-budget
    /// rule (#282): every field individually optional.
    init?(json dict: [String: Any]) {
        guard dict["type"] as? String == "guid_target" else { return nil }
        self.init(seq: dict["seq"] as? Int ?? 0,
                  status: dict["st"] as? Int ?? 0,
                  lastRc: dict["rc"] as? Int ?? 0,
                  lat: (dict["lat"] as? Double) ?? 0,
                  lon: (dict["lon"] as? Double) ?? 0,
                  altM: dict["alt"] as? Int ?? 0)
    }

    init(seq: Int, status: Int, lastRc: Int, lat: Double, lon: Double, altM: Int) {
        self.seq = seq
        self.status = status
        self.lastRc = lastRc
        self.lat = lat
        self.lon = lon
        self.altM = altM
    }

    // Confirmation-match tolerances: the echo is f32-rounded (ulp ~4e-6 deg),
    // leaving >4x margin under these. Lon is looser than lat because a
    // degree of longitude shrinks with cos(lat) while the f32 ulp doesn't.
    static let latMatchTolDeg = 2e-5   // ~2.2 m
    static let lonMatchTolDeg = 3e-5   // ~2.6 m at mid-latitudes

    /// Pure confirmation predicate for a cmd-28 upload (unit-tested).
    /// `baselineSeq` is the echo seq captured immediately before sending.
    ///
    /// Attribution caveats (the reason `GuidanceSendFlow` exists):
    /// - `baselineSeq < 0` means NO baseline — nothing can be attributed to
    ///   the upload, so every frame is `.pending` (the flow adopts the first
    ///   frame as its baseline instead of misreading it as a verdict).
    /// - `tgt_seq` also bumps on echo-changing cmd-65 applies, which never
    ///   write `tgt_last_rc` — so `rc == NONE` can never be a cmd-28 verdict
    ///   (the FC writes a nonzero GUID_RC_* on every processed cmd 28) and
    ///   maps to `.pending`, and a `.rejected`/`.mismatch` result may still
    ///   be an interleaved cmd-65 frame rather than OUR verdict.  Only
    ///   `.confirmed` is self-authenticating (fresh ACCEPTED + GEO_ACTIVE +
    ///   coordinate match); the flow treats the failures as provisional.
    func outcome(baselineSeq: Int, sentLat: Double, sentLon: Double) -> GuidanceSendOutcome {
        if baselineSeq < 0 { return .pending }
        if seq == baselineSeq { return .pending }
        if lastRc == GuidancePointRc.none.rawValue { return .pending }
        if lastRc != GuidancePointRc.accepted.rawValue { return .rejected(rc: lastRc) }
        guard status == Self.statusGeoActive,
              abs(lat - sentLat) < Self.latMatchTolDeg,
              abs(lon - sentLon) < Self.lonMatchTolDeg else { return .mismatch }
        return .confirmed
    }
}

/// Result of matching a guidance-target echo against a just-sent point.
nonisolated enum GuidanceSendOutcome: Equatable {
    case pending             // seq hasn't advanced — FC hasn't processed it yet
    case confirmed           // accepted, geodetic-active, coordinates match
    case rejected(rc: Int)   // FC processed and refused (GUID_RC_* reason)
    case mismatch            // FC accepted SOMETHING, but not our point
}

/// Pure verdict state machine for one cmd-28 send (#435, unit-tested).
///
/// Frame-attribution rules — each one closes a demonstrated false-verdict
/// path in the naive "first seq advance is the answer" reading:
/// - No baseline (`baselineSeq < 0`, e.g. send raced the connect-time
///   readback): the FIRST frame — typically the OC's stale connect push of a
///   PREVIOUS session's state — becomes the baseline, never a verdict.
///   Previously it could paint a false green for an upload the FC rejected,
///   or never even received.
/// - `.confirmed` is self-authenticating (fresh ACCEPTED + GEO_ACTIVE +
///   coordinate match after a real seq advance) and settles immediately.
/// - `.rejected` / `.mismatch` frames are only PROVISIONAL: an interleaved
///   cmd-65 apply (connect-time profile sync, settings self-apply) also bumps
///   seq while carrying a STALE rc, so the true accept may still be one poll
///   behind. The provisional reason settles as `.failed` only at timeout, and
///   a later genuine `.confirmed` upgrades over it.
/// Net effect: false green and false red are both closed; the worst case for
/// a genuine rejection is that its reason displays at the timeout instead of
/// instantly.
nonisolated struct GuidanceSendFlow: Equatable {
    enum Verdict: Equatable {
        case waiting
        case confirmed
        case failed(String)
    }

    static let timeoutMessage =
        "No confirmation from the rocket within 6 s. The firmware may predate #435, or the link dropped — the aim point is NOT confirmed."

    private(set) var baselineSeq: Int
    let sentLat: Double
    let sentLon: Double
    private(set) var provisionalFailure: String?
    private(set) var verdict: Verdict = .waiting

    init(baselineSeq: Int, sentLat: Double, sentLon: Double) {
        self.baselineSeq = baselineSeq
        self.sentLat = sentLat
        self.sentLon = sentLon
    }

    mutating func onEcho(_ echo: GuidanceTargetEcho) {
        guard verdict == .waiting else { return }
        if baselineSeq < 0 {
            // Unattributable — adopt as baseline (see rules above).
            baselineSeq = echo.seq
            return
        }
        switch echo.outcome(baselineSeq: baselineSeq, sentLat: sentLat, sentLon: sentLon) {
        case .pending:
            break
        case .confirmed:
            verdict = .confirmed
        case .rejected(let rc):
            provisionalFailure = GuidancePointRc.message(forRawValue: rc)
        case .mismatch:
            provisionalFailure =
                "The rocket confirmed a different aim point than the one sent — resend, and check nothing else is pushing settings."
        }
    }

    mutating func onTimeout() {
        guard verdict == .waiting else { return }
        verdict = .failed(provisionalFailure ?? Self.timeoutMessage)
    }
}

/// FC cmd-28 result codes — mirror GUID_RC_* in RocketComputerTypes.h (#435).
nonisolated enum GuidancePointRc: Int {
    case none = 0
    case accepted = 1
    case rejectedRadius = 2
    case rejectedState = 3
    case rejectedLaw = 4
    case rejectedNoRef = 5
    case rejectedBadValue = 6

    var message: String {
        switch self {
        case .none:
            return "No upload has been processed."
        case .accepted:
            return "Aim point accepted."
        case .rejectedRadius:
            return "Aim point is more than 100 m from the launch reference — recompute closer to the pad."
        case .rejectedState:
            return "Rocket must be in READY or PRELAUNCH (and not post-flight)."
        case .rejectedLaw:
            return "Rocket guidance law is PN — set the profile's law to Station-Keep and push settings first."
        case .rejectedNoRef:
            return "Rocket has no GNSS reference yet — wait for pad fix averaging."
        case .rejectedBadValue:
            return "Rejected: invalid values."
        }
    }

    static func message(forRawValue rc: Int) -> String {
        GuidancePointRc(rawValue: rc)?.message ?? "Rejected (unknown code \(rc))."
    }
}
