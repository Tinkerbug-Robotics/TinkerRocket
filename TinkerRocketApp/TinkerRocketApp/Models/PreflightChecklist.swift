//
//  PreflightChecklist.swift
//  TinkerRocketApp
//
//  Pre-flight checklist data model.  A single MASTER checklist (a template
//  of steps every rocket starts from) plus a per-rocket config that
//  references it: each rocket can exclude master steps and add its own.
//  The master is LIVE — editing a master step updates it on every rocket
//  that includes it; the per-rocket file stores only the diff (excluded
//  ids + extra items) plus the checked state of the current run.
//
//  Steps come in two kinds.  MANUAL steps are free-text items the operator
//  checks off by hand ("wadding installed", "motor retained").  AUTO steps
//  are verified by the app from live telemetry / sync state and can never
//  be hand-checked — the checklist only completes when the app has actually
//  observed the condition (camera recording, logging active, …).
//
//  Storage schema is shared with the Android app (same JSON key names,
//  Apple-epoch dates, uppercase UUID strings) so checklists could
//  round-trip across platforms — the same contract RocketProfile keeps.
//

import Foundation

// MARK: - Item kinds

/// What a checklist step is and, for auto steps, which app-observable
/// condition satisfies it.  Raw string rides in the JSON.
enum PreflightItemKind: String, Codable, CaseIterable {
    case manual                       // operator checks it off by hand

    // Auto steps — verified from telemetry / app state, never hand-checked.
    case connected                    // rocket powered on & telemetry flowing
    case settingsSynced               // active profile pushed to the rocket
    case gnssFix                      // usable GNSS solution
    case cameraRecording              // camera reports recording
    case loggingActive                // flight logging running
    case pyroArmed                    // pyro ARM FET on
    case pyroContinuity               // continuity on every enabled channel

    var isAuto: Bool { self != .manual }

    /// Default title when the step is added from the auto library.  The
    /// title stays user-editable — it's just a label; `kind` drives the
    /// verification.
    var defaultTitle: String {
        switch self {
        case .manual:          return ""
        case .connected:       return "Rocket powered on & connected"
        case .settingsSynced:  return "Settings applied to rocket"
        case .gnssFix:         return "GNSS fix acquired"
        case .cameraRecording: return "Camera recording"
        case .loggingActive:   return "Flight logging active"
        case .pyroArmed:       return "Pyro channels armed"
        case .pyroContinuity:  return "Pyro continuity good"
        }
    }

    var defaultDetail: String {
        switch self {
        case .manual:          return ""
        case .connected:       return "Verified from the live telemetry link."
        case .settingsSynced:  return "The active rocket profile has been pushed and confirmed."
        case .gnssFix:         return "Position fix with enough satellites for a 3D solution."
        case .cameraRecording: return "The flight computer reports the camera is recording."
        case .loggingActive:   return "The flight computer is writing the flight log."
        case .pyroArmed:       return "The shared ARM FET is on."
        case .pyroContinuity:  return "Every channel enabled in this rocket's profile shows continuity. Requires the flight battery — USB power alone fakes continuity on all channels."
        }
    }
}

// MARK: - Items

/// One checklist step.  Lives either in the master list or in a rocket's
/// extra items; `id` is the checked-state key, so it must be stable.
struct PreflightItem: Codable, Equatable, Identifiable {
    var id: UUID = UUID()
    var title: String
    var detail: String = ""
    var kind: PreflightItemKind = .manual

    /// A step seeded from the auto library.
    static func auto(_ kind: PreflightItemKind) -> PreflightItem {
        PreflightItem(title: kind.defaultTitle, detail: kind.defaultDetail, kind: kind)
    }

    // Decode-with-defaults (the RocketProfile rule): a missing key must
    // never drop the file, and an auto kind added by a NEWER app decodes
    // as .manual here rather than failing — the step stays visible and
    // checkable instead of vanishing.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try c.decodeIfPresent(UUID.self, forKey: .id) ?? UUID()
        title = try c.decodeIfPresent(String.self, forKey: .title) ?? ""
        detail = try c.decodeIfPresent(String.self, forKey: .detail) ?? ""
        let raw = try c.decodeIfPresent(String.self, forKey: .kind) ?? "manual"
        kind = PreflightItemKind(rawValue: raw) ?? .manual
    }

    init(id: UUID = UUID(), title: String, detail: String = "",
         kind: PreflightItemKind = .manual) {
        self.id = id
        self.title = title
        self.detail = detail
        self.kind = kind
    }
}

// MARK: - Master + per-rocket config

/// The master checklist: the ordered template every rocket starts from.
struct PreflightMaster: Codable, Equatable {
    var items: [PreflightItem] = []
    var updatedAt: Date = Date()

    init(items: [PreflightItem] = [], updatedAt: Date = Date()) {
        self.items = items
        self.updatedAt = updatedAt
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        items = try c.decodeIfPresent([PreflightItem].self, forKey: .items) ?? []
        updatedAt = try c.decodeIfPresent(Date.self, forKey: .updatedAt) ?? Date()
    }
}

/// A rocket's diff against the master, plus the checked state of the
/// current run.  `checked` maps item id (uppercase UUID string — JSON
/// dictionary keys must be strings) to when the operator checked it;
/// only manual steps ever appear in it.
struct PreflightRocketConfig: Codable, Equatable {
    var profileId: UUID
    var disabledMasterIds: [UUID] = []
    var extraItems: [PreflightItem] = []
    var checked: [String: Date] = [:]
    var updatedAt: Date = Date()

    init(profileId: UUID, disabledMasterIds: [UUID] = [],
         extraItems: [PreflightItem] = [], checked: [String: Date] = [:],
         updatedAt: Date = Date()) {
        self.profileId = profileId
        self.disabledMasterIds = disabledMasterIds
        self.extraItems = extraItems
        self.checked = checked
        self.updatedAt = updatedAt
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        profileId = try c.decode(UUID.self, forKey: .profileId)
        disabledMasterIds = try c.decodeIfPresent([UUID].self, forKey: .disabledMasterIds) ?? []
        extraItems = try c.decodeIfPresent([PreflightItem].self, forKey: .extraItems) ?? []
        checked = try c.decodeIfPresent([String: Date].self, forKey: .checked) ?? [:]
        updatedAt = try c.decodeIfPresent(Date.self, forKey: .updatedAt) ?? Date()
    }

    func isChecked(_ itemId: UUID) -> Bool {
        checked[itemId.uuidString] != nil
    }
}

// MARK: - Effective list + progress

/// Status of one AUTO step, evaluated from live app state.
enum PreflightAutoStatus: Equatable {
    case satisfied
    /// Condition not yet observed; the string says what's missing.
    case pending(String)
    /// The step doesn't apply to this rocket (e.g. no camera configured).
    /// Counts as done — an inapplicable step must not block completion.
    case notApplicable(String)

    /// Whether this status counts toward completion.
    var countsAsDone: Bool {
        if case .pending = self { return false }
        return true
    }
}

/// Everything the auto evaluation can see.  Bundled so the pure functions
/// stay unit-testable without a live BLE stack.
struct PreflightAutoContext {
    var isConnected: Bool = false
    var hasTelemetry: Bool = false
    /// Base-station relay link: some signals never ride LoRa (settings
    /// sync is a direct-BLE flow), so the affected steps go N/A instead
    /// of pending forever.
    var isRelay: Bool = false
    var telemetry: TelemetryData = TelemetryData()
    var syncState: ActiveRocketSyncer.SyncState = .idle
    var profile: RocketProfile? = nil
}

struct PreflightProgress: Equatable {
    var done: Int = 0
    var total: Int = 0
    var isComplete: Bool { total > 0 && done == total }
}

enum PreflightChecklist {

    /// A rocket's effective checklist: master items in master order, minus
    /// the ones this rocket excludes, then the rocket's own extras.
    static func effectiveItems(master: PreflightMaster,
                               config: PreflightRocketConfig?) -> [PreflightItem] {
        guard let config else { return master.items }
        let disabled = Set(config.disabledMasterIds)
        return master.items.filter { !disabled.contains($0.id) } + config.extraItems
    }

    /// Live status of one auto step; nil for manual steps.
    static func autoStatus(_ kind: PreflightItemKind,
                           in ctx: PreflightAutoContext) -> PreflightAutoStatus? {
        guard kind.isAuto else { return nil }

        // Everything below reads telemetry — without a link there is
        // nothing to verify against.
        guard ctx.isConnected else { return .pending("Not connected") }

        switch kind {
        case .manual:
            return nil

        case .connected:
            return ctx.hasTelemetry ? .satisfied : .pending("Waiting for telemetry")

        case .settingsSynced:
            if ctx.isRelay { return .notApplicable("Needs a direct connection") }
            switch ctx.syncState {
            case .synced:    return .satisfied
            case .noProfile: return .pending("No active rocket selected")
            case .failed(let reason): return .pending(reason)
            default:         return .pending("Not yet applied")
            }

        case .gnssFix:
            guard ctx.hasTelemetry else { return .pending("Waiting for telemetry") }
            let t = ctx.telemetry
            // Same usability bar as the map's latched fix (LastValidRocketFix):
            // a real position and enough satellites for a 3D solution.
            if let lat = t.latitude, let lon = t.longitude,
               !(lat == 0 && lon == 0),
               t.num_sats >= LastValidRocketFix.minSatsForValidFix {
                return .satisfied
            }
            return .pending("No fix (\(t.num_sats) sats)")

        case .cameraRecording:
            if let p = ctx.profile, p.cameraType == 0 {
                return .notApplicable("No camera on this rocket")
            }
            guard ctx.hasTelemetry else { return .pending("Waiting for telemetry") }
            return ctx.telemetry.camera_recording ? .satisfied : .pending("Not recording")

        case .loggingActive:
            guard ctx.hasTelemetry else { return .pending("Waiting for telemetry") }
            return ctx.telemetry.logging_active ? .satisfied : .pending("Not logging")

        case .pyroArmed:
            guard let p = ctx.profile, !enabledPyroChannels(p).isEmpty else {
                return .notApplicable("No pyro channels enabled")
            }
            guard ctx.hasTelemetry else { return .pending("Waiting for telemetry") }
            return ctx.telemetry.pyro_armed ? .satisfied : .pending("Not armed")

        case .pyroContinuity:
            guard let p = ctx.profile else {
                return .notApplicable("No pyro channels enabled")
            }
            let channels = enabledPyroChannels(p)
            guard !channels.isEmpty else {
                return .notApplicable("No pyro channels enabled")
            }
            guard ctx.hasTelemetry else { return .pending("Waiting for telemetry") }
            let open = channels.filter { !ctx.telemetry.pyroCont(channel: $0) }
            if open.isEmpty { return .satisfied }
            return .pending("Ch \(open.map(String.init).joined(separator: ", ")) open")
        }
    }

    /// Rollup: manual steps count when checked, auto steps when satisfied
    /// or N/A.  An empty list is never "complete" — no checklist, no badge.
    static func progress(items: [PreflightItem],
                         config: PreflightRocketConfig?,
                         ctx: PreflightAutoContext) -> PreflightProgress {
        var done = 0
        for item in items {
            if let status = autoStatus(item.kind, in: ctx) {
                if status.countsAsDone { done += 1 }
            } else if config?.isChecked(item.id) == true {
                done += 1
            }
        }
        return PreflightProgress(done: done, total: items.count)
    }

    /// Pyro channels (1–4) enabled in the profile — the set continuity and
    /// arming are judged against.
    static func enabledPyroChannels(_ p: RocketProfile) -> [Int] {
        var channels: [Int] = []
        if p.pyro1Enabled { channels.append(1) }
        if p.pyro2Enabled { channels.append(2) }
        if p.pyro3Enabled { channels.append(3) }
        if p.pyro4Enabled { channels.append(4) }
        return channels
    }
}
