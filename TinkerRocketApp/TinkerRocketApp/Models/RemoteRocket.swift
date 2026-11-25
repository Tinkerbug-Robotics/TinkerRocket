//
//  RemoteRocket.swift
//  TinkerRocketApp
//
//  Represents a rocket seen via a base station's LoRa relay,
//  not directly connected over BLE.
//

import Foundation
import Combine

class RemoteRocket: ObservableObject, Identifiable {
    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim (swift_task_deinitOnExecutorMainActorBackDeploy), which
    // aborts with "pointer being freed was not allocated" when the object is
    // deallocated inside a synchronous XCTest. This class has no deinit-time
    // logic, so skipping the executor hop is free — and it un-crashes every
    // test that creates and tears down an instance.
    nonisolated deinit {}

    let baseStationDeviceID: UUID   // Which base station sees this rocket
    let rocketID: UInt8             // rocket_id from LoRa header

    @Published var unitName: String // Learned from LoRa name beacon
    @Published var telemetry = TelemetryData()
    @Published var lastSeen: Date = Date()

    var id: String { "\(baseStationDeviceID):\(rocketID)" }

    var displayName: String {
        unitName.isEmpty ? "Rocket \(rocketID)" : unitName
    }

    init(baseStationDeviceID: UUID, rocketID: UInt8, unitName: String = "") {
        self.baseStationDeviceID = baseStationDeviceID
        self.rocketID = rocketID
        self.unitName = unitName
        self.lastHeardName = unitName.isEmpty ? nil : unitName
    }

    // MARK: - Two rockets under one ID

    /// Two rockets set to the same rocket ID reach the base station as ONE
    /// rocket whose relayed name ("run") flips between theirs, frame by frame.
    /// The base station says so itself with "dup" when its firmware knows how;
    /// this is the app's own view of the same symptom, so an older base station
    /// still gets the warning and the pyro block.
    ///
    /// A rename is one change, so it takes two inside the window. Frames
    /// without a name (trimmed, or before the beacon) say nothing either way.
    static let nameAlternationWindow: TimeInterval = 10
    /// How long the flag holds after the last change seen. Long enough that a
    /// rocket briefly out of range does not clear it between alternations.
    static let nameAlternationHold: TimeInterval = 30

    private var lastHeardName: String?
    private var nameChangeTimes: [Date] = []
    /// When two name changes inside the window were last seen; nil = never.
    @Published private(set) var nameAlternationSeenAt: Date?

    /// True while this rocket ID looks like two rockets: the base station
    /// flags it, or the relayed name has been alternating.
    func duplicateIDSuspected(now: Date = Date()) -> Bool {
        if telemetry.duplicateIDSuspected { return true }
        guard let seen = nameAlternationSeenAt else { return false }
        return now.timeIntervalSince(seen) <= Self.nameAlternationHold
    }

    private func noteRelayedName(_ name: String?, now: Date) {
        guard let name, !name.isEmpty else { return }
        defer { lastHeardName = name }
        guard let last = lastHeardName, last != name else { return }
        nameChangeTimes.append(now)
        nameChangeTimes.removeAll { now.timeIntervalSince($0) > Self.nameAlternationWindow }
        if nameChangeTimes.count >= 2 { nameAlternationSeenAt = now }
    }

    func updateTelemetry(_ telemetry: TelemetryData, unitName: String?, now: Date = Date()) {
        noteRelayedName(unitName, now: now)
        self.telemetry = telemetry
        // #1036: the base station re-pushes this rocket's CACHED frame every
        // 2 s and tags it STALE once the underlying LoRa packet is older than
        // 3 s — but the re-push still carries the rocket id. Stamping lastSeen
        // off it made every age derived from this clock bounded by that 2 s
        // period, so it could never cross a 3 s threshold for as long as the
        // base station held the slot, and slots are never released.
        //
        // The consequences were all silent: the fleet card kept rendering
        // .live at full opacity through a recovery walk-out, pyroCommandPathReady
        // degenerated to "a rocket was heard at least once since connect" so
        // the stand-back test's pre-fire abort never fired, and
        // autoApplyRefusalReason could never return .noRocketPresent.
        //
        // The telemetry assignment above stays unconditional so the cached
        // values still render; only the "when did we last actually hear it"
        // clock is withheld. A missing "ds" decodes as .live, so older
        // base-station firmware stamps exactly as it does today. Android
        // carries the identical guard in DeviceSession; the two must not
        // diverge.
        if telemetry.data_status == .live {
            self.lastSeen = now
        }
        if let name = unitName, !name.isEmpty {
            self.unitName = name
        }
    }
}
