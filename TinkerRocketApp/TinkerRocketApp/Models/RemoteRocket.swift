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
    }

    func updateTelemetry(_ telemetry: TelemetryData, unitName: String?) {
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
            self.lastSeen = Date()
        }
        if let name = unitName, !name.isEmpty {
            self.unitName = name
        }
    }
}
