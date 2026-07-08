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
        self.lastSeen = Date()
        if let name = unitName, !name.isEmpty {
            self.unitName = name
        }
    }
}
