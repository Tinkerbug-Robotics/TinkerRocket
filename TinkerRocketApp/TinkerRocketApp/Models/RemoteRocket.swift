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
