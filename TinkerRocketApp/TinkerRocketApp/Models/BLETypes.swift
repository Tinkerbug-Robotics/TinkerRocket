//
//  BLETypes.swift
//  TinkerRocketApp
//
//  Shared types used by BLEFleet, BLEDevice, and views.
//  Extracted from BLEManager.swift.
//

import Foundation
import CoreBluetooth

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
    let name: String
    var rssi: Int

    var isBaseStation: Bool {
        name.hasPrefix("TR-B-") || name.contains("Base") || name.contains("BS")
    }

    var typeLabel: String {
        isBaseStation ? "Base Station" : "Rocket"
    }
}

struct RocketConfig {
    var servoBias1: Int16 = 85
    var servoHz: Int16 = 333
    var servoMinUs: Int16 = 1250
    var servoMaxUs: Int16 = 1750
    var pidKp: Float = 0.08
    var pidKi: Float = 0.005
    var pidKd: Float = 0.003
    var pidMinCmd: Float = -10.0
    var pidMaxCmd: Float = 10.0
    var servoEnabled: Bool = true
    var gainScheduleEnabled: Bool = true
    var useAngleControl: Bool = false
    var rollDelayMs: UInt16 = 0
    var guidanceEnabled: Bool = false
    var cameraType: UInt8 = 2
    var loraFreqMHz: Float? = nil
    var loraSF: UInt8? = nil
    var loraBwKHz: Float? = nil
    var loraCR: UInt8? = nil
    var loraTxPower: Int8? = nil
    var loraHopDisabled: Bool? = nil   // #106 fixed-frequency override (BS-controlled)
    var pyro1Enabled: Bool = false
    var pyro1TriggerMode: UInt8 = 0
    var pyro1TriggerValue: Float = 1.0
    var pyro2Enabled: Bool = false
    var pyro2TriggerMode: UInt8 = 0
    var pyro2TriggerValue: Float = 100.0
}
