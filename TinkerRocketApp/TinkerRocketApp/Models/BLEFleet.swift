//
//  BLEFleet.swift
//  TinkerRocketApp
//
//  Multi-device BLE manager — owns CBCentralManager, scanning, and
//  routes connection events to individual BLEDevice instances.
//  Extracted from BLEManager.swift for multi-device support.
//

import Foundation
import CoreBluetooth
import Combine
import UIKit

class BLEFleet: NSObject, ObservableObject {
    // MARK: - Published state

    @Published var isScanning = false
    @Published var statusMessage = "Not connected"
    @Published var discoveredDevices: [DiscoveredDevice] = []

    /// Currently connected devices (for now, max 1 — multi-connect in PR #4).
    @Published var devices: [BLEDevice] = []

    /// Which device the dashboard is currently showing.
    @Published var activeDeviceID: UUID?

    /// Latest usable rocket GPS fix per rocketID.  Lives on the fleet
    /// (not on a BLEDevice) because BLEFleet destroys+recreates a
    /// BLEDevice on every BLE disconnect/reconnect — without an outer
    /// cache, the map would blank every time the phone briefly loses
    /// the base station.  In-memory only; cleared on app kill.  #140.
    @Published var lastValidRocketFixes: [UInt8: LastValidRocketFix] = [:]

    /// Convenience: the active device, or nil.
    var activeDevice: BLEDevice? {
        guard let id = activeDeviceID else { return devices.first }
        return devices.first { $0.peripheral?.identifier == id }
    }

    /// All remote rockets seen via connected base stations.
    var remoteRockets: [RemoteRocket] {
        devices.flatMap { $0.remoteRockets }
    }

    /// True if any device is connected.
    var isConnected: Bool {
        !devices.isEmpty
    }

    // MARK: - Private state

    private var centralManager: CBCentralManager!
    private let serviceUUID = CBUUID(string: "4fafc201-1fb5-459e-8fcc-c5c9c331914b")

    // Reconnection state
    private var reconnectAttempts: Int = 0
    private let maxReconnectAttempts = 3
    private var lastPeripheralIdentifier: UUID?
    private var userInitiatedDisconnect = false
    private var reconnectingPeripheral: CBPeripheral?

    // Strong references to peripherals while a connection is in flight (#173).
    // CBCentralManager only retains peripherals weakly, so without our own
    // strong reference the CBPeripheral can be deallocated mid-connect (e.g.
    // when `discoveredDevices` is mutated), and CoreBluetooth cancels with
    // "API MISUSE: Cancelling connection for unused peripheral … Did you
    // forget to keep a reference to it?".  Held from connect() until the
    // connection resolves (didConnect / didFailToConnect / didDisconnect).
    private var connectingPeripherals: [UUID: CBPeripheral] = [:]

    // MARK: - Init

    override init() {
        super.init()
        centralManager = CBCentralManager(
            delegate: self,
            queue: nil,
            options: [CBCentralManagerOptionRestoreIdentifierKey: "TinkerRocketBLE"]
        )
    }

    // MARK: - Scanning

    func startScanning() {
        guard centralManager.state == .poweredOn else {
            statusMessage = "Bluetooth not ready"
            return
        }
        discoveredDevices = []
        statusMessage = "Scanning..."
        isScanning = true
        centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)

        DispatchQueue.main.asyncAfter(deadline: .now() + 15) { [weak self] in
            if self?.isScanning == true {
                self?.stopScanning()
                if self?.devices.isEmpty == true {
                    if self?.discoveredDevices.isEmpty == true {
                        self?.statusMessage = "No devices found"
                    } else {
                        self?.statusMessage = "Scan complete"
                    }
                }
            }
        }
    }

    func stopScanning() {
        centralManager.stopScan()
        isScanning = false
    }

    // MARK: - Connection

    func connect(to device: DiscoveredDevice) {
        stopScanning()
        statusMessage = "Connecting to \(device.name)..."
        // Retain the peripheral for the duration of the connect (#173).
        connectingPeripherals[device.peripheral.identifier] = device.peripheral
        centralManager.connect(device.peripheral, options: nil)
    }

    func disconnect(_ device: BLEDevice) {
        userInitiatedDisconnect = true
        if let peripheral = device.peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }

    /// Disconnect all devices (convenience for single-device mode).
    func disconnectAll() {
        userInitiatedDisconnect = true
        for device in devices {
            if let peripheral = device.peripheral {
                centralManager.cancelPeripheralConnection(peripheral)
            }
        }
    }

    // MARK: - Device lookup

    private func device(for peripheral: CBPeripheral) -> BLEDevice? {
        devices.first { $0.peripheral?.identifier == peripheral.identifier }
    }

    // MARK: - Last-valid rocket fix cache (#140)

    /// Called by BLEDevice on every telemetry packet.  Updates the
    /// cached fix when the packet carries a usable GPS solution, and
    /// returns the resulting fix (new or pre-existing) so the caller
    /// can mirror it onto its own @Published property — including the
    /// hydrate-on-reconnect case where a brand-new BLEDevice's first
    /// packet has no GPS but a prior session cached one.
    @discardableResult
    func recordRocketFix(from telemetry: TelemetryData,
                         rocketID: UInt8) -> LastValidRocketFix? {
        if let fresh = LastValidRocketFix.fromTelemetry(telemetry, rocketID: rocketID) {
            lastValidRocketFixes[rocketID] = fresh
            return fresh
        }
        return lastValidRocketFixes[rocketID]
    }
}

// MARK: - CBCentralManagerDelegate

extension BLEFleet: CBCentralManagerDelegate {

    func centralManager(_ central: CBCentralManager,
                       willRestoreState dict: [String: Any]) {
        if let peripherals = dict[CBCentralManagerRestoredStatePeripheralsKey] as? [CBPeripheral],
           let restored = peripherals.first {
            print("[BLE] State restoration: holding reference to \(restored.name ?? "unknown")")
            reconnectingPeripheral = restored
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            statusMessage = "Bluetooth ready"
            if let p = reconnectingPeripheral, p.state == .connected {
                print("[BLE] Restored peripheral still connected — resuming")
                let device = BLEDevice(peripheral: p, name: p.name ?? "Restored")
                device.onConnect()
                devices.append(device)
                activeDeviceID = p.identifier
                reconnectingPeripheral = nil
            } else {
                startScanning()
            }
        case .poweredOff:
            statusMessage = "Bluetooth is off"
        case .unauthorized:
            statusMessage = "Bluetooth not authorized"
        case .unsupported:
            statusMessage = "Bluetooth not supported"
        default:
            statusMessage = "Bluetooth unknown state"
        }
    }

    func centralManager(_ central: CBCentralManager,
                       didDiscover peripheral: CBPeripheral,
                       advertisementData: [String: Any],
                       rssi RSSI: NSNumber) {
        let name = peripheral.name ?? "Unknown"
        print("Discovered: \(name) RSSI: \(RSSI)")

        // Only list Tinker devices (legacy "TinkerRocket"/"TinkerBaseStation" + new "TR-R-"/"TR-B-")
        guard name.hasPrefix("TR-") || name.contains("Tinker") else { return }

        if let idx = discoveredDevices.firstIndex(where: { $0.id == peripheral.identifier }) {
            discoveredDevices[idx].rssi = RSSI.intValue
        } else {
            let device = DiscoveredDevice(
                id: peripheral.identifier,
                peripheral: peripheral,
                name: name,
                rssi: RSSI.intValue
            )
            discoveredDevices.append(device)
        }
        statusMessage = "Found \(discoveredDevices.count) device\(discoveredDevices.count == 1 ? "" : "s")"
    }

    func centralManager(_ central: CBCentralManager,
                       didConnect peripheral: CBPeripheral) {
        let name = peripheral.name ?? "Unknown"
        print("Connected to \(name)")
        reconnectAttempts = 0
        // Connection resolved — the BLEDevice now owns the peripheral (#173).
        connectingPeripherals[peripheral.identifier] = nil

        let device = BLEDevice(peripheral: peripheral, name: name)
        device.fleet = self
        device.onConnect()
        devices.append(device)

        if activeDeviceID == nil {
            activeDeviceID = peripheral.identifier
        }

        statusMessage = "Connected to \(name)"
    }

    func centralManager(_ central: CBCentralManager,
                       didDisconnectPeripheral peripheral: CBPeripheral,
                       error: Error?) {
        let deviceName = device(for: peripheral)?.connectedDeviceName ?? peripheral.name ?? "Unknown"
        print("Disconnected from \(deviceName)")

        // Clean up the device
        if let device = device(for: peripheral) {
            device.onDisconnect()
        }
        devices.removeAll { $0.peripheral?.identifier == peripheral.identifier }
        // No longer connecting/connected — drop the strong ref (the reconnect
        // path below re-adds it before its connect attempt) (#173).
        connectingPeripherals[peripheral.identifier] = nil

        // Update active device
        if activeDeviceID == peripheral.identifier {
            activeDeviceID = devices.first?.peripheral?.identifier
        }

        if userInitiatedDisconnect {
            userInitiatedDisconnect = false
            reconnectAttempts = 0
            statusMessage = "Disconnected"
            discoveredDevices = []
            if devices.isEmpty {
                UIApplication.shared.isIdleTimerDisabled = false
            }
            return
        }

        // Attempt automatic reconnection
        if reconnectAttempts < maxReconnectAttempts {
            reconnectAttempts += 1
            statusMessage = "Reconnecting (\(reconnectAttempts)/\(maxReconnectAttempts))..."
            let delay = pow(2.0, Double(reconnectAttempts - 1))
            DispatchQueue.main.asyncAfter(deadline: .now() + delay) { [weak self] in
                guard let self = self, self.device(for: peripheral) == nil else { return }
                // Retain across the reconnect attempt too (#173).
                self.connectingPeripherals[peripheral.identifier] = peripheral
                self.centralManager.connect(peripheral, options: nil)
            }
        } else {
            reconnectAttempts = 0
            statusMessage = "Disconnected"
            discoveredDevices = []
            if devices.isEmpty {
                UIApplication.shared.isIdleTimerDisabled = false
            }
        }
    }

    func centralManager(_ central: CBCentralManager,
                       didFailToConnect peripheral: CBPeripheral,
                       error: Error?) {
        print("Failed to connect: \(error?.localizedDescription ?? "Unknown error")")
        // Connection attempt resolved (failed) — release the strong ref (#173).
        connectingPeripherals[peripheral.identifier] = nil
        statusMessage = "Connection failed"
    }
}
