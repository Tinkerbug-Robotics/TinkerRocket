//
//  BLEManager.swift
//  TinkerRocketApp
//
//  Manages BLE connection to TinkerRocket ESP32-S3
//

import Foundation
import CoreBluetooth
import Combine
import UIKit

struct FileInfo: Identifiable, Codable, Equatable {
    let name: String
    let size: UInt32
    var id: String { name }

    // Reusable formatters — DateFormatter is expensive to allocate, and these
    // computed properties are called on every SwiftUI redraw.
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

    /// Parse UTC timestamp from filename into a Date
    /// Supports "flight_YYYYMMDD_HHMMSS.bin" and "lora_YYYYMMDD_HHMMSS.csv"
    var flightDate: Date? {
        // Both formats: "<prefix>_YYYYMMDD_HHMMSS.<ext>"
        // prefix is "flight" (7 chars) or "lora" (4 chars)
        let prefixLen: Int
        if name.hasPrefix("flight_") && name.count >= 26 {
            prefixLen = 7  // "flight_"
        } else if name.hasPrefix("lora_") && name.count >= 23 {
            prefixLen = 5  // "lora_"
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

    /// Localized display title (e.g. "2/23/26  9:34 PM"), adjusted to device timezone
    var displayTitle: String {
        guard let date = flightDate else { return name }
        return Self.localDisplayFormatter.string(from: date)
    }

    /// Kept for backward compatibility (same as displayTitle)
    var timestampDisplay: String { displayTitle }
}

enum DownloadState {
    case notDownloaded
    case downloading
    case generatingCSV
    case completed
    case failed
}

// Represents a BLE peripheral discovered during scanning
struct DiscoveredDevice: Identifiable {
    let id: UUID                    // CBPeripheral identifier
    let peripheral: CBPeripheral
    let name: String
    var rssi: Int                   // BLE signal strength (not LoRa RSSI)

    var isBaseStation: Bool {
        name.contains("Base") || name.contains("BS")
    }

    var typeLabel: String {
        isBaseStation ? "Base Station" : "Rocket"
    }
}

/// Config readback from connected rocket (sent on BLE connect + cmd 20)
/// Defaults match config.h so the app shows correct values before first readback.
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
    var cameraType: UInt8 = 2  // 0=None, 1=GoPro, 2=RunCam
    // LoRa settings (nil = not present in readback, e.g. older firmware)
    var loraFreqMHz: Float? = nil
    var loraSF: UInt8? = nil
    var loraBwKHz: Float? = nil
    var loraCR: UInt8? = nil
    var loraTxPower: Int8? = nil
    // Pyro channel configuration
    var pyro1Enabled: Bool = false
    var pyro1TriggerMode: UInt8 = 0    // 0=time_after_apogee, 1=altitude_on_descent
    var pyro1TriggerValue: Float = 1.0
    var pyro2Enabled: Bool = false
    var pyro2TriggerMode: UInt8 = 0
    var pyro2TriggerValue: Float = 100.0
}

class BLEManager: NSObject, ObservableObject {
    // Published properties that trigger UI updates
    @Published var isScanning = false
    @Published var isConnected = false
    @Published var telemetry = TelemetryData()
    @Published var statusMessage = "Not connected"
    @Published var discoveredDevices: [DiscoveredDevice] = []
    @Published var connectedDeviceName: String = ""
    @Published var files: [FileInfo] = []
    @Published var currentPage: UInt8 = 0
    @Published var hasMoreFiles = false  // True if there might be more files to load
    @Published var downloadProgress: Double = 0.0
    @Published var isDownloading = false
    @Published var downloadingFilename: String?
    @Published var csvGenerationProgress: Double = 0.0
    @Published var downloadStates: [String: DownloadState] = [:]
    @Published var simLaunched = false  // True after sim start, cleared when sim completes
    private var simSawNonReady = false  // Tracks whether sim progressed past READY state
    @Published var groundTestActive = false  // True while ground test (live sensor servo test) is running
    @Published var connectedRSSI: Int?  // BLE RSSI of connected device (updated every 2s)
    @Published var rocketConfig: RocketConfig?  // Config received from rocket on connect

    // Whether connected device is a base station (no file/camera access)
    var isBaseStation: Bool {
        connectedDeviceName.contains("Base") || connectedDeviceName.contains("BS")
    }

    // Flight voice announcer (set by DashboardView)
    var flightAnnouncer: FlightAnnouncer?

    // Reusable JSON decoder — avoid allocating on every BLE notification
    private let jsonDecoder = JSONDecoder()

    // Download state
    private var downloadExpectedSize: Int = 0
    private var downloadedData = Data()
    private var downloadCompletionHandler: ((URL?) -> Void)?
    private var downloadStallTimer: Timer?
    private var rssiTimer: Timer?

    // Reconnection state
    private var reconnectAttempts: Int = 0
    private let maxReconnectAttempts = 3
    private var lastPeripheralIdentifier: UUID?
    private var userInitiatedDisconnect = false

    // CoreBluetooth objects
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var telemetryCharacteristic: CBCharacteristic?
    private var commandCharacteristic: CBCharacteristic?
    private var fileOpsCharacteristic: CBCharacteristic?
    private var fileTransferCharacteristic: CBCharacteristic?  // New: separate characteristic for file downloads

    // UUIDs matching ESP32 (must match TR_BLE_To_APP.h)
    private let serviceUUID = CBUUID(string: "4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    private let telemetryCharUUID = CBUUID(string: "beb5483e-36e1-4688-b7f5-ea07361b26a8")
    private let commandCharUUID = CBUUID(string: "cba1d466-344c-4be3-ab3f-189f80dd7518")
    private let fileOpsCharUUID = CBUUID(string: "8d53dc1d-1db7-4cd3-868b-8a527460aa84")
    private let fileTransferCharUUID = CBUUID(string: "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d")  // New: for file chunks

    override init() {
        super.init()
        centralManager = CBCentralManager(
            delegate: self,
            queue: nil,
            options: [CBCentralManagerOptionRestoreIdentifierKey: "TinkerRocketBLE"]
        )
    }

    // Start scanning for TinkerRocket devices
    func startScanning() {
        guard centralManager.state == .poweredOn else {
            statusMessage = "Bluetooth not ready"
            return
        }

        discoveredDevices = []
        statusMessage = "Scanning..."
        isScanning = true
        centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)

        // Auto-stop scanning after 15 seconds
        DispatchQueue.main.asyncAfter(deadline: .now() + 15) { [weak self] in
            if self?.isScanning == true {
                self?.stopScanning()
                if self?.isConnected == false {
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

    // Connect to a specific discovered device
    func connect(to device: DiscoveredDevice) {
        stopScanning()
        statusMessage = "Connecting to \(device.name)..."
        self.peripheral = device.peripheral
        centralManager.connect(device.peripheral, options: nil)
    }

    func disconnect() {
        userInitiatedDisconnect = true
        if let peripheral = peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }

    // MARK: - BLE RSSI Polling

    private func startRSSITimer() {
        rssiTimer?.invalidate()
        rssiTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
            self?.peripheral?.readRSSI()
        }
    }

    private func stopRSSITimer() {
        rssiTimer?.invalidate()
        rssiTimer = nil
        connectedRSSI = nil
    }

    /// Mark sim as launched — resets the auto-clear state tracker.
    func markSimLaunched() {
        simLaunched = true
        simSawNonReady = false
    }

    /// Clear the sim banner (manual stop or sim completion).
    func clearSimBanner() {
        simLaunched = false
        simSawNonReady = false
    }

    // Send command to ESP32 (1 = camera toggle, 2 = request file list, 3 = delete file,
    // 5 = sim config, 6 = sim start, 7 = sim stop, 15 = ground test start, 16 = ground test stop,
    // 21 = gyro calibration, 22 = gain schedule enable/disable)
    func sendCommand(_ command: UInt8) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            print("Cannot send command: not connected")
            return
        }

        let data = Data([command])
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("Sent command: \(command)")
    }

    /// Toggle the power rail (PWR_PIN) on the OutComputer
    func sendPowerToggle() {
        sendCommand(8)
    }

    /// Send a raw command with payload data to the ESP32
    func sendRawCommand(_ command: UInt8, payload: Data = Data()) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            print("Cannot send command: not connected")
            return
        }

        var data = Data([command])
        data.append(payload)
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("Sent command \(command) with \(payload.count) bytes payload")
    }

    /// Send LoRa configuration to the connected device.
    /// Command 10: [cmd][freq_mhz:4f_LE][bw_khz:4f_LE][sf:1][cr:1][tx_power:1]
    func sendLoRaConfig(freqMHz: Float, bwKHz: Float, sf: UInt8, cr: UInt8, txPower: Int8) {
        var payload = Data()
        var freq = freqMHz
        var bw = bwKHz
        payload.append(Data(bytes: &freq, count: 4))
        payload.append(Data(bytes: &bw, count: 4))
        payload.append(sf)
        payload.append(cr)
        payload.append(UInt8(bitPattern: txPower))
        sendRawCommand(10, payload: payload)
        print("Sent LoRa config: \(freqMHz) MHz SF\(sf) BW\(bwKHz) CR\(cr) \(txPower) dBm")

        // Update local config immediately so SettingsView doesn't revert
        // to stale values when it re-subscribes to $rocketConfig.
        if var cfg = rocketConfig {
            cfg.loraFreqMHz = freqMHz
            cfg.loraSF = sf
            cfg.loraBwKHz = bwKHz
            cfg.loraCR = cr
            cfg.loraTxPower = txPower
            rocketConfig = cfg
        }
    }

    /// Enable or disable piezo sounds on the rocket computer.
    /// Command 11: [cmd][enabled:1]  —  relayed to FlightComputer via I2C pending command.
    func sendSoundConfig(enabled: Bool) {
        sendRawCommand(11, payload: Data([enabled ? 0x01 : 0x00]))
        print("Sent sound config: \(enabled ? "ENABLE" : "DISABLE")")
    }

    /// Send servo configuration to the rocket computer.
    /// Command 12: [cmd][bias1:i16_LE][bias2:i16_LE][bias3:i16_LE][bias4:i16_LE][hz:i16_LE][min:i16_LE][max:i16_LE]
    func sendServoConfig(biases: [Int16], hz: Int16, minUs: Int16, maxUs: Int16) {
        var payload = Data()
        for i in 0..<4 {
            var b: Int16 = i < biases.count ? biases[i] : 0
            payload.append(Data(bytes: &b, count: 2))
        }
        var h = hz;  payload.append(Data(bytes: &h, count: 2))
        var mn = minUs; payload.append(Data(bytes: &mn, count: 2))
        var mx = maxUs; payload.append(Data(bytes: &mx, count: 2))
        sendRawCommand(12, payload: payload)
        print("Sent servo config: biases=\(biases) hz=\(hz) min=\(minUs) max=\(maxUs)")

        // Update local config so SettingsView doesn't revert to stale values
        if var cfg = rocketConfig {
            cfg.servoBias1 = biases.count > 0 ? biases[0] : 0
            cfg.servoHz = hz
            cfg.servoMinUs = minUs
            cfg.servoMaxUs = maxUs
            rocketConfig = cfg
        }
    }

    /// Send PID configuration to the rocket computer.
    /// Command 13: [cmd][kp:f32_LE][ki:f32_LE][kd:f32_LE][min_cmd:f32_LE][max_cmd:f32_LE]
    func sendPIDConfig(kp: Float, ki: Float, kd: Float, minCmd: Float, maxCmd: Float) {
        var payload = Data()
        var _kp = kp, _ki = ki, _kd = kd, _min = minCmd, _max = maxCmd
        payload.append(Data(bytes: &_kp, count: 4))
        payload.append(Data(bytes: &_ki, count: 4))
        payload.append(Data(bytes: &_kd, count: 4))
        payload.append(Data(bytes: &_min, count: 4))
        payload.append(Data(bytes: &_max, count: 4))
        sendRawCommand(13, payload: payload)
        print("Sent PID config: kp=\(kp) ki=\(ki) kd=\(kd) min=\(minCmd) max=\(maxCmd)")

        // Update local config so SettingsView doesn't revert to stale values
        if var cfg = rocketConfig {
            cfg.pidKp = kp
            cfg.pidKi = ki
            cfg.pidKd = kd
            cfg.pidMinCmd = minCmd
            cfg.pidMaxCmd = maxCmd
            rocketConfig = cfg
        }
    }

    /// Enable or disable servo control on the rocket computer.
    /// Command 14: [cmd][enabled:1]
    func sendServoControlConfig(enabled: Bool) {
        sendRawCommand(14, payload: Data([enabled ? 0x01 : 0x00]))
        print("Sent servo control config: \(enabled ? "ENABLE" : "DISABLE")")

        if var cfg = rocketConfig {
            cfg.servoEnabled = enabled
            rocketConfig = cfg
        }
    }

    /// Enable or disable velocity-based gain scheduling on the rocket computer.
    /// Command 22: [cmd][enabled:1]
    func sendGainScheduleConfig(enabled: Bool) {
        sendRawCommand(22, payload: Data([enabled ? 0x01 : 0x00]))
        print("Sent gain schedule config: \(enabled ? "ENABLE" : "DISABLE")")

        if var cfg = rocketConfig {
            cfg.gainScheduleEnabled = enabled
            rocketConfig = cfg
        }
    }

    /// Send roll control config (angle control mode + activation delay).
    /// Command 31: [use_angle_control:1][pad:1][roll_delay_ms:2 LE]
    func sendRollControlConfig(useAngleControl: Bool, rollDelayMs: UInt16) {
        var payload = Data()
        payload.append(useAngleControl ? 0x01 : 0x00)
        payload.append(0x00)  // padding
        var delay = rollDelayMs
        payload.append(Data(bytes: &delay, count: 2))
        sendRawCommand(31, payload: payload)
        print("Sent roll control config: angleCtrl=\(useAngleControl) delay=\(rollDelayMs) ms")

        if var cfg = rocketConfig {
            cfg.useAngleControl = useAngleControl
            cfg.rollDelayMs = rollDelayMs
            rocketConfig = cfg
        }
    }

    /// Send servo test angles to the rocket.
    /// Command 24: [cmd][angle0_lo][angle0_hi][angle1_lo][angle1_hi][angle2_lo][angle2_hi][angle3_lo][angle3_hi]
    /// Each angle is int16_t in centi-degrees (-2000 to +2000).
    func sendServoTestAngles(_ angles: [Double]) {
        var payload = Data()
        for i in 0..<4 {
            let angle = i < angles.count ? angles[i] : 0.0
            let cdeg = Int16(clamping: Int(angle * 100.0))
            payload.append(UInt8(truncatingIfNeeded: cdeg))
            payload.append(UInt8(truncatingIfNeeded: cdeg >> 8))
        }
        sendRawCommand(24, payload: payload)
    }

    /// Stop servo test mode — servos return to midpoints.
    /// Command 25: no payload.
    func sendServoTestStop() {
        sendCommand(25)
        print("Sent servo test stop")
    }

    /// Send a roll profile to the rocket computer.
    /// Command 26: [cmd][num_wp:1][pad:3][wp0_time:4f][wp0_angle:4f]...[wp7_time:4f][wp7_angle:4f] = 68 bytes
    func sendRollProfile(waypoints: [(time: Float, angle: Float)]) {
        var payload = Data()
        let n = UInt8(min(waypoints.count, 8))
        payload.append(n)
        payload.append(contentsOf: [0, 0, 0])  // padding
        for i in 0..<8 {
            var t: Float = i < waypoints.count ? waypoints[i].time : 0.0
            var a: Float = i < waypoints.count ? waypoints[i].angle : 0.0
            payload.append(Data(bytes: &t, count: 4))
            payload.append(Data(bytes: &a, count: 4))
        }
        sendRawCommand(26, payload: payload)
        print("Sent roll profile: \(n) waypoints")
    }

    /// Clear the roll profile on the rocket (reverts to rate-only mode).
    /// Command 27: no payload.
    func sendRollProfileClear() {
        sendCommand(27)
        print("Sent roll profile clear")
    }

    /// Enable or disable PN guidance on the flight computer.
    /// Command 32: [cmd][enabled:1]
    /// When enabled: roll control during boost, full PN guidance from coast to apogee.
    /// When disabled: roll-only control during all phases.
    func sendGuidanceConfig(enabled: Bool) {
        sendRawCommand(32, payload: Data([enabled ? 0x01 : 0x00]))
        print("Sent guidance config: \(enabled ? "ENABLE" : "DISABLE")")
    }

    /// Send camera type config: 0=None, 1=GoPro, 2=RunCam
    /// Command 33: [cmd][camera_type:1]
    func sendCameraConfig(cameraType: UInt8) {
        sendRawCommand(33, payload: Data([cameraType]))
        let name = cameraType == 1 ? "GoPro" : cameraType == 2 ? "RunCam" : "None"
        print("Sent camera config: \(name)")
    }

    /// Send pyro channel configuration for both channels.
    /// Command 34: [ch1_en:1][ch1_mode:1][ch1_val:4f][ch2_en:1][ch2_mode:1][ch2_val:4f] = 12 bytes
    func sendPyroConfig(ch1Enabled: Bool, ch1Mode: UInt8, ch1Value: Float,
                        ch2Enabled: Bool, ch2Mode: UInt8, ch2Value: Float) {
        var payload = Data()
        payload.append(ch1Enabled ? 0x01 : 0x00)
        payload.append(ch1Mode)
        var v1 = ch1Value
        payload.append(Data(bytes: &v1, count: 4))
        payload.append(ch2Enabled ? 0x01 : 0x00)
        payload.append(ch2Mode)
        var v2 = ch2Value
        payload.append(Data(bytes: &v2, count: 4))
        sendRawCommand(34, payload: payload)
        print("Sent pyro config: ch1=\(ch1Enabled)/\(ch1Mode)/\(ch1Value) ch2=\(ch2Enabled)/\(ch2Mode)/\(ch2Value)")
    }

    /// Request a momentary pyro continuity test (arm → read → disarm) for one channel.
    /// Command 35: [channel:1] where channel = 1 or 2.
    /// Results come back via telemetry pyro_status bits.
    func sendPyroContTest(channel: UInt8) {
        sendRawCommand(35, payload: Data([channel]))
        print("Sent pyro continuity test for CH\(channel)")
    }

    /// Fire a pyro channel (test fire from app).
    /// Command 36: [channel:1] where channel = 1 or 2.
    func sendPyroFire(channel: UInt8) {
        sendRawCommand(36, payload: Data([channel]))
        print("Sent pyro fire for CH\(channel)")
    }

    /// Toggle flight data logging on the out computer.
    /// Command 23: no payload — toggles logger on/off.
    func sendToggleLogging() {
        sendCommand(23)
        print("Sent toggle logging")
    }

    /// Send guidance point to the flight computer for apogee steering.
    /// Command 28: [cmd][lat:f64_LE][lon:f64_LE][alt_m:f32_LE] = 21 bytes
    func sendGuidancePoint(lat: Double, lon: Double, altitudeM: Float) {
        var payload = Data()
        var latVal = lat
        var lonVal = lon
        var altVal = altitudeM
        payload.append(Data(bytes: &latVal, count: 8))
        payload.append(Data(bytes: &lonVal, count: 8))
        payload.append(Data(bytes: &altVal, count: 4))
        sendRawCommand(28, payload: payload)
        print("Sent guidance point: (\(lat), \(lon)) at \(altitudeM)m AGL")
    }

    /// Request config readback from the connected device.
    /// Command 20: triggers the device to send its current config JSON.
    func requestConfig() {
        sendCommand(20)
        print("Requested config readback")
    }

    /// Send current UTC time to the base station for timestamped log file naming.
    /// Command 9: [cmd][year_lo][year_hi][month][day][hour][minute][second]
    func sendTimeSync() {
        let now = Date()
        var calendar = Calendar(identifier: .gregorian)
        calendar.timeZone = TimeZone(identifier: "UTC")!
        let comps = calendar.dateComponents([.year, .month, .day, .hour, .minute, .second], from: now)

        guard let year = comps.year, let month = comps.month, let day = comps.day,
              let hour = comps.hour, let minute = comps.minute, let second = comps.second else {
            return
        }

        var payload = Data()
        payload.append(UInt8(year & 0xFF))          // year low byte
        payload.append(UInt8((year >> 8) & 0xFF))   // year high byte
        payload.append(UInt8(month))
        payload.append(UInt8(day))
        payload.append(UInt8(hour))
        payload.append(UInt8(minute))
        payload.append(UInt8(second))

        sendRawCommand(9, payload: payload)
        print("Sent time sync: \(year)-\(month)-\(day) \(hour):\(minute):\(second) UTC")
    }

    // Request file list from ESP32 with pagination
    func requestFileList(page: UInt8 = 0) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            print("Cannot request file list: not connected")
            return
        }

        // Send command 2 with page number
        let data = Data([2, page])
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("Sent file list request for page: \(page)")

        // Update current page
        currentPage = page

        // Read the file list after a short delay (gives ESP32 time to build it).
        // 500ms is more reliable than 200ms for large file lists.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
            guard let self = self,
                  let characteristic = self.fileOpsCharacteristic,
                  let peripheral = self.peripheral else {
                return
            }
            peripheral.readValue(for: characteristic)
        }
    }

    // Load next page of files
    func nextPage() {
        requestFileList(page: currentPage + 1)
    }

    // Load previous page of files
    func previousPage() {
        if currentPage > 0 {
            requestFileList(page: currentPage - 1)
        }
    }

    // Delete file on ESP32
    func deleteFile(_ filename: String) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            print("Cannot delete file: not connected")
            return
        }

        // Send command 3 followed by filename bytes
        var data = Data([3])
        if let filenameData = filename.data(using: .utf8) {
            data.append(filenameData)
        }

        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("Sent delete request for: \(filename)")

        // Optimistic UI update: remove file from local list immediately
        // so the UI refreshes without waiting for the BLE notification.
        // The firmware will also push an updated file list via notification.
        files.removeAll { $0.name == filename }
        downloadStates.removeValue(forKey: filename)
    }

    func downloadFile(_ filename: String, completion: @escaping (URL?) -> Void) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            print("Cannot download file: not connected")
            completion(nil)
            return
        }

        // Reset download state
        downloadingFilename = filename
        downloadedData = Data()
        downloadCompletionHandler = completion
        isDownloading = true
        downloadProgress = 0.0

        // Get expected file size from files array (search all loaded pages)
        if let fileInfo = files.first(where: { $0.name == filename }), fileInfo.size > 0 {
            downloadExpectedSize = Int(fileInfo.size)
        } else {
            // File not in loaded page or size unknown — progress will show
            // indeterminate until EOF arrives
            downloadExpectedSize = 0
            print("[DOWNLOAD] Warning: file size unknown, progress will be estimated")
        }

        // Send command 4 followed by filename bytes
        var data = Data([4])
        if let filenameData = filename.data(using: .utf8) {
            data.append(filenameData)
        }

        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("[DOWNLOAD] Started download for: \(filename), isDownloading=\(isDownloading)")
    }

    private func handleFileChunk(_ data: Data) {
        guard isDownloading else { return }  // Ignore late-arriving chunks after download completes

        // Parse chunk: [offset(4)][length(2)][flags(1)][data(N)]
        guard data.count >= 7 else {
            print("Invalid chunk: too short")
            return
        }

        // Extract offset (4 bytes, little-endian) — used for progress only
        let offset = UInt32(data[0]) | (UInt32(data[1]) << 8) |
                     (UInt32(data[2]) << 16) | (UInt32(data[3]) << 24)

        // Extract length (2 bytes, little-endian)
        let length = UInt16(data[4]) | (UInt16(data[5]) << 8)

        // Extract flags
        let flags = data[6]
        let isEOF = (flags & 0x01) != 0

        // Append chunk data (frame-aligned: each notification contains complete
        // binary frames, so just append — no offset-based placement needed).
        // A dropped BLE notification only loses whole frames, no corruption.
        if length > 0 && data.count >= 7 + Int(length) {
            let chunkData = data.subdata(in: 7..<(7 + Int(length)))
            downloadedData.append(chunkData)
        }

        // Update progress on main thread
        let received = downloadedData.count
        let expectedSize = downloadExpectedSize

        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            if expectedSize > 0 {
                self.downloadProgress = min(Double(received) / Double(expectedSize), 1.0)
            }
            if isEOF {
                self.downloadProgress = 1.0
            }
        }

        // Diagnostic: log near end of download and EOF chunks
        if isEOF || (expectedSize > 0 && Double(received) / Double(expectedSize) > 0.98) {
            print("[DOWNLOAD] chunk offset=\(offset) len=\(length) eof=\(isEOF) received=\(received)/\(expectedSize)")
        }

        // If EOF, complete the download
        if isEOF {
            downloadStallTimer?.invalidate()
            downloadStallTimer = nil
            print("[DOWNLOAD] EOF received: \(received) bytes")
            completeDownload(fromStallTimer: false)
        } else {
            // Reset stall timer — if no new chunks arrive within 3 seconds,
            // assume the EOF notification was dropped and complete with what we have.
            resetDownloadStallTimer()
        }
    }

    private func resetDownloadStallTimer() {
        // Invalidate and recreate synchronously.  BLE callbacks already run
        // on the main thread (CBCentralManager queue: nil), so there is no
        // need to dispatch.  Using DispatchQueue.main.async here would cause
        // orphaned timers when multiple chunks arrive in the same run-loop
        // pass (batched BLE notifications).
        downloadStallTimer?.invalidate()
        downloadStallTimer = Timer.scheduledTimer(withTimeInterval: 3.0, repeats: false) { [weak self] _ in
            guard let self = self, self.isDownloading else { return }
            let received = self.downloadedData.count
            let expected = self.downloadExpectedSize
            print("[DOWNLOAD] Stall timeout: no new chunks for 3s. Received \(received)/\(expected) bytes — completing")
            self.completeDownload(fromStallTimer: true)
        }
    }

    private func completeDownload(fromStallTimer: Bool = false) {
        downloadStallTimer?.invalidate()
        downloadStallTimer = nil

        guard let filename = downloadingFilename else {
            print("[DOWNLOAD] Complete called but no filename, isDownloading was \(isDownloading)")
            DispatchQueue.main.async { self.isDownloading = false }
            downloadCompletionHandler?(nil)
            return
        }

        // If triggered by the stall timer and we know the expected size,
        // check whether the download is truncated.  If so, warn and do NOT
        // cache — let the user retry instead.
        if fromStallTimer && downloadExpectedSize > 0 && downloadedData.count < downloadExpectedSize {
            let received = downloadedData.count
            let expected = downloadExpectedSize
            print("[DOWNLOAD] WARNING: Stall-timer recovery produced truncated flight log " +
                  "(\(received)/\(expected) bytes). Discarding — user should retry.")

            let handler = downloadCompletionHandler
            downloadingFilename = nil
            downloadedData = Data()
            downloadCompletionHandler = nil

            DispatchQueue.main.async { [weak self] in
                self?.isDownloading = false
            }
            handler?(nil)
            return
        }

        // Capture and reset download state BEFORE calling completion handler,
        // because the completion handler may immediately start a new download
        // (e.g., recovery file) which sets new state that we'd otherwise overwrite.
        let completedData = downloadedData
        let handler = downloadCompletionHandler
        downloadingFilename = nil
        downloadedData = Data()
        downloadCompletionHandler = nil

        // Save to temporary directory
        let tempDir = FileManager.default.temporaryDirectory
        let fileURL = tempDir.appendingPathComponent(filename)

        do {
            try completedData.write(to: fileURL)
            print("[DOWNLOAD] Complete: \(filename) (\(completedData.count) bytes) saved to \(fileURL)")

            DispatchQueue.main.async { [weak self] in
                self?.isDownloading = false
                self?.downloadProgress = 1.0
            }
            handler?(fileURL)
        } catch {
            print("[DOWNLOAD] Failed to save file: \(error)")
            DispatchQueue.main.async { [weak self] in
                self?.isDownloading = false
            }
            handler?(nil)
        }
    }

    // MARK: - CSV Generation

    func generateAndCacheCSV(
        from binaryURL: URL,
        filename: String,
        completion: @escaping (URL?) -> Void
    ) {
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else {
                DispatchQueue.main.async {
                    completion(nil)
                }
                return
            }

            do {
                // Check cache first
                if let cachedCSV = FileCache.shared.getCachedCSV(for: filename) {
                    print("[CSV] Using cached CSV for \(filename)")
                    DispatchQueue.main.async {
                        completion(cachedCSV)
                    }
                    return
                }

                print("[CSV] Generating CSV for \(filename)...")

                // Generate new CSV
                // Build clean CSV temp name: flight_xxx.bin → flight_xxx.csv
                let csvTempName: String
                if filename.hasSuffix(".bin") {
                    csvTempName = String(filename.dropLast(4)) + ".csv"
                } else {
                    csvTempName = filename + ".csv"
                }
                let tempCSV = FileManager.default.temporaryDirectory
                    .appendingPathComponent(csvTempName)

                let generator = CSVGenerator()
                let summary = try generator.generateCSV(
                    from: binaryURL,
                    to: tempCSV,
                    progressCallback: { [weak self] progress in
                        DispatchQueue.main.async {
                            self?.csvGenerationProgress = progress
                        }
                    }
                )

                print("[CSV] Generated CSV successfully, caching...")

                // Write and cache summary JSON
                let summaryTempName: String
                if filename.hasSuffix(".bin") {
                    summaryTempName = String(filename.dropLast(4)) + ".json"
                } else {
                    summaryTempName = filename + ".json"
                }
                let tempSummary = FileManager.default.temporaryDirectory
                    .appendingPathComponent(summaryTempName)
                try generator.writeSummary(summary, to: tempSummary)
                let cachedSummaryURL = try FileCache.shared.cacheSummary(
                    at: tempSummary,
                    for: filename
                )
                print("[CSV] Cached summary at \(cachedSummaryURL)")

                // Cache and return CSV
                let cachedURL = try FileCache.shared.cacheCSV(
                    at: tempCSV,
                    for: filename
                )

                print("[CSV] Cached CSV at \(cachedURL)")

                DispatchQueue.main.async {
                    completion(cachedURL)
                }
            } catch {
                print("[CSV] Generation failed: \(error)")
                DispatchQueue.main.async {
                    completion(nil)
                }
            }
        }
    }

    // MARK: - Download State Management

    /// Get the download state for a specific flight
    /// - Parameter filename: Name of the flight file
    /// - Returns: Current download state
    func getDownloadState(for filename: String) -> DownloadState {
        // Check cache first (base station uses direct CSV cache)
        if isBaseStation {
            if FileCache.shared.isDirectCSVCached(filename) {
                return .completed
            }
        } else {
            // Use size-aware cache check to handle Legacy's sequential
            // filenames (rocket_data_000.bin) — a name match alone isn't
            // enough because different flights can reuse the same name.
            if let deviceFile = files.first(where: { $0.name == filename }),
               FileCache.shared.isFlightCached(filename, expectedSize: deviceFile.size) {
                return .completed
            }
        }

        // Otherwise return tracked state (or .notDownloaded if not tracked)
        return downloadStates[filename] ?? .notDownloaded
    }

    /// Download a flight from the rocket and cache binary + CSV
    /// - Parameters:
    ///   - filename: Name of the flight file
    ///   - completion: Completion handler with success status
    func downloadAndCacheFlight(
        _ filename: String,
        completion: @escaping (Bool) -> Void
    ) {
        // Set state to downloading
        DispatchQueue.main.async {
            self.downloadStates[filename] = .downloading
        }

        // Base station: files are already CSV, download and cache directly
        if isBaseStation {
            downloadFile(filename) { [weak self] fileURL in
                guard let self = self, let fileURL = fileURL else {
                    DispatchQueue.main.async {
                        self?.downloadStates[filename] = .failed
                        completion(false)
                    }
                    return
                }

                do {
                    let _ = try FileCache.shared.cacheDirectCSV(at: fileURL, filename: filename)
                    print("[DOWNLOAD] Base station CSV cached: \(filename)")
                    DispatchQueue.main.async {
                        self.downloadStates[filename] = .completed
                        completion(true)
                    }
                } catch {
                    print("[DOWNLOAD] Failed to cache base station CSV: \(error)")
                    DispatchQueue.main.async {
                        self.downloadStates[filename] = .failed
                        completion(false)
                    }
                }
            }
            return
        }

        // Rocket: download binary, cache, generate CSV
        downloadFile(filename) { [weak self] binaryURL in
            guard let self = self, let binaryURL = binaryURL else {
                DispatchQueue.main.async {
                    self?.downloadStates[filename] = .failed
                    completion(false)
                }
                return
            }

            print("[DOWNLOAD] Binary downloaded: \(filename)")

            // Cache binary file
            let cachedBinaryURL: URL
            do {
                cachedBinaryURL = try FileCache.shared.cacheBinary(
                    at: binaryURL,
                    for: filename
                )
                print("[DOWNLOAD] Binary cached at: \(cachedBinaryURL)")
            } catch {
                print("[DOWNLOAD] Failed to cache binary: \(error)")
                DispatchQueue.main.async {
                    self.downloadStates[filename] = .failed
                    completion(false)
                }
                return
            }

            // Generate and cache CSV — use cachedBinaryURL (persistent) instead of
            // binaryURL (temp directory, may be cleaned up before CSV generation completes)
            DispatchQueue.main.async {
                self.downloadStates[filename] = .generatingCSV
                self.csvGenerationProgress = 0.0
            }
            self.generateAndCacheCSV(
                from: cachedBinaryURL,
                filename: filename
            ) { [weak self] csvURL in
                DispatchQueue.main.async {
                    if csvURL != nil {
                        print("[DOWNLOAD] CSV cached successfully")
                        self?.downloadStates[filename] = .completed
                        completion(true)
                    } else {
                        print("[DOWNLOAD] CSV generation failed")
                        self?.downloadStates[filename] = .failed
                        completion(false)
                    }
                }
            }
        }
    }

    /// Get cached flight files for sharing
    /// - Parameter filename: Name of the flight file
    /// - Returns: Array of URLs [binary, CSV, summary] or nil if not cached
    func getCachedFlightFiles(_ filename: String) -> [URL]? {
        // Base station: just return the single CSV file
        if isBaseStation {
            guard let csvURL = FileCache.shared.getCachedDirectCSV(filename) else {
                return nil
            }
            return [csvURL]
        }

        // Rocket: return binary + CSV + summary (if available)
        guard let binaryURL = FileCache.shared.getCachedBinary(for: filename),
              let csvURL = FileCache.shared.getCachedCSV(for: filename) else {
            return nil
        }

        var files = [binaryURL, csvURL]
        if let summaryURL = FileCache.shared.getCachedSummary(for: filename) {
            files.append(summaryURL)
        }
        return files
    }

    /// Rebuild download states from cached files
    /// - Parameter filenames: List of filenames to check
    func rebuildDownloadStates(for filenames: [String]) {
        DispatchQueue.main.async {
            for filename in filenames {
                if self.isBaseStation {
                    if FileCache.shared.isDirectCSVCached(filename) {
                        self.downloadStates[filename] = .completed
                    }
                } else {
                    if let deviceFile = self.files.first(where: { $0.name == filename }),
                       FileCache.shared.isFlightCached(filename, expectedSize: deviceFile.size) {
                        self.downloadStates[filename] = .completed
                    }
                }
            }
        }
    }
}

// MARK: - CBCentralManagerDelegate

extension BLEManager: CBCentralManagerDelegate {

    // MARK: - BLE State Restoration (background mode)

    func centralManager(_ central: CBCentralManager,
                       willRestoreState dict: [String: Any]) {
        // Hold peripheral reference so CoreBluetooth can report its state,
        // but do NOT mark as connected — let didConnect handle that so the
        // app always starts on the home screen.
        if let peripherals = dict[CBCentralManagerRestoredStatePeripheralsKey] as? [CBPeripheral],
           let restored = peripherals.first {
            print("[BLE] State restoration: holding reference to \(restored.name ?? "unknown")")
            self.peripheral = restored
            restored.delegate = self
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            statusMessage = "Bluetooth ready"
            // If a restored peripheral is still connected, resume the session
            if let p = peripheral, p.state == .connected {
                print("[BLE] Restored peripheral still connected — resuming")
                connectedDeviceName = p.name ?? "Restored"
                isConnected = true
                p.discoverServices(nil)
            } else {
                // Auto-start scanning on launch
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

        // Only list Tinker devices (TinkerRocket, TinkerBaseStation)
        guard name.contains("Tinker") else { return }

        // Update RSSI if already in list, otherwise add
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
        connectedDeviceName = name
        statusMessage = "Connected to \(name)"
        isConnected = true
        reconnectAttempts = 0  // Reset reconnect counter on successful connection

        // Keep screen on while connected to a rocket / base station
        UIApplication.shared.isIdleTimerDisabled = true

        peripheral.delegate = self
        peripheral.discoverServices([serviceUUID])
        startRSSITimer()
    }

    func centralManager(_ central: CBCentralManager,
                       didDisconnectPeripheral peripheral: CBPeripheral,
                       error: Error?) {
        print("Disconnected from \(connectedDeviceName)")
        stopRSSITimer()

        // Save identifier before clearing peripheral so we can attempt reconnect
        let savedIdentifier = peripheral.identifier
        lastPeripheralIdentifier = savedIdentifier

        isConnected = false
        telemetryCharacteristic = nil
        commandCharacteristic = nil
        fileOpsCharacteristic = nil
        fileTransferCharacteristic = nil
        files = []
        currentPage = 0
        hasMoreFiles = false
        clearSimBanner()
        rocketConfig = nil  // Prevent stale config from overwriting next device's settings
        flightAnnouncer?.reset()

        // Attempt automatic reconnection (up to 3 times), but not if user chose to disconnect
        if userInitiatedDisconnect {
            userInitiatedDisconnect = false
            reconnectAttempts = 0
            statusMessage = "Disconnected"
            self.peripheral = nil
            connectedDeviceName = ""
            discoveredDevices = []
            UIApplication.shared.isIdleTimerDisabled = false
            return
        }
        if reconnectAttempts < maxReconnectAttempts {
            reconnectAttempts += 1
            statusMessage = "Reconnecting (\(reconnectAttempts)/\(maxReconnectAttempts))..."
            print("[BLE] Attempting reconnect \(reconnectAttempts)/\(maxReconnectAttempts) to \(savedIdentifier)")

            // Keep peripheral reference for reconnect — connect using the same CBPeripheral
            // Exponential backoff: 1s, 2s, 4s
            self.peripheral = peripheral
            let delay = pow(2.0, Double(reconnectAttempts - 1))
            DispatchQueue.main.asyncAfter(deadline: .now() + delay) { [weak self] in
                guard let self = self, !self.isConnected else { return }
                self.centralManager.connect(peripheral, options: nil)
            }
        } else {
            // Exhausted reconnect attempts — fall back to scanning
            print("[BLE] Reconnect attempts exhausted, falling back to scan")
            reconnectAttempts = 0
            statusMessage = "Disconnected"
            self.peripheral = nil
            connectedDeviceName = ""
            discoveredDevices = []
            // Re-enable auto-lock when fully disconnected
            UIApplication.shared.isIdleTimerDisabled = false
        }
    }

    func centralManager(_ central: CBCentralManager,
                       didFailToConnect peripheral: CBPeripheral,
                       error: Error?) {
        print("Failed to connect: \(error?.localizedDescription ?? "Unknown error")")
        statusMessage = "Connection failed"
        isConnected = false
    }
}

// MARK: - CBPeripheralDelegate

extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral,
                   didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }

        for service in services {
            if service.uuid == serviceUUID {
                print("Found telemetry service")
                peripheral.discoverCharacteristics(
                    [telemetryCharUUID, commandCharUUID, fileOpsCharUUID, fileTransferCharUUID],
                    for: service
                )
            }
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didDiscoverCharacteristicsFor service: CBService,
                   error: Error?) {
        guard let characteristics = service.characteristics else { return }

        for characteristic in characteristics {
            if characteristic.uuid == telemetryCharUUID {
                print("Found telemetry characteristic")
                telemetryCharacteristic = characteristic
                // Subscribe to notifications
                peripheral.setNotifyValue(true, for: characteristic)
            } else if characteristic.uuid == commandCharUUID {
                print("Found command characteristic")
                commandCharacteristic = characteristic
                // Send phone UTC time so the rocket can timestamp log files.
                // For sim mode this provides unique filenames (the sim's
                // synthetic GNSS always produces 2025-01-01 12:00).
                sendTimeSync()
            } else if characteristic.uuid == fileOpsCharUUID {
                print("Found file ops characteristic")
                fileOpsCharacteristic = characteristic
                // Subscribe to notifications
                print("Subscribing to file ops notifications...")
                peripheral.setNotifyValue(true, for: characteristic)
            } else if characteristic.uuid == fileTransferCharUUID {
                print("Found file transfer characteristic")
                fileTransferCharacteristic = characteristic
                // Subscribe to notifications
                print("Subscribing to file transfer notifications...")
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }

        // Request config readback after a short delay (let notifications subscribe first).
        // Both rocket and base station support Command 20 — the LoRa settings in the
        // response are the device's own settings, needed so SettingsView shows correct values.
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            self?.requestConfig()
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
        guard error == nil else { return }
        let newRSSI = RSSI.intValue
        // Only publish when value actually changes — avoids triggering
        // objectWillChange (and a full DashboardView re-render) every 2s
        // when the RSSI hasn't moved.
        if connectedRSSI != newRSSI {
            connectedRSSI = newRSSI
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didUpdateNotificationStateFor characteristic: CBCharacteristic,
                   error: Error?) {
        if let error = error {
            print("Notification subscription error: \(error.localizedDescription)")
            return
        }

        if characteristic.uuid == fileOpsCharUUID {
            print("File ops notifications: \(characteristic.isNotifying ? "ENABLED" : "DISABLED")")
        } else if characteristic.uuid == fileTransferCharUUID {
            print("File transfer notifications: \(characteristic.isNotifying ? "ENABLED" : "DISABLED")")
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didUpdateValueFor characteristic: CBCharacteristic,
                   error: Error?) {
        if characteristic.uuid == telemetryCharUUID {
            parseTelemetryData(characteristic.value)
        } else if characteristic.uuid == fileOpsCharUUID {
            // File ops characteristic is for file lists only (JSON)
            if let data = characteristic.value {
                parseFileList(data)
            }
        } else if characteristic.uuid == fileTransferCharUUID {
            // File transfer characteristic is for file chunks only (binary)
            if let data = characteristic.value {
                print("[BLE] Received file chunk: \(data.count) bytes")
                handleFileChunk(data)
            }
        }
    }

    // Parse JSON telemetry from ESP32
    private func parseTelemetryData(_ data: Data?) {
        guard let data = data else { return }

        // Check for config readback JSON (sent on connect + cmd 20)
        // Config JSON has "type":"config" field, regular telemetry does not
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config" {
            var cfg = RocketConfig()
            cfg.servoBias1 = Int16(dict["sb1"] as? Int ?? Int(cfg.servoBias1))
            cfg.servoHz    = Int16(dict["shz"] as? Int ?? Int(cfg.servoHz))
            cfg.servoMinUs = Int16(dict["smn"] as? Int ?? Int(cfg.servoMinUs))
            cfg.servoMaxUs = Int16(dict["smx"] as? Int ?? Int(cfg.servoMaxUs))
            // JSON number values may come as Int, Double, or String
            cfg.pidKp     = parseFloat(dict["kp"]) ?? cfg.pidKp
            cfg.pidKi     = parseFloat(dict["ki"]) ?? cfg.pidKi
            cfg.pidKd     = parseFloat(dict["kd"]) ?? cfg.pidKd
            cfg.pidMinCmd = parseFloat(dict["pmn"]) ?? cfg.pidMinCmd
            cfg.pidMaxCmd = parseFloat(dict["pmx"]) ?? cfg.pidMaxCmd
            cfg.servoEnabled = dict["sen"] as? Bool ?? cfg.servoEnabled
            cfg.gainScheduleEnabled = dict["gs"] as? Bool ?? cfg.gainScheduleEnabled
            cfg.useAngleControl = dict["ac"] as? Bool ?? cfg.useAngleControl
            cfg.rollDelayMs = UInt16(dict["rdly"] as? Int ?? Int(cfg.rollDelayMs))
            cfg.guidanceEnabled = dict["ge"] as? Bool ?? cfg.guidanceEnabled
            cfg.cameraType = UInt8(dict["camt"] as? Int ?? Int(cfg.cameraType))
            // LoRa settings from device (nil if not present in readback)
            cfg.loraFreqMHz = parseFloat(dict["lf"])
            cfg.loraSF = (dict["lsf"] as? Int).map { UInt8($0) }
            cfg.loraBwKHz = parseFloat(dict["lbw"])
            cfg.loraCR = (dict["lcr"] as? Int).map { UInt8($0) }
            cfg.loraTxPower = (dict["lpw"] as? Int).map { Int8($0) }
            // Preserve pyro config if we already have it (from config_pyro message)
            if let existing = self.rocketConfig {
                cfg.pyro1Enabled = existing.pyro1Enabled
                cfg.pyro1TriggerMode = existing.pyro1TriggerMode
                cfg.pyro1TriggerValue = existing.pyro1TriggerValue
                cfg.pyro2Enabled = existing.pyro2Enabled
                cfg.pyro2TriggerMode = existing.pyro2TriggerMode
                cfg.pyro2TriggerValue = existing.pyro2TriggerValue
            }
            self.rocketConfig = cfg
            print("[CFG] Received config: Hz=\(cfg.servoHz) Kp=\(cfg.pidKp) Ki=\(cfg.pidKi) Kd=\(cfg.pidKd) GS=\(cfg.gainScheduleEnabled)")
            if let lf = cfg.loraFreqMHz, let lsf = cfg.loraSF, let lbw = cfg.loraBwKHz {
                print("[CFG] LoRa from device: \(lf) MHz SF\(lsf) BW\(lbw)")
            }
            return
        }

        // Pyro config readback (separate message to stay within MTU)
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_pyro" {
            var cfg = self.rocketConfig ?? RocketConfig()
            cfg.pyro1Enabled = dict["p1e"] as? Bool ?? cfg.pyro1Enabled
            cfg.pyro1TriggerMode = UInt8(dict["p1m"] as? Int ?? Int(cfg.pyro1TriggerMode))
            cfg.pyro1TriggerValue = parseFloat(dict["p1v"]) ?? cfg.pyro1TriggerValue
            cfg.pyro2Enabled = dict["p2e"] as? Bool ?? cfg.pyro2Enabled
            cfg.pyro2TriggerMode = UInt8(dict["p2m"] as? Int ?? Int(cfg.pyro2TriggerMode))
            cfg.pyro2TriggerValue = parseFloat(dict["p2v"]) ?? cfg.pyro2TriggerValue
            self.rocketConfig = cfg
            print("[CFG] Received pyro config: ch1=\(cfg.pyro1Enabled)/\(cfg.pyro1TriggerMode)/\(cfg.pyro1TriggerValue) ch2=\(cfg.pyro2Enabled)/\(cfg.pyro2TriggerMode)/\(cfg.pyro2TriggerValue)")
            return
        }

        do {
            let newTelemetry = try jsonDecoder.decode(TelemetryData.self, from: data)

            // BLE callbacks already run on the main thread (queue: nil),
            // so we update @Published properties directly.  Using
            // DispatchQueue.main.async here would allocate a closure
            // capturing self + newTelemetry on every notification (~6s),
            // which accumulates in the dispatch queue and causes steady
            // memory growth over time.

            // Clear sim banner when flight simulation completes.
            // The sim starts in READY state (waiting for GNSS/out_ready),
            // so we track whether it progressed past READY before allowing
            // auto-clear on return to READY (which signals sim completion).
            if self.simLaunched {
                if newTelemetry.state != "READY" && newTelemetry.state != "INITIALIZATION" {
                    self.simSawNonReady = true
                }
                if self.simSawNonReady && newTelemetry.state == "READY" {
                    self.simLaunched = false
                    self.simSawNonReady = false
                }
            }
            self.telemetry = newTelemetry
            self.flightAnnouncer?.processTelemetry(newTelemetry)
        } catch {
            print("Failed to parse telemetry: \(error)")
            // Print raw JSON for debugging
            if let jsonString = String(data: data, encoding: .utf8) {
                print("Raw JSON: \(jsonString)")
            }
        }
    }

    /// Helper: parse a JSON value that could be Int, Double, or String into Float
    private func parseFloat(_ value: Any?) -> Float? {
        if let d = value as? Double { return Float(d) }
        if let i = value as? Int { return Float(i) }
        if let s = value as? String { return Float(s) }
        return nil
    }

    // Parse JSON file list from ESP32
    private func parseFileList(_ data: Data?) {
        guard let data = data else { return }

        do {
            let fileList = try jsonDecoder.decode([FileInfo].self, from: data)

            // Already on main thread (CBCentralManager queue: nil)
            self.files = fileList
            // If we got 5 files (full page), there might be more on the next page
            self.hasMoreFiles = (fileList.count == 5)
            print("Received file list: \(fileList.count) files, page: \(self.currentPage)")
        } catch {
            print("Failed to parse file list: \(error)")
            // Print raw JSON for debugging
            if let jsonString = String(data: data, encoding: .utf8) {
                print("Raw file list JSON: \(jsonString)")
            }
        }
    }
}
