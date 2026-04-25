//
//  BLEDevice.swift
//  TinkerRocketApp
//
//  Per-peripheral BLE device state — telemetry, config, files, commands.
//  Extracted from BLEManager.swift for multi-device support.
//

import Foundation
import CoreBluetooth
import Combine
import UIKit

/// Device type derived from BLE name prefix or config_identity readback.
enum BLEDeviceType: String {
    case rocket = "R"
    case baseStation = "B"
    case unknown = "?"

    /// Parse from BLE advertised name (e.g. "TR-R-Atlas" → .rocket)
    static func from(name: String) -> BLEDeviceType {
        if name.hasPrefix("TR-R-") { return .rocket }
        if name.hasPrefix("TR-B-") { return .baseStation }
        // Legacy names
        if name.contains("Base") || name.contains("BS") { return .baseStation }
        if name.contains("Tinker") { return .rocket }
        return .unknown
    }
}

class BLEDevice: NSObject, ObservableObject, CBPeripheralDelegate {
    // MARK: - Published per-device state

    @Published var isConnected = false
    @Published var telemetry = TelemetryData()
    @Published var connectedDeviceName: String = ""
    @Published var files: [FileInfo] = []
    @Published var currentPage: UInt8 = 0
    @Published var hasMoreFiles = false
    @Published var downloadProgress: Double = 0.0
    @Published var isDownloading = false
    @Published var downloadingFilename: String?
    @Published var csvGenerationProgress: Double = 0.0
    @Published var downloadStates: [String: DownloadState] = [:]
    @Published var simLaunched = false
    @Published var groundTestActive = false
    @Published var connectedRSSI: Int?
    @Published var rocketConfig: RocketConfig?

    // Frequency scan state (base-station pre-launch collision avoidance)
    @Published var scanSamples: [FrequencyScanSample] = []
    @Published var isScanning: Bool = false

    /// Set once per connected-session after the first-time-seen base station
    /// has auto-picked and pushed a quiet channel.  Gates the auto-pick so it
    /// only runs once per session; cleared on disconnect so that a BS reboot
    /// (which bounces BLE) re-triggers the flow.  User-initiated rescans
    /// from FrequencyScanView go through autoApplyFrequency directly and do
    /// not touch this flag.
    @Published private(set) var hasAutoSelectedChannel: Bool = false

    /// When true, the pending scan was kicked off by triggerAutoChannelSelect
    /// and its result should be auto-applied (pick quietest + push to both
    /// sides).  User-initiated scans leave this false so results just sit in
    /// the chart for review.
    private var pendingAutoApply: Bool = false

    // MARK: - Device identity (populated from config_identity readback)

    @Published var unitID: String = ""      // e.g. "a1b2c3d4" (immutable hardware ID)
    @Published var unitName: String = ""    // e.g. "Atlas" (user-settable)
    @Published var networkID: UInt8 = 0
    @Published var rocketID: UInt8 = 0
    @Published var deviceType: BLEDeviceType = .unknown

    /// Display name: unitName if set, otherwise connectedDeviceName
    var displayName: String {
        unitName.isEmpty ? connectedDeviceName : unitName
    }

    var isBaseStation: Bool {
        deviceType == .baseStation
    }

    /// Rockets seen via this device's LoRa relay (base station only)
    @Published var remoteRockets: [RemoteRocket] = []

    // Flight voice announcer (set by DashboardView)
    var flightAnnouncer: FlightAnnouncer?

    // MARK: - Internal state

    private let jsonDecoder = JSONDecoder()
    private var simSawNonReady = false

    // Download state
    private var downloadExpectedSize: Int = 0
    private var downloadedData = Data()
    private var downloadCompletionHandler: ((URL?) -> Void)?
    private var downloadStallTimer: Timer?
    private var rssiTimer: Timer?

    // CoreBluetooth objects (peripheral is set by BLEFleet on connect)
    var peripheral: CBPeripheral?
    private var telemetryCharacteristic: CBCharacteristic?
    private var commandCharacteristic: CBCharacteristic?
    private var fileOpsCharacteristic: CBCharacteristic?
    private var fileTransferCharacteristic: CBCharacteristic?

    // UUIDs matching ESP32 (must match TR_BLE_To_APP.h)
    private let serviceUUID = CBUUID(string: "4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    private let telemetryCharUUID = CBUUID(string: "beb5483e-36e1-4688-b7f5-ea07361b26a8")
    private let commandCharUUID = CBUUID(string: "cba1d466-344c-4be3-ab3f-189f80dd7518")
    private let fileOpsCharUUID = CBUUID(string: "8d53dc1d-1db7-4cd3-868b-8a527460aa84")
    private let fileTransferCharUUID = CBUUID(string: "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d")

    // MARK: - Init

    init(peripheral: CBPeripheral, name: String) {
        self.connectedDeviceName = name
        self.deviceType = BLEDeviceType.from(name: name)
        super.init()
        self.peripheral = peripheral
        peripheral.delegate = self
    }

    // MARK: - Connection lifecycle (called by BLEFleet)

    func onConnect() {
        isConnected = true
        UIApplication.shared.isIdleTimerDisabled = true
        peripheral?.discoverServices([serviceUUID])
        startRSSITimer()
    }

    func onDisconnect() {
        isConnected = false
        stopRSSITimer()
        telemetryCharacteristic = nil
        commandCharacteristic = nil
        fileOpsCharacteristic = nil
        fileTransferCharacteristic = nil
        files = []
        currentPage = 0
        hasMoreFiles = false
        clearSimBanner()
        rocketConfig = nil
        // Reset so the next reconnect (including after a BS reboot) triggers
        // another auto-pick.  Any pending scan-and-apply is also dropped —
        // a scan kicked off just before disconnect would not be able to
        // finish anyway.
        hasAutoSelectedChannel = false
        pendingAutoApply = false
        flightAnnouncer?.reset()
        UIApplication.shared.isIdleTimerDisabled = false
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

    // MARK: - Sim state

    func markSimLaunched() {
        simLaunched = true
        simSawNonReady = false
    }

    func clearSimBanner() {
        simLaunched = false
        simSawNonReady = false
    }

    // MARK: - Commands

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

    func sendPowerToggle() {
        sendCommand(8)
    }

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

    /// Kick off a base-station frequency scan.  Clears any previous results
    /// and flips `isScanning` true; the result arrives asynchronously on the
    /// FILE_OPS characteristic and resets `isScanning` when parsed.
    func startFrequencyScan(startMHz: Float, stopMHz: Float, stepKHz: UInt16, dwellMs: UInt16) {
        var payload = Data()
        var start = startMHz
        var stop  = stopMHz
        var step  = stepKHz
        var dwell = dwellMs
        payload.append(Data(bytes: &start, count: 4))
        payload.append(Data(bytes: &stop,  count: 4))
        payload.append(Data(bytes: &step,  count: 2))
        payload.append(Data(bytes: &dwell, count: 2))
        scanSamples = []
        isScanning = true
        sendRawCommand(60, payload: payload)
    }

    /// If this base-station session has not auto-picked a channel yet, and
    /// we now have both a config readback and at least one tracked rocket,
    /// kick off a scan-and-apply cycle so the link lands on the quietest
    /// channel automatically.  Safe to call as often as needed — it no-ops
    /// after the first successful trigger.
    ///
    /// Rocket must be on-air because the config change is relayed over the
    /// existing LoRa link; if the rocket has not yet been heard, the BS has
    /// no way to tell it about the new channel and would strand it.
    func triggerAutoChannelSelectIfNeeded() {
        // Same gating as manual apply — no point scanning if we couldn't
        // apply the result anyway.
        guard !hasAutoSelectedChannel,
              !isScanning,
              !pendingAutoApply,
              autoApplyRefusalReason() == nil else { return }

        hasAutoSelectedChannel = true
        pendingAutoApply = true

        // 5 s breathing room so the user sees "Connected" before the scan
        // interrupts telemetry for ~1.6 s.  Without this delay the dashboard
        // flashes into view and immediately stalls, which looks broken.
        DispatchQueue.main.asyncAfter(deadline: .now() + 5.0) { [weak self] in
            guard let self = self,
                  self.pendingAutoApply,
                  self.isConnected,
                  !self.remoteRockets.isEmpty else { return }
            print("[FREQ] Auto-selecting channel (first-time session)")
            self.startFrequencyScan(startMHz: 902.0, stopMHz: 928.0, stepKHz: 500, dwellMs: 30)
        }
    }

    /// Reasons an auto-apply can be refused, surfaced to the UI so the user
    /// gets a concrete next step instead of a silent no-op.
    enum AutoApplyRefusal: String {
        case notBaseStation       = "Connect to the base station first."
        case notConnected         = "Base station is not connected over BLE."
        case configMissing        = "Waiting for base-station config readback."
        case noRocketPresent      = "No rocket has beaconed recently — power it on and wait for it to show up."
    }

    /// Maximum age of the most-recent rocket beacon for auto-apply to be
    /// allowed.  The whole point of the gating is to prevent the app from
    /// pushing a new frequency to a base station while the rocket is off or
    /// out of range — doing so strands the rocket on the old channel.
    /// 10 s comfortably covers the ~2 Hz beacon cadence.
    static let autoApplyMaxBeaconAgeSeconds: TimeInterval = 10.0

    /// Pure decision logic for whether auto-apply should proceed.  Lives
    /// here as a static so it can be unit-tested without standing up a
    /// CoreBluetooth peripheral, and so the rules stay in one place
    /// rather than getting duplicated between this method and the view.
    static func autoApplyRefusalReason(
        isBaseStation: Bool,
        isConnected: Bool,
        config: RocketConfig?,
        rocketLastSeenTimes: [Date],
        now: Date = Date()
    ) -> AutoApplyRefusal? {
        if !isBaseStation { return .notBaseStation }
        if !isConnected   { return .notConnected }
        guard let cfg = config,
              cfg.loraBwKHz != nil, cfg.loraSF != nil,
              cfg.loraCR != nil, cfg.loraTxPower != nil else {
            return .configMissing
        }
        // At least one tracked rocket beaconed within the freshness window.
        // Beacons fire at ~0.5 Hz in READY/PRELAUNCH/INIT, so "silent > 10 s"
        // is a strong signal the rocket isn't on-air.
        let cutoff = now.addingTimeInterval(-autoApplyMaxBeaconAgeSeconds)
        let haveFreshRocket = rocketLastSeenTimes.contains { $0 >= cutoff }
        if !haveFreshRocket { return .noRocketPresent }
        return nil
    }

    /// Returns `.none` if auto-apply is currently allowed, otherwise the
    /// specific reason it's being refused.  Called by the Frequency Scan
    /// view to decide whether to enable the Apply button and what message
    /// to surface if it's disabled.
    func autoApplyRefusalReason() -> AutoApplyRefusal? {
        return Self.autoApplyRefusalReason(
            isBaseStation: isBaseStation,
            isConnected:   isConnected,
            config:        rocketConfig,
            rocketLastSeenTimes: remoteRockets.map { $0.lastSeen }
        )
    }

    /// Relay a LoRa reconfig to every tracked rocket, then apply the same
    /// config to this base station after the uplink retries have had time
    /// to land.  Keeps SF/BW/CR/power from the current base-station config
    /// — only the frequency changes.
    ///
    /// Refuses unless `autoApplyRefusalReason()` returns nil.  The base
    /// station's transactional Cmd 10 handler (issue #71) will commit the
    /// new frequency only after verifying the rocket joined the new
    /// channel, so a missed relay rolls back instead of stranding.
    @discardableResult
    func autoApplyFrequency(_ freqMHz: Float) -> Bool {
        if let refusal = autoApplyRefusalReason() {
            print("[FREQ] Auto-apply refused: \(refusal.rawValue)")
            return false
        }
        guard let cfg = rocketConfig,
              let bw = cfg.loraBwKHz,
              let sf = cfg.loraSF,
              let cr = cfg.loraCR,
              let pwr = cfg.loraTxPower else { return false }

        // Send a single Cmd 10 to the base station carrying the new config.
        // The base station (firmware) now owns the relay + verify handshake:
        // it issues Cmd 50 → inner Cmd 10 uplink to every tracked rocket,
        // switches to the new frequency, listens for a beacon on the new
        // channel, and either commits or rolls back atomically.  The app
        // no longer has to coordinate the 2-step dance.
        sendLoRaConfig(freqMHz: freqMHz, bwKHz: bw, sf: sf, cr: cr, txPower: pwr)
        return true
    }

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
        if var cfg = rocketConfig {
            cfg.loraFreqMHz = freqMHz
            cfg.loraSF = sf
            cfg.loraBwKHz = bwKHz
            cfg.loraCR = cr
            cfg.loraTxPower = txPower
            rocketConfig = cfg
        }
    }

    func sendSoundConfig(enabled: Bool) {
        sendRawCommand(11, payload: Data([enabled ? 0x01 : 0x00]))
    }

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
        if var cfg = rocketConfig {
            cfg.servoBias1 = biases.count > 0 ? biases[0] : 0
            cfg.servoHz = hz
            cfg.servoMinUs = minUs
            cfg.servoMaxUs = maxUs
            rocketConfig = cfg
        }
    }

    func sendPIDConfig(kp: Float, ki: Float, kd: Float, minCmd: Float, maxCmd: Float) {
        var payload = Data()
        var _kp = kp, _ki = ki, _kd = kd, _min = minCmd, _max = maxCmd
        payload.append(Data(bytes: &_kp, count: 4))
        payload.append(Data(bytes: &_ki, count: 4))
        payload.append(Data(bytes: &_kd, count: 4))
        payload.append(Data(bytes: &_min, count: 4))
        payload.append(Data(bytes: &_max, count: 4))
        sendRawCommand(13, payload: payload)
        if var cfg = rocketConfig {
            cfg.pidKp = kp
            cfg.pidKi = ki
            cfg.pidKd = kd
            cfg.pidMinCmd = minCmd
            cfg.pidMaxCmd = maxCmd
            rocketConfig = cfg
        }
    }

    func sendServoControlConfig(enabled: Bool) {
        sendRawCommand(14, payload: Data([enabled ? 0x01 : 0x00]))
        if var cfg = rocketConfig {
            cfg.servoEnabled = enabled
            rocketConfig = cfg
        }
    }

    func sendGainScheduleConfig(enabled: Bool) {
        sendRawCommand(22, payload: Data([enabled ? 0x01 : 0x00]))
        if var cfg = rocketConfig {
            cfg.gainScheduleEnabled = enabled
            rocketConfig = cfg
        }
    }

    func sendRollControlConfig(useAngleControl: Bool, rollDelayMs: UInt16) {
        var payload = Data()
        payload.append(useAngleControl ? 0x01 : 0x00)
        payload.append(0x00)
        var delay = rollDelayMs
        payload.append(Data(bytes: &delay, count: 2))
        sendRawCommand(31, payload: payload)
        if var cfg = rocketConfig {
            cfg.useAngleControl = useAngleControl
            cfg.rollDelayMs = rollDelayMs
            rocketConfig = cfg
        }
    }

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

    func sendServoTestStop() {
        sendCommand(25)
    }

    func sendRollProfile(waypoints: [(time: Float, angle: Float)]) {
        var payload = Data()
        let n = UInt8(min(waypoints.count, 8))
        payload.append(n)
        payload.append(contentsOf: [0, 0, 0])
        for i in 0..<8 {
            var t: Float = i < waypoints.count ? waypoints[i].time : 0.0
            var a: Float = i < waypoints.count ? waypoints[i].angle : 0.0
            payload.append(Data(bytes: &t, count: 4))
            payload.append(Data(bytes: &a, count: 4))
        }
        sendRawCommand(26, payload: payload)
    }

    func sendRollProfileClear() {
        sendCommand(27)
    }

    func sendGuidanceConfig(enabled: Bool) {
        sendRawCommand(32, payload: Data([enabled ? 0x01 : 0x00]))
    }

    func sendCameraConfig(cameraType: UInt8) {
        sendRawCommand(33, payload: Data([cameraType]))
    }

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
    }

    func sendPyroContTest(channel: UInt8) {
        sendRawCommand(35, payload: Data([channel]))
    }

    func sendPyroFire(channel: UInt8) {
        sendRawCommand(36, payload: Data([channel]))
    }

    func sendToggleLogging() {
        sendCommand(23)
    }

    func sendGuidancePoint(lat: Double, lon: Double, altitudeM: Float) {
        var payload = Data()
        var latVal = lat
        var lonVal = lon
        var altVal = altitudeM
        payload.append(Data(bytes: &latVal, count: 8))
        payload.append(Data(bytes: &lonVal, count: 8))
        payload.append(Data(bytes: &altVal, count: 4))
        sendRawCommand(28, payload: payload)
    }

    func requestConfig() {
        sendCommand(20)
    }

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
        payload.append(UInt8(year & 0xFF))
        payload.append(UInt8((year >> 8) & 0xFF))
        payload.append(UInt8(month))
        payload.append(UInt8(day))
        payload.append(UInt8(hour))
        payload.append(UInt8(minute))
        payload.append(UInt8(second))
        sendRawCommand(9, payload: payload)
    }

    // MARK: - Identity commands

    func sendSetUnitName(_ name: String) {
        guard let data = name.prefix(20).data(using: .utf8) else { return }
        sendRawCommand(40, payload: data)
    }

    func sendSetNetworkID(_ nid: UInt8) {
        sendRawCommand(41, payload: Data([nid]))
    }

    func sendSetRocketID(_ rid: UInt8) {
        sendRawCommand(42, payload: Data([rid]))
    }

    /// Relay a command to a specific rocket via this base station.
    /// Command 50: [target_rid:1][inner_cmd:1][inner_payload:0..18]
    /// The base station unpacks this and queues a LoRa uplink.
    func sendRelayCommand(targetRocketID: UInt8, innerCommand: UInt8, innerPayload: Data = Data()) {
        var payload = Data([targetRocketID, innerCommand])
        payload.append(innerPayload.prefix(18))
        sendRawCommand(50, payload: payload)
    }

    // MARK: - File operations

    func requestFileList(page: UInt8 = 0) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else { return }
        let data = Data([2, page])
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        currentPage = page
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
            guard let self = self,
                  let characteristic = self.fileOpsCharacteristic,
                  let peripheral = self.peripheral else { return }
            peripheral.readValue(for: characteristic)
        }
    }

    func nextPage() { requestFileList(page: currentPage + 1) }
    func previousPage() { if currentPage > 0 { requestFileList(page: currentPage - 1) } }

    func deleteFile(_ filename: String) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else { return }
        var data = Data([3])
        if let filenameData = filename.data(using: .utf8) {
            data.append(filenameData)
        }
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        files.removeAll { $0.name == filename }
        downloadStates.removeValue(forKey: filename)
    }

    func downloadFile(_ filename: String, completion: @escaping (URL?) -> Void) {
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            completion(nil)
            return
        }
        downloadingFilename = filename
        downloadedData = Data()
        downloadCompletionHandler = completion
        isDownloading = true
        downloadProgress = 0.0
        if let fileInfo = files.first(where: { $0.name == filename }), fileInfo.size > 0 {
            downloadExpectedSize = Int(fileInfo.size)
        } else {
            downloadExpectedSize = 0
        }
        var data = Data([4])
        if let filenameData = filename.data(using: .utf8) {
            data.append(filenameData)
        }
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
    }

    // MARK: - File chunk handling (private)

    private func handleFileChunk(_ data: Data) {
        guard isDownloading else { return }
        guard data.count >= 7 else { return }
        let offset = UInt32(data[0]) | (UInt32(data[1]) << 8) |
                     (UInt32(data[2]) << 16) | (UInt32(data[3]) << 24)
        let length = UInt16(data[4]) | (UInt16(data[5]) << 8)
        let flags = data[6]
        let isEOF = (flags & 0x01) != 0
        if length > 0 && data.count >= 7 + Int(length) {
            let chunkData = data.subdata(in: 7..<(7 + Int(length)))
            downloadedData.append(chunkData)
        }
        let received = downloadedData.count
        let expectedSize = downloadExpectedSize
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            if expectedSize > 0 {
                self.downloadProgress = min(Double(received) / Double(expectedSize), 1.0)
            }
            if isEOF { self.downloadProgress = 1.0 }
        }
        if isEOF || (expectedSize > 0 && Double(received) / Double(expectedSize) > 0.98) {
            print("[DOWNLOAD] chunk offset=\(offset) len=\(length) eof=\(isEOF) received=\(received)/\(expectedSize)")
        }
        if isEOF {
            downloadStallTimer?.invalidate()
            downloadStallTimer = nil
            completeDownload(fromStallTimer: false)
        } else {
            resetDownloadStallTimer()
        }
    }

    private func resetDownloadStallTimer() {
        downloadStallTimer?.invalidate()
        downloadStallTimer = Timer.scheduledTimer(withTimeInterval: 3.0, repeats: false) { [weak self] _ in
            guard let self = self, self.isDownloading else { return }
            self.completeDownload(fromStallTimer: true)
        }
    }

    private func completeDownload(fromStallTimer: Bool = false) {
        downloadStallTimer?.invalidate()
        downloadStallTimer = nil
        guard let filename = downloadingFilename else {
            DispatchQueue.main.async { self.isDownloading = false }
            downloadCompletionHandler?(nil)
            return
        }
        if fromStallTimer && downloadExpectedSize > 0 && downloadedData.count < downloadExpectedSize {
            let handler = downloadCompletionHandler
            downloadingFilename = nil
            downloadedData = Data()
            downloadCompletionHandler = nil
            DispatchQueue.main.async { [weak self] in self?.isDownloading = false }
            handler?(nil)
            return
        }
        let completedData = downloadedData
        let handler = downloadCompletionHandler
        downloadingFilename = nil
        downloadedData = Data()
        downloadCompletionHandler = nil
        let tempDir = FileManager.default.temporaryDirectory
        let fileURL = tempDir.appendingPathComponent(filename)
        do {
            try completedData.write(to: fileURL)
            DispatchQueue.main.async { [weak self] in
                self?.isDownloading = false
                self?.downloadProgress = 1.0
            }
            handler?(fileURL)
        } catch {
            DispatchQueue.main.async { [weak self] in self?.isDownloading = false }
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
                DispatchQueue.main.async { completion(nil) }
                return
            }
            do {
                if let cachedCSV = FileCache.shared.getCachedCSV(for: filename) {
                    DispatchQueue.main.async { completion(cachedCSV) }
                    return
                }
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
                        DispatchQueue.main.async { self?.csvGenerationProgress = progress }
                    }
                )
                let summaryTempName: String
                if filename.hasSuffix(".bin") {
                    summaryTempName = String(filename.dropLast(4)) + ".json"
                } else {
                    summaryTempName = filename + ".json"
                }
                let tempSummary = FileManager.default.temporaryDirectory
                    .appendingPathComponent(summaryTempName)
                try generator.writeSummary(summary, to: tempSummary)
                let _ = try FileCache.shared.cacheSummary(at: tempSummary, for: filename)
                let cachedURL = try FileCache.shared.cacheCSV(at: tempCSV, for: filename)
                DispatchQueue.main.async { completion(cachedURL) }
            } catch {
                DispatchQueue.main.async { completion(nil) }
            }
        }
    }

    // MARK: - Download State Management

    func getDownloadState(for filename: String) -> DownloadState {
        if isBaseStation {
            if FileCache.shared.isDirectCSVCached(filename) { return .completed }
        } else {
            if let deviceFile = files.first(where: { $0.name == filename }),
               FileCache.shared.isFlightCached(filename, expectedSize: deviceFile.size) {
                return .completed
            }
        }
        return downloadStates[filename] ?? .notDownloaded
    }

    func downloadAndCacheFlight(_ filename: String, completion: @escaping (Bool) -> Void) {
        DispatchQueue.main.async { self.downloadStates[filename] = .downloading }
        if isBaseStation {
            downloadFile(filename) { [weak self] fileURL in
                guard let self = self, let fileURL = fileURL else {
                    DispatchQueue.main.async { self?.downloadStates[filename] = .failed; completion(false) }
                    return
                }
                do {
                    let _ = try FileCache.shared.cacheDirectCSV(at: fileURL, filename: filename)
                    DispatchQueue.main.async { self.downloadStates[filename] = .completed; completion(true) }
                } catch {
                    DispatchQueue.main.async { self.downloadStates[filename] = .failed; completion(false) }
                }
            }
            return
        }
        downloadFile(filename) { [weak self] binaryURL in
            guard let self = self, let binaryURL = binaryURL else {
                DispatchQueue.main.async { self?.downloadStates[filename] = .failed; completion(false) }
                return
            }
            let cachedBinaryURL: URL
            do {
                cachedBinaryURL = try FileCache.shared.cacheBinary(at: binaryURL, for: filename)
            } catch {
                DispatchQueue.main.async { self.downloadStates[filename] = .failed; completion(false) }
                return
            }
            DispatchQueue.main.async { self.downloadStates[filename] = .generatingCSV; self.csvGenerationProgress = 0.0 }
            self.generateAndCacheCSV(from: cachedBinaryURL, filename: filename) { [weak self] csvURL in
                DispatchQueue.main.async {
                    if csvURL != nil {
                        self?.downloadStates[filename] = .completed; completion(true)
                    } else {
                        self?.downloadStates[filename] = .failed; completion(false)
                    }
                }
            }
        }
    }

    func getCachedFlightFiles(_ filename: String) -> [URL]? {
        if isBaseStation {
            guard let csvURL = FileCache.shared.getCachedDirectCSV(filename) else { return nil }
            return [csvURL]
        }
        guard let binaryURL = FileCache.shared.getCachedBinary(for: filename),
              let csvURL = FileCache.shared.getCachedCSV(for: filename) else { return nil }
        var files = [binaryURL, csvURL]
        if let summaryURL = FileCache.shared.getCachedSummary(for: filename) { files.append(summaryURL) }
        return files
    }

    func rebuildDownloadStates(for filenames: [String]) {
        DispatchQueue.main.async {
            for filename in filenames {
                if self.isBaseStation {
                    if FileCache.shared.isDirectCSVCached(filename) { self.downloadStates[filename] = .completed }
                } else {
                    if let deviceFile = self.files.first(where: { $0.name == filename }),
                       FileCache.shared.isFlightCached(filename, expectedSize: deviceFile.size) {
                        self.downloadStates[filename] = .completed
                    }
                }
            }
        }
    }

    // MARK: - CBPeripheralDelegate

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services {
            if service.uuid == serviceUUID {
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
                telemetryCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            } else if characteristic.uuid == commandCharUUID {
                commandCharacteristic = characteristic
                sendTimeSync()
            } else if characteristic.uuid == fileOpsCharUUID {
                fileOpsCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            } else if characteristic.uuid == fileTransferCharUUID {
                fileTransferCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            self?.requestConfig()
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
        guard error == nil else { return }
        let newRSSI = RSSI.intValue
        if connectedRSSI != newRSSI { connectedRSSI = newRSSI }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didUpdateNotificationStateFor characteristic: CBCharacteristic,
                   error: Error?) {
        if let error = error {
            print("Notification subscription error: \(error.localizedDescription)")
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didUpdateValueFor characteristic: CBCharacteristic,
                   error: Error?) {
        if characteristic.uuid == telemetryCharUUID {
            parseTelemetryData(characteristic.value)
        } else if characteristic.uuid == fileOpsCharUUID {
            if let data = characteristic.value {
                // Scan results use a 0xAA binary prefix; JSON responses start
                // with '{' or '[' so the first byte is enough to disambiguate.
                if data.first == 0xAA {
                    parseScanResult(data)
                } else {
                    parseFileList(data)
                }
            }
        } else if characteristic.uuid == fileTransferCharUUID {
            if let data = characteristic.value { handleFileChunk(data) }
        }
    }

    // MARK: - Telemetry parsing

    private func parseTelemetryData(_ data: Data?) {
        guard let data = data else { return }

        // Config readback: "type":"config"
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config" {
            var cfg = RocketConfig()
            cfg.servoBias1 = Int16(dict["sb1"] as? Int ?? Int(cfg.servoBias1))
            cfg.servoHz    = Int16(dict["shz"] as? Int ?? Int(cfg.servoHz))
            cfg.servoMinUs = Int16(dict["smn"] as? Int ?? Int(cfg.servoMinUs))
            cfg.servoMaxUs = Int16(dict["smx"] as? Int ?? Int(cfg.servoMaxUs))
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
            cfg.loraFreqMHz = parseFloat(dict["lf"])
            cfg.loraSF = (dict["lsf"] as? Int).map { UInt8($0) }
            cfg.loraBwKHz = parseFloat(dict["lbw"])
            cfg.loraCR = (dict["lcr"] as? Int).map { UInt8($0) }
            cfg.loraTxPower = (dict["lpw"] as? Int).map { Int8($0) }
            if let existing = self.rocketConfig {
                cfg.pyro1Enabled = existing.pyro1Enabled
                cfg.pyro1TriggerMode = existing.pyro1TriggerMode
                cfg.pyro1TriggerValue = existing.pyro1TriggerValue
                cfg.pyro2Enabled = existing.pyro2Enabled
                cfg.pyro2TriggerMode = existing.pyro2TriggerMode
                cfg.pyro2TriggerValue = existing.pyro2TriggerValue
            }
            self.rocketConfig = cfg
            triggerAutoChannelSelectIfNeeded()
            return
        }

        // Pyro config readback: "type":"config_pyro"
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
            return
        }

        // Identity readback: "type":"config_identity"
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_identity" {
            if let uid = dict["uid"] as? String { unitID = uid }
            if let un = dict["un"] as? String { unitName = un }
            if let nid = dict["nid"] as? Int { networkID = UInt8(nid) }
            if let rid = dict["rid"] as? Int { rocketID = UInt8(rid) }
            if let dt = dict["dt"] as? String {
                deviceType = BLEDeviceType(rawValue: dt) ?? deviceType
            }
            print("[CFG] Identity: uid=\(unitID) name=\(unitName) nid=\(networkID) rid=\(rocketID) type=\(deviceType.rawValue)")
            return
        }

        // Regular telemetry
        do {
            let newTelemetry = try jsonDecoder.decode(TelemetryData.self, from: data)

            // If telemetry has a source_rocket_id, it's relayed via base station
            // → route to RemoteRocket instead of updating our own telemetry
            if let rid = newTelemetry.source_rocket_id, rid > 0, isBaseStation,
               let bsID = peripheral?.identifier {
                let rocketID = UInt8(rid)
                if let existing = remoteRockets.first(where: { $0.rocketID == rocketID }) {
                    existing.updateTelemetry(newTelemetry, unitName: newTelemetry.source_unit_name)
                } else {
                    let remote = RemoteRocket(
                        baseStationDeviceID: bsID,
                        rocketID: rocketID,
                        unitName: newTelemetry.source_unit_name ?? ""
                    )
                    remote.telemetry = newTelemetry
                    remoteRockets.append(remote)
                    print("[BS] New remote rocket: rid=\(rocketID) name=\(remote.unitName)")
                    triggerAutoChannelSelectIfNeeded()
                }
                // Still update base station's own telemetry for BS-specific fields
                // (battery, logging state) but don't overwrite rocket telemetry
                self.telemetry = newTelemetry
                return
            }

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
            if let jsonString = String(data: data, encoding: .utf8) {
                print("Raw JSON: \(jsonString)")
            }
        }
    }

    private func parseFloat(_ value: Any?) -> Float? {
        if let d = value as? Double { return Float(d) }
        if let i = value as? Int { return Float(i) }
        if let s = value as? String { return Float(s) }
        return nil
    }

    /// Parse the base-station scan-result binary blob.
    /// Format: [0xAA][start_mhz f32][step_khz f32][n u8][rssi i8 × n]
    /// Floats live at unaligned offsets (1 and 5), so we use `loadUnaligned`
    /// — plain `load` there is undefined behaviour and was silently failing.
    /// We copy into a `[UInt8]` first so indexing is always zero-based, which
    /// avoids the `Data` slice-startIndex footgun if CoreBluetooth ever hands
    /// us a sliced buffer.
    private func parseScanResult(_ data: Data) {
        print("[SCAN] parseScanResult: \(data.count) bytes, first=\(data.first.map { String(format: "0x%02X", $0) } ?? "nil")")
        let bytes = [UInt8](data)
        guard bytes.count >= 10, bytes[0] == 0xAA else {
            print("[SCAN] Malformed scan result (len=\(bytes.count))")
            isScanning = false
            return
        }
        let start: Float = bytes.withUnsafeBufferPointer {
            UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: 1, as: Float.self)
        }
        let stepKHz: Float = bytes.withUnsafeBufferPointer {
            UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: 5, as: Float.self)
        }
        let n = Int(bytes[9])
        guard bytes.count >= 10 + n else {
            print("[SCAN] Truncated scan result: need \(10 + n) bytes, got \(bytes.count)")
            isScanning = false
            return
        }
        var samples: [FrequencyScanSample] = []
        samples.reserveCapacity(n)
        for i in 0..<n {
            let rssi = Int8(bitPattern: bytes[10 + i])
            let freq = start + (stepKHz * Float(i)) / 1000.0
            samples.append(FrequencyScanSample(freqMHz: freq, rssiDbm: Int(rssi)))
        }
        scanSamples = samples
        isScanning = false
        print("[SCAN] Parsed \(n) samples, start=\(start) MHz step=\(stepKHz) kHz")

        // If this scan was kicked off by the automatic first-time-connection
        // flow, apply the quietest channel now.  User-initiated scans leave
        // pendingAutoApply false, so the chart just sits there for review.
        if pendingAutoApply {
            pendingAutoApply = false
            if let quiet = samples.min(by: { $0.rssiDbm < $1.rssiDbm }) {
                print("[FREQ] Auto-applying quietest channel: \(quiet.freqMHz) MHz (\(quiet.rssiDbm) dBm)")
                _ = autoApplyFrequency(quiet.freqMHz)
            }
        }
    }

    private func parseFileList(_ data: Data?) {
        guard let data = data else { return }
        do {
            let fileList = try jsonDecoder.decode([FileInfo].self, from: data)
            self.files = fileList
            self.hasMoreFiles = (fileList.count == 5)
        } catch {
            print("Failed to parse file list: \(error)")
        }
    }
}
