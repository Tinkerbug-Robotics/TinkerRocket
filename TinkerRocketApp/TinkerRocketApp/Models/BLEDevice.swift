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
    // #159: drives the Power On button's busy state.  The OC can spend
    // seconds-to-tens-of-seconds flushing the flight log before it acts on
    // cmd 8, during which the press otherwise looks lost.
    @Published var poweringOn = false
    @Published var connectedRSSI: Int?
    @Published var rocketConfig: RocketConfig?

    // Frequency scan state (base-station pre-launch collision avoidance)
    @Published var scanSamples: [FrequencyScanSample] = []
    @Published var isScanning: Bool = false

    // Magnetometer hard-iron calibration status (issue #96).  Latest frame
    // received from the FC over BLE on the file_ops characteristic with a
    // 0xCA discriminator.  nil until the first frame arrives; reset on
    // disconnect so a stale REVIEW state doesn't bleed across sessions.
    @Published var magCalStatus: MagCalStatus?

    /// Latest sensor (gyro + high-g) cal readback from the FC, received on
    /// the file_ops characteristic behind a 0xCB discriminator (#132).
    @Published var sensorCalStatus: SensorCalStatus?

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

    /// Firmware version stamp from config_identity "fw" field (#8).
    /// Format: `<git_short_sha>+<build_yyyymmdd-hhmm>`, optionally with
    /// `-dirty` between the sha and `+`. Empty if the device hasn't
    /// pushed its identity yet or is running pre-#8 firmware (no "fw" field).
    @Published var firmwareVersion: String = ""

    /// Flight Computer firmware version, relayed by the OUT computer from the FC
    /// over I2C (config "fc_identity" / "fc_fw"; #8 Phase 4). Same format as
    /// `firmwareVersion`. Empty until the OC has relayed it (or "unknown" if the
    /// OC can't reach the FC). For an *FC*-targeted OTA, rollback detection must
    /// compare THIS — the connected device's `firmwareVersion` is the OC's and
    /// never changes when only the FC is updated.
    @Published var fcFirmwareVersion: String = ""

    /// Latest OTA status frame from the device (#8 phase 2). Driven by
    /// ota_status JSON notifications on the file-ops characteristic.
    /// nil between sessions; OTASession observes this to advance its
    /// state machine.
    @Published var otaStatus: OTAStatusUpdate?

    /// FC's active board→rocket mounting orientation ("imu_orient" config
    /// message, relayed by the OC). Display name like "+X" or "-Z r90" plus
    /// how it was determined. Lets the flyer confirm the auto-detected
    /// mounting before arming. Empty until a v3-orientation FC reports it.
    @Published var imuOrientationName: String = ""
    @Published var imuOrientationMode: IMUOrientationMode = .unknown

    /// Display name: unitName if set, otherwise connectedDeviceName
    var displayName: String {
        unitName.isEmpty ? connectedDeviceName : unitName
    }

    var isBaseStation: Bool {
        deviceType == .baseStation
    }

    /// Rockets seen via this device's LoRa relay (base station only)
    @Published var remoteRockets: [RemoteRocket] = []

    /// Latched copy of the most recent usable rocket GPS fix (#140).
    /// Stays populated when a telemetry packet arrives without fresh
    /// GPS so the map marker doesn't blank — only a newer valid fix
    /// replaces it.  Hydrated from the fleet's cache on the first
    /// telemetry packet after a BLE reconnect so a brief disconnect
    /// doesn't lose the last known position.
    @Published var lastValidRocketFix: LastValidRocketFix?

    /// Back-reference to the fleet so this device can read/write the
    /// cross-session last-valid-fix cache.  Weak: the fleet owns this
    /// device's lifetime via its `devices` array (didDisconnectPeripheral
    /// removes us), and a strong link would form a retain cycle.
    weak var fleet: BLEFleet?

    // Flight voice announcer (set by DashboardView).  Typed as a protocol so
    // dispatch tests can inject a spy without touching AVAudioSession.
    var flightAnnouncer: (any TelemetryAnnouncer)?

    // MARK: - Internal state

    private let jsonDecoder = JSONDecoder()
    private var simSawNonReady = false

    // Download state
    private var downloadExpectedSize: Int = 0
    private var downloadedData = Data()
    private var downloadCompletionHandler: ((URL?) -> Void)?
    private var downloadStallTimer: Timer?
    private var rssiTimer: Timer?
    private var poweringOnTimer: Timer?

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

    init(peripheral: CBPeripheral?, name: String) {
        self.connectedDeviceName = name
        self.deviceType = BLEDeviceType.from(name: name)
        super.init()
        self.peripheral = peripheral
        peripheral?.delegate = self
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
        drainOtaReadyContinuation()
        telemetryCharacteristic = nil
        commandCharacteristic = nil
        fileOpsCharacteristic = nil
        fileTransferCharacteristic = nil
        files = []
        currentPage = 0
        hasMoreFiles = false
        clearSimBanner()
        clearPoweringOn()
        rocketConfig = nil
        // Reset so the next reconnect (including after a BS reboot) triggers
        // another auto-pick.  Any pending scan-and-apply is also dropped —
        // a scan kicked off just before disconnect would not be able to
        // finish anyway.
        hasAutoSelectedChannel = false
        pendingAutoApply = false
        // Drop any cached mag-cal status so a stale REVIEW from a previous
        // session doesn't show up on reconnect (issue #96).  The FC
        // republishes IDLE on reconnect anyway.
        magCalStatus = nil
        sensorCalStatus = nil
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

    /// Power-on press handler (#159).  Lights the button's busy state and
    /// arms a watchdog before sending the toggle.  `poweringOn` clears on the
    /// first `pwr_pin_on` telemetry frame (see telemetry decode) or on
    /// disconnect; the watchdog is a backstop so a silently-dropped command
    /// can't leave the button spinning forever.  Cmd 8 is a toggle, so the
    /// button stays disabled while busy to prevent a double-press from
    /// powering the rocket back off.
    func beginPowerOn() {
        poweringOn = true
        poweringOnTimer?.invalidate()
        // 3 min: after cmd 8 the OC blocks its main loop (no telemetry sent)
        // while it flushes and recovers the on-NAND flight log, which can run
        // up to ~90s on a large/unclean log.  The watchdog must outlast that
        // worst case so it only fires on a genuinely-dropped command, not a
        // slow-but-normal recovery.
        poweringOnTimer = Timer.scheduledTimer(withTimeInterval: 180.0, repeats: false) { [weak self] _ in
            DispatchQueue.main.async { self?.clearPoweringOn() }
        }
        sendPowerToggle()
    }

    func clearPoweringOn() {
        poweringOnTimer?.invalidate()
        poweringOnTimer = nil
        poweringOn = false
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

    // MARK: - OTA helpers (#8 phase 2)

    enum OTAError: Error, LocalizedError {
        case notConnected
        case writeInFlight
        case writeFailed(String)

        var errorDescription: String? {
            switch self {
            case .notConnected:        return "Device is not connected"
            case .writeInFlight:       return "A chunk write is already in flight"
            case .writeFailed(let m):  return "Chunk write failed: \(m)"
            }
        }
    }

    /// Send OTA_BEGIN (cmd 70). Payload: [target:1][size:4 LE][sha256:32].
    /// targetIsFC=false → flashes this BS/OC device's own app. targetIsFC=true
    /// is reserved for phase 4 (OC-relayed Flight Computer OTA).
    func sendOtaBegin(targetIsFC: Bool, totalSize: UInt32, sha256: Data) {
        precondition(sha256.count == 32, "SHA-256 must be exactly 32 bytes")
        var payload = Data(capacity: 37)
        payload.append(targetIsFC ? 0x01 : 0x00)
        var sizeLE = totalSize.littleEndian
        withUnsafeBytes(of: &sizeLE) { payload.append(contentsOf: $0) }
        payload.append(sha256)
        sendRawCommand(70, payload: payload)
    }

    /// Send OTA_FINISH (cmd 71). Device verifies SHA, sets boot partition,
    /// then reboots ~500 ms later.
    func sendOtaFinish() { sendRawCommand(71) }

    /// Send OTA_ABORT (cmd 72). Always safe; clears any in-flight session.
    func sendOtaAbort() { sendRawCommand(72) }

    /// Maximum image-payload bytes per file-transfer write. Negotiated MTU
    /// minus 7-byte chunk header. Falls back to 170 (iOS default 185 MTU)
    /// before MTU negotiation completes.
    ///
    /// Uses .withoutResponse limit (which CoreBluetooth advertises separately)
    /// — typically equal to the with-response limit for the same MTU.
    var otaMaxChunkSize: Int {
        guard let peripheral = peripheral else { return 170 }
        let maxWrite = peripheral.maximumWriteValueLength(for: .withoutResponse)
        return max(20, maxWrite - 7)
    }

    /// Resumed by peripheralIsReady(toSendWriteWithoutResponse:) when the BLE
    /// stack's outgoing write buffer drains and we can queue more chunks.
    /// At most one waiter at a time — the OTASession chunk pump is serial.
    private var otaReadyContinuation: CheckedContinuation<Void, Never>?

    /// Send a single OTA image chunk over the file-transfer characteristic.
    /// Frame: [offset:4 LE][length:2 LE][flags:1][data:N].
    ///
    /// Uses writeWithoutResponse for throughput — write-with-response forced
    /// the BLE link into a serial req/resp cycle per chunk, giving ~2.5 KB/s.
    /// writeWithoutResponse + canSendWriteWithoutResponse backpressure
    /// lets iOS batch multiple writes per connection event (~10-20× faster).
    /// No per-chunk ATT ack — relies on link-layer CRC + retransmit for
    /// reliability and on OTA_FINISH's SHA-256 check to catch any drops.
    ///
    /// Throws .writeFailed if canSendWriteWithoutResponse stays false for
    /// 5 s straight — that means the characteristic isn't advertising the
    /// WRITE_NO_RSP property (firmware-side bug) and we'd otherwise hang.
    func sendOtaChunk(offset: UInt32, data: Data, isLast: Bool) async throws {
        guard let characteristic = fileTransferCharacteristic,
              let peripheral = peripheral else {
            throw OTAError.notConnected
        }

        // Backpressure: if iOS's outgoing buffer is full, wait for it to drain
        // before queuing another write. Bound the wait so a missing
        // WRITE_NO_RSP property on the firmware side doesn't hang the pump
        // forever (we hit exactly that on bench 2026-05-28).
        if !peripheral.canSendWriteWithoutResponse {
            let ready = await withTimeout(seconds: 5.0) {
                await withCheckedContinuation { (cont: CheckedContinuation<Void, Never>) in
                    if peripheral.canSendWriteWithoutResponse {
                        cont.resume()
                        return
                    }
                    self.otaReadyContinuation = cont
                }
            }
            if !ready {
                // Drain the pending continuation if it was set so the
                // delegate doesn't try to resume a dead waiter later.
                otaReadyContinuation = nil
                throw OTAError.writeFailed("Timed out waiting for canSendWriteWithoutResponse — firmware may not advertise WRITE_NO_RSP on file-transfer characteristic")
            }
        }

        var frame = Data(capacity: 7 + data.count)
        var offLE = offset.littleEndian
        withUnsafeBytes(of: &offLE) { frame.append(contentsOf: $0) }
        var lenLE = UInt16(data.count).littleEndian
        withUnsafeBytes(of: &lenLE) { frame.append(contentsOf: $0) }
        frame.append(isLast ? 0x01 : 0x00)
        frame.append(data)

        peripheral.writeValue(frame, for: characteristic, type: .withoutResponse)
    }

    /// Run `body`; return true if it completed within `seconds`, false on
    /// timeout. Used to bound waits on CoreBluetooth state changes.
    private func withTimeout(seconds: TimeInterval,
                             body: @escaping @Sendable () async -> Void) async -> Bool {
        await withTaskGroup(of: Bool.self) { group in
            group.addTask {
                await body()
                return true
            }
            group.addTask {
                try? await Task.sleep(nanoseconds: UInt64(seconds * 1_000_000_000))
                return false
            }
            let first = await group.next() ?? false
            group.cancelAll()
            return first
        }
    }

    /// Wake any pending sendOtaChunk awaiter with an error when the device
    /// drops mid-transfer (called from onDisconnect).
    private func drainOtaReadyContinuation() {
        if let cont = otaReadyContinuation {
            otaReadyContinuation = nil
            cont.resume()   // Non-throwing — sendOtaChunk will fail on the
                            // next peripheral guard since peripheral is nil.
        }
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

    /// Auto-channel-select was disabled in #136 — the BS now stays on the
    /// hardcoded rendezvous (915 MHz SF8 BW250) for the duration of a test
    /// instead of scanning and trying to move both ends.  See #150 for the
    /// follow-up that reintroduces scan-and-move alongside FHSS hopping.
    ///
    /// Function kept (rather than ripped out) so the call sites that ping
    /// it on config-readback and on new-rocket-seen events stay valid; it
    /// just no-ops.
    func triggerAutoChannelSelectIfNeeded() {
        // Intentionally empty — see comment above.
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

    /// Relay a new LoRa TX-power value to every tracked rocket and apply
    /// the same power to this base station.  Keeps freq/SF/BW/CR from the
    /// current base-station config — only TX power changes.  Reuses the
    /// transactional Cmd 10 path: the BS relays on OLD, switches to NEW
    /// power, and rolls back if no rocket beacon is heard within the verify
    /// window.  Refuses unless `autoApplyRefusalReason()` returns nil.
    @discardableResult
    func autoApplyTxPower(_ txPower: Int8) -> Bool {
        if let refusal = autoApplyRefusalReason() {
            print("[TXPWR] Auto-apply refused: \(refusal.rawValue)")
            return false
        }
        guard let cfg = rocketConfig,
              let freq = cfg.loraFreqMHz,
              let bw = cfg.loraBwKHz,
              let sf = cfg.loraSF,
              let cr = cfg.loraCR else { return false }
        sendLoRaConfig(freqMHz: freq, bwKHz: bw, sf: sf, cr: cr, txPower: txPower)
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

    /// Disable / re-enable LoRa frequency hopping (#106).  Only meaningful
    /// when sent to the base station — the BS persists the new value, hands
    /// off to the rocket via the corresponding uplink cmd, and restores
    /// fixed-frequency operation on lora_freq_mhz.  Sending this directly
    /// to a rocket is rejected by the rocket-side firmware.
    func sendLoRaHopDisabled(_ disabled: Bool) {
        sendRawCommand(17, payload: Data([disabled ? 0x01 : 0x00]))
        if var cfg = rocketConfig {
            cfg.loraHopDisabled = disabled
            rocketConfig = cfg
        }
    }

    func sendServoConfig(biases: [Int16], hz: Int16, minUs: Int16, maxUs: Int16,
                         finMinDeg: Float, finMaxDeg: Float) {
        var payload = Data()
        for i in 0..<4 {
            var b: Int16 = i < biases.count ? biases[i] : 0
            payload.append(Data(bytes: &b, count: 2))
        }
        var h = hz;  payload.append(Data(bytes: &h, count: 2))
        var mn = minUs; payload.append(Data(bytes: &mn, count: 2))
        var mx = maxUs; payload.append(Data(bytes: &mx, count: 2))
        // #267: fin-angle calibration (physical deg at min/max pulse) — 14 -> 22 bytes
        var fmn = finMinDeg; payload.append(Data(bytes: &fmn, count: 4))
        var fmx = finMaxDeg; payload.append(Data(bytes: &fmx, count: 4))
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

    func sendRollControlConfig(useAngleControl: Bool, rollDelayMs: UInt16, rateCapDps: Float,
                               kpAngle: Float, integralSepThreshold: Float) {
        var payload = Data()
        payload.append(useAngleControl ? 0x01 : 0x00)
        payload.append(0x00)
        var delay = rollDelayMs
        payload.append(Data(bytes: &delay, count: 2))
        var cap = rateCapDps                      // outer-loop angle→rate cap (deg/s), LE float
        payload.append(Data(bytes: &cap, count: 4))
        var kpa = kpAngle                         // outer angle-loop P-gain, LE float
        payload.append(Data(bytes: &kpa, count: 4))
        var iwind = integralSepThreshold          // PID integral-separation anti-windup threshold (deg/s), LE float
        payload.append(Data(bytes: &iwind, count: 4))
        sendRawCommand(31, payload: payload)      // RollControlConfigData = 16 bytes
        if var cfg = rocketConfig {
            cfg.useAngleControl = useAngleControl
            cfg.rollDelayMs = rollDelayMs
            cfg.rateCapDps = rateCapDps
            cfg.kpAngle = kpAngle
            cfg.integralSepThreshold = integralSepThreshold
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

    // Wire format matches firmware RollProfileData (packed): 1-byte
    // num_waypoints + 3 pad bytes, then 8 entries of RollWaypoint where
    // each entry is float time_s (4) + float angle_deg (4) + uint8_t mode (1)
    // = 9 bytes per waypoint. Total payload = 1 + 3 + 8*9 = 76 bytes.
    // mode: 0 = ROLL_SEG_ANGLE, 1 = ROLL_SEG_NULL_RATE.
    func sendRollProfile(waypoints: [(time: Float, angle: Float, mode: UInt8)]) {
        var payload = Data()
        let n = UInt8(min(waypoints.count, 8))
        payload.append(n)
        payload.append(contentsOf: [0, 0, 0])
        for i in 0..<8 {
            var t: Float = i < waypoints.count ? waypoints[i].time  : 0.0
            var a: Float = i < waypoints.count ? waypoints[i].angle : 0.0
            let  m: UInt8 = i < waypoints.count ? waypoints[i].mode : 0
            payload.append(Data(bytes: &t, count: 4))
            payload.append(Data(bytes: &a, count: 4))
            payload.append(m)
        }
        sendRawCommand(26, payload: payload)
    }

    func sendRollProfileClear() {
        sendCommand(27)
    }

    func sendGuidanceConfig(enabled: Bool) {
        sendRawCommand(32, payload: Data([enabled ? 0x01 : 0x00]))
    }

    /// Full PN guidance config (GuidanceConfigData = 36 bytes, cmd 65). Floats
    /// LE in struct order, then coast_delay(u16), enable(u8), target_mode(u8).
    func sendGuidanceConfig(enabled: Bool, navGain: Float, maxAccel: Float, accelToFin: Float,
                            maxFinDeg: Float, minSpeed: Float, coastDelayMs: UInt16,
                            targetMode: UInt8, targetE: Float, targetN: Float, targetAlt: Float) {
        var payload = Data()
        var ng = navGain, ma = maxAccel, a2f = accelToFin, mf = maxFinDeg, ms = minSpeed
        var te = targetE, tn = targetN, ta = targetAlt
        payload.append(Data(bytes: &ng,  count: 4))
        payload.append(Data(bytes: &ma,  count: 4))
        payload.append(Data(bytes: &a2f, count: 4))
        payload.append(Data(bytes: &mf,  count: 4))
        payload.append(Data(bytes: &ms,  count: 4))
        payload.append(Data(bytes: &te,  count: 4))
        payload.append(Data(bytes: &tn,  count: 4))
        payload.append(Data(bytes: &ta,  count: 4))
        var cd = coastDelayMs; payload.append(Data(bytes: &cd, count: 2))
        payload.append(enabled ? 0x01 : 0x00)
        payload.append(targetMode)
        sendRawCommand(65, payload: payload)   // GuidanceConfigData = 36 bytes
        if var cfg = rocketConfig { cfg.guidanceEnabled = enabled; rocketConfig = cfg }
    }

    func sendCameraConfig(cameraType: UInt8) {
        sendRawCommand(33, payload: Data([cameraType]))
    }

    /// IMU mounting orientation: 0xFF = auto (pad-gravity detect), 0..23 =
    /// manual board→rocket code (authoritative incl. roll clocking).
    func sendImuOrientationConfig(_ setting: UInt8) {
        sendRawCommand(64, payload: Data([setting]))
        if var cfg = rocketConfig {
            cfg.imuOrientSetting = setting
            rocketConfig = cfg
        }
    }

    /// 4-channel pyro config. Each tuple is (enabled, trigger_mode, trigger_value).
    /// Wire layout (24 bytes): 4 × {u8 enabled, u8 mode, f32 value}.
    func sendPyroConfig(channels: [(enabled: Bool, mode: UInt8, value: Float)]) {
        precondition(channels.count == 4, "pyro config requires exactly 4 channels")
        var payload = Data()
        for ch in channels {
            payload.append(ch.enabled ? 0x01 : 0x00)
            payload.append(ch.mode)
            var v = ch.value
            payload.append(Data(bytes: &v, count: 4))
        }
        sendRawCommand(34, payload: payload)
    }

    func sendPyroContTest(channel: UInt8) {
        sendRawCommand(35, payload: Data([channel]))
    }

    func sendPyroFire(channel: UInt8) {
        sendRawCommand(36, payload: Data([channel]))
    }

    // MARK: - Magnetometer hard-iron calibration (issue #96)
    //
    // Five single-byte commands.  The OC routes each to the FC over I2C,
    // and the FC publishes back a binary status frame on the file_ops
    // characteristic (parsed into magCalStatus).  Entry guard lives on
    // the FC: START is only honoured when rocket_state == READY.
    func sendMagCalStart()      { sendCommand(50) }
    func sendMagCalAbort()      { sendCommand(51) }
    func sendMagCalAccept()     { sendCommand(52) }
    func sendMagCalRetry()      { sendCommand(53) }
    /// Tell the FC: stop accumulating, run the sphere fit on what we
    /// have, transition to REVIEW.  Replaces the old
    /// auto-completion-at-buffer-fill so the user owns when the cal
    /// "finishes."
    func sendMagCalComputeFit() { sendCommand(54) }
    /// #148 — user-driven verify completion.  Tells the FC to evaluate
    /// the verify min/max/coverage immediately (instead of waiting for
    /// the 60 s safety timeout).  Pass → APPLIED + NVS write; fail →
    /// REVIEW with one of the verify reject sub-codes set.
    func sendMagCalVerifyDone()  { sendCommand(56) }
    /// #148 — user-driven verify retry.  Clears the verify accumulators
    /// without leaving VERIFYING; the proposed-new cal stays programmed
    /// on the chip so the next rotation pass measures the same stream.
    func sendMagCalVerifyReset() { sendCommand(57) }
    /// #148 — user-override save.  Writes NVS regardless of verify-gate
    /// status; used when the user taps "Save anyway" on the Verifying
    /// screen with some gates still red.
    func sendMagCalForceApply()  { sendCommand(58) }

    // Issue #132 — rocket-profile auto-sync.  The app holds source-of-truth
    // for mag cal as part of the active rocket profile.  APPLY pushes the
    // saved cal back into FC NVS on connect; READ queries what's currently
    // in NVS so the syncer can diff against the profile.  Both bypass the
    // sampling / sphere-fit flow — APPLY is gated FC-side to READY.

    /// Push a saved cal (hard-iron offsets + R/residual diagnostics) into
    /// the rocket's NVS.  Wire format mirrors `MagCalApplyData` in
    /// RocketComputerTypes.h: int16 cx, cy, cz, float R_uT, float res_uT
    /// (little-endian, packed, 14 bytes total).
    func sendMagCalApply(cx: Int16, cy: Int16, cz: Int16,
                         fieldR_uT: Float, residualUT: Float) {
        var payload = Data()
        var cxv = cx;       payload.append(Data(bytes: &cxv, count: 2))
        var cyv = cy;       payload.append(Data(bytes: &cyv, count: 2))
        var czv = cz;       payload.append(Data(bytes: &czv, count: 2))
        var rv  = fieldR_uT; payload.append(Data(bytes: &rv,  count: 4))
        var sv  = residualUT; payload.append(Data(bytes: &sv,  count: 4))
        sendRawCommand(55, payload: payload)
    }

    /// Ask the FC to publish a status frame built from current NVS values.
    /// The reply lands on `magCalStatus` like any other cal status frame
    /// (subType=APPLIED if cal is present, IDLE otherwise).
    // NOTE: the #132 profile-cal commands use 61/62/63, NOT the 56/57/58 that
    // would naturally follow the cal block — 56/57/58 are the #148 mag-cal
    // verify commands above (sendMagCalVerifyDone/Reset/ForceApply).  The OC
    // dispatch matches those first, so re-using them here made these reads
    // mis-fire as mag-verify commands (e.g. a connect-time sensor-cal read was
    // refused as "force_apply").  Keep in sync with the OC firmware
    // (out_computer/main/main.cpp ble_cmd dispatch).
    func sendMagCalRead() { sendCommand(61) }

    /// Push a saved sensor cal (gyro zero-rate bias + high-g accel bias) into
    /// the rocket's NVS (#132).  Wire format mirrors `SensorCalApplyData`:
    /// int16 gyro x/y/z, float hg x/y/z (little-endian, packed, 18 bytes).
    func sendSensorCalApply(gyroX: Int16, gyroY: Int16, gyroZ: Int16,
                            hgX: Float, hgY: Float, hgZ: Float) {
        var payload = Data()
        var gx = gyroX; payload.append(Data(bytes: &gx, count: 2))
        var gy = gyroY; payload.append(Data(bytes: &gy, count: 2))
        var gz = gyroZ; payload.append(Data(bytes: &gz, count: 2))
        var hx = hgX;   payload.append(Data(bytes: &hx, count: 4))
        var hy = hgY;   payload.append(Data(bytes: &hy, count: 4))
        var hz = hgZ;   payload.append(Data(bytes: &hz, count: 4))
        sendRawCommand(62, payload: payload)
    }

    /// Ask the FC to publish its stored sensor cal; the reply lands on
    /// `sensorCalStatus` (valid=false when the rocket has none).
    func sendSensorCalRead() { sendCommand(63) }

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

    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        // Wake the OTA chunk pump if it parked on a full outgoing buffer.
        // sendOtaChunk uses writeWithoutResponse for throughput; this
        // delegate is iOS's signal that the buffer has drained.
        if let cont = otaReadyContinuation {
            otaReadyContinuation = nil
            cont.resume()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                   didUpdateValueFor characteristic: CBCharacteristic,
                   error: Error?) {
        if characteristic.uuid == telemetryCharUUID {
            parseTelemetryData(characteristic.value)
        } else if characteristic.uuid == fileOpsCharUUID {
            if let data = characteristic.value {
                // Multiple binary frame kinds + JSON share this characteristic;
                // disambiguate by the first byte.  JSON always starts with '{'
                // or '[' so the binary discriminators (0xAA scan, 0xCA mag-cal)
                // can never collide with it.
                switch data.first {
                case 0xAA: parseScanResult(data)
                case 0xCA: parseMagCalStatus(data)     // issue #96
                case 0xCB: parseSensorCalStatus(data)  // issue #132
                case 0x7B:                              // '{' → JSON object
                    if let s = OTAStatusUpdate.parse(data) {
                        otaStatus = s
                    } else {
                        parseFileList(data)
                    }
                default:   parseFileList(data)         // '[' or anything else (file list arrays)
                }
            }
        } else if characteristic.uuid == fileTransferCharUUID {
            if let data = characteristic.value { handleFileChunk(data) }
        }
    }

    // MARK: - Telemetry parsing

    func parseTelemetryData(_ data: Data?) {
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
            cfg.rateCapDps = parseFloat(dict["rcap"]) ?? cfg.rateCapDps
            cfg.guidanceEnabled = dict["ge"] as? Bool ?? cfg.guidanceEnabled
            cfg.cameraType = UInt8(dict["camt"] as? Int ?? Int(cfg.cameraType))
            cfg.loraFreqMHz = parseFloat(dict["lf"])
            cfg.loraSF = (dict["lsf"] as? Int).map { UInt8($0) }
            cfg.loraBwKHz = parseFloat(dict["lbw"])
            cfg.loraCR = (dict["lcr"] as? Int).map { UInt8($0) }
            cfg.loraTxPower = (dict["lpw"] as? Int).map { Int8($0) }
            cfg.loraHopDisabled = dict["lhd"] as? Bool   // #106 (nil if device doesn't report it)
            if let existing = self.rocketConfig {
                cfg.pyro1Enabled = existing.pyro1Enabled
                cfg.pyro1TriggerMode = existing.pyro1TriggerMode
                cfg.pyro1TriggerValue = existing.pyro1TriggerValue
                cfg.pyro2Enabled = existing.pyro2Enabled
                cfg.pyro2TriggerMode = existing.pyro2TriggerMode
                cfg.pyro2TriggerValue = existing.pyro2TriggerValue
                cfg.pyro3Enabled = existing.pyro3Enabled
                cfg.pyro3TriggerMode = existing.pyro3TriggerMode
                cfg.pyro3TriggerValue = existing.pyro3TriggerValue
                cfg.pyro4Enabled = existing.pyro4Enabled
                cfg.pyro4TriggerMode = existing.pyro4TriggerMode
                cfg.pyro4TriggerValue = existing.pyro4TriggerValue
            }
            self.rocketConfig = cfg
            triggerAutoChannelSelectIfNeeded()
            return
        }

        // Pyro config readback: "type":"config_pyro" (4 channels)
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_pyro" {
            var cfg = self.rocketConfig ?? RocketConfig()
            cfg.pyro1Enabled = dict["p1e"] as? Bool ?? cfg.pyro1Enabled
            cfg.pyro1TriggerMode = UInt8(dict["p1m"] as? Int ?? Int(cfg.pyro1TriggerMode))
            cfg.pyro1TriggerValue = parseFloat(dict["p1v"]) ?? cfg.pyro1TriggerValue
            cfg.pyro2Enabled = dict["p2e"] as? Bool ?? cfg.pyro2Enabled
            cfg.pyro2TriggerMode = UInt8(dict["p2m"] as? Int ?? Int(cfg.pyro2TriggerMode))
            cfg.pyro2TriggerValue = parseFloat(dict["p2v"]) ?? cfg.pyro2TriggerValue
            cfg.pyro3Enabled = dict["p3e"] as? Bool ?? cfg.pyro3Enabled
            cfg.pyro3TriggerMode = UInt8(dict["p3m"] as? Int ?? Int(cfg.pyro3TriggerMode))
            cfg.pyro3TriggerValue = parseFloat(dict["p3v"]) ?? cfg.pyro3TriggerValue
            cfg.pyro4Enabled = dict["p4e"] as? Bool ?? cfg.pyro4Enabled
            cfg.pyro4TriggerMode = UInt8(dict["p4m"] as? Int ?? Int(cfg.pyro4TriggerMode))
            cfg.pyro4TriggerValue = parseFloat(dict["p4v"]) ?? cfg.pyro4TriggerValue
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
            if let fw = dict["fw"] as? String { firmwareVersion = fw }
            print("[CFG] Identity: uid=\(unitID) name=\(unitName) nid=\(networkID) rid=\(rocketID) type=\(deviceType.rawValue) fw=\(firmwareVersion)")
            return
        }

        // FC identity readback: "type":"fc_identity" — the Flight Computer's own
        // firmware version, relayed by the OUT computer over I2C (#8 Phase 4).
        // A separate small message so it stays well under the BLE notify MTU.
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "fc_identity" {
            if let fcFw = dict["fc_fw"] as? String { fcFirmwareVersion = fcFw }
            print("[CFG] FC identity: fc_fw=\(fcFirmwareVersion)")
            return
        }

        // Board→rocket mounting orientation: "type":"imu_orient" — which board
        // axis the FC has mapped to the rocket nose and how it decided
        // (default / manual / pad-gravity auto-detect). Re-sent whenever the
        // FC re-orients on the pad, so the display tracks the live mapping.
        // "set" is the user's setting (0xFF auto / manual code), cached into
        // rocketConfig like the other readback values.
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "imu_orient" {
            if let name = dict["name"] as? String { imuOrientationName = name }
            if let mode = dict["mode"] as? Int {
                imuOrientationMode = IMUOrientationMode(rawValue: mode) ?? .unknown
            }
            if let set = dict["set"] as? Int, var cfg = rocketConfig {
                cfg.imuOrientSetting = UInt8(clamping: set)
                rocketConfig = cfg
            }
            print("[CFG] IMU orientation: \(imuOrientationName) (\(imuOrientationMode.label))")
            return
        }

        // Regular telemetry
        do {
            let newTelemetry = try jsonDecoder.decode(TelemetryData.self, from: data)

            // If telemetry has a source_rocket_id, it's relayed via base station
            // → route to RemoteRocket instead of updating our own telemetry
            if let rid = newTelemetry.source_rocket_id, rid > 0, isBaseStation {
                let bsID = peripheral?.identifier ?? UUID()
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
                self.telemetry = newTelemetry
                // Mirror the latched fix (#140).  Only assign when the
                // cache returns a non-nil value so a GPS-less first
                // packet for a newly-relayed rocket can't blank a fix
                // this device just mirrored for a different rocketID.
                if let fix = fleet?.recordRocketFix(from: newTelemetry, rocketID: rocketID) {
                    self.lastValidRocketFix = fix
                }
                // The relayed JSON carries the full rocket state (st/aapo/lnch/
                // land/mspd/palt — see TR_BLE_To_APP.cpp).  Without this call
                // voice callouts only fire when paired directly to the rocket,
                // never during a real flight (phone↔BS↔LoRa↔rocket).  #138.
                self.flightAnnouncer?.processTelemetry(newTelemetry)
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
            // #159: the rocket has finished flushing and powered on — drop the
            // Power On button's busy state.  Guarded so we don't republish on
            // every frame while already powered on.
            if newTelemetry.pwr_pin_on && self.poweringOn {
                self.clearPoweringOn()
            }
            // Direct rocket connection — use this device's own rocketID
            // (relayed-telemetry path above uses source_rocket_id instead).
            // Skip when rocketID is unset so a BS-self packet (which falls
            // through to here without a source_rocket_id) doesn't nil out
            // a fix this device just mirrored from a relay packet.
            if self.rocketID > 0,
               let fix = fleet?.recordRocketFix(from: newTelemetry, rocketID: self.rocketID) {
                self.lastValidRocketFix = fix
            }
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
    /// Parse a magnetometer hard-iron cal status frame (issue #96).
    /// Format: [0xCA][22-byte MagCalStatusData LE].  The FC sends one
    /// frame on every state transition (SAMPLING entry, REVIEW entry,
    /// APPLIED, ABORTED) plus a 5 Hz heartbeat in SAMPLING — so the iOS
    /// UI can update progress smoothly without polling.
    private func parseMagCalStatus(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count >= 23, bytes[0] == 0xCA else {
            print("[MAGCAL] malformed status (\(bytes.count) bytes)")
            return
        }
        let payload = Array(bytes[1..<bytes.count])
        guard let status = MagCalStatus.decode(payload) else {
            print("[MAGCAL] decode failed (payload=\(payload.count) bytes)")
            return
        }
        magCalStatus = status
    }

    private func parseSensorCalStatus(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count >= 20, bytes[0] == 0xCB else {
            print("[SENSORCAL] malformed status (\(bytes.count) bytes)")
            return
        }
        let payload = Array(bytes[1..<bytes.count])
        guard let status = SensorCalStatus.decode(payload) else {
            print("[SENSORCAL] decode failed (payload=\(payload.count) bytes)")
            return
        }
        sensorCalStatus = status
    }

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
