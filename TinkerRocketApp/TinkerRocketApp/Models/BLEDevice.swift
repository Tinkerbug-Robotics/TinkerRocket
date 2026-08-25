//
//  BLEDevice.swift
//  TinkerRocketApp
//
//  Per-peripheral BLE device state — telemetry, config, files, commands.
//  Extracted from BLEManager.swift for multi-device support.
//

import Foundation
// CoreBluetooth predates Swift concurrency: CBPeripheral et al. aren't marked
// Sendable, so capturing them in @Sendable closures (e.g. the OTA backpressure
// continuation below) warns. @preconcurrency suppresses those module-level
// Sendable warnings — the CB objects are only ever touched on the main actor.
@preconcurrency import CoreBluetooth
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
    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim (swift_task_deinitOnExecutorMainActorBackDeploy), which
    // aborts with "pointer being freed was not allocated" when the object is
    // deallocated inside a synchronous XCTest. This class has no deinit-time
    // logic, so skipping the executor hop is free — and it un-crashes every
    // test that creates and tears down an instance.
    nonisolated deinit {}

    // MARK: - Published per-device state

    @Published var isConnected = false
    @Published var telemetry = TelemetryData()
    // #377: `telemetry` starts as TelemetryData() (all zeros), which reads as
    // pwr_pin_on == false — indistinguishable from a genuinely-off rocket. In
    // the (re)connect→first-frame window the power state is UNKNOWN, and UI
    // must not offer the blind cmd-8 power toggle (one tap would power OFF an
    // already-on rocket). False until the first decoded telemetry frame.
    @Published private(set) var hasReceivedTelemetry = false
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
    // #385: monotonically identifies the latest scan request so the wedge
    // timeout can't clear a newer scan's spinner.
    private var scanGeneration = 0

    // Magnetometer hard-iron calibration status (issue #96).  Latest frame
    // received from the FC over BLE on the file_ops characteristic with a
    // 0xCA discriminator.  nil until the first frame arrives; reset on
    // disconnect so a stale REVIEW state doesn't bleed across sessions.
    @Published var magCalStatus: MagCalStatus?

    /// Latest sensor (gyro + high-g) cal readback from the FC, received on
    /// the file_ops characteristic behind a 0xCB discriminator (#132).
    @Published var sensorCalStatus: SensorCalStatus?

    /// Latest flash-space stats: rocketStorage on a direct rocket link (0xCC),
    /// bsStorage on a base-station link (0xCD). Each is shown by its own bar.
    @Published var rocketStorage: RocketStorageStats?
    @Published var bsStorage: BaseStationStorageStats?

    /// Latest rail-off pyro-test refusal from the OC (0xCE on file_ops): the
    /// firmware refused to queue a cmd-35/36 pyro test because the FC power
    /// rail is off — a held pyro command would deliver its fire/ARM pulse at
    /// the next power-on. PyroTestView observes this to abort the fire flow;
    /// the cmd-35 TESTING window is closed at parse time. Reset on disconnect.
    @Published private(set) var pyroTestRefusal: PyroTestRefusal?

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

    /// Live guidance-target echo (#435): FC-authoritative state of the
    /// station-keep aim point ("guid_target" frames relayed by the OC).
    /// nil until the first frame — which also means "pre-#435 firmware,
    /// echo unsupported" (the OC only emits it for format_version >= 5
    /// FCs). The Drift-Cast send button gates success on this; nothing
    /// else should infer target state from a send having "gone through".
    @Published var guidanceTargetEcho: GuidanceTargetEcho?

    /// Display name: unitName if set, otherwise connectedDeviceName
    var displayName: String {
        unitName.isEmpty ? connectedDeviceName : unitName
    }

    var isBaseStation: Bool {
        deviceType == .baseStation
    }

    /// Rockets seen via this device's LoRa relay (base station only)
    @Published var remoteRockets: [RemoteRocket] = []

    /// #390: which relayed rocket this base-station link is pinned to.
    /// `telemetry`, the announcer, and the device-level fix latch follow
    /// ONLY this rocket — a second rocket in range no longer flip-flops
    /// every dashboard card at packet rate. Latches onto the FIRST rocket
    /// heard (sticky); the fleet re-seeds it across reconnects and the user
    /// switches it explicitly. nil = no rocket heard yet this session.
    @Published var focusRocketID: UInt8?

    /// Earliest moment the sticky focus pin may be re-evaluated (#390 follow-up).
    /// Set whenever the pin is applied — on first latch and on every fleet
    /// re-seed — so a rocket that simply has not transmitted yet since reconnect
    /// cannot lose focus to whichever rocket happens to speak first.
    var focusPinGraceUntil: Date?
    /// How long after the pin is applied before staleness is considered at all.
    static let focusPinGraceInterval: TimeInterval = 20
    /// A pinned rocket unheard for longer than this is treated as gone.
    /// Rockets relay at ~2 Hz, so this is ~30 missed frames.
    static let focusPinStaleAfter: TimeInterval = 15

    /// When the last telemetry frame (any kind) was decoded on this link.
    /// Roster freshness for direct rocket links reads this.
    private(set) var lastTelemetryAt: Date?

    /// #390: with the relay mirror pinned to the focused rocket, the BS's
    /// periodic stale re-push can describe a DIFFERENT rocket (old firmware
    /// re-pushes the last-heard one) and get dropped by the pin — freezing
    /// `telemetry.data_status` at .live while the focused rocket is silent.
    /// This is the app-computed age of the focused rocket's stream,
    /// refreshed on the 2 s RSSI tick; `effectiveDataStatus` overlays it.
    @Published private(set) var focusedRelayAgeMs: UInt32?

    /// Mirrors the BS firmware's BLE_TELEMETRY_STALE_MS.
    static let relayStaleThresholdMs: UInt32 = 3000

    /// #831: how long a connected-but-silent link keeps counting as live for
    /// continuity. Matches the relay threshold and the OC's own
    /// FC_FRAME_STALE_MS so all three age at the same rate.
    static let telemetryStaleThresholdMs: UInt32 = 3000

    /// Clock seam so the #831 staleness window is testable without waiting on
    /// the wall clock. Production never replaces it.
    var nowProvider: () -> Date = { Date() }

    /// Freshness the dashboard should trust for this link's rocket stream:
    /// the frame-carried status, worsened by the app-computed focused-rocket
    /// age when that is staler (never improved — a BS-reported STALE stays).
    var effectiveDataStatus: TelemetryData.DataStatus {
        guard isBaseStation, telemetry.data_status != .syncing,
              let age = focusedRelayAgeMs, age > Self.relayStaleThresholdMs
        else { return telemetry.data_status }
        return .stale
    }

    var effectiveDataAgeMs: UInt32 {
        guard isBaseStation, let age = focusedRelayAgeMs,
              age > telemetry.data_age_ms else { return telemetry.data_age_ms }
        return age
    }

    func refreshFocusedRelayFreshness(now: Date = Date()) {
        guard isBaseStation, let focus = focusRocketID,
              let remote = remoteRockets.first(where: { $0.rocketID == focus })
        else {
            if focusedRelayAgeMs != nil { focusedRelayAgeMs = nil }
            return
        }
        let age = UInt32(clamping: Int(now.timeIntervalSince(remote.lastSeen) * 1000))
        if focusedRelayAgeMs != age { focusedRelayAgeMs = age }
    }

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
        pyroTestRefusal = nil
        // #435: the guidance-target echo is per-connection live state; the
        // OC re-pushes it in the connect-time config readback, so a stale
        // copy must not bridge sessions (it could "confirm" a send made
        // over a new connection to a rebooted FC).
        guidanceTargetEcho = nil
        // Power state is unknown again until the next session's first frame
        // (#377). Reconnects build a new BLEDevice anyway; this covers the
        // state-restoration path that reuses one.
        hasReceivedTelemetry = false
        flightAnnouncer?.reset()
        // #385: only re-arm screen auto-lock when this was the last connected
        // device — with BS + rocket both up, either one dropping used to
        // re-enable sleep while telemetry still streamed on the other.
        // (BLEFleet calls onDisconnect() before removing us from devices,
        // so exclude self and check live connections.)
        let anotherStillConnected =
            fleet?.devices.contains { $0 !== self && $0.isConnected } ?? false
        if !anotherStillConnected {
            UIApplication.shared.isIdleTimerDisabled = false
        }
    }

    // MARK: - BLE RSSI Polling

    private func startRSSITimer() {
        rssiTimer?.invalidate()
        rssiTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
            self?.peripheral?.readRSSI()
            // #390: piggyback the focused-rocket staleness overlay on the
            // same tick so it advances even when no frames arrive.
            self?.refreshFocusedRelayFreshness()
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

    /// Drop the SIM MODE banner once a launched sim's flight completes: arm
    /// on the first non-READY state, clear on the return to READY.  Called
    /// for direct-rocket frames AND the focused rocket's relayed frames —
    /// on a base-station link the sim runs relay-wrapped (#390), and the
    /// latch used to only clear on the direct path, leaving the banner
    /// stuck after every relayed sim flight (found via the Virtual Rocket,
    /// which is a BS; Android's latch already watches the focused stream).
    private func updateSimBannerLatch(_ t: TelemetryData) {
        guard simLaunched else { return }
        if t.state != "READY" && t.state != "INITIALIZATION" {
            simSawNonReady = true
        }
        if simSawNonReady && t.state == "READY" {
            simLaunched = false
            simSawNonReady = false
        }
    }

    // MARK: - Commands

    /// Wire-tap for outgoing commands.  The Virtual Rocket driver is the
    /// consumer: its device has no peripheral, so the tap IS the link (the
    /// same seam FakeFirmware provides on Android).  With a real peripheral
    /// attached the tap observes without swallowing.
    var commandTap: ((UInt8, Data) -> Void)?

    func sendCommand(_ command: UInt8) {
        commandTap?(command, Data())
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            if commandTap == nil { print("Cannot send command: not connected") }
            return
        }
        let data = Data([command])
        peripheral.writeValue(data, for: characteristic, type: .withResponse)
        print("Sent command: \(command)")
    }

    /// cmd 8 with the desired rail state.  Explicit rather than a blind
    /// toggle: a desync would otherwise cut the FC's rail when the operator
    /// asked to power it up.  The UI gates the button on having received
    /// telemetry (#377), so the state passed here is one we have seen.
    /// Firmware treats a bare cmd 8 as a legacy toggle.
    func sendPowerState(railOn: Bool) {
        sendRawCommand(8, payload: Data([railOn ? 1 : 0]))
    }

    /// Power-on press handler (#159).  Lights the button's busy state and
    /// arms a watchdog before commanding the rail ON.  `poweringOn` clears on
    /// the first `pwr_pin_on` telemetry frame (see telemetry decode) or on
    /// disconnect; the watchdog is a backstop so a silently-dropped command
    /// can't leave the button spinning forever.  The command now carries the
    /// desired state, so a double-press is idempotent rather than powering
    /// the rocket back off; the button still disables while busy.
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
        sendPowerState(railOn: true)
    }

    func clearPoweringOn() {
        poweringOnTimer?.invalidate()
        poweringOnTimer = nil
        poweringOn = false
    }

    func sendRawCommand(_ command: UInt8, payload: Data = Data()) {
        commandTap?(command, payload)
        guard let characteristic = commandCharacteristic,
              let peripheral = peripheral else {
            if commandTap == nil { print("Cannot send command: not connected") }
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
        // #385: only a parsed 0xAA result cleared isScanning, so a lost result
        // notification wedged the scan UI until reconnect. The BS runs 5
        // passes (~10-45 s); if nothing arrived well past that, declare the
        // result lost. The generation guard keeps a stale timeout from
        // clearing a newer scan.
        scanGeneration += 1
        let gen = scanGeneration
        DispatchQueue.main.asyncAfter(deadline: .now() + 75) { [weak self] in
            guard let self, self.isScanning, self.scanGeneration == gen else { return }
            print("[SCAN] result timeout — clearing stuck scan state")
            self.isScanning = false
        }
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

    /// Full guidance config (GuidanceConfigData = 45 bytes, cmd 65).  v1 floats LE
    /// in struct order, then coast_delay(u16), enable(u8), target_mode(u8); #534
    /// appended kp_pos(f32), kd_vel(f32), guidance_law(u8).  THE FIRST 36 BYTES
    /// ARE FROZEN — the FC parses by offset, so append only, never insert.
    ///
    /// Both laws' parameters are sent unconditionally, every time, regardless of
    /// which law is selected or which fields the UI is showing: the FC keeps the
    /// unused law fully configured so switching laws never flies an untuned one.
    ///
    /// A 45-byte-era FC REJECTS a 36-byte frame outright (length check) and logs
    /// it, so app and firmware must ship in lockstep.
    func sendGuidanceConfig(enabled: Bool, navGain: Float, maxAccel: Float, accelToFin: Float,
                            maxFinDeg: Float, minSpeed: Float, coastDelayMs: UInt16,
                            targetMode: UInt8, targetE: Float, targetN: Float, targetAlt: Float,
                            kpPos: Float, kdVel: Float, guidanceLaw: UInt8) {
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
        // --- appended #534 (offsets 36/40/44) ---
        var kp = kpPos, kd = kdVel
        payload.append(Data(bytes: &kp, count: 4))
        payload.append(Data(bytes: &kd, count: 4))
        payload.append(guidanceLaw)
        #if DEBUG
        assert(payload.count == 45, "GuidanceConfigData must be 45 bytes, built \(payload.count)")
        #endif
        sendRawCommand(65, payload: payload)   // GuidanceConfigData = 45 bytes
        if var cfg = rocketConfig { cfg.guidanceEnabled = enabled; rocketConfig = cfg }
    }

    /// Fin layout (FinConfigData = 18 bytes, cmd 66). Derives each servo's fin
    /// RING-POSITION azimuth from the ring slot it occupies (slot azimuths
    /// {0,90,180,270} for "+" or {45,135,225,315} for "×" — positions, not force
    /// directions; the FC maps position→tangential force in
    /// TR_ControlMixer::setFinLayout), then sends 4 LE floats + a per-servo
    /// tilt-reverse bitmask + an independent per-servo roll-reverse bitmask. The ring
    /// GUI's nose-down view is a rendering choice and does not change these azimuths.
    func sendFinConfig(ringMode: UInt8, servoAtSlot: [Int], reverse: [Bool], rollReverse: [Bool]) {
        guard servoAtSlot.count == 4, reverse.count == 4, rollReverse.count == 4 else { return }
        let slotAz: [Float] = ringMode == 1 ? [45, 135, 225, 315] : [0, 90, 180, 270]
        var az: [Float] = [0, 90, 180, 270]            // per servo index (0=servo 1)
        for slot in 0..<4 {
            let servo = servoAtSlot[slot]              // 1-4
            if servo >= 1 && servo <= 4 { az[servo - 1] = slotAz[slot] }
        }
        var payload = Data()
        for i in 0..<4 { var a = az[i]; payload.append(Data(bytes: &a, count: 4)) }
        var mask: UInt8 = 0
        for i in 0..<4 where reverse[i] { mask |= (UInt8(1) << UInt8(i)) }
        payload.append(mask)
        var rollMask: UInt8 = 0
        for i in 0..<4 where rollReverse[i] { rollMask |= (UInt8(1) << UInt8(i)) }
        payload.append(rollMask)
        sendRawCommand(66, payload: payload)           // FinConfigData = 18 bytes
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

    /// IMU logging rate — `RocketProfile.imuRateDynamic` (0) or a whitelisted
    /// ISM6HG256 ODR (960/1920/3840). The FC applies it live on the pad and
    /// persists it in FC NVS. In dynamic mode the FC owns the in-flight
    /// step-down; the app sends the mode once and never revisits it.
    func sendImuRateConfig(_ rateHz: UInt16) {
        var payload = Data()
        payload.append(UInt8(rateHz & 0xFF))
        payload.append(UInt8(rateHz >> 8))
        sendRawCommand(67, payload: payload)
        if var cfg = rocketConfig {
            cfg.imuRateHz = rateHz
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

    // Continuity-test pending window (#411 UX). A manual PYRO_CONT_TEST
    // round-trips BLE -> OC -> I2C -> FC and back via a status frame (a few
    // seconds). Until the fresh reading returns, the previously-cached
    // CONT/NO-CONT value would misleadingly display as if it were the
    // answer — the FC even re-reads the same pin, so an open channel can
    // briefly show a stale "CONT" and a good one a stale "NO CONT". Track a
    // per-channel deadline so the UI can show "TESTING" during the window
    // instead of a value it can't yet trust.
    @Published private(set) var contTestPendingUntil: [UInt8: Date] = [:]
    static let contTestPendingWindow: TimeInterval = 2.5
    /// BS relay round trip is longer: BLE → uplink retry train (≤ ~1.5 s of
    /// blind retries + TX-window defers) → FC test → 2 Hz LoRa downlink →
    /// BS → BLE push.
    static let contTestPendingWindowRelay: TimeInterval = 8.0

    private var contTestWindow: TimeInterval {
        isBaseStation ? Self.contTestPendingWindowRelay : Self.contTestPendingWindow
    }

    /// True while a manual continuity test for `channel` is still round-
    /// tripping — the UI shows "TESTING" rather than the (stale) reading.
    func contTestPending(channel: UInt8) -> Bool {
        guard let until = contTestPendingUntil[channel] else { return false }
        return Date() < until
    }

    /// True when a pyro test command sent right now will actually go out.
    /// Direct link: connected. BS link: connected AND a focused rocket whose
    /// relayed stream is fresh — a BS that hasn't heard the rocket recently
    /// would queue a blind uplink into silence (no ACK, #285) while the UI
    /// advances as if the command were delivered. The fire-instant guard in
    /// PyroTestView (#292) checks THIS, not just isConnected.
    ///
    /// Freshness is computed HERE from the focused rocket's lastSeen, not
    /// read from `focusedRelayAgeMs` — that property only refreshes on the
    /// 2 s RSSI tick, so at a fire instant it can be stale-true by up to a
    /// full tick. A T-0 check deserves the real clock.
    var pyroCommandPathReady: Bool {
        guard isConnected else { return false }
        guard isBaseStation else { return true }
        guard let focus = focusRocketID,
              let remote = remoteRockets.first(where: { $0.rocketID == focus })
        else { return false }
        let ageMs = UInt32(clamping: Int(Date().timeIntervalSince(remote.lastSeen) * 1000))
        return ageMs <= Self.relayStaleThresholdMs
    }

    /// What the link can actually say about a pyro channel's continuity.
    /// Distinguishing these four is the whole point: a red "NO CONT" must mean
    /// "we measured an open circuit", never "we have no measurement". Folding
    /// them into a Bool is what made a never-tested channel display a
    /// confident red NO CONT on the LoRa path (bench 2026-08-17).
    enum PyroContinuity: Equatable {
        case present      // measured: continuity OK
        case open         // measured: no continuity (fired, or nothing connected)
        case untested     // link is good, but no reading has been taken yet
        case noData       // stream stale / disconnected — nothing trustworthy
    }

    /// Path-aware continuity for the pyro UIs, #297 fail-safe included (a
    /// non-live stream is .noData, never a held-over green).
    ///
    /// Direct link: prefers the measured sensor-health bits, falling back to
    /// the "ps" cont bit for pre-#803 rockets. BS relay: the 65-byte LoRa
    /// downlink carries no pyro_status, so continuity rides the sensor-health
    /// scorecard — preferring the MEASURED bits (reported for every channel)
    /// and falling back to the config-gated ones for pre-#803 rockets.
    ///
    /// #831 — the fail-safe above used to be vacuous on a direct link, in two
    /// layers. The rocket only ever sends "ds" when it is NOT live and the
    /// decoder defaults a missing one to .live, so the guard reduced to
    /// `isConnected`; and the OC republishes every rocket-derived field from
    /// its last NonSensorData snapshot whether or not the FC is still alive,
    /// so a dead FC left a green CONT standing indefinitely. The OC now ages
    /// that snapshot and sends STALE/SYNCING accordingly, which makes the
    /// existing guard real. The `lastTelemetryAt` check below covers the
    /// remaining case the OC cannot report on: frames stopping altogether
    /// while the link still counts as connected.
    func pyroContinuity(channel: Int) -> PyroContinuity {
        guard isConnected, effectiveDataStatus == .live else { return .noData }
        // A connected link that has gone quiet is not a live reading, whatever
        // the last frame said.
        guard let seen = lastTelemetryAt,
              nowProvider().timeIntervalSince(seen) * 1000
                  <= Double(Self.telemetryStaleThresholdMs)
        else { return .noData }
        if isBaseStation {
            if let measured = telemetry.pyroMeasuredContinuity(channel: channel) {
                switch measured {
                case .ok:  return .present
                case .bad: return .open
                default:   return .untested      // .na here = not tested yet
                }
            }
            // Older rocket firmware: only the config-gated bits exist. An
            // unconfigured channel (.na) is unknowable on this path, and
            // .degraded means configured-but-untested — both are "untested",
            // NOT an open circuit.
            switch telemetry.pyroHealth(channel: channel) {
            case .ok:  return .present
            case .bad: return .open
            default:   return .untested
            }
        }
        // Direct link: prefer the measured scorecard bits when present, since
        // they distinguish untested from open; the raw cont bit cannot.
        if let measured = telemetry.pyroMeasuredContinuity(channel: channel) {
            switch measured {
            case .ok:  return .present
            case .bad: return .open
            default:   return .untested
            }
        }
        // The raw "ps" cont bit can only ever prove PRESENCE, never an open.
        // The FC sets it as `cont_known[i] && cont_state[i]`
        // (flight_computer/main.cpp), so a SET bit is unambiguous — measured,
        // and continuous — while a CLEAR bit conflates "never measured" with
        // "measured open". Red is reserved for a measurement, so a clear bit
        // is .untested and a measured open can only come from the scorecard's
        // SH_PYRO_MEAS = BAD above.
        //
        // Found on the bench 2026-08-22, and it is the state every session
        // STARTS in: before any continuity test, all four SH_PYRO_MEAS fields
        // read NA, so pyroMeasuredContinuity returns nil for every channel and
        // execution reaches here. The live board reported
        // sensor_health = 1092981 with a clear cont bit, and all four channels
        // rendered a confident red "NO CONT". #828 fixed that at the view
        // layer and #831 fixed the powered-off case, but both missed this one:
        // their tests always had at least one channel measured, which is what
        // makes pyroMeasuredContinuity return non-nil.
        return telemetry.pyroCont(channel: channel) ? .present : .untested
    }

    /// Fail-safe Bool for callers that must reduce to go/no-go: ONLY a
    /// measured .present counts. Never use this to render a verdict — it
    /// cannot distinguish "open" from "no reading".
    func pyroContinuityLive(channel: Int) -> Bool {
        pyroContinuity(channel: channel) == .present
    }

    func sendPyroContTest(channel: UInt8) {
        if isBaseStation {
            // Stand-back pyro test: relay over LoRa to the focused rocket.
            // Broadcast (0xFF) is deliberately never used here — a pyro
            // command must name its rocket.
            guard let rid = focusRocketID else { return }
            sendRelayCommand(targetRocketID: rid, innerCommand: 35,
                             innerPayload: Data([channel]))
        } else {
            sendRawCommand(35, payload: Data([channel]))
        }
        // Open the "TESTING" window; clear it (a published mutation, so the
        // UI re-renders the reveal) once the reading has had time to return.
        let window = contTestWindow
        contTestPendingUntil[channel] = Date().addingTimeInterval(window)
        DispatchQueue.main.asyncAfter(deadline: .now() + window) { [weak self] in
            guard let self = self else { return }
            // Only clear if a newer tap on this channel hasn't pushed the
            // deadline out past now.
            if let until = self.contTestPendingUntil[channel], until <= Date() {
                self.contTestPendingUntil.removeValue(forKey: channel)
            }
        }
    }

    func sendPyroFire(channel: UInt8) {
        if isBaseStation {
            // Stand-back pyro test: FIRE relayed over LoRa (BS cmd 50 wrap →
            // uplink cmd 36) so the operator keeps LoRa distance from a live
            // charge. Blind fire-and-retry (#285): the rocket dedups the
            // retry train; delivery is confirmed by the operator's own eyes
            // and the post-fire continuity re-test, not by any ACK.
            guard let rid = focusRocketID else { return }
            sendRelayCommand(targetRocketID: rid, innerCommand: 36,
                             innerPayload: Data([channel]))
        } else {
            sendRawCommand(36, payload: Data([channel]))
        }
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

    /// Rocket flash logging around a pyro test, path-aware. Direct link: the
    /// legacy cmd-23 toggle (PyroTestView only calls this when the current
    /// state differs from `on`, per the #385 guard). BS link: the BS's own
    /// BLE cmd 23 is the wrong tool — its toggle authority is the BS CSV
    /// logging state, so with BS logging on and rocket logging off it would
    /// STOP the BS log mid-test and never start the rocket's. Relay uplink
    /// cmd 23 with the desired-state payload instead: rocket-only, and
    /// idempotent across the blind retry train.
    func setRocketLoggingForTest(on: Bool) {
        if isBaseStation {
            guard let rid = focusRocketID else { return }
            sendRelayCommand(targetRocketID: rid, innerCommand: 23,
                             innerPayload: Data([on ? 1 : 0]))
        } else {
            sendCommand(23)
        }
    }

    /// #435: Drift-Cast guidance aim point, BLE cmd 28 — handled end-to-end
    /// since #435. The OC queues the 20-byte GuidancePointData to the FC,
    /// which converts geodetic → pad-relative ENU and (when the state / law /
    /// GNSS-ref / 100 m-radius gates all pass) retargets station-keep.
    /// Sending proves NOTHING by itself: success is gated on the
    /// "guid_target" readback echo (`guidanceTargetEcho`) — the FC bumps its
    /// seq on every processed upload, accept or reject. DriftCastSendButton
    /// owns the confirm/timeout state machine; keep any new caller behind
    /// the same echo gate.
    func sendGuidancePoint(lat: Double, lon: Double, altitudeM: Float) {
        sendRawCommand(28, payload: Self.guidancePointPayload(
            lat: lat, lon: lon, altitudeM: altitudeM))
    }

    /// Wire payload for cmd 28: {lat f64, lon f64, alt f32} little-endian,
    /// 20 bytes — byte-for-byte the firmware's GuidancePointData. Shared by
    /// the direct and BS-relay paths.
    static func guidancePointPayload(lat: Double, lon: Double, altitudeM: Float) -> Data {
        var payload = Data()
        var latVal = lat
        var lonVal = lon
        var altVal = altitudeM
        payload.append(Data(bytes: &latVal, count: 8))
        payload.append(Data(bytes: &lonVal, count: 8))
        payload.append(Data(bytes: &altVal, count: 4))
        return payload
    }

    /// #435 BS path: the same cmd-28 payload wrapped in relay cmd 50 for a
    /// base-station link. Fire-and-forget: BS uplink has no rocket ACK
    /// (#285) and the guid_target echo only rides the direct OC BLE link,
    /// so a relayed point can NEVER be confirmed — callers must show a
    /// permanent "sent, unconfirmed" state, never success.
    func sendGuidancePointViaRelay(rid: UInt8, lat: Double, lon: Double, altitudeM: Float) {
        sendRelayCommand(targetRocketID: rid, innerCommand: 28,
                         innerPayload: Self.guidancePointPayload(
                             lat: lat, lon: lon, altitudeM: altitudeM))
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
        // Byte clamp, not char clamp: the firmware rejects plen > 20 outright
        // (no write, no echo), and 20 chars of multibyte UTF-8 can exceed it.
        guard let data = name.utf8Clamped(maxBytes: 20).data(using: .utf8) else { return }
        sendRawCommand(40, payload: data)
    }

    func sendSetNetworkID(_ nid: UInt8) {
        sendRawCommand(41, payload: Data([nid]))
    }

    func sendSetRocketID(_ rid: UInt8) {
        sendRawCommand(42, payload: Data([rid]))
    }

    /// Relay a command to a specific rocket via this base station.
    /// Command 50: [target_rid:1][inner_cmd:1][inner_payload:0..33]
    /// The base station unpacks this and queues a LoRa uplink. The inner
    /// cap is bs_uplink_queue::kMaxPayload = 33 (#435 widened the stale 18
    /// so the 20-byte cmd-28 guidance point fits; the OC's uplink RX buffer
    /// further caps delivered payloads at 26 — keep inner payloads ≤26).
    func sendRelayCommand(targetRocketID: UInt8, innerCommand: UInt8, innerPayload: Data = Data()) {
        var payload = Data([targetRocketID, innerCommand])
        payload.append(innerPayload.prefix(33))
        sendRawCommand(50, payload: payload)
    }

    /// #390: pin this base station's radio focus (cmd 45, payload [rid],
    /// 0 = auto). RAM-only on the BS — re-sent on every connect. Firmware
    /// without the handler ignores the unknown command; app-side pinning
    /// still governs display and voice, so the pairing degrades gracefully.
    func sendSetFocusRocket(_ rocketID: UInt8) {
        sendRawCommand(45, payload: Data([rocketID]))
    }

    /// #390: BS-only CSV logging control (cmd 46, payload [on]) — unlike
    /// legacy cmd 23 it never uplinks a rocket-logging command.
    func sendSetBSLogging(_ on: Bool) {
        sendRawCommand(46, payload: Data([on ? 1 : 0]))
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

    /// Bulk delete for multi-select. Each name is an independent cmd-3 write;
    /// CoreBluetooth serializes the `.withResponse` writes on its own queue, so
    /// the firmware processes them one at a time. The caller should refresh the
    /// file list afterward (totals/pagination shift once the deletes land).
    func deleteFiles(_ filenames: [String]) {
        for name in filenames { deleteFile(name) }
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

#if DEBUG
    // Test seam (#832). Production always enters through downloadFile(), which
    // needs a live characteristic, so the chunk handler had no reachable entry
    // point from a test — which is why nothing covered it. DEBUG-only, so it
    // cannot be called from a shipped build.
    func beginDownloadForTesting(filename: String, completion: @escaping (URL?) -> Void) {
        downloadingFilename = filename
        downloadedData = Data()
        downloadCompletionHandler = completion
        isDownloading = true
        downloadProgress = 0.0
        downloadExpectedSize = Int(files.first(where: { $0.name == filename })?.size ?? 0)
    }

    func handleFileChunkForTesting(_ data: Data) { handleFileChunk(data) }
#endif

    // MARK: - File chunk handling (private)

    private func handleFileChunk(_ data: Data) {
        guard isDownloading else { return }
        guard data.count >= 7 else { return }
        let offset = UInt32(data[0]) | (UInt32(data[1]) << 8) |
                     (UInt32(data[2]) << 16) | (UInt32(data[3]) << 24)
        let length = UInt16(data[4]) | (UInt16(data[5]) << 8)
        let flags = data[6]
        let isEOF = (flags & 0x01) != 0
        let isAbort = (flags & 0x02) != 0   // #526

        // #526: the firmware could not finish this transfer (rocket INFLIGHT, a
        // flash read error, or a BLE send failure). The bytes we have are a
        // truncated fragment, NOT a complete file. Fail the download and write
        // nothing — previously an abort arrived as a bare EOF and the partial file
        // was saved and cached as if it were whole.
        if isEOF && isAbort {
            downloadStallTimer?.invalidate()
            downloadStallTimer = nil
            print("[DOWNLOAD] ABORTED by device after \(downloadedData.count) bytes")
            failDownload()
            return
        }

        // #832: the offset field was parsed and then used only in a debug
        // print, so chunks were appended blindly. A notification dropped after
        // the peripheral queued it — the exact residual case the firmware's
        // redundant EOF (#524) exists to compensate for — left a silent hole
        // with no gap detection, and the file was saved as complete.
        //
        // On a data chunk the offset IS the position this data belongs at, so
        // contiguity is free: anything other than "exactly where we are" means
        // a frame went missing.
        if length > 0 {
            guard data.count >= 7 + Int(length) else {
                // A frame shorter than its own length header used to be
                // dropped silently, leaving a hole indistinguishable from a
                // clean transfer.
                print("[DOWNLOAD] short frame: len=\(length) but \(data.count) bytes — failing")
                failDownload()
                return
            }
            guard Int(offset) == downloadedData.count else {
                print("[DOWNLOAD] GAP: chunk at offset \(offset), expected \(downloadedData.count) — failing")
                failDownload()
                return
            }
            downloadedData.append(data.subdata(in: 7..<(7 + Int(length))))
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
            // #832: the EOF frame carries the device's own bytes_sent in its
            // offset field (out_computer/main.cpp, base_station/main.cpp), so
            // the app is handed the authoritative total for free and used to
            // throw it away. A mismatch means we are missing bytes the device
            // believes it sent.
            // The EOF frame may carry DATA as well as the flag, in which case
            // its offset is that data's POSITION, not the total. Measured on
            // the bench 2026-08-22: the base station's last frame for an
            // 18322-byte file is `offset=18190 len=132 eof=true`. A
            // zero-length EOF has offset == total, so offset+length covers
            // both shapes.
            //
            // The first cut of this compared offset alone, which would have
            // failed every base-station download — the tests missed it because
            // every EOF frame they built had an empty payload.
            if Int(offset) + Int(length) != received {
                print("[DOWNLOAD] EOF says \(Int(offset) + Int(length)) bytes, have \(received) — failing")
                failDownload()
                return
            }
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

    // #526: end a download in failure — write nothing, cache nothing, hand the
    // caller nil. Used when the device signals an abort (EOF|ABORT). Mirrors the
    // cleanup in completeDownload's failure branches so state can't leak.
    private func failDownload() {
        downloadStallTimer?.invalidate()
        downloadStallTimer = nil
        let handler = downloadCompletionHandler
        downloadingFilename = nil
        downloadedData = Data()
        downloadCompletionHandler = nil
        DispatchQueue.main.async { [weak self] in self?.isDownloading = false }
        handler?(nil)
    }

    private func completeDownload(fromStallTimer: Bool = false) {
        downloadStallTimer?.invalidate()
        downloadStallTimer = nil
        guard let filename = downloadingFilename else {
            DispatchQueue.main.async { self.isDownloading = false }
            downloadCompletionHandler?(nil)
            return
        }
        // #832: this shortfall check used to be gated on `fromStallTimer`, so
        // the EOF path — the common one — wrote whatever it had and reported
        // success. The listing's size is just as authoritative on either path.
        if downloadExpectedSize > 0 && downloadedData.count < downloadExpectedSize {
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
        } else if let deviceFile = files.first(where: { $0.name == filename }) {
            // #832: this comparison was a fast path only — on a size MISMATCH
            // it fell through to downloadStates, which downloadAndCacheFlight
            // had already written .completed into. The one place that knows
            // the true size failed OPEN, showing a green check on a truncated
            // file and disabling the download button (downloadButtonDisabled
            // returns true for .completed), so the operator could not re-pull
            // the log in that session.
            if FileCache.shared.isFlightCached(filename, expectedSize: deviceFile.size) {
                return .completed
            }
            // Cached but the wrong size: treat as not downloaded so it can be
            // retried, rather than trusting a stale .completed.
            if FileCache.shared.isFlightCached(filename) {
                return .notDownloaded
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
            // #390: re-assert the radio-focus pin after every (re)connect —
            // the BS keeps it in RAM only, so a BS reboot (which bounces
            // BLE) forgets it. The fleet seeded focusRocketID before
            // characteristics were up; this is the earliest safe write.
            if let self, self.isBaseStation, let focus = self.focusRocketID {
                self.sendSetFocusRocket(focus)
            }
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
                case 0xCC: parseRocketStorageStats(data)
                case 0xCD: parseBaseStationStorageStats(data)
                case 0xCE: parsePyroTestRefusal(data)  // rail-off pyro refusal
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
            // #253: roll-control gain readback. The device reports sentinels
            // (rcap/kpang <= 0, iwind < 0) for "firmware default" — keep the
            // local value in that case rather than displaying the sentinel.
            if let rcap = parseFloat(dict["rcap"]), rcap > 0 {
                cfg.rateCapDps = rcap
                cfg.rollGainsReported = true
            }
            if let kpa = parseFloat(dict["kpang"]), kpa > 0 { cfg.kpAngle = kpa }
            if let iw = parseFloat(dict["iwind"]), iw >= 0 { cfg.integralSepThreshold = iw }
            cfg.guidanceEnabled = dict["ge"] as? Bool ?? cfg.guidanceEnabled
            cfg.cameraType = UInt8(dict["camt"] as? Int ?? Int(cfg.cameraType))
            if let irate = dict["irate"] as? Int, let hz = UInt16(exactly: irate) {
                cfg.imuRateHz = hz
            }
            cfg.loraFreqMHz = parseFloat(dict["lf"])
            cfg.loraSF = (dict["lsf"] as? Int).map { UInt8($0) }
            cfg.loraBwKHz = parseFloat(dict["lbw"])
            cfg.loraCR = (dict["lcr"] as? Int).map { UInt8($0) }
            cfg.loraTxPower = (dict["lpw"] as? Int).map { Int8($0) }
            cfg.loraHopDisabled = dict["lhd"] as? Bool   // #106 (nil if device doesn't report it)
            cfg.loraHopDwell = dict["lhdw"] as? Int      // #150 (0 = hopping unavailable)
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
                // #915: the config report rides its own frames, so a `config`
                // rebuild must carry it over — same reason as the pyro fields
                // above. Without this a re-sent readback would reset the app
                // to "this rocket reports nothing" and put every group back
                // on the can't-verify list.
                cfg.servoExtras = existing.servoExtras
                cfg.guidanceExtras = existing.guidanceExtras
                cfg.rollWaypoints = existing.rollWaypoints
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

        // #915 config report, relayed by the OC from the FC's own state as
        // three small frames.  Everything here was previously invisible to
        // the app, which is why the settings screen could show a value the
        // rocket had never agreed to.  Defensive per the MTU-budget rule
        // (#282): every key individually optional, and a frame that arrives
        // malformed leaves the group nil (= "not reported") rather than
        // half-filled.
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_servo" {
            var cfg = self.rocketConfig ?? RocketConfig()
            let az = (dict["faz"] as? [Any])?.compactMap { parseFloat($0) } ?? []
            if let b2 = dict["sb2"] as? Int, let b3 = dict["sb3"] as? Int,
               let b4 = dict["sb4"] as? Int,
               let fmn = parseFloat(dict["fmn"]), let fmx = parseFloat(dict["fmx"]),
               az.count == 4,
               let frv = dict["frv"] as? Int, let frrv = dict["frrv"] as? Int,
               let snd = dict["snd"] as? Bool {
                cfg.servoExtras = RocketServoExtras(
                    bias2: Int16(clamping: b2), bias3: Int16(clamping: b3),
                    bias4: Int16(clamping: b4),
                    finMinDeg: fmn, finMaxDeg: fmx,
                    finAzimuths: az,
                    finReverseMask: UInt8(clamping: frv),
                    finRollReverseMask: UInt8(clamping: frrv),
                    soundsEnabled: snd)
                self.rocketConfig = cfg
            }
            return
        }

        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_guid" {
            var cfg = self.rocketConfig ?? RocketConfig()
            if let ng = parseFloat(dict["gng"]), let ma = parseFloat(dict["gma"]),
               let af = parseFloat(dict["gaf"]), let mf = parseFloat(dict["gmf"]),
               let ms = parseFloat(dict["gms"]), let cd = dict["gcd"] as? Int,
               let tm = dict["gtm"] as? Int,
               let te = parseFloat(dict["gte"]), let tn = parseFloat(dict["gtn"]),
               let ta = parseFloat(dict["gta"]),
               let kp = parseFloat(dict["gkp"]), let kd = parseFloat(dict["gkd"]),
               let law = dict["glw"] as? Int {
                cfg.guidanceExtras = RocketGuidanceExtras(
                    navGain: ng, maxAccel: ma, accelToFin: af, maxFinDeg: mf,
                    minSpeed: ms, coastDelayMs: UInt16(clamping: cd),
                    targetMode: UInt8(clamping: tm),
                    targetE: te, targetN: tn, targetAltM: ta,
                    kpPos: kp, kdVel: kd, guidanceLaw: UInt8(clamping: law))
                self.rocketConfig = cfg
            }
            return
        }

        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           dict["type"] as? String == "config_roll" {
            var cfg = self.rocketConfig ?? RocketConfig()
            // "n": 0 with an empty list is a real answer — this rocket is
            // flying rate-only — so an empty array must still set the value
            // rather than leaving it nil ("we don't know").
            if let n = dict["n"] as? Int, let raw = dict["wp"] as? [Any] {
                var wps: [(time: Float, angle: Float)] = []
                for entry in raw.prefix(n) {
                    guard let pair = entry as? [Any], pair.count == 2,
                          let t = parseFloat(pair[0]), let a = parseFloat(pair[1])
                    else { continue }
                    wps.append((time: t, angle: a))
                }
                cfg.rollWaypoints = wps
                self.rocketConfig = cfg
            }
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
            // Fold the readback into the known-device registry and let it
            // push any edits queued while this device was offline.  The
            // firmware echoes a fresh config_identity after each identity-set
            // command, so pushed values confirm through this same path.
            fleet?.knownDevices.deviceDidReportIdentity(
                unitID: unitID, name: unitName, deviceType: deviceType,
                networkID: networkID, rocketID: rocketID, pusher: self)
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

        // Guidance-target echo: "type":"guid_target" (#435) — FC-authoritative
        // aim-point state (OutStatusQueryData v5 tail), relayed by the OC on
        // connect, on every processed cmd 28, and on any cmd-65 apply that
        // changes it. Live state, not part of rocketConfig: the Drift-Cast
        // send button gates its success on this echo, and the "Unit target"
        // row tracks it (e.g. a connect-time profile push visibly wiping a
        // previously sent Drift-Cast point back to "none / overhead").
        if let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
           let echo = GuidanceTargetEcho(json: dict) {
            guidanceTargetEcho = echo
            print("[CFG] Guidance target: seq=\(echo.seq) st=\(echo.status) rc=\(echo.lastRc) " +
                  String(format: "lat=%.6f lon=%.6f alt=%dm", echo.lat, echo.lon, echo.altM))
            return
        }

        // Regular telemetry
        do {
            let newTelemetry = try jsonDecoder.decode(TelemetryData.self, from: data)

            // #377: power state (and the rest of the flags) are now confirmed —
            // the UI may trust telemetry.pwr_pin_on from here on. Guarded so we
            // don't republish on every frame.
            if !self.hasReceivedTelemetry {
                self.hasReceivedTelemetry = true
            }
            self.lastTelemetryAt = Date()

            // If telemetry has a source_rocket_id, it's relayed via base station
            // → route to RemoteRocket; only the FOCUSED rocket mirrors into
            // this device's own telemetry/announcer/fix latch (#390 — a
            // second rocket in range used to flip-flop all of them).
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

                // #390: sticky first-heard focus. Never moves on its own after
                // this — the user switches it (or the fleet re-seeds it on
                // reconnect). This is what kills the last-heard recency race.
                if focusRocketID == nil {
                    focusRocketID = rocketID
                    focusPinGraceUntil = Date().addingTimeInterval(Self.focusPinGraceInterval)
                    fleet?.noteAutoFocus(baseStation: self, rocketID: rocketID)
                } else if let pinned = focusRocketID, pinned != rocketID,
                          Date() >= (focusPinGraceUntil ?? .distantPast),
                          !remoteRockets.contains(where: {
                              $0.rocketID == pinned &&
                              Date().timeIntervalSince($0.lastSeen) < Self.focusPinStaleAfter
                          }) {
                    // SELF-HEAL a dangling pin. "Sticky" must not mean
                    // "unfalsifiable": the pin is latched once above, re-seeded
                    // from the in-memory fleet cache on every reconnect
                    // (BLEFleet.adopt), and re-pushed to the BS as cmd 45 on
                    // every reconnect — and that push sets focus_rid_pinned,
                    // which DISABLES the BS's own 30 s auto-fallback. So a pin
                    // naming a rocket that is no longer on the air wedged the
                    // app onto the relay branch (no Signal card, no arrow, no
                    // bars) with nothing left to recover it. Seen on the bench
                    // 2026-08-03: strip showed "Rocket 1" while the section
                    // header resolved the rocket actually being heard.
                    //
                    // Both guards are load-bearing. The grace window stops a
                    // rocket that has merely not transmitted since reconnect
                    // from losing focus to whoever speaks first — without it
                    // this fires immediately after adopt(), when the roster is
                    // still empty, and steals focus every single reconnect. The
                    // lastSeen test (not mere roster membership) stops a stale
                    // roster entry from propping up a pin for a rocket that has
                    // been off the air for minutes.
                    print("[BS] focus pin rid=\(pinned) is stale — re-latching to rid=\(rocketID)")
                    focusPinGraceUntil = Date().addingTimeInterval(Self.focusPinGraceInterval)
                    // setFocus, NOT noteAutoFocus. noteAutoFocus only writes the
                    // fleet cache when it is still nil, so healing through it
                    // would fix this device and leave the STALE rid in bsFocus —
                    // which BLEFleet.adopt re-seeds on the very next reconnect,
                    // resurrecting the wedge. setFocus overwrites the cache and
                    // also re-mirrors telemetry, re-points the latched fix, and
                    // pushes cmd 45 so the BS stops relaying under the pin we
                    // just abandoned (its own fallback is disabled while pinned).
                    fleet?.setFocus(baseStation: self, rocketID: rocketID)
                }

                // Record the fix for EVERY relayed rocket (map/roster read the
                // fleet cache), keyed by (networkID, rocketID) — rocket IDs
                // are only unique per network (#390 two-pair support). The BS
                // only forwards packets matching its own network id, so its
                // networkID is the right scope for its relayed rockets.
                let fix = fleet?.recordRocketFix(
                    from: newTelemetry,
                    key: RocketKey(networkID: networkID, rocketID: rocketID))

                if rocketID == focusRocketID {
                    self.updateSimBannerLatch(newTelemetry)
                    self.telemetry = newTelemetry
                    // Mirror the latched fix (#140).  Only assign when the
                    // cache returns a non-nil value so a GPS-less packet
                    // can't blank the marker.
                    if let fix { self.lastValidRocketFix = fix }
                    // The relayed JSON carries the full rocket state (st/aapo/
                    // lnch/land/mspd/palt — see TR_BLE_To_APP.cpp).  Without
                    // this call voice callouts only fire when paired directly
                    // to the rocket, never during a real flight (#138).
                    // Focused rocket only — two interleaved flights' callouts
                    // are noise (#390).
                    self.flightAnnouncer?.processTelemetry(newTelemetry)
                }
                return
            }

            self.updateSimBannerLatch(newTelemetry)
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
               let fix = fleet?.recordRocketFix(
                   from: newTelemetry,
                   key: RocketKey(networkID: networkID, rocketID: rocketID)) {
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

    /// 0xCE — rail-off pyro-test refusal (OC sendPyroTestRefusal).
    /// Frame: [0]=0xCE [1]=refused BLE cmd (35 cont / 36 fire) [2]=channel
    /// [3]=reason (1 = FC power rail off).
    private func parsePyroTestRefusal(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count >= 4, bytes[0] == 0xCE else {
            print("[PYRO] malformed refusal frame (\(bytes.count) bytes)")
            return
        }
        let refusal = PyroTestRefusal(cmd: bytes[1], channel: bytes[2],
                                      reason: bytes[3], at: Date())
        pyroTestRefusal = refusal
        // A refused continuity test gets no reading back — close the TESTING
        // window now instead of letting the spinner run out its 2.5 s
        // implying a round trip happened.
        if refusal.cmd == 35 {
            contTestPendingUntil.removeValue(forKey: refusal.channel)
        }
        print("[PYRO] cmd \(refusal.cmd) CH\(refusal.channel) refused by OC (reason \(refusal.reason))")
    }

    private func parseRocketStorageStats(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count >= 15, bytes[0] == 0xCC,
              let stats = RocketStorageStats.decode(Array(bytes[1...])) else { return }
        rocketStorage = stats
    }

    private func parseBaseStationStorageStats(_ data: Data) {
        let bytes = [UInt8](data)
        guard bytes.count >= 27, bytes[0] == 0xCD,
              let stats = BaseStationStorageStats.decode(Array(bytes[1...])) else { return }
        bsStorage = stats
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
