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
    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim (swift_task_deinitOnExecutorMainActorBackDeploy), which
    // aborts with "pointer being freed was not allocated" when the object is
    // deallocated inside a synchronous XCTest. This class has no deinit-time
    // logic, so skipping the executor hop is free — and it un-crashes every
    // test that creates and tears down an instance.
    nonisolated deinit {}

    // MARK: - Published state

    @Published var isScanning = false
    // #394: true only when the user explicitly asked to add a device (the "Add"
    // button), false for automatic/background scans (reconnect fallback,
    // auto-scan-on-disconnect, session restore). The dashboard's "Add Device"
    // sheet gates on this so a returning unit re-pairing silently doesn't pop
    // an unsolicited modal.
    @Published var userInitiatedScan = false
    @Published var statusMessage = "Not connected"
    @Published var discoveredDevices: [DiscoveredDevice] = []

    /// Currently connected devices (for now, max 1 — multi-connect in PR #4).
    @Published var devices: [BLEDevice] = []

    /// Registry of every device ever provisioned/seen, keyed by hardware
    /// unitID.  Lives on the fleet (not a view) because BLEDevice feeds it
    /// on every config_identity readback — including for non-active devices.
    let knownDevices = KnownDeviceStore()

    /// The live BLEDevice for a hardware unitID, if that device is currently
    /// connected AND has completed its identity readback.  Used by the
    /// device manager to decide push-now vs queue-for-next-connect.
    func connectedDevice(unitID: String) -> BLEDevice? {
        guard !unitID.isEmpty else { return nil }
        return devices.first { $0.unitID == unitID }
    }

    /// Which device the dashboard is currently showing.
    @Published var activeDeviceID: UUID?

    /// Latest usable rocket GPS fix per rocket.  Lives on the fleet
    /// (not on a BLEDevice) because BLEFleet destroys+recreates a
    /// BLEDevice on every BLE disconnect/reconnect — without an outer
    /// cache, the map would blank every time the phone briefly loses
    /// the base station.  In-memory only; cleared on app kill.  #140.
    /// Keyed by (networkID, rocketID) — rocket IDs are only unique per
    /// network, and two BS/rocket pairs on different networks may reuse
    /// an id (#390). The key uses the LIVE networkID at record time, so
    /// a fix recorded in the first ~second after connect (before the
    /// identity readback populates networkID) can land under nid 0 and
    /// be superseded once the real nid is known — harmless, the next
    /// valid fix re-latches under the right key.
    @Published var lastValidRocketFixes: [RocketKey: LastValidRocketFix] = [:]

    /// #390: roster rockets the user chose to hide from the dashboard.
    /// Everything discovered displays by default; hiding is a pure view
    /// filter (it never changes radios or command routing).
    @Published var hiddenRocketKeys: Set<RocketKey> = []

    /// #390: which base station's "pair" (the BS + the rockets it carries)
    /// the dashboard foregrounds. nil = auto (the first connected BS).
    /// Only set explicitly by the user via the pair switcher.
    @Published var foregroundBSID: UUID?

    /// #390: per-base-station radio focus (rocketID), keyed by peripheral
    /// identifier so it survives the BLEDevice recreation on reconnect.
    /// Auto-latched to the first rocket a BS hears; switched explicitly by
    /// the user. Pushed to the BS via SET_FOCUS_ROCKET when supported.
    @Published var bsFocus: [UUID: UInt8] = [:]

    /// Per-device OTA driver (#8 phase 2, #15 second pass).  Same rationale
    /// as lastValidRocketFixes: BLEDevice is recreated on each reconnect,
    /// so OTASession must live above it — otherwise the "Updated successfully"
    /// state on FirmwareUpdateView gets wiped by the post-OTA reboot's
    /// disconnect cycle. Keyed by peripheral.identifier (stable across
    /// reconnect for the same physical device).
    @Published var otaSessions: [UUID: OTASession] = [:]

    /// Get-or-create the OTA driver for `device`. The same OTASession
    /// instance is returned across reconnects for the same peripheral.
    @MainActor
    func otaSession(for device: BLEDevice) -> OTASession {
        let id = device.peripheral?.identifier ?? UUID()  // transient if no peripheral yet
        if let existing = otaSessions[id] { return existing }
        let session = OTASession(fleet: self, peripheralID: id)
        otaSessions[id] = session
        return session
    }

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

    // MARK: - Rocket roster (#390)

    /// Connected base-station links.
    var baseStations: [BLEDevice] {
        devices.filter { $0.isBaseStation && $0.isConnected }
    }

    /// The base station whose pair is on screen: the user's explicit pick
    /// while that BS is still connected, else the first connected BS.
    var foregroundBaseStation: BLEDevice? {
        if let id = foregroundBSID,
           let bs = baseStations.first(where: { $0.peripheral?.identifier == id }) {
            return bs
        }
        return baseStations.first
    }

    /// Every rocket currently known, merged across links (#390).
    var rockets: [RocketSubject] {
        RocketRoster.build(devices: devices)
    }

    /// The roster minus rockets the user hid — what the dashboard shows.
    var displayedRockets: [RocketSubject] {
        rockets.filter { subject in
            guard let key = subject.key else { return true }  // identifying…
            return !hiddenRocketKeys.contains(key)
        }
    }

    /// Record a base station's sticky auto-focus (first rocket heard) so it
    /// survives that BLEDevice being recreated on reconnect. Never
    /// overwrites an existing choice — that's the point of stickiness.
    /// Pushes the pin to the BS too: its own auto pick is the same
    /// first-heard rocket, but pinning makes the agreement explicit and
    /// disables the BS-side silent fallback while the app is attached.
    func noteAutoFocus(baseStation: BLEDevice, rocketID: UInt8) {
        guard let pid = baseStation.peripheral?.identifier else { return }
        if bsFocus[pid] == nil {
            bsFocus[pid] = rocketID
            baseStation.sendSetFocusRocket(rocketID)
        }
    }

    /// User-driven focus switch for one base station. Re-pins the device's
    /// mirrored stream immediately (cached telemetry + latched fix) so the
    /// dashboard doesn't wait for the next frame, and pushes the pin to the
    /// BS (cmd 45) so hop-follow / stale re-push / uplink targeting move
    /// with it.
    func setFocus(baseStation: BLEDevice, rocketID: UInt8) {
        if let pid = baseStation.peripheral?.identifier {
            bsFocus[pid] = rocketID
        }
        baseStation.focusRocketID = rocketID
        // A deliberate choice — by the user or by the staleness heal — earns a
        // fresh grace window, so the newly pinned rocket cannot be re-healed
        // away before it has had a chance to transmit.
        baseStation.focusPinGraceUntil =
            Date().addingTimeInterval(BLEDevice.focusPinGraceInterval)
        if let remote = baseStation.remoteRockets.first(where: { $0.rocketID == rocketID }) {
            baseStation.telemetry = remote.telemetry
        }
        let key = RocketKey(networkID: baseStation.networkID, rocketID: rocketID)
        baseStation.lastValidRocketFix = lastValidRocketFixes[key]
        baseStation.refreshFocusedRelayFreshness()
        baseStation.sendSetFocusRocket(rocketID)
    }

    // MARK: - Private state

    private var centralManager: CBCentralManager!
    private let serviceUUID = CBUUID(string: "4fafc201-1fb5-459e-8fcc-c5c9c331914b")

    // Reconnection state
    private var reconnectAttempts: Int = 0
    private let maxReconnectAttempts = 8   // #291: was 3 (~7 s); raised for flight dropouts
    private var lastPeripheralIdentifier: UUID?
    /// Peripherals the USER asked to disconnect, awaiting their
    /// didDisconnectPeripheral callback (#836 item 8).
    ///
    /// This was a single fleet-wide Bool. disconnectAll() set it once and then
    /// cancelled every connection in a loop, but the FIRST didDisconnect
    /// consumed it — so with a base station and a rocket both up (an
    /// explicitly supported pairing), the second device fell through to the
    /// automatic-reconnect ladder and came back ~1 s later. On the resulting
    /// didConnect the syncer re-attached and re-pushed the whole active
    /// profile, sendPyroConfig included, to a rocket the user had just
    /// disconnected from.
    ///
    /// Per-peripheral, so each cancel is matched by exactly the callback it
    /// caused. It also fixes a second-order leak of the old Bool: disconnect()
    /// set it even when the device had no peripheral, and with no callback
    /// coming nothing ever cleared it — poisoning the next UNRELATED dropout
    /// into looking user-initiated, which suppressed its reconnect.
    private var userInitiatedDisconnects = UserDisconnectLedger()
    // #571: ALL peripherals handed back by CoreBluetooth state restoration —
    // a BS + direct-rocket session restores two, and keeping only .first
    // silently lost the second (it never reappeared because the restored
    // link's isConnected suppressed the scan trigger).
    private var reconnectingPeripherals: [CBPeripheral] = []

    // Strong references to peripherals while a connection is in flight (#173).
    // CBCentralManager only retains peripherals weakly, so without our own
    // strong reference the CBPeripheral can be deallocated mid-connect (e.g.
    // when `discoveredDevices` is mutated), and CoreBluetooth cancels with
    // "API MISUSE: Cancelling connection for unused peripheral … Did you
    // forget to keep a reference to it?".  Held from connect() until the
    // connection resolves (didConnect / didFailToConnect / didDisconnect).
    private var connectingPeripherals: [UUID: CBPeripheral] = [:]

    // #390: fleet-level views (roster, units bar) derive from state that
    // lives on child objects (device.remoteRockets, identity readbacks).
    // BLEFleet doesn't otherwise republish when a child mutates, so forward
    // each device's objectWillChange while it is connected.
    private var deviceObservers: [UUID: AnyCancellable] = [:]

    /// Wire a freshly connected device into the fleet: back-reference,
    /// focus re-seed (sticky across the BLEDevice recreation on reconnect),
    /// and child-change forwarding for roster-derived views.
    private func adopt(_ device: BLEDevice, peripheralID: UUID) {
        device.fleet = self
        // #547: seed the type from the known-device registry before the
        // config_identity readback — renamed devices carry no TR- prefix
        // and can mis-parse via the legacy substring check.
        seedTypeFromRegistry(device, peripheralID: peripheralID)
        if let focus = bsFocus[peripheralID] {
            device.focusRocketID = focus
            // Arm the same grace window the first-latch path uses. Without it
            // the re-seeded pin is immediately eligible for the staleness check
            // in BLEDevice, whose roster is empty at this instant — so the
            // first rocket heard after every reconnect would steal focus.
            device.focusPinGraceUntil =
                Date().addingTimeInterval(BLEDevice.focusPinGraceInterval)
        }
        deviceObservers[peripheralID] = device.objectWillChange
            .sink { [weak self] _ in self?.objectWillChange.send() }
    }

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

    func startScanning(userInitiated: Bool = false) {
        guard centralManager.state == .poweredOn else {
            statusMessage = "Bluetooth not ready"
            return
        }
        discoveredDevices = []
        statusMessage = "Scanning..."
        isScanning = true
        userInitiatedScan = userInitiated  // #394: only the "Add" button drives the sheet
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
        userInitiatedScan = false  // #394
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
        // Virtual rocket: no peripheral, no CoreBluetooth teardown — stop
        // the driver and remove directly (the didDisconnect path can never
        // fire for it).
        if device === virtualDriver?.device {
            virtualDriver?.stop()
            virtualDriver = nil
            device.onDisconnect()
            devices.removeAll { $0 === device }
            return
        }
        if let peripheral = device.peripheral {
            userInitiatedDisconnects.mark(peripheral.identifier)
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }

    /// Disconnect all devices (convenience for single-device mode).
    func disconnectAll() {
        if let v = virtualDriver {
            v.stop()
            v.device.onDisconnect()
            devices.removeAll { $0 === v.device }
            virtualDriver = nil
        }
        // Mark EVERY peripheral before cancelling any: each cancel is matched
        // by its own callback, so one shared flag could only ever cover one.
        for device in devices {
            if let peripheral = device.peripheral {
                userInitiatedDisconnects.mark(peripheral.identifier)
                centralManager.cancelPeripheralConnection(peripheral)
            }
        }
    }

    // MARK: - Virtual Rocket (iOS twin of Android demo mode, 2026-07-30)

    private var virtualDriver: VirtualRocketDriver?

    /// True while the virtual rocket is in the fleet (the picker's button
    /// flips to a hint instead of stacking duplicates).
    var virtualRocketActive: Bool { virtualDriver != nil }

    /// Add the virtual base station + its scripted rocket to the fleet.
    /// Idempotent; runs entirely without CoreBluetooth (works in the
    /// Simulator, on the couch, in a talk).
    func startVirtualRocket() {
        guard virtualDriver == nil else { return }
        let driver = VirtualRocketDriver()
        virtualDriver = driver
        devices.append(driver.device)
        driver.start()
    }

    // MARK: - Device lookup

    private func device(for peripheral: CBPeripheral) -> BLEDevice? {
        devices.first { $0.peripheral?.identifier == peripheral.identifier }
    }

    /// Seed a just-created BLEDevice's type from what we already know,
    /// BEFORE the config_identity readback arrives.  Registry knowledge
    /// wins over BLEDevice.init's name parse (mirrors DiscoveredDevice's
    /// precedence): the firmware advertises the raw user-set name, so a
    /// renamed device either parses .unknown or — worse — mis-parses via
    /// the legacy substring check (a rocket called "SUBSONIC" reads as a
    /// base station), and a wrong role at connect latches the profile
    /// syncer (#375).  Prefers the scan-time DiscoveredDevice identity:
    /// peripheral.name is iOS-cached and can be nil/stale after a
    /// re-flash, while the scan response carried the real name.  The
    /// readback's "dt" still overrides everything ~1 s later.
    private func seedTypeFromRegistry(_ device: BLEDevice, peripheralID: UUID) {
        let discovered = discoveredDevices.first { $0.id == peripheralID }
        let advertisedName = discovered?.name ?? device.connectedDeviceName
        if let known = discovered?.knownType
            ?? knownDevices.deviceType(forAdvertisedName: advertisedName) {
            device.deviceType = known
        }
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
                         key: RocketKey) -> LastValidRocketFix? {
        if let fresh = LastValidRocketFix.fromTelemetry(telemetry, rocketID: key.rocketID) {
            lastValidRocketFixes[key] = fresh
            return fresh
        }
        return lastValidRocketFixes[key]
    }
}

// MARK: - CBCentralManagerDelegate

extension BLEFleet: CBCentralManagerDelegate {

    func centralManager(_ central: CBCentralManager,
                       willRestoreState dict: [String: Any]) {
        // #571: keep EVERY restored peripheral, not just .first — a two-device
        // session (BS + direct rocket) restores both, and dropping one here
        // lost it for the whole relaunched session.
        if let peripherals = dict[CBCentralManagerRestoredStatePeripheralsKey] as? [CBPeripheral],
           !peripherals.isEmpty {
            print("[BLE] State restoration: holding \(peripherals.count) peripheral(s): "
                  + peripherals.map { $0.name ?? "unknown" }.joined(separator: ", "))
            reconnectingPeripherals = peripherals
        }
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            statusMessage = "Bluetooth ready"
            // #571: resume every restored peripheral that is still connected.
            var resumedAny = false
            var anyNotConnected = false
            for p in reconnectingPeripherals {
                guard p.state == .connected else {
                    // Restored but the link didn't survive — the scan below
                    // picks it back up (this is the case the old single-slot
                    // code silently lost for the second device).
                    anyNotConnected = true
                    continue
                }
                print("[BLE] Restored peripheral still connected — resuming \(p.name ?? "unknown")")
                let device = BLEDevice(peripheral: p, name: p.name ?? "Restored")
                // #290: mirror the didConnect path. Without the fleet backref,
                // fleet?.recordRocketFix() no-ops, so lastValidRocketFix never
                // populates and the map marker / direction arrow / landing anchor
                // stay blank for the whole restored session.
                adopt(device, peripheralID: p.identifier)
                device.onConnect()
                devices.append(device)
                if activeDeviceID == nil { activeDeviceID = p.identifier }
                resumedAny = true
            }
            reconnectingPeripherals = []
            // Scan when anything is missing: nothing restored at all, or a
            // restored link that didn't survive the relaunch. (When every
            // restored link resumed, behave as before — no scan while
            // connected.)
            if !resumedAny || anyNotConnected {
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
        // peripheral.name is iOS-cached and is nil on a fresh-scan discovery
        // until iOS has connected at least once or otherwise persisted the
        // name (e.g. an active pair). After a firmware re-flash that wipes
        // bond state, the cache is invalidated and peripheral.name comes
        // back nil even though the device is advertising correctly — the
        // name lives in the scan-response payload, surfaced through
        // advertisementData[CBAdvertisementDataLocalNameKey]. Prefer that
        // when present so the prefix filter works on first discovery.
        let advertisedName = advertisementData[CBAdvertisementDataLocalNameKey] as? String
        let name = advertisedName ?? peripheral.name ?? "Unknown"
        print("Discovered: \(name) RSSI: \(RSSI)")

        // No name filter here: the scan is already service-UUID-filtered
        // (scanForPeripherals(withServices:)), so anything delivered to this
        // callback IS a TinkerRocket board.  The old belt-and-suspenders
        // TR-*/Tinker name guard silently dropped every device renamed
        // through My Devices — the firmware advertises the raw user-set
        // unit name; only factory defaults carry the TR-R-/TR-B- prefix.

        if let idx = discoveredDevices.firstIndex(where: { $0.id == peripheral.identifier }) {
            discoveredDevices[idx].rssi = RSSI.intValue
            // Upgrade the name when a later report brings a real one. The
            // board carries its name in the SCAN RESPONSE — flags plus the
            // 128-bit service UUID leave only 8 characters of the 31-byte
            // primary payload, too few for a unit name — and iOS reports the
            // plain ADV_IND before the scan response merges into it, so a
            // peripheral's FIRST sighting routinely has no local name and
            // falls back to peripheral.name (the iOS-cached GAP name).
            // Latching that first value pinned the fallback in the row
            // forever, even once the real name arrived milliseconds later.
            // Only a genuinely advertised name upgrades (never the
            // peripheral.name fallback), so the row can't flap between the
            // two sources.
            if let advertisedName, advertisedName != discoveredDevices[idx].name {
                discoveredDevices[idx].name = advertisedName
                discoveredDevices[idx].knownType =
                    knownDevices.deviceType(forAdvertisedName: advertisedName)
            }
        } else {
            let device = DiscoveredDevice(
                id: peripheral.identifier,
                peripheral: peripheral,
                name: name,
                rssi: RSSI.intValue,
                // Renames happen through this app, so a renamed device's
                // advertised name matches its registry record — recover the
                // type the name prefix no longer carries.
                knownType: knownDevices.deviceType(forAdvertisedName: name)
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
        // Any pending user-disconnect marker for this peripheral is now stale:
        // it is connected again, so the NEXT disconnect is a real dropout and
        // must be allowed to reconnect (#836 item 8).
        userInitiatedDisconnects.forget(peripheral.identifier)

        let device = BLEDevice(peripheral: peripheral, name: name)
        adopt(device, peripheralID: peripheral.identifier)
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
        deviceObservers[peripheral.identifier] = nil  // #390

        // Update active device
        if activeDeviceID == peripheral.identifier {
            activeDeviceID = devices.first?.peripheral?.identifier
        }

        if userInitiatedDisconnects.consume(peripheral.identifier) {
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
            // #291: cap the backoff — 2^(n-1) blows up fast once the cap is raised.
            let delay = min(8.0, pow(2.0, Double(reconnectAttempts - 1)))
            DispatchQueue.main.asyncAfter(deadline: .now() + delay) { [weak self] in
                guard let self = self, self.device(for: peripheral) == nil else { return }
                // Retain across the reconnect attempt too (#173).
                self.connectingPeripherals[peripheral.identifier] = peripheral
                self.centralManager.connect(peripheral, options: nil)
            }
        } else {
            // #291: don't abandon the flight after a handful of tries. CB
            // connect() has no timeout — it completes whenever the peripheral
            // returns to range — so leave a connect pending AND fall back to
            // scanning so a returning device re-pairs automatically, instead of
            // the old dead-end ("Disconnected" + cleared list).
            reconnectAttempts = 0
            statusMessage = "Searching for \(peripheral.name ?? "rocket")…"
            connectingPeripherals[peripheral.identifier] = peripheral
            centralManager.connect(peripheral, options: nil)
            startScanning()
        }
    }

    func centralManager(_ central: CBCentralManager,
                       didFailToConnect peripheral: CBPeripheral,
                       error: Error?) {
        print("Failed to connect: \(error?.localizedDescription ?? "Unknown error")")
        // Connection attempt resolved (failed) — release the strong ref (#173).
        connectingPeripherals[peripheral.identifier] = nil
        userInitiatedDisconnects.forget(peripheral.identifier)
        statusMessage = "Connection failed"
    }
}
