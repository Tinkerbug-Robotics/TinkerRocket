//
//  KnownDeviceStore.swift
//  TinkerRocketApp
//
//  App-side registry of every physical device (rocket / base station) the
//  app has ever completed an identity readback with, keyed by the immutable
//  hardware unitID.  Backs the "My Devices" management screen: names and
//  network IDs live ON the device (identity NVS, #150), so this registry is
//  a cache of the last readback plus a queue of edits made while the device
//  was offline.  Pending edits are pushed on the next config_identity
//  readback; the firmware echoes a fresh readback after every identity-set
//  command (cmds 40/41/42), which folds device truth back into the record.
//
//  Subsumes the legacy "knownDeviceIDs" UserDefaults array that only gated
//  the first-connect provisioning sheet — migrated once, then removed.
//

import Foundation
import Combine

/// The three identity pushes + nothing else, so the store can be unit-tested
/// with a spy instead of a live CBPeripheral (same seam as TelemetryAnnouncer).
protocol DeviceIdentityPusher: AnyObject {
    func sendSetUnitName(_ name: String)
    func sendSetNetworkID(_ nid: UInt8)
    func sendSetRocketID(_ rid: UInt8)
}

extension BLEDevice: DeviceIdentityPusher {}

extension BLEDeviceType: Codable {}

extension String {
    /// Longest prefix that fits `maxBytes` of UTF-8 without splitting a
    /// Character.  The firmware identity handlers hard-reject longer
    /// payloads (plen > 20) with no write and no readback echo, so a
    /// character-count clamp is not enough for multibyte names.
    func utf8Clamped(maxBytes: Int) -> String {
        var s = self
        while s.utf8.count > maxBytes { s.removeLast() }
        return s
    }
}

/// One physical device as last seen over BLE, plus queued offline edits.
struct KnownDevice: Identifiable, Codable, Equatable {
    let unitID: String              // immutable hardware id — the key
    var name: String = ""           // device's own unit_name at last readback
    var deviceType: BLEDeviceType = .unknown
    var networkID: UInt8 = 0        // last readback nid (0 = factory/unset)
    var rocketID: UInt8 = 0         // rockets only; 0 = unset
    var provisioned: Bool = false   // user finished (or skipped) first-connect setup
    var lastSeen: Date?

    // Firmware and boards as last reported over Bluetooth, so a rocket heard
    // only through a base station can still say what it runs. `ocFirmware`
    // is the MCU that answers BLE (config_identity "fw"): the out computer on
    // a rocket, the base station itself on a base station. The other three
    // come from the out computer's fc_identity relay. nil = never reported.
    var ocFirmware: String?
    var fcFirmware: String?
    var ocBoard: String?
    var fcBoard: String?
    /// When the four fields above were last written (a Bluetooth link).
    var firmwareSeen: Date?

    // Edits queued while the device was offline; cleared when pushed.
    var pendingName: String?
    var pendingNetworkID: UInt8?
    var pendingRocketID: UInt8?

    var id: String { unitID }

    var hasPendingChanges: Bool {
        pendingName != nil || pendingNetworkID != nil || pendingRocketID != nil
    }

    // Target state for display: what the device will be once pending applies.
    var effectiveName: String { pendingName ?? name }
    var effectiveNetworkID: UInt8 { pendingNetworkID ?? networkID }
    var effectiveRocketID: UInt8 { pendingRocketID ?? rocketID }

    init(unitID: String) {
        self.unitID = unitID
    }

    // Tolerate additive schema changes: decode field-by-field with defaults
    // so a registry written by a newer build never bricks an older one.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        unitID = try c.decode(String.self, forKey: .unitID)
        name = try c.decodeIfPresent(String.self, forKey: .name) ?? ""
        deviceType = (try? c.decodeIfPresent(BLEDeviceType.self, forKey: .deviceType)) ?? .unknown
        networkID = try c.decodeIfPresent(UInt8.self, forKey: .networkID) ?? 0
        rocketID = try c.decodeIfPresent(UInt8.self, forKey: .rocketID) ?? 0
        provisioned = try c.decodeIfPresent(Bool.self, forKey: .provisioned) ?? false
        lastSeen = try c.decodeIfPresent(Date.self, forKey: .lastSeen)
        ocFirmware = try c.decodeIfPresent(String.self, forKey: .ocFirmware)
        fcFirmware = try c.decodeIfPresent(String.self, forKey: .fcFirmware)
        ocBoard = try c.decodeIfPresent(String.self, forKey: .ocBoard)
        fcBoard = try c.decodeIfPresent(String.self, forKey: .fcBoard)
        firmwareSeen = try c.decodeIfPresent(Date.self, forKey: .firmwareSeen)
        pendingName = try c.decodeIfPresent(String.self, forKey: .pendingName)
        pendingNetworkID = try c.decodeIfPresent(UInt8.self, forKey: .pendingNetworkID)
        pendingRocketID = try c.decodeIfPresent(UInt8.self, forKey: .pendingRocketID)
    }
}

final class KnownDeviceStore: ObservableObject {
    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim, which aborts when the object is deallocated inside a
    // synchronous XCTest.  No deinit-time logic here, so the hop is free to skip.
    nonisolated deinit {}

    /// Rockets first, then base stations, each sorted by display name.
    @Published private(set) var devices: [KnownDevice] = []

    private let defaults: UserDefaults

    private static let storeKey = "knownDevices.v1"
    private static let legacyKnownIDsKey = "knownDeviceIDs"

    init(defaults: UserDefaults = .standard) {
        self.defaults = defaults
        load()
        migrateLegacyIfNeeded()
    }

    // MARK: - Lookup

    func device(for unitID: String) -> KnownDevice? {
        devices.first { $0.unitID == unitID }
    }

    /// Gates the first-connect provisioning sheet.  Empty unitID reads as
    /// provisioned so the sheet never pops before the identity readback.
    func isProvisioned(_ unitID: String) -> Bool {
        guard !unitID.isEmpty else { return true }
        return device(for: unitID)?.provisioned ?? false
    }

    /// Best-effort device type for a scan result, matched by advertised
    /// name.  The firmware advertises the raw unit name, and renames happen
    /// through this app, so a renamed device's adv name equals its record's
    /// name.  nil when there's no single unambiguous match (unknown name,
    /// or two records sharing one).
    func deviceType(forAdvertisedName name: String) -> BLEDeviceType? {
        guard !name.isEmpty else { return nil }
        let matches = devices.filter { $0.name == name && $0.deviceType != .unknown }
        guard matches.count == 1 else { return nil }
        return matches[0].deviceType
    }

    // MARK: - Rocket IDs (three rockets, one base station)

    /// Rocket IDs the OTHER known rockets on `networkID` hold — target values,
    /// so an ID queued for a rocket that is offline counts as taken. Base
    /// stations carry no rocket ID; a record of unknown type (pre-"dt"
    /// firmware) counts when it has one.
    func rocketIDsInUse(networkID: UInt8, excluding unitID: String) -> Set<UInt8> {
        Set(devices.lazy
            .filter { $0.unitID != unitID && $0.deviceType != .baseStation }
            .filter { $0.effectiveNetworkID == networkID && $0.effectiveRocketID > 0 }
            .map(\.effectiveRocketID))
    }

    /// Lowest rocket ID from 1 that neither another known rocket on the
    /// network nor `alsoInUse` (the rockets the fleet can hear right now)
    /// holds.
    func nextFreeRocketID(networkID: UInt8, excluding unitID: String,
                          alsoInUse: Set<UInt8> = []) -> UInt8 {
        Self.lowestFreeRocketID(avoiding: rocketIDsInUse(networkID: networkID, excluding: unitID)
                                    .union(alsoInUse))
    }

    static func lowestFreeRocketID(avoiding taken: Set<UInt8>) -> UInt8 {
        (UInt8(1)...UInt8(254)).first { !taken.contains($0) } ?? 1
    }

    /// What the provisioning sheet pre-fills: the rocket's own ID when it is
    /// valid and nobody else holds it, else the lowest free one. Keeping a
    /// taken ID is how two rockets end up interleaving under one ID on the
    /// base station (the `dup` warning).
    static func provisioningRocketID(reported: UInt8, taken: Set<UInt8>) -> UInt8 {
        if (1...254).contains(reported), !taken.contains(reported) { return reported }
        return lowestFreeRocketID(avoiding: taken)
    }

    /// The transmit slot a rocket ID asks for when slots are on: (rid - 1) % 3,
    /// one third of the 500 ms period each (RocketComputerTypes.h,
    /// LORA_TX_SLOT_COUNT). The base station moves a rocket whose slot is
    /// taken, so a shared slot costs a reshuffle, not a collision — but only
    /// IDs 1, 2 and 3 are certain to land where they ask. nil for ID 0.
    static func transmitSlot(rocketID: UInt8) -> Int? {
        rocketID > 0 ? (Int(rocketID) - 1) % 3 : nil
    }

    /// Other known rockets on the same network that collide with `unitID`:
    /// the same rocket ID (the base station cannot tell them apart), or a
    /// different ID that asks for the same transmit slot (the base station has
    /// to move one of them). Target values, like the list.
    struct RocketIDConflicts: Equatable {
        var sameID: [KnownDevice] = []
        var sharedSlot: [KnownDevice] = []
    }

    func rocketIDConflicts(for unitID: String) -> RocketIDConflicts {
        guard let rec = device(for: unitID), rec.deviceType != .baseStation,
              let slot = Self.transmitSlot(rocketID: rec.effectiveRocketID)
        else { return RocketIDConflicts() }
        var out = RocketIDConflicts()
        for other in devices where other.unitID != unitID && other.deviceType != .baseStation
            && other.effectiveNetworkID == rec.effectiveNetworkID && other.effectiveRocketID > 0 {
            if other.effectiveRocketID == rec.effectiveRocketID {
                out.sameID.append(other)
            } else if Self.transmitSlot(rocketID: other.effectiveRocketID) == slot {
                out.sharedSlot.append(other)
            }
        }
        return out
    }

    /// The registry record for a rocket heard as `rocketID` on `networkID`,
    /// by its last REPORTED identity (a queued change has not happened yet).
    /// Two records claiming the pair is the duplicate-ID case: `name` (the
    /// relayed unit name) picks between them, and with no name to go on the
    /// answer is nil rather than a guess.
    func rocket(networkID: UInt8, rocketID: UInt8, name: String? = nil) -> KnownDevice? {
        guard rocketID > 0 else { return nil }
        let matches = devices.filter {
            $0.deviceType != .baseStation && $0.networkID == networkID && $0.rocketID == rocketID
        }
        if matches.count <= 1 { return matches.first }
        guard let name, !name.isEmpty else { return nil }
        let named = matches.filter { $0.name == name }
        return named.count == 1 ? named[0] : nil
    }

    // MARK: - Readback intake

    /// Fold a config_identity readback into the registry, then push any
    /// pending offline edits.  Pending fields are cleared at push time
    /// (fire-and-forget): the firmware echoes a fresh readback after each
    /// identity-set command, so the record converges on device truth, and
    /// clearing first makes the echo pass a plain upsert — no re-push loop
    /// even if the firmware rejects or clamps a value.
    func deviceDidReportIdentity(unitID: String,
                                 name: String,
                                 deviceType: BLEDeviceType,
                                 networkID: UInt8,
                                 rocketID: UInt8,
                                 firmware: String? = nil,
                                 pusher: DeviceIdentityPusher?) {
        guard !unitID.isEmpty else { return }
        withRecord(unitID) { rec in
            rec.name = name
            // Keep a previously learned type if this readback doesn't know it
            // (pre-"dt" firmware) — a device never changes species.
            if deviceType != .unknown { rec.deviceType = deviceType }
            rec.networkID = networkID
            rec.rocketID = rocketID
            rec.lastSeen = Date()
            // Absent "fw" (pre-#8 firmware) keeps what an earlier link said.
            if let firmware, !firmware.isEmpty {
                rec.ocFirmware = firmware
                rec.firmwareSeen = rec.lastSeen
            }

            applyPending(&rec, readback: name,
                         current: \.name, pending: \.pendingName,
                         push: pusher.map { p in { p.sendSetUnitName($0) } })
            applyPending(&rec, readback: networkID,
                         current: \.networkID, pending: \.pendingNetworkID,
                         push: pusher.map { p in { p.sendSetNetworkID($0) } })
            applyPending(&rec, readback: rocketID,
                         current: \.rocketID, pending: \.pendingRocketID,
                         push: pusher.map { p in { p.sendSetRocketID($0) } })
        }
    }

    /// Apply one queued offline edit against a fresh readback.  The queue is
    /// cleared BEFORE pushing so the firmware's echo readback runs this as a
    /// plain upsert — no re-push loop even if the device rejects or clamps
    /// the value.  A queued value the device already has is just dropped.
    private func applyPending<V: Equatable>(_ rec: inout KnownDevice,
                                            readback: V,
                                            current: WritableKeyPath<KnownDevice, V>,
                                            pending: WritableKeyPath<KnownDevice, V?>,
                                            push: ((V) -> Void)?) {
        guard let queued = rec[keyPath: pending] else { return }
        rec[keyPath: pending] = nil
        guard queued != readback else { return }
        rec[keyPath: current] = queued
        push?(queued)
    }

    /// Fold an fc_identity readback (the out computer's relay of the flight
    /// computer's version, plus both boards' provisioned revisions) into the
    /// record. Boards are written as reported, nil included ("unprovisioned"
    /// is a real answer); a firmware version the OC could not read
    /// ("unknown", or empty) keeps the last good one.
    func deviceDidReportFlightComputer(unitID: String,
                                       fcFirmware: String,
                                       fcBoard: String?,
                                       ocBoard: String?) {
        guard !unitID.isEmpty else { return }
        withRecord(unitID) { rec in
            if !fcFirmware.isEmpty, fcFirmware != "unknown" { rec.fcFirmware = fcFirmware }
            rec.fcBoard = fcBoard
            rec.ocBoard = ocBoard
            rec.firmwareSeen = Date()
        }
    }

    /// Called by the provisioning sheet on Save/Skip so the device stops
    /// re-prompting.  Creates the record if the readback hasn't yet.
    func markProvisioned(_ unitID: String) {
        guard !unitID.isEmpty else { return }
        withRecord(unitID) { $0.provisioned = true }
    }

    // MARK: - Edits (push now when connected, queue when offline)

    /// Rename.  With a live pusher the new name is sent immediately and the
    /// record updated optimistically (the firmware's readback echo confirms);
    /// offline it's queued for the next connect.  Clamped to the firmware's
    /// 20-byte payload limit so the device can never silently reject it.
    func setName(_ newName: String, for unitID: String, pusher: DeviceIdentityPusher?) {
        let clamped = newName.trimmingCharacters(in: .whitespacesAndNewlines)
            .utf8Clamped(maxBytes: 20)
        guard !clamped.isEmpty, !unitID.isEmpty else { return }
        applyEdit(clamped, for: unitID, current: \.name, pending: \.pendingName,
                  push: pusher.map { p in { p.sendSetUnitName($0) } })
    }

    func setNetworkID(_ nid: UInt8, for unitID: String, pusher: DeviceIdentityPusher?) {
        guard nid > 0, !unitID.isEmpty else { return }   // 0 is the unset sentinel (#150)
        applyEdit(nid, for: unitID, current: \.networkID, pending: \.pendingNetworkID,
                  push: pusher.map { p in { p.sendSetNetworkID($0) } })
    }

    func setRocketID(_ rid: UInt8, for unitID: String, pusher: DeviceIdentityPusher?) {
        guard (1...254).contains(rid), !unitID.isEmpty else { return }  // firmware rejects 0/255
        applyEdit(rid, for: unitID, current: \.rocketID, pending: \.pendingRocketID,
                  push: pusher.map { p in { p.sendSetRocketID($0) } })
    }

    /// Shared push-now-vs-queue tail for the setters above.  Connected:
    /// push + optimistic record update + clear any stale queue.  Offline:
    /// queue the target, where queueing the device's current value just
    /// clears the queue (a changed-my-mind no-op).
    private func applyEdit<V: Equatable>(_ value: V, for unitID: String,
                                         current: WritableKeyPath<KnownDevice, V>,
                                         pending: WritableKeyPath<KnownDevice, V?>,
                                         push: ((V) -> Void)?) {
        withRecord(unitID) { rec in
            if let push {
                push(value)
                rec[keyPath: current] = value
                rec[keyPath: pending] = nil
            } else {
                rec[keyPath: pending] = value == rec[keyPath: current] ? nil : value
            }
        }
    }

    /// Drop all queued offline edits without touching the device.
    func clearPending(for unitID: String) {
        guard device(for: unitID)?.hasPendingChanges == true else { return }
        withRecord(unitID) { rec in
            rec.pendingName = nil
            rec.pendingNetworkID = nil
            rec.pendingRocketID = nil
        }
    }

    /// Drop the record entirely.  The device is treated as brand new on its
    /// next connect (provisioning sheet pops again).
    func forget(_ unitID: String) {
        devices.removeAll { $0.unitID == unitID }
        persist()
    }

    // MARK: - Record plumbing

    /// Get-or-create the record, mutate it, then sort + persist + publish.
    private func withRecord(_ unitID: String, _ mutate: (inout KnownDevice) -> Void) {
        var rec = device(for: unitID) ?? KnownDevice(unitID: unitID)
        mutate(&rec)
        devices.removeAll { $0.unitID == unitID }
        devices.append(rec)
        sortDevices()
        persist()
    }

    private func sortDevices() {
        func rank(_ t: BLEDeviceType) -> Int {
            switch t {
            case .rocket: return 0
            case .baseStation: return 1
            case .unknown: return 2
            }
        }
        devices.sort {
            if rank($0.deviceType) != rank($1.deviceType) {
                return rank($0.deviceType) < rank($1.deviceType)
            }
            let n0 = $0.effectiveName, n1 = $1.effectiveName
            if n0.localizedCaseInsensitiveCompare(n1) != .orderedSame {
                return n0.localizedCaseInsensitiveCompare(n1) == .orderedAscending
            }
            return $0.unitID < $1.unitID
        }
    }

    // MARK: - Persistence

    private func load() {
        guard let data = defaults.data(forKey: Self.storeKey),
              let decoded = try? JSONDecoder().decode([KnownDevice].self, from: data)
        else { return }
        devices = decoded
        sortDevices()
    }

    private func persist() {
        guard let data = try? JSONEncoder().encode(devices) else { return }
        defaults.set(data, forKey: Self.storeKey)
    }

    /// One-time fold of the legacy provisioning-gate array ("knownDeviceIDs",
    /// bare unitIDs) into real records.  Names/types arrive on each device's
    /// next readback.  The legacy key is deleted afterwards so a forgotten
    /// device can't be resurrected by re-migration.
    private func migrateLegacyIfNeeded() {
        guard let legacy = defaults.stringArray(forKey: Self.legacyKnownIDsKey) else { return }
        for unitID in legacy where !unitID.isEmpty {
            if let idx = devices.firstIndex(where: { $0.unitID == unitID }) {
                devices[idx].provisioned = true
            } else {
                var rec = KnownDevice(unitID: unitID)
                rec.provisioned = true
                devices.append(rec)
            }
        }
        sortDevices()
        persist()
        defaults.removeObject(forKey: Self.legacyKnownIDsKey)
    }
}
