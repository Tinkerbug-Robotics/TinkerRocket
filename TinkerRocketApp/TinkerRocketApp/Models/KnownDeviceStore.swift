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
