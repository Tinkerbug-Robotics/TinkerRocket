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

            if let pendingName = rec.pendingName {
                rec.pendingName = nil
                if pendingName != name {
                    rec.name = pendingName
                    pusher?.sendSetUnitName(pendingName)
                }
            }
            if let pendingNid = rec.pendingNetworkID {
                rec.pendingNetworkID = nil
                if pendingNid != networkID {
                    rec.networkID = pendingNid
                    pusher?.sendSetNetworkID(pendingNid)
                }
            }
            if let pendingRid = rec.pendingRocketID {
                rec.pendingRocketID = nil
                if pendingRid != rocketID {
                    rec.rocketID = pendingRid
                    pusher?.sendSetRocketID(pendingRid)
                }
            }
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
    /// offline it's queued for the next connect.
    func setName(_ newName: String, for unitID: String, pusher: DeviceIdentityPusher?) {
        let trimmed = String(newName.trimmingCharacters(in: .whitespacesAndNewlines).prefix(20))
        guard !trimmed.isEmpty, !unitID.isEmpty else { return }
        withRecord(unitID) { rec in
            if let pusher {
                pusher.sendSetUnitName(trimmed)
                rec.name = trimmed
                rec.pendingName = nil
            } else {
                rec.pendingName = trimmed == rec.name ? nil : trimmed
            }
        }
    }

    func setNetworkID(_ nid: UInt8, for unitID: String, pusher: DeviceIdentityPusher?) {
        guard nid > 0, !unitID.isEmpty else { return }   // 0 is the unset sentinel (#150)
        withRecord(unitID) { rec in
            if let pusher {
                pusher.sendSetNetworkID(nid)
                rec.networkID = nid
                rec.pendingNetworkID = nil
            } else {
                rec.pendingNetworkID = nid == rec.networkID ? nil : nid
            }
        }
    }

    func setRocketID(_ rid: UInt8, for unitID: String, pusher: DeviceIdentityPusher?) {
        guard (1...254).contains(rid), !unitID.isEmpty else { return }  // firmware rejects 0/255
        withRecord(unitID) { rec in
            if let pusher {
                pusher.sendSetRocketID(rid)
                rec.rocketID = rid
                rec.pendingRocketID = nil
            } else {
                rec.pendingRocketID = rid == rec.rocketID ? nil : rid
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
