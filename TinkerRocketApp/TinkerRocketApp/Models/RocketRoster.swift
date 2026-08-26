//
//  RocketRoster.swift
//  TinkerRocketApp
//
//  #390: the rocket-centric model. A RocketSubject is one logical rocket,
//  merged across every link that can currently reach it — a direct BLE
//  connection and/or one or more base stations relaying it over LoRa.
//  The roster is rebuilt on demand from live BLEDevice/RemoteRocket state;
//  it holds references, not copies, so SwiftUI views observe the leaf
//  objects for per-frame updates and only re-derive the roster when the
//  set of links/rockets changes.
//
//  Identity is (networkID, rocketID) — rocket IDs are only unique within a
//  network, and two base-station/rocket pairs on different network IDs may
//  legitimately reuse the same rocket ID.
//

import Foundation
import CoreBluetooth

/// Network-scoped rocket identity. `rocketID` alone is only unique within
/// one network id, so every per-rocket cache/selection keys on the pair.
struct RocketKey: Hashable {
    let networkID: UInt8
    let rocketID: UInt8
}

/// How recently a rocket was heard from, across its best link.
/// Thresholds mirror the BS firmware's BLE_TELEMETRY_STALE_MS (3 s) so the
/// app and the base station agree on what "stale" means.
enum RocketFreshness: Equatable {
    case live
    case stale(age: TimeInterval)
    case lost(lastSeen: Date?)

    static let staleAfter: TimeInterval = 3.0
    static let lostAfter: TimeInterval = 60.0

    /// Compact age for operator-facing labels: seconds under a minute, then
    /// whole minutes.  Shared so the section header, the stale advisory line
    /// and the held go/no-go all phrase the same age identically — three
    /// different renderings of "how old is this" on one screen is how an
    /// operator ends up trusting the freshest-sounding one.
    /// `.lost(lastSeen: nil)` — a rocket in the roster that has never been
    /// heard — yields a non-finite age, and `Int(Double.infinity)` TRAPS in
    /// Swift.  Callers get a word instead of a crash.
    static func ageText(_ age: TimeInterval) -> String {
        guard age.isFinite, age >= 0 else { return "unknown" }
        return age < 60 ? "\(Int(age)) s" : "\(Int(age) / 60) min"
    }

    /// How much to fade a card stack fed by this stream.  Matches the
    /// focused dashboard's own 0.5 (DashboardView's `staleOpacity`) so a
    /// rocket does not change appearance when it gains or loses focus.
    var cardOpacity: Double {
        switch self {
        case .live:  return 1.0
        case .stale: return 0.5
        case .lost:  return 0.35
        }
    }

    /// Age to hand the go/no-go card, or nil while live.
    func staleAgeSec(now: Date = Date()) -> TimeInterval? {
        switch self {
        case .live:               return nil
        case .stale(let age):     return age
        case .lost(let seen):     return seen.map { now.timeIntervalSince($0) } ?? .infinity
        }
    }

    static func from(lastSeen: Date?, now: Date) -> RocketFreshness {
        guard let seen = lastSeen else { return .lost(lastSeen: nil) }
        let age = now.timeIntervalSince(seen)
        if age <= staleAfter { return .live }
        if age <= lostAfter { return .stale(age: age) }
        return .lost(lastSeen: seen)
    }
}

/// One base station currently relaying a rocket.
struct RelayPath {
    let baseStation: BLEDevice
    let remote: RemoteRocket
}

/// The transport a rocket-targeted command should take.
enum RocketCommandLink {
    case direct(BLEDevice)
    /// Send via this base station's targeted relay (BS cmd 50 → rid).
    case relay(baseStation: BLEDevice, rocketID: UInt8)
}

/// One logical rocket as the app sees it right now.
struct RocketSubject: Identifiable {
    /// Stable identity: network-scoped for identified rockets; a direct
    /// link whose config_identity readback hasn't arrived yet (rocketID 0)
    /// is provisionally identified by its peripheral so it can render as
    /// "identifying…" without colliding with anything.
    enum Identity: Hashable {
        case known(RocketKey)
        case identifying(UUID)
    }

    let id: Identity
    let name: String
    let direct: BLEDevice?
    let relays: [RelayPath]

    var key: RocketKey? {
        if case .known(let k) = id { return k }
        return nil
    }

    /// True while only reachable through a base station.
    var isRelayOnly: Bool { direct == nil && !relays.isEmpty }

    /// Latest telemetry from the best source: a connected direct link wins,
    /// otherwise the most recently heard relay.
    var telemetry: TelemetryData? {
        if let d = direct, d.isConnected { return d.telemetry }
        return freshestRelay?.remote.telemetry
    }

    var freshestRelay: RelayPath? {
        relays.max { $0.remote.lastSeen < $1.remote.lastSeen }
    }

    /// When this rocket was last heard on ANY link.
    var lastSeen: Date? {
        var candidates: [Date] = relays.map { $0.remote.lastSeen }
        if let d = direct, d.isConnected, let t = d.lastTelemetryAt {
            candidates.append(t)
        }
        return candidates.max()
    }

    func freshness(now: Date = Date()) -> RocketFreshness {
        RocketFreshness.from(lastSeen: lastSeen, now: now)
    }

    /// Preferred transport for commands: a live direct BLE link
    /// unconditionally (writes work even between telemetry frames), else
    /// the foreground base station when it carries this rocket, else the
    /// relay that heard it most recently.
    func commandLink(foregroundBSID: UUID? = nil) -> RocketCommandLink? {
        if let d = direct, d.isConnected { return .direct(d) }
        guard !relays.isEmpty else { return nil }
        // Only honour the foreground preference when one is actually set —
        // a bare `id == foregroundBSID` would match nil == nil and pick an
        // arbitrary relay instead of the freshest.
        let foreground = foregroundBSID.flatMap { fg in
            relays.first { $0.baseStation.peripheral?.identifier == fg }
        }
        guard let relay = foreground ?? freshestRelay else { return nil }
        return .relay(baseStation: relay.baseStation,
                      rocketID: relay.remote.rocketID)
    }
}

enum RocketRoster {

    /// Build the merged roster from the currently connected links.
    /// Pure with respect to its inputs so it can be unit-tested by
    /// constructing peripheral-less BLEDevices.
    ///
    /// Keys are derived from the LIVE networkID on each device, so an
    /// entry migrates from (0, rid) to (nid, rid) when the identity
    /// readback lands — the roster never splits one rocket in two.
    static func build(devices: [BLEDevice]) -> [RocketSubject] {
        var direct: [RocketKey: BLEDevice] = [:]
        var identifying: [(UUID, BLEDevice)] = []
        var relays: [RocketKey: [RelayPath]] = [:]

        for device in devices where device.isConnected {
            switch device.deviceType {
            case .rocket, .unknown:
                // .unknown still gets a provisional row: the link exists and
                // the user connected it on purpose; the role usually
                // resolves within a beat (#330).
                if device.deviceType == .rocket, device.rocketID > 0 {
                    let key = RocketKey(networkID: device.networkID,
                                        rocketID: device.rocketID)
                    direct[key] = device
                } else {
                    let pid = device.peripheral?.identifier ?? UUID()
                    identifying.append((pid, device))
                }
            case .baseStation:
                for remote in device.remoteRockets {
                    let key = RocketKey(networkID: device.networkID,
                                        rocketID: remote.rocketID)
                    relays[key, default: []].append(
                        RelayPath(baseStation: device, remote: remote))
                }
            }
        }

        var subjects: [RocketSubject] = []
        let knownKeys = Set(direct.keys).union(relays.keys)
        for key in knownKeys.sorted(by: {
            ($0.networkID, $0.rocketID) < ($1.networkID, $1.rocketID)
        }) {
            let dev = direct[key]
            let paths = relays[key] ?? []
            let name = resolveName(key: key, direct: dev, relays: paths)
            subjects.append(RocketSubject(id: .known(key),
                                          name: name,
                                          direct: dev,
                                          relays: paths))
        }
        for (pid, device) in identifying {
            subjects.append(RocketSubject(id: .identifying(pid),
                                          name: device.displayName,
                                          direct: device,
                                          relays: []))
        }
        return subjects
    }

    private static func resolveName(key: RocketKey,
                                    direct: BLEDevice?,
                                    relays: [RelayPath]) -> String {
        if let d = direct, !d.displayName.isEmpty { return d.displayName }
        if let named = relays.first(where: { !$0.remote.unitName.isEmpty }) {
            return named.remote.unitName
        }
        return "Rocket \(key.rocketID)"
    }
}
