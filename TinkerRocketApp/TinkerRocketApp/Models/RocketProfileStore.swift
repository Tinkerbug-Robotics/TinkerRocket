//
//  RocketProfileStore.swift
//  TinkerRocketApp
//
//  Owns the set of rocket profiles and the active selection (issue #132).
//
//  Persistence is one JSON file per profile under Application Support
//  (RocketProfiles/<uuid>.json) plus the active id in UserDefaults.  One
//  file per profile keeps each write small and means a single corrupt file
//  can't take down the whole set — it's skipped on load and the rest
//  survive.  The directory + UserDefaults are injectable so the store can
//  be unit-tested against a temp dir without touching the real app state.
//

import Foundation
import Combine

final class RocketProfileStore: ObservableObject {
    // Workaround for swiftlang/swift#87316: with SWIFT_DEFAULT_ACTOR_ISOLATION
    // = MainActor, the implicit isolated deinit routes through the runtime's
    // back-deploy shim (swift_task_deinitOnExecutorMainActorBackDeploy), which
    // aborts with "pointer being freed was not allocated" when the object is
    // deallocated inside a synchronous XCTest. This class has no deinit-time
    // logic, so skipping the executor hop is free — and it un-crashes every
    // test that creates and tears down an instance.
    nonisolated deinit {}

    @Published private(set) var profiles: [RocketProfile] = []
    @Published private(set) var activeId: UUID?

    private let dir: URL
    private let defaults: UserDefaults

    private static let activeIdKey = "activeRocketProfileId"
    private static let migratedKey = "rocketProfilesMigratedV1"

    /// The active profile, or nil if none is selected (valid empty state —
    /// the UI prompts the user to pick or create one).
    var activeProfile: RocketProfile? {
        guard let id = activeId else { return nil }
        return profiles.first { $0.id == id }
    }

    init(directory: URL? = nil, defaults: UserDefaults = .standard) {
        self.defaults = defaults
        if let directory {
            self.dir = directory
        } else {
            let base = FileManager.default.urls(for: .applicationSupportDirectory,
                                                in: .userDomainMask)[0]
            self.dir = base.appendingPathComponent("RocketProfiles", isDirectory: true)
        }
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)

        load()
        migrateLegacyIfNeeded()
        reconcileActiveId()
    }

    // MARK: - CRUD

    /// Create a new profile with factory defaults.  Does not change the
    /// active selection — the caller decides whether to activate it.
    @discardableResult
    func add(name: String) -> RocketProfile {
        let profile = RocketProfile.makeDefault(name: dedupedName(name))
        profiles.append(profile)
        sortProfiles()
        save(profile)
        return profile
    }

    /// Deep-copy an existing profile under a new id + "(copy)" name.  The
    /// mag cal is intentionally NOT copied: cal is tied to a physical board,
    /// so a duplicate (likely for a different airframe) starts uncalibrated.
    @discardableResult
    func duplicate(_ id: UUID) -> RocketProfile? {
        guard let src = profiles.first(where: { $0.id == id }) else { return nil }
        var copy = src
        copy.id = UUID()
        copy.name = dedupedName("\(src.name) copy")
        copy.createdAt = Date()
        copy.updatedAt = Date()
        copy.lastUsedUnitID = nil
        copy.magCal = nil
        copy.sensorCal = nil
        profiles.append(copy)
        sortProfiles()
        save(copy)
        return copy
    }

    func rename(_ id: UUID, to newName: String) {
        update(id) { $0.name = newName }
    }

    func delete(_ id: UUID) {
        profiles.removeAll { $0.id == id }
        try? FileManager.default.removeItem(at: fileURL(id))
        if activeId == id {
            setActive(profiles.first?.id)
        }
    }

    /// Make `id` the ONE profile bound to `unitID`, releasing any other that
    /// still claims that board.
    ///
    /// Binding has to be exclusive or it isn't a binding. Assigning a second
    /// profile to a rocket used to leave the first one still claiming it, and
    /// the connect-time lookup takes the FIRST match in a list sorted by NAME
    /// — so which profile a board came back on was decided alphabetically
    /// rather than by anything the user did. Bench-caught on #915: set a new
    /// profile on a rocket, connect to another rocket, come back, and the
    /// selection had silently reverted to the older profile.
    func bind(_ id: UUID, toUnitID unitID: String) {
        guard !unitID.isEmpty else { return }
        for other in profiles where other.lastUsedUnitID == unitID && other.id != id {
            update(other.id) { $0.lastUsedUnitID = nil }
        }
        update(id) { $0.lastUsedUnitID = unitID }
    }

    func setActive(_ id: UUID?) {
        activeId = id
        persistActiveId()
    }

    /// Mutate a profile in place, bump `updatedAt`, and persist it.  All
    /// edits funnel through here so persistence + the timestamp can't be
    /// forgotten at a call site.
    func update(_ id: UUID, _ mutate: (inout RocketProfile) -> Void) {
        guard let idx = profiles.firstIndex(where: { $0.id == id }) else { return }
        mutate(&profiles[idx])
        profiles[idx].updatedAt = Date()
        sortProfiles()
        // Re-find: sort may have moved it.
        if let p = profiles.first(where: { $0.id == id }) { save(p) }
    }

    // MARK: - Persistence

    private func fileURL(_ id: UUID) -> URL {
        dir.appendingPathComponent("\(id.uuidString).json")
    }

    private func load() {
        let urls = (try? FileManager.default.contentsOfDirectory(
            at: dir, includingPropertiesForKeys: nil)) ?? []
        let decoder = JSONDecoder()
        var loaded: [RocketProfile] = []
        for url in urls where url.pathExtension == "json" {
            guard let data = try? Data(contentsOf: url),
                  let profile = try? decoder.decode(RocketProfile.self, from: data)
            else { continue }   // skip corrupt/foreign files — don't fail the set
            loaded.append(profile)
        }
        profiles = loaded
        sortProfiles()
        if let s = defaults.string(forKey: Self.activeIdKey) {
            activeId = UUID(uuidString: s)
        }
    }

    private func save(_ profile: RocketProfile) {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? encoder.encode(profile) else { return }
        try? data.write(to: fileURL(profile.id), options: .atomic)
    }

    private func persistActiveId() {
        if let id = activeId {
            defaults.set(id.uuidString, forKey: Self.activeIdKey)
        } else {
            defaults.removeObject(forKey: Self.activeIdKey)
        }
    }

    private func sortProfiles() {
        profiles.sort {
            $0.name.localizedCaseInsensitiveCompare($1.name) == .orderedAscending
        }
    }

    /// Drop a dangling active id (e.g. its file was deleted out from under
    /// us) so `activeProfile` never points at a profile that isn't loaded.
    private func reconcileActiveId() {
        if let id = activeId, !profiles.contains(where: { $0.id == id }) {
            setActive(profiles.first?.id)
        }
    }

    /// Make `name` unique among existing profiles by appending " 2", " 3", …
    private func dedupedName(_ name: String) -> String {
        let trimmed = name.trimmingCharacters(in: .whitespacesAndNewlines)
        let base = trimmed.isEmpty ? "Rocket" : trimmed
        let existing = Set(profiles.map { $0.name })
        guard existing.contains(base) else { return base }
        var n = 2
        while existing.contains("\(base) \(n)") { n += 1 }
        return "\(base) \(n)"
    }

    // MARK: - Legacy migration (#132)

    /// On first launch after the profiles feature ships, fold the old global
    /// `@AppStorage` settings into a single "Default" profile so existing
    /// users don't lose their tuning.  Runs once (guarded by `migratedKey`).
    /// Legacy keys are left in place as a one-release safety net; a later
    /// change removes them.  Fresh installs (no legacy keys) migrate nothing
    /// and start with an empty set.
    private func migrateLegacyIfNeeded() {
        guard !defaults.bool(forKey: Self.migratedKey) else { return }
        defer { defaults.set(true, forKey: Self.migratedKey) }

        // Only migrate if the user actually has legacy settings AND we don't
        // already have profiles (a fresh install has neither).
        guard profiles.isEmpty, hasLegacySettings() else { return }

        var p = RocketProfile.makeDefault(name: "Default")

        if defaults.object(forKey: "rocketSoundsEnabled") != nil {
            p.soundsEnabled = defaults.bool(forKey: "rocketSoundsEnabled")
        }
        if defaults.object(forKey: "servoControlEnabled") != nil {
            p.servoControlEnabled = defaults.bool(forKey: "servoControlEnabled")
        }
        if defaults.object(forKey: "gainScheduleEnabled") != nil {
            p.gainScheduleEnabled = defaults.bool(forKey: "gainScheduleEnabled")
        }
        if defaults.object(forKey: "useAngleControl") != nil {
            p.useAngleControl = defaults.bool(forKey: "useAngleControl")
        }
        if defaults.object(forKey: "guidanceEnabled") != nil {
            p.guidanceEnabled = defaults.bool(forKey: "guidanceEnabled")
        }
        if defaults.object(forKey: "rollDelayMs") != nil {
            p.rollDelayMs = UInt16(clamping: Int(defaults.double(forKey: "rollDelayMs")))
        }
        if defaults.object(forKey: "cameraType") != nil {
            p.cameraType = UInt8(clamping: defaults.integer(forKey: "cameraType"))
        }

        if let v = legacyInt16("servoBias1") { p.servoBias1 = v }
        if let v = legacyInt16("servoBias2") { p.servoBias2 = v }
        if let v = legacyInt16("servoBias3") { p.servoBias3 = v }
        if let v = legacyInt16("servoBias4") { p.servoBias4 = v }
        if let v = legacyInt16("servoHz")    { p.servoHz    = v }
        if let v = legacyInt16("servoMinUs") { p.servoMinUs = v }
        if let v = legacyInt16("servoMaxUs") { p.servoMaxUs = v }

        if let v = legacyFloat("pidKp")     { p.pidKp     = v }
        if let v = legacyFloat("pidKi")     { p.pidKi     = v }
        if let v = legacyFloat("pidKd")     { p.pidKd     = v }
        if let v = legacyFloat("pidMinCmd") { p.pidMinCmd = v }
        if let v = legacyFloat("pidMaxCmd") { p.pidMaxCmd = v }

        p.rollWaypoints = legacyRollWaypoints()

        profiles = [p]
        save(p)
        setActive(p.id)
    }

    private func hasLegacySettings() -> Bool {
        let probe = ["pidKp", "servoBias1", "rollProfileJSON", "cameraType",
                     "useAngleControl", "servoHz"]
        return probe.contains { defaults.object(forKey: $0) != nil }
    }

    private func legacyInt16(_ key: String) -> Int16? {
        guard defaults.object(forKey: key) != nil else { return nil }
        return Int16(clamping: Int(defaults.double(forKey: key).rounded()))
    }

    private func legacyFloat(_ key: String) -> Float? {
        guard defaults.object(forKey: key) != nil else { return nil }
        return Float(defaults.double(forKey: key))
    }

    /// Decode the legacy `rollProfileJSON` blob — an array of [time, angle,
    /// mode] string triples (older builds wrote [time, angle] pairs).
    private func legacyRollWaypoints() -> [RollWaypoint] {
        guard let json = defaults.string(forKey: "rollProfileJSON"),
              let data = json.data(using: .utf8),
              let decoded = try? JSONDecoder().decode([[String]].self, from: data)
        else { return [] }
        return decoded.map { entry in
            let t = entry.count > 0 ? (Float(entry[0]) ?? 0) : 0
            let a = entry.count > 1 ? (Float(entry[1]) ?? 0) : 0
            let m = entry.count > 2 ? (UInt8(entry[2]) ?? 0) : 0
            return RollWaypoint(timeSeconds: t, angleDeg: a,
                                mode: RollSegmentMode(rawValue: m) ?? .angle)
        }
    }
}
