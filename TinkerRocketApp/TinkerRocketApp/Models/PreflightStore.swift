//
//  PreflightStore.swift
//  TinkerRocketApp
//
//  Owns the master pre-flight checklist and the per-rocket configs.
//
//  Persistence mirrors RocketProfileStore: JSON files under Application
//  Support (PreflightChecklist/master.json + one <profile-uuid>.json per
//  configured rocket), so a corrupt file loses one rocket's diff, never
//  the master or the set.  The directory is injectable for unit tests.
//
//  A config file whose profile was since deleted is inert (nothing lists
//  it) — deleteConfig(_:) is called from the profile-delete UI, and any
//  orphan that slips through is a few hundred bytes of harmless JSON.
//

import Foundation
import Combine

final class PreflightStore: ObservableObject {
    // Same swiftlang/swift#87316 workaround as RocketProfileStore: no
    // deinit-time logic, so skip the MainActor executor hop that crashes
    // synchronous XCTest teardown.
    nonisolated deinit {}

    @Published private(set) var master = PreflightMaster()
    @Published private(set) var configs: [UUID: PreflightRocketConfig] = [:]

    private let dir: URL

    init(directory: URL? = nil) {
        if let directory {
            self.dir = directory
        } else {
            let base = FileManager.default.urls(for: .applicationSupportDirectory,
                                                in: .userDomainMask)[0]
            self.dir = base.appendingPathComponent("PreflightChecklist", isDirectory: true)
        }
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        load()
    }

    // MARK: - Master CRUD

    /// Append a step to the master list.
    @discardableResult
    func addMasterItem(_ item: PreflightItem) -> PreflightItem {
        mutateMaster { $0.items.append(item) }
        return item
    }

    func updateMasterItem(_ id: UUID, _ mutate: (inout PreflightItem) -> Void) {
        mutateMaster { m in
            guard let idx = m.items.firstIndex(where: { $0.id == id }) else { return }
            mutate(&m.items[idx])
        }
    }

    /// Remove a master step everywhere: from the master, from every
    /// rocket's exclusion list, and from every rocket's checked state.
    func deleteMasterItem(_ id: UUID) {
        mutateMaster { $0.items.removeAll { $0.id == id } }
        for profileId in configs.keys {
            mutateConfig(profileId) { c in
                c.disabledMasterIds.removeAll { $0 == id }
                c.checked.removeValue(forKey: id.uuidString)
            }
        }
    }

    func moveMasterItems(fromOffsets: IndexSet, toOffset: Int) {
        mutateMaster { Self.move(&$0.items, fromOffsets: fromOffsets, toOffset: toOffset) }
    }

    /// Auto kinds already present in the master — the add menu greys these
    /// out (two "camera recording" steps verify the same bit twice).
    var masterAutoKinds: Set<PreflightItemKind> {
        Set(master.items.map(\.kind).filter(\.isAuto))
    }

    // MARK: - Per-rocket config

    /// The stored config for a rocket, or nil if it has never been
    /// customized (= full master list, nothing checked).
    func config(for profileId: UUID) -> PreflightRocketConfig? {
        configs[profileId]
    }

    /// This rocket's effective checklist (master minus exclusions + extras).
    func effectiveItems(for profileId: UUID) -> [PreflightItem] {
        PreflightChecklist.effectiveItems(master: master, config: configs[profileId])
    }

    func setMasterItem(_ itemId: UUID, enabled: Bool, for profileId: UUID) {
        mutateConfig(profileId) { c in
            c.disabledMasterIds.removeAll { $0 == itemId }
            if !enabled {
                c.disabledMasterIds.append(itemId)
                c.checked.removeValue(forKey: itemId.uuidString)
            }
        }
    }

    @discardableResult
    func addExtraItem(_ item: PreflightItem, for profileId: UUID) -> PreflightItem {
        mutateConfig(profileId) { $0.extraItems.append(item) }
        return item
    }

    func updateExtraItem(_ itemId: UUID, for profileId: UUID,
                         _ mutate: (inout PreflightItem) -> Void) {
        mutateConfig(profileId) { c in
            guard let idx = c.extraItems.firstIndex(where: { $0.id == itemId }) else { return }
            mutate(&c.extraItems[idx])
        }
    }

    func deleteExtraItem(_ itemId: UUID, for profileId: UUID) {
        mutateConfig(profileId) { c in
            c.extraItems.removeAll { $0.id == itemId }
            c.checked.removeValue(forKey: itemId.uuidString)
        }
    }

    func moveExtraItems(fromOffsets: IndexSet, toOffset: Int, for profileId: UUID) {
        mutateConfig(profileId) { Self.move(&$0.extraItems, fromOffsets: fromOffsets, toOffset: toOffset) }
    }

    /// SwiftUI's `Array.move(fromOffsets:toOffset:)` semantics without the
    /// SwiftUI import (this is a model file): remove the offsets, then
    /// insert the block at the destination adjusted for the removals above it.
    private static func move(_ items: inout [PreflightItem],
                             fromOffsets: IndexSet, toOffset: Int) {
        let moving = fromOffsets.sorted(by: >).map { items.remove(at: $0) }.reversed()
        let dest = toOffset - fromOffsets.count(where: { $0 < toOffset })
        items.insert(contentsOf: moving, at: dest)
    }

    /// Drop a rocket's whole config (called when its profile is deleted).
    func deleteConfig(for profileId: UUID) {
        guard configs[profileId] != nil else { return }
        configs.removeValue(forKey: profileId)
        try? FileManager.default.removeItem(at: configURL(profileId))
    }

    // MARK: - Run state (manual checks)

    func setChecked(_ itemId: UUID, checked: Bool, for profileId: UUID) {
        mutateConfig(profileId) { c in
            if checked {
                c.checked[itemId.uuidString] = Date()
            } else {
                c.checked.removeValue(forKey: itemId.uuidString)
            }
        }
    }

    func isChecked(_ itemId: UUID, for profileId: UUID) -> Bool {
        configs[profileId]?.isChecked(itemId) ?? false
    }

    /// Clear every manual check for the next flight.
    func resetRun(for profileId: UUID) {
        mutateConfig(profileId) { $0.checked = [:] }
    }

    // MARK: - Persistence

    private var masterURL: URL { dir.appendingPathComponent("master.json") }

    private func configURL(_ profileId: UUID) -> URL {
        dir.appendingPathComponent("\(profileId.uuidString).json")
    }

    private func mutateMaster(_ mutate: (inout PreflightMaster) -> Void) {
        mutate(&master)
        master.updatedAt = Date()
        write(master, to: masterURL)
    }

    /// All config edits funnel through here — get-or-create, bump
    /// `updatedAt`, persist — so none of the three can be forgotten.
    private func mutateConfig(_ profileId: UUID,
                              _ mutate: (inout PreflightRocketConfig) -> Void) {
        var config = configs[profileId] ?? PreflightRocketConfig(profileId: profileId)
        mutate(&config)
        config.updatedAt = Date()
        configs[profileId] = config
        write(config, to: configURL(profileId))
    }

    private func load() {
        let decoder = JSONDecoder()
        if let data = try? Data(contentsOf: masterURL),
           let m = try? decoder.decode(PreflightMaster.self, from: data) {
            master = m
        }

        let urls = (try? FileManager.default.contentsOfDirectory(
            at: dir, includingPropertiesForKeys: nil)) ?? []
        var loaded: [UUID: PreflightRocketConfig] = [:]
        for url in urls where url.pathExtension == "json" && url != masterURL {
            guard let data = try? Data(contentsOf: url),
                  let c = try? decoder.decode(PreflightRocketConfig.self, from: data)
            else { continue }   // skip corrupt/foreign files — don't fail the set
            loaded[c.profileId] = c
        }
        configs = loaded
    }

    private func write<T: Encodable>(_ value: T, to url: URL) {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? encoder.encode(value) else { return }
        try? data.write(to: url, options: .atomic)
    }
}
