//
//  PreflightRocketConfigView.swift
//  TinkerRocketApp
//
//  Tailor the pre-flight checklist for one rocket: include/exclude master
//  steps, add rocket-specific extras, and put the WHOLE list in the order
//  the rocket is actually prepped in (Edit → drag; master steps and
//  extras interleave freely).  Reached from the master checklist screen's
//  Rockets section.
//
//  The list shown here is the rocket's effective checklist — exactly what
//  the run screen walks — so the order edited is the order flown.
//

import SwiftUI

struct PreflightRocketConfigView: View {
    let profile: RocketProfile

    @EnvironmentObject var preflight: PreflightStore

    @State private var editingExtra: PreflightItem?

    var body: some View {
        List {
            Section {
                if effective.isEmpty {
                    Text("No steps yet — add one below, or add master steps on the previous screen.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                ForEach(effective) { item in
                    row(item)
                        // Master rows aren't deletable here (they're EXCLUDED
                        // via the toggle, deleted on the master screen) — this
                        // also keeps Edit mode's minus badge off them.
                        .deleteDisabled(masterIds.contains(item.id))
                }
                .onMove { offsets, dest in
                    preflight.moveEffectiveItems(fromOffsets: offsets, toOffset: dest,
                                                 for: profile.id)
                }
                // .onDelete rather than per-row swipeActions: swipe actions
                // are suppressed in Edit mode, which would leave extras
                // undeletable exactly while the list is being edited.
                .onDelete { offsets in
                    for idx in offsets where !masterIds.contains(effective[idx].id) {
                        preflight.deleteExtraItem(effective[idx].id, for: profile.id)
                    }
                }

                addMenu
            } header: {
                Text("Steps")
            } footer: {
                Text("This is \u{201C}\(profile.name)\u{201D}'s checklist in run order — tap Edit and drag to reorder. Master steps (toggle to exclude) are edited on the previous screen and update every rocket; steps added here belong to this rocket only.")
            }

            if !excludedMaster.isEmpty {
                Section {
                    ForEach(excludedMaster) { item in
                        Toggle(isOn: includeBinding(item.id)) {
                            PreflightItemRow(item: item)
                                .opacity(0.5)
                        }
                    }
                } header: {
                    Text("Excluded Master Steps")
                } footer: {
                    Text("Master steps switched off for this rocket. Switch one back on to return it to the list.")
                }
            }
        }
        .navigationTitle(profile.name)
        .toolbar { EditButton() }
        .sheet(item: $editingExtra) { item in
            PreflightItemEditSheet(item: item) { edited in
                if extras.contains(where: { $0.id == edited.id }) {
                    preflight.updateExtraItem(edited.id, for: profile.id) { $0 = edited }
                } else {
                    preflight.addExtraItem(edited, for: profile.id)
                }
            }
        }
    }

    // MARK: - Rows

    /// One effective-list row.  Master steps carry the include toggle
    /// (their content is edited on the master screen); extras are tapped
    /// to edit and deleted via the ForEach's onDelete.  Both kinds drag
    /// in Edit mode.
    @ViewBuilder
    private func row(_ item: PreflightItem) -> some View {
        if masterIds.contains(item.id) {
            Toggle(isOn: includeBinding(item.id)) {
                PreflightItemRow(item: item)
            }
        } else {
            PreflightItemRow(item: item)
                .contentShape(Rectangle())
                .onTapGesture { editingExtra = item }
        }
    }

    private func includeBinding(_ itemId: UUID) -> Binding<Bool> {
        Binding(
            get: { !(preflight.config(for: profile.id)?.disabledMasterIds.contains(itemId) ?? false) },
            set: { preflight.setMasterItem(itemId, enabled: $0, for: profile.id) }
        )
    }

    // MARK: - Derived lists

    private var effective: [PreflightItem] {
        preflight.effectiveItems(for: profile.id)
    }

    private var extras: [PreflightItem] {
        preflight.config(for: profile.id)?.extraItems ?? []
    }

    private var masterIds: Set<UUID> {
        Set(preflight.master.items.map(\.id))
    }

    private var excludedMaster: [PreflightItem] {
        let disabled = Set(preflight.config(for: profile.id)?.disabledMasterIds ?? [])
        return preflight.master.items.filter { disabled.contains($0.id) }
    }

    /// Auto kinds already anywhere in this rocket's effective list — the
    /// same condition twice verifies nothing new.
    private var usedAutoKinds: Set<PreflightItemKind> {
        Set(effective.map(\.kind).filter(\.isAuto))
    }

    private var addMenu: some View {
        Menu {
            Button {
                editingExtra = PreflightItem(title: "")
            } label: {
                Label("Custom Step…", systemImage: "square.and.pencil")
            }
            Divider()
            ForEach(PreflightItemKind.allCases.filter(\.isAuto), id: \.self) { kind in
                Button {
                    preflight.addExtraItem(.auto(kind), for: profile.id)
                } label: {
                    Label(kind.defaultTitle, systemImage: "bolt.badge.checkmark")
                }
                .disabled(usedAutoKinds.contains(kind))
            }
        } label: {
            Label("Add Step", systemImage: "plus")
        }
    }
}
