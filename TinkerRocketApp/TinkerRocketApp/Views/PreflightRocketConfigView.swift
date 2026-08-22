//
//  PreflightRocketConfigView.swift
//  TinkerRocketApp
//
//  Tailor the pre-flight checklist for one rocket: include/exclude master
//  steps and manage rocket-specific extras.  Reached from the master
//  checklist screen's Rockets section.
//

import SwiftUI

struct PreflightRocketConfigView: View {
    let profile: RocketProfile

    @EnvironmentObject var preflight: PreflightStore

    @State private var editingExtra: PreflightItem?

    var body: some View {
        List {
            Section {
                if preflight.master.items.isEmpty {
                    Text("The master checklist is empty — add steps there first.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                ForEach(preflight.master.items) { item in
                    Toggle(isOn: Binding(
                        get: { !(preflight.config(for: profile.id)?.disabledMasterIds.contains(item.id) ?? false) },
                        set: { preflight.setMasterItem(item.id, enabled: $0, for: profile.id) }
                    )) {
                        PreflightItemRow(item: item)
                    }
                }
            } header: {
                Text("Master Steps")
            } footer: {
                Text("Switch off master steps that don't apply to \u{201C}\(profile.name)\u{201D}. Master steps are edited on the previous screen and update every rocket.")
            }

            Section {
                ForEach(extras) { item in
                    PreflightItemRow(item: item)
                        .contentShape(Rectangle())
                        .onTapGesture { editingExtra = item }
                }
                .onDelete { offsets in
                    for idx in offsets { preflight.deleteExtraItem(extras[idx].id, for: profile.id) }
                }
                .onMove { offsets, dest in
                    preflight.moveExtraItems(fromOffsets: offsets, toOffset: dest, for: profile.id)
                }

                addMenu
            } header: {
                Text("\(profile.name) Steps")
            } footer: {
                Text("Extra steps only this rocket needs — they run after the master steps.")
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

    private var extras: [PreflightItem] {
        preflight.config(for: profile.id)?.extraItems ?? []
    }

    /// Auto kinds already anywhere in this rocket's effective list — the
    /// same condition twice verifies nothing new.
    private var usedAutoKinds: Set<PreflightItemKind> {
        Set(preflight.effectiveItems(for: profile.id).map(\.kind).filter(\.isAuto))
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
