//
//  PreflightMasterView.swift
//  TinkerRocketApp
//
//  Master pre-flight checklist editor, reached from the front page.  The
//  master is the live template every rocket starts from: edits here show
//  up on every rocket that includes the step.  Per-rocket tailoring
//  (exclude master steps / add rocket-specific ones) is one tap away in
//  the Rockets section below the list.
//

import SwiftUI

struct PreflightMasterView: View {
    @EnvironmentObject var preflight: PreflightStore
    @EnvironmentObject var profileStore: RocketProfileStore

    /// Item being edited in the sheet; nil id sentinel isn't needed — a
    /// fresh item (not yet in the master) means "adding".
    @State private var editingItem: PreflightItem?

    var body: some View {
        List {
            Section {
                if preflight.master.items.isEmpty {
                    Text("No steps yet. Add the things you never want to forget at the pad — wadding, motor retention, igniter…")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                ForEach(preflight.master.items) { item in
                    PreflightItemRow(item: item)
                        .contentShape(Rectangle())
                        .onTapGesture { editingItem = item }
                }
                .onDelete { offsets in
                    for idx in offsets { preflight.deleteMasterItem(preflight.master.items[idx].id) }
                }
                .onMove { offsets, dest in
                    preflight.moveMasterItems(fromOffsets: offsets, toOffset: dest)
                }

                addMenu
            } header: {
                Text("Master Checklist")
            } footer: {
                Text("Every rocket starts from this list. Steps marked \u{201C}auto\u{201D} are verified by the app from live telemetry and can't be checked by hand.")
            }

            Section {
                if profileStore.profiles.isEmpty {
                    Text("Add a rocket first (rocket icon on the dashboard) to tailor its checklist.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                ForEach(profileStore.profiles) { profile in
                    NavigationLink {
                        PreflightRocketConfigView(profile: profile)
                    } label: {
                        HStack {
                            Image("RocketIcon")
                                .resizable().renderingMode(.template)
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 22, height: 22)
                                .foregroundColor(.secondary)
                            Text(profile.name)
                            Spacer()
                            Text(rocketSummary(profile))
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                }
            } header: {
                Text("Rockets")
            } footer: {
                Text("Tailor the checklist per rocket: leave out master steps that don't apply and add rocket-specific ones.")
            }
        }
        .navigationTitle("Preflight Checklist")
        .toolbar { EditButton() }
        .sheet(item: $editingItem) { item in
            PreflightItemEditSheet(item: item) { edited in
                if preflight.master.items.contains(where: { $0.id == edited.id }) {
                    preflight.updateMasterItem(edited.id) { $0 = edited }
                } else {
                    preflight.addMasterItem(edited)
                }
            }
        }
    }

    private var addMenu: some View {
        Menu {
            Button {
                editingItem = PreflightItem(title: "")
            } label: {
                Label("Custom Step…", systemImage: "square.and.pencil")
            }
            Divider()
            ForEach(PreflightItemKind.allCases.filter(\.isAuto), id: \.self) { kind in
                Button {
                    preflight.addMasterItem(.auto(kind))
                } label: {
                    Label(kind.defaultTitle, systemImage: "bolt.badge.checkmark")
                }
                .disabled(preflight.masterAutoKinds.contains(kind))
            }
        } label: {
            Label("Add Step", systemImage: "plus")
        }
    }

    private func rocketSummary(_ profile: RocketProfile) -> String {
        let count = preflight.effectiveItems(for: profile.id).count
        return count == 1 ? "1 step" : "\(count) steps"
    }
}

/// One checklist step as it renders in the editors: kind badge + title +
/// detail caption.
struct PreflightItemRow: View {
    let item: PreflightItem

    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: item.kind.isAuto ? "bolt.badge.checkmark" : "square.and.pencil")
                .foregroundColor(item.kind.isAuto ? .teal : .secondary)
                .frame(width: 24)
            VStack(alignment: .leading, spacing: 2) {
                Text(item.title.isEmpty ? "Untitled step" : item.title)
                if !item.detail.isEmpty {
                    Text(item.detail)
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .lineLimit(2)
                }
                if item.kind.isAuto {
                    Text("Auto — verified by the app")
                        .font(.caption2)
                        .foregroundColor(.teal)
                }
            }
        }
    }
}

/// Add/edit sheet for a single step.  The kind is fixed at creation (a
/// manual step can't become an auto one — the id would carry stale checked
/// state); title and detail stay editable for both.
struct PreflightItemEditSheet: View {
    @State var item: PreflightItem
    let onSave: (PreflightItem) -> Void
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationView {
            Form {
                Section {
                    TextField("Title", text: $item.title)
                    TextField("Detail (optional)", text: $item.detail, axis: .vertical)
                        .lineLimit(1...4)
                } footer: {
                    if item.kind.isAuto {
                        Text("This step is verified automatically — the label is editable, the check is not.")
                    }
                }
            }
            .navigationTitle(item.title.isEmpty ? "New Step" : "Edit Step")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") { dismiss() }
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") {
                        onSave(item)
                        dismiss()
                    }
                    .disabled(item.title.trimmingCharacters(in: .whitespaces).isEmpty)
                }
            }
        }
    }
}
