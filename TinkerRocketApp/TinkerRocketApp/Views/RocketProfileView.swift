//
//  RocketProfileView.swift
//  TinkerRocketApp
//
//  Pick / manage rocket profiles (issue #132).  Each airframe is a saved
//  profile; the active one is what the app pushes to a connected rocket.
//  Reached from the dashboard rocket icon.  Settings for the active rocket
//  are edited in SettingsView, presented from here.
//

import SwiftUI

struct RocketProfileView: View {
    /// The connected device, if any — drives the connect-time sync status,
    /// mag-cal advisory, and "last flown as…" suggestion.
    var device: BLEDevice?

    @EnvironmentObject var store: RocketProfileStore
    @EnvironmentObject var syncer: ActiveRocketSyncer
    @EnvironmentObject var preflight: PreflightStore

    @State private var showingAdd = false
    @State private var newName = ""
    @State private var renamingId: UUID?
    @State private var renameText = ""
    @State private var showSettings = false

    private var isConnectedRocket: Bool {
        guard let device else { return false }
        return device.isConnected && !device.isBaseStation
    }

    var body: some View {
        List {
            if isConnectedRocket { connectionSection }

            Section(header: Text("Rockets")) {
                ForEach(store.profiles) { profile in
                    profileRow(profile)
                }
                Button {
                    newName = ""
                    showingAdd = true
                } label: {
                    Label("Add Rocket", systemImage: "plus")
                }
            }
        }
        .navigationTitle("Rockets")
        .alert("New Rocket", isPresented: $showingAdd) {
            TextField("Name", text: $newName)
            Button("Cancel", role: .cancel) {}
            Button("Add") {
                let p = store.add(name: newName)
                store.setActive(p.id)
            }
        } message: {
            Text("Name this airframe (e.g. \u{201C}Big Bertha\u{201D}).")
        }
        .alert("Rename Rocket", isPresented: Binding(
            get: { renamingId != nil },
            set: { if !$0 { renamingId = nil } })) {
            TextField("Name", text: $renameText)
            Button("Cancel", role: .cancel) { renamingId = nil }
            Button("Save") {
                if let id = renamingId { store.rename(id, to: renameText) }
                renamingId = nil
            }
        }
        .sheet(isPresented: $showSettings) {
            if let device { SettingsView(device: device) }
        }
    }

    // MARK: - Rows

    private func profileRow(_ profile: RocketProfile) -> some View {
        let selected = profile.id == store.activeId
        return HStack(spacing: 12) {
            // Tapping the icon or the name makes this the active rocket and
            // opens its settings.
            Button {
                store.setActive(profile.id)
                showSettings = true
            } label: {
                HStack(spacing: 12) {
                    Image("RocketIcon")
                        .resizable().renderingMode(.template)
                        .aspectRatio(contentMode: .fit)
                        .frame(width: 26, height: 26)
                        .foregroundColor(selected ? .blue : .secondary)
                    VStack(alignment: .leading, spacing: 2) {
                        Text(profile.name)
                            .foregroundColor(selected ? .blue : .primary)
                        if !profile.notes.isEmpty {
                            Text(profile.notes)
                                .font(.caption).foregroundColor(.secondary).lineLimit(1)
                        }
                    }
                    Spacer()
                }
                .contentShape(Rectangle())
            }
            .buttonStyle(.borderless)

            if selected {
                Image(systemName: "checkmark.circle.fill").foregroundColor(.blue)
            }
        }
        .swipeActions(edge: .trailing) {
            Button(role: .destructive) { deleteProfile(profile.id) } label: {
                Label("Delete", systemImage: "trash")
            }
        }
        .contextMenu {
            Button { renamingId = profile.id; renameText = profile.name } label: {
                Label("Rename", systemImage: "pencil")
            }
            Button { store.duplicate(profile.id) } label: {
                Label("Duplicate", systemImage: "plus.square.on.square")
            }
            Button(role: .destructive) { deleteProfile(profile.id) } label: {
                Label("Delete", systemImage: "trash")
            }
        }
    }

    /// Delete a profile and its pre-flight checklist config together — the
    /// checklist diff is keyed by profile id and dies with it.
    private func deleteProfile(_ id: UUID) {
        store.delete(id)
        preflight.deleteConfig(for: id)
    }

    // MARK: - Connection-aware section

    @ViewBuilder
    private var connectionSection: some View {
        Section {
            SyncStatusRow(state: syncer.syncState)

            if let suggestedId = syncer.suggestedProfileId,
               let suggested = store.profiles.first(where: { $0.id == suggestedId }) {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Last flown as \u{201C}\(suggested.name)\u{201D}")
                            .font(.subheadline)
                        Text("Switch to apply that profile?")
                            .font(.caption).foregroundColor(.secondary)
                    }
                    Spacer()
                    Button("Switch") { store.setActive(suggested.id) }
                        .buttonStyle(.borderedProminent)
                }
            }

            if let created = syncer.createdProfileName {
                advisory("New rocket — created \u{201C}\(created)\u{201D} from its own settings.",
                         systemImage: "plus.circle", color: .blue)
            }

            calAdvisoryView("magnetometer", advisory: syncer.magCalAdvisory) {
                syncer.importRocketCalIntoActiveProfile()
            }
            calAdvisoryView("sensor", advisory: syncer.sensorCalAdvisory) {
                syncer.importRocketSensorCalIntoActiveProfile()
            }

            // #915: the rocket keeps its own settings unless the user asks
            // otherwise, so say plainly which ones the app cannot check and
            // put the deliberate override next to that admission.
            if isConnectedRocket {
                VStack(alignment: .leading, spacing: 6) {
                    if !syncer.unreportedGroups.isEmpty {
                        advisory("Can't verify: \(syncer.unreportedGroups.joined(separator: ", ")). This rocket doesn't report them, so they're shown from the profile.",
                                 systemImage: "eye.slash", color: .secondary)
                    }
                    Button("Send All Settings to Rocket") {
                        syncer.pushProfileToRocket()
                    }
                    .buttonStyle(.bordered)
                    .disabled(store.activeProfile == nil)
                }
            }
        }
    }

    @ViewBuilder
    private func calAdvisoryView(_ kind: String,
                                 advisory adv: ActiveRocketSyncer.CalAdvisory,
                                 onImport: @escaping () -> Void) -> some View {
        switch adv {
        case .none:
            EmptyView()
        case .missing:
            advisory("No \(kind) calibration saved. Run calibration in Settings.",
                     systemImage: "exclamationmark.triangle", color: .orange)
        case .boardMismatch(let savedOn, let current):
            advisory("This profile's \(kind) cal was done on board \(short(savedOn)). The connected board is \(short(current)) — re-run calibration.",
                     systemImage: "exclamationmark.triangle.fill", color: .orange)
        case .rocketHasUnsavedCal:
            VStack(alignment: .leading, spacing: 6) {
                advisory("This rocket has a \(kind) calibration the profile doesn't. Save it to the profile?",
                         systemImage: "square.and.arrow.down", color: .blue)
                Button("Save \(kind.capitalized) Cal to Profile", action: onImport)
                    .buttonStyle(.bordered)
            }
        }
    }

    // MARK: - Bits

    private func advisory(_ text: String, systemImage: String, color: Color) -> some View {
        Label(text, systemImage: systemImage)
            .font(.caption).foregroundColor(color)
    }

    /// Shorten a long hardware id for display.
    private func short(_ id: String) -> String {
        id.count > 8 ? "…" + id.suffix(6) : id
    }
}

/// One-line sync state, reused by the dashboard header.
struct SyncStatusRow: View {
    let state: ActiveRocketSyncer.SyncState

    var body: some View {
        switch state {
        case .idle:
            EmptyView()
        case .awaitingSync:
            // #375: a connected-but-not-reconciled rocket used to render as
            // silent .idle/EmptyView — invisible, so a rocket whose settings
            // the app had never seen could reach the pad unnoticed.
            Label("Reading this rocket's settings…", systemImage: "arrow.down.circle")
                .font(.caption).foregroundColor(.orange)
        case .noProfile:
            Label("No active rocket — pick one to hold this rocket's settings",
                  systemImage: "questionmark.circle")
                .font(.caption).foregroundColor(.secondary)
        case .syncing:
            Label("Sending settings to rocket…", systemImage: "arrow.triangle.2.circlepath")
                .font(.caption).foregroundColor(.secondary)
        case .synced:
            Label("Profile matches this rocket", systemImage: "checkmark.circle.fill")
                .font(.caption).foregroundColor(.green)
        case .adopted(let groups):
            // #915: informational, not a fault — the rocket kept flying what
            // it had and the profile was brought into line with it.  Stays a
            // quiet line; the rocket-state banner is not recoloured for this.
            Label("Updated from this rocket: \(groups.joined(separator: ", "))",
                  systemImage: "arrow.down.circle.fill")
                .font(.caption).foregroundColor(.blue)
        case .failed(let reason):
            Label(reason, systemImage: "xmark.circle.fill")
                .font(.caption).foregroundColor(.red)
        }
    }
}
