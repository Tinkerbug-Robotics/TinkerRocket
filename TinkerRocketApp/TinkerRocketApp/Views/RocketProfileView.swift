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

            if let active = store.activeProfile {
                activeSection(active)
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
        Button {
            store.setActive(profile.id)
        } label: {
            HStack {
                VStack(alignment: .leading, spacing: 2) {
                    Text(profile.name)
                        .foregroundColor(.primary)
                    if !profile.notes.isEmpty {
                        Text(profile.notes)
                            .font(.caption).foregroundColor(.secondary).lineLimit(1)
                    }
                }
                Spacer()
                if profile.id == store.activeId {
                    Image(systemName: "checkmark.circle.fill").foregroundColor(.accentColor)
                }
            }
        }
        .swipeActions(edge: .trailing) {
            Button(role: .destructive) { store.delete(profile.id) } label: {
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
            Button(role: .destructive) { store.delete(profile.id) } label: {
                Label("Delete", systemImage: "trash")
            }
        }
    }

    // MARK: - Active profile section

    private func activeSection(_ active: RocketProfile) -> some View {
        Section(header: Text("Active: \(active.name)")) {
            summaryRow("Control mode", active.useAngleControl ? "Track Profile" : "Null Roll")
            summaryRow("Camera", cameraLabel(active.cameraType))
            summaryRow("Gain scheduling", active.gainScheduleEnabled ? "On" : "Off")
            summaryRow("Mag cal", active.magCal == nil ? "Not saved" : "Saved")
            summaryRow("Sensor cal", active.sensorCal == nil ? "Not saved" : "Saved")

            if let device, device.isConnected, !device.isBaseStation {
                Button {
                    showSettings = true
                } label: {
                    Label("Edit Settings", systemImage: "slider.horizontal.3")
                }
            } else {
                Label("Connect to a rocket to edit and apply settings",
                      systemImage: "antenna.radiowaves.left.and.right.slash")
                    .font(.caption).foregroundColor(.secondary)
            }
        }
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

            calAdvisoryView("magnetometer", advisory: syncer.magCalAdvisory) {
                syncer.importRocketCalIntoActiveProfile()
            }
            calAdvisoryView("sensor", advisory: syncer.sensorCalAdvisory) {
                syncer.importRocketSensorCalIntoActiveProfile()
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

    private func summaryRow(_ label: String, _ value: String) -> some View {
        HStack { Text(label); Spacer(); Text(value).foregroundColor(.secondary) }
    }

    private func advisory(_ text: String, systemImage: String, color: Color) -> some View {
        Label(text, systemImage: systemImage)
            .font(.caption).foregroundColor(color)
    }

    private func cameraLabel(_ t: UInt8) -> String {
        switch t { case 1: return "GoPro"; case 2: return "RunCam"; default: return "None" }
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
        case .noProfile:
            Label("No active rocket — pick one to apply settings", systemImage: "questionmark.circle")
                .font(.caption).foregroundColor(.secondary)
        case .syncing:
            Label("Applying settings to rocket…", systemImage: "arrow.triangle.2.circlepath")
                .font(.caption).foregroundColor(.secondary)
        case .synced:
            Label("Settings applied", systemImage: "checkmark.circle.fill")
                .font(.caption).foregroundColor(.green)
        case .failed(let reason):
            Label(reason, systemImage: "xmark.circle.fill")
                .font(.caption).foregroundColor(.red)
        }
    }
}
