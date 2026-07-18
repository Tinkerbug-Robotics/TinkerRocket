//
//  DeviceManagerView.swift
//  TinkerRocketApp
//
//  "My Devices" — one place to see and manage every provisioned rocket and
//  base station: rename them, fix rocket IDs, and keep the whole fleet on
//  the app's network ID.  Reachable from the first screen, with or without
//  anything connected: edits to offline devices queue in the registry and
//  are pushed automatically on that device's next connect.
//

import SwiftUI

struct DeviceManagerView: View {
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var store: KnownDeviceStore

    init(fleet: BLEFleet) {
        _fleet = ObservedObject(initialValue: fleet)
        _store = ObservedObject(initialValue: fleet.knownDevices)
    }

    // #150: network identity (app side; onboarding writes these).
    @AppStorage("networkName") private var networkName: String = ""
    @AppStorage("networkID") private var networkID: Int = 0

    @State private var editingNetworkName = false
    @State private var networkNameInput = ""

    private var appNid: UInt8 { UInt8(clamping: networkID) }

    /// Devices whose target network ID differs from the app's.
    private var mismatchedDevices: [KnownDevice] {
        guard networkID > 0 else { return [] }
        return store.devices.filter { $0.effectiveNetworkID != appNid }
    }

    var body: some View {
        List {
            networkSection
            devicesSection
        }
        .navigationTitle("My Devices")
        .navigationBarTitleDisplayMode(.inline)
    }

    // MARK: - Network

    private var networkSection: some View {
        Section(header: Text("Network"), footer: Text(networkFooter)) {
            if editingNetworkName {
                TextField("Network name", text: $networkNameInput)
                    .autocapitalization(.words)
                    .onSubmit(commitNetworkName)
                // Live wire-ID preview, same as onboarding — the ID is what
                // actually goes over the air.
                if !trimmedNetworkInput.isEmpty {
                    Text("Network ID: \(networkIdForName(trimmedNetworkInput))")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                HStack {
                    Button("Cancel") { editingNetworkName = false }
                        .foregroundColor(.secondary)
                    Spacer()
                    Button("Save") { commitNetworkName() }
                        .fontWeight(.semibold)
                        .disabled(trimmedNetworkInput.isEmpty)
                }
            } else {
                HStack {
                    Text(networkName.isEmpty ? "Not set" : networkName)
                    Spacer()
                    Text("ID: \(networkID)")
                        .foregroundColor(.secondary)
                        .font(.system(.body, design: .monospaced))
                }
                Button {
                    networkNameInput = networkName
                    editingNetworkName = true
                } label: {
                    Label("Change network name…", systemImage: "pencil")
                }
            }

            if !mismatchedDevices.isEmpty {
                Button {
                    syncAllToAppNetwork()
                } label: {
                    Label("Move all devices to ID \(networkID)",
                          systemImage: "arrow.triangle.2.circlepath")
                }
            }
        }
    }

    private var trimmedNetworkInput: String {
        networkNameInput.trimmingCharacters(in: .whitespacesAndNewlines)
    }

    private func commitNetworkName() {
        let trimmed = trimmedNetworkInput
        guard !trimmed.isEmpty else { return }
        networkName = trimmed
        networkID = Int(networkIdForName(trimmed))   // #150: never 0
        editingNetworkName = false
    }

    private func syncAllToAppNetwork() {
        guard networkID > 0 else { return }
        for rec in mismatchedDevices {
            store.setNetworkID(appNid, for: rec.unitID,
                               pusher: fleet.connectedDevice(unitID: rec.unitID))
        }
    }

    private var networkFooter: String {
        if !mismatchedDevices.isEmpty {
            let n = mismatchedDevices.count
            return "\(n) device\(n == 1 ? " is" : "s are") on a different network ID and can't hear the others. Moving them pushes connected devices right away and queues the rest for their next connect."
        }
        return "Devices only hear each other on the same network ID. Edits to offline devices are queued and apply automatically the next time each one connects."
    }

    // MARK: - Devices

    private var devicesSection: some View {
        Section(header: Text("Devices")) {
            if store.devices.isEmpty {
                Text("No devices yet. Scan and connect to a rocket or base station and it will appear here.")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            ForEach(store.devices) { rec in
                NavigationLink {
                    KnownDeviceDetailView(unitID: rec.unitID, fleet: fleet)
                } label: {
                    deviceRow(rec)
                }
                .swipeActions(edge: .trailing) {
                    Button(role: .destructive) { store.forget(rec.unitID) } label: {
                        Label("Forget", systemImage: "trash")
                    }
                }
            }
        }
    }

    private func deviceRow(_ rec: KnownDevice) -> some View {
        let connected = fleet.connectedDevice(unitID: rec.unitID) != nil
        let mismatch = networkID > 0 && rec.effectiveNetworkID != appNid
        return HStack(spacing: 12) {
            DeviceTypeIcon(type: rec.deviceType)

            VStack(alignment: .leading, spacing: 2) {
                Text(rec.effectiveName.isEmpty ? "Unnamed" : rec.effectiveName)
                    .fontWeight(.medium)
                    .foregroundColor(rec.effectiveName.isEmpty ? .secondary : .primary)
                Text(subtitle(rec))
                    .font(.caption)
                    .foregroundColor(.secondary)
            }

            Spacer()

            VStack(alignment: .trailing, spacing: 2) {
                HStack(spacing: 6) {
                    if connected {
                        Circle().fill(Color.green).frame(width: 8, height: 8)
                    }
                    if mismatch {
                        Image(systemName: "exclamationmark.triangle.fill")
                            .font(.caption)
                            .foregroundColor(.orange)
                    }
                    Text("ID \(rec.effectiveNetworkID)")
                        .font(.system(.caption, design: .monospaced))
                        .foregroundColor(mismatch ? .orange : .secondary)
                }
                if rec.hasPendingChanges {
                    Label("queued", systemImage: "clock")
                        .font(.caption2)
                        .foregroundColor(.blue)
                }
            }
        }
        .padding(.vertical, 2)
    }

    private func subtitle(_ rec: KnownDevice) -> String {
        var parts: [String] = []
        switch rec.deviceType {
        case .rocket:
            parts.append("Rocket")
            if rec.effectiveRocketID > 0 { parts.append("R\(rec.effectiveRocketID)") }
        case .baseStation:
            parts.append("Base Station")
        case .unknown:
            parts.append("Unknown type")
        }
        parts.append(rec.unitID)
        return parts.joined(separator: " · ")
    }
}

/// Rocket / base-station / unknown glyph, sized for a list row.
private struct DeviceTypeIcon: View {
    let type: BLEDeviceType

    var body: some View {
        Group {
            switch type {
            case .baseStation:
                Image(systemName: "antenna.radiowaves.left.and.right")
                    .font(.title3)
                    .foregroundColor(.orange)
            case .rocket:
                Image("RocketIcon")
                    .resizable()
                    .renderingMode(.template)
                    .aspectRatio(contentMode: .fit)
                    .frame(height: 24)
                    .foregroundColor(.blue)
            case .unknown:
                Image(systemName: "questionmark.circle")
                    .font(.title3)
                    .foregroundColor(.secondary)
            }
        }
        .frame(width: 32)
    }
}

// MARK: - Detail / editor

struct KnownDeviceDetailView: View {
    let unitID: String
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var store: KnownDeviceStore
    @Environment(\.dismiss) private var dismiss

    init(unitID: String, fleet: BLEFleet) {
        self.unitID = unitID
        _fleet = ObservedObject(initialValue: fleet)
        _store = ObservedObject(initialValue: fleet.knownDevices)
    }

    @AppStorage("networkName") private var networkName: String = ""
    @AppStorage("networkID") private var networkID: Int = 0

    @State private var showRename = false
    @State private var renameText = ""
    @State private var showForgetConfirm = false

    /// Live BLE handle when this device is currently connected — nil means
    /// edits queue for the next connect.
    private var live: BLEDevice? { fleet.connectedDevice(unitID: unitID) }

    var body: some View {
        Group {
            if let rec = store.device(for: unitID) {
                detail(rec)
            } else {
                // Forgotten while open — nothing left to show.
                Text("Device removed.")
                    .foregroundColor(.secondary)
            }
        }
        .navigationTitle(store.device(for: unitID)?.effectiveName ?? "Device")
        .navigationBarTitleDisplayMode(.inline)
    }

    private func detail(_ rec: KnownDevice) -> some View {
        let connected = live != nil
        let mismatch = networkID > 0 && rec.effectiveNetworkID != appNid

        return List {
            Section(footer: rec.pendingName != nil
                    ? Text("Rename to \u{201C}\(rec.pendingName!)\u{201D} is queued — it applies the next time this device connects.")
                    : Text("")) {
                HStack(spacing: 12) {
                    DeviceTypeIcon(type: rec.deviceType)
                    VStack(alignment: .leading, spacing: 2) {
                        Text(rec.effectiveName.isEmpty ? "Unnamed" : rec.effectiveName)
                            .font(.headline)
                            .foregroundColor(rec.effectiveName.isEmpty ? .secondary : .primary)
                        HStack(spacing: 6) {
                            Circle()
                                .fill(connected ? Color.green : Color(.systemGray4))
                                .frame(width: 8, height: 8)
                            Text(connected ? "Connected" : "Not connected")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    Spacer()
                }
                Button {
                    renameText = rec.effectiveName
                    showRename = true
                } label: {
                    Label("Rename…", systemImage: "pencil")
                }
            }

            if rec.deviceType != .baseStation {
                rocketIDSection(rec, connected: connected)
            }

            networkSection(rec, mismatch: mismatch, connected: connected)

            Section(header: Text("Info")) {
                HStack {
                    Text("Hardware ID")
                    Spacer()
                    Text(rec.unitID)
                        .font(.system(.body, design: .monospaced))
                        .foregroundColor(.secondary)
                }
                if let seen = rec.lastSeen {
                    HStack {
                        Text("Last seen")
                        Spacer()
                        if connected {
                            Text("Now").foregroundColor(.secondary)
                        } else {
                            Text(seen, style: .relative)
                                .foregroundColor(.secondary)
                            Text("ago").foregroundColor(.secondary)
                        }
                    }
                }
            }

            if rec.hasPendingChanges {
                Section(footer: Text("Queued changes are pushed automatically the next time this device connects.")) {
                    Button {
                        store.clearPending(for: unitID)
                    } label: {
                        Label("Cancel queued changes", systemImage: "clock.badge.xmark")
                    }
                }
            }

            Section(footer: Text("The device itself keeps its name and IDs — forgetting only removes it from this app, and it will be treated as new the next time it connects.")) {
                Button(role: .destructive) {
                    showForgetConfirm = true
                } label: {
                    Label("Forget This Device", systemImage: "trash")
                }
            }
        }
        .alert("Rename Device", isPresented: $showRename) {
            TextField("Name (max 20 chars)", text: $renameText)
            Button("Cancel", role: .cancel) {}
            Button("Save") {
                store.setName(renameText, for: unitID, pusher: live)
            }
        } message: {
            Text(connected
                 ? "The new name is sent to the device now."
                 : "The device is offline — the new name applies on its next connect.")
        }
        .confirmationDialog("Forget \u{201C}\(rec.effectiveName.isEmpty ? rec.unitID : rec.effectiveName)\u{201D}?",
                            isPresented: $showForgetConfirm, titleVisibility: .visible) {
            Button("Forget Device", role: .destructive) {
                store.forget(unitID)
                dismiss()
            }
        }
    }

    private var appNid: UInt8 { UInt8(clamping: networkID) }

    private func rocketIDSection(_ rec: KnownDevice, connected: Bool) -> some View {
        Section(header: Text("Rocket ID"),
                footer: Text(connected
                             ? "Unique ID within your network (1–254). Each rocket needs a different ID."
                             : "Unique ID within your network (1–254). Changes apply on the next connect.")) {
            Stepper("ID: \(displayRocketID(rec))\(rec.pendingRocketID != nil ? "  (queued)" : "")",
                    value: Binding(
                        get: { Int(displayRocketID(rec)) },
                        set: { store.setRocketID(UInt8(clamping: $0), for: unitID, pusher: live) }
                    ), in: 1...254)
        }
    }

    private func displayRocketID(_ rec: KnownDevice) -> UInt8 {
        rec.effectiveRocketID == 0 ? 1 : rec.effectiveRocketID
    }

    private func networkSection(_ rec: KnownDevice, mismatch: Bool, connected: Bool) -> some View {
        Section(header: Text("Network"), footer: Text(networkSectionFooter(rec, mismatch: mismatch, connected: connected))) {
            HStack {
                Text("App network")
                Spacer()
                Text(networkName.isEmpty ? "Not set" : networkName)
                    .foregroundColor(.secondary)
                Text("ID: \(networkID)")
                    .foregroundColor(.secondary)
                    .font(.system(.body, design: .monospaced))
            }
            HStack {
                Text("This device")
                Spacer()
                if mismatch {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.orange)
                }
                Text("ID: \(rec.effectiveNetworkID)\(rec.pendingNetworkID != nil ? "  (queued)" : "")")
                    .foregroundColor(mismatch ? .orange : .secondary)
                    .font(.system(.body, design: .monospaced))
            }
            if mismatch && networkID > 0 {
                Button("Set to \u{201C}\(networkName)\u{201D} (ID \(networkID))") {
                    store.setNetworkID(appNid, for: unitID, pusher: live)
                }
            }
        }
    }

    private func networkSectionFooter(_ rec: KnownDevice, mismatch: Bool, connected: Bool) -> String {
        if mismatch {
            return connected
                ? "This device is on a different network ID than the app expects — it can't hear your other devices. Tap to sync it now."
                : "This device is on a different network ID than the app expects. Syncing queues the change for its next connect."
        }
        return "Devices only hear each other on the same network ID."
    }
}
