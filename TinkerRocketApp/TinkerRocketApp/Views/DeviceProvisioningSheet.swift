//
//  DeviceProvisioningSheet.swift
//  TinkerRocketApp
//
//  Shown on first connect to a new device. Lets the user
//  name the device, set its rocket ID, and push the network ID.
//

import SwiftUI

struct DeviceProvisioningSheet: View {
    @ObservedObject var device: BLEDevice
    @ObservedObject var store: KnownDeviceStore
    @Environment(\.dismiss) var dismiss

    @State private var nameInput: String = ""
    @State private var rocketIDInput: Int = 1
    @State private var initialized = false

    // #150: the app's network (from onboarding).  Pushed to the device on
    // provision; the device persists it across reboots now, and the
    // Settings Network section shows the device's own readback so a
    // mismatch can never hide again.
    @AppStorage("networkName") private var networkName: String = ""
    @AppStorage("networkID") private var networkID: Int = 0

    var body: some View {
        NavigationView {
            Form {
                Section {
                    if device.isBaseStation {
                        Label("New Base Station", systemImage: "antenna.radiowaves.left.and.right")
                            .font(.headline)
                    } else {
                        Label("New Rocket", systemImage: "airplane")
                            .font(.headline)
                    }

                    Text("Hardware ID: \(device.unitID)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                Section(header: Text("Device Name")) {
                    TextField("Name (max 20 chars)", text: $nameInput)
                        .autocapitalization(.words)
                        .onChange(of: nameInput) { newValue in
                            // Firmware limit is 20 UTF-8 BYTES (multibyte
                            // names hit it before 20 chars).
                            let clamped = newValue.utf8Clamped(maxBytes: 20)
                            if clamped != newValue { nameInput = clamped }
                        }
                }

                if !device.isBaseStation {
                    Section(header: Text("Rocket ID"),
                            footer: Text("Unique ID within your network (1-254). Each rocket needs a different ID.")) {
                        Stepper("ID: \(rocketIDInput)", value: $rocketIDInput, in: 1...254)
                    }
                }

                // #150: Network section restored — the firmware persists
                // nid across reboots now (with a one-time migration that
                // guards against the #133-era silent drift).
                if networkID > 0 {
                    Section(header: Text("Network"),
                            footer: Text("All your devices are set to this network. Devices only hear each other on the same network ID.")) {
                        HStack {
                            Text(networkName.isEmpty ? "Not set" : networkName)
                            Spacer()
                            Text("ID: \(networkID)")
                                .foregroundColor(.secondary)
                        }
                    }
                }

                Section {
                    Button {
                        saveAndDismiss()
                    } label: {
                        HStack {
                            Spacer()
                            Text("Save & Connect")
                                .fontWeight(.semibold)
                            Spacer()
                        }
                    }
                    .disabled(nameInput.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)

                    Button {
                        skipAndDismiss()
                    } label: {
                        HStack {
                            Spacer()
                            Text("Skip")
                                .foregroundColor(.secondary)
                            Spacer()
                        }
                    }
                }
            }
            .navigationTitle("Set Up Device")
            .navigationBarTitleDisplayMode(.inline)
            .onAppear {
                if !initialized {
                    // Pre-fill from device's current values
                    nameInput = device.unitName.isEmpty ? device.connectedDeviceName : device.unitName
                    rocketIDInput = Int(device.rocketID)
                    if rocketIDInput == 0 { rocketIDInput = 1 }
                    initialized = true
                }
            }
        }
    }

    private func saveAndDismiss() {
        let trimmed = nameInput.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }

        // All edits go through the known-device registry, which pushes to the
        // connected device and keeps the "My Devices" record in sync.

        store.setName(trimmed, for: device.unitID, pusher: device)

        // #150: push the network ID (cmd 41) — the firmware persists it in
        // identity NVS across reboots now, so the push is durable.  The
        // Settings Network section verifies via the device's readback.
        if networkID > 0 {
            store.setNetworkID(UInt8(clamping: networkID), for: device.unitID, pusher: device)
        }

        if !device.isBaseStation {
            store.setRocketID(UInt8(rocketIDInput), for: device.unitID, pusher: device)
        }

        store.markProvisioned(device.unitID)
        dismiss()
    }

    private func skipAndDismiss() {
        store.markProvisioned(device.unitID)
        dismiss()
    }
}
