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
    @Environment(\.dismiss) var dismiss

    @AppStorage("networkName") private var networkName: String = ""
    @AppStorage("networkID") private var networkID: Int = 0

    @State private var nameInput: String = ""
    @State private var rocketIDInput: Int = 1
    @State private var initialized = false

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
                            if newValue.count > 20 {
                                nameInput = String(newValue.prefix(20))
                            }
                        }
                }

                if !device.isBaseStation {
                    Section(header: Text("Rocket ID"),
                            footer: Text("Unique ID within your network (1-254). Each rocket needs a different ID.")) {
                        Stepper("ID: \(rocketIDInput)", value: $rocketIDInput, in: 1...254)
                    }
                }

                Section(header: Text("Network"),
                        footer: Text("All your devices will be set to this network. Change in Settings.")) {
                    HStack {
                        Text(networkName.isEmpty ? "Not set" : networkName)
                        Spacer()
                        Text("ID: \(networkID)")
                            .foregroundColor(.secondary)
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

        // Send name to device
        device.sendSetUnitName(trimmed)

        // Send network ID
        device.sendSetNetworkID(UInt8(networkID))

        // Send rocket ID (rockets only)
        if !device.isBaseStation {
            device.sendSetRocketID(UInt8(rocketIDInput))
        }

        // Mark as known
        markDeviceKnown(device.unitID)
        dismiss()
    }

    private func skipAndDismiss() {
        markDeviceKnown(device.unitID)
        dismiss()
    }

    // MARK: - Known devices persistence

    static func isDeviceKnown(_ unitID: String) -> Bool {
        guard !unitID.isEmpty else { return true }  // Don't prompt for empty IDs
        let known = UserDefaults.standard.stringArray(forKey: "knownDeviceIDs") ?? []
        return known.contains(unitID)
    }

    private func markDeviceKnown(_ unitID: String) {
        guard !unitID.isEmpty else { return }
        var known = UserDefaults.standard.stringArray(forKey: "knownDeviceIDs") ?? []
        if !known.contains(unitID) {
            known.append(unitID)
            UserDefaults.standard.set(known, forKey: "knownDeviceIDs")
        }
    }
}
