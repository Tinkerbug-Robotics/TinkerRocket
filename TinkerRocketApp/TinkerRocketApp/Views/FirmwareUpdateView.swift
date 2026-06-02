//
//  FirmwareUpdateView.swift
//  TinkerRocketApp
//
//  OTA firmware update flow — pick a .bin from Files, push it over BLE,
//  wait for device reboot, confirm the new firmware version.
//
//  Phase 2 (#8): scoped to Base Station only via the call-site gate in
//  SettingsView. Phase 3 will drop the gate to include the Out Computer.
//

import SwiftUI
import UniformTypeIdentifiers

struct FirmwareUpdateView: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var fleet: BLEFleet

    /// OTASession lives on BLEFleet (not BLEDevice) because BLEFleet
    /// destroys+recreates BLEDevice on every BLE disconnect/reconnect —
    /// including the post-OTA reboot cycle. Same pattern as #140's
    /// lastValidRocketFixes. The fleet returns the same session instance
    /// for this peripheral across reconnects, so the .verified result
    /// survives the view being torn down + re-shown. See #15 second pass.
    var body: some View {
        FirmwareUpdateContent(device: device, session: fleet.otaSession(for: device))
    }
}

private struct FirmwareUpdateContent: View {
    @ObservedObject var device: BLEDevice
    @ObservedObject var session: OTASession
    @State private var showingFilePicker = false
    @State private var pickedFileURL: URL?
    @State private var pickedFileSize: Int = 0
    @State private var pickedFileName: String = ""
    @State private var targetIsFC = false   // #14: relay the OTA to the Flight Computer via the OC

    var body: some View {
        VStack(alignment: .leading, spacing: 16) {

            // ----- Current device state -----
            GroupBox("Device") {
                LabeledRow(label: "Name", value: device.displayName)
                LabeledRow(label: "Hardware ID", value: device.unitID.isEmpty ? "—" : device.unitID)
                LabeledRow(label: "Firmware",
                           value: device.firmwareVersion.isEmpty ? "(pre-#8 image)" : device.firmwareVersion,
                           mono: true)
                // FC firmware is relayed by the OC (#8 Phase 4). Show it when
                // targeting the FC so the user can watch it flip after an update
                // (or stay put on a rollback) — the OC's own version above never
                // changes on an FC-only OTA.
                if targetIsFC && !device.isBaseStation {
                    LabeledRow(label: "FC firmware",
                               value: device.fcFirmwareVersion.isEmpty ? "(awaiting relay…)" : device.fcFirmwareVersion,
                               mono: true)
                }
                LabeledRow(label: "Connection", value: device.isConnected ? "Connected" : "Disconnected")
            }

            // ----- Picked-file summary -----
            GroupBox("Firmware image") {
                if let url = pickedFileURL {
                    LabeledRow(label: "File", value: pickedFileName)
                    LabeledRow(label: "Size", value: byteCountString(pickedFileSize))
                    LabeledRow(label: "Path", value: url.lastPathComponent, mono: true)
                } else {
                    Text("No file selected")
                        .foregroundColor(.secondary)
                        .font(.subheadline)
                }
                Button(action: { showingFilePicker = true }) {
                    HStack {
                        Image(systemName: "doc.badge.plus")
                        Text(pickedFileURL == nil ? "Choose .bin…" : "Choose a different file…")
                    }
                }
                .buttonStyle(.bordered)
                .padding(.top, 4)
                .disabled(isInProgress)
            }

            // ----- Target picker (#14) -----
            // A rocket's BLE peer is the Out Computer, which can relay an OTA
            // over the OC↔FC link to the Flight Computer. (The Base Station has
            // no FC, so it only ever flashes itself.)
            if !device.isBaseStation {
                GroupBox("Target") {
                    Picker("Target", selection: $targetIsFC) {
                        Text("This device").tag(false)
                        Text("Flight Computer").tag(true)
                    }
                    .pickerStyle(.segmented)
                    .disabled(isInProgress)
                }
            }

            // ----- Action / status block -----
            statusBlock

            Spacer(minLength: 0)
        }
        .padding()
        .navigationTitle("Firmware update")
        .navigationBarTitleDisplayMode(.inline)
        .fileImporter(
            isPresented: $showingFilePicker,
            allowedContentTypes: [.data, UTType(filenameExtension: "bin") ?? .data],
            allowsMultipleSelection: false
        ) { result in
            handleFileImport(result)
        }
    }

    // MARK: - Action / status

    @ViewBuilder
    private var statusBlock: some View {
        switch session.state {
        case .idle:
            flashButton

        case .loading:
            ProgressView("Reading file + computing SHA…")
                .frame(maxWidth: .infinity)

        case .uploading(let sent, let total):
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("Uploading…").font(.subheadline)
                    Spacer()
                    Text("\(byteCountString(sent)) / \(byteCountString(total))")
                        .font(.caption.monospacedDigit())
                        .foregroundColor(.secondary)
                }
                ProgressView(value: Double(sent), total: Double(total))
                    .progressViewStyle(LinearProgressViewStyle())
                cancelButton
            }

        case .verifying:
            VStack(alignment: .leading, spacing: 8) {
                ProgressView("Verifying SHA on device…")
                cancelButton
            }

        case .rebooting:
            VStack(alignment: .leading, spacing: 8) {
                ProgressView("Device rebooting — waiting for reconnect (60 s)…")
            }

        case .verified(let newVersion):
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Image(systemName: "checkmark.seal.fill").foregroundColor(.green)
                    Text("Updated successfully").font(.headline)
                }
                LabeledRow(label: "Previous", value: session.preFlashFirmwareVersion, mono: true)
                LabeledRow(label: "Now running", value: newVersion, mono: true)
                flashButton
            }

        case .rollbackDetected(let version):
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Image(systemName: "arrow.uturn.backward.circle.fill").foregroundColor(.orange)
                    Text("Rollback detected").font(.headline)
                }
                Text("Device reconnected but is still running the previous firmware (\(version)). The new image likely failed to boot — bootloader auto-reverted to the prior partition.")
                    .font(.subheadline).foregroundColor(.secondary)
                flashButton
            }

        case .failed(let reason):
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Image(systemName: "xmark.octagon.fill").foregroundColor(.red)
                    Text("Failed").font(.headline)
                }
                Text(reason).font(.subheadline).foregroundColor(.secondary)
                flashButton
            }
        }
    }

    // The Flash action, shown in .idle and (greyed) in the terminal states.
    // After a result it's disabled — choosing a file (which re-arms the
    // session, see handleFileImport) is the explicit way to start the next
    // flash, so there's no separate "flash another firmware" reset button.
    private var flashButton: some View {
        Button(action: startFlash) {
            HStack {
                Image(systemName: "arrow.up.circle.fill")
                Text(targetIsFC ? "Flash Flight Computer" : "Flash \(device.displayName)")
            }
            .frame(maxWidth: .infinity)
        }
        .buttonStyle(.borderedProminent)
        .disabled(pickedFileURL == nil || !device.isConnected || isTerminalState)
    }

    private var isTerminalState: Bool {
        switch session.state {
        case .verified, .rollbackDetected, .failed: return true
        default: return false
        }
    }

    private var cancelButton: some View {
        Button(role: .destructive, action: { session.cancel() }) {
            Text("Cancel")
        }
        .buttonStyle(.bordered)
    }

    private var isInProgress: Bool {
        switch session.state {
        case .idle, .verified, .rollbackDetected, .failed: return false
        default: return true
        }
    }

    // MARK: - File picker

    private func handleFileImport(_ result: Result<[URL], Error>) {
        switch result {
        case .success(let urls):
            guard let url = urls.first else { return }
            pickedFileURL = url
            pickedFileName = url.lastPathComponent
            // Size: peek with security-scoped access
            let didOpen = url.startAccessingSecurityScopedResource()
            defer { if didOpen { url.stopAccessingSecurityScopedResource() } }
            if let attrs = try? FileManager.default.attributesOfItem(atPath: url.path),
               let size = attrs[.size] as? Int {
                pickedFileSize = size
            } else {
                pickedFileSize = 0
            }
            // Re-arm after a completed/failed run: clearing the terminal state
            // back to .idle re-enables the Flash button for this new file.
            if isTerminalState { session.reset() }
        case .failure(let error):
            print("File picker error: \(error)")
        }
    }

    private func startFlash() {
        guard let url = pickedFileURL else { return }
        session.start(fileURL: url, targetIsFC: targetIsFC)
    }

    private func byteCountString(_ bytes: Int) -> String {
        ByteCountFormatter.string(fromByteCount: Int64(bytes), countStyle: .binary)
    }
}

private struct LabeledRow: View {
    let label: String
    let value: String
    var mono: Bool = false

    var body: some View {
        HStack(alignment: .firstTextBaseline) {
            Text(label).foregroundColor(.secondary)
            Spacer()
            Text(value)
                .font(mono ? .body.monospaced() : .body)
                .multilineTextAlignment(.trailing)
                .lineLimit(2)
        }
    }
}
