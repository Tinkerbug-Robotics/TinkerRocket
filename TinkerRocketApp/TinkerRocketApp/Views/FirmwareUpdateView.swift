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
import CryptoKit

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
    @State private var pickedFileSha: String = ""   // SHA-256 of the picked file, for build-machine comparison
    @State private var targetIsFC = false   // #14: relay the OTA to the Flight Computer via the OC
    // #773: what the picked image actually IS, checked before a byte goes over
    // BLE. Recomputed when the target changes, because the expected program
    // depends on which unit is being flashed.
    @State private var pickedFileData: Data?
    @State private var imageVerdict: EspImageVerdict?

    // #773 step 4c: the other way to get an image — a published release,
    // instead of a file the operator had to build or be sent. It ends by
    // adopting the download as the picked file, so a downloaded image and a
    // hand-picked one go through exactly the same verdict and flash path.
    @StateObject private var catalog = FirmwareCatalogSession()

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
                    LabeledRow(label: "Size", value: "\(pickedFileSize) B · \(byteCountString(pickedFileSize))")
                    LabeledRow(label: "Path", value: url.lastPathComponent, mono: true)
                    if !pickedFileSha.isEmpty {
                        LabeledRow(label: "SHA-256",
                                   value: String(pickedFileSha.prefix(16)),
                                   mono: true)
                    }
                } else {
                    Text("No file selected")
                        .foregroundColor(.secondary)
                        .font(.subheadline)
                }
                if let verdict = imageVerdict {
                    imageVerdictView(verdict)
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

                Divider().padding(.vertical, 4)
                catalogView
            }

            // ----- Target picker (#14) -----
            // A rocket's BLE peer is the Out Computer, which can relay an OTA
            // over the OC↔FC link to the Flight Computer. (The Base Station has
            // no FC, so it only ever flashes itself.)
            if !device.isBaseStation {
                GroupBox("Target") {
                    Picker("Target", selection: $targetIsFC) {
                        // Named, not "This device": the picker only appears for a
                        // rocket, whose BLE peer IS the Out Computer, and the
                        // choice that matters is OC vs FC — "this device" made the
                        // default read as "the whole rocket".
                        Text("Out Computer").tag(false)
                        Text("Flight Computer").tag(true)
                    }
                    .pickerStyle(.segmented)
                    .disabled(isInProgress)
                    .onChange(of: targetIsFC) { _ in revalidateImage() }
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
                // #1337: an FC relay never drops this link — the peer is the
                // out computer, which stays up while the FC reboots behind it.
                // Promising a reconnect there misdescribes the wait.
                ProgressView(
                    session.targetIsFC
                        ? "Flight computer rebooting — waiting for it to report in…"
                        : "Device rebooting — waiting for reconnect (60 s)…"
                )
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
                Image(systemName: isFailedState ? "arrow.clockwise" : "arrow.up.circle.fill")
                Text(isFailedState
                     ? "Try again"
                     : (targetIsFC ? "Flash Flight Computer" : "Flash \(device.displayName)"))
            }
            .frame(maxWidth: .infinity)
        }
        .buttonStyle(.borderedProminent)
        // A FAILED run stays retryable. Only a run that actually landed
        // (.verified / .rollbackDetected) disables the button, so a finished
        // flash can't be repeated by a stray tap. Previously any terminal state
        // disabled it and re-picking the file was the sole way back to .idle —
        // after a failure that reads as "the app is stuck", and it makes the
        // obvious response to a transient failure (press it again) impossible.
        // #773: a refused image is not flashable. The far end validates only
        // size and SHA-256, and the out computer and base station are both
        // ESP32-S3 with identical app slots — so if this does not stop it,
        // nothing does until the wrong image fails to boot.
        .disabled(pickedFileURL == nil || !device.isConnected || isCompletedState
                  || (imageVerdict?.isRefusal ?? false))
    }

    // #773: what the image says it is, and whether it belongs on this unit.
    @ViewBuilder
    private func imageVerdictView(_ verdict: EspImageVerdict) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            if let img = verdict.image {
                LabeledRow(label: "Program", value: img.projectName, mono: true)
                LabeledRow(label: "Version", value: img.version, mono: true)
                LabeledRow(label: "Built for", value: img.chipName)
                LabeledRow(label: "Built", value: "\(img.buildDate) \(img.buildTime)")
            }
            switch verdict {
            case .ok:
                Label("Matches this unit", systemImage: "checkmark.seal")
                    .font(.footnote).foregroundColor(.green)
            case .warn(_, let why):
                Label(why, systemImage: "exclamationmark.triangle")
                    .font(.footnote).foregroundColor(.orange)
            case .refuse(_, let why):
                Label(why, systemImage: "xmark.octagon")
                    .font(.footnote).foregroundColor(.red)
            }
        }
        .padding(.top, 4)
    }

    /// Which program the connected unit should be running, and the chip it is
    /// on. The flight computer's chip differs by board (ESP32-P4 on V9, S3 on
    /// the mini), so it is left unchecked rather than warning on every mini.
    private var expectedProject: String {
        if device.isBaseStation { return EspImage.projectBS }
        return targetIsFC ? EspImage.projectFC : EspImage.projectOC
    }
    private var expectedChipId: Int? {
        if device.isBaseStation { return 0x0009 }
        return targetIsFC ? nil : 0x0009
    }

    private func revalidateImage() {
        guard let data = pickedFileData else { imageVerdict = nil; return }
        // #773 step 2: the provisioned revision is the board's own answer and
        // beats the running version, which is the image's claim about itself.
        imageVerdict = EspImage.check(data,
                                      expectedProject: expectedProject,
                                      expectedChipId: expectedChipId,
                                      runningVersion: targetIsFC
                                          ? device.fcFirmwareVersion
                                          : device.firmwareVersion,
                                      provisionedBoard: targetIsFC
                                          ? device.fcBoardRev
                                          : device.ocBoardRev)
    }

    private var isTerminalState: Bool {
        switch session.state {
        case .verified, .rollbackDetected, .failed: return true
        default: return false
        }
    }

    private var isFailedState: Bool {
        if case .failed = session.state { return true }
        return false
    }

    /// Terminal AND the image reached the device — the cases where re-flashing
    /// the same file is not what the user wants. `.failed` is deliberately
    /// excluded: retrying is exactly what they want.
    private var isCompletedState: Bool {
        switch session.state {
        case .verified, .rollbackDetected: return true
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

    // MARK: - Published releases (#773 step 4c)

    /// The published-release source, rendered from `catalog.state` and nothing
    /// else.
    ///
    /// Every image the release holds for this unit is listed, not just the
    /// best one. The catalog ranks a matching board first and an unsuffixed
    /// image second, but it deliberately refuses to DEFAULT to a revision this
    /// board is not — so when `best` is nil the list is still here for a
    /// deliberate choice, with the reason said out loud rather than an empty
    /// panel.
    @ViewBuilder
    private var catalogView: some View {
        switch catalog.state {
        case .idle:
            Button(action: startCheck) {
                HStack {
                    Image(systemName: "arrow.down.circle")
                    Text("Check for a published release…")
                }
            }
            .buttonStyle(.bordered)
            .disabled(isInProgress)

        case .checking:
            HStack(spacing: 8) { ProgressView(); Text("Looking for a release…").font(.subheadline) }

        case .ready(let release, let images, let best, let alreadyRunning, let boardKnown,
                    let offline, let held):
            VStack(alignment: .leading, spacing: 6) {
                Text("Release \(release.tag)").font(.subheadline)
                if offline {
                    // The images are real and still verified on the way out; it
                    // is the CATALOG that may be older than what has since been
                    // published, and the operator should know which they are
                    // looking at rather than assume it is current.
                    Text("Offline — showing what this phone downloaded earlier. A newer release may exist.")
                        .font(.caption).foregroundColor(.secondary)
                }
                if alreadyRunning {
                    // Not hidden and not blocked: re-flashing the running
                    // version is a legitimate repair. It just should not look
                    // like an update when it is not one.
                    Text("This unit already runs this build — flashing it again is a re-flash, not an update.")
                        .font(.caption).foregroundColor(.secondary)
                }
                if best == nil {
                    // Different problem, different sentence: an unknown board
                    // cannot be ranked at all, and guessing there recommended
                    // the mini's image to a V9 (#773, bench 2026-09-10).
                    Text(boardKnown
                         ? "Nothing in this release is built for this board, so none is offered by default. Choose one only if you know it fits."
                         : "This unit has not reported which board revision it is, so none is recommended. Choose the one matching your hardware.")
                        .font(.caption).foregroundColor(.red)
                }
                ForEach(images, id: \.file) { img in
                    Button { catalog.download(img) } label: {
                        HStack {
                            if img == best { Image(systemName: "checkmark") }
                            Text("\(img.summary) · \(byteCountString(Int(img.sizeBytes)))"
                                 // What is already on the phone, so the operator
                                 // can see at a glance what a dead signal still
                                 // leaves them.
                                 + (held.contains(img.sha256) ? " · on this phone" : ""))
                                .font(.caption)
                            Spacer()
                        }
                    }
                    .buttonStyle(.bordered)
                    .disabled(isInProgress)
                }
                HStack {
                    // The "do this at home" action, which is what makes a field
                    // with no signal survivable at all.
                    Button("Download all for offline use") { catalog.prefetch(images) }
                        .font(.caption)
                    Button("Cancel") { catalog.reset() }.font(.caption)
                }
            }

        case .downloading(let image):
            HStack(spacing: 8) {
                ProgressView()
                Text("Downloading \(image.file)…").font(.subheadline)
            }

        case .prefetching(let image, let index, let total):
            HStack(spacing: 8) {
                ProgressView()
                Text("Downloading \(index) of \(total): \(image.file)…").font(.subheadline)
            }

        case .prefetched(_, let stored, let failed, let bytesHeld):
            VStack(alignment: .leading, spacing: 4) {
                Text("\(stored) image(s) ready offline · \(byteCountString(Int(bytesHeld))) on this phone")
                    .font(.caption)
                if !failed.isEmpty {
                    // Named rather than counted: knowing WHICH one is missing is
                    // what lets someone retry the one that matters before leaving.
                    Text("Could not download: \(failed.joined(separator: ", "))")
                        .font(.caption).foregroundColor(.red)
                }
                Button("Back to the list", action: startCheck).font(.caption)
            }

        // Terminal only for an instant: adoptDownload picks the bytes up and
        // resets the session.
        case .downloaded(let release, let image, let bytes):
            Text("Downloaded \(image.file)").font(.caption)
                .onAppear { adoptDownload(release: release, image: image, bytes: bytes) }

        case .failed(let reason):
            VStack(alignment: .leading, spacing: 4) {
                Text(reason).font(.caption).foregroundColor(.red)
                Button("Try again", action: startCheck).font(.caption)
            }
        }
    }

    private func startCheck() {
        catalog.check(
            expectedProject: expectedProject,
            provisionedBoard: targetIsFC ? device.fcBoardRev : device.ocBoardRev,
            runningVersion: targetIsFC ? device.fcFirmwareVersion : device.firmwareVersion
        )
    }

    /// Adopt a verified download as the picked file.
    ///
    /// Written to a temp file rather than flashed from memory because the OTA
    /// session takes a URL — and routing it through the same field as a
    /// hand-picked file means EspImage.check reads the DOWNLOADED image's own
    /// header, so the manifest that described it is never the last word on
    /// what is about to be flashed.
    private func adoptDownload(release: FirmwareRelease, image: FirmwareImage, bytes: Data) {
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent(image.file)
        do {
            try bytes.write(to: url, options: .atomic)
        } catch {
            catalog.reset()
            return
        }
        pickedFileURL = url
        pickedFileName = "\(image.file) (\(release.tag))"
        pickedFileSize = bytes.count
        pickedFileSha = Data(SHA256.hash(data: bytes)).map { String(format: "%02x", $0) }.joined()
        pickedFileData = bytes
        revalidateImage()
        if isTerminalState { session.reset() }
        catalog.reset()
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
            // Hash the picked file up front and show it. A `sha_mismatch` from
            // the device says the bytes it received differ from the bytes we
            // hashed — but NOT whether the file on the phone was already wrong
            // (bad copy off the Mac) or the BLE transfer corrupted it. Reading
            // this against the build machine's sha256sum separates the two
            // before committing to a multi-minute flash.
            if let data = try? Data(contentsOf: url) {
                pickedFileSha = Data(SHA256.hash(data: data))
                    .map { String(format: "%02x", $0) }.joined()
                pickedFileData = data
            } else {
                pickedFileSha = ""
                pickedFileData = nil
            }
            revalidateImage()   // #773
            // Re-arm after a completed/failed run: clearing the terminal state
            // back to .idle re-enables the Flash button for this new file.
            if isTerminalState { session.reset() }
        case .failure(let error):
            print("File picker error: \(error)")
        }
    }

    private func startFlash() {
        guard let url = pickedFileURL else { return }
        // Retry after a failure: clear the terminal state first so the run
        // starts from .idle. Without this the previous .failed reason would
        // linger behind the new attempt's progress.
        if isTerminalState { session.reset() }
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
