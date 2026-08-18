import Foundation
import Combine
import CoreBluetooth
import CryptoKit

/// Drives an OTA upload on a single BLEDevice end-to-end:
/// load file → SHA-256 → OTA_BEGIN → chunk pump → OTA_FINISH →
/// wait for reboot → wait for reconnect → compare new fw vs pre-flash fw.
///
/// Lifetime: owned by BLEDevice as a lazy property — survives view churn
/// across the post-OTA disconnect/reconnect so FirmwareUpdateView can pick
/// up the final state when the user navigates back. Call reset() to clear
/// for the next run.
///
/// Per design doc §5 (docs/plans/08-ota-firmware-update.md).
@MainActor
final class OTASession: ObservableObject {

    enum State: Equatable {
        case idle
        case loading                                              // reading file + computing SHA
        case uploading(bytesSent: Int, totalBytes: Int)
        case verifying                                            // OTA_FINISH sent, awaiting ready_to_boot
        case rebooting                                            // disconnected, waiting up to 60 s for reconnect
        case verified(newVersion: String)                         // new fw confirmed on reconnect
        case rollbackDetected(version: String)                    // post-reboot fw same as pre-flash fw
        case failed(reason: String)
    }

    @Published private(set) var state: State = .idle

    let peripheralID: UUID

    // Link lookup, re-resolved on every access — the device object behind it
    // is destroyed+recreated on every BLE disconnect/reconnect (#140) while
    // the session lives on, so it must never be captured.  Same design as
    // Android's `sessionLookup`; the fleet-owning init below builds the
    // closure with a weak fleet (BLEFleet owns this OTASession via
    // `otaSessions[peripheralID]`, so a strong cycle would leak both).
    private let linkLookup: () -> (any OTALink)?

    /// The current link for our peripheral, or nil between reconnects.
    var device: (any OTALink)? { linkLookup() }

    /// Test hook: scales every wait (timeouts + poll interval) so the flow
    /// tests run the REAL windows at 1/200th speed instead of wall-clock.
    /// Reported timeout seconds in failure text stay unscaled — the messages
    /// are part of the contract (the FC-window test asserts on "60s").
    var timeScale: Double = 1.0

    private(set) var preFlashFirmwareVersion: String = ""
    private(set) var imageSize: Int = 0
    private(set) var imageSha256Hex: String = ""

    private var task: Task<Void, Never>?

    init(fleet: BLEFleet, peripheralID: UUID) {
        self.peripheralID = peripheralID
        self.linkLookup = { [weak fleet] in
            fleet?.devices.first { $0.peripheral?.identifier == peripheralID }
        }
    }

    /// Seam init for tests (and any future non-fleet owner).
    init(peripheralID: UUID = UUID(), linkLookup: @escaping () -> (any OTALink)?) {
        self.peripheralID = peripheralID
        self.linkLookup = linkLookup
    }

    deinit {
        task?.cancel()
    }

    /// Kick off the OTA flow. Reads `fileURL` (typically from `.fileImporter`)
    /// into memory, computes its SHA-256, then walks the state machine.
    /// Re-callable: cancels any prior in-flight run.
    func start(fileURL: URL, targetIsFC: Bool = false) {
        task?.cancel()
        task = Task { [weak self] in
            await self?.runFlow(fileURL: fileURL, targetIsFC: targetIsFC)
        }
    }

    /// Data-direct entry (the flow tests use this; file loading is the only
    /// step they skip — everything downstream is the production path).
    func start(data: Data, targetIsFC: Bool = false) {
        task?.cancel()
        task = Task { [weak self] in
            await self?.runFlow(fileData: data, targetIsFC: targetIsFC)
        }
    }

    /// User-initiated cancel. Sends OTA_ABORT and tears the task down.
    func cancel() {
        task?.cancel()
        device?.sendOtaAbort()
        state = .failed(reason: "Cancelled")
    }

    /// Reset back to .idle so the same instance can drive another OTA run.
    /// Called by the UI's "Flash another firmware" button after .verified /
    /// .rollbackDetected / .failed.
    func reset() {
        task?.cancel()
        preFlashFirmwareVersion = ""
        imageSize = 0
        imageSha256Hex = ""
        state = .idle
    }

    // MARK: - Flow

    private func runFlow(fileURL: URL, targetIsFC: Bool) async {
        state = .loading
        let didOpen = fileURL.startAccessingSecurityScopedResource()
        defer { if didOpen { fileURL.stopAccessingSecurityScopedResource() } }

        let fileData: Data
        do {
            fileData = try Data(contentsOf: fileURL)
        } catch {
            state = .failed(reason: "Could not read file: \(error.localizedDescription)")
            return
        }
        await runFlow(fileData: fileData, targetIsFC: targetIsFC)
    }

    private func runFlow(fileData: Data, targetIsFC: Bool) async {
        // ---- 1. Size gate + SHA-256 ----
        state = .loading
        if fileData.count < 64 {   // ESP32 app images have a non-trivial header
            state = .failed(reason: "File looks too small to be firmware (\(fileData.count) bytes)")
            return
        }

        let sha = Data(SHA256.hash(data: fileData))
        imageSize = fileData.count
        imageSha256Hex = sha.map { String(format: "%02x", $0) }.joined()
        // An FC OTA changes the *FC's* version (relayed as fcFirmwareVersion);
        // the OC's own firmwareVersion is untouched by an FC-only update — so the
        // rollback check must compare the right device's version for the chosen
        // target, else an FC OTA always looks like "version unchanged" (#8 P4).
        preFlashFirmwareVersion = (targetIsFC ? device?.fcFirmwareVersion
                                              : device?.firmwareVersion) ?? ""

        // ---- 2. Send OTA_BEGIN ----
        guard let beginDevice = device, beginDevice.isConnected else {
            state = .failed(reason: "Device disconnected before OTA_BEGIN")
            return
        }
        beginDevice.sendOtaBegin(targetIsFC: targetIsFC, totalSize: UInt32(fileData.count), sha256: sha)

        // ---- 3. Wait for status=ready ----
        // The FC relay round-trip (BLE -> OC -> I2C poll -> FC erases ota_1 ->
        // I2S -> OC -> BLE) is far slower than a local OTA's begin, so give it
        // a much longer window before declaring failure (#8 Phase 4).
        let beginTimeoutS = OTATimeouts.seconds(.begin, targetIsFC: targetIsFC)
        do {
            try await awaitOtaState(.ready, timeout: beginTimeoutS)
        } catch {
            state = .failed(reason: "Device did not accept OTA_BEGIN within \(Int(beginTimeoutS))s")
            return
        }

        // ---- 4. Chunk pump (writeWithoutResponse, serial) ----
        // Pipelining at the GATT API level didn't help (#16 first attempt) —
        // CoreBluetooth still serializes write-with-response at the link
        // layer, ~2.5 KB/s. Switching the underlying writeValue type to
        // .withoutResponse lets iOS batch multiple chunks per BLE
        // connection event. sendOtaChunk parks on canSendWriteWithoutResponse
        // when iOS's outgoing buffer fills, so this serial loop won't outrun
        // the link. No per-chunk ATT ack — the firmware's OTA_FINISH SHA
        // check is what catches a dropped chunk.
        let chunkSize = max(64, beginDevice.otaMaxChunkSize)
        var offset = 0
        // #627: the relay path has a drain rate the OC can actually sustain;
        // exceed it and its BLE stack wedges (see fcRelayMaxBytesPerSec). A
        // local OC OTA has no relay and stays uncapped.
        let pumpStart = Date()
        state = .uploading(bytesSent: 0, totalBytes: fileData.count)
        while offset < fileData.count {
            if Task.isCancelled { return }

            // VerifyFailed during the pump? Surface and bail.
            if let st = device?.otaStatus, st.state == .verifyFailed {
                state = .failed(reason: "Device rejected chunk: \(st.err ?? "unknown") — accepted \(st.bytes) of \(fileData.count) B")
                device?.sendOtaAbort()
                return
            }

            guard let pumpDevice = device, pumpDevice.isConnected else {
                state = .failed(reason: "Device disconnected mid-upload at offset \(offset)")
                return
            }

            let end = min(offset + chunkSize, fileData.count)
            let chunk = fileData.subdata(in: offset..<end)
            let isLast = (end == fileData.count)
            do {
                try await pumpDevice.sendOtaChunk(offset: UInt32(offset), data: chunk, isLast: isLast)
            } catch {
                state = .failed(reason: "Chunk write failed at offset \(offset): \(error.localizedDescription)")
                device?.sendOtaAbort()
                return
            }
            offset = end
            state = .uploading(bytesSent: offset, totalBytes: fileData.count)

            if targetIsFC, offset < fileData.count {
                let pause = OTATimeouts.fcRelayPaceDelay(
                    bytesSent: offset, elapsed: Date().timeIntervalSince(pumpStart))
                if pause > 0 {
                    try? await Task.sleep(nanoseconds: UInt64(pause * timeScale * 1_000_000_000))
                }
            }
        }

        // ---- 4b. Re-hash the source AFTER the pump ----
        // Splits a `sha_mismatch` into its two very different causes. The
        // device reports the full byte count and contiguous offsets (a drop or
        // reorder fails earlier as bad_offset), so the bytes it received simply
        // differ from the ones we hashed at step 1. Either the buffer we sent
        // FROM changed underneath us during the multi-minute pump —
        // Data(contentsOf:) can memory-map, and a file the OS evicts or
        // re-downloads mid-flight then reads back different — or the transfer
        // itself corrupted them. Hashing the same buffer again answers which:
        // a changed hash indicts the source, an unchanged one indicts the wire.
        let shaAfterPump = Data(SHA256.hash(data: fileData))
        if shaAfterPump != sha {
            state = .failed(reason: "Source image changed during upload — "
                + "hashed \(sha.prefix(4).map { String(format: "%02x", $0) }.joined()) before, "
                + "\(shaAfterPump.prefix(4).map { String(format: "%02x", $0) }.joined()) after. "
                + "Copy the .bin to local storage (not iCloud) and retry.")
            device?.sendOtaAbort()
            return
        }

        // ---- 5. OTA_FINISH ----
        state = .verifying
        device?.sendOtaFinish()

        // ---- 6. Wait for status=ready_to_boot (or verify_failed) ----
        // Same story as begin: the FC drains the I2S ring, writes the tail,
        // SHA-256s the whole image and sets the boot partition, with every
        // status hop crossing the relay.  Bench-measured on 2026-07-28 (591.7
        // kB FC image): 15 s expired while the flash was still running — the FC
        // went on to finish, reboot and run the new image, but the app had
        // already declared failure.
        let finishTimeoutS = OTATimeouts.seconds(.finish, targetIsFC: targetIsFC)
        do {
            try await awaitOtaState(.readyToBoot, timeout: finishTimeoutS)
        } catch {
            if let st = device?.otaStatus, st.state == .verifyFailed {
                // Byte count included deliberately: short of the image size means
                // the relay dropped chunks, the full size with a SHA failure means
                // they arrived corrupted. Two very different faults that the error
                // token alone doesn't separate on a rocket with no serial access.
                state = .failed(reason: "Verify failed: \(st.err ?? "unknown") — device took \(st.bytes) of \(fileData.count) B")
            } else {
                state = .failed(reason: "Device did not finalize OTA within \(Int(finishTimeoutS))s")
            }
            return
        }

        // ---- 7/8. Wait for the reboot round-trip (local OTA only) ----
        // Only the BLE peer's OWN firmware makes it drop the link. On an FC
        // relay the peer is the OC, which doesn't reboot at all — the FC does,
        // behind it — so there is no disconnect to wait for and no reconnect to
        // confirm. Step 9's comment already said as much ("the OC<->app link
        // never drops"); running these two stages anyway just burned the
        // disconnect timeout on every FC flash and made the app look hung
        // during the window when the FC was in fact rebooting normally.
        state = .rebooting
        if !targetIsFC {
            // BLEFleet destroys the BLEDevice on disconnect, so `device == nil`
            // also counts as "disconnected".
            _ = await waitFor(timeout: OTATimeouts.seconds(.disconnect, targetIsFC: targetIsFC)) { [weak self] in
                self?.device == nil || self?.device?.isConnected == false
            }

            // After BLEFleet creates a fresh BLEDevice for our peripheralID,
            // self.device starts returning the new instance.
            let reconnected = await waitFor(timeout: OTATimeouts.seconds(.reconnect, targetIsFC: targetIsFC)) { [weak self] in
                self?.device?.isConnected == true
            }
            if !reconnected {
                state = .failed(reason: "Device did not reconnect within 60s — try power-cycling")
                return
            }
        }

        // ---- 9. Wait for the new version to publish ----
        // OC/local OTA: the device reboots and republishes config_identity on
        // reconnect (fast). FC OTA: the OC<->app link never drops; we wait for
        // the OC to relay the FC's *new* version (fc_identity), which can't
        // arrive until the FC finishes rebooting (~8 s, longer if GNSS bootstrap
        // is slow) and pushes FC_IDENTITY — so give it a much wider window.
        let preFlash = preFlashFirmwareVersion
        let fwTimeout = OTATimeouts.seconds(.fwPublish, targetIsFC: targetIsFC)
        let gotNewFw = await waitFor(timeout: fwTimeout) { [weak self] in
            let fw = (targetIsFC ? self?.device?.fcFirmwareVersion
                                 : self?.device?.firmwareVersion) ?? ""
            guard !fw.isEmpty else { return false }
            return fw != preFlash
        }
        let postFw = (targetIsFC ? device?.fcFirmwareVersion
                                 : device?.firmwareVersion) ?? ""
        if !gotNewFw {
            if postFw == preFlash && !postFw.isEmpty {
                state = .rollbackDetected(version: preFlash)
            } else {
                state = .failed(reason: "Reconnected but device didn't publish a new firmware version within \(Int(fwTimeout))s")
            }
            return
        }

        state = .verified(newVersion: postFw)
    }

    // MARK: - Wait helpers

    /// Spin-wait for `device.otaStatus.state == expected` (or VerifyFailed,
    /// which fails fast). Polls every 50 ms up to `timeout` seconds.
    /// Reads `device?.otaStatus` directly each iteration so a BLEDevice
    /// reconnect (new instance) is picked up automatically.
    private func awaitOtaState(_ expected: OTAStatusUpdate.State, timeout: TimeInterval) async throws {
        struct TimedOut: Error {}
        let deadline = Date().addingTimeInterval(timeout * timeScale)
        while Date() < deadline {
            if Task.isCancelled { throw CancellationError() }
            if let st = device?.otaStatus {
                if st.state == expected { return }
                if st.state == .verifyFailed { throw TimedOut() }
            }
            try await Task.sleep(nanoseconds: UInt64(OTATimeouts.pollSeconds * timeScale * 1_000_000_000))
        }
        throw TimedOut()
    }

    /// Generic predicate-based wait. Returns true if predicate fired before
    /// timeout. 50 ms poll interval.
    private func waitFor(timeout: TimeInterval, _ predicate: @MainActor @escaping () -> Bool) async -> Bool {
        let deadline = Date().addingTimeInterval(timeout * timeScale)
        while Date() < deadline {
            if Task.isCancelled { return false }
            if predicate() { return true }
            try? await Task.sleep(nanoseconds: UInt64(OTATimeouts.pollSeconds * timeScale * 1_000_000_000))
        }
        return false
    }
}
