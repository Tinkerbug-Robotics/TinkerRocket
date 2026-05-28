import Foundation
import Combine
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

    // Owning BLEDevice — unowned because BLEDevice owns this object via
    // `lazy var otaSession`, so the cycle is structural and the session
    // can't outlive its device.
    private unowned let device: BLEDevice
    private(set) var preFlashFirmwareVersion: String = ""
    private(set) var imageSize: Int = 0
    private(set) var imageSha256Hex: String = ""

    private var task: Task<Void, Never>?
    private var statusCancellable: AnyCancellable?
    private var fwCancellable: AnyCancellable?
    private var connCancellable: AnyCancellable?

    // Reboot/reconnect plumbing — set by Combine sinks, consumed by Task.sleep loops.
    private var lastStatusUpdate: OTAStatusUpdate?
    private var sawDisconnect: Bool = false

    init(device: BLEDevice) {
        self.device = device
    }

    deinit {
        task?.cancel()
    }

    /// Kick off the OTA flow. Reads `fileURL` (typically from `.fileImporter`)
    /// into memory, computes its SHA-256, then walks the state machine.
    /// Re-callable: cancels any prior in-flight run.
    func start(fileURL: URL) {
        task?.cancel()
        statusCancellable?.cancel()
        fwCancellable?.cancel()
        connCancellable?.cancel()
        lastStatusUpdate = nil
        sawDisconnect = false

        task = Task { [weak self] in
            await self?.runFlow(fileURL: fileURL)
        }
    }

    /// User-initiated cancel. Sends OTA_ABORT and tears the task down.
    func cancel() {
        task?.cancel()
        device.sendOtaAbort()
        state = .failed(reason: "Cancelled")
    }

    /// Reset back to .idle so the same instance can drive another OTA run.
    /// Called by the UI's "Flash another firmware" button after .verified /
    /// .rollbackDetected / .failed.
    func reset() {
        task?.cancel()
        statusCancellable?.cancel()
        fwCancellable?.cancel()
        connCancellable?.cancel()
        statusCancellable = nil
        fwCancellable = nil
        connCancellable = nil
        preFlashFirmwareVersion = ""
        imageSize = 0
        imageSha256Hex = ""
        lastStatusUpdate = nil
        sawDisconnect = false
        state = .idle
    }

    // MARK: - Flow

    private func runFlow(fileURL: URL) async {
        // ---- 1. Load file + compute SHA-256 ----
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
        if fileData.count < 64 {   // ESP32 app images have a non-trivial header
            state = .failed(reason: "File looks too small to be firmware (\(fileData.count) bytes)")
            return
        }

        let sha = Data(SHA256.hash(data: fileData))
        imageSize = fileData.count
        imageSha256Hex = sha.map { String(format: "%02x", $0) }.joined()
        preFlashFirmwareVersion = device.firmwareVersion

        // ---- 2. Subscribe to device.$otaStatus + connection edges ----
        statusCancellable = device.$otaStatus
            .compactMap { $0 }
            .sink { [weak self] s in self?.lastStatusUpdate = s }

        connCancellable = device.$isConnected
            .removeDuplicates()
            .sink { [weak self] connected in
                if !connected { self?.sawDisconnect = true }
            }

        // ---- 3. Send OTA_BEGIN ----
        guard device.isConnected else {
            state = .failed(reason: "Device disconnected before OTA_BEGIN")
            return
        }
        device.sendOtaBegin(targetIsFC: false, totalSize: UInt32(fileData.count), sha256: sha)

        // ---- 4. Wait for status=ready ----
        do {
            try await awaitOtaState(.ready, timeout: 5.0)
        } catch {
            state = .failed(reason: "Device did not accept OTA_BEGIN within 5s")
            return
        }

        // ---- 5. Chunk pump ----
        let chunkSize = max(64, device.otaMaxChunkSize)
        var offset = 0
        state = .uploading(bytesSent: 0, totalBytes: fileData.count)
        while offset < fileData.count {
            if Task.isCancelled { return }

            // VerifyFailed during the pump? Surface and bail.
            if let st = lastStatusUpdate, st.state == .verifyFailed {
                state = .failed(reason: "Device rejected chunk: \(st.err ?? "unknown")")
                return
            }

            let end = min(offset + chunkSize, fileData.count)
            let chunk = fileData.subdata(in: offset..<end)
            let isLast = (end == fileData.count)
            do {
                try await device.sendOtaChunk(offset: UInt32(offset), data: chunk, isLast: isLast)
            } catch {
                state = .failed(reason: "Chunk write failed at offset \(offset): \(error.localizedDescription)")
                device.sendOtaAbort()
                return
            }
            offset = end
            state = .uploading(bytesSent: offset, totalBytes: fileData.count)
        }

        // ---- 6. OTA_FINISH ----
        state = .verifying
        device.sendOtaFinish()

        // ---- 7. Wait for status=ready_to_boot (or verify_failed) ----
        do {
            try await awaitOtaState(.readyToBoot, timeout: 15.0)
        } catch {
            if let st = lastStatusUpdate, st.state == .verifyFailed {
                state = .failed(reason: "Verify failed: \(st.err ?? "unknown")")
            } else {
                state = .failed(reason: "Device did not finalize OTA within 15s")
            }
            return
        }

        // ---- 8. Wait for disconnect (device reboots ~500ms after ready_to_boot) ----
        state = .rebooting
        sawDisconnect = false
        if !(await waitFor(timeout: 5.0, { [weak self] in self?.sawDisconnect == true || self?.device.isConnected == false })) {
            // No disconnect seen — odd, but proceed to reconnect-await anyway.
        }

        // ---- 9. Wait for reconnect ----
        if !(await waitFor(timeout: 60.0, { [weak self] in self?.device.isConnected == true })) {
            state = .failed(reason: "Device did not reconnect within 60s — try power-cycling")
            return
        }

        // ---- 10. Wait for the new identity push (fw field) ----
        // The firmware republishes config_identity on each connect; we wait
        // for any non-empty fw value that arrives AFTER reconnect.
        let preFlash = preFlashFirmwareVersion
        if !(await waitFor(timeout: 10.0, { [weak self] in
            guard let self else { return false }
            return !self.device.firmwareVersion.isEmpty &&
                   self.device.firmwareVersion != preFlash
        })) {
            // Either no fw push came in (old firmware?) or it matches pre-flash.
            if device.firmwareVersion == preFlash {
                state = .rollbackDetected(version: preFlash)
            } else {
                state = .failed(reason: "Reconnected but device didn't publish a new firmware version within 10s")
            }
            return
        }

        state = .verified(newVersion: device.firmwareVersion)
    }

    // MARK: - Wait helpers

    /// Spin-wait for `lastStatusUpdate.state == expected` (or VerifyFailed,
    /// which fails fast). Polls every 50 ms up to `timeout` seconds.
    private func awaitOtaState(_ expected: OTAStatusUpdate.State, timeout: TimeInterval) async throws {
        struct TimedOut: Error {}
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if Task.isCancelled { throw CancellationError() }
            if let st = lastStatusUpdate {
                if st.state == expected { return }
                if st.state == .verifyFailed { throw TimedOut() }
            }
            try await Task.sleep(nanoseconds: 50_000_000)
        }
        throw TimedOut()
    }

    /// Generic predicate-based wait. Returns true if predicate fired before
    /// timeout. 50 ms poll interval.
    private func waitFor(timeout: TimeInterval, _ predicate: @MainActor @escaping () -> Bool) async -> Bool {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if Task.isCancelled { return false }
            if predicate() { return true }
            try? await Task.sleep(nanoseconds: 50_000_000)
        }
        return false
    }
}
