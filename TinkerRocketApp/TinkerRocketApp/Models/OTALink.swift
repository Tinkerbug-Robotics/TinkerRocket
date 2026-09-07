import Foundation

/// The ten members OTASession actually touches on a device — split out so
/// the flow tests can drive the whole state machine against a scripted link
/// (the parity ledger's Phase 9 seam; this is the same shape as Android's
/// `sessionLookup: () -> DeviceSession?` design, converging the two apps
/// rather than adding a parallel one).
///
/// BLEDevice conforms retroactively: nine members are its existing surface,
/// and `clearOtaStatus()` is the one addition, implemented below.
protocol OTALink: AnyObject {
    var isConnected: Bool { get }
    var otaStatus: OTAStatusUpdate? { get }
    /// Forget the last `ota_status`. OTASession calls this right before a new
    /// OTA_BEGIN goes out, so its begin wait can only read a status the device
    /// sent AFTER that begin. The cache otherwise keeps a `verify_failed` from
    /// the previous run for the life of the connection, and the next run's
    /// wait read it on its first poll and failed in 0 ms with a stale token
    /// (#1049). Android's twin is `DeviceSession.clearOtaStatus()`.
    func clearOtaStatus()
    var otaMaxChunkSize: Int { get }
    var firmwareVersion: String { get }
    var fcFirmwareVersion: String { get }

    func sendOtaBegin(targetIsFC: Bool, totalSize: UInt32, sha256: Data)
    func sendOtaChunk(offset: UInt32, data: Data, isLast: Bool) async throws
    func sendOtaFinish()
    func sendOtaAbort()
}

extension BLEDevice: OTALink {
    func clearOtaStatus() { otaStatus = nil }
}
