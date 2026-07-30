import Foundation

/// The nine members OTASession actually touches on a device — split out so
/// the flow tests can drive the whole state machine against a scripted link
/// (the parity ledger's Phase 9 seam; this is the same shape as Android's
/// `sessionLookup: () -> DeviceSession?` design, converging the two apps
/// rather than adding a parallel one).
///
/// BLEDevice conforms retroactively with zero changes: the protocol is cut
/// exactly along its existing surface.
protocol OTALink: AnyObject {
    var isConnected: Bool { get }
    var otaStatus: OTAStatusUpdate? { get }
    var otaMaxChunkSize: Int { get }
    var firmwareVersion: String { get }
    var fcFirmwareVersion: String { get }

    func sendOtaBegin(targetIsFC: Bool, totalSize: UInt32, sha256: Data)
    func sendOtaChunk(offset: UInt32, data: Data, isLast: Bool) async throws
    func sendOtaFinish()
    func sendOtaAbort()
}

extension BLEDevice: OTALink {}
