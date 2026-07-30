import XCTest
@testable import TinkerRocketApp

/// OTA state machine against a scripted link — the port of Android's 9
/// `OtaSessionTest` cases enabled by the `OTALink` seam (parity-ledger
/// Phase 9 item).  Every timeout branch, the rollback path, and the
/// survives-a-reconnect property run against the REAL timeout table at
/// `timeScale = 1/200` (5 s → 25 ms), so the cases finish in milliseconds
/// while exercising the production windows and their relative ordering.
///
/// Deliberate divergences from the Kotlin twin, both structural:
/// - Wire-byte assertions (chunk framing, begin layout) stay Android-side —
///   here the seam records ARGUMENTS; the byte encoding lives in BLEDevice
///   and is golden-pinned separately.
/// - No connection-priority assertions — iOS has no priority boost to
///   release (writeWithoutResponse batching is its whole strategy).
@MainActor
final class OTASessionFlowTests: XCTestCase {

    // MARK: - Scripted link

    final class ScriptedLink: OTALink {
        var isConnected = true
        var otaStatus: OTAStatusUpdate?
        var otaMaxChunkSize = 507          // MTU 517 parity with the Kotlin rig
        var firmwareVersion: String
        var fcFirmwareVersion: String

        var beginCalls: [(targetIsFC: Bool, totalSize: UInt32, sha256: Data)] = []
        var chunks: [(offset: UInt32, data: Data, isLast: Bool)] = []
        var finishCount = 0
        var abortCount = 0
        /// Set to N to flip otaStatus to verify_failed as the Nth chunk lands.
        var failAtChunk: Int?

        init(firmwareVersion: String = "v1-old", fcFirmwareVersion: String = "fc-v1-old") {
            self.firmwareVersion = firmwareVersion
            self.fcFirmwareVersion = fcFirmwareVersion
        }

        func sendOtaBegin(targetIsFC: Bool, totalSize: UInt32, sha256: Data) {
            beginCalls.append((targetIsFC, totalSize, sha256))
        }
        func sendOtaChunk(offset: UInt32, data: Data, isLast: Bool) async throws {
            chunks.append((offset, data, isLast))
            if let n = failAtChunk, chunks.count >= n {
                otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: 0,
                                            err: "bad_offset", fw: nil)
            }
        }
        func sendOtaFinish() { finishCount += 1 }
        func sendOtaAbort() { abortCount += 1 }
    }

    /// Holder so the session's lookup tracks swaps — the fleet-recreates-the-
    /// device property under test.
    final class LinkBox {
        var link: ScriptedLink?
        init(_ link: ScriptedLink?) { self.link = link }
    }

    private func makeSession(_ box: LinkBox) -> OTASession {
        let s = OTASession(linkLookup: { box.link })
        s.timeScale = 1.0 / 200.0
        return s
    }

    private func image(_ n: Int) -> Data { Data((0..<n).map { UInt8($0 % 251) }) }

    /// Poll a predicate on the main actor; real-time ceiling generous enough
    /// for CI (all scaled windows are ≤ 600 ms).
    private func waitUntil(_ what: String = "",
                           timeout: TimeInterval = 3.0,
                           _ predicate: () -> Bool) async throws {
        let deadline = Date().addingTimeInterval(timeout)
        while Date() < deadline {
            if predicate() { return }
            try await Task.sleep(nanoseconds: 2_000_000)
        }
        XCTFail("timed out waiting for \(what)")
        struct TimedOut: Error {}
        throw TimedOut()
    }

    private func failureReason(_ s: OTASession.State) -> String? {
        if case .failed(let reason) = s { return reason }
        return nil
    }

    // MARK: - Happy path

    func testFullFlow_beginChunksFinishRebootReconnect_verifies() async throws {
        let link = ScriptedLink(firmwareVersion: "v1-old")
        let box = LinkBox(link)
        let session = makeSession(box)
        let img = image(1500)

        session.start(data: img)
        try await waitUntil("OTA_BEGIN") { link.beginCalls.count == 1 }
        XCTAssertEqual(link.beginCalls[0].targetIsFC, false)
        XCTAssertEqual(link.beginCalls[0].totalSize, 1500)
        XCTAssertEqual(link.beginCalls[0].sha256.count, 32)
        XCTAssertEqual(session.preFlashFirmwareVersion, "v1-old")

        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("chunk pump + finish") { link.finishCount == 1 }

        // 1500 B at 507 B/chunk → offsets 0/507/1014, isLast only on the tail,
        // and the reassembled payloads are the original image in order.
        XCTAssertEqual(link.chunks.map(\.offset), [0, 507, 1014])
        XCTAssertEqual(link.chunks.map(\.isLast), [false, false, true])
        XCTAssertEqual(link.chunks.last?.data.count, 486)
        XCTAssertEqual(link.chunks.reduce(Data()) { $0 + $1.data }, img)
        XCTAssertEqual(session.state, .verifying)

        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: "v2-new")
        try await waitUntil("rebooting after ready_to_boot") { session.state == .rebooting }

        // Reboot: the fleet destroys the device...
        box.link = nil
        try await Task.sleep(nanoseconds: 40_000_000)
        // ...and builds a fresh one running the new firmware. The session
        // must pick it up through the lookup — it never captured the old one.
        box.link = ScriptedLink(firmwareVersion: "v2-new")

        try await waitUntil("verified") { session.state == .verified(newVersion: "v2-new") }
    }

    // MARK: - Failure branches

    func testRollback_sameVersionBack_reportsRollbackNotSuccess() async throws {
        let link = ScriptedLink(firmwareVersion: "v1-old")
        let box = LinkBox(link)
        let session = makeSession(box)

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("finish") { link.finishCount == 1 }
        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: nil)
        try await waitUntil("rebooting") { session.state == .rebooting }

        box.link = nil
        try await Task.sleep(nanoseconds: 40_000_000)
        box.link = ScriptedLink(firmwareVersion: "v1-old")   // bootloader rolled back

        try await waitUntil("rollback verdict") {
            session.state == .rollbackDetected(version: "v1-old")
        }
    }

    func testVerifyFailedDuringPump_abortsAndReportsFirmwareError() async throws {
        let link = ScriptedLink()
        link.failAtChunk = 2
        let box = LinkBox(link)
        let session = makeSession(box)

        session.start(data: image(200_000))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)

        try await waitUntil("pump failure") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("bad_offset"), "surfaces the firmware token: \(reason)")
        XCTAssertGreaterThanOrEqual(link.abortCount, 1, "abort sent on the failure path")
        XCTAssertLessThan(link.chunks.count, 10, "pump stopped near the rejection")
    }

    func testBeginNotAccepted_timesOutWithoutPumping() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("OTA_BEGIN"), reason)
        XCTAssertTrue(reason.contains("5s"), "reports the UNSCALED local window: \(reason)")
        XCTAssertEqual(link.chunks.count, 0, "no chunks before the firmware says ready")
    }

    func testFinishNeverAcked_failsAfterFinishTimeout() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        try await waitUntil("finish timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("finalize"), reason)
        XCTAssertTrue(reason.contains("15s"), "local finish window in the message: \(reason)")
    }

    func testFcFinishWindowOutlastsTheLocalOne() async throws {
        // Bench 2026-07-28: a real 591.7 kB FC flash was still running when
        // the 15 s local window expired — the FC finished and ran the new
        // image, but the app had already declared failure.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600), targetIsFC: true)
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        XCTAssertEqual(link.beginCalls[0].targetIsFC, true)
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        // Sit past the LOCAL window (15 s → 75 ms scaled, wait 120 ms):
        // a relayed flash must not be cut off by it.
        try await Task.sleep(nanoseconds: 120_000_000)
        XCTAssertEqual(session.state, .verifying,
                       "the local finish window must not cut off a relayed flash")

        try await waitUntil("FC finish timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("60s"), "reports the FC window it actually waited: \(reason)")
    }

    func testNeverReconnects_failsWithPowerCycleHint() async throws {
        let link = ScriptedLink()
        let box = LinkBox(link)
        let session = makeSession(box)

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("finish") { link.finishCount == 1 }
        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: nil)
        try await waitUntil("rebooting") { session.state == .rebooting }

        box.link = nil   // gone, and never comes back

        try await waitUntil("reconnect timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("power-cycling"), reason)
    }

    func testTooSmallImage_rejectedBeforeAnyWrite() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(32))
        try await waitUntil("size rejection") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("too small"), reason)
        XCTAssertEqual(link.beginCalls.count, 0, "nothing sent for a bogus file")
        XCTAssertEqual(link.chunks.count, 0)
    }

    func testCancel_sendsAbort() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(200_000))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("pump running") { !link.chunks.isEmpty }

        session.cancel()
        XCTAssertGreaterThanOrEqual(link.abortCount, 1)
        XCTAssertEqual(failureReason(session.state), "Cancelled")
    }
}
