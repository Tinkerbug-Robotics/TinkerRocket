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
        /// Flip otaStatus to verify_failed when FINISH is sent, i.e. the
        /// terminal status arrives AFTER the pump has drained rather than
        /// during it. Both arrivals are real (#1425): the relay's refusal is
        /// raced against a pump that may already have run out of image.
        var failAtFinish = false
        /// What that flip reports. Defaults keep the existing cases on the
        /// local-path `bad_offset`; the relay cases script the FC's own token
        /// and byte count.
        var failErr = "bad_offset"
        var failBytes = 0

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
                otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: failBytes,
                                            err: failErr, fw: nil)
            }
        }
        /// When FINISH was first sent, so a test can measure how long the
        /// session then took to decide — the difference between reading a
        /// terminal status and waiting out the finish window.
        var finishAt: Date?
        func sendOtaFinish() {
            finishCount += 1
            if finishAt == nil { finishAt = Date() }
            if failAtFinish {
                otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: failBytes,
                                            err: failErr, fw: nil)
            }
        }
        func sendOtaAbort() { abortCount += 1 }
        func clearOtaStatus() { otaStatus = nil }
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

    /// A Data EspImage.parse accepts, carrying `version` in its
    /// `esp_app_desc_t`: image magic 0xE9 at 0, app-desc magic 0xABCD5432 at
    /// 32, version[32] at 32+16. `image(_:)` above is deliberately NOT one —
    /// a file with no readable descriptor is the fallback path.
    private func espImage(_ version: String, _ n: Int = 600) -> Data {
        var b = [UInt8]((0..<n).map { UInt8($0 % 251) })
        b[0] = 0xE9
        b[32] = 0x32; b[33] = 0x54; b[34] = 0xCD; b[35] = 0xAB
        let v = Array(version.utf8)
        precondition(v.count < 32, "version must fit esp_app_desc_t.version[32]")
        for (i, c) in v.enumerated() { b[48 + i] = c }
        b[48 + v.count] = 0
        return Data(b)
    }

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

    /// #1337. Bench 2026-09-10: a relay flash landed byte-exact
    /// (`state=3 err=0 bytes=647280`) and the FC rebooted cleanly, but the app
    /// announced a rollback — the image was built from the same commit the FC
    /// already ran, so the version string did not move and the old check read
    /// that as a bootloader revert. Success is "running the image we sent".
    func testSameVersionImage_landsSuccessfully_verifiesInsteadOfCryingRollback() async throws {
        let v = "e1a4bee-v9+20260910-1020"
        let link = ScriptedLink(firmwareVersion: "oc-unchanged", fcFirmwareVersion: v)
        let box = LinkBox(link)
        let session = makeSession(box)

        // Flash the FC an image carrying exactly the version it already runs.
        session.start(data: espImage(v), targetIsFC: true)
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("finish") { link.finishCount == 1 }
        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: nil)

        // An FC relay never drops this link — the peer is the OC — so there is
        // no swap here. The FC reboots and reports the same string back.
        try await waitUntil("verified, not rollback") {
            session.state == .verified(newVersion: v)
        }
    }

    /// The other half of #1337: narrowing the rollback verdict must not delete
    /// it. A DIFFERENT image version coming back as the old one is a revert.
    func testRealImage_deviceBackOnOldVersion_stillReportsRollback() async throws {
        let link = ScriptedLink(firmwareVersion: "v1-old")
        let box = LinkBox(link)
        let session = makeSession(box)

        session.start(data: espImage("v2-new"))
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

    // MARK: - #1425 / #1125: a wrong-board image refused over the FC relay

    /// The bench case behind #1125, pinned to the app's own decision.
    ///
    /// An FC image built with the wrong board flag is refused by the FC after
    /// the descriptor completes — measured on hardware 2026-09-12 as
    /// `{"state":"verify_failed","bytes":220,"err":"fc_image_identity_mismatch"}`
    /// reaching the app's characteristic 47 ms after the triggering chunk. What
    /// the operator must then read is that token, not a timeout: #1267/#1273
    /// exist because "did not finalize within 60s" reads as "it might have
    /// half-landed" when in fact 220 bytes were accepted and nothing was
    /// written. The 60 s finish window is the relay's own (OTATimeouts), so a
    /// missed short-circuit here costs a full minute before a wrong message.
    func testRelayVerifyFailedMidPump_reportsTheIdentityTokenNotATimeout() async throws {
        let link = ScriptedLink()
        link.failAtChunk = 2
        link.failErr = "fc_image_identity_mismatch"
        link.failBytes = 220
        let session = makeSession(LinkBox(link))

        session.start(data: image(200_000), targetIsFC: true)
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        XCTAssertTrue(link.beginCalls[0].targetIsFC, "the relay path, not a local OC OTA")
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)

        try await waitUntil("pump failure") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("fc_image_identity_mismatch"),
                      "the FC's own token, not a generic failure: \(reason)")
        XCTAssertTrue(reason.contains("220 of 200000 B"),
                      "carries how little was accepted — a partial transfer and a "
                      + "corrupt full one are different faults: \(reason)")
        XCTAssertFalse(reason.contains("did not finalize"),
                       "the timeout wording is exactly what #1267 was filed for: \(reason)")
        XCTAssertEqual(link.finishCount, 0, "a refused image never reaches FINISH")
        XCTAssertGreaterThanOrEqual(link.abortCount, 1, "abort sent on the failure path")
        XCTAssertLessThan(link.chunks.count, 10, "pump stopped near the rejection")
    }

    /// The same refusal, arriving AFTER the pump has drained.
    ///
    /// This is the arrival the original #1425 report actually represents: the
    /// app had pushed the whole image and was sitting in the finish wait when
    /// it gave up at 60 s. Two code paths carry the token — the pump loop's
    /// check and `awaitFinish`'s — and only the first is covered by the
    /// mid-pump case above.
    ///
    /// **The timing assertion is the load-bearing one here, not the message.**
    /// Deleting `awaitFinish`'s `verifyFailed` short-circuit does NOT change
    /// what the operator eventually reads: the terminal-status branch in the
    /// finish handler still finds the same status and prints the same token.
    /// It changes only WHEN — the status change resets the no-progress
    /// deadline, so the session sits for the full relay finish window (60 s
    /// live, 300 ms at `timeScale`) before saying anything. A message-only
    /// assertion would pass right through that regression, which is why this
    /// case measures from FINISH.
    ///
    /// Note the wording legitimately differs between the two arrivals ("Device
    /// rejected chunk" vs "Verify failed"): the phase is real information.
    /// What must not differ is that the token and the byte count survive.
    func testRelayVerifyFailedAfterPump_shortCircuitsTheFinishWindow() async throws {
        let link = ScriptedLink()
        link.failAtFinish = true
        link.failErr = "fc_image_identity_mismatch"
        link.failBytes = 220
        let session = makeSession(LinkBox(link))

        session.start(data: image(1500), targetIsFC: true)
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)

        try await waitUntil("finish failure") { self.failureReason(session.state) != nil }
        let decidedIn = Date().timeIntervalSince(try XCTUnwrap(link.finishAt))
        let reason = failureReason(session.state) ?? ""
        XCTAssertEqual(link.finishCount, 1, "the pump drained, so FINISH was sent")
        // The relay finish window is 60 s; at timeScale that is 300 ms. Reading
        // the status takes one scaled poll (0.25 ms), so anything approaching
        // the window means the short-circuit is gone. 100 ms sits 3x under the
        // window and ~100x over the fast path, which is the widest gap a
        // wall-clock assertion can take here.
        XCTAssertLessThan(decidedIn, 0.1,
                          "decided \(decidedIn)s after FINISH — the 300 ms scaled window "
                          + "means awaitFinish waited the refusal out instead of reading it")
        XCTAssertTrue(reason.contains("fc_image_identity_mismatch"),
                      "the token survives the late arrival: \(reason)")
        XCTAssertTrue(reason.contains("220 of 1500 B"), reason)
        XCTAssertFalse(reason.contains("did not finalize"),
                       "burning the 60 s window and then blaming no-progress is "
                       + "the #1425 failure mode: \(reason)")
        XCTAssertGreaterThanOrEqual(link.abortCount, 1, "abort sent on the failure path")
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
        XCTAssertGreaterThanOrEqual(link.abortCount, 1,
            "a begin the device never answered is aborted, so a session it may have opened is closed (#1049)")
    }

    func testBeginRefused_reportsTheFirmwareTokenNotTheTimeout() async throws {
        // #1106: the OC refuses to flash its own image while the FC reports
        // INFLIGHT, answering OTA_BEGIN with verify_failed/inflight_refused
        // (bad_payload and bad_target take the same path). The begin wait
        // fails fast on that status, so the message must carry the token —
        // the timeout wording would hide the one thing the firmware said.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: 0,
                                         err: "inflight_refused", fw: nil)

        try await waitUntil("begin refusal") { self.failureReason(session.state) != nil }
        XCTAssertEqual(failureReason(session.state), "Device refused OTA_BEGIN: inflight_refused")
        XCTAssertEqual(link.chunks.count, 0, "no chunks after a refused begin")
        XCTAssertGreaterThanOrEqual(link.abortCount, 1, "every failure exit after OTA_BEGIN aborts (#1049)")
    }

    // MARK: - #1049: the cached status must not outlive the run

    func testRetryAfterFinishVerifyFailed_beginReadsOnlyAFreshStatus() async throws {
        // The cached ota_status lived for the whole connection and nothing
        // cleared it. After a finish-stage verify_failed the next run's begin
        // wait read that stale value on its first poll and failed in 0 ms with
        // "did not accept OTA_BEGIN within 5s" — for a begin the firmware was
        // in fact accepting.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("finish") { link.finishCount == 1 }
        let abortsBefore = link.abortCount
        link.otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: 600, err: "sha_mismatch", fw: nil)
        try await waitUntil("verify failure") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("sha_mismatch"), reason)
        XCTAssertEqual(link.abortCount, abortsBefore + 1,
                       "a finish-stage failure tells the device the session is over")

        // "Try again". The scripted link never answers the abort, so the
        // stale verify_failed is still the last status this session saw.
        session.reset()
        session.start(data: image(600))
        try await waitUntil("second begin") { link.beginCalls.count == 2 }
        XCTAssertNil(link.otaStatus, "cache forgotten at OTA_BEGIN")
        XCTAssertNil(failureReason(session.state), "still waiting, not failed in 0 ms")
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("second finish") { link.finishCount == 2 }
        XCTAssertEqual(session.state, .verifying, "the second run pumped and finished")
    }

    func testRetryAfterBeginRefusal_beginReadsOnlyAFreshStatus() async throws {
        // The same trap from the other refusal: a rocket that refused in
        // flight (#1106) and is retried on the same connection after landing
        // must wait for the new verdict, not replay the old one.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .verifyFailed, bytes: 0, err: "inflight_refused", fw: nil)
        try await waitUntil("refusal") { self.failureReason(session.state) != nil }

        session.reset()
        session.start(data: image(600))
        try await waitUntil("second begin") { link.beginCalls.count == 2 }
        XCTAssertNil(failureReason(session.state), "still waiting, not failed in 0 ms")
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("second finish") { link.finishCount == 1 }
        XCTAssertEqual(session.state, .verifying)
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
        // Read the window from the table rather than repeating the number: it
        // has now moved twice (#627 stretched the FC path, #773 the local one),
        // and a literal here fails the build for the wrong reason each time.
        let localFinish = Int(OTATimeouts.seconds(.finish, targetIsFC: false))
        XCTAssertTrue(reason.contains("\(localFinish)s"),
                      "local finish window in the message: \(reason)")
        XCTAssertGreaterThanOrEqual(link.abortCount, 1, "a finish the device never answered is aborted (#1049)")
    }

    /// The local finish window in this suite's compressed time, in nanoseconds.
    private var scaledLocalFinishNs: UInt64 {
        UInt64(OTATimeouts.seconds(.finish, targetIsFC: false) / 200.0 * 1_000_000_000)
    }

    func testAMovingByteCountKeepsTheFinishWaitAlive() async throws {
        // THE BENCH CASE, out-computer console 2026-09-10 against an
        // 815,696 B image:
        //
        //   [ 5.87] OTA_BEGIN: size=815696
        //   [ 9.10] OTA begin: partition 'ota_1'        <- 3.2 s erase
        //   [43.71] OTA_FINISH (bytes_written=815696)   <- 34.6 s RECEIVING
        //   [44.09] OTA: ready to boot                  <- 0.38 s finish work
        //
        // The app stopped pumping ~26 s before the device saw FINISH — its
        // writes drain out of the phone's BLE stack long after the pump loop
        // returns. So this window is spent watching a transfer still arriving,
        // and the device says so twice a second in `writing` updates whose
        // BYTE COUNT climbs while the state does not change. Keying on the
        // state alone made all of that invisible.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        var written = 100_000
        for _ in 0..<4 {
            try await Task.sleep(nanoseconds: scaledLocalFinishNs * 7 / 10)
            written += 50_000
            link.otaStatus = OTAStatusUpdate(state: .writing, bytes: written, err: nil, fw: nil)
        }
        XCTAssertNil(failureReason(session.state),
                     "bytes were still climbing, so this was never a stall")

        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: written, err: nil, fw: nil)
        try await Task.sleep(nanoseconds: 40_000_000)
        XCTAssertNil(failureReason(session.state))
    }

    func testAStuckByteCountStillTimesOut() async throws {
        // The other half, and the reason this is progress rather than mere
        // chatter: a device repeating the SAME byte count is not making any,
        // and must still fail. Otherwise a wedged transfer waits forever.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        for _ in 0..<3 {
            link.otaStatus = OTAStatusUpdate(state: .writing, bytes: 250_000, err: nil, fw: nil)
            try await Task.sleep(nanoseconds: scaledLocalFinishNs / 3)
        }
        try await waitUntil("finish timeout") { self.failureReason(session.state) != nil }
        XCTAssertNotNil(failureReason(session.state))
    }

    func testVerifyingRestartsTheFinishBudget() async throws {
        // #773: the finish window is a NO-PROGRESS budget, not a total. A
        // device that keeps saying it is working must never be cut off — the
        // app cannot see inside esp_ota_end(), so "has it said anything
        // lately" is the only honest question it can ask.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        // Sit most of the budget, then prove we are alive.
        try await Task.sleep(nanoseconds: scaledLocalFinishNs * 7 / 10)
        link.otaStatus = OTAStatusUpdate(state: .verifying, bytes: 0, err: nil, fw: nil)

        // Past the point the old fixed window would have given up.
        try await Task.sleep(nanoseconds: scaledLocalFinishNs * 7 / 10)
        XCTAssertNil(failureReason(session.state),
                     "the heartbeat restarted the budget, so this is not a timeout")

        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: nil)
        try await Task.sleep(nanoseconds: 40_000_000)
        XCTAssertNil(failureReason(session.state), "finish completed")
    }

    func testOlderFirmwareThatSaysNothingStillTimesOutTheSameWay() async throws {
        // Firmware from before the heartbeat goes ready -> silence -> terminal.
        // Nothing restarts the budget, so the wait must behave exactly as the
        // old fixed one did — this is the compatibility the change rests on.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }

        try await waitUntil("finish timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("no progress"),
                      "says the device never reported: \(reason)")
    }

    func testADeviceThatGoesQuietAfterVerifyingSaysWhichFailureItWas() async throws {
        // The two silences mean different things to an operator: nothing at
        // all may be a lost FINISH, whereas stopping mid-verify means the
        // image may already be committed — do not power-cycle yet.
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }
        link.otaStatus = OTAStatusUpdate(state: .verifying, bytes: 0, err: nil, fw: nil)

        try await waitUntil("finish timeout") { self.failureReason(session.state) != nil }
        let reason = failureReason(session.state) ?? ""
        XCTAssertTrue(reason.contains("verifying"),
                      "names what the device last said: \(reason)")
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

    // MARK: - Cancel stays "Cancelled" in every phase

    // The Kotlin twin gets these for free: job.cancel() throws a
    // CancellationException out of every delay(), which unwinds runFlow before
    // it can write state. Here the sleep's CancellationError used to land in
    // the begin/finish catch blocks and overwrite "Cancelled" with the timeout
    // wording (sending a second OTA_ABORT since #1203), and a cancel during
    // the reboot phase fell through the reconnect and version waits to a
    // spurious rollback verdict. Each case waits out the scaled window it
    // cancelled inside, then checks nothing overwrote the cancel.

    func testCancelDuringBeginWait_staysCancelled() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        session.cancel()
        XCTAssertEqual(failureReason(session.state), "Cancelled")

        try await Task.sleep(nanoseconds: 80_000_000)   // begin window is 25 ms scaled
        XCTAssertEqual(failureReason(session.state), "Cancelled",
                       "the begin timeout must not overwrite the cancel")
        XCTAssertEqual(link.abortCount, 1, "one abort, from cancel() — not a second from the catch")
        XCTAssertEqual(link.chunks.count, 0)
    }

    func testCancelDuringFinishWait_staysCancelled() async throws {
        let link = ScriptedLink()
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("verifying") { session.state == .verifying }
        session.cancel()
        XCTAssertEqual(failureReason(session.state), "Cancelled")

        try await Task.sleep(nanoseconds: 150_000_000)  // finish window is 75 ms scaled
        XCTAssertEqual(failureReason(session.state), "Cancelled",
                       "the finish timeout must not overwrite the cancel")
        XCTAssertEqual(link.abortCount, 1, "one abort, from cancel() — not a second from the catch")
    }

    func testCancelWhileRebooting_staysCancelled() async throws {
        // Worst case before the fix: with the link still connected the
        // disconnect wait times out, the reconnect wait passes at once, the
        // version wait times out on the unchanged firmware, and the flow
        // declared rollbackDetected("v1-old") over the user's own cancel.
        let link = ScriptedLink(firmwareVersion: "v1-old")
        let session = makeSession(LinkBox(link))

        session.start(data: image(600))
        try await waitUntil("begin") { link.beginCalls.count == 1 }
        link.otaStatus = OTAStatusUpdate(state: .ready, bytes: 0, err: nil, fw: nil)
        try await waitUntil("finish") { link.finishCount == 1 }
        link.otaStatus = OTAStatusUpdate(state: .readyToBoot, bytes: 0, err: nil, fw: nil)
        try await waitUntil("rebooting") { session.state == .rebooting }
        session.cancel()
        XCTAssertEqual(failureReason(session.state), "Cancelled")

        try await Task.sleep(nanoseconds: 450_000_000)  // disconnect+reconnect+fw = 375 ms scaled
        XCTAssertEqual(failureReason(session.state), "Cancelled",
                       "neither the reconnect timeout nor a rollback verdict may overwrite the cancel: \(session.state)")
        XCTAssertEqual(link.abortCount, 1)
    }
}
