import XCTest
@testable import TinkerRocketApp

/// #435: Drift-Cast guidance-point upload confirmation.
/// Covers (a) the "guid_target" JSON frame → GuidanceTargetEcho parse,
/// (b) the pure confirm-match predicate the DriftCast send button gates
/// green on, and (c) the FlightSettingsData v6 flown-target tail.
final class GuidancePointEchoTests: XCTestCase {

    // MARK: - (a) guid_target JSON parse

    private func decode(_ json: String) -> GuidanceTargetEcho? {
        guard let data = json.data(using: .utf8),
              let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any]
        else { return nil }
        return GuidanceTargetEcho(json: dict)
    }

    func testGuidTargetFrameParses() {
        // Byte-for-byte the OC's sendGuidTarget frame shape.
        let echo = decode(
            #"{"type":"guid_target","seq":3,"st":1,"rc":1,"lat":38.123456,"lon":-122.123456,"alt":250}"#)
        XCTAssertNotNil(echo)
        XCTAssertEqual(echo?.seq, 3)
        XCTAssertEqual(echo?.status, GuidanceTargetEcho.statusGeoActive)
        XCTAssertEqual(echo?.lastRc, GuidancePointRc.accepted.rawValue)
        XCTAssertEqual(echo!.lat, 38.123456, accuracy: 1e-9)
        XCTAssertEqual(echo!.lon, -122.123456, accuracy: 1e-9)
        XCTAssertEqual(echo?.altM, 250)
    }

    func testGuidTargetRejectsOtherFrameTypes() {
        XCTAssertNil(decode(#"{"type":"imu_orient","mode":2}"#))
        XCTAssertNil(decode(#"{"type":"config","ge":true}"#))
    }

    func testGuidTargetMissingFieldsDefaultDefensively() {
        // MTU-budget rule (#282): parse must survive a trimmed frame.
        let echo = decode(#"{"type":"guid_target","seq":1}"#)
        XCTAssertNotNil(echo)
        XCTAssertEqual(echo?.status, GuidanceTargetEcho.statusNone)
        XCTAssertEqual(echo?.lastRc, GuidancePointRc.none.rawValue)
        XCTAssertEqual(echo?.lat, 0)
        XCTAssertEqual(echo?.altM, 0)
    }

    // MARK: - (b) confirm-match predicate

    private let sentLat = 38.1234567
    private let sentLon = -122.7654321

    private func echo(seq: Int = 1, st: Int = 1, rc: Int = 1,
                      lat: Double? = nil, lon: Double? = nil) -> GuidanceTargetEcho {
        GuidanceTargetEcho(seq: seq, status: st, lastRc: rc,
                           lat: lat ?? sentLat, lon: lon ?? sentLon, altM: 120)
    }

    func testOutcomePendingWhenSeqUnchanged() {
        // A connect-time re-push of the SAME echo must not confirm anything.
        let e = echo(seq: 5)
        XCTAssertEqual(e.outcome(baselineSeq: 5, sentLat: sentLat, sentLon: sentLon), .pending)
    }

    func testOutcomeConfirmedExactMatch() {
        let e = echo(seq: 6)
        XCTAssertEqual(e.outcome(baselineSeq: 5, sentLat: sentLat, sentLon: sentLon), .confirmed)
    }

    func testOutcomeNeverSettlesWithoutABaseline() {
        // Baseline -1 = no echo received before the send: NOTHING is
        // attributable, so even a perfect-looking accepted/GEO/matching
        // frame must stay pending — it is typically the OC's stale
        // connect-time push of a PREVIOUS session's state (the false-green
        // reconnect path; GuidanceSendFlow adopts it as the baseline).
        let e = echo(seq: 0)
        XCTAssertEqual(e.outcome(baselineSeq: -1, sentLat: sentLat, sentLon: sentLon), .pending)
        // Same for a stale frame with a bigger seq and for the FC boot frame.
        XCTAssertEqual(echo(seq: 4).outcome(baselineSeq: -1, sentLat: sentLat, sentLon: sentLon),
                       .pending)
        XCTAssertEqual(echo(seq: 0, st: 0, rc: 0).outcome(baselineSeq: -1,
                                                          sentLat: sentLat, sentLon: sentLon),
                       .pending)
    }

    func testOutcomeRcNoneIsNeverAVerdict() {
        // The FC writes a nonzero GUID_RC_* on every processed cmd 28, and a
        // cmd-65 apply bumps seq WITHOUT touching rc — so a seq-advanced
        // frame with rc == NONE (e.g. the boot-state echo pushed at connect,
        // or a config wipe before any upload) means "no cmd-28 result yet",
        // NOT "rejected". Mapping it to .rejected showed a false
        // "No upload has been processed" failure moments before the real
        // accept arrived.
        let boot = echo(seq: 0, st: 0, rc: GuidancePointRc.none.rawValue)
        XCTAssertEqual(boot.outcome(baselineSeq: 5, sentLat: sentLat, sentLon: sentLon), .pending)
        let wipe = echo(seq: 3, st: GuidanceTargetEcho.statusEnActive,
                        rc: GuidancePointRc.none.rawValue)
        XCTAssertEqual(wipe.outcome(baselineSeq: 2, sentLat: sentLat, sentLon: sentLon), .pending)
    }

    func testOutcomeConfirmedSurvivesF32Rounding() {
        // The echo carries f32-rounded degrees (ulp ~4e-6 at these
        // magnitudes) — the tolerances must absorb that loss.
        let e = echo(seq: 2,
                     lat: Double(Float(sentLat)),
                     lon: Double(Float(sentLon)))
        XCTAssertEqual(e.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon), .confirmed)
    }

    func testOutcomeRejectedMapsRc() {
        for rc in [2, 3, 4, 5, 6] {
            let e = echo(seq: 2, st: 0, rc: rc)
            XCTAssertEqual(e.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon),
                           .rejected(rc: rc), "rc=\(rc)")
        }
    }

    func testOutcomeRejectionKeepsPreviousTargetVisible() {
        // status/lastRc are separate fields exactly so a rejected upload can
        // ride alongside a still-active previous target: st stays GEO_ACTIVE
        // with the OLD coordinates, rc reports the failure. Rejection must
        // win over the coordinate match.
        let e = GuidanceTargetEcho(seq: 7, status: GuidanceTargetEcho.statusGeoActive,
                                   lastRc: GuidancePointRc.rejectedRadius.rawValue,
                                   lat: 38.0, lon: -122.0, altM: 100)
        XCTAssertEqual(e.outcome(baselineSeq: 6, sentLat: sentLat, sentLon: sentLon),
                       .rejected(rc: 2))
    }

    func testOutcomeMismatchOnDifferentCoordinates() {
        let e = echo(seq: 2, lat: sentLat + 1e-3)   // ~110 m off
        XCTAssertEqual(e.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon), .mismatch)
    }

    func testOutcomeMismatchJustOutsideTolerance() {
        let eLat = echo(seq: 2, lat: sentLat + 2.1e-5)
        XCTAssertEqual(eLat.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon), .mismatch)
        let eLon = echo(seq: 2, lon: sentLon + 3.1e-5)
        XCTAssertEqual(eLon.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon), .mismatch)
    }

    func testOutcomeMismatchWhenStatusNotGeoActive() {
        // rc=ACCEPTED but the active target is not the geodetic point (a
        // cmd-65 apply raced in between) — never green.
        let e = echo(seq: 2, st: GuidanceTargetEcho.statusEnActive)
        XCTAssertEqual(e.outcome(baselineSeq: 1, sentLat: sentLat, sentLon: sentLon), .mismatch)
    }

    // MARK: - (b2) GuidanceSendFlow — verdict attribution across frames

    /// The stale frame the OC re-pushes at connect time: a PREVIOUS upload's
    /// accepted state, coordinates identical to what the user re-sends.
    private var staleConnectPush: GuidanceTargetEcho { echo(seq: 4) }

    func testFlowReconnectStaleFrameCannotConfirm_RealVerdictWins() {
        // Finding scenario A: reconnect cleared the echo, user re-sends the
        // same point with baseline -1, the stale connect push (seq 4, rc
        // ACCEPTED, GEO, matching coords) arrives first, then the FC's REAL
        // verdict — a post-flight REJ_STATE. The old machine went green on
        // the stale frame and swallowed the rejection.
        var flow = GuidanceSendFlow(baselineSeq: -1, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(staleConnectPush)
        XCTAssertEqual(flow.verdict, .waiting)          // adopted as baseline
        XCTAssertEqual(flow.baselineSeq, 4)
        flow.onEcho(echo(seq: 5, rc: GuidancePointRc.rejectedState.rawValue))
        flow.onTimeout()
        XCTAssertEqual(flow.verdict,
                       .failed(GuidancePointRc.rejectedState.message))
    }

    func testFlowNothingSentNothingArrives_TimesOutInsteadOfGreen() {
        // Finding scenario C shape: only the stale connect push ever arrives
        // (e.g. the write never reached the rocket). Must end in the timeout
        // failure, never confirmed.
        var flow = GuidanceSendFlow(baselineSeq: -1, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(staleConnectPush)
        flow.onTimeout()
        XCTAssertEqual(flow.verdict, .failed(GuidanceSendFlow.timeoutMessage))
    }

    func testFlowInterleavedConfigWipeDoesNotSettle_AcceptUpgrades() {
        // Finding scenario D: a cmd-65 apply queued ahead of cmd 28 wipes a
        // previously active geo point — its frame bumps seq with a STALE
        // rc=ACCEPTED, status EN, zeroed coords. The old machine latched a
        // terminal mismatch failure and ignored the genuine accept one poll
        // later.
        var flow = GuidanceSendFlow(baselineSeq: 7, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(GuidanceTargetEcho(seq: 8, status: GuidanceTargetEcho.statusEnActive,
                                       lastRc: GuidancePointRc.accepted.rawValue,
                                       lat: 0, lon: 0, altM: 0))
        XCTAssertEqual(flow.verdict, .waiting)          // provisional only
        XCTAssertNotNil(flow.provisionalFailure)
        flow.onEcho(echo(seq: 9))                       // the genuine accept
        XCTAssertEqual(flow.verdict, .confirmed)
    }

    func testFlowInterleavedWipeWithRcNoneStaysPending() {
        // Finding scenario D': no cmd-28 processed this boot, so the wipe
        // frame carries rc = NONE — not even a provisional failure.
        var flow = GuidanceSendFlow(baselineSeq: 2, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(GuidanceTargetEcho(seq: 3, status: GuidanceTargetEcho.statusNone,
                                       lastRc: GuidancePointRc.none.rawValue,
                                       lat: 0, lon: 0, altM: 0))
        XCTAssertEqual(flow.verdict, .waiting)
        XCTAssertNil(flow.provisionalFailure)
        flow.onEcho(echo(seq: 4))
        XCTAssertEqual(flow.verdict, .confirmed)
    }

    func testFlowGenuineRejectionSettlesAtTimeoutWithItsReason() {
        // A real rejection is indistinguishable from an interleaved stale-rc
        // frame at arrival time, so it settles at the timeout — with the
        // rc-mapped reason, not the generic silence message.
        var flow = GuidanceSendFlow(baselineSeq: 3, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(echo(seq: 4, st: 0, rc: GuidancePointRc.rejectedRadius.rawValue))
        XCTAssertEqual(flow.verdict, .waiting)
        flow.onTimeout()
        XCTAssertEqual(flow.verdict,
                       .failed(GuidancePointRc.rejectedRadius.message))
    }

    func testFlowDuplicateBaselineFrameStaysWaiting_ConfirmIsTerminal() {
        var flow = GuidanceSendFlow(baselineSeq: 5, sentLat: sentLat, sentLon: sentLon)
        flow.onEcho(echo(seq: 5))                       // re-push of baseline
        XCTAssertEqual(flow.verdict, .waiting)
        flow.onEcho(echo(seq: 6))
        XCTAssertEqual(flow.verdict, .confirmed)
        // Terminal: a later rejection frame or timeout can't un-confirm.
        flow.onEcho(echo(seq: 7, st: 0, rc: GuidancePointRc.rejectedState.rawValue))
        flow.onTimeout()
        XCTAssertEqual(flow.verdict, .confirmed)
    }

    func testRcMessagesCoverAllCodes() {
        for rc in 0...6 {
            XCTAssertNotNil(GuidancePointRc(rawValue: rc), "rc=\(rc)")
        }
        // Unknown codes must still produce a readable failure string.
        XCTAssertTrue(GuidancePointRc.message(forRawValue: 99).contains("99"))
    }

    // MARK: - (c) FlightSettingsData v6 tail

    /// 219-byte v6 frame: version byte at offset 4, guidance-target tail
    /// {f32 e, f32 n, u8 src} at fixed offset 210.
    private func v6Frame(e: Float, n: Float, src: UInt8) -> Data {
        var bytes = [UInt8](repeating: 0, count: 219)
        bytes[4] = 6   // version
        withUnsafeBytes(of: e.bitPattern.littleEndian) { bytes.replaceSubrange(210..<214, with: $0) }
        withUnsafeBytes(of: n.bitPattern.littleEndian) { bytes.replaceSubrange(214..<218, with: $0) }
        bytes[218] = src
        return Data(bytes)
    }

    func testFlightSettingsV6TailParses() throws {
        let fs = try FlightSettingsData(from: v6Frame(e: 12.5, n: -9.25, src: 1))
        XCTAssertEqual(fs.guid_tgt_e_m, 12.5)
        XCTAssertEqual(fs.guid_tgt_n_m, -9.25)
        XCTAssertEqual(fs.guid_tgt_src, 1)
    }

    func testFlightSettingsV5FrameParsesWithNilTail() throws {
        // Pre-#435 (v5, 210 B) frames must keep parsing, tail nil.
        var bytes = [UInt8](repeating: 0, count: 210)
        bytes[4] = 5
        let fs = try FlightSettingsData(from: Data(bytes))
        XCTAssertNil(fs.guid_tgt_e_m)
        XCTAssertNil(fs.guid_tgt_n_m)
        XCTAssertNil(fs.guid_tgt_src)
        XCTAssertEqual(fs.version, 5)
    }

    func testFlightSettingsV6VersionButShortFrameStaysNil() throws {
        // Defensive: version claims v6 but the frame is truncated at 210 —
        // don't read past the buffer, report the tail as absent.
        var bytes = [UInt8](repeating: 0, count: 210)
        bytes[4] = 6
        let fs = try FlightSettingsData(from: Data(bytes))
        XCTAssertNil(fs.guid_tgt_e_m)
        XCTAssertNil(fs.guid_tgt_src)
    }
}
