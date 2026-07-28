package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonObject
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #435: Drift-Cast guidance-point upload confirmation — port of the iOS
 * GuidancePointEchoTests (every case).  Covers (a) the "guid_target" JSON
 * frame → GuidanceTargetEcho parse, (b) the pure confirm-match predicate the
 * DriftCast send button gates green on, (b2) the GuidanceSendFlow verdict
 * attribution across frames, and (c) the FlightSettingsData v6 flown-target
 * tail.
 */
class GuidanceSendFlowTest {

    // MARK: - (a) guid_target JSON parse

    private fun decode(json: String): GuidanceTargetEcho? = try {
        GuidanceTargetEcho.fromJson(Json.parseToJsonElement(json).jsonObject)
    } catch (_: Exception) {
        null
    }

    @Test
    fun `guid_target frame parses`() {
        // Byte-for-byte the OC's sendGuidTarget frame shape.
        val echo = decode(
            """{"type":"guid_target","seq":3,"st":1,"rc":1,"lat":38.123456,"lon":-122.123456,"alt":250}""",
        )
        assertNotNull(echo)
        assertEquals(3, echo.seq)
        assertEquals(GuidanceTargetEcho.STATUS_GEO_ACTIVE, echo.status)
        assertEquals(GuidancePointRc.ACCEPTED.raw, echo.lastRc)
        assertEquals(38.123456, echo.lat, 1e-9)
        assertEquals(-122.123456, echo.lon, 1e-9)
        assertEquals(250, echo.altM)
    }

    @Test
    fun `guid_target rejects other frame types`() {
        assertNull(decode("""{"type":"imu_orient","mode":2}"""))
        assertNull(decode("""{"type":"config","ge":true}"""))
    }

    @Test
    fun `guid_target missing fields default defensively`() {
        // MTU-budget rule (#282): parse must survive a trimmed frame.
        val echo = decode("""{"type":"guid_target","seq":1}""")
        assertNotNull(echo)
        assertEquals(GuidanceTargetEcho.STATUS_NONE, echo.status)
        assertEquals(GuidancePointRc.NONE.raw, echo.lastRc)
        assertEquals(0.0, echo.lat)
        assertEquals(0, echo.altM)
    }

    // MARK: - (b) confirm-match predicate

    private val sentLat = 38.1234567
    private val sentLon = -122.7654321

    private fun echo(
        seq: Int = 1,
        st: Int = 1,
        rc: Int = 1,
        lat: Double? = null,
        lon: Double? = null,
    ): GuidanceTargetEcho = GuidanceTargetEcho(
        seq = seq, status = st, lastRc = rc,
        lat = lat ?: sentLat, lon = lon ?: sentLon, altM = 120,
    )

    @Test
    fun `outcome pending when seq unchanged`() {
        // A connect-time re-push of the SAME echo must not confirm anything.
        val e = echo(seq = 5)
        assertEquals(GuidanceSendOutcome.Pending, e.outcome(5, sentLat, sentLon))
    }

    @Test
    fun `outcome confirmed exact match`() {
        val e = echo(seq = 6)
        assertEquals(GuidanceSendOutcome.Confirmed, e.outcome(5, sentLat, sentLon))
    }

    @Test
    fun `outcome never settles without a baseline`() {
        // Baseline -1 = no echo received before the send: NOTHING is
        // attributable, so even a perfect-looking accepted/GEO/matching
        // frame must stay pending — it is typically the OC's stale
        // connect-time push of a PREVIOUS session's state (the false-green
        // reconnect path; GuidanceSendFlow adopts it as the baseline).
        val e = echo(seq = 0)
        assertEquals(GuidanceSendOutcome.Pending, e.outcome(-1, sentLat, sentLon))
        // Same for a stale frame with a bigger seq and for the FC boot frame.
        assertEquals(GuidanceSendOutcome.Pending, echo(seq = 4).outcome(-1, sentLat, sentLon))
        assertEquals(
            GuidanceSendOutcome.Pending,
            echo(seq = 0, st = 0, rc = 0).outcome(-1, sentLat, sentLon),
        )
    }

    @Test
    fun `outcome rc NONE is never a verdict`() {
        // The FC writes a nonzero GUID_RC_* on every processed cmd 28, and a
        // cmd-65 apply bumps seq WITHOUT touching rc — so a seq-advanced
        // frame with rc == NONE (e.g. the boot-state echo pushed at connect,
        // or a config wipe before any upload) means "no cmd-28 result yet",
        // NOT "rejected". Mapping it to Rejected showed a false
        // "No upload has been processed" failure moments before the real
        // accept arrived.
        val boot = echo(seq = 0, st = 0, rc = GuidancePointRc.NONE.raw)
        assertEquals(GuidanceSendOutcome.Pending, boot.outcome(5, sentLat, sentLon))
        val wipe = echo(seq = 3, st = GuidanceTargetEcho.STATUS_EN_ACTIVE, rc = GuidancePointRc.NONE.raw)
        assertEquals(GuidanceSendOutcome.Pending, wipe.outcome(2, sentLat, sentLon))
    }

    @Test
    fun `outcome confirmed survives f32 rounding`() {
        // The echo carries f32-rounded degrees (ulp ~4e-6 at these
        // magnitudes) — the tolerances must absorb that loss.
        val e = echo(
            seq = 2,
            lat = sentLat.toFloat().toDouble(),
            lon = sentLon.toFloat().toDouble(),
        )
        assertEquals(GuidanceSendOutcome.Confirmed, e.outcome(1, sentLat, sentLon))
    }

    @Test
    fun `outcome rejected maps rc`() {
        for (rc in listOf(2, 3, 4, 5, 6)) {
            val e = echo(seq = 2, st = 0, rc = rc)
            assertEquals(
                GuidanceSendOutcome.Rejected(rc), e.outcome(1, sentLat, sentLon), "rc=$rc",
            )
        }
    }

    @Test
    fun `outcome rejection keeps previous target visible`() {
        // status/lastRc are separate fields exactly so a rejected upload can
        // ride alongside a still-active previous target: st stays GEO_ACTIVE
        // with the OLD coordinates, rc reports the failure. Rejection must
        // win over the coordinate match.
        val e = GuidanceTargetEcho(
            seq = 7, status = GuidanceTargetEcho.STATUS_GEO_ACTIVE,
            lastRc = GuidancePointRc.REJECTED_RADIUS.raw,
            lat = 38.0, lon = -122.0, altM = 100,
        )
        assertEquals(GuidanceSendOutcome.Rejected(2), e.outcome(6, sentLat, sentLon))
    }

    @Test
    fun `outcome mismatch on different coordinates`() {
        val e = echo(seq = 2, lat = sentLat + 1e-3)   // ~110 m off
        assertEquals(GuidanceSendOutcome.Mismatch, e.outcome(1, sentLat, sentLon))
    }

    @Test
    fun `outcome mismatch just outside tolerance`() {
        val eLat = echo(seq = 2, lat = sentLat + 2.1e-5)
        assertEquals(GuidanceSendOutcome.Mismatch, eLat.outcome(1, sentLat, sentLon))
        val eLon = echo(seq = 2, lon = sentLon + 3.1e-5)
        assertEquals(GuidanceSendOutcome.Mismatch, eLon.outcome(1, sentLat, sentLon))
    }

    @Test
    fun `outcome mismatch when status not GEO_ACTIVE`() {
        // rc=ACCEPTED but the active target is not the geodetic point (a
        // cmd-65 apply raced in between) — never green.
        val e = echo(seq = 2, st = GuidanceTargetEcho.STATUS_EN_ACTIVE)
        assertEquals(GuidanceSendOutcome.Mismatch, e.outcome(1, sentLat, sentLon))
    }

    // MARK: - (b2) GuidanceSendFlow — verdict attribution across frames

    /**
     * The stale frame the OC re-pushes at connect time: a PREVIOUS upload's
     * accepted state, coordinates identical to what the user re-sends.
     */
    private val staleConnectPush: GuidanceTargetEcho get() = echo(seq = 4)

    @Test
    fun `flow reconnect stale frame cannot confirm - real verdict wins`() {
        // Finding scenario A: reconnect cleared the echo, user re-sends the
        // same point with baseline -1, the stale connect push (seq 4, rc
        // ACCEPTED, GEO, matching coords) arrives first, then the FC's REAL
        // verdict — a post-flight REJ_STATE. The old machine went green on
        // the stale frame and swallowed the rejection.
        val flow = GuidanceSendFlow(baselineSeq = -1, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(staleConnectPush)
        assertEquals(GuidanceSendFlow.Verdict.Waiting, flow.verdict)   // adopted as baseline
        assertEquals(4, flow.baselineSeq)
        flow.onEcho(echo(seq = 5, rc = GuidancePointRc.REJECTED_STATE.raw))
        flow.onTimeout()
        assertEquals(
            GuidanceSendFlow.Verdict.Failed(GuidancePointRc.REJECTED_STATE.message),
            flow.verdict,
        )
    }

    @Test
    fun `flow nothing sent nothing arrives - times out instead of green`() {
        // Finding scenario C shape: only the stale connect push ever arrives
        // (e.g. the write never reached the rocket). Must end in the timeout
        // failure, never confirmed.
        val flow = GuidanceSendFlow(baselineSeq = -1, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(staleConnectPush)
        flow.onTimeout()
        assertEquals(
            GuidanceSendFlow.Verdict.Failed(GuidanceSendFlow.TIMEOUT_MESSAGE), flow.verdict,
        )
    }

    @Test
    fun `flow interleaved config wipe does not settle - accept upgrades`() {
        // Finding scenario D: a cmd-65 apply queued ahead of cmd 28 wipes a
        // previously active geo point — its frame bumps seq with a STALE
        // rc=ACCEPTED, status EN, zeroed coords. The old machine latched a
        // terminal mismatch failure and ignored the genuine accept one poll
        // later.
        val flow = GuidanceSendFlow(baselineSeq = 7, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(
            GuidanceTargetEcho(
                seq = 8, status = GuidanceTargetEcho.STATUS_EN_ACTIVE,
                lastRc = GuidancePointRc.ACCEPTED.raw,
                lat = 0.0, lon = 0.0, altM = 0,
            ),
        )
        assertEquals(GuidanceSendFlow.Verdict.Waiting, flow.verdict)   // provisional only
        assertNotNull(flow.provisionalFailure)
        flow.onEcho(echo(seq = 9))                                     // the genuine accept
        assertEquals(GuidanceSendFlow.Verdict.Confirmed, flow.verdict)
    }

    @Test
    fun `flow interleaved wipe with rc NONE stays pending`() {
        // Finding scenario D': no cmd-28 processed this boot, so the wipe
        // frame carries rc = NONE — not even a provisional failure.
        val flow = GuidanceSendFlow(baselineSeq = 2, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(
            GuidanceTargetEcho(
                seq = 3, status = GuidanceTargetEcho.STATUS_NONE,
                lastRc = GuidancePointRc.NONE.raw,
                lat = 0.0, lon = 0.0, altM = 0,
            ),
        )
        assertEquals(GuidanceSendFlow.Verdict.Waiting, flow.verdict)
        assertNull(flow.provisionalFailure)
        flow.onEcho(echo(seq = 4))
        assertEquals(GuidanceSendFlow.Verdict.Confirmed, flow.verdict)
    }

    @Test
    fun `flow genuine rejection settles at timeout with its reason`() {
        // A real rejection is indistinguishable from an interleaved stale-rc
        // frame at arrival time, so it settles at the timeout — with the
        // rc-mapped reason, not the generic silence message.
        val flow = GuidanceSendFlow(baselineSeq = 3, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(echo(seq = 4, st = 0, rc = GuidancePointRc.REJECTED_RADIUS.raw))
        assertEquals(GuidanceSendFlow.Verdict.Waiting, flow.verdict)
        flow.onTimeout()
        assertEquals(
            GuidanceSendFlow.Verdict.Failed(GuidancePointRc.REJECTED_RADIUS.message),
            flow.verdict,
        )
    }

    @Test
    fun `flow duplicate baseline frame stays waiting - confirm is terminal`() {
        val flow = GuidanceSendFlow(baselineSeq = 5, sentLat = sentLat, sentLon = sentLon)
        flow.onEcho(echo(seq = 5))                                     // re-push of baseline
        assertEquals(GuidanceSendFlow.Verdict.Waiting, flow.verdict)
        flow.onEcho(echo(seq = 6))
        assertEquals(GuidanceSendFlow.Verdict.Confirmed, flow.verdict)
        // Terminal: a later rejection frame or timeout can't un-confirm.
        flow.onEcho(echo(seq = 7, st = 0, rc = GuidancePointRc.REJECTED_STATE.raw))
        flow.onTimeout()
        assertEquals(GuidanceSendFlow.Verdict.Confirmed, flow.verdict)
    }

    @Test
    fun `rc messages cover all codes`() {
        for (rc in 0..6) {
            assertNotNull(GuidancePointRc.fromRaw(rc), "rc=$rc")
        }
        // Unknown codes must still produce a readable failure string.
        assertTrue(GuidancePointRc.messageForRaw(99).contains("99"))
    }

    // MARK: - (c) FlightSettingsData v6 tail

    /**
     * 219-byte v6 frame: version byte at offset 4, guidance-target tail
     * {f32 e, f32 n, u8 src} at fixed offset 210.
     */
    private fun v6Frame(e: Float, n: Float, src: Int): ByteArray {
        val bytes = ByteArray(219)
        bytes[4] = 6   // version
        putF32Le(bytes, 210, e)
        putF32Le(bytes, 214, n)
        bytes[218] = src.toByte()
        return bytes
    }

    private fun putF32Le(bytes: ByteArray, offset: Int, v: Float) {
        val bits = v.toRawBits()
        bytes[offset] = (bits and 0xFF).toByte()
        bytes[offset + 1] = ((bits shr 8) and 0xFF).toByte()
        bytes[offset + 2] = ((bits shr 16) and 0xFF).toByte()
        bytes[offset + 3] = ((bits shr 24) and 0xFF).toByte()
    }

    @Test
    fun `flight settings v6 tail parses`() {
        val fs = FlightSettingsData.decode(v6Frame(e = 12.5f, n = -9.25f, src = 1))
        assertNotNull(fs)
        assertEquals(12.5f, fs.guidTgtEM)
        assertEquals(-9.25f, fs.guidTgtNM)
        assertEquals(1, fs.guidTgtSrc)
    }

    @Test
    fun `flight settings v5 frame parses with null tail`() {
        // Pre-#435 (v5, 210 B) frames must keep parsing, tail null.
        val bytes = ByteArray(210)
        bytes[4] = 5
        val fs = FlightSettingsData.decode(bytes)
        assertNotNull(fs)
        assertNull(fs.guidTgtEM)
        assertNull(fs.guidTgtNM)
        assertNull(fs.guidTgtSrc)
        assertEquals(5, fs.version)
    }

    @Test
    fun `flight settings v6 version but short frame stays null`() {
        // Defensive: version claims v6 but the frame is truncated at 210 —
        // don't read past the buffer, report the tail as absent.
        val bytes = ByteArray(210)
        bytes[4] = 6
        val fs = FlightSettingsData.decode(bytes)
        assertNotNull(fs)
        assertNull(fs.guidTgtEM)
        assertNull(fs.guidTgtSrc)
    }
}
