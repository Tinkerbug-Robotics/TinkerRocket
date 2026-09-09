package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.LeBuffer
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import java.security.MessageDigest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * OTA state machine against a scripted `ota_status` stream (the Phase 8
 * exit criterion).  The firmware side is FakeFirmware: the test drives
 * every transition by hand, so each timeout, the rollback branch and the
 * survives-a-reconnect property are all pinned without a radio.
 */
class OtaSessionTest {

    private class Rig(
        val fw: FakeFirmware,
        val ota: OtaSession,
        val sessionRef: () -> DeviceSession?,
        val swapSession: (DeviceSession?) -> Unit,
    )

    private fun TestScope.rig(
        identityJson: String? =
            """{"type":"config_identity","uid":"oc1","un":"Rocket","nid":5,"rid":1,"dt":"R","fw":"v1-old"}""",
    ): Rig {
        val fw = FakeFirmware(backgroundScope).apply { configIdentityJson = identityJson }
        var session: DeviceSession? = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-R-Rocket",
            clock = { currentTime },
        )
        session!!.start()
        runCurrent()
        val ota = OtaSession(
            scope = backgroundScope,
            sessionLookup = { session },
            // Tie the #627 pacer's clock to virtual time, so a paced pump
            // is deterministic instead of reading ~0 elapsed forever.
            nanoTime = { currentTime * 1_000_000L },
        )
        return Rig(fw, ota, { session }, { session = it })
    }

    /** Deterministic image; 64 B is the firmware-plausibility floor. */
    private fun image(n: Int) = ByteArray(n) { (it % 251).toByte() }

    // ── Happy path ───────────────────────────────────────────────────────

    @Test
    fun fullFlow_beginChunksFinishRebootReconnect_verifies() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()          // identity readback lands
        val img = image(1500)

        r.ota.start(img)
        runCurrent()
        assertIs<OtaSession.State.Loading>(r.ota.state.value)

        // OTA_BEGIN went out with the right size + SHA.
        advanceTimeBy(100); runCurrent()
        val begin = r.fw.commandFrames.last { it[0].toInt() == BleCommandId.OTA_BEGIN }
        val b = LeBuffer(begin.copyOfRange(1, begin.size))
        assertEquals(0, b.u8(), "target byte: 0 = this device")
        assertEquals(1500L, b.u32())
        assertContentEquals(MessageDigest.getInstance("SHA-256").digest(img), b.bytes(32))

        // Firmware accepts; the pump runs.
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()

        // MTU 517 → chunk payload 517-3-7 = 507 → 1500 B = 3 chunks.
        assertEquals(3, r.fw.otaChunks.size)
        val offsets = r.fw.otaChunks.map { LeBuffer(it).u32() }
        assertEquals(listOf(0L, 507L, 1014L), offsets)
        // Last chunk carries isLast=1 and the tail length; others 0.
        val last = LeBuffer(r.fw.otaChunks.last())
        assertEquals(1014L, last.u32())
        assertEquals(486, last.u16())
        assertEquals(1, last.u8(), "isLast on the final chunk")
        assertEquals(0, LeBuffer(r.fw.otaChunks[0]).also { it.u32(); it.u16() }.u8())
        // Reassembled chunk payloads are the original image, in order.
        val rebuilt = r.fw.otaChunks.fold(ByteArray(0)) { acc, f ->
            acc + f.copyOfRange(Commands.OTA_CHUNK_HEADER_BYTES, f.size)
        }
        assertContentEquals(img, rebuilt)
        // Pump asked for a tighter interval and gave it back.
        assertEquals(listOf("high", "release"), r.fw.priorityCalls)

        // OTA_FINISH → ready_to_boot.
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)
        assertTrue(r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_FINISH })
        r.fw.emitOtaStatus("ready_to_boot", fw = "v2-new")
        advanceTimeBy(200); runCurrent()
        assertIs<OtaSession.State.Rebooting>(r.ota.state.value)

        // Device drops, the fleet destroys the session, then rebuilds it —
        // the OTA flow must survive that swap (it never captured a session).
        r.swapSession(null)
        advanceTimeBy(300); runCurrent()
        val fresh = FakeFirmware(backgroundScope).apply {
            configIdentityJson =
                """{"type":"config_identity","uid":"oc1","un":"Rocket","nid":5,"rid":1,"dt":"R","fw":"v2-new"}"""
        }
        val newSession = DeviceSession(
            scope = backgroundScope, transport = fresh,
            connectedDeviceName = "TR-R-Rocket", clock = { currentTime },
        )
        newSession.start()
        r.swapSession(newSession)
        advanceTimeBy(1_500); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Verified>(st)
        assertEquals("v2-new", st.newVersion)
    }

    // ── Failure branches ─────────────────────────────────────────────────

    @Test
    fun rollback_sameVersionBack_reportsRollbackNotSuccess() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        r.fw.emitOtaStatus("ready_to_boot")
        advanceTimeBy(200); runCurrent()

        // Comes back on the OLD version — the bootloader rolled us back.
        r.swapSession(null)
        advanceTimeBy(300); runCurrent()
        val fresh = FakeFirmware(backgroundScope).apply {
            configIdentityJson =
                """{"type":"config_identity","uid":"oc1","un":"Rocket","nid":5,"rid":1,"dt":"R","fw":"v1-old"}"""
        }
        val back = DeviceSession(
            scope = backgroundScope, transport = fresh,
            connectedDeviceName = "TR-R-Rocket", clock = { currentTime },
        )
        back.start()
        r.swapSession(back)
        advanceTimeBy(OtaSession.FW_TIMEOUT_MS + 1_500); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.RollbackDetected>(st)
        assertEquals("v1-old", st.version)
    }

    @Test
    fun verifyFailedDuringPump_abortsAndReportsFirmwareError() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        // 2 ms/write so status notifications interleave with the pump as on
        // a real link; the firmware rejects the 3rd chunk mid-flight.
        r.fw.chunkWriteDelayMs = 2
        r.fw.failOtaAtChunk = 3
        r.ota.start(image(200_000))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("bad_offset" in st.reason, "surfaces the firmware token: ${st.reason}")
        assertTrue(r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_ABORT })
        assertTrue("release" in r.fw.priorityCalls, "priority released on the failure path")
    }

    @Test
    fun beginNotAccepted_timesOutWithoutPumping() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(OtaSession.BEGIN_TIMEOUT_MS + 500); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("OTA_BEGIN" in st.reason)
        assertEquals(0, r.fw.otaChunks.size, "no chunks before the firmware says ready")
        assertTrue(
            r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_ABORT },
            "a begin the device never answered is aborted, so a session it may have opened is closed (#1049)",
        )
    }

    @Test
    fun beginRefused_reportsTheFirmwareTokenNotTheTimeout() = runTest {
        // #1106: the OC refuses to flash its own image while the FC reports
        // INFLIGHT, answering OTA_BEGIN with verify_failed/inflight_refused
        // (bad_payload and bad_target take the same path).  The begin wait
        // fails fast on that status, so the message must carry the token —
        // the timeout wording would hide the one thing the firmware said.
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("verify_failed", err = "inflight_refused")
        // 200 ms, not the 5 s window: a refusal is a verdict, not a timeout.
        advanceTimeBy(200); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertEquals("Device refused OTA_BEGIN: inflight_refused", st.reason)
        assertEquals(0, r.fw.otaChunks.size, "no chunks after a refused begin")
        assertTrue(
            r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_ABORT },
            "every failure exit after OTA_BEGIN aborts (#1049)",
        )
    }

    // ── #1049: the cached status must not outlive the run ────────────────

    @Test
    fun retryAfterFinishVerifyFailed_beginReadsOnlyAFreshStatus() = runTest {
        // The cached ota_status lived for the whole connection and nothing
        // cleared it.  After a finish-stage verify_failed the next run's
        // begin wait read that stale value on its first poll and failed in
        // 0 ms with "did not accept OTA_BEGIN within 5s" — for a begin the
        // firmware was in fact accepting.
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)
        val abortsBefore = r.fw.commandFrames.count { it[0].toInt() == BleCommandId.OTA_ABORT }
        r.fw.emitOtaStatus("verify_failed", err = "sha_mismatch")
        advanceTimeBy(200); runCurrent()
        val failed = r.ota.state.value
        assertIs<OtaSession.State.Failed>(failed)
        // #1094: the byte counts iOS has always printed.
        assertEquals("Verify failed: sha_mismatch — device took 0 of 600 B", failed.reason)
        assertEquals(
            abortsBefore + 1,
            r.fw.commandFrames.count { it[0].toInt() == BleCommandId.OTA_ABORT },
            "a finish-stage failure tells the device the session is over",
        )

        // "Flash another firmware".  The fake never answers the abort, so the
        // stale verify_failed is still the last status this session saw.
        r.ota.reset()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        assertNull(r.sessionRef()?.otaStatus?.value, "cache forgotten at OTA_BEGIN")
        assertIs<OtaSession.State.Loading>(r.ota.state.value, "still waiting, not failed in 0 ms")
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value, "the second run pumped and finished")
    }

    @Test
    fun retryAfterBeginRefusal_beginReadsOnlyAFreshStatus() = runTest {
        // The same trap from the other refusal: a rocket that refused in
        // flight (#1106) and is retried on the same connection after landing
        // must wait for the new verdict, not replay the old one.
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("verify_failed", err = "inflight_refused")
        advanceTimeBy(200); runCurrent()
        assertIs<OtaSession.State.Failed>(r.ota.state.value)

        r.ota.reset()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        assertIs<OtaSession.State.Loading>(r.ota.state.value, "still waiting, not failed in 0 ms")
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)
    }

    @Test
    fun finishNeverAcked_failsAfterFinishTimeout() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)
        advanceTimeBy(OtaSession.FINISH_TIMEOUT_MS + 500); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("finalize" in st.reason)
        assertTrue(
            r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_ABORT },
            "a finish the device never answered is aborted (#1049)",
        )
    }

    @Test
    fun fcFinishWindowOutlastsTheLocalOne() = runTest {
        // Bench 2026-07-28: a real 591.7 kB FC flash was still running when the
        // 15 s local window expired — the FC finished, rebooted and ran the new
        // image, but the app had already called it a failure.  Finish gets the
        // same FC-path stretch that begin and fw-publish already had.
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600), targetIsFc = true)
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)

        advanceTimeBy(OtaSession.FINISH_TIMEOUT_MS + 500); runCurrent()
        assertIs<OtaSession.State.Verifying>(
            r.ota.state.value,
            "the local finish window must not cut off a relayed flash",
        )

        advanceTimeBy(OtaSession.FINISH_TIMEOUT_FC_MS); runCurrent()
        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("60s" in st.reason, "reports the FC window it actually waited: ${st.reason}")
    }

    @Test
    fun neverReconnects_failsWithPowerCycleHint() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        r.fw.emitOtaStatus("ready_to_boot")
        advanceTimeBy(200); runCurrent()
        r.swapSession(null)             // gone, and never comes back
        advanceTimeBy(OtaSession.RECONNECT_TIMEOUT_MS + 6_000); runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("power-cycling" in st.reason, st.reason)
    }

    @Test
    fun tooSmallImage_rejectedBeforeAnyWrite() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        val before = r.fw.commandFrames.size
        r.ota.start(image(32))
        runCurrent()

        val st = r.ota.state.value
        assertIs<OtaSession.State.Failed>(st)
        assertTrue("too small" in st.reason)
        assertEquals(before, r.fw.commandFrames.size, "nothing sent for a bogus file")
    }

    @Test
    fun cancel_sendsAbort() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(200_000))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        runCurrent()
        r.ota.cancel()
        runCurrent()

        assertIs<OtaSession.State.Failed>(r.ota.state.value)
        assertTrue(r.fw.commandFrames.any { it[0].toInt() == BleCommandId.OTA_ABORT })
    }

    // ── Cancel stays "Cancelled" in every phase ──────────────────────────
    // job.cancel() throws a CancellationException out of every delay(), which
    // unwinds runFlow before it can write state; these pin that, because the
    // iOS twin had to add explicit early returns to get the same (its waits
    // caught the cancellation as a timeout, and a cancel during the reboot
    // phase fell through to a spurious rollback verdict).

    private fun abortCount(r: Rig) = r.fw.commandFrames.count { it[0].toInt() == BleCommandId.OTA_ABORT }

    @Test
    fun cancelDuringBeginWait_staysCancelled() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.ota.cancel()
        runCurrent()
        assertEquals(OtaSession.State.Failed("Cancelled"), r.ota.state.value)

        advanceTimeBy(OtaSession.BEGIN_TIMEOUT_MS + 500); runCurrent()
        assertEquals(OtaSession.State.Failed("Cancelled"), r.ota.state.value, "the begin timeout must not overwrite the cancel")
        assertEquals(1, abortCount(r), "one abort, from cancel()")
        assertEquals(0, r.fw.otaChunks.size)
    }

    @Test
    fun cancelDuringFinishWait_staysCancelled() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        assertIs<OtaSession.State.Verifying>(r.ota.state.value)
        r.ota.cancel()
        runCurrent()
        assertEquals(OtaSession.State.Failed("Cancelled"), r.ota.state.value)

        advanceTimeBy(OtaSession.FINISH_TIMEOUT_MS + 500); runCurrent()
        assertEquals(OtaSession.State.Failed("Cancelled"), r.ota.state.value, "the finish timeout must not overwrite the cancel")
        assertEquals(1, abortCount(r), "one abort, from cancel()")
    }

    @Test
    fun cancelWhileRebooting_staysCancelled() = runTest {
        val r = rig()
        advanceTimeBy(1_200); runCurrent()
        r.ota.start(image(600))
        advanceTimeBy(100); runCurrent()
        r.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        r.fw.emitOtaStatus("ready_to_boot")
        advanceTimeBy(200); runCurrent()
        assertIs<OtaSession.State.Rebooting>(r.ota.state.value)
        r.ota.cancel()
        runCurrent()

        // Link still up, firmware unchanged: the path that read as a rollback.
        advanceTimeBy(OtaSession.RECONNECT_TIMEOUT_MS + OtaSession.FW_TIMEOUT_MS + 6_000); runCurrent()
        assertEquals(OtaSession.State.Failed("Cancelled"), r.ota.state.value, "neither the reconnect timeout nor a rollback verdict may overwrite the cancel")
        assertEquals(1, abortCount(r))
    }

    // ── FC-target specifics (#8 P4) ──────────────────────────────────────

    @Test
    fun fcTarget_setsTargetByteAndComparesTheFcVersion() = runTest {
        // OC reports its own fw AND the FC's; an FC OTA must watch the latter.
        val r = rig(
            identityJson =
                """{"type":"config_identity","uid":"oc1","un":"Rocket","nid":5,"rid":1,"dt":"R","fw":"oc-1"}""",
        )
        advanceTimeBy(1_200); runCurrent()
        r.fw.emitTelemetryJson("""{"type":"fc_identity","fc_fw":"fc-old"}""")
        advanceTimeBy(100); runCurrent()
        assertEquals("fc-old", r.sessionRef()?.identity?.value?.fcFirmwareVersion)

        r.ota.start(image(600), targetIsFc = true)
        advanceTimeBy(100); runCurrent()
        val begin = r.fw.commandFrames.last { it[0].toInt() == BleCommandId.OTA_BEGIN }
        assertEquals(1, begin[1].toInt(), "target byte 1 = relay to the FC")
        assertEquals("fc-old", r.ota.preFlashVersion, "pre-flash version read from the FC, not the OC")

        // The FC begin window is the long one — still waiting at the local timeout.
        advanceTimeBy(OtaSession.BEGIN_TIMEOUT_MS + 500); runCurrent()
        assertIs<OtaSession.State.Loading>(r.ota.state.value.let { if (it is OtaSession.State.Loading) it else it })
    }

    // ── #627 relay pacing ────────────────────────────────────────────────

    @Test
    fun fcRelayPumpIsPacedButTheLocalPumpIsNot() = runTest {
        // 60 kB at the 12 kB/s relay cap = ~5 s of pacing. The local path has
        // no relay to overrun, so it must stay uncapped — a cap there would
        // turn the bench-proven ~68 kB/s OC flash into a 60-second crawl.
        val bytes = 60_000

        val local = rig()
        advanceTimeBy(1_200); runCurrent()
        val localStart = currentTime
        local.ota.start(image(bytes), targetIsFc = false)
        advanceTimeBy(100); runCurrent()
        local.fw.emitOtaStatus("ready")
        advanceTimeBy(500); runCurrent()
        val localElapsed = currentTime - localStart
        assertIs<OtaSession.State.Verifying>(
            local.ota.state.value,
            "local pump should have run straight through",
        )
        assertTrue(
            localElapsed < 2_000,
            "local pump must not be throttled (took ${localElapsed}ms)",
        )

        val relay = rig()
        advanceTimeBy(1_200); runCurrent()
        val relayStart = currentTime
        relay.ota.start(image(bytes), targetIsFc = true)
        advanceTimeBy(100); runCurrent()
        relay.fw.emitOtaStatus("ready")
        // Well past the local pump's duration, the relay pump is still going.
        advanceTimeBy(2_000); runCurrent()
        assertIs<OtaSession.State.Uploading>(
            relay.ota.state.value,
            "relay pump should still be throttled here, not finished",
        )

        advanceTimeBy(10_000); runCurrent()
        assertIs<OtaSession.State.Verifying>(relay.ota.state.value)
        val relayElapsed = currentTime - relayStart
        val impliedRate = bytes * 1000L / relayElapsed
        assertTrue(
            impliedRate <= OtaTimeouts.FC_RELAY_MAX_BYTES_PER_SEC,
            "relay pump ran at ${impliedRate} B/s, over the " +
                "${OtaTimeouts.FC_RELAY_MAX_BYTES_PER_SEC} B/s cap that keeps " +
                "the OC's mbuf pool alive (#627)",
        )
    }

    // ── Chunk sizing ─────────────────────────────────────────────────────

    @Test
    fun chunkSize_isMtuMinusAttAndHeader_withFloor() {
        assertEquals(507, Commands.otaMaxChunkSize(517))
        assertEquals(502, Commands.otaMaxChunkSize(512))
        assertEquals(20, Commands.otaMaxChunkSize(23), "BLE default MTU floors at 20 B")
        assertEquals(20, Commands.otaMaxChunkSize(0), "never returns a non-positive chunk")
    }
}
