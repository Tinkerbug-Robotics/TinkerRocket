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
        val ota = OtaSession(scope = backgroundScope, sessionLookup = { session })
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

    // ── Chunk sizing ─────────────────────────────────────────────────────

    @Test
    fun chunkSize_isMtuMinusAttAndHeader_withFloor() {
        assertEquals(507, Commands.otaMaxChunkSize(517))
        assertEquals(502, Commands.otaMaxChunkSize(512))
        assertEquals(20, Commands.otaMaxChunkSize(23), "BLE default MTU floors at 20 B")
        assertEquals(20, Commands.otaMaxChunkSize(0), "never returns a non-positive chunk")
    }
}
