package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.PyroContinuity
import com.tinkerbug.tinkerrocket.session.PyroTestFire.FireBlock
import com.tinkerbug.tinkerrocket.session.PyroTestFire.Phase
import com.tinkerbug.tinkerrocket.session.PyroTestFire.Verdict
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1650: the test-fire flow ([PyroTestFire]) on both links: the countdown,
 * logging from T-5 only when it was off, the fire at T-0 behind every iOS
 * guard re-checked at that instant, SAFE, the relayed re-test, and the
 * stand-back verdict.  Over real DeviceSessions fed by FakeFirmware.
 */
class PyroTestFireTest {

    private fun TestScope.link(identityJson: String): Pair<DeviceSession, FakeFirmware> {
        val fw = FakeFirmware(backgroundScope).apply {
            configJson = null
            configPyroJson = null
            configIdentityJson = identityJson
            pushEchoWithConfig = false
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-test",
            clock = { currentTime },
        )
        session.start()
        runCurrent()
        advanceTimeBy(1100)
        runCurrent()
        return session to fw
    }

    private fun TestScope.directRocket(fs: Int = RAIL_ON): Pair<DeviceSession, FakeFirmware> {
        val (s, fw) = link("""{"type":"config_identity","uid":"r","un":"Atlas","nid":5,"rid":1,"dt":"R"}""")
        fw.emitTelemetryJson("""{"st":"READY","fs":$fs}""")
        runCurrent()
        return s to fw
    }

    private fun TestScope.baseStation(): Pair<DeviceSession, FakeFirmware> =
        link("""{"type":"config_identity","uid":"b","un":"Base","nid":5,"dt":"B"}""")

    private fun FakeFirmware.relay(rid: Int, name: String = "Rocket $rid", extra: String = "") {
        val more = if (extra.isEmpty()) "" else ",$extra"
        emitTelemetryJson("{\"rid\":$rid,\"run\":\"$name\",\"st\":\"READY\"$more}")
    }

    /** Let [ms] pass while [rids] keep transmitting, so the relay stays live. */
    private fun TestScope.relaying(fw: FakeFirmware, ms: Long, vararg rids: Int) {
        var left = ms
        while (left > 0) {
            rids.forEach { fw.relay(it) }
            val step = minOf(500L, left)
            advanceTimeBy(step)
            left -= step
        }
        runCurrent()
    }

    /** Command frames as unsigned ints, oldest first. */
    private fun FakeFirmware.frames(): List<List<Int>> =
        commandFrames.map { f -> f.map { it.toInt() and 0xFF } }

    private fun FakeFirmware.framesStartingWith(cmd: Int) = frames().filter { it.first() == cmd }

    // ── Direct link ──────────────────────────────────────────────────────

    @Test
    fun direct_countsDown_logsFromTMinus5_firesAtZero_stopsTheLogItStarted() = runTest {
        val (session, fw) = directRocket()
        val test = PyroTestFire(session, 2, backgroundScope)
        test.open()
        runCurrent()
        assertEquals(listOf(listOf(35, 2)), fw.framesStartingWith(35), "open tests continuity once")

        test.fire()
        advanceTimeBy(4_500); runCurrent()
        assertTrue(fw.framesStartingWith(23).isEmpty(), "no logging before T-5")
        advanceTimeBy(600); runCurrent()
        assertEquals(listOf(listOf(23, 1)), fw.framesStartingWith(23), "desired-state start at T-5")
        assertTrue(fw.framesStartingWith(36).isEmpty())

        advanceTimeBy(5_000); runCurrent()
        assertEquals(listOf(listOf(36, 2)), fw.framesStartingWith(36), "bare cmd 36 at T-0")
        assertEquals(Phase.AFTER_FIRE, test.state.value.phase)

        advanceTimeBy(5_000); runCurrent()
        assertEquals(listOf(listOf(23, 1), listOf(23, 0)), fw.framesStartingWith(23))
        assertEquals(Phase.DONE, test.state.value.phase)
        assertTrue(test.state.value.fired)
        assertEquals(1, fw.framesStartingWith(35).size, "no re-test on a direct link")
        assertNull(test.verdict(PyroContinuity.OPEN), "the stand-back verdict is the relay's")
    }

    @Test
    fun aLogTheOperatorStarted_isLeftAlone() = runTest {
        val (session, fw) = directRocket(fs = RAIL_ON or LOGGING)
        val test = PyroTestFire(session, 1, backgroundScope)
        test.open()
        test.fire()
        advanceTimeBy(15_100); runCurrent()
        assertEquals(listOf(listOf(36, 1)), fw.framesStartingWith(36))
        assertTrue(fw.framesStartingWith(23).isEmpty(), "never stop a log this test did not start")
    }

    @Test
    fun safe_sendsNothing_andStopsTheLogItStarted() = runTest {
        val (session, fw) = directRocket()
        val test = PyroTestFire(session, 3, backgroundScope)
        test.open()
        test.fire()
        advanceTimeBy(6_100); runCurrent()
        test.safe()
        runCurrent()
        assertEquals(Phase.IDLE, test.state.value.phase)
        advanceTimeBy(20_000); runCurrent()
        assertTrue(fw.framesStartingWith(36).isEmpty())
        assertEquals(listOf(listOf(23, 1), listOf(23, 0)), fw.framesStartingWith(23))
    }

    @Test
    fun railOff_blocksFire_andARailDropBeforeZeroAborts() = runTest {
        val (session, fw) = directRocket(fs = 0)
        val test = PyroTestFire(session, 1, backgroundScope)
        test.open()
        assertEquals(FireBlock.RAIL_OFF, test.fireBlock())
        test.fire()
        assertEquals(Phase.IDLE, test.state.value.phase, "a blocked FIRE does nothing")

        fw.emitTelemetryJson("""{"st":"READY","fs":$RAIL_ON}""")
        runCurrent()
        test.fire()
        advanceTimeBy(8_100); runCurrent()
        fw.emitTelemetryJson("""{"st":"READY","fs":0}""")
        advanceTimeBy(2_000); runCurrent()

        assertTrue(fw.framesStartingWith(36).isEmpty(), "the OC would refuse it; never send it")
        assertEquals(Phase.IDLE, test.state.value.phase)
        assertEquals("Rocket power is off — no fire command sent", test.state.value.error)
    }

    @Test
    fun aDroppedLinkAtZero_sendsNothing() = runTest {
        val (session, fw) = directRocket()
        val test = PyroTestFire(session, 1, backgroundScope)
        test.open()
        test.fire()
        advanceTimeBy(7_100); runCurrent()
        fw.fireDisconnect()
        advanceTimeBy(3_000); runCurrent()
        assertTrue(fw.framesStartingWith(36).isEmpty())
        assertEquals(Phase.DONE, test.state.value.phase)
        assertEquals("Link lost before fire — no fire command sent", test.state.value.error)
    }

    @Test
    fun aFireRefusal_abortsThroughSafe() = runTest {
        val (session, fw) = directRocket()
        val test = PyroTestFire(session, 2, backgroundScope)
        test.open()
        test.fire()
        advanceTimeBy(10_100); runCurrent()
        assertEquals(Phase.AFTER_FIRE, test.state.value.phase)

        fw.emitFileOpsFrame(byteArrayOf(0xCE.toByte(), 36, 2, 1))
        runCurrent()
        assertEquals(Phase.IDLE, test.state.value.phase)
        assertEquals("Rocket refused the fire — power is off", test.state.value.error)
        assertEquals(listOf(23, 0), fw.framesStartingWith(23).last(), "and the test's log stops")
    }

    // ── Base-station link (LoRa) ─────────────────────────────────────────

    @Test
    fun relay_firesTheFollowedRocket_thenTestsContinuityAgain() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1)
        runCurrent()
        val test = PyroTestFire(bs, 3, backgroundScope)
        test.open()
        runCurrent()
        assertEquals(1, test.latchedTargetRid)

        test.fire()
        relaying(fw, 10_100, 1)
        relaying(fw, 5_000, 1)

        val relayed = fw.framesStartingWith(50)
        assertEquals(
            listOf(
                listOf(50, 1, 35, 3),   // open
                listOf(50, 1, 23, 1),   // T-5
                listOf(50, 1, 36, 3),   // T-0
                listOf(50, 1, 23, 0),   // T+5
                listOf(50, 1, 35, 3),   // the stand-back re-test
            ),
            relayed,
        )
        assertEquals(Phase.DONE, test.state.value.phase)
        assertEquals(Verdict.RETESTING, test.verdict(PyroContinuity.OPEN))
    }

    @Test
    fun relay_silentAtZero_sendsNoFire() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1)
        runCurrent()
        val test = PyroTestFire(bs, 1, backgroundScope)
        test.open()
        test.fire()
        advanceTimeBy(10_100); runCurrent()   // the rocket stopped transmitting

        assertTrue(fw.framesStartingWith(50).none { it.getOrNull(2) == 36 })
        assertEquals(Phase.DONE, test.state.value.phase)
        assertEquals("Rocket LoRa link not ready — no fire command sent", test.state.value.error)
        assertEquals(listOf(50, 1, 23, 0), fw.framesStartingWith(50).last(), "the T-5 log is stopped")
    }

    @Test
    fun relay_focusMovedDuringTheCountdown_sendsNoFire() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1)
        fw.relay(2)
        runCurrent()
        val test = PyroTestFire(bs, 1, backgroundScope)
        test.open()
        test.fire()
        relaying(fw, 6_000, 1, 2)
        bs.setFocusRocket(2)
        relaying(fw, 4_100, 1, 2)

        assertTrue(fw.framesStartingWith(50).none { it.getOrNull(2) == 36 }, "neither rocket fires")
        assertEquals("Focused rocket changed — no fire command sent", test.state.value.error)
    }

    @Test
    fun relay_aSharedRocketId_blocksFire() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, extra = "\"dup\":1")
        runCurrent()
        val test = PyroTestFire(bs, 1, backgroundScope)
        test.open()
        assertEquals(FireBlock.ID_SHARED, test.fireBlock())
        test.fire()
        relaying(fw, 12_000, 1)
        assertEquals(Phase.IDLE, test.state.value.phase)
        assertTrue(fw.framesStartingWith(50).none { it.getOrNull(2) == 36 })
    }

    @Test
    fun relay_sendPyroFire_refusesASharedIdOnItsOwn() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, extra = "\"dup\":1")
        runCurrent()
        bs.sendPyroFire(1)
        runCurrent()
        assertTrue(fw.framesStartingWith(50).none { it.getOrNull(2) == 36 })
    }

    // ── The stand-back verdict ───────────────────────────────────────────

    @Test
    fun onlyAMeasuredOpenOnALiveLinkSaysFired() {
        assertEquals(Verdict.FIRED, PyroTestFire.verdictOf(true, PyroContinuity.OPEN))
        assertEquals(Verdict.STILL_PRESENT, PyroTestFire.verdictOf(true, PyroContinuity.PRESENT))
        assertEquals(Verdict.NO_READING, PyroTestFire.verdictOf(true, PyroContinuity.UNTESTED))
        assertEquals(Verdict.NO_READING, PyroTestFire.verdictOf(true, PyroContinuity.NO_DATA))
        // A stale link or a moved focus says nothing, even with an open reading.
        assertEquals(Verdict.LINK_LOST, PyroTestFire.verdictOf(false, PyroContinuity.OPEN))
    }

    @Test
    fun fireBlock_rules() {
        assertEquals(FireBlock.NOT_CONNECTED, PyroTestFire.fireBlockOf(false, false, true, false))
        assertEquals(FireBlock.RAIL_OFF, PyroTestFire.fireBlockOf(true, false, false, false))
        assertNull(PyroTestFire.fireBlockOf(true, true, false, false), "a relay has no rail bit")
        assertEquals(FireBlock.ID_SHARED, PyroTestFire.fireBlockOf(true, true, false, true))
        assertNull(PyroTestFire.fireBlockOf(true, false, true, false))
    }

    private companion object {
        const val RAIL_ON = 0x10
        const val LOGGING = 0x40
    }
}
