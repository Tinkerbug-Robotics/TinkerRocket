package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #1650: three rockets, one base station — what a pyro command reaches.  Two
 * rockets under one rocket ID (the base station's "dup", and the app's own
 * name-alternation check) and the LoRa continuity test it blocks.  Port of
 * the matching iOS MultiRocketSafetyTests cases.
 */
class MultiRocketSafetyTest {

    private fun TestScope.baseStation(): Pair<DeviceSession, FakeFirmware> {
        val fw = FakeFirmware(backgroundScope).apply {
            configJson = null
            configPyroJson = null
            configIdentityJson =
                """{"type":"config_identity","uid":"bs","un":"TR-B-Test","nid":5,"dt":"B"}"""
            pushEchoWithConfig = false
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-B-Test",
            clock = { currentTime },
        )
        session.start()
        runCurrent()
        advanceTimeBy(1100)
        runCurrent()
        return session to fw
    }

    private fun FakeFirmware.relay(rid: Int, name: String? = null, extra: String = "") {
        val run = name?.let { ",\"run\":\"$it\"" } ?: ""
        val more = if (extra.isEmpty()) "" else ",$extra"
        emitTelemetryJson("{\"rid\":$rid$run,\"st\":\"READY\"$more}")
    }

    private fun rocket(name: String) =
        RelayedRocket(rocketId = 1, unitName = name, telemetry = TelemetryData(), lastSeenMs = 0)

    // ── Name alternation, the app's own check ────────────────────────────

    @Test
    fun twoNameChangesInsideTheWindow_flag() {
        var r = rocket("Booster").noteRelayedName("Twin", 0)
        assertFalse(r.duplicateIdSuspected(0), "one change is a rename")
        r = r.noteRelayedName("Booster", 1_000)
        assertTrue(r.duplicateIdSuspected(1_000))
    }

    @Test
    fun aRename_isNotADuplicate() {
        var r = rocket("Old name")
        for (i in 0 until 20) r = r.noteRelayedName("New name", i * 500L)
        assertFalse(r.duplicateIdSuspected(10_000))
    }

    @Test
    fun aMissingName_neverCountsAsAChange() {
        var r = rocket("Booster")
        // Trimmed frames and frames before the beacon carry no "run".
        listOf(null, "Booster", null, "", "Booster", null).forEachIndexed { i, name ->
            r = r.noteRelayedName(name, i * 1_000L)
        }
        assertFalse(r.duplicateIdSuspected(6_000))
        // And a gap between two names is still one change, not two.
        r = r.noteRelayedName(null, 7_000).noteRelayedName("Renamed", 8_000)
        assertFalse(r.duplicateIdSuspected(8_000))
    }

    @Test
    fun changesFurtherApartThanTheWindow_doNotFlag() {
        val r = rocket("A").noteRelayedName("B", 0).noteRelayedName("A", 11_000)
        assertFalse(r.duplicateIdSuspected(11_000))
    }

    @Test
    fun theFlagHoldsThirtySecondsAfterTheLastAlternation() {
        val r = rocket("A").noteRelayedName("B", 0).noteRelayedName("A", 500)
        assertTrue(r.duplicateIdSuspected(20_000))
        assertFalse(r.duplicateIdSuspected(31_000))
    }

    @Test
    fun alternationThroughTheRealParsePath() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, "Booster")
        fw.relay(1, "Twin")
        fw.relay(1, "Booster")
        runCurrent()
        assertTrue(bs.isDuplicateIdSuspected(1))
        assertEquals(1, bs.remoteRockets.value.size, "one rid, one roster entry — that is the problem")
    }

    // ── The LoRa continuity test and its gate ─────────────────────────────

    private fun FakeFirmware.relayedPyroFrames() =
        commandFrames.filter { it[0].toInt() == BleCommandId.RELAY_TO_ROCKET_BS }
            .map { it.map { b -> b.toInt() and 0xFF } }

    @Test
    fun aFreshUnflaggedRocket_getsTheStandBackTest() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, "Booster")
        runCurrent()
        assertEquals(1, bs.focusRocketId.value)
        assertTrue(bs.pyroCommandPathReady())

        fw.commandFrames.clear()
        bs.sendPyroContTest(3)
        runCurrent()
        assertEquals(listOf(listOf(50, 1, 35, 3)), fw.relayedPyroFrames(), "cmd 50 → rocket 1, inner 35")
        // The LoRa round trip gets the longer TESTING window.
        assertTrue(bs.contTestPending(3))
        advanceTimeBy(DeviceSession.CONT_TEST_PENDING_WINDOW_RELAY_MS - 1)
        assertTrue(bs.contTestPending(3))
        advanceTimeBy(1)
        assertFalse(bs.contTestPending(3))
    }

    @Test
    fun aSharedRocketId_getsNoPyroUplink() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, "Booster")
        fw.relay(1, "Twin")
        fw.relay(1, "Booster")
        runCurrent()
        assertFalse(bs.pyroCommandPathReady(), "the gate must see the shared ID")

        fw.commandFrames.clear()
        bs.sendPyroContTest(3)
        runCurrent()
        assertTrue(fw.relayedPyroFrames().isEmpty(), "no pyro uplink may name an ID two rockets answer to")
        assertFalse(bs.contTestPending(3), "and no TESTING spinner for a test that never went out")
    }

    @Test
    fun theBaseStationsDupFlag_blocksToo() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, "Booster", extra = "\"dup\":1")
        runCurrent()
        assertFalse(bs.pyroCommandPathReady())
        fw.commandFrames.clear()
        bs.sendPyroContTest(1)
        runCurrent()
        assertTrue(fw.relayedPyroFrames().isEmpty())
    }

    @Test
    fun aRocketSilentForThreeSeconds_closesThePath() = runTest {
        val (bs, fw) = baseStation()
        fw.relay(1, "Booster")
        runCurrent()
        assertTrue(bs.pyroCommandPathReady())
        advanceTimeBy(DeviceSession.RELAY_STALE_THRESHOLD_MS + 1)
        assertFalse(bs.pyroCommandPathReady(), "a blind uplink into silence is not offered")
    }

    @Test
    fun aBaseStationWithNoRocket_hasNoPath() = runTest {
        val (bs, _) = baseStation()
        assertFalse(bs.pyroCommandPathReady())
    }
}
