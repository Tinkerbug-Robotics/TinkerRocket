package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNotSame
import kotlin.test.assertTrue

/**
 * Reconnect ladder (iOS BLEFleet didDisconnectPeripheral, #291): unexpected
 * disconnect → up to 8 direct attempts spaced min(8, 2^(n-1)) s apart, then
 * the endgame.  iOS endgame = park a CB connect() forever + scan fallback;
 * the Android analog under test = background RE-SCAN, THEN autoConnect once
 * the device is sighted — never autoConnect blind from a MAC (plan §3).
 * All timing runs on runTest virtual time.
 */
class ReconnectLadderTest {

    /** Expected attempt offsets from the disconnect: cumulative 1,2,4,8,8,8,8,8 s. */
    private val expectedOffsetsMs =
        listOf(1_000L, 3_000L, 7_000L, 15_000L, 23_000L, 31_000L, 39_000L, 47_000L)

    @Test
    fun backoffFormula_pinned() {
        assertEquals(
            listOf(1_000L, 2_000L, 4_000L, 8_000L, 8_000L, 8_000L, 8_000L, 8_000L),
            (1..8).map { FleetManager.reconnectBackoffMs(it) },
        )
    }

    @Test
    fun unexpectedDisconnect_attemptsSpaced_1_2_4_8_8_8_8_8() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        advanceTimeBy(48_000)
        runCurrent()

        val ladderCalls = h.transports.calls.drop(1)  // drop the initial connect
        assertEquals(FleetManager.MAX_RECONNECT_ATTEMPTS, ladderCalls.size)
        assertEquals(expectedOffsetsMs, ladderCalls.map { it.atMillis })
        assertTrue(ladderCalls.all { !it.autoConnect },
            "Ladder attempts are direct connects (autoConnect=false)")
        assertTrue(ladderCalls.all { it.deviceId == "aa:01" })
    }

    @Test
    fun reconnectSuccessMidLadder_adoptsNewGeneration_stopsLadder() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        val firstSession = h.fleet.devices.value["aa:01"]!!.session

        h.transports.failNextConnects = 2   // attempts 1+2 fail, 3 succeeds
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        advanceTimeBy(7_000)
        runCurrent()
        val dev = assertNotNull(h.fleet.devices.value["aa:01"], "Attempt 3 (t=7 s) reconnects")
        assertEquals(2, dev.generation)
        assertNotSame(firstSession, dev.session, "Same id, NEW session object")
        assertEquals("Connected to TR-R-Atlas", h.fleet.statusMessage.value)

        // No further attempts after success.
        val callsAfter = h.transports.calls.size
        advanceTimeBy(60_000)
        runCurrent()
        assertEquals(callsAfter, h.transports.calls.size)
    }

    @Test
    fun ladderResetsAfterSuccessfulReconnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failNextConnects = 2
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        advanceTimeBy(7_000)   // reconnected at t=7 s (attempt 3)
        runCurrent()
        assertEquals(2, h.fleet.devices.value["aa:01"]!!.generation)

        // Second drop: attempts must start back at a 1 s backoff, not
        // continue where the last ladder left off.
        val dropTime = testScheduler.currentTime
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        advanceTimeBy(1_000)
        runCurrent()
        val dev = assertNotNull(h.fleet.devices.value["aa:01"])
        assertEquals(3, dev.generation)
        assertEquals(dropTime + 1_000, h.transports.calls.last().atMillis,
            "Fresh ladder: first attempt 1 s after the new drop")
    }

    @Test
    fun userInitiatedDisconnect_suppressesLadder() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.fleet.disconnect("aa:01")
        runCurrent()
        advanceTimeBy(300_000)
        runCurrent()
        assertEquals(1, h.transports.calls.size, "Only the initial connect — no ladder")
        assertEquals("Disconnected", h.fleet.statusMessage.value)
    }

    @Test
    fun manualReconnectDuringBackoff_ladderStandsDown() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        // User reconnects by hand 500 ms into the 1 s backoff.
        advanceTimeBy(500)
        h.fleet.connect("aa:01")
        runCurrent()
        assertEquals(2, h.fleet.devices.value["aa:01"]!!.generation)

        // The ladder wakes at t=1 s, sees the device back, and stands down.
        advanceTimeBy(60_000)
        runCurrent()
        assertEquals(2, h.transports.calls.size, "No ladder attempt after the manual reconnect")
    }

    @Test
    fun endgame_afterEightFailures_rescanThenAutoConnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        // Through all 8 failed attempts and into the endgame.
        advanceTimeBy(47_001)
        runCurrent()
        assertTrue(h.fleet.isScanning.value, "Endgame starts a background scan")
        assertFalse(h.fleet.userInitiatedScan.value,
            "#394: reconnect scans must never pop the Add-Device UI")
        assertTrue(h.transports.calls.none { it.autoConnect },
            "NEVER autoConnect blind from a stored MAC — a scan sighting must come first")

        // Silence: no sighting → no connect attempts, however long we wait.
        val callsBefore = h.transports.calls.size
        advanceTimeBy(600_000)
        runCurrent()
        assertEquals(callsBefore, h.transports.calls.size)

        // The device comes back into range: sighting unlocks ONE
        // autoConnect attempt, which succeeds and re-adopts.
        h.transports.failAllConnects = false
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -70))
        runCurrent()
        val last = h.transports.calls.last()
        assertTrue(last.autoConnect, "Endgame connect is the parked autoConnect")
        assertEquals("aa:01", last.deviceId)
        val dev = assertNotNull(h.fleet.devices.value["aa:01"])
        assertEquals(2, dev.generation)
        assertEquals("Connected to TR-R-Atlas", h.fleet.statusMessage.value)

        // Ladder is done — nothing else fires later.
        val settled = h.transports.calls.size
        advanceUntilIdle()
        assertEquals(settled, h.transports.calls.size)
    }

    @Test
    fun endgame_failedAutoConnect_waitsForNextSighting() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        advanceTimeBy(47_001)
        runCurrent()

        // First sighting: the autoConnect attempt itself fails (device
        // ducked out again) — the endgame goes back to waiting instead of
        // spinning on the MAC.
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -80))
        runCurrent()
        assertEquals(1, h.transports.calls.count { it.autoConnect })
        assertTrue(h.fleet.devices.value.isEmpty())

        val callsBefore = h.transports.calls.size
        advanceTimeBy(120_000)
        runCurrent()
        assertEquals(callsBefore, h.transports.calls.size, "No retry without a fresh sighting")

        h.transports.failAllConnects = false
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -60))
        runCurrent()
        assertEquals(2, h.transports.calls.count { it.autoConnect })
        assertNotNull(h.fleet.devices.value["aa:01"])
    }

    @Test
    fun unexpectedDisconnect_closesOldSession_keepsSurvivors() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.setFocus("bs:01", rocketId = 2)
        val key = RocketKey(networkId = 5, rocketId = 2)
        h.fleet.recordRocketFix(
            com.tinkerbug.tinkerrocket.protocol.TelemetryData(
                latitude = 33.7, longitude = -118.4, numSats = 8,
            ),
            key,
        )
        val session = h.fleet.devices.value["bs:01"]!!.session

        h.transports.failAllConnects = true
        h.transports.lastFor("bs:01").dropUnexpectedly()
        runCurrent()

        assertTrue(h.sessions.closed.contains(session), "Old session torn down")
        // Fleet-scoped survivors outlive the connection (#140/#390).
        assertEquals(2, h.fleet.focusFor("bs:01"))
        assertNotNull(h.fleet.lastValidRocketFix(key))
    }
}
