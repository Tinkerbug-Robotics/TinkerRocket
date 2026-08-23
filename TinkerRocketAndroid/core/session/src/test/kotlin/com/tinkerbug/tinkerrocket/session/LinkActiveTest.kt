package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #829: [FleetManager.linkActive] — the signal the foreground service keys on.
 *
 * The service used to key on [FleetManager.devices] and stop itself the moment
 * the map emptied. [FleetManager.handleDisconnect] removes the device BEFORE
 * the reconnect ladder starts, so on a single-device fleet every transient
 * drop tore down the service that exists to keep the process (and its BLE
 * links) alive — at exactly the moment the link needed recovering. Nothing
 * could restart it either: the only starter was a Compose LaunchedEffect,
 * which cannot re-run while the activity is stopped, and once it resumes the
 * ladder has refilled the map so the effect's key reads unchanged.
 *
 * The property these pin: linkActive must NOT dip false across a
 * drop-and-reconnect, and must still go false when the user walks away.
 */
class LinkActiveTest {

    @Test
    fun falseBeforeAnythingConnects() = runTest {
        val h = fleetHarness()
        assertFalse(h.fleet.linkActive.value)
    }

    @Test
    fun trueOnceConnected() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        assertTrue(h.fleet.linkActive.value)
    }

    /** The bug, stated directly. */
    @Test
    fun staysTrueAcrossAnUnexpectedDrop() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")

        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        assertTrue(
            h.fleet.devices.value.isEmpty(),
            "premise: the fleet DOES go empty — that is what the service saw",
        )
        assertTrue(
            h.fleet.linkActive.value,
            "the ladder is running; the process pin must not be released here",
        )
    }

    @Test
    fun staysTrueThroughFailingLadderRungs() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true

        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        // Walk well past the 1,2,4,8,8,8,8,8 s rungs into the endgame's
        // unbounded sighting wait. There is no "gave up" state by design.
        repeat(12) {
            advanceTimeBy(10_000)
            runCurrent()
            assertTrue(h.fleet.linkActive.value, "must hold across every rung")
        }
    }

    @Test
    fun staysTrueAcrossAFullDropAndReconnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")

        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        assertTrue(h.fleet.linkActive.value)

        advanceTimeBy(2_000)
        runCurrent()
        assertTrue(
            h.fleet.devices.value.containsKey("aa:01"),
            "premise: the ladder reconnects on an early rung",
        )
        assertTrue(h.fleet.linkActive.value)
    }

    // ---------------------------------------------------------- it still stops

    @Test
    fun goesFalseOnUserDisconnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.fleet.disconnect("aa:01")
        runCurrent()
        assertFalse(
            h.fleet.linkActive.value,
            "walking away is the one case that releases the process pin",
        )
    }

    @Test
    fun goesFalseOnDisconnectAll() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        discoverAndConnect(h, "bs:02", "TR-B-Base")
        h.fleet.disconnectAll()
        runCurrent()
        assertFalse(h.fleet.linkActive.value)
    }

    /**
     * A user disconnect must not release the pin while ANOTHER device is
     * still mid-ladder — the naive "clear it in the user-disconnect branch"
     * fix gets this wrong.
     */
    @Test
    fun staysTrueWhenOneDeviceIsDroppedAndAnotherIsUserDisconnected() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        discoverAndConnect(h, "bs:02", "TR-B-Base")

        // Rocket drops unexpectedly → its ladder starts.
        h.transports.failAllConnects = true
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        // User walks away from the base station.
        h.fleet.disconnect("bs:02")
        runCurrent()

        assertTrue(h.fleet.devices.value.isEmpty(), "premise: no device is connected")
        assertTrue(
            h.fleet.linkActive.value,
            "the rocket's ladder is still trying — the flight is not over",
        )
    }

    /**
     * Superseding a ladder with a manual connect must leave the flag
     * consistent, not stranded true by the cancelled job.
     */
    @Test
    fun manualConnectDuringALadderSettlesTrueOnSuccess() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        assertTrue(h.fleet.linkActive.value)

        h.transports.failAllConnects = false
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        assertTrue(h.fleet.devices.value.containsKey("aa:01"))
        assertTrue(h.fleet.linkActive.value)
    }
}
