package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.Job
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #838 item 6 — every DeviceSession leaked a live transport-events collector.
 *
 * `start()` launched `scope.launch { transport.events.collect { … } }` and
 * DISCARDED the Job (only the second launch's Job is returned), on the
 * app-lifetime fleetScope. `transport.events` is a MutableSharedFlow that
 * never completes, so that coroutine ran forever. DeviceSession exposed no
 * stop or close, and the production `FleetSessionFactory.close` was literally
 * `= Unit` — despite the seam documenting it as "Teardown for a session whose
 * connection dropped".
 *
 * FleetManager builds a brand-new session on every (re)connect and drops the
 * old one from `_devices`, but the live collector kept the old DeviceSession
 * reachable, and with it its transport (a BluetoothGatt and its chars map),
 * its remoteMap, and any partially assembled download buffer. A flaky launch
 * day that cycles the link 40 times left 40 session graphs and 40 live
 * coroutines with nothing able to reclaim them — on the same phone holding
 * the flight's live telemetry.
 */
class SessionCloseTest {

    private fun TestScope.session(transport: FakeTransport): DeviceSession =
        DeviceSession(
            scope = backgroundScope,
            transport = transport,
            connectedDeviceName = "TR-R-Atlas",
            clock = { testScheduler.currentTime },
        )

    /** THE regression: the collector must not outlive the session. */
    @Test
    fun closeEndsTheTransportEventsCollector() = runTest {
        val transport = FakeTransport("aa:bb")
        val session = session(transport)
        session.start()
        runCurrent()

        assertTrue(transport.eventCollectorCount > 0, "precondition: the collector is running")

        session.close()
        runCurrent()

        assertEquals(
            0, transport.eventCollectorCount,
            "the events collector outlived the session — this is the leak",
        )
    }

    /** Everything the session launched goes, not just the collector. */
    @Test
    fun closeCancelsAllSessionWork() = runTest {
        val transport = FakeTransport("aa:bb")
        val session = session(transport)
        val startJob = session.start()
        runCurrent()

        session.close()
        runCurrent()

        assertTrue(startJob.isCancelled || startJob.isCompleted)
    }

    /**
     * The session's scope is a CHILD of the fleet's, so closing one session
     * must not touch the fleet or its siblings — FleetManager keeps other
     * devices connected while one drops.
     */
    @Test
    fun closingOneSessionLeavesTheFleetAndItsSiblingsAlive() = runTest {
        val a = FakeTransport("aa:bb")
        val b = FakeTransport("aa:bb")
        val sessionA = session(a)
        val sessionB = session(b)
        sessionA.start()
        sessionB.start()
        runCurrent()

        sessionA.close()
        runCurrent()

        assertFalse(a.eventCollectorCount > 0)
        assertTrue(b.eventCollectorCount > 0, "closing one session killed its sibling")
        assertTrue(backgroundScope.coroutineContext[Job]!!.isActive, "the fleet scope was cancelled")
    }

    /** handleDisconnect can fire more than once for a device (ghost events). */
    @Test
    fun closeIsIdempotent() = runTest {
        val transport = FakeTransport("aa:bb")
        val session = session(transport)
        session.start()
        runCurrent()

        session.close()
        session.close()
        runCurrent()

        assertFalse(transport.eventCollectorCount > 0)
    }

    /** A session that never connected still closes cleanly. */
    @Test
    fun closeWithoutStartIsSafe() = runTest {
        val session = session(FakeTransport("aa:bb"))
        session.close()
        runCurrent()
    }

    /**
     * The leak's actual shape: N reconnects leave N live collectors. One
     * transport per connection, as FleetManager does it.
     */
    @Test
    fun fortyReconnectsLeaveNothingRunning() = runTest {
        val transports = List(40) { FakeTransport("aa:bb") }
        for (t in transports) {
            val s = session(t)
            s.start()
            runCurrent()
            s.close()
            runCurrent()
        }
        assertEquals(
            0,
            transports.count { it.eventCollectorCount > 0 },
            "sessions accumulated on the fleet scope",
        )
    }
}
