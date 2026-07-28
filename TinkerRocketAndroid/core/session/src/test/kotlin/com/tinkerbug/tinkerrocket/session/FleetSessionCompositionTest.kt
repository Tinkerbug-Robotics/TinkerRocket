package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * The COMPOSED stack: FleetManager driving REAL DeviceSessions over
 * [FakeFirmware] — the production wiring shape the Phase 2 review found
 * untested (both cmd-45 double-push compositions and the unreachable #140
 * fix path only appear here, never in the per-class suites).
 */
class FleetSessionCompositionTest {

    /** TransportFactory minting one FakeFirmware per connection attempt. */
    private class FirmwareFactory(private val scope: CoroutineScope) : TransportFactory {
        val created = mutableListOf<FakeFirmware>()
        override fun create(deviceId: String, autoConnect: Boolean): BleTransport =
            FakeFirmware(scope).also { created += it }
    }

    private class ScriptedScanner : BleScanner {
        val flow = MutableSharedFlow<BleAdvertisement>(extraBufferCapacity = 16)
        override fun advertisements(): Flow<BleAdvertisement> = flow
    }

    private class Composed(
        val fleet: FleetManager<DeviceSession>,
        val transports: FirmwareFactory,
        val scanner: ScriptedScanner,
    )

    private fun TestScope.composed(): Composed {
        val transports = FirmwareFactory(backgroundScope)
        val scanner = ScriptedScanner()
        val store = KnownDeviceStore(InMemoryKnownStorage())
        lateinit var fleet: FleetManager<DeviceSession>
        val factory = object : FleetSessionFactory<DeviceSession> {
            override fun create(
                deviceId: String,
                advertisedName: String,
                generation: Int,
                transport: BleTransport,
                seedFocusRocket: Int?,
            ): DeviceSession {
                val session = DeviceSession(
                    scope = backgroundScope,
                    transport = transport,
                    connectedDeviceName = advertisedName,
                    clock = { currentTime },
                    knownDevices = store,
                    onAutoFocus = { rid -> fleet.noteAutoFocus(deviceId, rid) },
                    onRocketFix = fleet::recordRocketFix,
                    fixLookup = fleet::lastValidRocketFix,
                )
                seedFocusRocket?.let { session.seedFocusRocket(it) }
                session.start()
                return session
            }

            override fun close(session: DeviceSession) = Unit
        }
        fleet = FleetManager(
            scope = backgroundScope,
            scanner = scanner,
            transportFactory = transports,
            sessionFactory = factory,
            knownDevices = store,
            nowMillis = { currentTime },
        )
        return Composed(fleet, transports, scanner)
    }

    private fun TestScope.connectBs(c: Composed, id: String = "bs:01") {
        c.fleet.scan(userInitiated = true)
        runCurrent()
        c.scanner.flow.tryEmit(BleAdvertisement(deviceId = id, advertisedName = "TR-B-Ground", rssi = -60))
        runCurrent()
        c.fleet.connect(id)
        runCurrent()
    }

    @Test
    fun reconnect_sendsExactlyOneCmd45_inIosOrder() = runTest {
        val c = composed()
        connectBs(c)
        advanceTimeBy(1000)   // let the first choreography settle
        runCurrent()
        c.fleet.setFocus("bs:01", rocketId = 3)
        runCurrent()

        // Unexpected drop → ladder reconnects on a NEW FakeFirmware 1 s later,
        // then the fresh choreography's 1.0 s config delay elapses.
        c.transports.created.last().fireDisconnect()
        runCurrent()
        advanceTimeBy(1000)
        runCurrent()
        advanceTimeBy(1000)
        runCurrent()

        val fresh = c.transports.created.last()
        val cmds = fresh.commandFrames.map { it[0].toInt() }
        // iOS wire contract: [9, 20, 45] — ONE cmd 45, strictly after cmd 20.
        // (The old adopt-time fleet push produced [45, 9, 20, 45].)
        assertEquals(listOf(9, 20, 45), cmds)
        val dev = assertNotNull(c.fleet.devices.value["bs:01"])
        assertEquals(2, dev.generation)
        assertEquals(3, dev.session.focusRocketId.value)
    }

    @Test
    fun relayedTelemetry_populatesFleetFixCache_andSessionMirror() = runTest {
        val c = composed()
        connectBs(c)
        advanceTimeBy(1000)
        runCurrent()

        // Relayed rocket 1 with a valid GPS fix: the per-packet hook must
        // land it in the fleet cache (#140) and — being auto-focused — the
        // session mirror.  This is the path the review proved unreachable.
        val fw = c.transports.created.last()
        fw.emitTelemetryJson(
            """{"st":"INFLIGHT","rid":1,"lat":33.7,"lon":-118.4,"nsat":8,"fs":8}""",
        )
        runCurrent()

        assertTrue(c.fleet.lastValidRocketFixes.value.isNotEmpty(), "fleet cache latched")
        val session = assertNotNull(c.fleet.devices.value["bs:01"]).session
        val mirror = assertNotNull(session.lastValidRocketFix.value, "session mirror latched")
        assertEquals(33.7, mirror.latitude, 1e-9)

        // A GPS-less packet must NOT blank either (#140's whole point).
        fw.emitTelemetryJson("""{"st":"LANDED","rid":1}""")
        runCurrent()
        assertEquals(33.7, assertNotNull(session.lastValidRocketFix.value).latitude, 1e-9)
    }
}
