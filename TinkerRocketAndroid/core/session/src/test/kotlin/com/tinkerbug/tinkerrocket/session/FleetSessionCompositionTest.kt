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
        /** Identity readback the NEXT minted firmware answers cmd 20 with (null = FakeFirmware's default). */
        var pendingIdentityJson: String? = null
        override fun create(deviceId: String, autoConnect: Boolean): BleTransport =
            FakeFirmware(scope).also { fw ->
                pendingIdentityJson?.let { fw.configIdentityJson = it }
                created += fw
            }
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
                initialDeviceType: BleDeviceType,
                seedFocusRocket: Int?,
            ): DeviceSession {
                val session = DeviceSession(
                    scope = backgroundScope,
                    transport = transport,
                    connectedDeviceName = advertisedName,
                    initialDeviceType = initialDeviceType,
                    clock = { currentTime },
                    knownDevices = store,
                    onAutoFocus = { rid -> fleet.noteAutoFocus(deviceId, rid) },
                    onUserFocus = { rid -> fleet.recordFocus(deviceId, rid) },
                    onIdentity = { msg, pusher -> fleet.onIdentityReadback(deviceId, msg, pusher) },
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

    // ── #1040 / #1041: what the session decides reaches the fleet ─────────

    @Test
    fun userFocusSwitch_survivesReconnect_andRepinsTheChosenRocket() = runTest {
        val c = composed()
        // The readback must keep this link a base station, or the relayed
        // path (and with it the auto-latch) is skipped after 1 s.
        c.transports.pendingIdentityJson =
            """{"type":"config_identity","uid":"b1","un":"Ground","nid":1,"dt":"B"}"""
        connectBs(c)
        advanceTimeBy(1000)
        runCurrent()
        val fw1 = c.transports.created.last()
        // Rocket 1 is heard first → sticky auto-focus 1 on the fleet map.
        fw1.emitTelemetryJson("""{"st":"READY","rid":1}""")
        runCurrent()
        assertEquals(1, c.fleet.focusFor("bs:01"))
        // The operator taps rocket 2 in the roster: session-level switch.
        val session1 = assertNotNull(c.fleet.devices.value["bs:01"]).session
        session1.setFocusRocket(2)
        runCurrent()
        // #1040: the FLEET map moved too (before this it still said 1).
        assertEquals(2, c.fleet.focusFor("bs:01"))
        // Link bounces → a NEW session is seeded from the map and its
        // choreography re-pushes cmd 45 — for rocket 2, not rocket 1.
        fw1.fireDisconnect()
        runCurrent()
        advanceTimeBy(1000)
        runCurrent()
        advanceTimeBy(1000)
        runCurrent()
        val fresh = c.transports.created.last()
        val dev = assertNotNull(c.fleet.devices.value["bs:01"])
        assertEquals(2, dev.generation)
        assertEquals(2, dev.session.focusRocketId.value, "re-seeded from the user's choice")
        assertEquals(listOf(2), fresh.focusPins, "cmd 45 carries the user's rocket, not the first heard")
    }

    @Test
    fun identityReadback_updatesTheFleetRecord_notJustTheSession() = runTest {
        val c = composed()
        // Advertised as a rocket; the config_identity readback says base station.
        c.fleet.scan(userInitiated = true)
        runCurrent()
        c.scanner.flow.tryEmit(BleAdvertisement(deviceId = "x:01", advertisedName = "TR-R-Guess", rssi = -60))
        runCurrent()
        c.transports.pendingIdentityJson =
            """{"type":"config_identity","uid":"u-9","un":"REALLY-A-BS","nid":4,"dt":"B","fw":"1.2.3"}"""
        c.fleet.connect("x:01")
        runCurrent()
        val before = assertNotNull(c.fleet.devices.value["x:01"])
        assertEquals(BleDeviceType.ROCKET, before.deviceType, "connect-time guess from the name")
        assertEquals(BleDeviceType.ROCKET, before.session.identity.value.deviceType, "#1041: the session starts from the fleet's seed")
        advanceTimeBy(1000)   // cmd 20 → identity readback
        runCurrent()
        val after = assertNotNull(c.fleet.devices.value["x:01"])
        // #1041: the fleet's OWN record follows the readback (voice routing and
        // the foreground-BS pick read this; it used to stay frozen at the guess).
        assertEquals(BleDeviceType.BASE_STATION, after.deviceType)
        assertEquals("REALLY-A-BS", after.unitName)
        assertEquals("u-9", after.unitId)
        assertEquals(BleDeviceType.BASE_STATION, after.session.identity.value.deviceType)
        // And the registry got it exactly once (through the fleet).
        assertEquals("REALLY-A-BS", c.fleet.knownDevices.device("u-9")?.name)
    }
}
