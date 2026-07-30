package com.tinkerbug.tinkerrocket.app

import com.tinkerbug.tinkerrocket.session.BleAdvertisement
import com.tinkerbug.tinkerrocket.session.BleScanner
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FakeFirmware
import com.tinkerbug.tinkerrocket.session.FleetManager
import com.tinkerbug.tinkerrocket.session.FleetSessionFactory
import com.tinkerbug.tinkerrocket.session.KnownDeviceStorage
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.TransportFactory
import com.tinkerbug.tinkerrocket.session.VirtualFlightScript
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.launch

/**
 * Virtual Rocket (né demo mode): the SAME fleet/session/UI stack over
 * [FakeFirmware].  The virtual rocket sits in READY like a real one and
 * flies ONE canned ~40 s flight per SIM_START — the standard Simulation
 * tool is the start control, exactly the ritual a real rocket uses, and
 * SIM_STOP aborts mid-flight (user call 2026-07-30: no endless loop).
 * Nothing is recorded; the downloadable log in Files is the pre-staged
 * golden corpus flight, not a capture of the show.  This is the replay-
 * transport seam from the plan (§3) earning its keep as the no-hardware
 * dev path; Android-only by design (ledger).
 */
fun buildDemoFleet(context: android.content.Context, scope: CoroutineScope): FleetManager<DeviceSession> {
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
                scope = scope,
                transport = transport,
                connectedDeviceName = advertisedName,
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
        scope = scope,
        scanner = DemoScanner(),
        transportFactory = DemoTransportFactory(context, scope),
        sessionFactory = factory,
        knownDevices = KnownDeviceStore(EphemeralStorage()),
    )
    return fleet
}

private class DemoScanner : BleScanner {
    override fun advertisements(): Flow<BleAdvertisement> = flow {
        while (true) {
            emit(BleAdvertisement("demo:01", "TR-B-Virtual", rssi = -47))
            delay(500)
        }
    }
}

private class DemoTransportFactory(
    private val context: android.content.Context,
    private val scope: CoroutineScope,
) : TransportFactory {
    override fun create(deviceId: String, autoConnect: Boolean): BleTransport {
        val fw = FakeFirmware(scope)
        fw.configIdentityJson =
            """{"type":"config_identity","uid":"demo0001","un":"Virtual Base Station",""" +
                """"nid":7,"dt":"B","fw":"demo+sim"}"""
        // A real downloadable flight log: the emitter's synthetic golden
        // flight, bundled from the corpus as an asset.
        runCatching {
            context.assets.open("tiny_flight.bin").use { it.readBytes() }
        }.getOrNull()?.let { bytes ->
            fw.deviceFiles += FakeFirmware.FakeFile("flight_20260723_141100.bin", bytes)
        }
        scope.launch { runVirtualRocket(fw) }
        return fw
    }
}

/**
 * READY idle at 1 Hz until SIM_START arrives, then ONE 42 s flight at 5 Hz —
 * the frames come from [VirtualFlightScript] (shared with the announcer
 * regression test; the script's first 12 s are a PRELAUNCH countdown after
 * Launch).  SIM_STOP aborts back to READY.  Commands are read off
 * [FakeFirmware.commandFrames] by index — cheap polling at telemetry cadence.
 */
private suspend fun runVirtualRocket(fw: FakeFirmware) {
    val health = VirtualFlightScript.HEALTH
    var cmdCursor = 0

    fun sawCommand(id: Int): Boolean {
        var hit = false
        while (cmdCursor < fw.commandFrames.size) {
            if (fw.commandFrames[cmdCursor].firstOrNull()?.toInt() == id) hit = true
            cmdCursor++
        }
        return hit
    }

    while (true) {
        // READY idle until the Simulation tool starts a flight.
        while (!sawCommand(com.tinkerbug.tinkerrocket.protocol.BleCommandId.SIM_START)) {
            fw.emitTelemetryJson(
                """{"rid":1,"run":"Booster","st":"READY","fs":16,"ps":${0x00A or 0x080},""" +
                    """"h":$health,"nsat":9,"vol":8.20,"soc":87.0,"palt":0.2,""" +
                    """"lat":34.6572,"lon":-118.2015}""",
            )
            fw.emitTelemetryJson(
                """{"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,""" +
                    """"h":$health,"nsat":8,"vol":8.31,"palt":0.4}""",
            )
            delay(1000)
        }

        var aborted = false
        for (tick in 0 until VirtualFlightScript.TICKS) {
            if (sawCommand(com.tinkerbug.tinkerrocket.protocol.BleCommandId.SIM_STOP)) {
                aborted = true
                break
            }
            fw.emitTelemetryJson(VirtualFlightScript.frameJson(tick))
            // Second relayed rocket sits READY on the pad (1 Hz).
            if (tick % 5 == 0) {
                fw.emitTelemetryJson(
                    """{"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,""" +
                        """"h":$health,"nsat":8,"vol":8.31,"palt":0.4}""",
                )
            }
            delay(VirtualFlightScript.TICK_MS)
        }
        // Flight over or aborted: back to the READY idle above.  The sim
        // banner latch (DeviceSession) clears on the next READY frame; the
        // `aborted` distinction exists only for readability — both paths
        // reset the same way, like a real rocket after touchdown + reset.
        if (aborted) {
            fw.emitTelemetryJson(
                """{"rid":1,"run":"Booster","st":"READY","fs":16,"h":$health,""" +
                    """"nsat":9,"vol":8.20,"palt":0.2}""",
            )
        }
    }
}

private class EphemeralStorage : KnownDeviceStorage {
    private var json: String? = null
    override fun loadDevicesJson(): String? = json
    override fun saveDevicesJson(json: String) { this.json = json }
    override fun loadLegacyKnownIds(): List<String>? = null
    override fun removeLegacyKnownIds() = Unit
}
