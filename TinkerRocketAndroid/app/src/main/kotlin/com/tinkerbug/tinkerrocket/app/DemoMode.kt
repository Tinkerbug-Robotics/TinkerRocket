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
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.launch
import java.util.Locale
import kotlin.math.max
import kotlin.math.roundToInt

/**
 * Demo mode: the SAME fleet/session/UI stack over [FakeFirmware], flying a
 * canned ~40 s flight on loop — pad wait → boost → coast → apogee → descent
 * → landed.  This is the replay-transport seam from the plan (§3) earning
 * its keep as the no-hardware dev path: every screen is developable against
 * it in the emulator, and it doubles as the in-app demo.
 */
fun buildDemoFleet(scope: CoroutineScope): FleetManager<DeviceSession> {
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
        transportFactory = DemoTransportFactory(scope),
        sessionFactory = factory,
        knownDevices = KnownDeviceStore(EphemeralStorage()),
    )
    return fleet
}

private class DemoScanner : BleScanner {
    override fun advertisements(): Flow<BleAdvertisement> = flow {
        while (true) {
            emit(BleAdvertisement("demo:01", "TR-B-Demo Base", rssi = -47))
            delay(500)
        }
    }
}

private class DemoTransportFactory(private val scope: CoroutineScope) : TransportFactory {
    override fun create(deviceId: String, autoConnect: Boolean): BleTransport {
        val fw = FakeFirmware(scope)
        fw.configIdentityJson =
            """{"type":"config_identity","uid":"demo0001","un":"Demo Base Station",""" +
                """"nid":7,"dt":"B","fw":"demo+sim"}"""
        scope.launch { flyDemoFlight(fw) }
        return fw
    }
}

/** ~40 s looped flight profile, 5 Hz telemetry, all-healthy sensors. */
private suspend fun flyDemoFlight(fw: FakeFirmware) {
    // OK (01) at baro/imu/ekf/mag/gnss/batt/storage shifts; pyro NA (hidden).
    val health = 0x100555
    var maxAlt = 0f
    while (true) {
        for (tick in 0 until 200) {
            val t = tick / 5.0f    // seconds into the loop
            val phase = when {
                t < 12f -> "READY"
                t < 14.5f -> "INFLIGHT"       // boost
                t < 22f -> "INFLIGHT"         // coast
                t < 36f -> "DESCENT"
                else -> "LANDED"
            }
            val alt = when {
                t < 12f -> 0f
                t < 22f -> {
                    val tt = t - 12f
                    max(0f, 80f * tt - 4f * tt * tt)
                }
                t < 36f -> max(0f, 400f - (t - 22f) * 28f)
                else -> 0f
            }
            maxAlt = max(maxAlt, alt)
            var fs = 0x10 or 0x40                      // pwr on + logging
            if (t >= 12f) fs = fs or 0x01              // launch
            if (t >= 14.5f) fs = fs or 0x200           // burnout
            if (t >= 22f) fs = fs or 0x02 or 0x04      // apogee votes
            if (t >= 36f) fs = fs or 0x08              // landed
            val vol = 8.2f - t * 0.004f
            val sats = if (t < 2f) (t * 4).roundToInt() else 9
            fw.emitTelemetryJson(
                String.format(
                    Locale.ROOT,
                    """{"rid":1,"run":"Booster","st":"%s","fs":%d,"ps":%d,"h":%d,""" +
                        """"nsat":%d,"vol":%.2f,"cur":%.1f,"soc":%.1f,"palt":%.1f,""" +
                        """"malt":%.1f,"arate":%.1f,"lat":34.6572,"lon":-118.2015}""",
                    phase, fs,
                    0x00A or 0x080,        // ch1+ch4 continuity (demo)
                    health, sats, vol,
                    120f + (if (phase == "INFLIGHT") 900f else 0f),
                    87f - t * 0.1f, alt, maxAlt,
                    when {
                        t in 12f..14.5f -> 95f
                        t in 14.5f..22f -> 30f - (t - 14.5f) * 4f
                        t in 22f..36f -> -28f
                        else -> 0f
                    },
                ),
            )
            // Second relayed rocket sits READY on the pad (1 Hz).
            if (tick % 5 == 0) {
                fw.emitTelemetryJson(
                    """{"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,""" +
                        """"h":$health,"nsat":8,"vol":8.31,"palt":0.4}""",
                )
            }
            delay(200)
        }
        maxAlt = 0f
    }
}

private class EphemeralStorage : KnownDeviceStorage {
    private var json: String? = null
    override fun loadDevicesJson(): String? = json
    override fun saveDevicesJson(json: String) { this.json = json }
    override fun loadLegacyKnownIds(): List<String>? = null
    override fun removeLegacyKnownIds() = Unit
}
