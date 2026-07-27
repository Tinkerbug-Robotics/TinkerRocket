package com.tinkerbug.tinkerrocket.app

import android.app.Application
import android.content.Context
import com.tinkerbug.tinkerrocket.ble.AndroidBleScanner
import com.tinkerbug.tinkerrocket.ble.AndroidTransportFactory
import com.tinkerbug.tinkerrocket.session.ActiveProfileStorage
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetManager
import com.tinkerbug.tinkerrocket.session.FleetSessionFactory
import com.tinkerbug.tinkerrocket.session.KnownDeviceStorage
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.asCoroutineDispatcher
import java.io.File
import java.util.concurrent.Executors

/**
 * Manual DI (android-port plan §1 — no Hilt): ~10 app-scoped singletons with
 * constructor wiring.  The fleet stack runs on ONE confined dispatcher (the
 * single-writer contract, plan §3) that is deliberately NOT Main, so the
 * pipeline keeps running during future service-only operation.
 *
 * This is the production twin of FleetSessionCompositionTest's wiring —
 * the composed shape the Phase 2 review demanded stays the tested shape.
 */
class AppContainer(app: Application) {

    private val fleetDispatcher =
        Executors.newSingleThreadExecutor { r -> Thread(r, "tr-fleet") }.asCoroutineDispatcher()
    val fleetScope = CoroutineScope(SupervisorJob() + fleetDispatcher)

    private val knownDevices = KnownDeviceStore(PrefsKnownStorage(app))

    // Loads a handful of small JSON files at app create — acceptable on Main;
    // all MUTATORS go through the fleet dispatcher (single-writer contract).
    val profileStore = RocketProfileStore(
        directory = File(app.filesDir, "rocket_profiles"),
        activeStorage = PrefsActiveProfileStorage(app),
    )
    val syncer = ActiveRocketSyncer(fleetScope)

    val fleet: FleetManager<DeviceSession> = run {
        lateinit var fleetRef: FleetManager<DeviceSession>
        val sessionFactory = object : FleetSessionFactory<DeviceSession> {
            override fun create(
                deviceId: String,
                advertisedName: String,
                generation: Int,
                transport: BleTransport,
                seedFocusRocket: Int?,
            ): DeviceSession {
                val session = DeviceSession(
                    scope = fleetScope,
                    transport = transport,
                    connectedDeviceName = advertisedName,
                    knownDevices = knownDevices,
                    onAutoFocus = { rid -> fleetRef.noteAutoFocus(deviceId, rid) },
                    onRocketFix = fleetRef::recordRocketFix,
                    fixLookup = fleetRef::lastValidRocketFix,
                )
                seedFocusRocket?.let { session.seedFocusRocket(it) }
                session.start()
                return session
            }

            override fun close(session: DeviceSession) = Unit
        }
        fleetRef = FleetManager(
            scope = fleetScope,
            scanner = AndroidBleScanner(app),
            transportFactory = AndroidTransportFactory(app),
            sessionFactory = sessionFactory,
            knownDevices = knownDevices,
        )
        fleetRef
    }
}

/** Active-profile id persistence (iOS: UserDefaults). */
private class PrefsActiveProfileStorage(app: Application) : ActiveProfileStorage {
    private val prefs = app.getSharedPreferences("rocket_profiles", Context.MODE_PRIVATE)
    override fun loadActiveId(): String? = prefs.getString("active_id", null)
    override fun saveActiveId(id: String?) {
        prefs.edit().putString("active_id", id).apply()
    }
}

/** KnownDeviceStore persistence on SharedPreferences — same JSON schema as iOS. */
private class PrefsKnownStorage(app: Application) : KnownDeviceStorage {
    private val prefs = app.getSharedPreferences("known_devices", Context.MODE_PRIVATE)
    override fun loadDevicesJson(): String? = prefs.getString("devices_json", null)
    override fun saveDevicesJson(json: String) {
        prefs.edit().putString("devices_json", json).apply()
    }
    override fun loadLegacyKnownIds(): List<String>? = null
    override fun removeLegacyKnownIds() = Unit
}

class TinkerRocketApp : Application() {
    lateinit var container: AppContainer
        private set

    override fun onCreate() {
        super.onCreate()
        container = AppContainer(this)
    }
}
