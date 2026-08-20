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
import com.tinkerbug.tinkerrocket.session.LastSessionStore
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.launch
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

    val knownDevices = KnownDeviceStore(PrefsKnownStorage(app))

    /** #150 app-side network identity (iOS @AppStorage networkName/networkID). */
    val networkStore = AppNetworkStore(app)

    // Loads a handful of small JSON files at app create — acceptable on Main;
    // all MUTATORS go through the fleet dispatcher (single-writer contract).
    val profileStore = RocketProfileStore(
        directory = File(app.filesDir, "rocket_profiles"),
        activeStorage = PrefsActiveProfileStorage(app),
    )
    val syncer = ActiveRocketSyncer(fleetScope)

    /** Phone GPS + heading (ref-counted; consumers start/stop). */
    val phoneLocation = PhoneLocationManager(app)

    /** GitHub-releases update check (plan §1); throttled + failure-silent. */
    val updateChecker = UpdateChecker(app)

    /**
     * App-wide display units (iOS #160 twin — same UserDefaults key name and
     * raw values for the parity ledger's key inventory).  Declared before the
     * announcer, which captures it for spoken units.  Wire/log/CSV stay SI;
     * conversion happens only at display and speech.
     */
    val units = UnitStore(app)

    /**
     * Voice callouts (#643): policy in :core:session, TextToSpeech behind the
     * AnnouncerSpeech seam.  App-scoped, unlike iOS's per-DashboardView
     * @StateObject — a mid-flight navigation away from the dashboard must not
     * tear down the announcer.  Enabled flag shares the iOS UserDefaults key
     * name for the parity ledger's key inventory.
     */
    val ttsSpeaker = TtsSpeaker(app)
    val announcer: com.tinkerbug.tinkerrocket.session.FlightAnnouncer = run {
        val prefs = app.getSharedPreferences("announcer", Context.MODE_PRIVATE)
        com.tinkerbug.tinkerrocket.session.FlightAnnouncer(
            speech = ttsSpeaker,
            initialEnabled = prefs.getBoolean("voiceAnnouncementsEnabled", false),
            onEnabledChanged = { prefs.edit().putBoolean("voiceAnnouncementsEnabled", it).apply() },
            unitSystem = { units.system.value },
            // #813: the flyer's own descent figures, so a deployment can be
            // graded against what THIS rocket is supposed to do.
            recovery = {
                profileStore.activeProfile?.let {
                    com.tinkerbug.tinkerrocket.session.RecoveryProfile.from(it)
                }
            },
        )
    }

    /**
     * OTA sessions keyed by device id.  Owned HERE, above the fleet, because
     * the fleet destroys and recreates a DeviceSession across every
     * disconnect — including the post-OTA reboot (#140).  Keeping the OTA
     * session here is what lets the flow survive that cycle and report the
     * post-reboot version, and what keeps a finished result visible when the
     * user navigates away and back.
     */
    private val otaSessions = mutableMapOf<String, com.tinkerbug.tinkerrocket.session.OtaSession>()

    /**
     * The OTA currently mid-flight, if any.  During the post-flash reboot the
     * device disappears from the fleet, so the normal device UI unmounts —
     * this is what lets the app keep showing progress instead of dropping the
     * user back to the scanner while their firmware is being written.
     */
    fun runningOta(): com.tinkerbug.tinkerrocket.session.OtaSession? =
        otaSessions.values.firstOrNull { it.isRunning }

    fun otaSessionFor(deviceId: String): com.tinkerbug.tinkerrocket.session.OtaSession =
        otaSessions.getOrPut(deviceId) {
            com.tinkerbug.tinkerrocket.session.OtaSession(
                scope = fleetScope,
                // Re-resolved on every poll, never captured: the session
                // object behind this id changes across the OTA reboot.
                sessionLookup = { fleet.devices.value[deviceId]?.session },
            )
        }

    // MapLibre keeps its own on-disk HTTP/tile cache (mbgl-offline.db) and
    // its native Cache-Control parser ignores no-store — left enabled it
    // would persist Google satellite tiles (forbidden by provider terms)
    // and shadow the proxy's cache semantics.  ALL intended caching lives
    // in OfflineTileCache, so kill the ambient cache: purge anything ever
    // stored, then cap it at zero.
    init {
        org.maplibre.android.MapLibre.getInstance(app)
        val offline = org.maplibre.android.offline.OfflineManager.getInstance(app)
        offline.clearAmbientCache(null)
        offline.setMaximumAmbientCacheSize(0, null)
    }

    // Offline maps: flat-file cache in noBackupFilesDir (never OS-purged,
    // never backed up — field data must survive storage pressure) + the
    // localhost read-through proxy MapLibre's raster sources fetch from.
    val tileCache = com.tinkerbug.tinkerrocket.maps.OfflineTileCache(
        File(app.noBackupFilesDir, "OfflineTiles"),
    )
    // Google satellite (Map Tiles API) is the online-only extra basemap:
    // keyed builds offer it in the picker; keyless forks never construct
    // the upstream and the picker skips it.  The proxy never lets these
    // tiles near the offline cache (provider terms) — offline stays USGS.
    val googleMapsAvailable: Boolean =
        com.tinkerbug.tinkerrocket.BuildConfig.TR_MAPS_API_KEY.isNotEmpty()
    val tileProxy = com.tinkerbug.tinkerrocket.maps.TileProxyServer(
        tileCache,
        googleUpstream = if (googleMapsAvailable) {
            com.tinkerbug.tinkerrocket.maps.GoogleTileUpstream(
                apiKey = com.tinkerbug.tinkerrocket.BuildConfig.TR_MAPS_API_KEY,
                // App-attestation for the Android-restricted key: computed
                // from whatever cert actually signed THIS apk, so debug and
                // release builds each present their own fingerprint (both
                // are enrolled in the key's console restriction).
                appPackage = app.packageName,
                appCertSha1 = signingCertSha1(app),
            )
        } else {
            null
        },
    ).apply { start() }
    // Saved-region manifest lives beside the tile tree (iOS: App Support root).
    val regionStore = com.tinkerbug.tinkerrocket.maps.OfflineRegionStore(app.noBackupFilesDir, tileCache)

    /**
     * SHA-1 of this APK's signing certificate, uppercase hex without
     * separators — the X-Android-Cert format Google validates against the
     * Maps key's "Android apps" restriction.
     */
    private fun signingCertSha1(app: Application): String? = runCatching {
        @Suppress("DEPRECATION")
        val info = app.packageManager.getPackageInfo(
            app.packageName,
            android.content.pm.PackageManager.GET_SIGNING_CERTIFICATES,
        )
        val signer = info.signingInfo?.apkContentsSigners?.firstOrNull()
            ?: return@runCatching null
        java.security.MessageDigest.getInstance("SHA-1")
            .digest(signer.toByteArray())
            .joinToString("") { "%02X".format(it) }
    }.getOrNull()

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
            lastSession = PrefsLastSessionStore(app),
        )
        fleetRef
    }

    init {
        // #829: the foreground service's lifetime belongs here, at process
        // scope — not in the composition. It used to be started by a Compose
        // LaunchedEffect, which cannot restart it after a mid-flight drop:
        // while the activity is stopped the Recomposer's frame clock is
        // paused, and by the time it resumes the reconnect ladder has already
        // refilled the device map, so the effect's key reads unchanged and
        // never re-runs. The service stayed dead for the rest of the session.
        //
        // linkActive stays true across a drop-and-reconnect, so this only
        // fires on the transitions that matter, and the service stops itself
        // when it goes false.
        fleetScope.launch {
            fleet.linkActive.collect { active ->
                if (active) FleetService.start(app)
            }
        }
    }
}

/**
 * #150 network identity, prefs-backed with reactive reads.  The ID is
 * always derived from the name via [NetworkIdentity.networkIdForName] —
 * never stored independently, so the pair can't drift.
 */
class AppNetworkStore(app: Application) {
    private val prefs = app.getSharedPreferences("network_identity", Context.MODE_PRIVATE)

    private val _name = kotlinx.coroutines.flow.MutableStateFlow(prefs.getString("networkName", "") ?: "")
    val name: kotlinx.coroutines.flow.StateFlow<String> = _name

    private val _id = kotlinx.coroutines.flow.MutableStateFlow(prefs.getInt("networkID", 0))
    val id: kotlinx.coroutines.flow.StateFlow<Int> = _id

    fun setNetwork(name: String) {
        val id = com.tinkerbug.tinkerrocket.session.NetworkIdentity.networkIdForName(name)
        _name.value = name
        _id.value = id
        prefs.edit().putString("networkName", name).putInt("networkID", id).apply()
    }
}

/**
 * #633 cold-start resume hint, prefs-backed.
 *
 * Deliberately survives process death — that IS the case it exists for. A
 * crash, an OEM kill or a phone reboot must still resume; only an explicit
 * user disconnect clears it (FleetManager owns that rule).
 */
private class PrefsLastSessionStore(app: Application) : LastSessionStore {
    private val prefs = app.getSharedPreferences("last_session", Context.MODE_PRIVATE)

    override fun saveLastConnected(deviceId: String, name: String) {
        prefs.edit().putString("device_id", deviceId).putString("name", name).apply()
    }

    override fun clearLastConnected() {
        prefs.edit().remove("device_id").remove("name").apply()
    }

    override fun loadLastConnected(): LastSessionStore.LastConnected? {
        val id = prefs.getString("device_id", null) ?: return null
        return LastSessionStore.LastConnected(id, prefs.getString("name", null) ?: "rocket")
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

/**
 * App-wide display-unit setting (iOS #160 twin).  Same key ("unitSystem") and
 * raw values ("metric"/"imperial") as the iOS @AppStorage, so the parity
 * ledger's key inventory stays one list.  StateFlow so Compose recomposes on
 * the spot — settings self-apply, never behind an Apply button (#144).
 */
class UnitStore(app: Application) {
    private val prefs = app.getSharedPreferences("display_units", Context.MODE_PRIVATE)

    private val _system = kotlinx.coroutines.flow.MutableStateFlow(
        when (prefs.getString("unitSystem", "metric")) {
            "imperial" -> com.tinkerbug.tinkerrocket.session.UnitSystem.IMPERIAL
            else -> com.tinkerbug.tinkerrocket.session.UnitSystem.METRIC
        },
    )
    val system: kotlinx.coroutines.flow.StateFlow<com.tinkerbug.tinkerrocket.session.UnitSystem> =
        _system

    fun set(value: com.tinkerbug.tinkerrocket.session.UnitSystem) {
        _system.value = value
        prefs.edit().putString(
            "unitSystem",
            if (value == com.tinkerbug.tinkerrocket.session.UnitSystem.IMPERIAL) {
                "imperial"
            } else {
                "metric"
            },
        ).apply()
    }
}

class TinkerRocketApp : Application() {
    lateinit var container: AppContainer
        private set

    override fun onCreate() {
        super.onCreate()
        container = AppContainer(this)
    }
}
