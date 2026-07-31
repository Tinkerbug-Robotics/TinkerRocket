package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.material3.Surface
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import com.tinkerbug.tinkerrocket.app.theme.TinkerRocketTheme
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetManager
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {

    private val blePermissions = buildList {
        add(Manifest.permission.BLUETOOTH_SCAN)
        add(Manifest.permission.BLUETOOTH_CONNECT)
        // Location rides the same prompt (direction-to-rocket arrow, map
        // phone dot) but never GATES the app — BLE does.
        add(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= 33) add(Manifest.permission.POST_NOTIFICATIONS)
    }.toTypedArray()

    private val permissionsGranted = mutableStateOf(false)

    private fun bleGranted(): Boolean =
        listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
            .all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }

    private val requestPermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
            permissionsGranted.value = bleGranted()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val container = (application as TinkerRocketApp).container

        permissionsGranted.value = bleGranted()
        if (!permissionsGranted.value) {
            requestPermissions.launch(blePermissions)
        } else {
            // #633: resume whatever we were connected to before the app went
            // away.  Android has no CoreBluetooth state restoration, so a
            // crash, OEM kill or reboot otherwise left the user tapping Scan
            // then Connect.  No-ops when there's nothing to resume, and only
            // ever after permissions are already granted — a resume must never
            // be what triggers a permission prompt.
            container.fleet.resumeLastSession()
        }

        // Update check: once per app start, one network hit per day, silent on
        // every failure (no signal at a launch site is normal). Fleet scope so
        // it survives the activity; the banner reads the StateFlow whenever.
        container.fleetScope.launch { container.updateChecker.checkThrottled() }

        setContent {
            // Design-token theme (iOS reference values) — see app/theme/.
            TinkerRocketTheme {
                Surface(Modifier.fillMaxSize().statusBarsPadding()) {
                    val granted by permissionsGranted
                    var demoFleet by remember {
                        mutableStateOf<FleetManager<DeviceSession>?>(null)
                    }
                    val fleet = demoFleet ?: container.fleet

                    val devices by fleet.devices.collectAsState()
                    val active by fleet.activeDeviceId.collectAsState()
                    val activeDevice = active?.let { devices[it] }

                    // Re-checked on every recomposition; the fleet's own state
                    // changes tick this often enough during a flash.
                    val otaInFlight = container.runningOta()

                    // Syncer follows the active device (#132); attach is
                    // idempotent per (session, role) and hops to the fleet
                    // dispatcher (single-writer contract).  Demo attaches
                    // too — the push lands harmlessly in FakeFirmware.
                    LaunchedEffect(activeDevice?.session) {
                        val session = activeDevice?.session
                        container.fleetScope.launch {
                            if (session != null) {
                                container.syncer.attach(session, container.profileStore)
                            } else {
                                container.syncer.detach()
                            }
                        }
                    }

                    // Voice follows iOS attachActiveDevice (#375, #390): the
                    // first connected direct rocket when one exists (full
                    // frame rate), else the foreground base station — whose
                    // stream is pinned to its focused rocket, so callouts
                    // never interleave two rockets.  Keyed on the device map
                    // so reconnects (new session objects) re-attach.
                    LaunchedEffect(devices) {
                        container.fleetScope.launch {
                            val direct = devices.values.firstOrNull {
                                it.session.isConnected.value &&
                                    it.deviceType ==
                                    com.tinkerbug.tinkerrocket.session.BleDeviceType.ROCKET
                            }
                            val voice = direct ?: fleet.foregroundBaseStation()
                            for (d in devices.values) {
                                if (d !== voice) d.session.telemetryAnnouncer = null
                            }
                            voice?.session?.telemetryAnnouncer = container.announcer
                        }
                    }

                    // #385: keep the screen awake while any device is
                    // connected; FGS pins the process on real connections
                    // (BLUETOOTH_CONNECT is granted by the time one exists).
                    LaunchedEffect(devices.isEmpty(), demoFleet) {
                        if (devices.isEmpty()) {
                            window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        } else {
                            window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                            if (demoFleet == null) FleetService.start(this@MainActivity)
                        }
                    }

                    when {
                        !granted -> PermissionScreen { requestPermissions.launch(blePermissions) }
                        // Simple 2-screen state nav; NavHost lands with the
                        // next batch of destinations (plan §4).
                        activeDevice != null -> {
                            var tab by remember { mutableStateOf(0) }
                            androidx.compose.foundation.layout.Column {
                                androidx.compose.foundation.layout.Row(
                                    Modifier.fillMaxWidth(),
                                    horizontalArrangement =
                                        androidx.compose.foundation.layout.Arrangement.Center,
                                ) {
                                    androidx.compose.material3.TextButton(onClick = { tab = 0 }) {
                                        androidx.compose.material3.Text(
                                            if (tab == 0) "● Dashboard" else "Dashboard",
                                        )
                                    }
                                    androidx.compose.material3.TextButton(onClick = { tab = 1 }) {
                                        androidx.compose.material3.Text(
                                            if (tab == 1) "● Files" else "Files",
                                        )
                                    }
                                    androidx.compose.material3.TextButton(onClick = { tab = 2 }) {
                                        androidx.compose.material3.Text(
                                            if (tab == 2) "● Map" else "Map",
                                        )
                                    }
                                    androidx.compose.material3.TextButton(onClick = { tab = 3 }) {
                                        androidx.compose.material3.Text(
                                            if (tab == 3) "● Settings" else "Settings",
                                        )
                                    }
                                }
                                when (tab) {
                                    0 -> DashboardScreen(
                                        device = activeDevice,
                                        demo = demoFleet != null,
                                        syncer = container.syncer,
                                        phoneLocation = container.phoneLocation,
                                        profileStore = container.profileStore,
                                        container = container,
                                        onDisconnect = {
                                            fleet.disconnect(activeDevice.deviceId)
                                            demoFleet = null
                                        },
                                    )
                                    1 -> FilesScreen(
                                        device = activeDevice,
                                        fleetScope = container.fleetScope,
                                    )
                                    2 -> MapTab(
                                        container = container,
                                        session = activeDevice.session,
                                    )
                                    else -> SettingsScreen(
                                        store = container.profileStore,
                                        syncer = container.syncer,
                                        fleetScope = container.fleetScope,
                                        session = activeDevice.session,
                                    )
                                }
                            }
                        }
                        // A flash in flight outranks the scanner: the device
                        // vanishes from the fleet while it reboots, and the
                        // user must not lose sight of an update in progress.
                        activeDevice == null && otaInFlight != null ->
                            OtaProgressScreen(otaInFlight)

                        else -> {
                            var showMyDevices by remember { mutableStateOf(false) }
                            var showSavedFlights by remember { mutableStateOf(false) }
                            var showDriftCast by remember { mutableStateOf(false) }
                            if (showDriftCast) {
                                // Standalone wind/trajectory planner — runs
                                // with no device, like the iOS top-screen
                                // entry (#42; design pass 2026-07-30 promoted
                                // it out of the map screen).
                                DriftCastScreen(
                                    container = container,
                                    onBack = { showDriftCast = false },
                                )
                            } else if (showSavedFlights) {
                                // #635: local cache only — no session needed.
                                SavedFlightsScreen(onBack = { showSavedFlights = false })
                            } else if (showMyDevices) {
                                DeviceManagerScreen(
                                    store = container.knownDevices,
                                    fleet = container.fleet,
                                    network = container.networkStore,
                                    fleetScope = container.fleetScope,
                                    onBack = { showMyDevices = false },
                                )
                            } else {
                                val update by container.updateChecker.available.collectAsState()
                                ScannerScreen(
                                    fleet = container.fleet,
                                    onDemo = {
                                        demoFleet = buildDemoFleet(
                                            this@MainActivity, container.fleetScope,
                                        ).also {
                                            it.scan(userInitiated = true)
                                            it.connect("demo:01")
                                        }
                                    },
                                    onMyDevices = { showMyDevices = true },
                                    onSavedFlights = { showSavedFlights = true },
                                    onDriftCast = { showDriftCast = true },
                                    updateVersion = update?.versionName,
                                    onGetUpdate = {
                                        update?.let {
                                            startActivity(
                                                android.content.Intent(
                                                    android.content.Intent.ACTION_VIEW,
                                                    android.net.Uri.parse(it.htmlUrl),
                                                ),
                                            )
                                        }
                                    },
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}
