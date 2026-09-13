package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.provider.Settings
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

    // #1413: whether the system will still show a prompt. Android silently
    // auto-denies a permanently-denied permission, so after the second refusal
    // the Grant button did nothing at all and said nothing about why. When this
    // is true the screen sends the user to Settings instead.
    private val permissionPermanentlyDenied = mutableStateOf(false)

    private fun refreshPermissionState() {
        permissionsGranted.value = bleGranted()
        permissionPermanentlyDenied.value = !permissionsGranted.value &&
            !shouldShowRequestPermissionRationale(Manifest.permission.BLUETOOTH_SCAN)
    }

    /** This app's own page in Settings, where its permissions live. */
    private fun openAppSettings() {
        startActivity(
            Intent(
                Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                Uri.fromParts("package", packageName, null),
            ),
        )
    }

    private fun bleGranted(): Boolean =
        listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
            .all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }

    private val requestPermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
            // shouldShowRequestPermissionRationale is only meaningful AFTER a
            // refusal, so this is the moment to read it (#1413).
            refreshPermissionState()
        }

    // #1413: granting in Settings and coming back used to leave the user
    // stranded on the permission screen — permissionsGranted was only ever read
    // in onCreate and in the launcher callback, and neither runs on that
    // return. Re-read it on every resume so the app recovers without a relaunch.
    override fun onResume() {
        super.onResume()
        refreshPermissionState()
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
                val unitSystem by container.units.system.collectAsState()
                androidx.compose.runtime.CompositionLocalProvider(
                    com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem provides unitSystem,
                    LocalOfflineTileCache provides container.tileCache,   // #1092 item 1
                ) {
                Surface(Modifier.fillMaxSize().statusBarsPadding()) {
                    // #1057: the scanner is not reachable until a network name
                    // exists. Without it the app sits on network ID 0 — the
                    // firmware's factory default AND the app's "unset"
                    // sentinel — so it pushes nothing to a new device and
                    // every mismatch surface stays gated off, which is
                    // invisible until the day an iOS-provisioned rocket has to
                    // talk to an Android-set-up base station.
                    val onboarded by container.networkStore.onboarded.collectAsState()
                    if (!onboarded) {
                        OnboardingScreen(onContinue = { container.networkStore.setNetwork(it) })
                        return@Surface
                    }
                    val granted by permissionsGranted
                    var demoFleet by remember {
                        mutableStateOf<FleetManager<DeviceSession>?>(null)
                    }
                    val fleet = demoFleet ?: container.fleet

                    val devices by fleet.devices.collectAsState()
                    val active by fleet.activeDeviceId.collectAsState()
                    val activeDevice = active?.let { devices[it] }

                    // #1057: first-connect provisioning. The identity readback
                    // lands ~1 s after connect and carries the board's unit id;
                    // the registry says whether this app has ever set that
                    // board up. Both halves already existed — isProvisioned /
                    // markProvisioned had no callers outside their unit tests.
                    val provisioningIdentity by (
                        activeDevice?.session?.identity
                            ?: kotlinx.coroutines.flow.MutableStateFlow(
                                com.tinkerbug.tinkerrocket.session.DeviceIdentity(),
                            )
                        ).collectAsState()
                    var provisioningDismissedFor by remember { mutableStateOf<String?>(null) }
                    val appNetworkId by container.networkStore.id.collectAsState()
                    val needsProvisioning = activeDevice != null &&
                        provisioningIdentity.unitId.isNotEmpty() &&
                        provisioningIdentity.unitId != provisioningDismissedFor &&
                        !container.knownDevices.isProvisioned(provisioningIdentity.unitId)
                    if (needsProvisioning && activeDevice != null) {
                        DeviceProvisioningDialog(
                            unitId = provisioningIdentity.unitId,
                            initialName = provisioningIdentity.unitName
                                .ifEmpty { activeDevice.advertisedName },
                            initialRocketId = provisioningIdentity.rocketId ?: 1,
                            isBaseStation = activeDevice.session.isBaseStation,
                            appNetworkId = appNetworkId,
                            store = container.knownDevices,
                            pusher = activeDevice.session,
                            onDone = { provisioningDismissedFor = provisioningIdentity.unitId },
                        )
                    }

                    // #1063: OBSERVE the session's state, don't just read a
                    // plain map. runningOta() over a non-snapshot mutableMapOf
                    // did not invalidate this branch when the session left
                    // isRunning, so a failed OTA kept OtaProgressScreen up
                    // with the reason it wrote never reaching the operator.
                    // Holding the candidate and collecting its state makes the
                    // `when` below re-evaluate on every transition.
                    val otaCandidate = container.runningOta() ?: container.lastOta()
                    val otaState by (otaCandidate?.state
                        ?: kotlinx.coroutines.flow.MutableStateFlow(
                            com.tinkerbug.tinkerrocket.session.OtaSession.State.Idle,
                        )).collectAsState()
                    val otaInFlight = otaCandidate?.takeIf {
                        otaState !is com.tinkerbug.tinkerrocket.session.OtaSession.State.Idle
                    }

                    // Voice and profile sync for the REAL fleet are bound at
                    // process scope in AppContainer, not here: a LaunchedEffect
                    // cannot re-bind them after a mid-flight drop that happens
                    // while the app is backgrounded (paused Recomposer, and the
                    // key reads unchanged once the reconnect ladder refills the
                    // map).  That is #829's mechanism and iOS #989's bug; both
                    // used to live right here.
                    //
                    // The DEMO fleet is a separate FleetManager the container
                    // does not own, so it still binds from the composition —
                    // which is fine, and only fine, because nothing is flying:
                    // demo mode exists to be looked at, and its "flight" ends
                    // when the screen does.
                    if (demoFleet != null) {
                        LaunchedEffect(demoFleet, devices, active) {
                            container.fleetScope.launch {
                                container.routeDeviceBindings(fleet)
                            }
                        }
                    }

                    // #385: keep the screen awake while any device is
                    // connected. This one genuinely IS activity-scoped — the
                    // window flag dies with the window either way.
                    //
                    // #829: starting the foreground service used to ride along
                    // here and must not — a LaunchedEffect cannot restart it
                    // after a mid-flight drop (paused Recomposer, and the key
                    // reads unchanged once the ladder refills the map). That
                    // now lives in AppContainer at process scope, keyed on
                    // fleet.linkActive. The demo fleet is a separate
                    // FleetManager, so container.fleet stays empty in demo
                    // mode and the service still never starts there.
                    LaunchedEffect(devices.isEmpty()) {
                        if (devices.isEmpty()) {
                            window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        } else {
                            window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        }
                    }

                    when {
                        !granted -> PermissionScreen(
                            permanentlyDenied = permissionPermanentlyDenied.value,
                            onRequest = { requestPermissions.launch(blePermissions) },
                            onOpenSettings = { openAppSettings() },
                        )
                        // Simple 2-screen state nav; NavHost lands with the
                        // next batch of destinations (plan §4).
                        activeDevice != null -> {
                            var tab by remember { mutableStateOf(0) }
                            // Which tool sub-screen is open, hoisted out of
                            // DashboardScreen so the top bar can see it. The bar
                            // is drawn ABOVE this `when`, so it stays on screen
                            // over a tool -- and its chevron used to read `tab`,
                            // which is still 0 there, so it took the DISCONNECT
                            // branch. That is the one exit a teardown hook cannot
                            // cover: the GATT goes down before the composition
                            // unwinds, so the cleanup write lands on a dead
                            // transport and the FC stays in servo-test or
                            // mag-cal with nothing on screen to say so.
                            var tool by remember { mutableStateOf<String?>(null) }
                            // An active-device hop swaps the session underneath a
                            // tool screen without disposing it (FleetManager falls
                            // through to a surviving key rather than null), which
                            // would leave the screen driving a different rocket.
                            LaunchedEffect(activeDevice.deviceId) { tool = null }
                            androidx.compose.foundation.layout.Column {
                                // iOS toolbar arrangement (design pass):
                                // chevron + units | book · map · voice · gear.
                                ConnectedTopBar(
                                    tab = tab,
                                    toolOpen = tool != null,
                                    onCloseTool = { tool = null },
                                    // Closing the tool on a tab change is load
                                    // bearing now that `tool` outlives the tab:
                                    // otherwise returning to tab 0 would re-enter
                                    // the tool and re-fire its entry command,
                                    // re-arming servo test after the stop went out.
                                    onTab = { tool = null; tab = it },
                                    onDisconnect = {
                                        fleet.disconnect(activeDevice.deviceId)
                                        demoFleet = null
                                    },
                                    unitStore = container.units,
                                    container = container,
                                )
                                when (tab) {
                                    0 -> DashboardScreen(
                                        device = activeDevice,
                                        demo = demoFleet != null,
                                        syncer = container.syncer,
                                        phoneLocation = container.phoneLocation,
                                        profileStore = container.profileStore,
                                        container = container,
                                        tool = tool,
                                        onTool = { tool = it },
                                        // Dashboard advisories that point at a
                                        // setting (iOS opens the settings
                                        // sheet); same tab-change discipline as
                                        // onTab above — close any open tool.
                                        onOpenSettings = { tool = null; tab = 3 },
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
                                        deviceKey = activeDevice.deviceId,   // #1056: reset only for another device
                                    )
                                    else -> SettingsScreen(
                                        store = container.profileStore,
                                        syncer = container.syncer,
                                        fleetScope = container.fleetScope,
                                        session = activeDevice.session,
                                        preflight = container.preflightStore,
                                        // #624: lets the Network section fix a
                                        // drifted device ID without first
                                        // disconnecting to reach My Devices.
                                        network = container.networkStore,
                                        knownDevices = container.knownDevices,
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
                            var showPreflight by remember { mutableStateOf(false) }
                            if (showPreflight) {
                                // Master pre-flight checklist editor — no
                                // device needed (iOS front-page entry).
                                PreflightMasterScreen(
                                    preflight = container.preflightStore,
                                    profiles = container.profileStore,
                                    fleetScope = container.fleetScope,
                                    onBack = { showPreflight = false },
                                )
                            } else if (showDriftCast) {
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
                                    onPreflight = { showPreflight = true },
                                    unitStore = container.units,
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
}
