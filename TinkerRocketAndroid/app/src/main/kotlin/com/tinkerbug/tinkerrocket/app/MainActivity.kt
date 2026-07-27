package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetManager

class MainActivity : ComponentActivity() {

    private val blePermissions = buildList {
        add(Manifest.permission.BLUETOOTH_SCAN)
        add(Manifest.permission.BLUETOOTH_CONNECT)
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
        if (!permissionsGranted.value) requestPermissions.launch(blePermissions)

        setContent {
            val scheme = if (isSystemInDarkTheme()) darkColorScheme() else lightColorScheme()
            MaterialTheme(colorScheme = scheme) {
                Surface(Modifier.fillMaxSize().statusBarsPadding()) {
                    val granted by permissionsGranted
                    var demoFleet by remember {
                        mutableStateOf<FleetManager<DeviceSession>?>(null)
                    }
                    val fleet = demoFleet ?: container.fleet

                    val devices by fleet.devices.collectAsState()
                    val active by fleet.activeDeviceId.collectAsState()
                    val activeDevice = active?.let { devices[it] }

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
                                }
                                if (tab == 0) {
                                    DashboardScreen(
                                        device = activeDevice,
                                        demo = demoFleet != null,
                                        onDisconnect = {
                                            fleet.disconnect(activeDevice.deviceId)
                                            demoFleet = null
                                        },
                                    )
                                } else {
                                    FilesScreen(
                                        device = activeDevice,
                                        fleetScope = container.fleetScope,
                                    )
                                }
                            }
                        }
                        else -> ScannerScreen(
                            fleet = container.fleet,
                            onDemo = {
                                demoFleet = buildDemoFleet(
                                    this@MainActivity, container.fleetScope,
                                ).also {
                                    it.scan(userInitiated = true)
                                    it.connect("demo:01")
                                }
                            },
                        )
                    }
                }
            }
        }
    }
}
