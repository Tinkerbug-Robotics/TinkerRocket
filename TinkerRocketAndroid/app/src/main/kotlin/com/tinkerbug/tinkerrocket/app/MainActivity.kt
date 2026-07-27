package com.tinkerbug.tinkerrocket.app

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat

class MainActivity : ComponentActivity() {

    private val blePermissions = arrayOf(
        Manifest.permission.BLUETOOTH_SCAN,
        Manifest.permission.BLUETOOTH_CONNECT,
    )

    private val permissionsGranted = mutableStateOf(false)

    private val requestPermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { grants ->
            permissionsGranted.value = grants.values.all { it }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val container = (application as TinkerRocketApp).container

        permissionsGranted.value = blePermissions.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
        if (!permissionsGranted.value) requestPermissions.launch(blePermissions)

        setContent {
            val scheme = if (isSystemInDarkTheme()) darkColorScheme() else lightColorScheme()
            MaterialTheme(colorScheme = scheme) {
                Surface(Modifier.fillMaxSize()) {
                    val granted by permissionsGranted
                    val devices by container.fleet.devices.collectAsState()
                    val active by container.fleet.activeDeviceId.collectAsState()
                    val activeDevice = active?.let { devices[it] }

                    when {
                        !granted -> PermissionScreen { requestPermissions.launch(blePermissions) }
                        // Simple 2-screen state nav; NavHost lands with the
                        // next batch of destinations (plan §4).
                        activeDevice != null -> DashboardScreen(
                            device = activeDevice,
                            onDisconnect = { container.fleet.disconnect(activeDevice.deviceId) },
                        )
                        else -> ScannerScreen(fleet = container.fleet)
                    }
                }
            }
        }
    }
}
