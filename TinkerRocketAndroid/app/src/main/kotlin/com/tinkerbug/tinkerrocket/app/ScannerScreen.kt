package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Button
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetManager

@Composable
fun ScannerScreen(
    fleet: FleetManager<DeviceSession>,
    onDemo: () -> Unit,
    onMyDevices: () -> Unit = {},
    onSavedFlights: () -> Unit = {},
    updateVersion: String? = null,
    onGetUpdate: () -> Unit = {},
) {
    val discovered by fleet.discoveredDevices.collectAsState()
    val scanning by fleet.isScanning.collectAsState()
    val status by fleet.statusMessage.collectAsState()

    Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text("TinkerRocket", style = MaterialTheme.typography.headlineMedium)
        // Update banner (plan §1 sideload loop): informational, never modal —
        // launch-day work must be able to ignore it entirely.
        updateVersion?.let { v ->
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    "Update available: v$v",
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.weight(1f),
                )
                OutlinedButton(onClick = onGetUpdate) { Text("Get") }
            }
        }
        // Primary row = the connected world; the spinner lives INSIDE the
        // Scan button so the row's geometry never changes mid-scan (design
        // pass 2026-07-30: a floating spinner squeezed the row and wrapped
        // a button label — the clipped-Disconnect lesson again).
        Row(verticalAlignment = Alignment.CenterVertically) {
            Button(onClick = { fleet.scan(userInitiated = true) }, enabled = !scanning) {
                if (scanning) {
                    CircularProgressIndicator(
                        Modifier.padding(end = 8.dp).size(16.dp),
                        strokeWidth = 2.dp,
                    )
                }
                Text(if (scanning) "Scanning…" else "Scan")
            }
            Spacer(Modifier.weight(1f))
            OutlinedButton(onClick = onMyDevices) { Text("My Devices") }
        }
        // Secondary row = the no-hardware world: cached flights (#635) and
        // the Virtual Rocket (a FakeFirmware fleet in the real app stack —
        // the no-hardware dev path, doubling as the try-the-app mode).
        // "Virtual Rocket" names the THING; "Simulation" flies the real one.
        Row(verticalAlignment = Alignment.CenterVertically) {
            OutlinedButton(onClick = onSavedFlights) { Text("Saved Flights") }
            Spacer(Modifier.weight(1f))
            OutlinedButton(onClick = onDemo) { Text("Virtual Rocket") }
        }
        Text(status, style = MaterialTheme.typography.bodyMedium)

        LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            items(discovered, key = { it.deviceId }) { dev ->
                Card(Modifier.fillMaxWidth()) {
                    Row(
                        Modifier.fillMaxWidth().padding(16.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Column {
                            Text(dev.name, style = MaterialTheme.typography.titleMedium)
                            Text(
                                "${dev.deviceId}  ${dev.rssi} dBm",
                                style = MaterialTheme.typography.bodySmall,
                            )
                        }
                        Button(onClick = { fleet.connect(dev.deviceId) }) { Text("Connect") }
                    }
                }
            }
        }
    }
}

@Composable
fun PermissionScreen(onRequest: () -> Unit) {
    Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp, Alignment.CenterVertically),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("Bluetooth access needed", style = MaterialTheme.typography.titleLarge)
        Text(
            "TinkerRocket finds and talks to your flight computers over " +
                "Bluetooth. Nearby-devices permission is required.",
            style = MaterialTheme.typography.bodyMedium,
        )
        Button(onClick = onRequest) { Text("Grant") }
    }
}
