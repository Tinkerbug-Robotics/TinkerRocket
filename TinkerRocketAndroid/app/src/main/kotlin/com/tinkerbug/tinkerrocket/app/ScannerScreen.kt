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
fun ScannerScreen(fleet: FleetManager<DeviceSession>, onDemo: () -> Unit) {
    val discovered by fleet.discoveredDevices.collectAsState()
    val scanning by fleet.isScanning.collectAsState()
    val status by fleet.statusMessage.collectAsState()

    Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text("TinkerRocket", style = MaterialTheme.typography.headlineMedium)
        Row(verticalAlignment = Alignment.CenterVertically) {
            Button(onClick = { fleet.scan(userInitiated = true) }, enabled = !scanning) {
                Text(if (scanning) "Scanning…" else "Scan")
            }
            if (scanning) CircularProgressIndicator(Modifier.padding(start = 12.dp))
            Spacer(Modifier.weight(1f))
            OutlinedButton(onClick = onDemo) { Text("Demo") }
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
