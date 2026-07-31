package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.HelpOutline
import androidx.compose.material.icons.automirrored.filled.ListAlt
import androidx.compose.material.icons.filled.Air
import androidx.compose.material.icons.filled.AutoAwesome
import androidx.compose.material.icons.filled.CellTower
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material.icons.filled.Inventory2
import androidx.compose.material.icons.filled.RocketLaunch
import androidx.compose.material.icons.filled.Straighten
import androidx.compose.material3.Button
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.R
import com.tinkerbug.tinkerrocket.app.theme.TrActionButton
import com.tinkerbug.tinkerrocket.app.theme.TrCard
import com.tinkerbug.tinkerrocket.app.theme.TrCompactButton
import com.tinkerbug.tinkerrocket.app.theme.TrShape
import com.tinkerbug.tinkerrocket.app.theme.TrSignalBars
import com.tinkerbug.tinkerrocket.app.theme.TrSpacing
import com.tinkerbug.tinkerrocket.app.theme.TrStatusPill
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.DiscoveredDevice
import com.tinkerbug.tinkerrocket.session.FleetManager
import com.tinkerbug.tinkerrocket.session.UnitSystem

/**
 * The top screen, mirroring the iOS layout (design pass 2026-07-30, iOS =
 * reference): stacked full-width semantic buttons (Saved Flights / My
 * Devices / Reverse Drift Cast), the Scan button + dot-status pill, and the
 * Available Devices card whose rows are the connect controls — with the
 * Virtual Rocket as the last row of the card, because it connects like a
 * device.  Android-only content keeps its place: the update banner (sideload
 * loop, plan §1) sits under the title.
 *
 * Behavior deltas kept on purpose: Android's scan self-terminates on the
 * epoch-guarded window (no Stop toggle — a restarted scan deserves its full
 * 15 s, ledger 2026-07-23), so the Scan button disables while scanning and
 * the pill carries the state.
 */
@Composable
fun ScannerScreen(
    fleet: FleetManager<DeviceSession>,
    onDemo: () -> Unit,
    onMyDevices: () -> Unit = {},
    onSavedFlights: () -> Unit = {},
    onDriftCast: () -> Unit = {},
    updateVersion: String? = null,
    onGetUpdate: () -> Unit = {},
    unitStore: UnitStore? = null,
) {
    val discovered by fleet.discoveredDevices.collectAsState()
    val scanning by fleet.isScanning.collectAsState()
    val status by fleet.statusMessage.collectAsState()
    val tr = TrTheme.colors

    Column(
        Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(TrSpacing.screenPadding),
        verticalArrangement = Arrangement.spacedBy(TrSpacing.stackSpacing),
    ) {
        // iOS toolbar mirror: units menu leading, Tinkerbug branding trailing
        // (#160 — the choice is global and reachable with nothing connected).
        Row(
            Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            unitStore?.let { store ->
                val current by store.system.collectAsState()
                var menuOpen by remember { mutableStateOf(false) }
                Box {
                    IconButton(onClick = { menuOpen = true }) {
                        Icon(
                            Icons.Filled.Straighten, contentDescription = "Display units",
                            tint = TrTheme.colors.savedFlights,
                        )
                    }
                    DropdownMenu(expanded = menuOpen, onDismissRequest = { menuOpen = false }) {
                        listOf(
                            UnitSystem.METRIC to "Metric (SI)",
                            UnitSystem.IMPERIAL to "Imperial",
                        ).forEach { (sys, label) ->
                            DropdownMenuItem(
                                text = { Text(label) },
                                leadingIcon = {
                                    if (current == sys) {
                                        Icon(Icons.Filled.Check, contentDescription = null)
                                    }
                                },
                                onClick = {
                                    store.set(sys)
                                    menuOpen = false
                                },
                            )
                        }
                    }
                }
            }
            Spacer(Modifier.weight(1f))
            // White circle chip like the iOS toolbar — the purple bug is
            // invisible straight on a dark background.
            Box(
                Modifier
                    .size(40.dp)
                    .shadow(2.dp, CircleShape)
                    .background(Color.White, CircleShape),
                contentAlignment = Alignment.Center,
            ) {
                Image(
                    painterResource(R.drawable.tinkerbug_logo),
                    contentDescription = "Tinkerbug Robotics",
                    modifier = Modifier.height(28.dp),
                )
            }
        }

        Text("TinkerRocket", fontSize = 34.sp, fontWeight = FontWeight.Bold)

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

        TrActionButton(
            "Saved Flights", tr.savedFlights, onSavedFlights,
            icon = Icons.Filled.Inventory2,
        )
        TrActionButton(
            "My Devices", tr.myDevices, onMyDevices,
            icon = Icons.AutoMirrored.Filled.ListAlt,
        )
        TrActionButton(
            "Reverse Drift Cast", tr.driftCast, onDriftCast,
            icon = Icons.Filled.Air,
        )

        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(TrSpacing.rowSpacing),
        ) {
            TrCompactButton(
                "Scan", tr.scan,
                onClick = { fleet.scan(userInitiated = true) },
                enabled = !scanning,
            )
            TrStatusPill(
                dotColor = if (scanning) tr.statusScanning else tr.statusIdle,
                text = status,
                modifier = Modifier.weight(1f),
            )
        }

        TrCard {
            Text("Available Devices", style = MaterialTheme.typography.titleMedium)

            if (discovered.isEmpty()) {
                if (scanning) {
                    Column(
                        Modifier.fillMaxWidth().padding(vertical = 20.dp),
                        horizontalAlignment = Alignment.CenterHorizontally,
                        verticalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        CircularProgressIndicator(Modifier.size(24.dp), strokeWidth = 2.dp)
                        Text(
                            "Searching…",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                } else {
                    Text(
                        "No devices found. Tap Scan to search.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.padding(vertical = 10.dp),
                    )
                }
            }

            discovered.forEach { dev ->
                DeviceRow(dev) { fleet.connect(dev.deviceId) }
            }

            // Virtual Rocket: the no-hardware world, last row of the device
            // card because it connects like a device (iOS placement).  Names
            // the THING; "Simulation" flies the real one.
            Row(
                Modifier
                    .fillMaxWidth()
                    .background(
                        tr.cardSecondary.copy(alpha = 0.6f),
                        RoundedCornerShape(TrShape.radiusButton),
                    )
                    .clickable(onClick = onDemo)
                    .padding(vertical = 8.dp, horizontal = 12.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Icon(
                    Icons.Filled.AutoAwesome, contentDescription = null,
                    modifier = Modifier.size(22.dp),
                    tint = MaterialTheme.colorScheme.onSurface,
                )
                Spacer(Modifier.width(12.dp))
                Text("Virtual Rocket", style = MaterialTheme.typography.bodyMedium)
            }
        }
    }
}

/** iOS DevicePickerView row: type icon, name + role, signal bars; the row
 *  itself is the connect control. */
@Composable
private fun DeviceRow(
    dev: DiscoveredDevice,
    onConnect: () -> Unit,
) {
    val tr = TrTheme.colors
    // Registry hint outranks the name heuristic, matching the iOS scan site
    // (renamed devices advertise raw names — only the registry knows them).
    val type = dev.knownType ?: BleDeviceType.fromName(dev.name)
    Row(
        Modifier
            .fillMaxWidth()
            .background(tr.cardSecondary, RoundedCornerShape(TrShape.radiusButton))
            .clickable(onClick = onConnect)
            .padding(vertical = 8.dp, horizontal = 12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            when (type) {
                BleDeviceType.ROCKET -> Icons.Filled.RocketLaunch
                BleDeviceType.BASE_STATION -> Icons.Filled.CellTower
                BleDeviceType.UNKNOWN -> Icons.AutoMirrored.Filled.HelpOutline
            },
            contentDescription = null,
            modifier = Modifier.size(26.dp),
            tint = MaterialTheme.colorScheme.onSurface,
        )
        Spacer(Modifier.width(12.dp))
        Column(Modifier.weight(1f)) {
            Text(
                dev.name,
                style = MaterialTheme.typography.bodyLarge,
                fontWeight = FontWeight.Medium,
            )
            Text(
                when (type) {
                    BleDeviceType.ROCKET -> "Rocket"
                    BleDeviceType.BASE_STATION -> "Base Station"
                    BleDeviceType.UNKNOWN -> "TinkerRocket device"
                },
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        TrSignalBars(dev.rssi)
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
