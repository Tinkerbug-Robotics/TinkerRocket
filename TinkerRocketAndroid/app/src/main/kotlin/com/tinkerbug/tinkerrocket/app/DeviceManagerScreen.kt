package com.tinkerbug.tinkerrocket.app

import android.text.format.DateUtils
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetManager
import com.tinkerbug.tinkerrocket.session.KnownDevice
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.NetworkIdentity
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch

/**
 * "My Devices" — port of iOS DeviceManagerView (#150 / PR #547): rename
 * devices, fix rocket IDs, and keep the whole fleet on the app's network ID.
 * Reachable without a connection: edits to offline devices queue in the
 * registry and push automatically on that device's next connect.
 *
 * The #150 copy discipline holds: the same condition is described in the
 * same words as iOS (NetworkCopy) — mismatched wording is how the nid drift
 * stayed undebuggable.
 */

// iOS NetworkCopy — keep these strings in lockstep with DeviceTypeIcon.swift.
private const val SAME_NETWORK_EXPLAINER =
    "Devices only hear each other on the same network ID."
private const val DEVICE_MISMATCH_WARNING =
    "This device is on a different network ID than the app expects — it can't hear your other devices."

@Composable
fun DeviceManagerScreen(
    store: KnownDeviceStore,
    fleet: FleetManager<DeviceSession>,
    network: AppNetworkStore,
    fleetScope: CoroutineScope,
    onBack: () -> Unit,
) {
    val devices by store.devices.collectAsState()
    val fleetDevices by fleet.devices.collectAsState()
    val networkName by network.name.collectAsState()
    val networkId by network.id.collectAsState()
    var detailUnit by remember { mutableStateOf<String?>(null) }

    // Live BLE pusher when a device is currently connected — null queues.
    fun pusher(unitID: String): DeviceSession? =
        fleetDevices.values.firstOrNull {
            it.session.isConnected.value && it.session.identity.value.unitId == unitID
        }?.session

    detailUnit?.let { unit ->
        KnownDeviceDetail(
            unitID = unit,
            store = store,
            networkName = networkName,
            networkId = networkId,
            connected = pusher(unit) != null,
            fleetScope = fleetScope,
            pusher = ::pusher,
            onBack = { detailUnit = null },
        )
        return
    }

    val mismatched = if (networkId > 0) devices.filter { it.effectiveNetworkID != networkId } else emptyList()
    var editingNetwork by remember { mutableStateOf(false) }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Back") }
            Text("My Devices", style = MaterialTheme.typography.titleLarge)
        }

        // ── Network ──────────────────────────────────────────────────────
        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Network", style = MaterialTheme.typography.titleMedium)
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Text(networkName.ifEmpty { "Not set" }, style = MaterialTheme.typography.bodyLarge)
                    Text(
                        "ID: $networkId",
                        style = MaterialTheme.typography.bodyMedium,
                        fontFamily = FontFamily.Monospace,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                TextButton(onClick = { editingNetwork = true }) { Text("Change network name…") }
                if (mismatched.isNotEmpty()) {
                    OutlinedButton(onClick = {
                        fleetScope.launch {
                            mismatched.forEach { rec ->
                                store.setNetworkId(networkId, rec.unitID, pusher(rec.unitID))
                            }
                        }
                    }) { Text("Move all devices to ID $networkId") }
                }
                Text(
                    if (mismatched.isNotEmpty()) {
                        val n = mismatched.size
                        "$n device${if (n == 1) " is" else "s are"} on a different network ID " +
                            "and can't hear the others. Moving them pushes connected devices " +
                            "right away and queues the rest for their next connect."
                    } else {
                        "$SAME_NETWORK_EXPLAINER Edits to offline devices are queued and " +
                            "apply automatically the next time each one connects."
                    },
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }

        // ── Devices ──────────────────────────────────────────────────────
        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Devices", style = MaterialTheme.typography.titleMedium)
                if (devices.isEmpty()) {
                    Text(
                        "No devices yet. Scan and connect to a rocket or base station " +
                            "and it will appear here.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                devices.forEach { rec ->
                    DeviceRow(
                        rec = rec,
                        connected = pusher(rec.unitID) != null,
                        mismatch = networkId > 0 && rec.effectiveNetworkID != networkId,
                        onClick = { detailUnit = rec.unitID },
                    )
                }
            }
        }
    }

    if (editingNetwork) {
        NetworkNameDialog(
            initial = networkName,
            onDismiss = { editingNetwork = false },
            onConfirm = { name ->
                editingNetwork = false
                if (name.isNotBlank()) network.setNetwork(name.trim())
            },
        )
    }
}

// ── Rows / dialogs ──────────────────────────────────────────────────────

@Composable
private fun DeviceRow(rec: KnownDevice, connected: Boolean, mismatch: Boolean, onClick: () -> Unit) {
    Row(
        Modifier.fillMaxWidth().clickable(onClick = onClick).padding(vertical = 6.dp),
        horizontalArrangement = Arrangement.spacedBy(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        DeviceTypeBadge(rec.deviceType)
        Column(Modifier.weight(1f)) {
            Text(
                rec.effectiveName.ifEmpty { "Unnamed" },
                style = MaterialTheme.typography.bodyLarge,
                color = if (rec.effectiveName.isEmpty()) {
                    MaterialTheme.colorScheme.onSurfaceVariant
                } else {
                    MaterialTheme.colorScheme.onSurface
                },
            )
            Text(
                deviceSubtitle(rec),
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Column(horizontalAlignment = Alignment.End) {
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
                if (connected) Box(Modifier.size(8.dp).background(Color(0xFF43A047), CircleShape))
                if (mismatch) Text("⚠", color = Color(0xFFFFA000))
                Text(
                    "ID ${rec.effectiveNetworkID}",
                    style = MaterialTheme.typography.bodySmall,
                    fontFamily = FontFamily.Monospace,
                    color = if (mismatch) Color(0xFFFFA000) else MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            if (rec.hasPendingChanges) {
                Text("queued", style = MaterialTheme.typography.labelSmall, color = Color(0xFF1E88E5))
            }
        }
    }
}

private fun deviceSubtitle(rec: KnownDevice): String {
    val parts = mutableListOf<String>()
    when (rec.deviceType) {
        BleDeviceType.ROCKET -> {
            parts += "Rocket"
            if (rec.effectiveRocketID > 0) parts += "R${rec.effectiveRocketID}"
        }
        BleDeviceType.BASE_STATION -> parts += "Base Station"
        else -> parts += "Unknown type"
    }
    parts += rec.unitID
    return parts.joinToString(" · ")
}

@Composable
private fun DeviceTypeBadge(type: BleDeviceType) {
    val (letter, color) = when (type) {
        BleDeviceType.ROCKET -> "R" to MaterialTheme.colorScheme.primary
        BleDeviceType.BASE_STATION -> "B" to Color(0xFF00838F)
        else -> "?" to MaterialTheme.colorScheme.onSurfaceVariant
    }
    Box(
        Modifier.size(32.dp).background(color.copy(alpha = 0.15f), CircleShape),
        contentAlignment = Alignment.Center,
    ) {
        Text(letter, style = MaterialTheme.typography.titleSmall, color = color)
    }
}

@Composable
private fun NetworkNameDialog(initial: String, onDismiss: () -> Unit, onConfirm: (String) -> Unit) {
    var text by remember { mutableStateOf(initial) }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Network name") },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                OutlinedTextField(value = text, onValueChange = { text = it }, singleLine = true)
                // Live wire-ID preview, same as iOS onboarding — the ID is
                // what actually goes over the air.
                if (text.trim().isNotEmpty()) {
                    Text(
                        "Network ID: ${NetworkIdentity.networkIdForName(text.trim())}",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        },
        confirmButton = { TextButton(onClick = { onConfirm(text) }) { Text("Save") } },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Cancel") } },
    )
}

// ── Detail ──────────────────────────────────────────────────────────────

@Composable
private fun KnownDeviceDetail(
    unitID: String,
    store: KnownDeviceStore,
    networkName: String,
    networkId: Int,
    connected: Boolean,
    fleetScope: CoroutineScope,
    pusher: (String) -> DeviceSession?,
    onBack: () -> Unit,
) {
    val devices by store.devices.collectAsState()
    val rec = devices.firstOrNull { it.unitID == unitID }
    var showRename by remember { mutableStateOf(false) }
    var showForget by remember { mutableStateOf(false) }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Devices") }
            Text(
                rec?.effectiveName?.ifEmpty { "Device" } ?: "Device",
                style = MaterialTheme.typography.titleLarge,
            )
        }
        if (rec == null) {
            // Forgotten while open — nothing left to show.
            Text("Device removed.", color = MaterialTheme.colorScheme.onSurfaceVariant)
            return
        }
        val mismatch = networkId > 0 && rec.effectiveNetworkID != networkId

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), verticalAlignment = Alignment.CenterVertically) {
                    DeviceTypeBadge(rec.deviceType)
                    Column {
                        Text(rec.effectiveName.ifEmpty { "Unnamed" }, style = MaterialTheme.typography.titleMedium)
                        Row(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
                            Box(
                                Modifier.size(8.dp).background(
                                    if (connected) Color(0xFF43A047) else MaterialTheme.colorScheme.surfaceVariant,
                                    CircleShape,
                                ),
                            )
                            Text(
                                if (connected) "Connected" else "Not connected",
                                style = MaterialTheme.typography.bodySmall,
                                color = MaterialTheme.colorScheme.onSurfaceVariant,
                            )
                        }
                    }
                }
                TextButton(onClick = { showRename = true }) { Text("Rename…") }
                rec.pendingName?.let {
                    Text(
                        "Rename to “$it” is queued — it applies the next time this device connects.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }

        // Positively-identified rockets only: a legacy-migrated record is
        // UNKNOWN until its next readback and may really be a base station.
        if (rec.deviceType == BleDeviceType.ROCKET) {
            Card {
                Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text("Rocket ID", style = MaterialTheme.typography.titleMedium)
                    Row(
                        Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            if (rec.effectiveRocketID > 0) {
                                "ID: ${rec.effectiveRocketID}" +
                                    if (rec.pendingRocketID != null) "  (queued)" else ""
                            } else {
                                "ID: not set"
                            },
                            style = MaterialTheme.typography.bodyLarge,
                        )
                        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                            OutlinedButton(onClick = { bumpRocketId(rec, -1, store, fleetScope, pusher) }) { Text("−") }
                            OutlinedButton(onClick = { bumpRocketId(rec, +1, store, fleetScope, pusher) }) { Text("+") }
                        }
                    }
                    Text(
                        if (connected) {
                            "Unique ID within your network (1–254). Each rocket needs a different ID."
                        } else {
                            "Unique ID within your network (1–254). Changes apply on the next connect."
                        },
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Network", style = MaterialTheme.typography.titleMedium)
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Text("App network")
                    Text(
                        "${networkName.ifEmpty { "Not set" }}  ID: $networkId",
                        fontFamily = FontFamily.Monospace,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Text("This device")
                    Text(
                        (if (mismatch) "⚠ " else "") +
                            "ID: ${rec.effectiveNetworkID}" +
                            if (rec.pendingNetworkID != null) "  (queued)" else "",
                        fontFamily = FontFamily.Monospace,
                        color = if (mismatch) Color(0xFFFFA000) else MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                if (mismatch && networkId > 0) {
                    OutlinedButton(onClick = {
                        fleetScope.launch { store.setNetworkId(networkId, unitID, pusher(unitID)) }
                    }) { Text("Set to “$networkName” (ID $networkId)") }
                }
                Text(
                    if (mismatch) {
                        DEVICE_MISMATCH_WARNING + if (connected) {
                            " Tap to sync it now."
                        } else {
                            " Syncing queues the change for its next connect."
                        }
                    } else {
                        SAME_NETWORK_EXPLAINER
                    },
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Info", style = MaterialTheme.typography.titleMedium)
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Text("Hardware ID")
                    Text(rec.unitID, fontFamily = FontFamily.Monospace, color = MaterialTheme.colorScheme.onSurfaceVariant)
                }
                rec.lastSeenEpochMillis?.let { seen ->
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text("Last seen")
                        Text(
                            if (connected) {
                                "Now"
                            } else {
                                DateUtils.getRelativeTimeSpanString(seen).toString()
                            },
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                }
            }
        }

        if (rec.hasPendingChanges) {
            Card {
                Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    TextButton(onClick = { fleetScope.launch { store.clearPending(unitID) } }) {
                        Text("Cancel queued changes")
                    }
                    Text(
                        "Queued changes are pushed automatically the next time this device connects.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                TextButton(onClick = { showForget = true }) {
                    Text("Forget This Device", color = MaterialTheme.colorScheme.error)
                }
                Text(
                    "The device itself keeps its name and IDs — forgetting only removes it " +
                        "from this app, and it will be treated as new the next time it connects.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }

    if (showRename && rec != null) {
        var text by remember { mutableStateOf(rec.effectiveName) }
        AlertDialog(
            onDismissRequest = { showRename = false },
            title = { Text("Rename Device") },
            text = {
                Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    OutlinedTextField(
                        value = text,
                        onValueChange = { text = it },
                        singleLine = true,
                        label = { Text("Name (max 20 chars)") },
                    )
                    Text(
                        if (connected) {
                            "The new name is sent to the device now."
                        } else {
                            "The device is offline — the new name applies on its next connect."
                        },
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            },
            confirmButton = {
                TextButton(onClick = {
                    showRename = false
                    fleetScope.launch { store.setName(text, unitID, pusher(unitID)) }
                }) { Text("Save") }
            },
            dismissButton = { TextButton(onClick = { showRename = false }) { Text("Cancel") } },
        )
    }
    if (showForget && rec != null) {
        AlertDialog(
            onDismissRequest = { showForget = false },
            title = { Text("Forget “${rec.effectiveName.ifEmpty { rec.unitID }}”?") },
            confirmButton = {
                TextButton(onClick = {
                    showForget = false
                    fleetScope.launch { store.forget(unitID) }
                    onBack()
                }) { Text("Forget Device", color = MaterialTheme.colorScheme.error) }
            },
            dismissButton = { TextButton(onClick = { showForget = false }) { Text("Cancel") } },
        )
    }
}

/**
 * iOS bumpRocketID verbatim: unset (0) becomes 1 on the first tap in either
 * direction; thereafter steps clamp to the firmware's 1–254.
 */
private fun bumpRocketId(
    rec: KnownDevice,
    delta: Int,
    store: KnownDeviceStore,
    fleetScope: CoroutineScope,
    pusher: (String) -> DeviceSession?,
) {
    val current = rec.effectiveRocketID
    val next = if (current == 0) 1 else (current + delta).coerceIn(1, 254)
    if (next == current) return
    fleetScope.launch { store.setRocketId(next, rec.unitID, pusher(rec.unitID)) }
}
