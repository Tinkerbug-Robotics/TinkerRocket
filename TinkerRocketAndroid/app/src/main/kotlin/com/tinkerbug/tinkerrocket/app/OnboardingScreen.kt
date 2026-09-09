package com.tinkerbug.tinkerrocket.app

import com.tinkerbug.tinkerrocket.R
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.DeviceIdentityPusher
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.NetworkIdentity

/**
 * #1057: first-launch onboarding — the operator picks a network name before
 * the scanner is reachable, exactly as iOS gates DashboardView behind
 * OnboardingView.
 *
 * Why it matters, precisely: the app's network ID starts at 0, which is ALSO
 * the firmware's factory default, so an all-factory fleet is internally
 * consistent and telemetry flows — the break is a MIXED fleet. Provision a
 * rocket from iOS onto a non-zero ID, then set the base station up from the
 * Android phone: Android is still on 0, pushes nothing, the base station stays
 * at 0, and the two no longer talk. Every mismatch surface in Device Manager
 * is also gated off while the app's ID is 0 (`if (networkId > 0)`), so nothing
 * on screen says why.
 *
 * The ID is derived from the name (FNV-1a, 0 remapped to 1 — see
 * [NetworkIdentity.networkIdForName]), so the same name yields the same ID on
 * both phones and the preview here is the value that actually goes on the air.
 */
@Composable
fun OnboardingScreen(onContinue: (String) -> Unit) {
    var nameInput by remember { mutableStateOf("") }
    val trimmed = nameInput.trim()
    Column(
        Modifier.fillMaxSize().padding(32.dp),
        verticalArrangement = Arrangement.spacedBy(24.dp, Alignment.CenterVertically),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Image(
            painterResource(R.drawable.tinkerbug_logo),
            contentDescription = "Tinkerbug Robotics",
            modifier = Modifier.height(80.dp),
        )
        Text("Welcome to TinkerRocket", style = MaterialTheme.typography.headlineSmall)
        Text(
            "Choose a network name for your devices. This keeps your rockets and " +
                "base stations separate from others at the same field.",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            textAlign = TextAlign.Center,
        )
        Column(Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Network name", style = MaterialTheme.typography.titleSmall)
            OutlinedTextField(
                value = nameInput,
                onValueChange = { nameInput = it },
                singleLine = true,
                placeholder = { Text("e.g. My Backyard, Skyhawks Club") },
                modifier = Modifier.fillMaxWidth(),
            )
            if (trimmed.isNotEmpty()) {
                // The ID is what actually goes over the air (iOS shows the
                // same preview) — devices are set to it during provisioning.
                Text(
                    "Network ID: ${NetworkIdentity.networkIdForName(trimmed)}",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
        Button(
            onClick = { onContinue(trimmed) },
            enabled = trimmed.isNotEmpty(),
            modifier = Modifier.fillMaxWidth(),
        ) { Text("Continue") }
    }
}

/**
 * #1057: the first-connect provisioning sheet (iOS `DeviceProvisioningSheet`).
 * Raised once per device, when its `config_identity` readback lands and the
 * registry has never marked it provisioned — the moment the app can push its
 * own network ID onto a factory-default board.
 *
 * Save pushes name, network ID and (rockets only) rocket ID; Skip pushes
 * nothing. Either way the device is marked provisioned so it stops prompting —
 * Device Manager stays the place to change any of it later, and forgetting a
 * device there makes it "treated as new the next time it connects", which that
 * screen has always promised and could not deliver until now.
 */
@Composable
fun DeviceProvisioningDialog(
    unitId: String,
    initialName: String,
    initialRocketId: Int,
    isBaseStation: Boolean,
    appNetworkId: Int,
    store: KnownDeviceStore,
    pusher: DeviceIdentityPusher?,
    onDone: () -> Unit,
) {
    var nameInput by remember(unitId) { mutableStateOf(initialName) }
    var rocketIdInput by remember(unitId) {
        mutableStateOf(if (initialRocketId > 0) initialRocketId.toString() else "1")
    }
    val rocketId = rocketIdInput.toIntOrNull()
    AlertDialog(
        onDismissRequest = { /* deliberate: Skip or Save, so the choice is recorded */ },
        title = { Text("Set up this device") },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    "This is the first time this device has connected. Give it a name " +
                        "and add it to your network so it stays with your fleet.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                OutlinedTextField(
                    value = nameInput,
                    onValueChange = { nameInput = it },
                    singleLine = true,
                    label = { Text("Name") },
                )
                if (!isBaseStation) {
                    OutlinedTextField(
                        value = rocketIdInput,
                        onValueChange = { rocketIdInput = it.filter { c -> c.isDigit() }.take(3) },
                        singleLine = true,
                        label = { Text("Rocket ID") },
                        isError = rocketId == null || rocketId !in 1..255,
                    )
                }
                if (appNetworkId > 0) {
                    Text(
                        "Network ID $appNetworkId will be pushed to this device.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        },
        confirmButton = {
            TextButton(
                enabled = nameInput.isNotBlank() && (isBaseStation || rocketId in 1..255),
                onClick = {
                    store.setName(nameInput.trim(), unitId, pusher)
                    // #150: only a real ID is pushed — 0 is the app's "unset"
                    // sentinel and the firmware's factory default at once.
                    if (appNetworkId > 0) store.setNetworkId(appNetworkId, unitId, pusher)
                    if (!isBaseStation) rocketId?.let { store.setRocketId(it, unitId, pusher) }
                    store.markProvisioned(unitId)
                    onDone()
                },
            ) { Text("Save") }
        },
        dismissButton = {
            TextButton(onClick = {
                store.markProvisioned(unitId)
                onDone()
            }) { Text("Skip") }
        },
    )
}
