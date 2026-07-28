package com.tinkerbug.tinkerrocket.app

import android.net.Uri
import android.provider.OpenableColumns
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
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
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.OtaSession
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * OTA firmware update — port of iOS FirmwareUpdateView: pick a .bin via
 * SAF, push it over BLE, wait for the reboot, confirm the new version.
 *
 * The [OtaSession] is owned by [AppContainer] keyed on device id, NOT by
 * this screen or the DeviceSession: the fleet destroys and recreates the
 * session across the post-OTA reboot (#140), and the result must survive
 * both that and this screen being navigated away from.
 */
@Composable
fun FirmwareUpdateScreen(
    container: AppContainer,
    deviceId: String,
    session: DeviceSession,
    onBack: () -> Unit,
) {
    val context = LocalContext.current
    val ota = remember(deviceId) { container.otaSessionFor(deviceId) }
    val state by ota.state.collectAsState()
    val identity by session.identity.collectAsState()
    val connected by session.isConnected.collectAsState()
    val mtu by session.negotiatedMtu.collectAsState()
    val scope = androidx.compose.runtime.rememberCoroutineScope()

    var pickedName by remember { mutableStateOf<String?>(null) }
    var pickedBytes by remember { mutableStateOf<ByteArray?>(null) }
    var pickError by remember { mutableStateOf<String?>(null) }
    // A rocket's BLE peer is the OC, which can relay the image on to the FC.
    // A base station has no FC, so it only ever flashes itself.
    var targetIsFc by remember { mutableStateOf(false) }

    val picker = rememberLauncherForActivityResult(
        ActivityResultContracts.OpenDocument(),
    ) { uri: Uri? ->
        if (uri == null) return@rememberLauncherForActivityResult
        scope.launch {
            runCatching {
                withContext(Dispatchers.IO) {
                    val bytes = context.contentResolver.openInputStream(uri)?.use { it.readBytes() }
                        ?: error("could not open the file")
                    val name = context.contentResolver.query(uri, null, null, null, null)?.use { c ->
                        val i = c.getColumnIndex(OpenableColumns.DISPLAY_NAME)
                        if (i >= 0 && c.moveToFirst()) c.getString(i) else null
                    } ?: uri.lastPathSegment.orEmpty()
                    name to bytes
                }
            }.onSuccess { (name, bytes) ->
                pickedName = name; pickedBytes = bytes; pickError = null
            }.onFailure {
                pickError = it.message ?: "Could not read the file"
            }
        }
    }

    val busy = ota.isRunning

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack, enabled = !busy) { Text("← Dashboard") }
            Text("Firmware update", style = MaterialTheme.typography.titleLarge)
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("Device", style = MaterialTheme.typography.titleMedium)
                OtaRow("Name", identity.unitName.ifEmpty { session.connectedDeviceName })
                OtaRow("Hardware ID", identity.unitId ?: "—")
                OtaRow("Firmware", identity.firmwareVersion?.ifEmpty { null } ?: "(pre-#8 image)")
                // The OC's own version never moves on an FC-only OTA, so show
                // the relayed FC version whenever that's the target.
                if (targetIsFc && !session.isBaseStation) {
                    OtaRow(
                        "FC firmware",
                        identity.fcFirmwareVersion?.ifEmpty { null } ?: "(awaiting relay…)",
                    )
                }
                OtaRow("Connection", if (connected) "Connected" else "Disconnected")
                OtaRow("Chunk size", "${com.tinkerbug.tinkerrocket.protocol.Commands.otaMaxChunkSize(mtu)} B (MTU $mtu)")
            }
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("Firmware image", style = MaterialTheme.typography.titleMedium)
                val bytes = pickedBytes
                if (bytes != null) {
                    OtaRow("File", pickedName ?: "—")
                    OtaRow("Size", humanBytes(bytes.size.toLong()))
                } else {
                    Text(
                        "No file selected",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                pickError?.let {
                    Text(it, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
                }
                OutlinedButton(
                    enabled = !busy,
                    // .bin has no registered MIME type; */* with the SAF picker
                    // is what actually lets the user reach a firmware file.
                    onClick = { picker.launch(arrayOf("*/*")) },
                ) { Text(if (bytes == null) "Choose .bin…" else "Choose a different file…") }
            }
        }

        if (!session.isBaseStation) {
            Card {
                Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("Target", style = MaterialTheme.typography.titleMedium)
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        TargetChip("This device", !targetIsFc, !busy) { targetIsFc = false }
                        TargetChip("Flight Computer", targetIsFc, !busy) { targetIsFc = true }
                    }
                    if (targetIsFc) {
                        Text(
                            "The image is relayed over the OC↔FC link. Begin takes " +
                                "longer (the FC erases its OTA slot first) and the new " +
                                "version only appears once the FC finishes rebooting.",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                }
            }
        }

        // ── Action / status ──────────────────────────────────────────────
        when (val s = state) {
            is OtaSession.State.Idle -> FlashButton(pickedBytes, connected) {
                ota.start(it, targetIsFc)
            }

            is OtaSession.State.Loading -> Row(
                horizontalArrangement = Arrangement.spacedBy(10.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                CircularProgressIndicator(Modifier.padding(2.dp))
                Text("Reading file + computing SHA…")
            }

            is OtaSession.State.Uploading -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Text("Uploading…")
                    Text(
                        "${humanBytes(s.bytesSent)} / ${humanBytes(s.totalBytes)}",
                        fontFamily = FontFamily.Monospace,
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
                LinearProgressIndicator(
                    progress = {
                        if (s.totalBytes > 0) s.bytesSent.toFloat() / s.totalBytes else 0f
                    },
                    modifier = Modifier.fillMaxWidth(),
                )
                OutlinedButton(onClick = { ota.cancel() }, modifier = Modifier.fillMaxWidth()) {
                    Text("Cancel")
                }
            }

            is OtaSession.State.Verifying -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    CircularProgressIndicator(Modifier.padding(2.dp))
                    Text("Verifying SHA on device…")
                }
                OutlinedButton(onClick = { ota.cancel() }, modifier = Modifier.fillMaxWidth()) {
                    Text("Cancel")
                }
            }

            is OtaSession.State.Rebooting -> Row(
                horizontalArrangement = Arrangement.spacedBy(10.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                CircularProgressIndicator(Modifier.padding(2.dp))
                Text("Device rebooting — waiting for reconnect (60 s)…")
            }

            is OtaSession.State.Verified -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("✅ Updated successfully", style = MaterialTheme.typography.titleMedium, color = Color(0xFF43A047))
                OtaRow("Previous", ota.preFlashVersion.ifEmpty { "—" })
                OtaRow("Now running", s.newVersion)
                FlashAnotherButton { ota.reset() }
            }

            is OtaSession.State.RollbackDetected -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("↩ Rollback detected", style = MaterialTheme.typography.titleMedium, color = Color(0xFFFFA000))
                Text(
                    "The device reconnected but is still running ${s.version}. The new " +
                        "image likely failed to boot, so the bootloader reverted to the " +
                        "previous partition.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                FlashAnotherButton { ota.reset() }
            }

            is OtaSession.State.Failed -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("✕ Failed", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.error)
                Text(s.reason, style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
                FlashAnotherButton { ota.reset() }
            }
        }

        Text(
            "Keep the phone near the device and the screen on until the update " +
                "finishes. A failed image is recoverable: the bootloader keeps the " +
                "previous partition and reverts to it if the new one doesn't boot.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun FlashButton(bytes: ByteArray?, connected: Boolean, onFlash: (ByteArray) -> Unit) {
    Button(
        enabled = bytes != null && connected,
        onClick = { bytes?.let(onFlash) },
        modifier = Modifier.fillMaxWidth(),
    ) { Text("Flash firmware") }
    if (!connected) {
        Text(
            "Not connected",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.error,
        )
    }
}

@Composable
private fun FlashAnotherButton(onReset: () -> Unit) {
    OutlinedButton(onClick = onReset, modifier = Modifier.fillMaxWidth()) {
        Text("Flash another firmware")
    }
}

@Composable
private fun TargetChip(label: String, selected: Boolean, enabled: Boolean, onClick: () -> Unit) {
    if (selected) {
        Button(onClick = onClick, enabled = enabled) { Text(label) }
    } else {
        OutlinedButton(onClick = onClick, enabled = enabled) { Text(label) }
    }
}

@Composable
private fun OtaRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace, style = MaterialTheme.typography.bodySmall)
    }
}

private fun humanBytes(n: Long): String = when {
    n >= 1_048_576 -> "%.2f MB".format(n / 1_048_576.0)
    n >= 1024 -> "%.1f kB".format(n / 1024.0)
    else -> "$n B"
}

/**
 * Device-less OTA progress, shown while the device is away rebooting and the
 * normal device UI has unmounted.  Bench 2026-07-28: without this the app
 * dropped to the scanner mid-flash — the OTA state survived (it is fleet-keyed
 * in AppContainer), but the user lost sight of it exactly when reassurance
 * matters most.
 */
@Composable
fun OtaProgressScreen(ota: OtaSession) {
    val state by ota.state.collectAsState()
    Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp, Alignment.CenterVertically),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("Firmware update in progress", style = MaterialTheme.typography.titleLarge)
        when (val s = state) {
            is OtaSession.State.Uploading -> {
                Text("${humanBytes(s.bytesSent)} / ${humanBytes(s.totalBytes)}", fontFamily = FontFamily.Monospace)
                LinearProgressIndicator(
                    progress = { if (s.totalBytes > 0) s.bytesSent.toFloat() / s.totalBytes else 0f },
                    modifier = Modifier.fillMaxWidth(),
                )
            }
            is OtaSession.State.Verifying -> {
                CircularProgressIndicator()
                Text("Verifying on device…")
            }
            is OtaSession.State.Rebooting -> {
                CircularProgressIndicator()
                Text("Device rebooting — waiting for it to come back…")
            }
            else -> {
                CircularProgressIndicator()
                Text("Working…")
            }
        }
        Text(
            "Keep the phone near the device. Don't power-cycle it — the " +
                "bootloader keeps the previous firmware and reverts to it if " +
                "the new image doesn't boot.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}
