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
import androidx.activity.compose.BackHandler
import com.tinkerbug.tinkerrocket.protocol.EspImage
import com.tinkerbug.tinkerrocket.protocol.EspImageVerdict

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
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
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

    // #773: what the picked image actually IS, checked before a byte goes over
    // BLE. The far end validates only size and SHA-256, and the OC and the base
    // station are both ESP32-S3 with byte-identical app slots — so a
    // base-station image pushed to an out computer is accepted by both ends
    // today, and only rollback catches it, and only if it fails to boot.
    //
    // The expected program depends on the target, so this recomputes when the
    // target chip changes. The flight computer's chip differs by board
    // (ESP32-P4 on V9, ESP32-S3 on the mini), so it is left unchecked rather
    // than warning on every mini.
    val expectedProject = when {
        session.isBaseStation -> EspImage.PROJECT_BS
        targetIsFc -> EspImage.PROJECT_FC
        else -> EspImage.PROJECT_OC
    }
    val expectedChipId = if (targetIsFc && !session.isBaseStation) null else 0x0009
    val verdict = remember(pickedBytes, expectedProject, identity.firmwareVersion,
                           identity.fcBoardRev, identity.ocBoardRev) {
        pickedBytes?.let {
            EspImage.check(
                it,
                expectedProject = expectedProject,
                expectedChipId = expectedChipId,
                runningVersion = if (targetIsFc) null else identity.firmwareVersion,
                // #773 step 2: the board's own answer, which beats the image's
                // claim about itself. Android can supply this for the FC too,
                // where it has no firmware version to fall back on.
                provisionedBoard = if (targetIsFc) identity.fcBoardRev else identity.ocBoardRev,
            )
        }
    }

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
                verdict?.let { ImageVerdictBlock(it) }
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
                        // Named, not "This device": the picker only appears for a
                        // rocket, whose BLE peer IS the Out Computer, and the choice
                        // that matters is OC vs FC — "this device" made the default
                        // read as "the whole rocket".
                        TargetChip("Out Computer", !targetIsFc, !busy) { targetIsFc = false }
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
            is OtaSession.State.Idle -> FlashButton(
                pickedBytes,
                connected = connected,
                refused = verdict is EspImageVerdict.Refuse,
            ) {
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
                Text("✅ Updated successfully", style = MaterialTheme.typography.titleMedium, color = tr.statusOk)
                OtaRow("Previous", ota.preFlashVersion.ifEmpty { "—" })
                OtaRow("Now running", s.newVersion)
                FlashAnotherButton { ota.reset() }
            }

            is OtaSession.State.RollbackDetected -> Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("↩ Rollback detected", style = MaterialTheme.typography.titleMedium, color = tr.statusWarn)
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

/** #773: what the image says it is, and whether it belongs on this unit. */
@Composable
private fun ImageVerdictBlock(verdict: EspImageVerdict) {
    val img = when (verdict) {
        is EspImageVerdict.Ok -> verdict.image
        is EspImageVerdict.Warn -> verdict.image
        is EspImageVerdict.Refuse -> verdict.image
    }
    Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
        if (img != null) {
            OtaRow("Program", img.projectName)
            OtaRow("Version", img.version)
            OtaRow("Built for", img.chipName)
            OtaRow("Built", "${img.buildDate} ${img.buildTime}")
        }
        when (verdict) {
            is EspImageVerdict.Ok -> Text(
                "Matches this unit",
                color = MaterialTheme.colorScheme.primary,
                style = MaterialTheme.typography.bodySmall,
            )
            is EspImageVerdict.Warn -> Text(
                verdict.reason,
                color = MaterialTheme.colorScheme.tertiary,
                style = MaterialTheme.typography.bodySmall,
            )
            is EspImageVerdict.Refuse -> Text(
                verdict.reason,
                color = MaterialTheme.colorScheme.error,
                style = MaterialTheme.typography.bodySmall,
            )
        }
    }
}

/**
 * The two reasons the button is off are kept apart deliberately.
 *
 * [connected] used to arrive as `connected && verdict !is Refuse`, so a
 * refused image printed "Not connected" under a Device card that said
 * Connected — the screen contradicting itself about a live link while the
 * real reason ("this image is base_station, but you are updating
 * out_computer") sat in red immediately above. Seen on the bench 2026-09-10
 * against a V9 at −41 dBm.
 *
 * A refusal prints nothing here: its own explanation is already on screen,
 * and that is what iOS's FirmwareUpdateView does too — it disables on
 * `!device.isConnected || imageVerdict?.isRefusal` as separate terms and
 * never conflates the message.
 */
@Composable
private fun FlashButton(
    bytes: ByteArray?,
    connected: Boolean,
    refused: Boolean,
    onFlash: (ByteArray) -> Unit,
) {
    Button(
        enabled = bytes != null && connected && !refused,
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
    // #1063: a terminal state is not "in progress", and the don't-power-cycle
    // line below is actively wrong once the run is over.
    val terminal = state is OtaSession.State.Failed ||
        state is OtaSession.State.Verified ||
        state is OtaSession.State.RollbackDetected
    BackHandler(enabled = terminal) { ota.reset() }
    Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp, Alignment.CenterVertically),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text(
            when {
                state is OtaSession.State.Failed -> "Firmware update failed"
                state is OtaSession.State.Verified -> "Firmware updated"
                state is OtaSession.State.RollbackDetected -> "Firmware did not take"
                else -> "Firmware update in progress"
            },
            style = MaterialTheme.typography.titleLarge,
        )
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
            // #1063: the three terminal states used to fall into the `else`
            // below — a spinner and "Working…" under a header that still said
            // "in progress", with the reason OtaSession had written (which
            // names the remedy) never shown at all.
            is OtaSession.State.Failed -> {
                Text(
                    s.reason,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.error,
                )
            }
            is OtaSession.State.Verified -> {
                Text("Now running ${s.newVersion}", fontFamily = FontFamily.Monospace)
            }
            is OtaSession.State.RollbackDetected -> {
                Text(
                    "The device came back on ${s.version} — the same firmware it " +
                        "had before, so the new image did not boot and the " +
                        "bootloader reverted. Nothing is broken; try the flash " +
                        "again, and if it repeats the image is the problem.",
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.error,
                )
            }
            else -> {
                CircularProgressIndicator()
                Text("Working…")
            }
        }
        if (terminal) {
            Button(onClick = { ota.reset() }) { Text("Dismiss") }
        } else {
            Text(
                "Keep the phone near the device. Don't power-cycle it — the " +
                    "bootloader keeps the previous firmware and reverts to it if " +
                    "the new image doesn't boot.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}
