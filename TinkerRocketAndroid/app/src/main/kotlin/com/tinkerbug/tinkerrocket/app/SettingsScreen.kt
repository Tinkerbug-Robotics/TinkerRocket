package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
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
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.CalAdvisory
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.ConfigGroup
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState
import com.tinkerbug.tinkerrocket.session.RocketProfile
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch

/**
 * Phase 5 settings — iOS SettingsView port.  Rules carried over:
 * settings self-apply on change, never behind an Apply button (#144);
 * numeric fields commit on blur/Done so typing never fights external
 * recomposition — the #361 analog (this screen also reads NO telemetry,
 * only sync/advisory state, so nothing recomposes at telemetry rate).
 * Roll-waypoint + fin-layout editors land in a later batch.
 *
 * All mutations hop to the fleet dispatcher (single-writer contract).
 */
@Composable
fun SettingsScreen(
    store: RocketProfileStore,
    syncer: ActiveRocketSyncer,
    fleetScope: CoroutineScope,
) {
    val profiles by store.profiles.collectAsState()
    val activeId by store.activeId.collectAsState()
    val syncState by syncer.syncState.collectAsState()
    val magAdvisory by syncer.magCalAdvisory.collectAsState()
    val sensorAdvisory by syncer.sensorCalAdvisory.collectAsState()
    val suggestedId by syncer.suggestedProfileId.collectAsState()
    val active = activeId?.let { id -> profiles.firstOrNull { it.id == id } }

    // Self-apply (#144): persist the edit, then push just its group.
    fun edit(group: ConfigGroup?, mutate: (RocketProfile) -> RocketProfile) {
        val id = activeId ?: return
        fleetScope.launch {
            store.update(id, mutate)
            group?.let { syncer.pushGroup(it) }
        }
    }

    var adding by remember { mutableStateOf(false) }
    var renaming by remember { mutableStateOf(false) }
    var confirmDelete by remember { mutableStateOf(false) }
    val focusManager = LocalFocusManager.current

    Column(
        Modifier
            .fillMaxSize()
            // Tap outside any field = blur = commit (see NumField).
            .pointerInput(Unit) {
                detectTapGestures(onTap = { focusManager.clearFocus() })
            }
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Rocket Settings", style = MaterialTheme.typography.titleLarge)
            SyncBadge(syncState)
        }

        // A profile that last flew on this board — suggested, never auto-applied.
        suggestedId?.let { sid ->
            val suggested = profiles.firstOrNull { it.id == sid } ?: return@let
            Banner("“${suggested.name}” last flew on this rocket.", "Switch") {
                fleetScope.launch { store.setActive(sid) }
            }
        }
        CalBanner("Mag cal", magAdvisory, onImport = {
            fleetScope.launch { syncer.importRocketCalIntoActiveProfile(System.currentTimeMillis()) }
        })
        CalBanner("Sensor cal", sensorAdvisory, onImport = null)

        // ── Profile picker ───────────────────────────────────────────────
        Row(
            Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            profiles.forEach { p ->
                val selected = p.id == activeId
                Text(
                    p.name,
                    style = MaterialTheme.typography.labelLarge,
                    color = if (selected) Color.White else MaterialTheme.colorScheme.onSurface,
                    modifier = Modifier
                        .background(
                            if (selected) MaterialTheme.colorScheme.primary
                            else MaterialTheme.colorScheme.surfaceVariant,
                            RoundedCornerShape(16.dp),
                        )
                        .clickable { fleetScope.launch { store.setActive(p.id) } }
                        .padding(horizontal = 12.dp, vertical = 6.dp),
                )
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            OutlinedButton(onClick = { adding = true }) { Text("Add") }
            if (active != null) {
                OutlinedButton(onClick = { fleetScope.launch { store.duplicate(active.id) } }) { Text("Duplicate") }
                OutlinedButton(onClick = { renaming = true }) { Text("Rename") }
                OutlinedButton(onClick = { confirmDelete = true }) { Text("Delete") }
            }
        }

        if (active == null) {
            Text(
                if (profiles.isEmpty()) "No profiles yet — add one to configure a rocket."
                else "Select a profile to edit.",
                style = MaterialTheme.typography.bodyMedium,
            )
            return@Column
        }

        // ── Rocket ───────────────────────────────────────────────────────
        Section("Rocket") {
            ToggleRow("Sounds", active.soundsEnabled) { v -> edit(ConfigGroup.SOUNDS) { it.copy(soundsEnabled = v) } }
            ToggleRow("Servo control", active.servoControlEnabled) { v ->
                edit(ConfigGroup.SERVO_ENABLE) { it.copy(servoControlEnabled = v) }
            }
            ToggleRow("Gain schedule", active.gainScheduleEnabled) { v ->
                edit(ConfigGroup.GAIN_SCHEDULE) { it.copy(gainScheduleEnabled = v) }
            }
            ToggleRow("Angle control", active.useAngleControl) { v ->
                edit(ConfigGroup.ROLL_CONTROL) { it.copy(useAngleControl = v) }
            }
            if (active.useAngleControl) {
                FieldRow {
                    NumField("Kp angle", fmt(active.kpAngle), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.ROLL_CONTROL) { it.copy(kpAngle = v) } }
                    }
                    NumField("Rate cap °/s", fmt(active.rateCapDps), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.ROLL_CONTROL) { it.copy(rateCapDps = v) } }
                    }
                }
            }
            FieldRow {
                NumField("Roll delay ms", active.rollDelayMs.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.ROLL_CONTROL) { it.copy(rollDelayMs = v) } }
                }
                NumField("Camera type", active.cameraType.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.CAMERA) { it.copy(cameraType = v) } }
                }
            }
            FieldRow {
                NumField("IMU orient", active.imuOrientSetting.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.IMU) { it.copy(imuOrientSetting = v) } }
                }
                NumField("IMU rate Hz", active.imuRateHz.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.IMU) { it.copy(imuRateHz = v) } }
                }
            }
        }

        // ── PID ──────────────────────────────────────────────────────────
        Section("PID") {
            FieldRow {
                NumField("Kp", fmt(active.pidKp), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.PID) { it.copy(pidKp = v) } }
                }
                NumField("Ki", fmt(active.pidKi), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.PID) { it.copy(pidKi = v) } }
                }
                NumField("Kd", fmt(active.pidKd), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.PID) { it.copy(pidKd = v) } }
                }
            }
            FieldRow {
                NumField("Min cmd", fmt(active.pidMinCmd), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.PID) { it.copy(pidMinCmd = v) } }
                }
                NumField("Max cmd", fmt(active.pidMaxCmd), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.PID) { it.copy(pidMaxCmd = v) } }
                }
                // Rides the roll-control frame, not the PID frame.
                NumField("I-sep °/s", fmt(active.integralSepThreshold), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.ROLL_CONTROL) { it.copy(integralSepThreshold = v) } }
                }
            }
        }

        // ── Servo ────────────────────────────────────────────────────────
        Section("Servo") {
            FieldRow {
                NumField("Bias 1", active.servoBias1.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoBias1 = v) } }
                }
                NumField("Bias 2", active.servoBias2.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoBias2 = v) } }
                }
                NumField("Bias 3", active.servoBias3.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoBias3 = v) } }
                }
                NumField("Bias 4", active.servoBias4.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoBias4 = v) } }
                }
            }
            FieldRow {
                NumField("Hz", active.servoHz.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoHz = v) } }
                }
                NumField("Min µs", active.servoMinUs.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoMinUs = v) } }
                }
                NumField("Max µs", active.servoMaxUs.toString(), Modifier.weight(1f)) { s ->
                    s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoMaxUs = v) } }
                }
                // #449: travel is the editable quantity; fin angles derive.
                NumField("Travel °", fmt(active.finTravelDeg), Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(finTravelDeg = v) } }
                }
            }
            Text(
                "Fin layout + roll waypoints: editors land in a later batch " +
                    "(defaults push the “+” mix).",
                style = MaterialTheme.typography.bodySmall,
            )
        }

        // ── Guidance ─────────────────────────────────────────────────────
        Section("Guidance") {
            ToggleRow("Guidance enabled", active.guidanceEnabled) { v ->
                edit(ConfigGroup.GUIDANCE) { it.copy(guidanceEnabled = v) }
            }
            if (active.guidanceEnabled) {
                FieldRow {
                    NumField("Nav gain", fmt(active.pnNavGain), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnNavGain = v) } }
                    }
                    NumField("Max accel", fmt(active.pnMaxAccel), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnMaxAccel = v) } }
                    }
                    NumField("Max fin °", fmt(active.pnMaxFinDeg), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnMaxFinDeg = v) } }
                    }
                }
                FieldRow {
                    NumField("Min speed", fmt(active.pnMinSpeed), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnMinSpeed = v) } }
                    }
                    NumField("Target alt m", fmt(active.pnTargetAltM), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnTargetAltM = v) } }
                    }
                    NumField("Coast ms", active.pnCoastDelayMs.toString(), Modifier.weight(1f)) { s ->
                        s.toIntOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnCoastDelayMs = v) } }
                    }
                }
            }
        }

        // ── Pyro (config only — ARMING stays an explicit dashboard action) ─
        Section("Pyro") {
            PyroChannelRow(1, active.pyro1Enabled, active.pyro1TriggerMode, active.pyro1TriggerValue,
                onEnabled = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro1Enabled = v) } },
                onMode = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro1TriggerMode = v) } },
                onValue = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro1TriggerValue = v) } })
            PyroChannelRow(2, active.pyro2Enabled, active.pyro2TriggerMode, active.pyro2TriggerValue,
                onEnabled = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro2Enabled = v) } },
                onMode = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro2TriggerMode = v) } },
                onValue = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro2TriggerValue = v) } })
            PyroChannelRow(3, active.pyro3Enabled, active.pyro3TriggerMode, active.pyro3TriggerValue,
                onEnabled = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro3Enabled = v) } },
                onMode = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro3TriggerMode = v) } },
                onValue = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro3TriggerValue = v) } })
            PyroChannelRow(4, active.pyro4Enabled, active.pyro4TriggerMode, active.pyro4TriggerValue,
                onEnabled = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro4Enabled = v) } },
                onMode = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro4TriggerMode = v) } },
                onValue = { v -> edit(ConfigGroup.PYRO) { it.copy(pyro4TriggerValue = v) } })
        }

        // ── Recovery — app-side landing prediction, nothing on the wire ──
        Section("Recovery (landing prediction)") {
            FieldRow {
                NumField("Drogue ft/s", fmtD(active.drogueRateFps), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.let { v -> edit(null) { it.copy(drogueRateFps = v) } }
                }
                NumField("Main ft/s", fmtD(active.mainRateFps), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.let { v -> edit(null) { it.copy(mainRateFps = v) } }
                }
                NumField("Main alt ft", fmtD(active.mainDeployAltAglFt), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.let { v -> edit(null) { it.copy(mainDeployAltAglFt = v) } }
                }
            }
        }

        Section("Notes") {
            NumField("Notes", active.notes, Modifier.fillMaxWidth(), keyboard = KeyboardType.Text) { s ->
                edit(null) { it.copy(notes = s) }
            }
        }
    }

    if (adding) {
        TextPromptDialog(
            title = "Name this airframe",
            initial = "",
            onDismiss = { adding = false },
            onConfirm = { name ->
                adding = false
                if (name.isNotBlank()) {
                    // iOS RocketProfileView: add then activate.
                    fleetScope.launch { store.setActive(store.add(name.trim()).id) }
                }
            },
        )
    }
    if (renaming && active != null) {
        TextPromptDialog(
            title = "Rename profile",
            initial = active.name,
            onDismiss = { renaming = false },
            onConfirm = { name ->
                renaming = false
                if (name.isNotBlank()) fleetScope.launch { store.rename(active.id, name.trim()) }
            },
        )
    }
    if (confirmDelete && active != null) {
        AlertDialog(
            onDismissRequest = { confirmDelete = false },
            title = { Text("Delete “${active.name}”?") },
            text = { Text("The profile and its saved calibration are removed from this phone.") },
            confirmButton = {
                TextButton(onClick = {
                    confirmDelete = false
                    fleetScope.launch { store.delete(active.id) }
                }) { Text("Delete") }
            },
            dismissButton = { TextButton(onClick = { confirmDelete = false }) { Text("Cancel") } },
        )
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

@Composable
private fun SyncBadge(state: SyncState) {
    val (label, color) = when (state) {
        SyncState.Idle -> return
        SyncState.AwaitingSync -> "awaiting sync" to Color(0xFF9E9E9E)
        SyncState.NoProfile -> "no profile" to Color(0xFFFFA000)
        SyncState.Syncing -> "syncing…" to Color(0xFF1E88E5)
        SyncState.Synced -> "synced" to Color(0xFF43A047)
        is SyncState.Failed -> "sync failed" to Color(0xFFE53935)
    }
    Text(
        label,
        style = MaterialTheme.typography.labelMedium,
        color = Color.White,
        modifier = Modifier
            .background(color, RoundedCornerShape(12.dp))
            .padding(horizontal = 10.dp, vertical = 4.dp),
    )
}

@Composable
private fun Banner(message: String, action: String, onAction: () -> Unit) {
    Card {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 6.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(message, style = MaterialTheme.typography.bodyMedium, modifier = Modifier.weight(1f))
            TextButton(onClick = onAction) { Text(action) }
        }
    }
}

@Composable
private fun CalBanner(label: String, advisory: CalAdvisory, onImport: (() -> Unit)?) {
    when (advisory) {
        CalAdvisory.None -> Unit
        CalAdvisory.Missing -> Card {
            Text(
                "$label: none saved in this profile and none on the rocket.",
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier.padding(12.dp),
            )
        }
        is CalAdvisory.BoardMismatch -> Card {
            Text(
                "$label was calibrated on ${advisory.savedOn}, but this rocket is " +
                    "${advisory.current} — recalibrate before flying.",
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier.padding(12.dp),
            )
        }
        CalAdvisory.RocketHasUnsavedCal ->
            if (onImport != null) {
                Banner("$label: the rocket has a calibration this profile doesn't.", "Import", onImport)
            } else {
                Card {
                    Text(
                        "$label: the rocket has a calibration this profile doesn't " +
                            "(run calibration to save one).",
                        style = MaterialTheme.typography.bodyMedium,
                        modifier = Modifier.padding(12.dp),
                    )
                }
            }
    }
}

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            content()
        }
    }
}

@Composable
private fun ToggleRow(label: String, value: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, style = MaterialTheme.typography.bodyLarge)
        Switch(checked = value, onCheckedChange = onChange)
    }
}

@Composable
private fun FieldRow(content: @Composable androidx.compose.foundation.layout.RowScope.() -> Unit) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp), content = content)
}

/**
 * Commit-on-blur/Done field (#361 analog): local text while focused, the
 * committed value only re-seeds when it changes externally (profile switch).
 * A parse-rejected commit leaves the profile value untouched.  Compose does
 * NOT blur on taps outside focusables — the screen root clears focus on
 * background taps, and the dispose hook catches tab switches, so a pending
 * edit is never silently dropped.
 */
@Composable
private fun NumField(
    label: String,
    value: String,
    modifier: Modifier = Modifier,
    keyboard: KeyboardType = KeyboardType.Decimal,
    onCommit: (String) -> Unit,
) {
    var text by remember(value) { mutableStateOf(value) }
    var focused by remember { mutableStateOf(false) }
    val focusManager = LocalFocusManager.current
    val pending by androidx.compose.runtime.rememberUpdatedState(Triple(text, value, onCommit))
    androidx.compose.runtime.DisposableEffect(Unit) {
        onDispose {
            val (t, v, commit) = pending
            if (t != v) commit(t)
        }
    }
    OutlinedTextField(
        value = text,
        onValueChange = { text = it },
        label = { Text(label) },
        singleLine = true,
        keyboardOptions = KeyboardOptions(keyboardType = keyboard, imeAction = ImeAction.Done),
        keyboardActions = KeyboardActions(onDone = { focusManager.clearFocus() }),
        modifier = modifier.onFocusChanged { st ->
            if (focused && !st.isFocused && text != value) onCommit(text)
            focused = st.isFocused
        },
    )
}

@Composable
private fun PyroChannelRow(
    channel: Int,
    enabled: Boolean,
    mode: Int,
    value: Float,
    onEnabled: (Boolean) -> Unit,
    onMode: (Int) -> Unit,
    onValue: (Float) -> Unit,
) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text("Ch $channel", Modifier.padding(end = 4.dp), style = MaterialTheme.typography.bodyLarge)
        Switch(checked = enabled, onCheckedChange = onEnabled)
        NumField("Mode", mode.toString(), Modifier.weight(1f)) { s -> s.toIntOrNull()?.let(onMode) }
        NumField("Value", fmt(value), Modifier.weight(1f)) { s -> s.toFloatOrNull()?.let(onValue) }
    }
}

@Composable
private fun TextPromptDialog(
    title: String,
    initial: String,
    onDismiss: () -> Unit,
    onConfirm: (String) -> Unit,
) {
    var text by remember { mutableStateOf(initial) }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text(title) },
        text = { OutlinedTextField(value = text, onValueChange = { text = it }, singleLine = true) },
        confirmButton = { TextButton(onClick = { onConfirm(text) }) { Text("OK") } },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Cancel") } },
    )
}

private fun fmt(f: Float): String =
    if (f == f.toLong().toFloat()) f.toLong().toString() else f.toString()

private fun fmtD(d: Double): String =
    if (d == d.toLong().toDouble()) d.toLong().toString() else d.toString()
