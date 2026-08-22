package com.tinkerbug.tinkerrocket.app

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowDownward
import androidx.compose.material.icons.filled.ArrowUpward
import androidx.compose.material.icons.filled.Bolt
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.RadioButtonUnchecked
import androidx.compose.material.icons.filled.RemoveCircleOutline
import androidx.compose.material.icons.outlined.Circle
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Card
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LinearProgressIndicator
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
import androidx.compose.ui.text.style.TextDecoration
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.PreflightAutoContext
import com.tinkerbug.tinkerrocket.session.PreflightAutoStatus
import com.tinkerbug.tinkerrocket.session.PreflightChecklist
import com.tinkerbug.tinkerrocket.session.PreflightItem
import com.tinkerbug.tinkerrocket.session.PreflightItemKind
import com.tinkerbug.tinkerrocket.session.PreflightStore
import com.tinkerbug.tinkerrocket.session.RocketProfile
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.launch

/**
 * Pre-flight checklist screens — ports of iOS PreflightMasterView /
 * PreflightRocketConfigView / PreflightRunView plus the dashboard advisory
 * line.  The master list is a LIVE template: editing a master step updates
 * it on every rocket that includes it; per-rocket tailoring stores only the
 * diff (see core/session PreflightChecklist.kt).
 *
 * All store MUTATORS hop to the fleet dispatcher (single-writer contract,
 * same as RocketProfileStore usage in SettingsScreen).
 */

// ── Master editor (front page → Preflight Checklist) ────────────────────

@Composable
fun PreflightMasterScreen(
    preflight: PreflightStore,
    profiles: RocketProfileStore,
    fleetScope: CoroutineScope,
    onBack: () -> Unit,
) {
    val master by preflight.master.collectAsState()
    val configs by preflight.configs.collectAsState()
    val profileList by profiles.profiles.collectAsState()

    // Per-rocket tailoring is an internal sub-screen (the SavedFlights →
    // FlightChartScreen pattern) so the master stays one entry point.
    var configRocket by remember { mutableStateOf<RocketProfile?>(null) }
    configRocket?.let { rocket ->
        PreflightRocketConfigScreen(
            profile = rocket,
            preflight = preflight,
            fleetScope = fleetScope,
            onBack = { configRocket = null },
        )
        return
    }

    var editing by remember { mutableStateOf<PreflightItem?>(null) }
    BackHandler(onBack = onBack)

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Back") }
            Text("Preflight Checklist", style = MaterialTheme.typography.titleMedium)
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("Master checklist", style = MaterialTheme.typography.titleMedium)
                Text(
                    "Every rocket starts from this list. Steps marked with ⚡ are " +
                        "verified by the app from live telemetry and can't be checked by hand.",
                    style = MaterialTheme.typography.bodySmall,
                )
                if (master.items.isEmpty()) {
                    Text(
                        "No steps yet. Add the things you never want to forget at the " +
                            "pad — wadding, motor retention, igniter…",
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier.padding(top = 8.dp),
                    )
                }
                master.items.forEachIndexed { idx, item ->
                    PreflightEditRow(
                        item = item,
                        canMoveUp = idx > 0,
                        canMoveDown = idx < master.items.lastIndex,
                        onClick = { editing = item },
                        onMoveUp = {
                            fleetScope.launch { preflight.moveMasterItem(idx, idx - 1) }
                        },
                        onMoveDown = {
                            fleetScope.launch { preflight.moveMasterItem(idx, idx + 1) }
                        },
                        onDelete = {
                            fleetScope.launch { preflight.deleteMasterItem(item.id) }
                        },
                    )
                }
                PreflightAddMenu(
                    usedAutoKinds = master.items.map { it.kind }.filter { it.isAuto }.toSet(),
                    onCustom = { editing = PreflightItem(title = "") },
                    onAuto = { kind ->
                        fleetScope.launch { preflight.addMasterItem(PreflightItem.auto(kind)) }
                    },
                )
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("Rockets", style = MaterialTheme.typography.titleMedium)
                Text(
                    "Tailor the checklist per rocket: leave out master steps that " +
                        "don't apply and add rocket-specific ones.",
                    style = MaterialTheme.typography.bodySmall,
                )
                if (profileList.isEmpty()) {
                    Text(
                        "Add a rocket first (Settings while connected) to tailor its checklist.",
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier.padding(top = 8.dp),
                    )
                }
                profileList.forEach { profile ->
                    val count = PreflightChecklist
                        .effectiveItems(master, configs[profile.id]).size
                    Row(
                        Modifier
                            .fillMaxWidth()
                            .clickable { configRocket = profile }
                            .padding(vertical = 10.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(profile.name, Modifier.weight(1f))
                        Text(
                            if (count == 1) "1 step" else "$count steps",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                }
            }
        }
    }

    editing?.let { item ->
        PreflightItemDialog(
            item = item,
            onDismiss = { editing = null },
            onConfirm = { edited ->
                editing = null
                fleetScope.launch {
                    if (master.items.any { it.id == edited.id }) {
                        preflight.updateMasterItem(edited.id) { edited }
                    } else {
                        preflight.addMasterItem(edited)
                    }
                }
            },
        )
    }
}

// ── Per-rocket tailoring ────────────────────────────────────────────────

@Composable
fun PreflightRocketConfigScreen(
    profile: RocketProfile,
    preflight: PreflightStore,
    fleetScope: CoroutineScope,
    onBack: () -> Unit,
) {
    val master by preflight.master.collectAsState()
    val configs by preflight.configs.collectAsState()
    val config = configs[profile.id]
    val disabled = config?.disabledMasterIds?.toSet().orEmpty()
    val extras = config?.extraItems.orEmpty()

    var editing by remember { mutableStateOf<PreflightItem?>(null) }
    BackHandler(onBack = onBack)

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Back") }
            Text(profile.name, style = MaterialTheme.typography.titleMedium)
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("Master steps", style = MaterialTheme.typography.titleMedium)
                Text(
                    "Switch off master steps that don't apply to “${profile.name}”. " +
                        "Master steps are edited on the previous screen and update every rocket.",
                    style = MaterialTheme.typography.bodySmall,
                )
                master.items.forEach { item ->
                    Row(
                        Modifier.fillMaxWidth().padding(vertical = 4.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        PreflightItemLabel(item, Modifier.weight(1f))
                        Switch(
                            checked = item.id !in disabled,
                            onCheckedChange = { enabled ->
                                fleetScope.launch {
                                    preflight.setMasterItem(item.id, enabled, profile.id)
                                }
                            },
                        )
                    }
                }
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("${profile.name} steps", style = MaterialTheme.typography.titleMedium)
                Text(
                    "Extra steps only this rocket needs — they run after the master steps.",
                    style = MaterialTheme.typography.bodySmall,
                )
                extras.forEachIndexed { idx, item ->
                    PreflightEditRow(
                        item = item,
                        canMoveUp = idx > 0,
                        canMoveDown = idx < extras.lastIndex,
                        onClick = { editing = item },
                        onMoveUp = {
                            fleetScope.launch {
                                preflight.moveExtraItem(profile.id, idx, idx - 1)
                            }
                        },
                        onMoveDown = {
                            fleetScope.launch {
                                preflight.moveExtraItem(profile.id, idx, idx + 1)
                            }
                        },
                        onDelete = {
                            fleetScope.launch { preflight.deleteExtraItem(item.id, profile.id) }
                        },
                    )
                }
                // The same auto condition twice verifies nothing new — grey out
                // kinds already anywhere in this rocket's effective list.
                PreflightAddMenu(
                    usedAutoKinds = PreflightChecklist.effectiveItems(master, config)
                        .map { it.kind }.filter { it.isAuto }.toSet(),
                    onCustom = { editing = PreflightItem(title = "") },
                    onAuto = { kind ->
                        fleetScope.launch {
                            preflight.addExtraItem(PreflightItem.auto(kind), profile.id)
                        }
                    },
                )
            }
        }
    }

    editing?.let { item ->
        PreflightItemDialog(
            item = item,
            onDismiss = { editing = null },
            onConfirm = { edited ->
                editing = null
                fleetScope.launch {
                    if (extras.any { it.id == edited.id }) {
                        preflight.updateExtraItem(edited.id, profile.id) { edited }
                    } else {
                        preflight.addExtraItem(edited, profile.id)
                    }
                }
            },
        )
    }
}

// ── Run screen (dashboard tool route "preflight") ───────────────────────

@Composable
fun PreflightRunScreen(
    session: DeviceSession,
    preflight: PreflightStore,
    profiles: RocketProfileStore,
    syncer: ActiveRocketSyncer?,
    fleetScope: CoroutineScope,
    onBack: () -> Unit,
) {
    val master by preflight.master.collectAsState()
    val configs by preflight.configs.collectAsState()
    val profileList by profiles.profiles.collectAsState()
    val activeId by profiles.activeId.collectAsState()
    val profile = profileList.firstOrNull { it.id == activeId }

    var confirmReset by remember { mutableStateOf(false) }
    BackHandler(onBack = onBack)

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Back") }
            Text("Preflight", style = MaterialTheme.typography.titleMedium)
            Spacer(Modifier.weight(1f))
            val hasChecks = profile != null &&
                configs[profile.id]?.checked?.isNotEmpty() == true
            TextButton(onClick = { confirmReset = true }, enabled = hasChecks) {
                Text("Reset")
            }
        }

        if (profile == null) {
            Text(
                "No active rocket — pick one in Settings to run its checklist.",
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier.padding(top = 16.dp),
            )
            return@Column
        }

        val config = configs[profile.id]
        val items = PreflightChecklist.effectiveItems(master, config)
        val ctx = preflightContext(session, syncer, profile)
        val progress = PreflightChecklist.progress(items, config, ctx)
        val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Column(Modifier.weight(1f)) {
                        Text(profile.name, style = MaterialTheme.typography.titleMedium)
                        Text(
                            if (progress.isComplete) "All ${progress.total} steps complete"
                            else "${progress.done} of ${progress.total} steps done",
                            style = MaterialTheme.typography.bodySmall,
                            color = if (progress.isComplete) tr.statusOk
                            else MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                    if (progress.isComplete) {
                        Icon(Icons.Filled.CheckCircle, null, tint = tr.statusOk)
                    }
                }
                LinearProgressIndicator(
                    progress = {
                        if (progress.total == 0) 0f
                        else progress.done.toFloat() / progress.total
                    },
                    modifier = Modifier.fillMaxWidth(),
                    color = if (progress.isComplete) tr.statusOk else tr.statusWarn,
                )
            }
        }

        if (items.isEmpty()) {
            Text(
                "No checklist for this rocket yet — set one up from Preflight " +
                    "Checklist on the front page.",
                style = MaterialTheme.typography.bodyMedium,
            )
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(2.dp)) {
                items.forEach { item ->
                    val status = PreflightChecklist.autoStatus(item.kind, ctx)
                    if (status != null) {
                        PreflightAutoRunRow(item, status)
                    } else {
                        PreflightManualRunRow(
                            item = item,
                            checked = config?.isChecked(item.id) == true,
                            onToggle = { checked ->
                                fleetScope.launch {
                                    preflight.setChecked(item.id, checked, profile.id)
                                }
                            },
                        )
                    }
                }
                if (items.any { it.kind.isAuto }) {
                    Text(
                        "Steps marked ⚡ are verified live by the app and can't be " +
                            "checked by hand.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.padding(top = 8.dp),
                    )
                }
            }
        }
    }

    if (confirmReset) {
        AlertDialog(
            onDismissRequest = { confirmReset = false },
            title = { Text("Reset checklist?") },
            text = {
                Text(
                    "Clears every hand-checked step for the next flight. Auto steps " +
                        "aren't affected — they always show the live state.",
                )
            },
            confirmButton = {
                TextButton(onClick = {
                    confirmReset = false
                    profile?.let { p -> fleetScope.launch { preflight.resetRun(p.id) } }
                }) { Text("Uncheck all") }
            },
            dismissButton = {
                TextButton(onClick = { confirmReset = false }) { Text("Cancel") }
            },
        )
    }
}

// ── Dashboard advisory line ─────────────────────────────────────────────

/**
 * One quiet line below the rocket state banner: checklist progress for the
 * active rocket, tap to open the run screen.  Deliberately compact — it
 * must not add dashboard clutter, and it never recolors the state banner
 * (design decision 2026-08-22: sensor health owns that).  Renders nothing
 * when the active rocket has no checklist.
 */
@Composable
fun PreflightAdvisoryLine(
    session: DeviceSession,
    preflight: PreflightStore,
    profiles: RocketProfileStore,
    syncer: ActiveRocketSyncer?,
    onOpen: () -> Unit,
) {
    val master by preflight.master.collectAsState()
    val configs by preflight.configs.collectAsState()
    val profileList by profiles.profiles.collectAsState()
    val activeId by profiles.activeId.collectAsState()
    val profile = profileList.firstOrNull { it.id == activeId } ?: return

    val config = configs[profile.id]
    val items = PreflightChecklist.effectiveItems(master, config)
    if (items.isEmpty()) return

    val ctx = preflightContext(session, syncer, profile)
    val progress = PreflightChecklist.progress(items, config, ctx)
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val color = if (progress.isComplete) tr.statusOk else tr.statusWarn

    Row(
        Modifier.fillMaxWidth().clickable(onClick = onOpen).padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            if (progress.isComplete) Icons.Filled.CheckCircle
            else Icons.Outlined.Circle,
            contentDescription = null,
            tint = color,
            modifier = Modifier.size(16.dp),
        )
        Text(
            if (progress.isComplete) "  Preflight complete"
            else "  Preflight ${progress.done}/${progress.total}",
            style = MaterialTheme.typography.labelMedium,
            color = if (progress.isComplete) MaterialTheme.colorScheme.onSurfaceVariant
            else color,
        )
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

/** The auto evaluation's view of the world, from live session state. */
@Composable
private fun preflightContext(
    session: DeviceSession,
    syncer: ActiveRocketSyncer?,
    profile: RocketProfile,
): PreflightAutoContext {
    val telemetry by session.telemetry.collectAsState()
    val connected by session.isConnected.collectAsState()
    val hasTelemetry by session.hasReceivedTelemetry.collectAsState()
    val syncState = syncer?.syncState?.collectAsState()?.value
        ?: ActiveRocketSyncer.SyncState.Idle
    return PreflightAutoContext(
        isConnected = connected,
        hasTelemetry = hasTelemetry,
        isRelay = session.isBaseStation,
        telemetry = telemetry,
        syncState = syncState,
        profile = profile,
    )
}

@Composable
private fun PreflightItemLabel(item: PreflightItem, modifier: Modifier = Modifier) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    Row(modifier, verticalAlignment = Alignment.CenterVertically) {
        if (item.kind.isAuto) {
            Icon(
                Icons.Filled.Bolt, contentDescription = "Auto",
                tint = tr.preflight, modifier = Modifier.size(18.dp),
            )
        }
        Column(Modifier.padding(start = if (item.kind.isAuto) 4.dp else 22.dp)) {
            Text(item.title.ifEmpty { "Untitled step" })
            if (item.detail.isNotEmpty()) {
                Text(
                    item.detail,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    maxLines = 2,
                )
            }
        }
    }
}

/** Editor row: label + move/delete controls; tap the label to edit. */
@Composable
private fun PreflightEditRow(
    item: PreflightItem,
    canMoveUp: Boolean,
    canMoveDown: Boolean,
    onClick: () -> Unit,
    onMoveUp: () -> Unit,
    onMoveDown: () -> Unit,
    onDelete: () -> Unit,
) {
    Row(
        Modifier.fillMaxWidth().padding(vertical = 2.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        PreflightItemLabel(item, Modifier.weight(1f).clickable(onClick = onClick))
        IconButton(onClick = onMoveUp, enabled = canMoveUp) {
            Icon(Icons.Filled.ArrowUpward, "Move up", Modifier.size(18.dp))
        }
        IconButton(onClick = onMoveDown, enabled = canMoveDown) {
            Icon(Icons.Filled.ArrowDownward, "Move down", Modifier.size(18.dp))
        }
        IconButton(onClick = onDelete) {
            Icon(Icons.Filled.Delete, "Delete", Modifier.size(18.dp))
        }
    }
}

/** "Add step" dropdown: one custom entry plus the auto library. */
@Composable
private fun PreflightAddMenu(
    usedAutoKinds: Set<PreflightItemKind>,
    onCustom: () -> Unit,
    onAuto: (PreflightItemKind) -> Unit,
) {
    var open by remember { mutableStateOf(false) }
    Box {
        OutlinedButton(onClick = { open = true }) { Text("+ Add step") }
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            DropdownMenuItem(
                text = { Text("Custom step…") },
                onClick = { open = false; onCustom() },
            )
            PreflightItemKind.entries.filter { it.isAuto }.forEach { kind ->
                DropdownMenuItem(
                    text = { Text("⚡ ${kind.defaultTitle}") },
                    enabled = kind !in usedAutoKinds,
                    onClick = { open = false; onAuto(kind) },
                )
            }
        }
    }
}

/** Add/edit dialog; the kind is fixed at creation (iOS rule). */
@Composable
private fun PreflightItemDialog(
    item: PreflightItem,
    onDismiss: () -> Unit,
    onConfirm: (PreflightItem) -> Unit,
) {
    var title by remember { mutableStateOf(item.title) }
    var detail by remember { mutableStateOf(item.detail) }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text(if (item.title.isEmpty()) "New step" else "Edit step") },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(
                    value = title, onValueChange = { title = it },
                    label = { Text("Title") }, singleLine = true,
                )
                OutlinedTextField(
                    value = detail, onValueChange = { detail = it },
                    label = { Text("Detail (optional)") },
                )
                if (item.kind.isAuto) {
                    Text(
                        "This step is verified automatically — the label is " +
                            "editable, the check is not.",
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = { onConfirm(item.copy(title = title.trim(), detail = detail)) },
                enabled = title.isNotBlank(),
            ) { Text("Save") }
        },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Cancel") } },
    )
}

@Composable
private fun PreflightManualRunRow(
    item: PreflightItem,
    checked: Boolean,
    onToggle: (Boolean) -> Unit,
) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    Row(
        Modifier
            .fillMaxWidth()
            .clickable { onToggle(!checked) }
            .padding(vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            if (checked) Icons.Filled.CheckCircle else Icons.Filled.RadioButtonUnchecked,
            contentDescription = null,
            tint = if (checked) tr.statusOk else MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Column(Modifier.padding(start = 10.dp).weight(1f)) {
            Text(
                item.title,
                textDecoration = if (checked) TextDecoration.LineThrough else null,
                color = if (checked) MaterialTheme.colorScheme.onSurfaceVariant
                else MaterialTheme.colorScheme.onSurface,
            )
            if (item.detail.isNotEmpty()) {
                Text(
                    item.detail,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }
}

@Composable
private fun PreflightAutoRunRow(item: PreflightItem, status: PreflightAutoStatus) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    Row(
        Modifier.fillMaxWidth().padding(vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        when (status) {
            is PreflightAutoStatus.Satisfied ->
                Icon(Icons.Filled.CheckCircle, null, tint = tr.statusOk)
            is PreflightAutoStatus.Pending ->
                Icon(Icons.Outlined.Circle, null, tint = tr.statusWarn)
            is PreflightAutoStatus.NotApplicable ->
                Icon(
                    Icons.Filled.RemoveCircleOutline, null,
                    tint = MaterialTheme.colorScheme.onSurfaceVariant,
                )
        }
        Column(Modifier.padding(start = 10.dp).weight(1f)) {
            Text(item.title)
            when (status) {
                is PreflightAutoStatus.Pending -> Text(
                    status.reason,
                    style = MaterialTheme.typography.bodySmall,
                    color = tr.statusWarn,
                )
                is PreflightAutoStatus.NotApplicable -> Text(
                    status.reason,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                is PreflightAutoStatus.Satisfied -> if (item.detail.isNotEmpty()) {
                    Text(
                        item.detail,
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }
        Icon(
            Icons.Filled.Bolt, contentDescription = "Auto",
            tint = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors.preflight,
            modifier = Modifier.size(18.dp),
        )
    }
}
