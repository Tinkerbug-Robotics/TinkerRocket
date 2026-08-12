package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.KeyboardArrowLeft
import androidx.compose.material.icons.automirrored.filled.MenuBook
import androidx.compose.material.icons.automirrored.filled.VolumeOff
import androidx.compose.material.icons.automirrored.filled.VolumeUp
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Straighten
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.session.UnitSystem

/**
 * The connected chrome, in the iOS toolbar arrangement (design pass, iOS =
 * reference): leading back-chevron + units menu; trailing Files (book), Map,
 * voice toggle, Settings (gear).  Navigation mechanics stay Android — the
 * icons select tabs in place rather than pushing screens — but the chevron
 * mirrors iOS's double duty: from a tab it returns to the dashboard, from
 * the dashboard it disconnects (iOS root chevron = disconnectAll).
 */
@Composable
fun ConnectedTopBar(
    tab: Int,
    onTab: (Int) -> Unit,
    onDisconnect: () -> Unit,
    unitStore: UnitStore,
    container: AppContainer,
    toolOpen: Boolean = false,
    onCloseTool: () -> Unit = {},
) {
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 4.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        // A tool sub-screen leaves `tab` at 0, so this chevron used to take its
        // disconnect branch while looking and reading like back -- and a
        // disconnect tears the GATT down before the tool screen's teardown hook
        // runs, so the servo-test stop or mag-cal abort never reached the
        // rocket. From a tool it now closes the tool, which is what it looks
        // like it does and what iOS does from a pushed screen.
        IconButton(
            onClick = {
                when {
                    toolOpen -> onCloseTool()
                    tab != 0 -> onTab(0)
                    else -> onDisconnect()
                }
            },
        ) {
            Icon(
                Icons.AutoMirrored.Filled.KeyboardArrowLeft,
                contentDescription = if (toolOpen || tab != 0) "Back to dashboard" else "Disconnect",
                tint = TrTheme.colors.savedFlights,
            )
        }
        UnitsMenuButton(unitStore)
        Spacer(Modifier.weight(1f))
        TabIcon(Icons.AutoMirrored.Filled.MenuBook, "Files", active = tab == 1) { onTab(1) }
        TabIcon(Icons.Filled.Map, "Map", active = tab == 2) { onTab(2) }
        VoiceIconButton(container)
        TabIcon(Icons.Filled.Settings, "Settings", active = tab == 3) { onTab(3) }
    }
}

@Composable
private fun TabIcon(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    label: String,
    active: Boolean,
    onClick: () -> Unit,
) {
    IconButton(onClick = onClick) {
        Icon(
            icon, contentDescription = label,
            tint = if (active) TrTheme.colors.savedFlights else MaterialTheme.colorScheme.onSurface,
        )
    }
}

/**
 * iOS voiceIcon twin: speaker slashed when off; colored by session health
 * (gray off, green ready, amber engine-not-ready, red error).  Tap toggles,
 * long-press speaks the test phrase — same gestures as the old text chip.
 */
@Composable
fun VoiceIconButton(container: AppContainer) {
    val enabled by container.announcer.enabled.collectAsState()
    val ready by container.ttsSpeaker.ready.collectAsState()
    val error by container.ttsSpeaker.lastError.collectAsState()
    val tint = when {
        !enabled -> Color.Gray
        error != null -> TrTheme.colors.voiceError
        ready -> TrTheme.colors.voiceReady
        else -> TrTheme.colors.voiceWarn
    }
    Box(
        Modifier
            .size(48.dp)
            .combinedClickable(
                onClick = { container.announcer.setEnabled(!enabled) },
                onLongClick = { container.announcer.testVoice() },
            ),
        contentAlignment = Alignment.Center,
    ) {
        Icon(
            if (enabled) Icons.AutoMirrored.Filled.VolumeUp else Icons.AutoMirrored.Filled.VolumeOff,
            contentDescription = if (enabled) "Voice on" else "Voice off",
            tint = tint,
        )
    }
}

/** The ruler units menu (iOS #160) — shared by the scanner and the connected
 *  top bar so the global setting is reachable in both worlds. */
@Composable
fun UnitsMenuButton(store: UnitStore) {
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
