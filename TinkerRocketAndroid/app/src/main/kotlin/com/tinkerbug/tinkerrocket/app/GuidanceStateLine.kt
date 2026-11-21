package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.HelpOutline
import androidx.compose.material.icons.filled.Report
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.GuidanceStateCheck
import com.tinkerbug.tinkerrocket.session.RocketProfile

/**
 * The flight computer's guidance on/off against [profile]'s, live from the
 * session (iOS: `GuidanceStateCheck.evaluate` at each call site).
 */
@Composable
fun rememberGuidanceStateCheck(session: DeviceSession, profile: RocketProfile?): GuidanceStateCheck {
    val connected by session.isConnected.collectAsState()
    val config by session.rocketConfig.collectAsState()
    return GuidanceStateCheck.evaluate(
        profile = profile,
        config = config,
        isConnected = connected,
        isBaseStation = session.isBaseStation,
    )
}

/**
 * One line saying whether the flight computer holds the guidance setting the
 * profile shows: red when it will fly something else, orange when nothing
 * from it confirms a guidance profile, nothing otherwise (iOS
 * `GuidanceStateLine`).  On the dashboard it is one more advisory line, under
 * the same rule as the others: it never recolours the state banner.
 */
@Composable
fun GuidanceStateLine(check: GuidanceStateCheck, modifier: Modifier = Modifier) {
    val line = check.message ?: return
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val tint = if (check.isMismatch) tr.statusBad else tr.statusWarn
    Row(
        modifier.fillMaxWidth().padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            if (check.isMismatch) Icons.Filled.Report else Icons.AutoMirrored.Filled.HelpOutline,
            contentDescription = null,
            tint = tint,
            modifier = Modifier.size(16.dp),
        )
        Text("  $line", style = MaterialTheme.typography.labelMedium, color = tint)
    }
}
