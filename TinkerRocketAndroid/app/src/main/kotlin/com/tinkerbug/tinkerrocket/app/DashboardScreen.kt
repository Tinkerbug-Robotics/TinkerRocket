package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import com.tinkerbug.tinkerrocket.session.FleetDevice
import com.tinkerbug.tinkerrocket.session.DeviceSession
import java.util.Locale

/**
 * Phase 3 dashboard slice — the pad-ops core: identity header, staleness
 * banner (worsen-only effectiveDataStatus), rocket state, power section
 * gated on #377 (no blind cmd-8 until the first telemetry frame confirms
 * power state), battery / GNSS / link cards, pyro tiles with the
 * continuity-AND-live fail-safe rendering.
 */
@Composable
fun DashboardScreen(device: FleetDevice<DeviceSession>, onDisconnect: () -> Unit) {
    val session = device.session
    val telemetry by session.telemetry.collectAsState()
    val identity by session.identity.collectAsState()
    val hasTelemetry by session.hasReceivedTelemetry.collectAsState()
    val dataStatus by session.effectiveDataStatus.collectAsState()
    val rssi by session.connectedRssi.collectAsState()
    val poweringOn by session.poweringOn.collectAsState()

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        // Header
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Column {
                Text(
                    identity.unitName.ifEmpty { device.advertisedName },
                    style = MaterialTheme.typography.titleLarge,
                )
                Text(
                    "nid ${identity.networkId ?: "—"} · rid ${identity.rocketId ?: "—"} · " +
                        "fw ${identity.firmwareVersion ?: "—"}",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            OutlinedButton(onClick = onDisconnect) { Text("Disconnect") }
        }

        // Staleness banner (#390 worsen-only overlay)
        if (dataStatus != TelemetryData.DataStatus.LIVE) {
            Banner(
                text = when (dataStatus) {
                    TelemetryData.DataStatus.SYNCING -> "Syncing…"
                    TelemetryData.DataStatus.STALE -> "STALE DATA — link degraded"
                    else -> "NO DATA"
                },
                color = Color(0xFFB00020),
            )
        }

        // Rocket state
        StatCard("State", telemetry.state, big = true)

        // Power section — #377: never offer the blind cmd-8 toggle until the
        // first telemetry frame of this session confirmed the power state.
        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Rocket power", style = MaterialTheme.typography.titleMedium)
                when {
                    !hasTelemetry -> Text("Waiting for telemetry — power state unknown")
                    poweringOn -> Text("Powering on… (flushing flight log)")
                    telemetry.pwrPinOn -> Row(
                        horizontalArrangement = Arrangement.spacedBy(12.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text("ON")
                        Button(onClick = { session.sendPowerToggle() }) { Text("Power off") }
                    }
                    else -> Row(
                        horizontalArrangement = Arrangement.spacedBy(12.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text("OFF")
                        Button(onClick = { session.beginPowerOn() }) { Text("Power on") }
                    }
                }
            }
        }

        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            StatCard(
                "Battery",
                telemetry.voltage?.let { String.format(Locale.ROOT, "%.2f V", it) } ?: "—",
                Modifier.weight(1f),
            )
            StatCard(
                "GNSS",
                if (telemetry.numSats > 0) "${telemetry.numSats} sats" else "no fix",
                Modifier.weight(1f),
            )
            StatCard("Link", rssi?.let { "$it dBm" } ?: "—", Modifier.weight(1f))
        }

        // Pyro tiles: fail-safe rendering — a channel shows green ONLY on
        // continuity AND live data (stale green continuity is a lie).
        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    "Pyro ${if (telemetry.pyroArmed) "ARMED" else "safe"}",
                    style = MaterialTheme.typography.titleMedium,
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    val live = dataStatus == TelemetryData.DataStatus.LIVE
                    PyroTile(1, telemetry.pyro1Cont, telemetry.pyro1Fired, live)
                    PyroTile(2, telemetry.pyro2Cont, telemetry.pyro2Fired, live)
                    PyroTile(3, telemetry.pyro3Cont, telemetry.pyro3Fired, live)
                    PyroTile(4, telemetry.pyro4Cont, telemetry.pyro4Fired, live)
                }
            }
        }

        // Altitude row (bench: baro only until the FC rail is up)
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            StatCard(
                "Baro alt",
                telemetry.pressureAlt?.let { String.format(Locale.ROOT, "%.1f m", it) } ?: "—",
                Modifier.weight(1f),
            )
            StatCard(
                "Max alt",
                telemetry.maxAltM?.let { String.format(Locale.ROOT, "%.1f m", it) } ?: "—",
                Modifier.weight(1f),
            )
        }
    }
}

@Composable
private fun Banner(text: String, color: Color) {
    Text(
        text,
        color = Color.White,
        style = MaterialTheme.typography.titleMedium,
        modifier = Modifier
            .fillMaxWidth()
            .background(color, RoundedCornerShape(8.dp))
            .padding(horizontal = 16.dp, vertical = 10.dp),
    )
}

@Composable
private fun StatCard(
    label: String,
    value: String,
    modifier: Modifier = Modifier,
    big: Boolean = false,
) {
    Card(modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp)) {
            Text(label, style = MaterialTheme.typography.labelMedium)
            Text(
                value,
                style = if (big) MaterialTheme.typography.headlineMedium
                else MaterialTheme.typography.titleLarge,
            )
        }
    }
}

@Composable
private fun PyroTile(ch: Int, continuity: Boolean, fired: Boolean, live: Boolean) {
    val color = when {
        fired -> Color(0xFF616161)
        continuity && live -> Color(0xFF2E7D32)   // green requires cont AND live
        else -> Color(0xFF9E9E9E)
    }
    Column(
        Modifier
            .background(color, RoundedCornerShape(8.dp))
            .padding(horizontal = 14.dp, vertical = 10.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("CH$ch", color = Color.White, style = MaterialTheme.typography.labelMedium)
        Text(
            when {
                fired -> "fired"
                continuity -> "cont"
                else -> "open"
            },
            color = Color.White,
            style = MaterialTheme.typography.bodySmall,
        )
    }
}
