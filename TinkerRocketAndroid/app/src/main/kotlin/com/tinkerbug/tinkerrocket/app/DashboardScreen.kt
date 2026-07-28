package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.horizontalScroll
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
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
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
fun DashboardScreen(
    device: FleetDevice<DeviceSession>,
    onDisconnect: () -> Unit,
    demo: Boolean = false,
    syncer: com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer? = null,
    phoneLocation: PhoneLocationManager? = null,
    profileStore: com.tinkerbug.tinkerrocket.session.RocketProfileStore? = null,
) {
    val session = device.session

    // Tools sub-routes (iOS: sheets from the dashboard).
    var tool by androidx.compose.runtime.remember {
        androidx.compose.runtime.mutableStateOf<String?>(null)
    }
    when (tool) {
        "servo" -> { ServoTestScreen(session, profileStore, onBack = { tool = null }); return }
        "sim" -> { SimulationScreen(session, onBack = { tool = null }); return }
        "scan" -> { FreqScanScreen(session, onBack = { tool = null }); return }
    }

    // Phone GPS/compass run only while the dashboard is visible (the iOS
    // onAppear/onDisappear discipline — continuous updates leak battery).
    if (phoneLocation != null) {
        androidx.compose.runtime.DisposableEffect(Unit) {
            phoneLocation.start()
            onDispose { phoneLocation.stop() }
        }
    }
    val telemetry by session.telemetry.collectAsState()
    val identity by session.identity.collectAsState()
    val hasTelemetry by session.hasReceivedTelemetry.collectAsState()
    val dataStatus by session.effectiveDataStatus.collectAsState()
    val rssi by session.connectedRssi.collectAsState()
    val poweringOn by session.poweringOn.collectAsState()
    val remoteRockets by session.remoteRockets.collectAsState()
    val focusRocketId by session.focusRocketId.collectAsState()

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
                    (identity.unitName.ifEmpty { device.advertisedName }) +
                        if (demo) "  (demo)" else "",
                    style = MaterialTheme.typography.titleLarge,
                )
                Text(
                    "nid ${identity.networkId ?: "—"} · rid ${identity.rocketId ?: "—"} · " +
                        "fw ${identity.firmwareVersion ?: "—"}",
                    style = MaterialTheme.typography.bodySmall,
                )
                // #375: "connected but not yet pushed" must never render
                // as silent nothing — the sync badge is always on the header.
                syncer?.let {
                    val syncState by it.syncState.collectAsState()
                    SyncStateLine(syncState)
                }
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

        // Direction/distance to rocket (BS links only — the recovery walk).
        // Reads the LATCHED lastValidRocketFix, never per-frame lat/lon:
        // relay frames frequently arrive with lat/lon = nil (#140).
        if (phoneLocation != null && session.isBaseStation) {
            val lastFix by session.lastValidRocketFix.collectAsState()
            DirectionToRocketCard(
                phoneLocation = phoneLocation,
                fix = lastFix,
                rocketAltM = (telemetry.gnssAlt ?: telemetry.pressureAlt)?.toDouble(),
            )
        }

        // Relayed rockets (#390): base-station links list every rocket the
        // BS hears; the focused one mirrors into this dashboard.  Tap to
        // switch focus (cmd 45 → the BS radio follows).
        if (remoteRockets.isNotEmpty()) {
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text("Rockets via base station", style = MaterialTheme.typography.titleMedium)
                    Row(
                        Modifier.horizontalScroll(rememberScrollState()),
                        horizontalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        remoteRockets.forEach { remote ->
                            val focused = remote.rocketId == focusRocketId
                            Column(
                                Modifier
                                    .background(
                                        if (focused) Color(0xFF4527A0) else Color(0x33777777),
                                        RoundedCornerShape(8.dp),
                                    )
                                    .clickable { session.setFocusRocket(remote.rocketId) }
                                    .padding(horizontal = 14.dp, vertical = 8.dp),
                                horizontalAlignment = Alignment.CenterHorizontally,
                            ) {
                                Text(
                                    remote.unitName.ifEmpty { "Rocket ${remote.rocketId}" },
                                    color = if (focused) Color.White else Color(0xFFBBBBBB),
                                    style = MaterialTheme.typography.labelLarge,
                                )
                                Text(
                                    if (focused) "focused" else remote.telemetry.state,
                                    color = if (focused) Color(0xFFB39DDB) else Color(0xFF888888),
                                    style = MaterialTheme.typography.labelSmall,
                                )
                            }
                        }
                    }
                }
            }
        }

        // Flight event flags
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            FlagChip("LAUNCH", telemetry.launchFlag)
            FlagChip("BURNOUT", telemetry.burnoutFlag)
            FlagChip("APOGEE", telemetry.altApo || telemetry.velApo)
            FlagChip("LANDED", telemetry.landedFlag)
            FlagChip("LOG", telemetry.loggingActive)
        }

        // Sensor health scorecard (#303) — shown once the frame carries it.
        if (telemetry.hasSensorHealth) {
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("Sensor health", style = MaterialTheme.typography.titleMedium)
                    Row(
                        Modifier.horizontalScroll(rememberScrollState()),
                        horizontalArrangement = Arrangement.spacedBy(10.dp),
                    ) {
                        telemetry.sensorHealthRows.forEach { row ->
                            HealthDot(row.name, row.state)
                        }
                    }
                }
            }
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

        // Tools (iOS: dashboard sheet buttons).  Scan runs on the BS
        // radio (cmd 60, BS links only); servo test + sim talk to a rocket.
        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Tools", style = MaterialTheme.typography.titleMedium)
                Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    if (!session.isBaseStation) {
                        OutlinedButton(onClick = { tool = "servo" }) { Text("Servo test") }
                    }
                    OutlinedButton(onClick = { tool = "sim" }) { Text("Simulation") }
                    if (session.isBaseStation) {
                        OutlinedButton(onClick = { tool = "scan" }) { Text("Freq scan") }
                    }
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

/**
 * Direction arrow + distance/altitude to the rocket (iOS Signal-panel
 * arrow).  Arrow angle = bearing(phone→rocket) − phone heading, so the
 * arrow points where the operator should walk.  Waiting states say WHY
 * the arrow is hidden instead of showing nothing; a denied permission is
 * its own state with a Grant button (the plan's denied-state UI).
 */
@Composable
private fun DirectionToRocketCard(
    phoneLocation: PhoneLocationManager,
    fix: com.tinkerbug.tinkerrocket.session.LastValidRocketFix?,
    rocketAltM: Double?,
) {
    val phoneFix by phoneLocation.location.collectAsState()
    val heading by phoneLocation.headingDeg.collectAsState()
    var permission by androidx.compose.runtime.remember {
        androidx.compose.runtime.mutableStateOf(phoneLocation.hasPermission())
    }
    val launcher = androidx.activity.compose.rememberLauncherForActivityResult(
        androidx.activity.result.contract.ActivityResultContracts.RequestPermission(),
    ) { granted ->
        permission = granted
        if (granted) phoneLocation.restartIfHeld()
    }

    Card(Modifier.fillMaxWidth()) {
        Column(
            Modifier.fillMaxWidth().padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            Text("Rocket bearing", style = MaterialTheme.typography.titleMedium)
            when {
                !permission -> {
                    Text(
                        "Location permission is needed to show direction " +
                            "and distance to the rocket.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                    OutlinedButton(onClick = {
                        launcher.launch(android.Manifest.permission.ACCESS_FINE_LOCATION)
                    }) { Text("Grant location") }
                }
                phoneFix == null -> Text(
                    "Getting phone location…",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                fix == null -> Text(
                    "Waiting for rocket GPS",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                else -> {
                    val p = phoneFix!!
                    val dist = com.tinkerbug.tinkerrocket.session.DriftCast.haversineM(
                        p.lat, p.lon, fix.latitude, fix.longitude,
                    )
                    val bear = com.tinkerbug.tinkerrocket.session.DriftCast.bearingDeg(
                        p.lat, p.lon, fix.latitude, fix.longitude,
                    )
                    val arrowAngle = ((bear - heading + 180.0).mod(360.0) - 180.0).toFloat()
                    Text(
                        "➤",
                        style = MaterialTheme.typography.displayMedium,
                        color = Color(0xFF1E88E5),
                        // Glyph points east (90°); rotate −90 to make it north-up.
                        modifier = Modifier.rotate(arrowAngle - 90f),
                    )
                    val distText =
                        if (dist < 1000) "%.0f m".format(dist) else "%.2f km".format(dist / 1000)
                    Text(
                        "$distText away",
                        style = MaterialTheme.typography.bodyMedium,
                    )
                    val phoneAlt = p.altMslM
                    if (phoneAlt != null && rocketAltM != null) {
                        val diff = rocketAltM - phoneAlt
                        Text(
                            "%.0f m %s".format(kotlin.math.abs(diff), if (diff >= 0) "up" else "down"),
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                }
            }
        }
    }
}

@Composable
private fun SyncStateLine(state: com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState) {
    val (label, color) = when (state) {
        com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.Idle -> return
        com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.AwaitingSync ->
            "profile: awaiting sync" to Color(0xFF9E9E9E)
        com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.NoProfile ->
            "profile: none active" to Color(0xFFFFA000)
        com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.Syncing ->
            "profile: syncing…" to Color(0xFF1E88E5)
        com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.Synced ->
            "profile: synced" to Color(0xFF43A047)
        is com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState.Failed ->
            "profile: sync failed" to Color(0xFFE53935)
    }
    Text(label, style = MaterialTheme.typography.bodySmall, color = color)
}

@Composable
private fun FlagChip(label: String, on: Boolean) {
    Text(
        label,
        color = if (on) Color.White else Color(0xFF757575),
        style = MaterialTheme.typography.labelMedium,
        modifier = Modifier
            .background(
                if (on) Color(0xFF2E7D32) else Color(0x33777777),
                RoundedCornerShape(6.dp),
            )
            .padding(horizontal = 8.dp, vertical = 5.dp),
    )
}

@Composable
private fun HealthDot(name: String, state: TelemetryData.SensorHealth) {
    val color = when (state) {
        TelemetryData.SensorHealth.OK -> Color(0xFF2E7D32)
        TelemetryData.SensorHealth.DEGRADED -> Color(0xFFF9A825)
        TelemetryData.SensorHealth.BAD -> Color(0xFFB00020)
        else -> Color(0xFF616161)
    }
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(
            "●",
            color = color,
            style = MaterialTheme.typography.titleMedium,
        )
        Text(name, style = MaterialTheme.typography.labelSmall)
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
