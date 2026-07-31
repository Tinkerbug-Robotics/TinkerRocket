package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
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
import com.tinkerbug.tinkerrocket.session.UnitFormatter
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
    container: AppContainer? = null,
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
        "magcal" -> { MagCalScreen(session, syncer, onBack = { tool = null }); return }
        "ota" -> {
            if (container != null) {
                FirmwareUpdateScreen(container, device.deviceId, session, onBack = { tool = null })
                return
            }
        }
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
            // weight(1f) so a wide device name YIELDS instead of shoving the
            // action row off-screen: #645's voice chip widened the right side
            // and "Rocket Computer V6" pushed Disconnect past the screen edge
            // — clipped to invisible, leaving no way off the device. The name
            // wraps; the actions never move.
            Column(Modifier.weight(1f)) {
                Text(
                    (identity.unitName.ifEmpty { device.advertisedName }) +
                        if (demo) "  (virtual)" else "",
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
            // Voice + disconnect moved to ConnectedTopBar (iOS toolbar
            // arrangement, design pass) — the header is identity only now,
            // which also retires the #645 clipped-Disconnect hazard for good.
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

        // Rocket state — iOS RocketStateView twin (#382 display mapping:
        // READY and PRELAUNCH both render "PRELAUNCH" with a readiness badge
        // carrying the real distinction; raw wire strings untouched).
        RocketStateBanner(telemetry.state)

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

        // iOS section order (screenshots 2026-07-31): the summary sits right
        // after the banners on BS links but after Signal/Battery/IMU on
        // direct links — matched exactly, per link type.
        if (session.isBaseStation) FlightSummaryCard(telemetry)

        SignalCard(telemetry, rssi, session.isBaseStation)
        BatteryCard(telemetry)
        ImuCard(telemetry, session.isBaseStation)

        if (!session.isBaseStation) FlightSummaryCard(telemetry)

        GpsRow(telemetry)

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

        // Status card (iOS StatusFlagsView port): camera + logging badges,
        // active file, BS silence-close countdown.  Before the flag chips —
        // the iOS order (StatusFlagsView, then FlightEventFlagsView).
        StatusCard(telemetry, session.isBaseStation)

        // Flight event flags.  No LOG chip — the Status card above owns
        // logging (re-converged with iOS 2026-07-30).
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            FlagChip("LAUNCH", telemetry.launchFlag)
            FlagChip("BURNOUT", telemetry.burnoutFlag)
            FlagChip("APOGEE", telemetry.altApo || telemetry.velApo)
            FlagChip("LANDED", telemetry.landedFlag)
        }

        // Sensor health scorecard (#303) — shown once the frame carries it.
        if (telemetry.hasSensorHealth) {
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("Sensor health", style = MaterialTheme.typography.titleMedium)
                    // Go/no-go banner (iOS HealthCardView port) — the rollup
                    // was in TelemetryData all along; only the verdict UI was
                    // missing here.  The banner carries the words; the dots
                    // below stay the glanceable per-sensor detail.
                    ReadinessBanner(telemetry.flightReadiness)
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

        // Pyro tiles: iOS card form (2x2 grid, direct rocket links only —
        // iOS hides pyro on BS links).  Fail-safe rendering — a channel
        // shows green ONLY on continuity AND live data.  Config-driven
        // "Disabled"/trigger text + Test Continuity need the 0xB1 config
        // readback Android doesn't parse yet (ledger follow-up).
        if (!session.isBaseStation) {
            Card(Modifier.fillMaxWidth()) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text(
                        "Pyro Channels${if (telemetry.pyroArmed) " — ARMED" else ""}",
                        style = MaterialTheme.typography.titleMedium,
                    )
                    val live = dataStatus == TelemetryData.DataStatus.LIVE
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        PyroTile(1, telemetry.pyro1Cont, telemetry.pyro1Fired, live, Modifier.weight(1f))
                        PyroTile(2, telemetry.pyro2Cont, telemetry.pyro2Fired, live, Modifier.weight(1f))
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        PyroTile(3, telemetry.pyro3Cont, telemetry.pyro3Fired, live, Modifier.weight(1f))
                        PyroTile(4, telemetry.pyro4Cont, telemetry.pyro4Fired, live, Modifier.weight(1f))
                    }
                }
            }
        }

        // Tools (iOS: dashboard sheet buttons).  Scan runs on the BS
        // Controls (iOS ControlsView port): camera + rocket-log toggles are
        // relay-aware — on a BS link they wrap in the cmd-50 envelope
        // targeting the FOCUSED rocket with the desired state computed from
        // ITS telemetry (#390: the legacy broadcast toggle keyed off
        // whichever rocket was heard last).  Direct links send the bare
        // toggle, exactly like iOS.
        ControlsCard(session, telemetry)

        // radio (cmd 60, BS links only); servo test + sim talk to a rocket.
        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Tools", style = MaterialTheme.typography.titleMedium)
                val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
                Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    if (!session.isBaseStation) {
                        com.tinkerbug.tinkerrocket.app.theme.TrCompactButton(
                            "Servo test", tr.servoTest, { tool = "servo" })
                        com.tinkerbug.tinkerrocket.app.theme.TrCompactButton(
                            "Mag cal", tr.myDevices, { tool = "magcal" })
                    }
                    com.tinkerbug.tinkerrocket.app.theme.TrCompactButton(
                        "Simulate", tr.simulate, { tool = "sim" })
                    if (session.isBaseStation) {
                        com.tinkerbug.tinkerrocket.app.theme.TrCompactButton(
                            "Freq scan", tr.freqScan, { tool = "scan" })
                    }
                    if (container != null) {
                        com.tinkerbug.tinkerrocket.app.theme.TrCompactButton(
                            "Firmware", tr.camera, { tool = "ota" })
                    }
                }
            }
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
                    val arrowAngle = com.tinkerbug.tinkerrocket.session.HeadingMath
                        .arrowAngleDeg(bear, heading).toFloat()
                    Text(
                        "➤",
                        style = MaterialTheme.typography.displayMedium,
                        color = Color(0xFF1E88E5),
                        // Glyph points east (90°); rotate −90 to make it north-up.
                        modifier = Modifier.rotate(arrowAngle - 90f),
                    )
                    val units = com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem.current
                    val distText = UnitFormatter.distance(dist, units)
                    Text(
                        "$distText away",
                        style = MaterialTheme.typography.bodyMedium,
                    )
                    val phoneAlt = p.altMslM
                    if (phoneAlt != null && rocketAltM != null) {
                        val diff = rocketAltM - phoneAlt
                        Text(
                            UnitFormatter.altitude(kotlin.math.abs(diff), units) +
                                (if (diff >= 0) " up" else " down"),
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
private fun PyroTile(ch: Int, continuity: Boolean, fired: Boolean, live: Boolean, modifier: Modifier = Modifier) {
    val color = when {
        fired -> Color(0xFF616161)
        continuity && live -> Color(0xFF2E7D32)   // green requires cont AND live
        else -> Color(0xFF9E9E9E)
    }
    Column(
        modifier
            .background(color, RoundedCornerShape(8.dp))
            .padding(horizontal = 14.dp, vertical = 10.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("CH $ch", color = Color.White, style = MaterialTheme.typography.labelMedium)
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

/**
 * Voice-callout toggle (#643) — the iOS toolbar speaker icon.  Tap toggles;
 * long-press speaks the volume/voice check.  Colour is the readiness ladder
 * from DashboardView: gray = off, red = engine error, orange = enabled but
 * engine not initialised yet, green = ready.
 */
@OptIn(androidx.compose.foundation.ExperimentalFoundationApi::class)
// ── iOS→Android dashboard-section ports (design-language queue, 2026-07-30) ──

/** iOS StatusBadge: label + filled/hollow state dot. */
@Composable
private fun StatusBadge(label: String, active: Boolean) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text(
            "●",
            color = if (active) Color(0xFF2E7D32) else Color(0x55777777),
            style = MaterialTheme.typography.bodyMedium,
        )
        Text(
            " $label",
            style = MaterialTheme.typography.bodyMedium,
            color = if (active) MaterialTheme.colorScheme.onSurface
            else MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

/**
 * iOS StatusFlagsView port: camera + logging badges, active file, and the BS
 * silence-close countdown with the same urgency colors (>60 s green,
 * 11..60 s amber, ≤10 s red — an imminent close is exactly what the operator
 * needs to catch mid-flight).  Logging uses [TelemetryData.rocketLoggingActive]
 * (LANDED-zeroed, #137) — the flag-chip row above used raw loggingActive,
 * which is why the LOG chip left when this card arrived.
 */
@Composable
private fun StatusCard(telemetry: TelemetryData, isBaseStation: Boolean) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Status", style = MaterialTheme.typography.titleMedium)
            Row(horizontalArrangement = Arrangement.spacedBy(20.dp)) {
                StatusBadge("Camera", telemetry.cameraRecording)
                StatusBadge(
                    if (isBaseStation) "Rocket log" else "Logging",
                    telemetry.rocketLoggingActive,
                )
                if (isBaseStation) {
                    StatusBadge("Base stn log", telemetry.bsLoggingActive)
                }
            }
            if (telemetry.activeFile.isNotEmpty()) {
                Text(
                    "File: ${telemetry.activeFile}",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            val remaining = telemetry.bsLogSilenceRemainingS
            if (isBaseStation && telemetry.bsLoggingActive && remaining != null) {
                val color = when {
                    remaining > 60 -> Color(0xFF2E7D32)
                    remaining > 10 -> Color(0xFFF9A825)
                    else -> Color(0xFFB00020)
                }
                Text(
                    "Auto-close in %d:%02d".format(remaining / 60, remaining % 60),
                    style = MaterialTheme.typography.bodySmall,
                    color = color,
                )
            }
        }
    }
}

/** iOS HealthCardView go/no-go banner: tinted row carrying the rollup verdict. */
@Composable
private fun ReadinessBanner(readiness: TelemetryData.FlightReadiness) {
    val color = when (readiness) {
        TelemetryData.FlightReadiness.READY -> Color(0xFF2E7D32)
        TelemetryData.FlightReadiness.CAUTION -> Color(0xFFF9A825)
        TelemetryData.FlightReadiness.NOT_READY -> Color(0xFFB00020)
        TelemetryData.FlightReadiness.UNKNOWN -> Color(0xFF616161)
    }
    val glyph = when (readiness) {
        TelemetryData.FlightReadiness.READY -> "✓"
        TelemetryData.FlightReadiness.CAUTION -> "⚠"
        TelemetryData.FlightReadiness.NOT_READY -> "✕"
        TelemetryData.FlightReadiness.UNKNOWN -> "…"
    }
    Row(
        Modifier
            .fillMaxWidth()
            .background(color.copy(alpha = 0.12f), RoundedCornerShape(8.dp))
            .padding(horizontal = 12.dp, vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Text(glyph, color = color, style = MaterialTheme.typography.titleMedium)
        Text(
            readiness.label,
            color = color,
            style = MaterialTheme.typography.titleSmall,
        )
    }
}

/**
 * iOS ControlsView port.  Relay-aware exactly like iOS: BS link + focused
 * rocket → cmd-50 envelope with the DESIRED state computed from that
 * rocket's telemetry; direct link → bare toggle.  BS logging (cmd 46) never
 * uplinks — it's the BS's own CSV (#390 decoupling).
 */
@Composable
private fun ControlsCard(session: DeviceSession, telemetry: TelemetryData) {
    val focus by session.focusRocketId.collectAsState()
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Controls", style = MaterialTheme.typography.titleMedium)
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                val cam = telemetry.cameraRecording
                Button(
                    onClick = {
                        val f = focus
                        if (session.isBaseStation && f != null) {
                            session.sendCommandFrame(
                                com.tinkerbug.tinkerrocket.protocol.Commands.relayToRocket(
                                    f,
                                    com.tinkerbug.tinkerrocket.protocol.Commands
                                        .cameraToggleWithState(!cam),
                                ),
                            )
                        } else {
                            session.sendBareCommand(
                                com.tinkerbug.tinkerrocket.protocol.BleCommandId.CAMERA_TOGGLE,
                            )
                        }
                    },
                    enabled = !session.isBaseStation || focus != null,
                    modifier = Modifier.weight(1f),
                ) { Text(if (cam) "Stop camera" else "Start camera") }

                val log = telemetry.rocketLoggingActive
                Button(
                    onClick = {
                        val f = focus
                        if (session.isBaseStation && f != null) {
                            session.sendCommandFrame(
                                com.tinkerbug.tinkerrocket.protocol.Commands.relayToRocket(
                                    f,
                                    com.tinkerbug.tinkerrocket.protocol.Commands
                                        .toggleLoggingWithState(!log),
                                ),
                            )
                        } else {
                            session.sendBareCommand(
                                com.tinkerbug.tinkerrocket.protocol.BleCommandId.TOGGLE_LOGGING,
                            )
                        }
                    },
                    enabled = !session.isBaseStation || focus != null,
                    modifier = Modifier.weight(1f),
                    colors = androidx.compose.material3.ButtonDefaults.buttonColors(
                        containerColor = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors.logging,
                    ),
                ) { Text(if (log) "Stop logging" else "Start logging") }
            }
            if (session.isBaseStation) {
                val bsLog = telemetry.bsLoggingActive
                Button(
                    onClick = {
                        session.sendCommandFrame(
                            com.tinkerbug.tinkerrocket.protocol.Commands.bsLogging(!bsLog),
                        )
                    },
                    modifier = Modifier.fillMaxWidth(),
                ) { Text(if (bsLog) "Stop base stn log" else "Start base stn log") }
            }
        }
    }
}

/** Binary-unit byte formatting to match iOS ByteCountFormatter(.binary). */
private fun fmtBytes(bytes: Long): String = when {
    bytes >= 1L shl 30 -> String.format(Locale.ROOT, "%.1f GB", bytes / 1073741824.0)
    else -> String.format(Locale.ROOT, "%.0f MB", bytes / 1048576.0)
}

/**
 * iOS StorageBarView port: segmented used/reserved/free bar + legend.
 * Rocket variant (0xCC) carries flight count + the #315 auto-evict note;
 * BS variant (0xCD) names its backend.  Absent stats render nothing —
 * the frames arrive on the file-ops characteristic after connect.
 */
@Composable
internal fun StorageCard(
    isBaseStation: Boolean,
    rocket: com.tinkerbug.tinkerrocket.protocol.RocketStorageStats?,
    bs: com.tinkerbug.tinkerrocket.protocol.BaseStationStorageStats?,
) {
    val (title, subtitle, used, reserved, free, total, autoEvicted) = when {
        isBaseStation && bs != null && bs.totalBytes > 0 -> {
            val backend = listOf("SPIFFS", "SD card", "NAND").getOrElse(bs.backend) { "?" }
            StorageRow("Base station storage", backend, bs.usedBytes, bs.reservedBytes,
                bs.freeBytes, bs.totalBytes, false)
        }
        !isBaseStation && rocket != null && rocket.initialized -> {
            val n = rocket.flightCount
            StorageRow("Rocket storage", "$n flight${if (n == 1) "" else "s"}",
                rocket.usedBytes, rocket.reservedBytes, rocket.freeBytes,
                rocket.totalBytes, rocket.autoEvicted)
        }
        else -> return
    }
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Text(title, style = MaterialTheme.typography.titleMedium)
                Text(
                    subtitle,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            // Segment bar: used | reserved | free, weighted by bytes.
            Row(
                Modifier
                    .fillMaxWidth()
                    .background(Color(0x22777777), RoundedCornerShape(4.dp)),
            ) {
                val t = total.coerceAtLeast(1)
                @Composable
                fun seg(bytes: Long, color: Color) {
                    val w = bytes.toFloat() / t
                    if (w > 0f) {
                        Text(
                            "",
                            modifier = Modifier
                                .weight(w.coerceAtLeast(0.001f))
                                .background(color)
                                .padding(vertical = 5.dp),
                        )
                    }
                }
                seg(used, Color(0xFFF57C00))
                seg(reserved, Color(0xFF9E9E9E))
                seg(free, Color(0xFF2E7D32))
            }
            Row(horizontalArrangement = Arrangement.spacedBy(14.dp)) {
                Text("Used ${fmtBytes(used)}", style = MaterialTheme.typography.bodySmall)
                if (reserved > 0) {
                    Text("Reserved ${fmtBytes(reserved)}", style = MaterialTheme.typography.bodySmall)
                }
                Text("Free ${fmtBytes(free)}", style = MaterialTheme.typography.bodySmall)
            }
            if (autoEvicted) {
                // #315 rolling buffer: surface that data rolled off at arm
                // time rather than being silently dropped.
                Text(
                    "↻ Auto-reclaimed oldest flight(s) this session",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }
}

private data class StorageRow(
    val title: String,
    val subtitle: String,
    val used: Long,
    val reserved: Long,
    val free: Long,
    val total: Long,
    val autoEvicted: Boolean,
)

/**
 * iOS RocketStateView twin (#382 display-only mapping): the wire states
 * READY and PRELAUNCH both mean "on the pad" — READY is still waiting on
 * the OC/GNSS gates, PRELAUNCH means the gates are met.  The raw names read
 * backwards, so both render one "PRELAUNCH" label and a readiness badge
 * carries the real distinction.  Colors match iOS: acquiring orange, ready
 * green, INFLIGHT red, COMPLETE/MAG_CAL blue, else gray.  Wire strings,
 * CSV columns, and the announcer's raw-state logic are untouched.
 */
@Composable
private fun RocketStateBanner(state: String) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val color = when (state) {
        "READY" -> tr.statusScanning      // acquiring (orange)
        "PRELAUNCH" -> tr.statusConnected // fully ready (green)
        "INFLIGHT" -> tr.driftCast        // red
        "COMPLETE", "MAG_CAL" -> tr.savedFlights
        else -> tr.statusIdle
    }
    // The COLOR carries the acquiring-vs-ready distinction; the text badge
    // that used to sit under the label was removed 2026-07-31 on both
    // platforms ("EKF Ready" beside an amber EKF health dot read as a
    // contradiction — the sensor dots own live quality).
    val label = if (state == "READY") "PRELAUNCH" else state
    Column(
        Modifier
            .fillMaxWidth()
            .background(color.copy(alpha = 0.1f), RoundedCornerShape(10.dp))
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(4.dp),
    ) {
        Text(
            label,
            color = color,
            fontSize = androidx.compose.ui.unit.TextUnit(
                36f, androidx.compose.ui.unit.TextUnitType.Sp),
            fontWeight = androidx.compose.ui.text.font.FontWeight.Bold,
        )
    }
}

/**
 * iOS FlightSummaryView twin: Current/Max monospace table for altitude and
 * speed, plus the attitude row (Roll = body-Z azimuth, Pitch/Yaw = ZYX
 * Euler — display convention, see the CSV-attitude note) when the frame
 * carries a quaternion.  Units per the global setting (#160).
 */
@Composable
private fun FlightSummaryCard(telemetry: TelemetryData) {
    val units = com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem.current
    val mono = androidx.compose.ui.text.font.FontFamily.Monospace
    val caption = MaterialTheme.typography.bodySmall

    @Composable
    fun row(label: String, current: String, max: String) {
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            Text(label, style = caption, modifier = Modifier.width(70.dp))
            Text(
                current, style = caption.copy(fontFamily = mono),
                textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                modifier = Modifier.weight(1f),
            )
            Text(
                max, style = caption.copy(fontFamily = mono),
                textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                modifier = Modifier.weight(1f),
            )
        }
    }

    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Row(Modifier.fillMaxWidth()) {
                Text("", modifier = Modifier.width(70.dp))
                Text(
                    "Current", style = caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                    modifier = Modifier.weight(1f),
                )
                Text(
                    "Max", style = caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                    modifier = Modifier.weight(1f),
                )
            }
            row(
                "Altitude",
                telemetry.pressureAlt?.let { UnitFormatter.altitude(it.toDouble(), units) } ?: "—",
                telemetry.maxAltM?.let { UnitFormatter.altitude(it.toDouble(), units) } ?: "—",
            )
            row(
                "Speed",
                telemetry.altitudeRate?.let { UnitFormatter.speed(it.toDouble(), units) } ?: "—",
                telemetry.maxSpeedMps?.let { UnitFormatter.speed(it.toDouble(), units) } ?: "—",
            )
            val roll = telemetry.roll
            val pitch = telemetry.pitch
            val yaw = telemetry.yaw
            Row(Modifier.fillMaxWidth()) {
                Text("", modifier = Modifier.width(70.dp))
                listOf("Roll", "Pitch", "Yaw").forEach {
                    Text(
                        it, style = caption,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.weight(1f),
                    )
                }
            }
            Row(Modifier.fillMaxWidth()) {
                Text("Attitude", style = caption, modifier = Modifier.width(70.dp))
                listOf(roll, pitch, yaw).forEach { v ->
                    Text(
                        v?.let { String.format(Locale.ROOT, "%.0f°", it) } ?: "—",
                        style = caption.copy(fontFamily = mono),
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.weight(1f),
                    )
                }
            }
        }
    }
}

/**
 * iOS SignalStrengthView twin: vertical bars for LoRa (BS links), GNSS
 * satellites, and BLE RSSI, value labels underneath.  Fill fraction uses the
 * same rough scaling ideas as iOS (RSSI −100..−30 dBm, sats 0..12).
 */
@Composable
private fun SignalCard(telemetry: TelemetryData, bleRssi: Int?, isBaseStation: Boolean) {
    Card(Modifier.fillMaxWidth()) {
        Column(
            Modifier.fillMaxWidth().padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text("Signal", style = MaterialTheme.typography.titleMedium)
            Row(horizontalArrangement = Arrangement.spacedBy(28.dp)) {
                if (isBaseStation) {
                    val lora = telemetry.rssi
                    SignalBar(
                        fraction = lora?.let { ((it + 100f) / 70f).coerceIn(0f, 1f) } ?: 0f,
                        value = lora?.let { String.format(Locale.ROOT, "%.0f", it) } ?: "——",
                        label = "LoRa",
                    )
                }
                SignalBar(
                    fraction = (telemetry.numSats / 12f).coerceIn(0f, 1f),
                    value = "${telemetry.numSats}",
                    label = "GNSS",
                )
                SignalBar(
                    fraction = bleRssi?.let { ((it + 100f) / 70f).coerceIn(0f, 1f) } ?: 0f,
                    value = bleRssi?.let { "$it" } ?: "——",
                    label = "BLE",
                )
            }
        }
    }
}

@Composable
private fun SignalBar(fraction: Float, value: String, label: String) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    Column(
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(4.dp),
    ) {
        Box(
            Modifier
                .height(120.dp)
                .width(36.dp)
                .background(tr.cardSecondary.copy(alpha = 0.5f), RoundedCornerShape(20.dp)),
            contentAlignment = Alignment.BottomCenter,
        ) {
            if (fraction > 0f) {
                Box(
                    Modifier
                        .padding(bottom = 6.dp)
                        .width(24.dp)
                        .height((10 + 100 * fraction).dp)
                        .background(tr.logging, RoundedCornerShape(12.dp)),
                )
            }
        }
        Text(value, style = MaterialTheme.typography.bodyLarge)
        Text(
            label, style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

/** iOS BatteryView twin: Charge / Voltage / Current row for the rocket. */
@Composable
private fun BatteryCard(telemetry: TelemetryData) {
    val caption = MaterialTheme.typography.bodySmall
    val mono = androidx.compose.ui.text.font.FontFamily.Monospace
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text("Battery", style = MaterialTheme.typography.titleMedium)
            Row(Modifier.fillMaxWidth()) {
                Text("", modifier = Modifier.width(70.dp))
                listOf("Charge", "Voltage", "Current").forEach {
                    Text(
                        it, style = caption,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.weight(1f),
                    )
                }
            }
            Row(Modifier.fillMaxWidth()) {
                Text("Rocket", style = caption, modifier = Modifier.width(70.dp))
                listOf(
                    telemetry.soc?.let { String.format(Locale.ROOT, "%.1f%%", it) } ?: "—",
                    telemetry.voltage?.let { String.format(Locale.ROOT, "%.2f V", it) } ?: "—",
                    telemetry.current?.let { String.format(Locale.ROOT, "%.0f mA", it) } ?: "—",
                ).forEach {
                    Text(
                        it, style = caption.copy(fontFamily = mono),
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.weight(1f),
                    )
                }
            }
        }
    }
}

/**
 * iOS IMUView twin: Low-G / High-G / Gyro XYZ triplets.  The nose
 * orientation annotation shows on BS links (relayed flags2, #390); direct
 * links need the 0xB1 config readback Android doesn't parse yet (ledger).
 */
@Composable
private fun ImuCard(telemetry: TelemetryData, isBaseStation: Boolean) {
    val caption = MaterialTheme.typography.bodySmall
    val mono = androidx.compose.ui.text.font.FontFamily.Monospace

    @Composable
    fun triplet(label: String, unit: String, x: Float?, y: Float?, z: Float?, decimals: Int) {
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.width(70.dp)) {
                Text(label, style = caption)
                Text(unit, style = caption, color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
            listOf(x, y, z).forEach { v ->
                Text(
                    v?.let { String.format(Locale.ROOT, "%.${decimals}f", it) } ?: "—",
                    style = caption.copy(fontFamily = mono),
                    textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                    modifier = Modifier.weight(1f),
                )
            }
        }
    }

    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Text("IMU", style = MaterialTheme.typography.titleMedium, modifier = Modifier.weight(1f))
                val orient = if (isBaseStation) telemetry.relayedOrientationName else ""
                if (orient.isNotEmpty()) {
                    Text(
                        "nose: $orient",
                        style = caption,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
            Row(Modifier.fillMaxWidth()) {
                Text("", modifier = Modifier.width(70.dp))
                listOf("X", "Y", "Z").forEach {
                    Text(
                        it, style = caption,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.weight(1f),
                    )
                }
            }
            triplet("Low-G", "m/s²", telemetry.lowGX, telemetry.lowGY, telemetry.lowGZ, 2)
            triplet("High-G", "m/s²", telemetry.highGX, telemetry.highGY, telemetry.highGZ, 1)
            triplet("Gyro", "°/s", telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ, 1)
        }
    }
}

/** iOS GPSView (compact) twin: lat, lon + sat count on one row. */
@Composable
private fun GpsRow(telemetry: TelemetryData) {
    val mono = androidx.compose.ui.text.font.FontFamily.Monospace
    Card(Modifier.fillMaxWidth()) {
        Row(
            Modifier.fillMaxWidth().padding(16.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("GPS", style = MaterialTheme.typography.titleMedium)
            Text(
                String.format(
                    Locale.ROOT, "%.6f, %.6f",
                    telemetry.latitude ?: 0.0, telemetry.longitude ?: 0.0,
                ),
                style = MaterialTheme.typography.bodyMedium.copy(fontFamily = mono),
                modifier = Modifier.weight(1f),
                textAlign = androidx.compose.ui.text.style.TextAlign.Center,
            )
            Text(
                "${telemetry.numSats} sats",
                style = MaterialTheme.typography.bodyMedium.copy(fontFamily = mono),
            )
        }
    }
}
