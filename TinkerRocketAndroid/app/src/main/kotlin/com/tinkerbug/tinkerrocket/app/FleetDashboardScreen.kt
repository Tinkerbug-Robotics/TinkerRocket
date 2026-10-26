package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.CellTower
import androidx.compose.material.icons.filled.ExpandLess
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material.icons.filled.GpsFixed
import androidx.compose.material.icons.filled.Smartphone
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.key
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import com.tinkerbug.tinkerrocket.session.DashboardSection
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.FleetDashboard
import com.tinkerbug.tinkerrocket.session.FleetDevice
import com.tinkerbug.tinkerrocket.session.FleetManager
import com.tinkerbug.tinkerrocket.session.RocketFreshness
import com.tinkerbug.tinkerrocket.session.RocketRoster
import com.tinkerbug.tinkerrocket.session.RocketSubject
import com.tinkerbug.tinkerrocket.session.UnitFormatter
import kotlinx.coroutines.delay

/**
 * #1650: the rocket-centric dashboard (iOS `ConnectedFleetView` and
 * FleetDashboardComponents.swift): the pair switcher with two or more base
 * stations, the units bar, the foreground base station's strip, then one
 * section per displayed rocket.
 *
 * A direct rocket, and the rocket the foreground base station follows, render
 * the full device dashboard through [deviceContent].  Every other relayed
 * rocket renders its own card stack and Camera/Logging, addressed to it.  The
 * decisions live in [FleetDashboard] (`:core:session`), where they are tested.
 *
 * @param activeDevice rendered alone when the roster has nothing to show yet
 *   (a link whose session has not come up).
 */
@Composable
internal fun FleetDashboardContent(
    fleet: FleetManager<DeviceSession>,
    activeDevice: FleetDevice<DeviceSession>,
    phoneLocation: PhoneLocationManager?,
    deviceContent: @Composable (FleetDevice<DeviceSession>, DashboardSection?) -> Unit,
) {
    val devices by fleet.devices.collectAsState()
    // Rebuilt on every connect, drop, identity readback and relayed-roster
    // change, which are also everything the base-station picks below read.
    val roster by remember(devices) { FleetDashboard.rosterFlow(devices.values) }
        .collectAsState(initial = RocketRoster.build(devices.values))
    val foregroundId by fleet.foregroundBSID.collectAsState()
    val hidden by fleet.hiddenRocketKeys.collectAsState()
    val stations = FleetDashboard.baseStations(devices.values)
    val foreground = FleetDashboard.foregroundBaseStation(devices.values, foregroundId)
    val focus = foreground?.session?.focusRocketId?.collectAsState()?.value
    val layout = FleetDashboard.layout(roster, foreground, focus, hidden)

    // The phone IS the base station's position — the BS has no GNSS — so push
    // it down and let the BS write it into its CSV.  Without this the range
    // between the two ends is displayed once and then discarded, and a range
    // test cannot be reconstructed from its own logs (the 2026-08-08 Kaua'i
    // test only survived because the site was remembered).  Throttled inside
    // reportFix; the BS ignores the write unless a logging session is open.
    // The foreground base station, as iOS's strip does.
    if (phoneLocation != null && foreground != null) {
        val phoneFix by phoneLocation.location.collectAsState()
        LaunchedEffect(foreground.deviceId, phoneFix) {
            if (phoneFix != null) phoneLocation.reportFix(foreground.session)
        }
    }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        if (stations.size > 1) {
            PairSwitcher(stations, foreground) { fleet.setForegroundBS(it) }
        }
        if (layout.scoped.isNotEmpty()) {
            UnitsBar(layout.scoped, foreground, focus, hidden, fleet)
        }
        foreground?.let { BaseStationStrip(it, focus) }

        for (section in layout.sections) {
            key(section.subject.id) {
                when (section) {
                    is DashboardSection.Direct -> DeviceSection(
                        section = section,
                        device = section.device,
                        collapsible = layout.collapsible,
                        // Solo, the units-bar chip already names the rocket.
                        showHeaderWhenSolo = { false },
                        deviceContent = deviceContent,
                        knownDevices = fleet.knownDevices,
                    )
                    is DashboardSection.FocusedRelay -> DeviceSection(
                        section = section,
                        device = section.baseStation,
                        collapsible = layout.collapsible,
                        // Solo, the strip says which rocket the radio follows —
                        // unless the header has something the strip cannot say.
                        showHeaderWhenSolo = { now ->
                            FleetDashboard.needsHeaderWhenSolo(section.subject, now)
                        },
                        deviceContent = deviceContent,
                        knownDevices = fleet.knownDevices,
                    )
                    is DashboardSection.Relay -> RelayRocketSection(
                        section = section,
                        collapsible = layout.collapsible,
                        followTarget = FleetDashboard.followTarget(section.subject, foreground, focus),
                        fleet = fleet,
                        phoneLocation = phoneLocation,
                    )
                }
            }
        }

        // Base station connected but nothing heard yet: its own dashboard
        // renders the searching state.
        layout.searchingBaseStation?.let { deviceContent(it, null) }
        if (layout.scoped.isEmpty() && layout.searchingBaseStation == null) {
            deviceContent(activeDevice, null)
        }
    }
}

/** Wall-clock millis, advancing every [periodMs] so ages move between frames. */
@Composable
private fun rememberNowMs(periodMs: Long = 1_000): Long {
    var now by remember { mutableLongStateOf(System.currentTimeMillis()) }
    LaunchedEffect(periodMs) {
        while (true) {
            delay(periodMs)
            now = System.currentTimeMillis()
        }
    }
    return now
}

@Composable
private fun freshnessColor(freshness: RocketFreshness): Color {
    val tr = TrTheme.colors
    return when (freshness) {
        RocketFreshness.Live -> tr.statusOk
        is RocketFreshness.Stale -> tr.statusWarn
        is RocketFreshness.Lost -> tr.statusIdle
    }
}

// ── Pair switcher ─────────────────────────────────────────────────────────

/**
 * Which base station's pair (the BS and the rockets it carries) the dashboard
 * foregrounds; only with two or more connected.  A pure view change: both
 * links keep streaming, and voice follows the pick (#1098).
 */
@Composable
private fun PairSwitcher(
    stations: List<FleetDevice<DeviceSession>>,
    foreground: FleetDevice<DeviceSession>?,
    onPick: (String) -> Unit,
) {
    val tr = TrTheme.colors
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        for (bs in stations) {
            val picked = bs.deviceId == foreground?.deviceId
            val fg = if (picked) Color.White else MaterialTheme.colorScheme.onSurface
            Row(
                Modifier
                    .weight(1f)
                    .clip(RoundedCornerShape(10.dp))
                    .background(if (picked) tr.accent else tr.cardSecondary)
                    .clickable { onPick(bs.deviceId) }
                    .padding(horizontal = 14.dp, vertical = 8.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp, Alignment.CenterHorizontally),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Icon(Icons.Filled.CellTower, null, tint = fg, modifier = Modifier.size(14.dp))
                Text(
                    bs.session.displayName,
                    color = fg,
                    style = MaterialTheme.typography.labelLarge,
                    fontWeight = FontWeight.SemiBold,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                )
            }
        }
    }
}

// ── Units bar ─────────────────────────────────────────────────────────────

/**
 * One chip per rocket of the foreground pair, plus every direct link.  A chip
 * is filled while its rocket is on the dashboard; a tap shows or hides it (a
 * pure view filter).  The long-press menu carries the radio action (point the
 * base station at this rocket) and link management (disconnect a direct link).
 */
@Composable
private fun UnitsBar(
    subjects: List<RocketSubject>,
    foreground: FleetDevice<DeviceSession>?,
    focus: Int?,
    hidden: Set<com.tinkerbug.tinkerrocket.session.RocketKey>,
    fleet: FleetManager<DeviceSession>,
) {
    val now = rememberNowMs()
    Row(
        Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        for (subject in subjects) {
            key(subject.id) {
                val key = subject.key
                val isHidden = key != null && key in hidden
                val follow = FleetDashboard.followTarget(subject, foreground, focus)
                RocketChip(
                    subject = subject,
                    freshness = subject.freshness(now),
                    hidden = isHidden,
                    focused = FleetDashboard.isFocused(subject, foreground, focus),
                    onToggleHidden = {
                        if (key != null) {
                            if (isHidden) fleet.unhideRocket(key) else fleet.hideRocket(key)
                        }
                    },
                    onFocusHere = follow?.let { bs -> { key?.let { bs.session.setFocusRocket(it.rocketId) } } },
                    onDisconnect = subject.direct?.let { d -> { fleet.disconnect(d.deviceId) } },
                )
            }
        }
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun RocketChip(
    subject: RocketSubject,
    freshness: RocketFreshness,
    hidden: Boolean,
    focused: Boolean,
    onToggleHidden: () -> Unit,
    onFocusHere: (() -> Unit)?,
    onDisconnect: (() -> Unit)?,
) {
    val tr = TrTheme.colors
    var menu by remember { mutableStateOf(false) }
    val fg = if (hidden) MaterialTheme.colorScheme.onSurfaceVariant else Color.White
    Box {
        Row(
            Modifier
                .clip(RoundedCornerShape(20.dp))
                .background(if (hidden) tr.cardSecondary else tr.accent)
                .combinedClickable(onClick = onToggleHidden, onLongClick = { menu = true })
                .padding(horizontal = 12.dp, vertical = 8.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Box(Modifier.size(7.dp).background(freshnessColor(freshness), CircleShape))
            Text(
                subject.name,
                color = fg,
                style = MaterialTheme.typography.labelLarge,
                maxLines = 1,
            )
            if (focused) {
                Icon(Icons.Filled.GpsFixed, "Base station follows", tint = fg, modifier = Modifier.size(12.dp))
            }
            if (subject.direct != null) {
                Icon(Icons.Filled.Smartphone, "Direct Bluetooth", tint = fg, modifier = Modifier.size(12.dp))
            }
            if (subject.relays.isNotEmpty()) {
                Icon(Icons.Filled.CellTower, "Via base station", tint = fg, modifier = Modifier.size(12.dp))
            }
        }
        DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
            if (onFocusHere != null) {
                DropdownMenuItem(
                    text = { Text("Focus base station here") },
                    leadingIcon = { Icon(Icons.Filled.GpsFixed, null) },
                    onClick = { menu = false; onFocusHere() },
                )
            }
            if (subject.key != null) {
                DropdownMenuItem(
                    text = { Text(if (hidden) "Show on dashboard" else "Hide from dashboard") },
                    onClick = { menu = false; onToggleHidden() },
                )
            }
            if (onDisconnect != null) {
                DropdownMenuItem(
                    text = { Text("Disconnect", color = tr.statusBad) },
                    onClick = { menu = false; onDisconnect() },
                )
            }
        }
    }
}

// ── Base-station strip ────────────────────────────────────────────────────

/**
 * The foreground base station: its name, network and firmware, and which
 * rocket its radio follows.  A label, not a control — Follow and the chip
 * menu move the focus.
 */
@Composable
private fun BaseStationStrip(bs: FleetDevice<DeviceSession>, focus: Int?) {
    val tr = TrTheme.colors
    val identity by bs.session.identity.collectAsState()
    val remotes by bs.session.remoteRockets.collectAsState()
    Row(
        Modifier
            .fillMaxWidth()
            .background(tr.cardSecondary, RoundedCornerShape(10.dp))
            .padding(horizontal = 12.dp, vertical = 10.dp),
        horizontalArrangement = Arrangement.spacedBy(10.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(Icons.Filled.CellTower, null, tint = tr.statusWarn)
        Column(Modifier.weight(1f)) {
            Text(
                bs.session.displayName,
                style = MaterialTheme.typography.titleSmall,
                fontWeight = FontWeight.SemiBold,
            )
            Text(
                "nid ${identity.networkId ?: "—"} · fw ${identity.firmwareVersion ?: "—"}",
                style = MaterialTheme.typography.labelSmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            if (focus != null) {
                val name = remotes.firstOrNull { it.rocketId == focus }?.unitName
                    ?.takeIf { it.isNotEmpty() } ?: "Rocket $focus"
                Row(
                    horizontalArrangement = Arrangement.spacedBy(4.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Icon(
                        Icons.Filled.GpsFixed, "Follows",
                        tint = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.size(12.dp),
                    )
                    Text(
                        name,
                        style = MaterialTheme.typography.labelSmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }
    }
}

// ── Sections ──────────────────────────────────────────────────────────────

/**
 * A section that renders a device's full dashboard: a direct rocket, or the
 * rocket the foreground base station follows (through that link's mirror).
 * The header shows when it does real work — separating sections and hosting
 * the collapse control — or when [showHeaderWhenSolo] says it must.
 */
@Composable
private fun DeviceSection(
    section: DashboardSection,
    device: FleetDevice<DeviceSession>,
    collapsible: Boolean,
    showHeaderWhenSolo: (nowMs: Long) -> Boolean,
    deviceContent: @Composable (FleetDevice<DeviceSession>, DashboardSection?) -> Unit,
    knownDevices: com.tinkerbug.tinkerrocket.session.KnownDeviceStore,
) {
    var collapsed by remember { mutableStateOf(false) }
    // A section left collapsed when the others go away must not stay shut
    // with its header (and so its chevron) gone.
    val isCollapsed = collapsed && collapsible
    val now = rememberNowMs()
    if (collapsible || showHeaderWhenSolo(now)) {
        RocketSectionHeader(
            subject = section.subject,
            nowMs = now,
            collapsible = collapsible,
            collapsed = isCollapsed,
            onToggle = { collapsed = !collapsed },
        )
    }
    // A direct link reads its board live (the device content's own line);
    // the followed relay's board is only known from its last Bluetooth link.
    if (section is DashboardSection.FocusedRelay && !isCollapsed) RegistryCalibrationLine(section.subject, knownDevices)
    if (isCollapsed) {
        val telemetry by device.session.telemetry.collectAsState()
        CollapsedRocketSummary(telemetry)
    } else {
        deviceContent(device, section)
    }
}

/**
 * Header above a rocket's cards (iOS `RocketSectionHeader`): name, transport
 * and age, the red shared-ID line, Follow on a relayed rocket the base
 * station is not following, and the collapse chevron.
 */
@Composable
private fun RocketSectionHeader(
    subject: RocketSubject,
    nowMs: Long,
    collapsible: Boolean,
    collapsed: Boolean,
    onToggle: () -> Unit,
    onFollow: (() -> Unit)? = null,
) {
    val tr = TrTheme.colors
    Row(
        Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(tr.cardSecondary)
            .clickable(enabled = collapsible, onClick = onToggle)
            .padding(horizontal = 12.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.size(8.dp).background(freshnessColor(subject.freshness(nowMs)), CircleShape))
        Column(Modifier.weight(1f)) {
            Text(subject.name, style = MaterialTheme.typography.titleMedium)
            Text(
                FleetDashboard.sourceLine(subject, nowMs),
                style = MaterialTheme.typography.labelSmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            FleetDashboard.duplicateIdWarning(subject, nowMs)?.let { warning ->
                Row(
                    horizontalArrangement = Arrangement.spacedBy(4.dp),
                    verticalAlignment = Alignment.Top,
                ) {
                    Icon(Icons.Filled.Warning, null, tint = tr.statusBad, modifier = Modifier.size(14.dp))
                    Text(
                        warning,
                        style = MaterialTheme.typography.labelSmall,
                        fontWeight = FontWeight.SemiBold,
                        color = tr.statusBad,
                    )
                }
            }
        }
        if (onFollow != null) {
            // The chip's long-press "Focus base station here" does the same;
            // this is the discoverable one.
            OutlinedButton(
                onClick = onFollow,
                contentPadding = PaddingValues(horizontal = 10.dp, vertical = 2.dp),
            ) {
                Icon(Icons.Filled.GpsFixed, null, modifier = Modifier.size(14.dp))
                Text(" Follow", style = MaterialTheme.typography.labelMedium)
            }
        }
        if (collapsible) {
            Icon(
                if (collapsed) Icons.Filled.ExpandMore else Icons.Filled.ExpandLess,
                if (collapsed) "Expand" else "Collapse",
                tint = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

/**
 * #1629: a relayed rocket's calibration can only be what its board said over
 * Bluetooth last time (the LoRa link carries none of it), so the line says so.
 * Nothing when that board was never read or lacks nothing.
 */
@Composable
private fun RegistryCalibrationLine(
    subject: RocketSubject,
    knownDevices: com.tinkerbug.tinkerrocket.session.KnownDeviceStore,
) {
    val records by knownDevices.devices.collectAsState()
    val key = subject.key ?: return
    val name = subject.freshestRelay?.remote?.unitName
    val line = com.tinkerbug.tinkerrocket.session.KnownDeviceStore
        .rocketIn(records, key.networkId, key.rocketId, name)?.calibration?.notCalibratedLine ?: return
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(6.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(Icons.Filled.Warning, null, tint = TrTheme.colors.statusWarn, modifier = Modifier.size(14.dp))
        Text(
            "$line (as of its last Bluetooth link)",
            style = MaterialTheme.typography.labelMedium,
            color = TrTheme.colors.statusWarn,
        )
    }
}

/** One line while a section is collapsed: state · altitude · rate · battery. */
@Composable
private fun CollapsedRocketSummary(telemetry: TelemetryData, modifier: Modifier = Modifier) {
    val tr = TrTheme.colors
    val units = com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem.current
    val mono = MaterialTheme.typography.labelMedium.copy(fontFamily = FontFamily.Monospace)
    Row(
        modifier
            .fillMaxWidth()
            .background(tr.cardSecondary, RoundedCornerShape(8.dp))
            .padding(horizontal = 12.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(
            stateDisplayLabel(telemetry.state),
            style = MaterialTheme.typography.labelMedium,
            fontWeight = FontWeight.Bold,
        )
        Text(telemetry.pressureAlt?.let { UnitFormatter.altitude(it.toDouble(), units) } ?: "N/A", style = mono)
        Text(telemetry.altitudeRate?.let { UnitFormatter.speed(it.toDouble(), units) } ?: "N/A", style = mono)
        Spacer(Modifier.weight(1f))
        Text(telemetry.socDisplay, style = mono)
    }
}

/**
 * A relayed rocket the base station is not following (iOS
 * `RelayRocketSectionView`): its state, the stale line, the shared card stack
 * dimmed by freshness, and Camera/Logging addressed to it alone, with the
 * desired state taken from its own telemetry.  The controls are not dimmed:
 * they still work on a stale rocket, and a faded button reads as disabled.
 */
@Composable
private fun RelayRocketSection(
    section: DashboardSection.Relay,
    collapsible: Boolean,
    followTarget: FleetDevice<DeviceSession>?,
    fleet: FleetManager<DeviceSession>,
    phoneLocation: PhoneLocationManager?,
) {
    var collapsed by remember { mutableStateOf(false) }
    val isCollapsed = collapsed && collapsible
    val now = rememberNowMs()
    val subject = section.subject
    val telemetry = section.remote.telemetry
    val freshness = subject.freshness(now)
    val opacity = freshness.cardOpacity

    RocketSectionHeader(
        subject = subject,
        nowMs = now,
        collapsible = collapsible,
        collapsed = isCollapsed,
        onToggle = { collapsed = !collapsed },
        onFollow = followTarget?.let { bs -> { bs.session.setFocusRocket(section.remote.rocketId) } },
    )
    if (isCollapsed) {
        CollapsedRocketSummary(telemetry, Modifier.alpha(opacity))
        return
    }

    RocketStateBanner(telemetry, Modifier.alpha(opacity))
    RegistryCalibrationLine(subject, fleet.knownDevices)
    FleetDashboard.staleAdvisory(freshness, now)?.let { line ->
        Text(
            line,
            style = MaterialTheme.typography.labelMedium,
            color = TrTheme.colors.statusWarn,
            modifier = Modifier.padding(horizontal = 4.dp),
        )
    }

    // The relayed frame's own status, worsened by the app's clock: a rocket
    // gone quiet keeps its last frame, which still says LIVE.
    val dataStatus = if (freshness == RocketFreshness.Live) telemetry.dataStatus
    else TelemetryData.DataStatus.STALE
    val ageMs = when (freshness) {
        RocketFreshness.Live -> 0L
        is RocketFreshness.Stale -> freshness.ageMs
        // A relayed rocket always has a last-heard time: its first frame sets it.
        is RocketFreshness.Lost -> freshness.lastSeenMs?.let { now - it } ?: 0L
    }
    val fixes by fleet.lastValidRocketFixes.collectAsState()
    val rssi by section.baseStation.session.connectedRssi.collectAsState()
    Column(Modifier.alpha(opacity), verticalArrangement = Arrangement.spacedBy(10.dp)) {
        // isBaseStation: relayed frames carry the relaying BS's own battery
        // fields, so the battery card shows the Rocket and "Base Stn" rows.
        RocketTelemetryCards(
            telemetry = telemetry,
            isBaseStation = true,
            dataStatus = dataStatus,
            bleRssi = rssi,
            phoneLocation = phoneLocation,
            // Latched per rocket by (network, rocket): a LOST rocket keeps
            // pointing at where it was last heard.
            rocketFix = subject.key?.let { fixes[it] },
        )
        RocketStatusCards(telemetry, isBaseStation = true, dataStatus = dataStatus, dataAgeMs = ageMs)
    }
    ControlsCard(section.baseStation.session, telemetry, section, showBaseStationLog = false)
}
