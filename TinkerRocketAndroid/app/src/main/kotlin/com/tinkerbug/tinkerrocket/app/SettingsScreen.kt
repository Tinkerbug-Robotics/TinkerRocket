package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.BuildConfig
import com.tinkerbug.tinkerrocket.app.theme.TrPyroContinuityBadge
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.PyroContinuity
import com.tinkerbug.tinkerrocket.session.LoraAutoApply
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.CalAdvisory
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.ConfigGroup
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SyncState
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.ProfileRollWaypoint
import com.tinkerbug.tinkerrocket.session.RocketProfile
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import com.tinkerbug.tinkerrocket.session.UnitFormatter
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch
import java.util.Locale
import com.tinkerbug.tinkerrocket.session.WireBounds

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
    session: DeviceSession? = null,
    preflight: com.tinkerbug.tinkerrocket.session.PreflightStore? = null,
    // #624: the network identity pair, so a device whose network ID has
    // drifted can be fixed HERE rather than only in Device Manager, which
    // cannot be opened while connected. Nullable so existing call sites and
    // tests compile unchanged; the section simply does not render without them.
    network: AppNetworkStore? = null,
    knownDevices: com.tinkerbug.tinkerrocket.session.KnownDeviceStore? = null,
) {
    val profiles by store.profiles.collectAsState()
    val activeId by store.activeId.collectAsState()
    val syncState by syncer.syncState.collectAsState()
    val magAdvisory by syncer.magCalAdvisory.collectAsState()
    val sensorAdvisory by syncer.sensorCalAdvisory.collectAsState()
    val suggestedId by syncer.suggestedProfileId.collectAsState()
    val createdProfileName by syncer.createdProfileName.collectAsState()
    val unreportedGroups by syncer.unreportedGroups.collectAsState()
    val active = activeId?.let { id -> profiles.firstOrNull { it.id == id } }
    // #1072: the roll sub-mode to restore when the control mode comes back
    // from Guidance (iOS's savedRollUsesAngle). Null until the operator has
    // switched at least once this visit — the profile's own value stands then.
    var savedRollUsesAngle by remember { mutableStateOf<Boolean?>(null) }

    // #361 analog: never subscribe this screen to raw telemetry — the jog
    // gate collects a distinct-until-changed Boolean, so recomposition only
    // happens when the power state actually flips, not at telemetry rate.
    val powerOn by remember(session) {
        session?.telemetry?.map { it.pwrPinOn }?.distinctUntilChanged()
            ?: kotlinx.coroutines.flow.flowOf(false)
    }.collectAsState(initial = false)
    val connected by (
        session?.isConnected ?: kotlinx.coroutines.flow.MutableStateFlow(false)
        ).collectAsState()
    val canJog = session != null && connected && powerOn

    // Inputs for the device-state gate below. Same distinct-until-changed
    // discipline as powerOn — never subscribe this screen at telemetry rate.
    val hasTelemetry by (
        session?.hasReceivedTelemetry ?: kotlinx.coroutines.flow.MutableStateFlow(false)
        ).collectAsState()
    val initializing by remember(session) {
        session?.telemetry?.map { it.state == "INITIALIZATION" }?.distinctUntilChanged()
            ?: kotlinx.coroutines.flow.flowOf(false)
    }.collectAsState(initial = false)

    // "LoRa off" is DEVICE state (rocket NVS), not a profile field, so it is
    // read from the config readback and written straight to the session.
    // A Pair, not just the flag: "no readback yet" and "a readback without the
    // key" both leave loraTxDisabled null but mean different things — only the
    // second is a firmware that cannot do this, and only that one is worth
    // saying out loud.
    val loraTx: Pair<Boolean, Boolean?> by remember(session) {
        val f: kotlinx.coroutines.flow.Flow<Pair<Boolean, Boolean?>> =
            session?.rocketConfig
                ?.map { Pair(it != null, it?.loraTxDisabled) }
                ?.distinctUntilChanged()
                ?: kotlinx.coroutines.flow.flowOf(Pair(false, null))
        f
    }.collectAsState(initial = Pair(false, null))
    val haveConfig = loraTx.first
    val loraTxDisabled = loraTx.second
    val inflight by remember(session) {
        session?.telemetry?.map { it.state == "INFLIGHT" }?.distinctUntilChanged()
            ?: kotlinx.coroutines.flow.flowOf(false)
    }.collectAsState(initial = false)

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
        // #915: a profile created for a board the app had never seen.
        createdProfileName?.let { name ->
            Card {
                Text(
                    "New rocket — created “$name” from its own settings.",
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(12.dp),
                )
            }
        }
        CalBanner("Mag cal", magAdvisory, onImport = {
            fleetScope.launch { syncer.importRocketCalIntoActiveProfile(System.currentTimeMillis()) }
        })
        // #1059: the sensor cal can be imported now, like the mag cal above.
        // Running one on the pad is still iOS-only — see the caption in
        // CalBanner, which no longer tells the operator to do something this
        // app cannot do.
        CalBanner("Sensor cal", sensorAdvisory, onImport = {
            fleetScope.launch { syncer.importRocketSensorCalIntoActiveProfile(System.currentTimeMillis()) }
        })

        // #915: the rocket keeps its own settings unless the user asks
        // otherwise, so say plainly which ones the app cannot check and put
        // the deliberate override next to that admission.
        if (connected && session?.isBaseStation == false) {
            if (unreportedGroups.isNotEmpty()) {
                Banner(
                    "Can't verify: " + unreportedGroups.joinToString(", ") +
                        ". This rocket doesn't report them, so they're shown " +
                        "from the profile.",
                    "Send all",
                ) {
                    fleetScope.launch { syncer.pushProfileToRocket() }
                }
            } else {
                Banner("Settings come from this rocket.", "Send all") {
                    fleetScope.launch { syncer.pushProfileToRocket() }
                }
            }
        }

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

        // #974: which build is this?  Deliberately ABOVE the active-profile
        // guard below — build provenance must be readable even with no profile
        // selected, which is exactly the state someone is in when they are
        // trying to work out what is installed.
        Text(
            "Build ${BuildConfig.VERSION_NAME} (${BuildConfig.TR_GIT_SHA})",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )

        if (active == null) {
            Text(
                if (profiles.isEmpty()) "No profiles yet — add one to configure a rocket."
                else "Select a profile to edit.",
                style = MaterialTheme.typography.bodyMedium,
            )
            return@Column
        }

        // ── Device-state gate ────────────────────────────────────────────
        // Everything below is FC configuration, and the FC answers none of it
        // until it is running. This fails silently rather than loudly: the OC's
        // #366 queue HOLDS commands while the rail is down and drains the batch
        // at power-on, so edits made now land much later, out of order, or not
        // at all. iOS SettingsView gates the same two states; Android had
        // neither.
        //
        // Both scopes are load-bearing:
        //  - !isBaseStation: the BS relay's TelemetryData never carries
        //    pwrPinOn (the LoRa packet has no room for it), so over a
        //    base-station link it reads false forever — an unscoped gate would
        //    lock Settings on every BS connection.
        //  - hasReceivedTelemetry (#377): before the first frame TelemetryData
        //    is all-defaults, and a defaulted pwrPinOn is indistinguishable
        //    from a genuinely-off rocket, so the notice would flash on every
        //    connect — including for a rocket that is already powered on.
        //
        // Unlike iOS, firmware update is its own screen here rather than a row
        // inside Settings, so nothing that still works with the rail off is
        // lost behind this gate.
        // #1038: over a BASE-STATION link this screen used to render the full
        // editor. Every edit persisted to the profile and called
        // syncer.pushGroup, which returns early on a base-station session
        // WITHOUT sending a frame — and the dashboard's pyro card then read the
        // trigger text back out of that same profile, so the un-sent value
        // rendered as though the rocket had confirmed it. Nothing on screen
        // said otherwise: the sync badge parks at Idle for a base station and
        // draws nothing.
        //
        // Worse than a lost edit. On the next DIRECT connect, adoptRocketConfig
        // overwrites the pyro fields from the rocket's own report, so the edit
        // vanishes silently. (pushGroup's docstring still promises the offline
        // edit "rides out on the next connect via the whole-profile push", but
        // post-#915 that push survives only an explicit mid-connection profile
        // switch, not a plain reconnect.)
        //
        // So: read-only, with iOS SettingsView.baseStationSections' wording.
        // Deliberately NOT done by widening the directLink gates below — their
        // !isBaseStation scope is load-bearing for the reason stated there.
        // Pushing config over the BS relay is a separate design question
        // (BLE_BS_CMD_RELAY_TO_ROCKET exists) and not what this fixes.
        if (session != null && connected && session.isBaseStation) {
            Section("Active Rocket") {
                Text(
                    active?.name ?: "None selected",
                    style = MaterialTheme.typography.titleSmall,
                )
                Text(
                    "Settings are stored per rocket in the app. To change them, " +
                        "connect directly to the rocket computer over Bluetooth.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            if (active != null) {
                Section("Summary") {
                    SummaryRow("Control mode", active.controlModeLabel)
                    SummaryRow("Camera", when (active.cameraType) {
                        0 -> "None"; 1 -> "GoPro"; else -> "RunCam"
                    })
                    SummaryRow("IMU mounting", if (active.imuOrientSetting == 0xFF) "Auto"
                                               else "Code ${active.imuOrientSetting}")
                    SummaryRow("Gain scheduling", if (active.gainScheduleEnabled) "On" else "Off")
                    SummaryRow("Mag cal", if (active.magCal == null) "Not saved" else "Saved")
                    SummaryRow("Sensor cal", if (active.sensorCal == null) "Not saved" else "Saved")
                }
            }

            // #624/#150: the base station's own radio. iOS shows these three
            // in this order after the summary (SettingsView.loRaSections), and
            // this branch used to stop right above here -- which is the whole
            // of what the "settings structural split" turned out to be.
            //
            // Base-station only on BOTH platforms, and not a style choice:
            // rocket-side firmware rejects cmd 17 outright, and cmd 10 is the
            // base station's transactional relay-and-verify path (#71).
            LoraSections(session)
            if (network != null && knownDevices != null) {
                NetworkSection(session, network, knownDevices)
            }

            return@Column
        }

        val directLink = session != null && connected && !session.isBaseStation
        if (directLink && hasTelemetry && !powerOn) {
            GateNotice(
                "Rocket is off",
                "The flight computer answers every setting on this screen, and it " +
                    "isn't running yet. Power on the rocket to configure it.",
            )
            return@Column
        }
        if (directLink && initializing) {
            GateNotice(
                "Initializing…",
                "Waiting for sensors to start up. Settings can be applied once ready.",
            )
            return@Column
        }

        // ── Rocket ───────────────────────────────────────────────────────
        // Control settings that used to live here (servo enable, gain
        // schedule, angle control, roll delay) moved down under the servo
        // control gate, where the switch that makes them do anything is.
        Section("Rocket") {
            ToggleRow("Sounds", active.soundsEnabled) { v -> edit(ConfigGroup.SOUNDS) { it.copy(soundsEnabled = v) } }
        }

        // ── Radio (iOS General-tab "Radio" section) ──────────────────────
        // Presented positively — the switch reads "LoRa telemetry", so ON
        // always means the radio is doing something.  The wire byte is the
        // inverse ("disabled"), inverted once, here.
        Section("Radio") {
            ToggleRow(
                "LoRa telemetry",
                loraTxDisabled != true,
                enabled = session != null && connected &&
                    loraTxDisabled != null && !inflight,
            ) { on -> session?.sendLoraTxDisabled(!on) }
            Caption(
                when {
                    inflight ->
                        "Locked while the rocket is flying — muting it now would drop " +
                            "the only link telling you where it is."
                    connected && haveConfig && loraTxDisabled == null ->
                        "This rocket's firmware doesn't support turning the radio off. " +
                            "Update it to use this."
                    loraTxDisabled == true ->
                        "OFF — the rocket transmits nothing: no telemetry, no beacon. It " +
                            "still listens, so the base station can turn it back on. You " +
                            "will have no tracking during flight."
                    else ->
                        "ON — the rocket downlinks telemetry to the base station at 2 Hz. " +
                            "Turn it off to fly silent (bench work, a busy band, or a site " +
                            "where you would rather not transmit)."
                },
            )
        }

        // #624: iOS carries the Network section on the rocket General tab as
        // well as in the base-station branch, and for the same reason on both:
        // a drifted network ID is noticed while connected and must be fixable
        // there. Android's rocket branch is one scroll rather than tabs, so it
        // sits where the General tab's content ends.
        if (session != null && network != null && knownDevices != null) {
            NetworkSection(session, network, knownDevices)
        }

        // ── IMU Mounting (iOS General-tab section; raw-int field replaced
        //    by the iOS pickers on the 2026-08-09 design pass) ─────────────
        Section("IMU Mounting") {
            val auto = active.imuOrientSetting == 0xFF
            // iOS: switching to auto stashes the manual code and restores it
            // on the way back.
            var savedOrientCode by remember { mutableStateOf<Int?>(null) }
            SegmentedPicker(listOf("Manual", "Pad auto-detect"), if (auto) 1 else 0) { i ->
                if (i == 1 && !auto) {
                    savedOrientCode = active.imuOrientSetting
                    edit(ConfigGroup.IMU_ORIENT) { it.copy(imuOrientSetting = 0xFF) }
                } else if (i == 0 && auto) {
                    val restore = savedOrientCode ?: 0
                    edit(ConfigGroup.IMU_ORIENT) { it.copy(imuOrientSetting = restore) }
                }
            }
            if (!auto) {
                val code = active.imuOrientSetting.coerceIn(0, 23)
                FieldRow {
                    DropdownField(
                        "Nose axis", listOf("+X", "-X", "+Y", "-Y", "+Z", "-Z"), code / 4,
                        { a -> edit(ConfigGroup.IMU_ORIENT) { it.copy(imuOrientSetting = a * 4 + code % 4) } },
                        Modifier.weight(1f),
                    )
                    DropdownField(
                        "Fin clocking", listOf("0°", "90°", "180°", "270°"), code % 4,
                        { c -> edit(ConfigGroup.IMU_ORIENT) { it.copy(imuOrientSetting = (code / 4) * 4 + c) } },
                        Modifier.weight(1f),
                    )
                }
                Caption(
                    "Which board axis points at the nose (up on the pad). Manual also fixes " +
                        "the fin clocking (quarter-turns about the nose) — required for " +
                        "roll-controlled or guided flight.",
                )
            } else {
                Caption(
                    "Detects the nose axis from gravity on the pad. It can’t observe fin " +
                        "clocking, so this is fine only for non-controlled flights — not " +
                        "roll or guidance.",
                )
            }
            session?.let { ActiveOrientRow(it) }
        }

        // ── IMU Logging Rate (iOS General-tab section) ───────────────────
        Section("IMU Logging Rate") {
            val rates = listOf(0, 960, 1920, 3840)   // Dynamic + ISM6HG256 ODR steps
            val rateIdx = rates.indexOf(active.imuRateHz)
            SegmentedPicker(listOf("Dynamic", "1k", "2k", "4k"), rateIdx) { i ->
                edit(ConfigGroup.IMU_RATE) { it.copy(imuRateHz = rates[i]) }
            }
            Caption(
                if (rateIdx == 0) {
                    "Logs at 4k (3840 Hz) from the pad through boost and coast, then drops " +
                        "to 1k (960 Hz) once the rocket detects its recovery deployment — " +
                        "full shock and vibration detail where it matters, without filling " +
                        "the log under canopy."
                } else {
                    "Samples logged per second from the IMU (actual: 960 / 1920 / 3840 Hz). " +
                        "Higher rates capture faster shock and vibration detail; the control " +
                        "loop is unaffected. Applies on the pad — never mid-flight."
                },
            )
        }

        // ── Camera (iOS Camera tab; raw-int field replaced by the picker) ─
        Section("Camera") {
            SegmentedPicker(listOf("None", "GoPro", "RunCam"), active.cameraType.coerceIn(0, 2)) { i ->
                edit(ConfigGroup.CAMERA) { it.copy(cameraType = i) }
            }
            Caption(
                when (active.cameraType) {
                    1 -> "GoPro: controlled via GPIO pulse on shutter pin."
                    2 -> "RunCam: controlled via UART serial command."
                    else -> "No camera connected."
                },
            )
        }

        // ── Servo control ────────────────────────────────────────────────
        // The master switch for every control setting below it, so it leads
        // the block instead of sitting mid-way down a mixed "Rocket" section
        // (iOS Control-tab twin).  Nothing under it reaches the fins except
        // through the servo driver — the bench jog included, which cannot move
        // a servo the FC never writes to.  Dimmed and locked rather than
        // removed: the values stay readable for a preflight check, and a block
        // that vanishes reads as broken rather than as switched off.
        val servoOn = active.servoControlEnabled
        // Close the keyboard when the gate shuts: the scrim swallows taps, not
        // keystrokes, so a field focused before the switch was turned off would
        // still take typing through it.  Blurring also commits that pending
        // edit (NumField commits on blur) instead of stranding it.
        LaunchedEffect(servoOn) { if (!servoOn) focusManager.clearFocus() }
        Section("Servo control") {
            ToggleRow("Servo control", servoOn) { v ->
                edit(ConfigGroup.SERVO_ENABLE) { it.copy(servoControlEnabled = v) }
            }
            Caption(
                if (servoOn) {
                    "The flight computer drives the fin servos. Everything below — gains, " +
                        "roll control, servo calibration, fin layout, guidance — applies."
                } else {
                    "Off — the fins never move: no roll control, no guidance, no bench jog. " +
                        "The settings below stay saved, and do nothing until this is on."
                },
            )
        }

        GatedControls(enabled = servoOn) {
            // ── PID ──────────────────────────────────────────────────────
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
                ToggleRow("Gain schedule", active.gainScheduleEnabled) { v ->
                    edit(ConfigGroup.GAIN_SCHEDULE) { it.copy(gainScheduleEnabled = v) }
                }
                Caption(
                    "Gain schedule scales the gains by (V_ref/V)² so fin authority stays " +
                        "roughly constant as speed changes. Off = fixed gains at all speeds.",
                )
            }

            // ── Roll control (iOS Control-tab "Roll Control" section) ────
            Section("Roll control") {
                // #1072: iOS makes this a top-level CONTROL MODE — Roll Control
                // or Guidance — because the two are exclusive in the firmware:
                // guidance rate-nulls roll in every phase, so a profile with
                // both on describes a flight the rocket will not fly. Android
                // had two independent toggles, so the operator could ask for
                // both and get neither explanation. Switching to Guidance
                // stashes the roll sub-mode and restores it on the way back,
                // exactly as controlModeBinding does.
                SegmentedPicker(
                    listOf("Roll control", "Guidance"),
                    if (active.guidanceEnabled) 1 else 0,
                ) { i ->
                    if (i == 1) {
                        savedRollUsesAngle = active.useAngleControl
                        edit(ConfigGroup.GUIDANCE) {
                            it.copy(guidanceEnabled = true, useAngleControl = false)
                        }
                        edit(ConfigGroup.ROLL_CONTROL) { it }
                    } else {
                        val restore = savedRollUsesAngle
                        edit(ConfigGroup.GUIDANCE) {
                            it.copy(
                                guidanceEnabled = false,
                                useAngleControl = restore ?: it.useAngleControl,
                            )
                        }
                        edit(ConfigGroup.ROLL_CONTROL) { it }
                    }
                }
                Caption(
                    if (active.guidanceEnabled) {
                        "Guidance steers the airframe; roll is rate-nulled (held at zero " +
                            "spin) in every phase, so the roll profile below is not followed."
                    } else {
                        "Roll control only — the fins hold or track roll, and no guidance " +
                            "law runs."
                    },
                )
                ToggleRow(
                    "Angle control",
                    active.useAngleControl,
                    enabled = !active.guidanceEnabled,
                ) { v ->
                    edit(ConfigGroup.ROLL_CONTROL) { it.copy(useAngleControl = v) }
                }
                Caption(
                    if (active.useAngleControl) {
                        "Cascaded angle control — fins track the roll profile waypoints below."
                    } else {
                        "Rate-only control — fins hold zero roll rate. No profile followed."
                    },
                )
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
                    NumField(
                        "Roll delay ms", active.rollDelayMs.toString(), Modifier.weight(1f),
                        range = WireBounds.U16,
                    ) { s ->
                        s.toIntOrNull()?.let { v -> edit(ConfigGroup.ROLL_CONTROL) { it.copy(rollDelayMs = v) } }
                    }
                    NumField("Min speed m/s", fmt(active.rollMinSpeedMps), Modifier.weight(1f)) { s ->
                        // Clamped to the firmware's accepted range: the rocket
                        // rejects anything above it, and a gate that never opens
                        // means no roll control for the whole flight.
                        s.toFloatOrNull()?.let { v ->
                            edit(ConfigGroup.ROLL_CONTROL) {
                                it.copy(rollMinSpeedMps = v.coerceIn(0f, 300f))
                            }
                        }
                    }
                }
                Caption(
                    "Milliseconds after launch before control activates — roll-rate-null and " +
                        "PN guidance both engage at this delay, keeping fins neutral through " +
                        "initial boost.",
                )
                Caption(
                    "Min speed is the airspeed the rocket must reach before control activates. " +
                        "Fin authority scales with speed squared, so a loop that starts on the " +
                        "rail commands full deflection the fins cannot deliver and departs when " +
                        "authority arrives. Applied on top of the roll delay: both must be " +
                        "satisfied. 0 disables the speed gate.",
                )
            }

            // ── Servo ────────────────────────────────────────────────────
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
                    NumField(
                        "Hz", active.servoHz.toString(), Modifier.weight(1f),
                        range = WireBounds.I16,
                    ) { s ->
                        s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoHz = v) } }
                    }
                    NumField(
                        "Min µs", active.servoMinUs.toString(), Modifier.weight(1f),
                        range = WireBounds.I16,
                    ) { s ->
                        s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoMinUs = v) } }
                    }
                    NumField(
                        "Max µs", active.servoMaxUs.toString(), Modifier.weight(1f),
                        range = WireBounds.I16,
                    ) { s ->
                        s.toIntOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(servoMaxUs = v) } }
                    }
                    // #449: travel is the editable quantity; fin angles derive.
                    NumField("Travel °", fmt(active.finTravelDeg), Modifier.weight(1f)) { s ->
                        s.toFloatOrNull()?.let { v -> edit(ConfigGroup.SERVO) { it.copy(finTravelDeg = v) } }
                    }
                }
            }

            // ── Fin layout (cmd 66) ──────────────────────────────────────
            Section("Fin layout") {
                FinLayoutEditor(
                    ringMode = active.finRingMode,
                    servoAtSlot = active.finServoAtSlot,
                    reverse = active.finReverse,
                    rollReverse = active.finRollReverse,
                    canJog = canJog,
                    finMinDeg = active.finMinDeg,
                    finMaxDeg = active.finMaxDeg,
                    onSetRingMode = { v -> edit(ConfigGroup.FIN_LAYOUT) { it.copy(finRingMode = v) } },
                    onSetServoAtSlot = { v -> edit(ConfigGroup.FIN_LAYOUT) { it.copy(finServoAtSlot = v) } },
                    onSetReverse = { v -> edit(ConfigGroup.FIN_LAYOUT) { it.copy(finReverse = v) } },
                    onSetRollReverse = { v -> edit(ConfigGroup.FIN_LAYOUT) { it.copy(finRollReverse = v) } },
                    onJogAngles = { angles -> session?.sendCommandFrame(Commands.servoTestAngles(angles)) },
                    onJogStop = { session?.sendBareCommand(BleCommandId.SERVO_TEST_STOP) },
                )
            }

            // ── Roll profile (cmd 26; Clear = cmd 27, matching iOS) ──────
            Section("Roll profile") {
                Text(
                    "The target roll angle ramps linearly between waypoints. Before " +
                        "the first waypoint the controller nulls roll rate (fins keep " +
                        "zero roll through boost); after the last waypoint the final " +
                        "angle is held. To hold an angle, give two consecutive " +
                        "waypoints the same angle.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                active.rollWaypoints.forEachIndexed { i, wp ->
                    Row(
                        Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text("WP ${i + 1}", style = MaterialTheme.typography.bodySmall)
                        NumField("Time s", fmt(wp.timeSeconds), Modifier.weight(1f)) { s ->
                            s.toFloatOrNull()?.let { v ->
                                edit(ConfigGroup.ROLL_PROFILE) { p ->
                                    p.copy(
                                        rollWaypoints = p.rollWaypoints.mapIndexed { j, w ->
                                            if (j == i) w.copy(timeSeconds = v) else w
                                        },
                                    )
                                }
                            }
                        }
                        NumField("Angle °", fmt(wp.angleDeg), Modifier.weight(1f)) { s ->
                            s.toFloatOrNull()?.let { v ->
                                edit(ConfigGroup.ROLL_PROFILE) { p ->
                                    p.copy(
                                        rollWaypoints = p.rollWaypoints.mapIndexed { j, w ->
                                            if (j == i) w.copy(angleDeg = v) else w
                                        },
                                    )
                                }
                            }
                        }
                        TextButton(onClick = {
                            edit(ConfigGroup.ROLL_PROFILE) { p ->
                                p.copy(rollWaypoints = p.rollWaypoints.filterIndexed { j, _ -> j != i })
                            }
                        }) { Text("✕") }
                    }
                }
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    if (active.rollWaypoints.size < 8) {
                        OutlinedButton(onClick = {
                            val t = (active.rollWaypoints.lastOrNull()?.timeSeconds ?: -1f) + 1f
                            edit(ConfigGroup.ROLL_PROFILE) { p ->
                                p.copy(
                                    rollWaypoints = p.rollWaypoints +
                                        ProfileRollWaypoint(timeSeconds = t, angleDeg = 0f),
                                )
                            }
                        }) { Text("Add Waypoint") }
                    }
                    if (active.rollWaypoints.isNotEmpty()) {
                        OutlinedButton(onClick = {
                            // iOS: empty the profile + explicit clear command (27).
                            edit(null) { it.copy(rollWaypoints = emptyList()) }
                            session?.sendBareCommand(BleCommandId.ROLL_PROFILE_CLEAR)
                        }) { Text("Clear Roll Profile") }
                    }
                }
            }

            // ── Guidance ─────────────────────────────────────────────────
            // #1072: the law selector and its per-law tuning were missing
            // entirely, so station-keeping was reachable on Android only by
            // adopting a rocket already configured that way — and once
            // adopted, its two gains could not be edited. The syncer has
            // always SENT every field from the profile; only the controls
            // were absent.
            Section("Guidance") {
                if (active.guidanceEnabled) {
                    SegmentedPicker(
                        listOf("Proportional Nav", "Station-Keep"),
                        active.pnGuidanceLaw.coerceIn(0, 1),
                    ) { i ->
                        edit(ConfigGroup.GUIDANCE) { it.copy(pnGuidanceLaw = i) }
                    }
                    Caption(
                        "Station-keep is a PD regulator that flies straight up over the aim " +
                            "point — no closest-approach singularity. PN steers toward a " +
                            "target altitude and is the legacy law.",
                    )
                    Caption(
                        "Aim point is currently the pad (overhead); wind-compensated aim " +
                            "points arrive with Drift Cast.",
                    )
                    // Per-law tuning. The other law's values stay in the
                    // profile and keep going out on the wire, so switching
                    // laws never zeroes its tune (iOS guidanceLawFields).
                    if (active.pnGuidanceLaw == 0) {
                        FieldRow {
                            NumField("Target alt m", fmt(active.pnTargetAltM), Modifier.weight(1f)) { s ->
                                s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnTargetAltM = v) } }
                            }
                            NumField("Nav gain (N)", fmt(active.pnNavGain), Modifier.weight(1f)) { s ->
                                s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnNavGain = v) } }
                            }
                        }
                        Caption(
                            "Aim-point altitude above the pad. Set near (or modestly above) " +
                                "expected apogee — aiming far above weakens guidance.",
                        )
                    } else {
                        FieldRow {
                            NumField("Pos gain (kp) 1/s²", fmt(active.pnKpPos), Modifier.weight(1f)) { s ->
                                s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnKpPos = v) } }
                            }
                            NumField("Vel gain (kd) 1/s", fmt(active.pnKdVel), Modifier.weight(1f)) { s ->
                                s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnKdVel = v) } }
                            }
                        }
                    }
                    FieldRow {
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
                        NumField("Accel→fin °", fmt(active.pnAccelToFin), Modifier.weight(1f)) { s ->
                            s.toFloatOrNull()?.let { v -> edit(ConfigGroup.GUIDANCE) { it.copy(pnAccelToFin = v) } }
                        }
                    }
                    Caption(
                        "Engages at the Activation Delay after launch (shared with roll " +
                            "control), not after burnout.",
                    )
                } else {
                    Caption("Switch the control mode above to Guidance to configure it.")
                }
            }
        }

        // ── Pyro (config only — ARMING stays an explicit dashboard action).
        //    iOS form: one section per channel, trigger picker + unit-aware
        //    value + continuity test controls.  Every edit does the iOS
        //    triple-write: profile → syncer cmd-34 push → rocketConfig
        //    mirror, so the dashboard tiles update without a readback. ─────
        PyroChannelSection(
            1, active.pyro1Enabled, active.pyro1TriggerMode, active.pyro1TriggerValue, session,
            onEnabled = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro1Enabled = v) }
            },
            onMode = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro1TriggerMode = v) }
            },
            onValue = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro1TriggerValue = v) }
            },
        )
        PyroChannelSection(
            2, active.pyro2Enabled, active.pyro2TriggerMode, active.pyro2TriggerValue, session,
            onEnabled = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro2Enabled = v) }
            },
            onMode = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro2TriggerMode = v) }
            },
            onValue = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro2TriggerValue = v) }
            },
        )
        PyroChannelSection(
            3, active.pyro3Enabled, active.pyro3TriggerMode, active.pyro3TriggerValue, session,
            onEnabled = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro3Enabled = v) }
            },
            onMode = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro3TriggerMode = v) }
            },
            onValue = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro3TriggerValue = v) }
            },
        )
        PyroChannelSection(
            4, active.pyro4Enabled, active.pyro4TriggerMode, active.pyro4TriggerValue, session,
            onEnabled = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro4Enabled = v) }
            },
            onMode = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro4TriggerMode = v) }
            },
            onValue = { v ->
                edit(ConfigGroup.PYRO) { it.copy(pyro4TriggerValue = v) }
            },
        )
        Caption(
            "Single shared arm FET arms momentarily for each fire pulse. Test continuity " +
                "before flight; test-fire (Ground Test) is iOS-only for now.",
        )

        // ── Recovery — app-side landing prediction, nothing on the wire ──
        Section("Recovery") {
            // iOS applyRecoveryConfig validation: rates and the deploy
            // altitude must be > 0 (a rate ≤ 0.1 fps degrades simulateDescent
            // to "lands where it is"); drag k accepts 0 (gravity-only) but
            // never a negative, which would turn the drag term into thrust
            // and blow the ballistic prediction up.
            FieldRow {
                NumField("Drogue ft/s", fmtD(active.drogueRateFps), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.takeIf { it > 0 }
                        ?.let { v -> edit(null) { it.copy(drogueRateFps = v) } }
                }
                NumField("Main ft/s", fmtD(active.mainRateFps), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.takeIf { it > 0 }
                        ?.let { v -> edit(null) { it.copy(mainRateFps = v) } }
                }
                NumField("Main alt ft", fmtD(active.mainDeployAltAglFt), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.takeIf { it > 0 }
                        ?.let { v -> edit(null) { it.copy(mainDeployAltAglFt = v) } }
                }
            }
            Caption(
                "Descent profile for the live landing prediction and Drift Cast: drogue " +
                    "rate above the main-deploy altitude (AGL), main rate below it. Stored " +
                    "per rocket in the app — not sent to the flight computer.",
            )
            // iOS Drag k row — was missing on Android (accidental omission,
            // the other three recovery fields were ported together).
            FieldRow {
                NumField("Drag k 1/m", fmtD(active.ballisticDragK), Modifier.weight(1f)) { s ->
                    s.toDoubleOrNull()?.takeIf { it >= 0 }
                        ?.let { v -> edit(null) { it.copy(ballisticDragK = v) } }
                }
            }
            Caption(
                "Quadratic drag coefficient for the coast-to-apogee ballistic prediction " +
                    "(a = −k·|v|·v). ≈0.0005 for typical 54–65 mm airframes; 0 = gravity-only.",
            )
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
                    fleetScope.launch {
                        store.delete(active.id)
                        // The pre-flight checklist diff is keyed by profile
                        // id and dies with it (iOS RocketProfileView twin).
                        preflight?.deleteConfig(active.id)
                    }
                }) { Text("Delete") }
            },
            dismissButton = { TextButton(onClick = { confirmDelete = false }) { Text("Cancel") } },
        )
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

@Composable
private fun SyncBadge(state: SyncState) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val (label, color) = when (state) {
        SyncState.Idle -> return
        SyncState.AwaitingSync -> "reading rocket" to tr.statusWarn
        SyncState.NoProfile -> "no profile" to tr.statusIdle
        SyncState.Syncing -> "sending…" to tr.statusIdle
        SyncState.Synced -> "matches rocket" to tr.statusOk
        is SyncState.Adopted -> "updated from rocket" to tr.statusIdle
        is SyncState.Failed -> "sync failed" to tr.statusBad
    }
    Text(
        label,
        style = MaterialTheme.typography.labelMedium,
        color = color,
        modifier = Modifier
            .background(color.copy(alpha = 0.15f), RoundedCornerShape(12.dp))
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
                        // #1059: no longer "run calibration to save one" — that
                        // is an instruction this app cannot carry out, and with
                        // Import wired for both cals this branch is now only
                        // reached with no connected session to import from.
                        "$label: the rocket has a calibration this profile doesn't. " +
                            "Connect to the rocket to import it.",
                        style = MaterialTheme.typography.bodyMedium,
                        modifier = Modifier.padding(12.dp),
                    )
                }
            }
    }
}

/** Full-width notice shown in place of the config sections when the rocket
 *  can't accept settings yet (rail off / initializing). */
@Composable
private fun GateNotice(title: String, body: String) {
    Card(Modifier.fillMaxWidth()) {
        Column(
            Modifier.fillMaxWidth().padding(20.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            Text(
                body,
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center,
            )
        }
    }
}

/** #1038: read-only label/value row for the base-station Summary section. */
@Composable
private fun SummaryRow(label: String, value: String) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
    ) {
        Text(label, style = MaterialTheme.typography.bodyMedium)
        Text(
            value,
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

/**
 * #624/#150: the base station's radio — link mode, the frequency it settled
 * on, and TX power. iOS twin: `SettingsView.loRaSections`.
 *
 * Everything here is DEVICE state (base-station NVS), never a profile field,
 * so it is read from the config readback and written straight to the session.
 *
 * Both writes are refused rather than silently dropped when the link cannot
 * carry them, and the reason is printed under the control instead of greying
 * it in silence: a dead control with no explanation is what makes an operator
 * think the app is broken, and here the fix is usually "switch the rocket on".
 */
@Composable
internal fun LoraSections(session: DeviceSession) {
    val config by session.rocketConfig.collectAsState()
    val connected by session.isConnected.collectAsState()
    val rockets by session.remoteRockets.collectAsState()

    // Recomputed whenever any input to the decision changes. The beacon age is
    // a clock read, so this is also refreshed by the readback and roster
    // updates that arrive at ~1 Hz on a live link -- fresh enough that the
    // footer does not sit on a stale reason.
    val refusal = remember(config, connected, rockets) { session.loraApplyRefusal() }
    val hoppingAvailable = LoraAutoApply.hoppingAvailable(config)

    // Local echo for the picker. The readback is the truth, but it lands a
    // beat after the write, so hydrating from it unconditionally would snap
    // the control back under the user's finger. iOS holds hydration for 3 s
    // after a tap for exactly this; the same trick, expressed as "ignore the
    // readback until it agrees with what we asked for, or the hold expires".
    var hopping by remember { mutableStateOf(config?.loraHopDisabled == false) }
    var touchedAtMs by remember { mutableStateOf(0L) }
    LaunchedEffect(config?.loraHopDisabled) {
        val reported = config?.loraHopDisabled ?: return@LaunchedEffect
        if (System.currentTimeMillis() - touchedAtMs > LINK_MODE_HOLD_MS) hopping = !reported
    }

    Section("Link Mode") {
        SegmentedPicker(
            options = listOf("Fixed Channel", "Frequency Hopping"),
            selected = if (hopping) 1 else 0,
        ) { idx ->
            val wantHopping = idx == 1
            // Refuse before echoing: an optimistic flip that the session then
            // declines to send would leave the picker asserting a mode the
            // link is not in.
            if (session.sendLoraHopDisabled(!wantHopping)) {
                touchedAtMs = System.currentTimeMillis()
                hopping = wantHopping
            }
        }
        Caption(
            when {
                refusal != null -> refusal.message
                !hoppingAvailable && !hopping ->
                    "Hopping needs a shorter airtime than this modulation allows, so the " +
                        "firmware would refuse it. Widen the bandwidth or lower the " +
                        "spreading factor first."
                hopping ->
                    "The rocket owns the hop sequence and the base station follows it. " +
                        "Noisy channels are skipped via the scan mask."
                else ->
                    "Both ends stay on one channel for the whole flight. This is the default."
            }
        )
    }

    Section("LoRa Frequency") {
        // Read-only, as on iOS. The write path is a transaction across both
        // ends and belongs with the scan that chooses the channel, not with a
        // free-text field -- see the Frequency Scan tool.
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text("Current")
            Text(
                config?.loraFreqMHz?.let { String.format("%.2f MHz", it) } ?: "—",
                fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Caption(
            "Both devices meet on the factory channel at boot. Both should report " +
                "the same frequency.",
        )
    }

    Section("LoRa TX Power") {
        val reported = config?.loraTxPower
        var power by remember(reported) { mutableStateOf(reported ?: 17) }
        // Locked while hopping: cmd 10 carries the modulation, and the
        // firmware refuses to retune mid-hop.
        val locked = refusal != null || hopping
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Power")
            Row(verticalAlignment = Alignment.CenterVertically) {
                TextButton(
                    onClick = { if (power > TX_POWER_MIN_DBM) power-- },
                    enabled = !locked && power > TX_POWER_MIN_DBM,
                ) { Text("−") }
                Text(
                    "$power dBm",
                    fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                TextButton(
                    onClick = { if (power < TX_POWER_MAX_DBM) power++ },
                    enabled = !locked && power < TX_POWER_MAX_DBM,
                ) { Text("+") }
            }
        }
        // Applied on an explicit tap rather than on every step. iOS debounces
        // a Stepper by 0.5 s; a debounce here would mean a coroutine per tap
        // racing the readback, and an operator changing transmit power on a
        // live link is better served by seeing the value first and committing
        // it deliberately.
        TextButton(
            onClick = { session.autoApplyTxPower(power) },
            enabled = !locked && power != reported,
        ) { Text(if (power == reported) "Applied" else "Apply $power dBm") }
        Caption(
            when {
                refusal != null -> refusal.message
                hopping -> "Transmit power is fixed while hopping."
                else ->
                    "The base station relays the change to the rocket, switches, and " +
                        "rolls back if the rocket does not answer on the new power."
            }
        )
    }
}

/**
 * #624: the connected device's network ID against the app's, and a one-tap fix
 * when they disagree. iOS twin: `SettingsView.networkSection`.
 *
 * Android detected nid drift only in Device Manager, which cannot be opened
 * while connected — so the one moment the operator is most likely to notice
 * ("why can't the base station hear my rocket?") was the one moment they could
 * do nothing about it without disconnecting first. Settled 2026-09-12.
 *
 * The write goes through [KnownDeviceStore], not straight down the wire, for
 * the same reason iOS routes it there: the registry is the single writer for
 * identity edits, so the My Devices record updates now rather than waiting on
 * the readback echo.
 */
@Composable
internal fun NetworkSection(
    session: DeviceSession,
    network: AppNetworkStore,
    knownDevices: com.tinkerbug.tinkerrocket.session.KnownDeviceStore,
) {
    val appName by network.name.collectAsState()
    val appId by network.id.collectAsState()
    val identity by session.identity.collectAsState()
    val connected by session.isConnected.collectAsState()

    // 0 is the unset sentinel, and an unprovisioned device is not a mismatch —
    // it is a device nobody has told yet, which the provisioning dialog owns.
    // Flagging it here would put a scary warning on every first connect.
    val deviceId = identity.networkId
    val mismatch = appId > 0 && deviceId > 0 && deviceId != appId

    Section("Network") {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(appName.ifEmpty { "Not set" })
            Text(
                "ID: ${if (appId > 0) "$appId" else "—"}",
                fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text("This device", color = MaterialTheme.colorScheme.onSurfaceVariant)
            Text(
                if (deviceId > 0) "ID: $deviceId" else "ID: —",
                fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                color = if (mismatch) com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors.statusWarn
                else MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        if (mismatch) {
            Text(
                NetworkCopy.DEVICE_MISMATCH_WARNING,
                style = MaterialTheme.typography.bodySmall,
                color = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors.statusWarn,
            )
            TextButton(
                onClick = {
                    knownDevices.setNetworkId(appId, identity.unitId, session)
                },
                enabled = connected && identity.unitId.isNotEmpty(),
            ) { Text("Set device to \"$appName\" (ID $appId)") }
        } else {
            Caption(NetworkCopy.SAME_NETWORK_EXPLAINER)
        }
    }
}

/** #150: how long a link-mode tap outranks the config readback. iOS: 3.0 s. */
private const val LINK_MODE_HOLD_MS = 3_000L

/** Firmware's accepted TX-power range, matching the iOS stepper bounds. */
private const val TX_POWER_MIN_DBM = -9
private const val TX_POWER_MAX_DBM = 22

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            content()
        }
    }
}

/**
 * Dim-and-lock wrapper for the control block under the servo-control switch.
 *
 * A scrim rather than an `enabled` flag threaded through every field: the
 * block holds NumFields, FilterChips, dropdowns and the fin-layout jog canvas,
 * none of which take one, and all of which must go inert together.  The scrim
 * detects taps only — it never consumes the drag, so the settings list still
 * scrolls through the grayed region.  [content] stays composed, so no
 * NumField is disposed (which would fire its commit-on-dispose) just because
 * servo control was switched off mid-edit.
 */
@Composable
private fun GatedControls(enabled: Boolean, content: @Composable ColumnScope.() -> Unit) {
    Box {
        Column(
            Modifier.fillMaxWidth().alpha(if (enabled) 1f else 0.5f),
            verticalArrangement = Arrangement.spacedBy(10.dp),   // matches the screen Column
            content = content,
        )
        if (!enabled) {
            Box(
                Modifier
                    .matchParentSize()
                    .pointerInput(Unit) { detectTapGestures { } },
            )
        }
    }
}

@Composable
private fun ToggleRow(
    label: String,
    value: Boolean,
    enabled: Boolean = true,
    onChange: (Boolean) -> Unit,
) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, style = MaterialTheme.typography.bodyLarge)
        Switch(checked = value, onCheckedChange = onChange, enabled = enabled)
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
    /**
     * #1054: the wire width this field is encoded at, when it has one. An
     * entry outside it is CLAMPED on commit, so the value stored (and pushed)
     * is the value the field then shows — before this the screen kept 40000
     * for a Servo Hz whose frame carried -25536. Null for the fields that are
     * floats on the wire and have no such edge.
     */
    range: IntRange? = null,
    onCommit: (String) -> Unit,
) {
    var text by remember(value) { mutableStateOf(value) }
    var focused by remember { mutableStateOf(false) }
    val focusManager = LocalFocusManager.current
    val commit: (String) -> Unit = { entered ->
        val clamped = range?.let { r -> entered.toIntOrNull()?.coerceIn(r)?.toString() } ?: entered
        onCommit(clamped)
    }
    val pending by androidx.compose.runtime.rememberUpdatedState(Triple(text, value, commit))
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
            if (focused && !st.isFocused && text != value) commit(text)
            focused = st.isFocused
        },
    )
}

/**
 * iOS Pyro-tab channel section: Enabled toggle; when enabled, the trigger
 * picker + unit-aware value field (canonical storage: seconds for mode 0,
 * METERS for mode 1 — entered/shown in the display unit, #160) + the iOS
 * explainer caption; then the continuity test controls.
 */
@Composable
private fun PyroChannelSection(
    channel: Int,
    enabled: Boolean,
    mode: Int,
    value: Float,
    session: DeviceSession?,
    onEnabled: (Boolean) -> Unit,
    onMode: (Int) -> Unit,
    onValue: (Float) -> Unit,
) {
    Section("Pyro Channel $channel") {
        ToggleRow("Enabled", enabled, onChange = onEnabled)
        if (enabled) {
            val isTime = mode == 0
            SegmentedPicker(
                listOf("Time after apogee", "Altitude on descent"),
                if (isTime) 0 else 1,
            ) { onMode(it) }
            val units = com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem.current
            val display =
                if (isTime) String.format(Locale.ROOT, "%.1f", value)
                else String.format(
                    Locale.ROOT, "%.0f",
                    UnitFormatter.altitudeValue(value.toDouble(), units),
                )
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                NumField(if (isTime) "Delay" else "Altitude", display, Modifier.weight(1f)) { s ->
                    s.toFloatOrNull()?.let { v ->
                        onValue(
                            if (isTime) v
                            else UnitFormatter.altitudeToMeters(v.toDouble(), units).toFloat(),
                        )
                    }
                }
                Text(
                    if (isTime) "s after apogee"
                    else "${UnitFormatter.altitudeUnit(units)} on descent",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            Caption(
                if (isTime) "Fires this many seconds after apogee is detected."
                else "Fires when the rocket descends through this altitude (AGL).",
            )
        }
        session?.let { PyroTestControls(it, channel) }
    }
}

/**
 * iOS PyroChannelTestControls twin — a CHILD composable so telemetry-rate
 * recomposition stays contained (the #361 keyboard fix; the Settings screen
 * itself reads no telemetry).  Direct connected rocket links only.  The
 * "Test Pyro Channel" test-fire button is NOT ported — the Ground Test
 * screen is its own safety-sensitive pass (ledger).
 */
@Composable
private fun PyroTestControls(session: DeviceSession, channel: Int) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    if (session.isBaseStation) return
    val connected by session.isConnected.collectAsState()
    if (!connected) return

    val armed by remember(session) {
        session.telemetry.map { it.pyroArmed }.distinctUntilChanged()
    }.collectAsState(initial = false)
    val fired by remember(session, channel) {
        session.telemetry.map { it.pyroFired(channel) }.distinctUntilChanged()
    }.collectAsState(initial = false)
    // #828: was `telemetry.pyroCont(channel)` ANDed with a LIVE check and
    // drawn green/red. That collapse cannot express "never measured this
    // session", so an untested channel rendered as a confident MEASURED open
    // — read by an operator as a dead igniter or a spent charge, on a channel
    // that is in fact live. The flow already folds in connection and the #297
    // live gate, and emits only when the verdict changes.
    val continuity by remember(session, channel) {
        session.pyroContinuityFlow(channel)
    }.collectAsState(initial = PyroContinuity.NO_DATA)
    val inflight by remember(session) {
        session.telemetry.map { it.state == "INFLIGHT" }.distinctUntilChanged()
    }.collectAsState(initial = false)
    val contPendingUntil by session.contTestPendingUntil.collectAsState()

    // iOS contTestActive: a 5 s reveal window after a manual test, so the
    // readout is visible even while unarmed.
    var contTestActive by remember { mutableStateOf(false) }
    LaunchedEffect(contTestActive) {
        if (contTestActive) {
            delay(5_000)
            contTestActive = false
        }
    }
    // TESTING ends on a clock edge — tick until the window expires.
    var nowMs by remember { mutableStateOf(System.currentTimeMillis()) }
    LaunchedEffect(contPendingUntil) {
        // The state and the loop condition must read the SAME sample —
        // sampling twice lets the deadline fall between them and strands the
        // spinner on forever (nothing rewrites nowMs once the loop exits).
        while (true) {
            nowMs = System.currentTimeMillis()
            if ((contPendingUntil[channel] ?: 0L) <= nowMs) break
            delay(150)
        }
    }
    val testing = (contPendingUntil[channel] ?: 0L) > nowMs

    if (armed || contTestActive) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Continuity", style = MaterialTheme.typography.bodyLarge)
            if (testing) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    CircularProgressIndicator(Modifier.size(14.dp), strokeWidth = 1.5.dp)
                    Text(
                        "TESTING",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            } else {
                TrPyroContinuityBadge(
                    state = continuity,
                    dotSize = 8.dp,
                    // This row sits beside a bodyLarge label, so it overrides
                    // the badge's compact default rather than forking it.
                    textStyle = MaterialTheme.typography.bodyMedium,
                )
            }
        }
    }
    val blocked = armed || fired || inflight
    if (!blocked) {
        // Rail gate (iOS PyroChannelTestControls twin): with the FC powered
        // off the OC refuses cmd 35 outright — a queued test would deliver
        // its ARM pulse at the next power-on — so don't offer a button that
        // can only be refused.  pwrPinOn reads false until the first
        // telemetry frame, which fails safe to disabled (#377).
        val railOn by remember(session) {
            session.telemetry.map { it.pwrPinOn }.distinctUntilChanged()
        }.collectAsState(initial = false)
        TextButton(
            enabled = railOn,
            onClick = {
                session.sendPyroContTest(channel)
                contTestActive = true
            },
        ) { Text("Test Continuity") }
        if (!railOn) {
            Text(
                "Power on the rocket to test.",
                style = MaterialTheme.typography.bodySmall,
                color = tr.statusWarn,
            )
        }
    }
}

/**
 * iOS segmented Picker analog: evenly-split single-select chips.  Labels
 * stay on ONE line and shrink to fit instead of wrapping — an equal-weight
 * four-up row is narrower than "Dynamic" at body size, and a chip reading
 * "Dynami / c" is worse than a smaller one that reads.
 */
@Composable
private fun SegmentedPicker(options: List<String>, selected: Int, onSelect: (Int) -> Unit) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        options.forEachIndexed { i, label ->
            FilterChip(
                selected = i == selected,
                onClick = { if (i != selected) onSelect(i) },
                label = {
                    Text(
                        label,
                        maxLines = 1,
                        style = MaterialTheme.typography.labelMedium,
                        textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                        modifier = Modifier.fillMaxWidth(),
                    )
                },
                modifier = Modifier.weight(1f),
            )
        }
    }
}

/** iOS Form footer caption. */
@Composable
private fun Caption(text: String) {
    Text(
        text,
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

/** iOS wheel/menu Picker analog: labeled dropdown. */
@Composable
private fun DropdownField(
    label: String,
    options: List<String>,
    selected: Int,
    onSelect: (Int) -> Unit,
    modifier: Modifier = Modifier,
) {
    var open by remember { mutableStateOf(false) }
    Box(modifier) {
        OutlinedButton(onClick = { open = true }, modifier = Modifier.fillMaxWidth()) {
            Text("$label: ${options.getOrElse(selected) { "?" }}")
        }
        DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
            options.forEachIndexed { i, o ->
                DropdownMenuItem(text = { Text(o) }, onClick = { open = false; onSelect(i) })
            }
        }
    }
}

/**
 * iOS "Active on rocket" row: the device-reported ACTIVE mounting beside the
 * profile SETTING (a v3-orientation FC reports it via imu_orient; hidden
 * until then).  A child composable so the session flows don't recompose the
 * whole Settings screen.
 */
@Composable
private fun ActiveOrientRow(session: DeviceSession) {
    val name by session.imuOrientationName.collectAsState()
    val mode by session.imuOrientationMode.collectAsState()
    if (name.isEmpty()) return
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text("Active on rocket", style = MaterialTheme.typography.bodyLarge)
        Text(
            "$name (${mode.label})",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
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

// iOS formatInt / formatDecimal: whole values print as integers, everything
// else via "%.6f" with trailing zeros stripped.  Kotlin's toString() would
// render small magnitudes in scientific notation — a Drag k of 0.0005 showed
// as "5.0E-4", which is both an iOS mismatch and a number nobody wants to
// retype.
private fun plainDecimal(v: Double): String =
    String.format(Locale.ROOT, "%.6f", v).trimEnd('0').let { if (it.endsWith(".")) it + "0" else it }

private fun fmt(f: Float): String =
    if (f == f.toLong().toFloat()) f.toLong().toString() else plainDecimal(f.toDouble())

private fun fmtD(d: Double): String =
    if (d == d.toLong().toDouble()) d.toLong().toString() else plainDecimal(d)
