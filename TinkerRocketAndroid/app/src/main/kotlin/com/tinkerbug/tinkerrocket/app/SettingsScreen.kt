package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
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
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
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
) {
    val profiles by store.profiles.collectAsState()
    val activeId by store.activeId.collectAsState()
    val syncState by syncer.syncState.collectAsState()
    val magAdvisory by syncer.magCalAdvisory.collectAsState()
    val sensorAdvisory by syncer.sensorCalAdvisory.collectAsState()
    val suggestedId by syncer.suggestedProfileId.collectAsState()
    val active = activeId?.let { id -> profiles.firstOrNull { it.id == id } }

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
            }
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
                    edit(ConfigGroup.IMU) { it.copy(imuOrientSetting = 0xFF) }
                } else if (i == 0 && auto) {
                    val restore = savedOrientCode ?: 0
                    edit(ConfigGroup.IMU) { it.copy(imuOrientSetting = restore) }
                }
            }
            if (!auto) {
                val code = active.imuOrientSetting.coerceIn(0, 23)
                FieldRow {
                    DropdownField(
                        "Nose axis", listOf("+X", "-X", "+Y", "-Y", "+Z", "-Z"), code / 4,
                        { a -> edit(ConfigGroup.IMU) { it.copy(imuOrientSetting = a * 4 + code % 4) } },
                        Modifier.weight(1f),
                    )
                    DropdownField(
                        "Fin clocking", listOf("0°", "90°", "180°", "270°"), code % 4,
                        { c -> edit(ConfigGroup.IMU) { it.copy(imuOrientSetting = (code / 4) * 4 + c) } },
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
                edit(ConfigGroup.IMU) { it.copy(imuRateHz = rates[i]) }
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
        }

        // ── Fin layout (cmd 66) ──────────────────────────────────────────
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

        // ── Roll profile (cmd 26; Clear = cmd 27, matching iOS) ──────────
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
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val (label, color) = when (state) {
        SyncState.Idle -> return
        SyncState.AwaitingSync -> "awaiting sync" to tr.statusWarn
        SyncState.NoProfile -> "no profile" to tr.statusIdle
        SyncState.Syncing -> "syncing…" to tr.statusIdle
        SyncState.Synced -> "synced" to tr.statusOk
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
        ToggleRow("Enabled", enabled, onEnabled)
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
    val contBit by remember(session, channel) {
        session.telemetry.map { it.pyroCont(channel) }.distinctUntilChanged()
    }.collectAsState(initial = false)
    val inflight by remember(session) {
        session.telemetry.map { it.state == "INFLIGHT" }.distinctUntilChanged()
    }.collectAsState(initial = false)
    val dataStatus by session.effectiveDataStatus.collectAsState()
    val contPendingUntil by session.contTestPendingUntil.collectAsState()

    // #297 fail-safe: continuity is trusted only from a LIVE frame.
    val cont = contBit && dataStatus == com.tinkerbug.tinkerrocket.protocol.TelemetryData.DataStatus.LIVE

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
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    val c = if (cont) tr.statusOk else tr.statusBad
                    Box(Modifier.size(8.dp).background(c, CircleShape))
                    Text(
                        if (cont) "CONT" else "NO CONT",
                        style = MaterialTheme.typography.bodyMedium,
                        color = c,
                    )
                }
            }
        }
    }
    val blocked = armed || fired || inflight
    if (!blocked) {
        TextButton(onClick = {
            session.sendPyroContTest(channel)
            contTestActive = true
        }) { Text("Test Continuity") }
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
