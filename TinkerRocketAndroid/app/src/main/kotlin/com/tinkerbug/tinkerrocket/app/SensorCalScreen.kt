package com.tinkerbug.tinkerrocket.app

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.protocol.PadCalState
import com.tinkerbug.tinkerrocket.protocol.PadCalStatus
import com.tinkerbug.tinkerrocket.protocol.SensorCalStatus
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.PadCalRun
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SensorCalRun
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.IonosphereSource
import com.tinkerbug.tinkerrocket.session.PadCalReadout
import com.tinkerbug.tinkerrocket.session.SensorCalGravity
import com.tinkerbug.tinkerrocket.session.UnitFormatter
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.time.ZoneId
import java.util.Locale

/**
 * Pad calibration (#1600 phase 5; #1059 before it) — the port of iOS
 * OnPadCalibrationView, on the dashboard's Tools row beside mag cal.  One
 * tap runs the flight computer's gyro + high-g calibration, restarts its
 * GNSS pad survey, and hands it the day's broadcast ionosphere for the
 * raw-GNSS filter.
 *
 * The logic lives elsewhere: [IonosphereSource.forPadCalibration] picks the
 * set to send (one short refresh at most, then the phone's cache, then
 * none); [ActiveRocketSyncer.runPadCalibration] sends command 75 — or 21 to a
 * rocket that predates it — and snapshots the 0xCB answer into the active
 * profile; [PadCalReadout] words the 0xD0 status.  This screen is the
 * button, the live status and the report.
 *
 * Two shapes, by firmware.  A rocket sending 0xD0 frames gets the live
 * status: the window's progress and verdict and the GNSS survey.  One that
 * does not gets #1059's elapsed-time spinner and report, unchanged —
 * including the one thing its 0xCB frame cannot say, that the rocket refused
 * (it answers inside a second instead of after its 10 s window), and the iOS
 * gravity check afterwards.
 */
@Composable
fun SensorCalScreen(
    session: DeviceSession,
    syncer: ActiveRocketSyncer?,
    ionosphere: IonosphereSource?,
    onBack: () -> Unit,
) {
    val telemetry by session.telemetry.collectAsState()
    val connected by session.isConnected.collectAsState()
    val padCal by session.padCalStatus.collectAsState()
    val phoneIono = ionosphere?.held?.collectAsState()?.value
    val units = LocalUnitSystem.current
    val tr = TrTheme.colors
    val scope = rememberCoroutineScope()

    var running by remember { mutableStateOf(false) }
    var fetchingIono by remember { mutableStateOf(false) }
    var startedAtMs by remember { mutableStateOf(0L) }
    var elapsedS by remember { mutableStateOf(0L) }
    var result by remember { mutableStateOf<PadCalRun?>(null) }
    var gravity by remember { mutableStateOf<SensorCalGravity.Warning?>(null) }

    // Opening the screen is a moment to bring the phone's set up to date.  It
    // never holds anything up: the refresh runs in the background.
    LaunchedEffect(ionosphere) { ionosphere?.refreshIfStale() }

    // The 0xD0 frames keep coming while the rocket is on the ground, so the
    // screen stays on while it shows them.  MainActivity's #385 flag already
    // does that for any connected device; this holds it here regardless.
    val view = LocalView.current
    val live = padCal != null
    DisposableEffect(view, live) {
        val before = view.keepScreenOn
        if (live) view.keepScreenOn = true
        onDispose { view.keepScreenOn = before }
    }

    // The legacy spinner's clock.  The run itself is not on a timer — it ends
    // when the frame arrives or the syncer's own timeout fires.
    LaunchedEffect(running) {
        if (!running) return@LaunchedEffect
        while (true) {
            elapsedS = (System.currentTimeMillis() - startedAtMs) / 1000
            delay(250)
        }
    }

    fun run() {
        val sy = syncer ?: return
        running = true
        result = null
        gravity = null
        startedAtMs = System.currentTimeMillis()
        scope.launch {
            fetchingIono = true
            val iono = ionosphere?.forPadCalibration()
            fetchingIono = false
            startedAtMs = System.currentTimeMillis()
            val r = sy.runPadCalibration(iono) { System.currentTimeMillis() }
            result = r
            running = false
            // #1059's check on the low-g vector once the cal is over, for the
            // legacy flow.  A rocket that sends 0xD0 reports the gravity its
            // window measured, which the live card judges instead.  Read the
            // live value, not the one this composition captured before the run.
            val cal = r.sensorCal
            if (r.legacy && cal is SensorCalRun.Answered && cal.ranWindow) {
                val t = session.telemetry.value
                gravity = SensorCalGravity.check(t.lowGX, t.lowGY, t.lowGZ)
            }
        }
    }

    // The same refusals the FC applies (sensor_cal::refusal): a rocket that
    // has launched or landed keeps its previous calibration.  Greying the
    // button out says so before the tap instead of after the round trip.
    val refusedState = telemetry.state == "INFLIGHT" || telemetry.state == "LANDED"
    val canRun = connected && telemetry.pwrPinOn && !refusedState && !running && syncer != null

    BackHandler { onBack() }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Dashboard") }
            Text("Pad Calibration", style = MaterialTheme.typography.titleLarge)
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text(
                    "Place the rocket on its pad and keep it still. The gyro & accelerometer " +
                        "calibration takes about 10 seconds; the GNSS pad survey keeps going " +
                        "until launch. The gyro & accel offsets are saved to the active " +
                        "profile and re-applied on connect.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Button(onClick = ::run, enabled = canRun, modifier = Modifier.fillMaxWidth()) {
                    if (running) {
                        CircularProgressIndicator(
                            Modifier.size(16.dp),
                            strokeWidth = 2.dp,
                            color = MaterialTheme.colorScheme.onPrimary,
                        )
                        Spacer(Modifier.width(8.dp))
                        Text(
                            when {
                                fetchingIono -> "Getting today's ionosphere…"
                                live -> "Calibrating — keep the rocket still"
                                else -> "Calibrating… ${elapsedS}s — keep the rocket still"
                            },
                        )
                    } else {
                        Text("Start pad calibration")
                    }
                }
                when {
                    !connected -> Text("Not connected.", style = MaterialTheme.typography.bodySmall, color = tr.statusWarn)
                    !telemetry.pwrPinOn -> Text(
                        "Power on the rocket to run the calibration.",
                        style = MaterialTheme.typography.bodySmall, color = tr.statusWarn,
                    )
                    refusedState -> Text(
                        "The rocket refuses a calibration after launch (state ${telemetry.state}) " +
                            "and keeps the one it has.",
                        style = MaterialTheme.typography.bodySmall, color = tr.statusWarn,
                    )
                }
                Text(
                    PadCalReadout.phoneIonosphere(phoneIono, ZoneId.systemDefault()),
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }

        padCal?.let { s ->
            GyroAccelCard(
                s,
                // 75 is out, and no frame has shown its window yet: the state
                // on screen is still the one from before the tap.
                starting = running && !fetchingIono && s.calState != PadCalState.RUNNING,
                answer = result?.takeUnless { it.legacy }?.sensorCal,
            )
            if (s.rawGnss) GnssCard(s)
        }

        result?.takeIf { it.legacy || padCal == null }?.let { r -> ResultCard(r.sensorCal) }

        gravity?.let { w ->
            Card(colors = CardDefaults.cardColors(containerColor = tr.statusWarn.copy(alpha = 0.15f))) {
                Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                    Text("Accelerometer warning", style = MaterialTheme.typography.titleSmall, color = tr.statusWarn)
                    Text(
                        "Low-G accelerometer magnitude (" +
                            UnitFormatter.acceleration(w.magnitudeMps2.toDouble(), units) +
                            ") differs from expected gravity (" +
                            UnitFormatter.acceleration(SensorCalGravity.G_MPS2.toDouble(), units) +
                            ") by ${String.format(Locale.ROOT, "%.1f", w.errorPct)}%. " +
                            "Consider running a bench calibration before flight.",
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            }
        }
    }
}

/** The gyro/accel window from the live 0xD0 status, and this run's 0xCB answer under it. */
@Composable
private fun GyroAccelCard(s: PadCalStatus, starting: Boolean, answer: SensorCalRun?) {
    val tr = TrTheme.colors
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("Gyro & accel", style = MaterialTheme.typography.titleMedium)
            when {
                starting -> {
                    LinearProgressIndicator(Modifier.fillMaxWidth())
                    Text("Starting…", style = MaterialTheme.typography.bodyMedium)
                }
                s.calState == PadCalState.RUNNING -> {
                    LinearProgressIndicator(
                        progress = { s.calProgress / 100f },
                        modifier = Modifier.fillMaxWidth(),
                    )
                    ReadoutText(PadCalReadout.gyroAccel(s))
                }
                else -> ReadoutText(PadCalReadout.gyroAccel(s))
            }
            // The window's own gravity, judged as #1059 judged the low-g vector:
            // the flight computer keeps anything within ±20 %, so a DONE can
            // still be worth a warning.
            if (!starting && s.calState == PadCalState.DONE) {
                SensorCalGravity.checkWindow(s.calGravityMg)?.let { w ->
                    Text(
                        "Gravity read ${String.format(Locale.ROOT, "%.1f", w.errorPct)} % off 1 g. " +
                            "Consider running a bench calibration before flight.",
                        style = MaterialTheme.typography.bodySmall,
                        color = tr.statusWarn,
                    )
                }
            }
            when (answer) {
                is SensorCalRun.Answered ->
                    if (answer.status.valid) {
                        SavedCal(answer.status, answer.saved)
                    } else {
                        Text(
                            "The rocket holds no gyro & accel calibration.",
                            style = MaterialTheme.typography.bodySmall,
                            color = tr.statusWarn,
                        )
                    }
                SensorCalRun.NoAnswer -> Text(
                    "No calibration status came back in " +
                        "${ActiveRocketSyncer.SENSOR_CAL_RUN_TIMEOUT_MS / 1000} s. The rocket keeps " +
                        "whatever it had.",
                    style = MaterialTheme.typography.bodySmall,
                    color = tr.statusWarn,
                )
                SensorCalRun.NotConnected ->
                    Text("Not connected.", style = MaterialTheme.typography.bodySmall, color = tr.statusWarn)
                null -> Unit
            }
        }
    }
}

/** The raw-GNSS filter's pad survey; only on a board that runs it (the LC86G). */
@Composable
private fun GnssCard(s: PadCalStatus) {
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("GNSS pad survey", style = MaterialTheme.typography.titleMedium)
            PadCalReadout.gnss(s).forEach { ReadoutText(it) }
        }
    }
}

@Composable
private fun ReadoutText(line: PadCalReadout.Line) {
    val tr = TrTheme.colors
    Text(
        line.text,
        style = MaterialTheme.typography.bodyMedium,
        color = when (line.tone) {
            PadCalReadout.Tone.OK -> tr.statusOk
            PadCalReadout.Tone.WARN -> tr.statusWarn
            PadCalReadout.Tone.NORMAL -> MaterialTheme.colorScheme.onSurface
        },
    )
}

/** Where the 0xCB answer went, and what it holds. */
@Composable
private fun SavedCal(status: SensorCalStatus, saved: Boolean) {
    val tr = TrTheme.colors
    val units = LocalUnitSystem.current
    Text(
        if (saved) "Saved to the active profile, tagged with this board."
        else "Not saved to a profile: there is no active profile, or the board " +
            "id has not arrived yet — Settings offers an Import once it has.",
        style = MaterialTheme.typography.bodySmall,
        color = if (saved) MaterialTheme.colorScheme.onSurfaceVariant else tr.statusWarn,
    )
    Text(
        "Gyro bias  ${status.gyroX} / ${status.gyroY} / ${status.gyroZ} LSB\n" +
            "High-g bias  ${UnitFormatter.acceleration(status.hgX.toDouble(), units)} / " +
            "${UnitFormatter.acceleration(status.hgY.toDouble(), units)} / " +
            UnitFormatter.acceleration(status.hgZ.toDouble(), units),
        style = MaterialTheme.typography.bodySmall,
        fontFamily = FontFamily.Monospace,
    )
}

/** #1059's report, for a rocket that sends no 0xD0 status. */
@Composable
private fun ResultCard(r: SensorCalRun) {
    val tr = TrTheme.colors
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            when (r) {
                is SensorCalRun.Answered -> {
                    val secs = String.format(Locale.ROOT, "%.1f", r.elapsedMs / 1000.0)
                    if (!r.ranWindow) {
                        Text("Refused", style = MaterialTheme.typography.titleMedium, color = tr.statusWarn)
                        Text(
                            "The rocket answered in $secs s without opening a calibration window. " +
                                "It refuses after a launch is detected, in flight, after landing, " +
                                "or while a window is already open — and keeps its previous " +
                                "calibration.",
                            style = MaterialTheme.typography.bodySmall,
                        )
                    } else if (!r.status.valid) {
                        Text("No calibration", style = MaterialTheme.typography.titleMedium, color = tr.statusWarn)
                        Text(
                            "The window closed after $secs s but the rocket holds no calibration — " +
                                "it was rejected. Check the rocket was still and try again.",
                            style = MaterialTheme.typography.bodySmall,
                        )
                    } else {
                        Text("Calibration complete", style = MaterialTheme.typography.titleMedium, color = tr.statusOk)
                        Text("Sampled for $secs s.", style = MaterialTheme.typography.bodySmall)
                    }
                    if (r.status.valid) SavedCal(r.status, r.saved)
                }
                SensorCalRun.NoAnswer -> {
                    Text("No answer", style = MaterialTheme.typography.titleMedium, color = tr.statusWarn)
                    Text(
                        "No calibration status came back in " +
                            "${ActiveRocketSyncer.SENSOR_CAL_RUN_TIMEOUT_MS / 1000} s. The rocket keeps " +
                            "whatever it had. Only a direct link runs this — the base station " +
                            "does not relay it.",
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
                SensorCalRun.NotConnected -> {
                    Text("Not connected", style = MaterialTheme.typography.titleMedium, color = tr.statusWarn)
                }
            }
        }
    }
}
