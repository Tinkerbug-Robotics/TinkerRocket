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
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.LocalUnitSystem
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer.SensorCalRun
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.SensorCalGravity
import com.tinkerbug.tinkerrocket.session.UnitFormatter
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.util.Locale

/**
 * On-pad gyro + high-g accelerometer calibration (#1059) — the port of iOS
 * OnPadCalibrationView, which had no Android twin: the settings card told
 * the operator the rocket held a calibration the profile lacked and, until
 * the import landed, to "run calibration to save one" with nothing to run
 * it.  Sits beside the mag-cal tool on the dashboard.
 *
 * All the logic is [ActiveRocketSyncer.runOnPadSensorCal]: cmd 21 out, wait
 * for the SENSOR_CAL_STATUS frame the FC ends every outcome with, snapshot
 * it into the active profile.  This screen is the button, the elapsed-time
 * spinner and the report — including the one thing the frame cannot say,
 * that the rocket refused (it answers inside a second instead of after its
 * 10 s window), and the iOS gravity check afterwards.
 */
@Composable
fun SensorCalScreen(
    session: DeviceSession,
    syncer: ActiveRocketSyncer?,
    onBack: () -> Unit,
) {
    val telemetry by session.telemetry.collectAsState()
    val connected by session.isConnected.collectAsState()
    val units = LocalUnitSystem.current
    val tr = TrTheme.colors
    val scope = rememberCoroutineScope()

    var running by remember { mutableStateOf(false) }
    var startedAtMs by remember { mutableStateOf(0L) }
    var elapsedS by remember { mutableStateOf(0L) }
    var result by remember { mutableStateOf<SensorCalRun?>(null) }
    var gravity by remember { mutableStateOf<SensorCalGravity.Warning?>(null) }

    // The spinner's clock.  The run itself is not on a timer — it ends when
    // the frame arrives or the syncer's own timeout fires.
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
            val r = sy.runOnPadSensorCal { System.currentTimeMillis() }
            result = r
            running = false
            // iOS checks the low-g vector once the cal is over: a rocket that
            // is sitting still should read 1 g.  Read the live value, not the
            // one this composition captured before the run.
            if (r is SensorCalRun.Answered && r.ranWindow) {
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
            Text("Sensor Calibration", style = MaterialTheme.typography.titleLarge)
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("Gyro & Accelerometer Calibration", style = MaterialTheme.typography.titleMedium)
                Text(
                    "Solves for the gyro zero-rate bias and the high-g accelerometer " +
                        "offsets against the low-g accelerometer. Place the rocket on the " +
                        "pad (or the bench) and keep it still — the flight computer " +
                        "samples for 10 seconds. Saved to the active profile and " +
                        "re-applied on connect.",
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
                        Text("Calibrating… ${elapsedS}s — keep the rocket still")
                    } else {
                        Text("Calibrate gyro & accel")
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
            }
        }

        result?.let { r -> ResultCard(r) }

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

@Composable
private fun ResultCard(r: SensorCalRun) {
    val tr = TrTheme.colors
    val units = LocalUnitSystem.current
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
                    if (r.status.valid) {
                        Text(
                            if (r.saved) "Saved to the active profile, tagged with this board."
                            else "Not saved to a profile: there is no active profile, or the board " +
                                "id has not arrived yet — Settings offers an Import once it has.",
                            style = MaterialTheme.typography.bodySmall,
                            color = if (r.saved) MaterialTheme.colorScheme.onSurfaceVariant else tr.statusWarn,
                        )
                        Text(
                            "Gyro bias  ${r.status.gyroX} / ${r.status.gyroY} / ${r.status.gyroZ} LSB\n" +
                                "High-g bias  ${UnitFormatter.acceleration(r.status.hgX.toDouble(), units)} / " +
                                "${UnitFormatter.acceleration(r.status.hgY.toDouble(), units)} / " +
                                UnitFormatter.acceleration(r.status.hgZ.toDouble(), units),
                            style = MaterialTheme.typography.bodySmall,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
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
