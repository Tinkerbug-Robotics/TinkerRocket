package com.tinkerbug.tinkerrocket.app

import android.content.Context
import androidx.activity.compose.BackHandler
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
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
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

/**
 * Phase 7 utility tools — ports of iOS ServoTestView, SimulationView, and
 * FrequencyScanView, reached from the Dashboard's Tools card.
 */

// ── Servo Test ──────────────────────────────────────────────────────────

/**
 * Manual servo test (cmd 24/25).  Sliders span the active profile's fin
 * calibration so the test can command the full mechanical travel —
 * per-profile, not a fixed ±20.  On leave, cmd 25 returns servos to
 * midpoints (the iOS onDisappear discipline).
 */
@Composable
fun ServoTestScreen(session: DeviceSession, profiles: RocketProfileStore?, onBack: () -> Unit) {
    val profile = profiles?.activeProfile
    val lo = min(profile?.finMinDeg ?: -20f, profile?.finMaxDeg ?: 20f).toDouble()
    val hi = max(profile?.finMinDeg ?: -20f, profile?.finMaxDeg ?: 20f).toDouble()
    val range = if (lo < hi) lo..hi else -20.0..20.0

    var angles by remember { mutableStateOf(listOf(0.0, 0.0, 0.0, 0.0)) }

    fun send() = session.sendCommandFrame(Commands.servoTestAngles(angles))

    LaunchedEffect(Unit) { send() } // initial zeros
    // Keyed on session, not Unit, so the hook re-arms if the session identity
    // changes underneath the screen (OfflineMapsScreen keys the same way).
    // No state gate: SERVO_TEST_STOP has no state or session precondition in
    // firmware -- it clears the flag, stows, and arms the pad wake -- so
    // sending it when nothing is running is inert.
    DisposableEffect(session) {
        onDispose { session.sendBareCommand(BleCommandId.SERVO_TEST_STOP) }
    }
    // Without this, the system back gesture fell through to the Activity
    // default and finished the app, racing the teardown above against process
    // death -- with the FC left in servo_test_active, its state machine and
    // pyro servicing suspended, and no UI anywhere to stop it.  Routing back
    // through onBack disposes this screen while the app stays alive, so the
    // stop is an ordinary write on a live link.
    BackHandler(onBack = onBack)

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Dashboard") }
            Text("Servo Test", style = MaterialTheme.typography.titleLarge)
        }
        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                for (i in 0 until 4) {
                    Row(
                        Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                    ) {
                        Text("Servo ${i + 1}", style = MaterialTheme.typography.titleSmall)
                        Text(
                            "%.1f°".format(angles[i]),
                            fontFamily = FontFamily.Monospace,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                    Slider(
                        value = angles[i].toFloat(),
                        onValueChange = { v ->
                            // 0.5° steps like iOS.
                            val stepped = (v * 2).toInt() / 2.0
                            if (stepped != angles[i]) {
                                angles = angles.toMutableList().also { it[i] = stepped }
                                send()
                            }
                        },
                        valueRange = range.start.toFloat()..range.endInclusive.toFloat(),
                    )
                }
            }
        }
        Button(
            onClick = {
                angles = listOf(0.0, 0.0, 0.0, 0.0)
                send()
            },
            modifier = Modifier.fillMaxWidth(),
        ) { Text("Center All") }
        Text(
            "Range ±%.0f°–%.0f° from the active profile's fin calibration. " // ktlint-disable
                .format(abs(range.start), abs(range.endInclusive)) +
                "Servos return to midpoints when you leave.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

// ── Simulation ──────────────────────────────────────────────────────────

/**
 * ESP32 flight-simulator config (cmd 5 + 6).  Wire mass is GRAMS — the OC
 * relay divides by 1000 into SimConfigData.mass_kg (firmware-internal kg;
 * do not "fix" this to pack kg — #592).
 */
@Composable
fun SimulationScreen(session: DeviceSession, onBack: () -> Unit) {
    val context = LocalContext.current
    val prefs = remember { context.getSharedPreferences("simulation", Context.MODE_PRIVATE) }
    val connected by session.isConnected.collectAsState()
    val scope = rememberCoroutineScope()

    var massGrams by remember { mutableStateOf(prefs.getString("massGrams", "500")!!) }
    var thrustN by remember { mutableStateOf(prefs.getString("thrustNewtons", "40")!!) }
    var burnS by remember { mutableStateOf(prefs.getString("burnTimeSeconds", "1.5")!!) }
    var descentMps by remember { mutableStateOf(prefs.getString("descentRateMps", "5.0")!!) }
    var launching by remember { mutableStateOf(false) }
    var confirmLaunch by remember { mutableStateOf(false) }

    fun persist() {
        prefs.edit()
            .putString("massGrams", massGrams).putString("thrustNewtons", thrustN)
            .putString("burnTimeSeconds", burnS).putString("descentRateMps", descentMps)
            .apply()
    }

    val inputsValid = listOf(massGrams, thrustN, burnS, descentMps)
        .all { (it.toFloatOrNull() ?: 0f) > 0f }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Dashboard") }
            Text("Simulation", style = MaterialTheme.typography.titleLarge)
        }
        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Rocket Parameters", style = MaterialTheme.typography.titleMedium)
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    SimField("Mass g", massGrams, Modifier.weight(1f)) { massGrams = it; persist() }
                    SimField("Thrust N", thrustN, Modifier.weight(1f)) { thrustN = it; persist() }
                }
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    SimField("Burn s", burnS, Modifier.weight(1f)) { burnS = it; persist() }
                    SimField("Descent m/s", descentMps, Modifier.weight(1f)) { descentMps = it; persist() }
                }
            }
        }
        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("Estimated Performance", style = MaterialTheme.typography.titleMedium)
                val est = estimateSim(
                    massGrams.toFloatOrNull(), thrustN.toFloatOrNull(),
                    burnS.toFloatOrNull(), descentMps.toFloatOrNull(),
                )
                EstRow("Burnout Speed", est?.let { "%.0f m/s".format(it.maxSpeed) } ?: "—")
                EstRow("Max Altitude", est?.let { "%.0f m".format(it.maxAlt) } ?: "—")
                EstRow("Flight Duration", est?.let { "%.0f s".format(it.totalTime) } ?: "—")
            }
        }
        if (confirmLaunch) {
            // Same informational confirm as iOS (design language): sim pyros
            // DRY-FIRE as of the 2026-07-30 firmware — sequence and report,
            // never energize.
            androidx.compose.material3.AlertDialog(
                onDismissRequest = { confirmLaunch = false },
                title = { Text("Simulated flight") },
                text = {
                    Text(
                        "The rocket flies the full pipeline on synthetic " +
                            "sensors. Pyro channels sequence and report but " +
                            "are dry-fired — no charges are energized " +
                            "(requires current FC firmware).",
                    )
                },
                confirmButton = {
                    TextButton(onClick = {
                        confirmLaunch = false
                        launching = true
                        scope.launch {
                            session.sendTimeSync()
                            session.sendCommandFrame(
                                Commands.simConfig(
                                    massGrams = massGrams.toFloat(),
                                    thrustN = thrustN.toFloat(),
                                    burnTimeS = burnS.toFloat(),
                                    descentRateMps = descentMps.toFloat(),
                                ),
                            )
                            delay(if (session.isBaseStation) 1000L else 300L)
                            session.markSimLaunched()
                            session.sendBareCommand(BleCommandId.SIM_START)
                            launching = false
                            onBack()
                        }
                    }) { Text("Launch") }
                },
                dismissButton = {
                    TextButton(onClick = { confirmLaunch = false }) { Text("Cancel") }
                },
            )
        }
        Button(
            enabled = connected && inputsValid && !launching,
            onClick = {
                persist()
                confirmLaunch = true
            },
            modifier = Modifier.fillMaxWidth(),
        ) { Text(if (launching) "Launching…" else "🔥 Launch Simulation") }
        if (!connected) {
            Text("Not connected to rocket", color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
        } else if (!inputsValid) {
            Text("All fields must be positive numbers", color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
        }
        Text(
            "The simulator runs a 1D physics model on the ESP32, generating " +
                "synthetic sensor data through the full telemetry pipeline. " +
                "Voice, LoRa, and data logging all work during simulation. " +
                "Pyro channels dry-fire: they sequence and report, but are " +
                "never energized.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun SimField(label: String, value: String, modifier: Modifier = Modifier, onChange: (String) -> Unit) {
    OutlinedTextField(
        value = value, onValueChange = onChange, label = { Text(label) },
        singleLine = true, modifier = modifier,
    )
}

@Composable
private fun EstRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace)
    }
}

private class SimEstimate(val maxSpeed: Float, val maxAlt: Float, val totalTime: Float)

/** iOS estimatedPerformance verbatim: Euler at 10 ms, cd 0.5, 50 mm body. */
private fun estimateSim(massG: Float?, thrustN: Float?, burnS: Float?, descentR: Float?): SimEstimate? {
    if (massG == null || massG <= 0 || thrustN == null || thrustN <= 0 ||
        burnS == null || burnS <= 0 || descentR == null || descentR <= 0
    ) {
        return null
    }
    val massKg = massG / 1000f
    val g = 9.80665f
    val cd = 0.5f
    val area = 0.0019635f
    fun rho(alt: Float): Float {
        val x = 1.0f - 2.2558e-5f * alt
        return if (x > 0) 1.225f * x.pow(4.2559f) else 0f
    }

    var alt = 0f
    var vel = 0f
    var maxSpeed = 0f
    var time = 0f
    val dt = 0.01f
    while (time < burnS) {
        val dragA = 0.5f * cd * area * rho(alt) * vel * vel / massKg
        val accel = (thrustN / massKg) - g - (if (vel > 0) dragA else -dragA)
        vel += accel * dt
        alt += vel * dt
        if (alt < 0) alt = 0f
        if (vel > maxSpeed) maxSpeed = vel
        time += dt
    }
    while (vel > 0 && time < 300) {
        val dragA = 0.5f * cd * area * rho(alt) * vel * vel / massKg
        vel += (-g - dragA) * dt
        alt += vel * dt
        time += dt
    }
    return SimEstimate(maxSpeed, alt, time + alt / descentR)
}

// ── Frequency Scan ──────────────────────────────────────────────────────

/**
 * Pre-launch RF collision avoidance (cmd 60, BS only): sweep the ISM band,
 * plot RSSI-per-channel as noise margin.  In hopping mode (#150 — the
 * fleet default) the scan's product is the channel mask the BS pushes
 * automatically; there is no single frequency to move to, so no Apply.
 */
@Composable
fun FreqScanScreen(session: DeviceSession, onBack: () -> Unit) {
    val context = LocalContext.current
    val prefs = remember { context.getSharedPreferences("freqscan", Context.MODE_PRIVATE) }
    val samples by session.scanSamples.collectAsState()
    val scanning by session.isScanning.collectAsState()
    val config by session.rocketConfig.collectAsState()

    var startMHz by remember { mutableStateOf(prefs.getString("startMHz", "902.0")!!) }
    var stopMHz by remember { mutableStateOf(prefs.getString("stopMHz", "928.0")!!) }
    var stepKHz by remember { mutableStateOf(prefs.getString("stepKHz", "500")!!) }
    var dwellMs by remember { mutableStateOf(prefs.getString("dwellMs", "30")!!) }

    fun persist() {
        prefs.edit()
            .putString("startMHz", startMHz).putString("stopMHz", stopMHz)
            .putString("stepKHz", stepKHz).putString("dwellMs", dwellMs)
            .apply()
    }

    val start = startMHz.toDoubleOrNull() ?: 0.0
    val stop = stopMHz.toDoubleOrNull() ?: 0.0
    val step = stepKHz.toDoubleOrNull() ?: 0.0
    val dwell = dwellMs.toDoubleOrNull() ?: 0.0
    val channels = if (step > 0 && stop > start) ((stop - start) * 1000.0 / step).toInt() + 1 else 0
    val paramsValid = stop > start && step > 0 && dwell > 0 && channels in 1..128
    val durationMs = channels * (dwell + 2.0)

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Dashboard") }
            Text("Frequency Scan", style = MaterialTheme.typography.titleLarge)
        }

        Card {
            Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Scan Range", style = MaterialTheme.typography.titleMedium)
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    SimField("Start MHz", startMHz, Modifier.weight(1f)) { startMHz = it; persist() }
                    SimField("Stop MHz", stopMHz, Modifier.weight(1f)) { stopMHz = it; persist() }
                }
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    SimField("Step kHz", stepKHz, Modifier.weight(1f)) { stepKHz = it; persist() }
                    SimField("Dwell ms", dwellMs, Modifier.weight(1f)) { dwellMs = it; persist() }
                }
                EstRow("Channels", if (channels in 1..128) "$channels" else "$channels (max 128)")
                EstRow(
                    "Est. Duration",
                    if (durationMs < 1000) "%.0f ms".format(durationMs) else "%.1f s".format(durationMs / 1000),
                )
            }
        }

        Button(
            enabled = !scanning && paramsValid,
            onClick = {
                session.startFrequencyScan(
                    startMHz = start.toFloat(), stopMHz = stop.toFloat(),
                    stepKHz = step.toInt(), dwellMs = dwell.toInt(),
                )
            },
            modifier = Modifier.fillMaxWidth(),
        ) {
            if (scanning) {
                CircularProgressIndicator(Modifier.height(18.dp))
                Text("  Scanning…")
            } else {
                Text("Start Scan")
            }
        }

        if (samples.isNotEmpty()) {
            Card {
                Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text("Result", style = MaterialTheme.typography.titleMedium)
                    ScanChart(samples.map { it.freqMhz.toDouble() to (REFERENCE_DBM - it.rssiDbm) })

                    val quiet = samples.minByOrNull { it.rssiDbm }
                    if (quiet != null) {
                        EstRow(
                            "Quietest channel",
                            "%.2f MHz · %d dB".format(quiet.freqMhz, REFERENCE_DBM - quiet.rssiDbm),
                        )
                        val noisy = samples.count { (REFERENCE_DBM - it.rssiDbm) < QUIET_MARGIN_DB }
                        EstRow("Below $QUIET_MARGIN_DB dB margin", "$noisy of ${samples.size}")
                    }
                }
            }

            if (config?.loraHopDisabled == false) {
                // #150: no single frequency to move to mid-hop — the mask is
                // the product and the BS already pushed it (cmd 15).
                Text(
                    "Hopping mode: the scan result was applied automatically as " +
                        "a channel mask — noisy channels are skipped and the link " +
                        "keeps hopping. No frequency move is needed.",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF43A047),
                )
            } else {
                Text(
                    "Fixed-frequency link: apply the quietest channel from the " +
                        "iOS app for now — the transactional frequency-move flow " +
                        "lands on Android with the OTA-era polish.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }
}

private const val REFERENCE_DBM = -40
private const val QUIET_MARGIN_DB = 70

/** Noise-margin bar chart: green = comfortable (≥70 dB), orange = noisy. */
@Composable
private fun ScanChart(points: List<Pair<Double, Int>>) {
    val textMeasurer = rememberTextMeasurer()
    val labelColor = MaterialTheme.colorScheme.onSurfaceVariant
    Canvas(Modifier.fillMaxWidth().height(220.dp)) {
        if (points.isEmpty()) return@Canvas
        val bottomPad = 18.dp.toPx()
        val plotH = size.height - bottomPad
        val freqLo = points.minOf { it.first }
        val freqHi = points.maxOf { it.first }
        val pad = if (points.size > 1) (freqHi - freqLo) / (points.size - 1) * 0.5 else 0.5
        val lo = freqLo - pad
        val hi = freqHi + pad
        val span = hi - lo
        val barW = 6.dp.toPx()

        for ((freq, margin) in points) {
            val x = ((freq - lo) / span * size.width).toFloat()
            val h = (margin.coerceIn(0, 100) / 100f) * plotH
            drawRect(
                color = if (margin >= QUIET_MARGIN_DB) Color(0xFF43A047) else Color(0xFFFB8C00),
                topLeft = Offset(x - barW / 2, plotH - h),
                size = Size(barW, h),
            )
        }
        val style = TextStyle(fontSize = 9.sp, color = labelColor)
        for (i in 0..4) {
            val f = lo + span * i / 4
            val label = textMeasurer.measure("%.0f".format(f), style)
            drawText(
                label,
                topLeft = Offset(
                    (((f - lo) / span * size.width).toFloat() - label.size.width / 2)
                        .coerceIn(0f, size.width - label.size.width),
                    plotH + 2.dp.toPx(),
                ),
            )
        }
    }
}
