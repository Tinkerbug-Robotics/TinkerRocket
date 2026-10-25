package com.tinkerbug.tinkerrocket.app

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.app.theme.TrPyroContinuityBadge
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.protocol.PyroContinuity
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.PyroTestFire
import com.tinkerbug.tinkerrocket.session.PyroTestFire.FireBlock
import com.tinkerbug.tinkerrocket.session.PyroTestFire.Phase
import com.tinkerbug.tinkerrocket.session.PyroTestFire.Verdict
import kotlinx.coroutines.delay

/** The dashboard tool route that opens the test fire for [channel]. */
internal fun pyroTestTool(channel: Int): String = "pyrotest:$channel"

/** The channel a [pyroTestTool] route names, or null for any other route. */
internal fun pyroTestChannel(tool: String?): Int? =
    tool?.takeIf { it.startsWith("pyrotest:") }?.removePrefix("pyrotest:")?.toIntOrNull()?.takeIf { it in 1..4 }

/**
 * Test-fire one pyro channel (#1650; iOS `PyroTestView` without the
 * slow-motion video, which is #1673).  A direct link fires over Bluetooth;
 * a base-station link fires the followed rocket over LoRa, from a distance,
 * and tests continuity again afterwards for the stand-back verdict.
 *
 * The flow and every guard live in [PyroTestFire]; this draws it.  Leaving
 * the screen (close, back, or the top bar) stops the countdown and any
 * logging the test started.
 */
@Composable
internal fun PyroTestFireScreen(session: DeviceSession, channel: Int, onClose: () -> Unit) {
    val tr = TrTheme.colors
    val scope = rememberCoroutineScope()
    val test = remember(session, channel) { PyroTestFire(session, channel, scope) }
    LaunchedEffect(test) { test.open() }
    DisposableEffect(test) { onDispose { test.close() } }
    BackHandler { onClose() }

    val state by test.state.collectAsState()
    val connected by session.isConnected.collectAsState()
    val telemetry by session.telemetry.collectAsState()
    val continuity by remember(session, channel) { session.pyroContinuityFlow(channel) }
        .collectAsState(initial = PyroContinuity.NO_DATA)
    val pendingUntil by session.contTestPendingUntil.collectAsState()
    // TESTING ends on a clock edge, not a state change: tick while a window is open.
    var nowMs by remember { mutableLongStateOf(System.currentTimeMillis()) }
    LaunchedEffect(pendingUntil) {
        while (true) {
            nowMs = System.currentTimeMillis()
            if (pendingUntil.values.none { it > nowMs }) break
            delay(150)
        }
    }
    val testing = (pendingUntil[channel] ?: 0L) > nowMs
    val relay = session.isBaseStation
    val focus by session.focusRocketId.collectAsState()
    val dataStatus by session.effectiveDataStatus.collectAsState()
    // The shared-ID check reads the relayed roster, which moves with the frames.
    val block = PyroTestFire.fireBlockOf(
        connected, relay, telemetry.pwrPinOn, session.focusedRocketIdShared(),
    )
    val verdict = test.verdict(testing, connected, dataStatus, focus, continuity)

    Box(
        Modifier
            .fillMaxSize()
            .background(Color.Black)
            .padding(16.dp),
    ) {
        Column(Modifier.fillMaxSize()) {
            // Top bar: channel, path, continuity, close.
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text(
                    "Pyro CH $channel",
                    color = Color.White,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold,
                )
                if (relay) {
                    // This test goes base station → LoRa → rocket: no
                    // Bluetooth range to the rocket is needed.
                    Text(
                        "VIA LoRa",
                        color = Color.White,
                        style = MaterialTheme.typography.labelSmall,
                        fontWeight = FontWeight.Bold,
                        modifier = Modifier
                            .background(tr.accent, RoundedCornerShape(4.dp))
                            .padding(horizontal = 6.dp, vertical = 2.dp),
                    )
                }
                if (testing) {
                    CircularProgressIndicator(Modifier.size(12.dp), color = Color.White, strokeWidth = 1.5.dp)
                    Text("TESTING", color = Color.White, style = MaterialTheme.typography.labelMedium)
                } else {
                    TrPyroContinuityBadge(state = continuity, dotSize = 10.dp)
                }
                Spacer(Modifier.weight(1f))
                IconButton(onClick = onClose) {
                    Icon(Icons.Filled.Close, "Close", tint = Color.White.copy(alpha = 0.8f))
                }
            }

            Spacer(Modifier.weight(1f))
            CenterDisplay(state, relay, verdict)
            Spacer(Modifier.weight(1f))
            BottomControls(test, state, block)
        }
    }
}

@Composable
private fun CenterDisplay(state: PyroTestFire.State, relay: Boolean, verdict: Verdict?) {
    val tr = TrTheme.colors
    Column(
        Modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        when (state.phase) {
            Phase.IDLE -> state.error?.let { ErrorLine(it) }
            Phase.COUNTDOWN -> {
                if (state.startedLogging) {
                    Text("LOGGING", color = tr.logging, style = MaterialTheme.typography.labelLarge)
                }
                Text(
                    "FIRING IN",
                    color = Color.White.copy(alpha = 0.7f),
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold,
                )
                Text(
                    "${state.secondsRemaining}",
                    color = Color.White,
                    fontSize = 120.sp,
                    fontWeight = FontWeight.Bold,
                )
            }
            Phase.AFTER_FIRE -> {
                Text(
                    "FIRE SENT",
                    color = tr.statusBad,
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold,
                )
                Text(
                    "${state.afterFireRemaining}s",
                    color = Color.White,
                    fontSize = 80.sp,
                    fontWeight = FontWeight.Bold,
                )
                Text(
                    "Stay back.",
                    color = Color.White.copy(alpha = 0.7f),
                    style = MaterialTheme.typography.bodyLarge,
                )
            }
            Phase.DONE -> {
                Text(
                    if (state.fired) "Fire command sent" else "No fire command sent",
                    color = Color.White,
                    style = MaterialTheme.typography.headlineSmall,
                    fontWeight = FontWeight.Bold,
                )
                state.error?.let { ErrorLine(it) }
                // The stand-back answer, LoRa path only: an operator on a
                // direct link is in Bluetooth range and re-tests from
                // Settings. Green needs positive evidence; every absence of
                // data says to treat the charge as live.
                if (relay && state.fired) {
                    when (verdict) {
                        Verdict.RETESTING -> Row(
                            horizontalArrangement = Arrangement.spacedBy(6.dp),
                            verticalAlignment = Alignment.CenterVertically,
                        ) {
                            CircularProgressIndicator(
                                Modifier.size(14.dp), color = Color.White, strokeWidth = 1.5.dp,
                            )
                            Text(
                                "Re-testing continuity…",
                                color = Color.White.copy(alpha = 0.8f),
                                style = MaterialTheme.typography.bodyMedium,
                            )
                        }
                        Verdict.FIRED -> VerdictLine("Continuity gone — charge fired", tr.statusOk, bold = false)
                        Verdict.STILL_PRESENT -> VerdictLine(
                            "CONTINUITY STILL PRESENT — charge may not have fired. Treat it as LIVE.",
                            tr.statusWarn,
                        )
                        Verdict.LINK_LOST -> VerdictLine(
                            "COULD NOT CONFIRM — rocket link lost. Treat the charge as LIVE; do not approach.",
                            tr.statusWarn,
                        )
                        Verdict.NO_READING -> VerdictLine(
                            "COULD NOT CONFIRM — no fresh continuity reading. Treat the charge as LIVE; do not approach.",
                            tr.statusWarn,
                        )
                        null -> Unit
                    }
                }
            }
        }
    }
}

@Composable
private fun ErrorLine(text: String) {
    Text(
        text,
        color = TrTheme.colors.statusBad,
        style = MaterialTheme.typography.titleMedium,
        textAlign = TextAlign.Center,
    )
}

@Composable
private fun VerdictLine(text: String, color: Color, bold: Boolean = true) {
    Text(
        text,
        color = color,
        style = MaterialTheme.typography.bodyLarge,
        fontWeight = if (bold) FontWeight.Bold else FontWeight.SemiBold,
        textAlign = TextAlign.Center,
    )
}

@Composable
private fun BottomControls(test: PyroTestFire, state: PyroTestFire.State, block: FireBlock?) {
    val tr = TrTheme.colors
    Column(
        Modifier.fillMaxWidth().padding(bottom = 30.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        when (state.phase) {
            Phase.IDLE, Phase.DONE -> {
                val canFire = block == null
                BigRoundButton(
                    "FIRE",
                    if (canFire) tr.statusBad else tr.statusIdle,
                    enabled = canFire,
                    onClick = { test.fire() },
                )
                when (block) {
                    FireBlock.RAIL_OFF -> Text(
                        "Rocket power is off",
                        color = tr.statusWarn,
                        style = MaterialTheme.typography.titleMedium,
                    )
                    FireBlock.ID_SHARED -> Row(
                        horizontalArrangement = Arrangement.spacedBy(6.dp),
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Icon(Icons.Filled.Warning, null, tint = tr.statusBad)
                        Text(
                            "Two rockets may be using this rocket ID — pyro commands are blocked",
                            color = tr.statusBad,
                            style = MaterialTheme.typography.titleMedium,
                            textAlign = TextAlign.Center,
                        )
                    }
                    FireBlock.NOT_CONNECTED -> Text(
                        "Not connected",
                        color = tr.statusWarn,
                        style = MaterialTheme.typography.titleMedium,
                    )
                    null -> Unit
                }
            }
            Phase.COUNTDOWN -> BigRoundButton("SAFE", tr.statusOk, enabled = true, onClick = { test.safe() })
            Phase.AFTER_FIRE -> Text(
                "Waiting for the charge…",
                color = Color.White.copy(alpha = 0.7f),
                style = MaterialTheme.typography.titleMedium,
            )
        }
    }
}

@Composable
private fun BigRoundButton(label: String, color: Color, enabled: Boolean, onClick: () -> Unit) {
    Box(
        Modifier
            .size(140.dp)
            .clip(CircleShape)
            .background(color)
            .clickable(enabled = enabled, onClick = onClick),
        contentAlignment = Alignment.Center,
    ) {
        Text(label, color = Color.White, fontSize = 26.sp, fontWeight = FontWeight.ExtraBold)
    }
}
