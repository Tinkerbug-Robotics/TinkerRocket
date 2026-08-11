package com.tinkerbug.tinkerrocket.app

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
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.MagCalRejectCode
import com.tinkerbug.tinkerrocket.protocol.MagCalStatus
import com.tinkerbug.tinkerrocket.protocol.MagCalSubType
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.DeviceSession
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Magnetometer hard-iron calibration (#96/#148/#206) — port of iOS
 * MagCalView.  All state lives on the FC; this is a renderer + one-byte
 * command buttons; magCalStatus is the source of truth at 5 Hz.
 *
 * APPLIED-trap semantics (PR #490): the FC RESTS in APPLIED whenever a
 * calibration exists, so APPLIED on entry means "already calibrated" and
 * shows the intro with a Recalibrate button — never the dead-end Saved
 * banner.  Only a live SAMPLING/REVIEW/VERIFYING frame seen this session
 * flips APPLIED to the one-shot Saved banner.
 *
 * The iOS SceneKit sphere renders here as a 4×8 unrolled coverage grid
 * (plan §1: 2D-projected Canvas, no 3D engine) — same 32 firmware bins,
 * green = covered, amber = partial, plus a dominant-gravity-axis readout
 * from the live low-g accel in place of the rolling ball.
 */
@Composable
fun MagCalScreen(
    session: DeviceSession,
    syncer: ActiveRocketSyncer?,
    onBack: () -> Unit,
) {
    val status by session.magCalStatus.collectAsState()
    val telemetry by session.telemetry.collectAsState()
    val connected by session.isConnected.collectAsState()

    // PR #490: only a live cal frame this session makes APPLIED terminal.
    var ranCalThisSession by remember { mutableStateOf(false) }
    // #148 verify-window |B| accumulators (reset on leave/retry).
    var verifyMin by remember { mutableStateOf<Float?>(null) }
    var verifyMax by remember { mutableStateOf<Float?>(null) }

    LaunchedEffect(status) {
        val s = status ?: return@LaunchedEffect
        if (s.subType == MagCalSubType.SAMPLING || s.subType == MagCalSubType.REVIEW ||
            s.subType == MagCalSubType.VERIFYING
        ) {
            ranCalThisSession = true
        }
        if (s.subType == MagCalSubType.VERIFYING) {
            val b = s.instantaneousFieldUt
            if (b > 0) { // FC sends 0 before the first verify sample
                verifyMin = min(verifyMin ?: b, b)
                verifyMax = max(verifyMax ?: b, b)
            }
        } else {
            verifyMin = null
            verifyMax = null
        }
        // #132: snapshot a freshly-accepted cal into the active profile
        // (board-tagged) so the syncer re-applies it on connect.
        if (s.subType == MagCalSubType.APPLIED && ranCalThisSession) {
            syncer?.importRocketCalIntoActiveProfile(System.currentTimeMillis())
        }
    }

    fun bare(cmd: Int) = session.sendBareCommand(cmd)

    val subType = status?.subType ?: MagCalSubType.IDLE
    val midCal = subType.needsAbortOnLeave

    // The abort is owned by teardown, not by the Cancel button, because Cancel
    // was never the only way out.  The screen also disappears when the top bar
    // switches tab, when the chevron disconnects, when the system back gesture
    // pops it, and when a rotation recreates the Activity -- and every one of
    // those left the FC sitting in MAG_CALIBRATION with the magnetometer
    // offsets zeroed, invisible from wherever the operator ended up.
    //
    // Gated on midCal so it is inert when nothing is running: the FC rests in
    // APPLIED whenever a calibration exists, and blanket-aborting that on every
    // visit to the screen would be its own bug.  `latestMidCal` because the
    // onDispose closure would otherwise capture the value from the composition
    // that installed it, which is IDLE on entry -- exactly when it must not be.
    //
    // The gate is widened past midCal by ranCalThisSession because midCal comes
    // from live 5 Hz FC telemetry: a null or stale magCalStatus at teardown
    // would silently degrade it to "send nothing" at exactly the moment it must
    // not. `subType != APPLIED` is the part that actually protects anything --
    // everything else in the FC's abort handler is individually guarded and it
    // never writes NVS, so APPLIED is the only state a stray abort can damage.
    val needsAbort = midCal || (ranCalThisSession && subType != MagCalSubType.APPLIED)
    val latestNeedsAbort by rememberUpdatedState(needsAbort)
    var abortSent by remember { mutableStateOf(false) }
    fun abortOnce() {
        if (!abortSent) {
            abortSent = true
            bare(BleCommandId.MAG_CAL_ABORT_OC)
        }
    }
    DisposableEffect(session) {
        onDispose { if (latestNeedsAbort) abortOnce() }
    }
    fun abortAndBack() {
        abortOnce()
        onBack()
    }
    // System back popped to the launcher instead of here, because the app has
    // no BackHandler and back fell through to the Activity default.  Routing it
    // to the same path as Cancel keeps the process alive long enough for the
    // abort to actually reach the rocket.
    BackHandler { if (midCal) abortAndBack() else onBack() }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            // Belt-and-suspenders abort on back-out mid-cal.
            TextButton(onClick = { if (midCal) abortAndBack() else onBack() }) {
                Text(if (midCal) "✕ Cancel" else "← Dashboard")
            }
            Text("Mag Calibration", style = MaterialTheme.typography.titleLarge)
        }

        when (subType) {
            MagCalSubType.IDLE, MagCalSubType.ABORTED ->
                IntroSection(status, telemetry.pwrPinOn, telemetry.state, connected, session) { bare(BleCommandId.MAG_CAL_START_OC) }
            MagCalSubType.APPLIED ->
                if (ranCalThisSession) {
                    AppliedSection(status)
                } else {
                    IntroSection(status, telemetry.pwrPinOn, telemetry.state, connected, session) { bare(BleCommandId.MAG_CAL_START_OC) }
                }
            MagCalSubType.SAMPLING -> SamplingSection(
                status, telemetry.lowGX, telemetry.lowGY, telemetry.lowGZ,
                onComputeFit = { bare(BleCommandId.MAG_CAL_COMPUTE_FIT_OC) },
            )
            MagCalSubType.REVIEW -> ReviewSection(
                status,
                onVerify = { bare(BleCommandId.MAG_CAL_ACCEPT_OC) },
                onForceApply = { bare(BleCommandId.MAG_CAL_FORCE_APPLY_OC) },
                onRetry = { bare(BleCommandId.MAG_CAL_RETRY_OC) },
                onAbort = ::abortAndBack,
            )
            MagCalSubType.VERIFYING -> VerifyingSection(
                status, verifyMin, verifyMax,
                onAcceptSave = { allGood ->
                    bare(if (allGood) BleCommandId.MAG_CAL_VERIFY_DONE_OC else BleCommandId.MAG_CAL_FORCE_APPLY_OC)
                },
                onRetryVerify = {
                    verifyMin = null; verifyMax = null
                    bare(BleCommandId.MAG_CAL_VERIFY_RESET_OC)
                },
                onRerunCal = {
                    verifyMin = null; verifyMax = null
                    bare(BleCommandId.MAG_CAL_RETRY_OC)
                },
                onAbort = ::abortAndBack,
            )
        }
    }
}

// ── Constants (iOS MagCalConstants) ─────────────────────────────────────

private object MagCalConst {
    const val MIN_SAMPLES = 500
    const val MIN_COVERAGE_BINS = 22 // of 32, #148
    const val VERIFY_MIN_UT = 20.0f
    const val VERIFY_MAX_UT = 70.0f
    const val VERIFY_RANGE_UT = 25.0f
    const val VERIFY_MIN_COVERAGE_BINS = 8
    const val VERIFY_MIN_SAMPLES = 100
}

// ── Sections ────────────────────────────────────────────────────────────

@Composable
private fun IntroSection(
    status: MagCalStatus?,
    powerOn: Boolean,
    state: String,
    connected: Boolean,
    session: DeviceSession,
    onStart: () -> Unit,
) {
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("Magnetometer Hard-Iron Calibration", style = MaterialTheme.typography.titleMedium)
            Text(
                "Solves for the fixed magnetic offset on the rocket PCB so the " +
                    "magnetometer reads true Earth field. Run once per board (or " +
                    "after any hardware change near the mag chip).\n\n" +
                    "1. Hold the rocket clear of laptops, phones, tools, and steel.\n" +
                    "2. Tap Start, then tumble the rocket through all orientations — " +
                    "the coverage grid fills green as the firmware bins fill.\n" +
                    "3. Tap Compute Fit once most cells are green.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
    if (status?.subType == MagCalSubType.ABORTED) {
        Text(
            "Previous run cancelled.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
    if (status?.subType == MagCalSubType.APPLIED) {
        Text(
            "✓ Already calibrated — Earth field %.1f µT, residual %.1f µT."
                .format(status.fieldRUt, status.residualUt),
            style = MaterialTheme.typography.bodySmall,
            color = Color(0xFF43A047),
        )
    }
    // Start gate: connected + rocket + powered + not INFLIGHT (FC refuses).
    val canStart = connected && !session.isBaseStation && powerOn && state != "INFLIGHT"
    Button(enabled = canStart, onClick = onStart, modifier = Modifier.fillMaxWidth()) {
        Text(if (status?.subType == MagCalSubType.APPLIED) "Recalibrate" else "Start Calibration")
    }
    Text(
        "For the calibration to match what the flight computer sees, ensure " +
            "the rocket is in the flight configuration before calibrating.",
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

@Composable
private fun SamplingSection(
    status: MagCalStatus?,
    ax: Float?,
    ay: Float?,
    az: Float?,
    onComputeFit: () -> Unit,
) {
    val s = status ?: return
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Tumble the rocket", style = MaterialTheme.typography.titleMedium)
            CoverageGrid(s.coverageMask, s.partialMask)
            // Dominant gravity axis stands in for the SceneKit rolling ball.
            val dom = dominantAxis(ax, ay, az)
            Text(
                if (dom != null) "Gravity through: $dom" else "Gravity: —",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            MetricRowM("Orientation coverage", "${s.coverageBins} / 32")
            MetricRowM("Samples", "${s.sampleCount}")
        }
    }
    val hasSamples = s.sampleCount >= MagCalConst.MIN_SAMPLES
    val coverageMet = s.coverageBins >= MagCalConst.MIN_COVERAGE_BINS
    val canFit = hasSamples && coverageMet
    Button(enabled = canFit, onClick = onComputeFit, modifier = Modifier.fillMaxWidth()) {
        Text("Compute Fit")
    }
    Text(
        when {
            canFit -> "Ready to fit. Keep rotating to cover remaining cells, or tap Compute Fit."
            !hasSamples -> "Collecting samples — keep rotating. Need at least ${MagCalConst.MIN_SAMPLES}."
            else -> "Need at least ${MagCalConst.MIN_COVERAGE_BINS} of 32 cells before the fit can run."
        },
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

@Composable
private fun ReviewSection(
    status: MagCalStatus?,
    onVerify: () -> Unit,
    onForceApply: () -> Unit,
    onRetry: () -> Unit,
    onAbort: () -> Unit,
) {
    val s = status ?: return
    val ok = s.rejectCode == MagCalRejectCode.OK
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("Fit result", style = MaterialTheme.typography.titleMedium)
            Text(
                rejectMessage(s.rejectCode),
                color = if (ok) Color(0xFF43A047) else Color(0xFFFFA000),
                style = MaterialTheme.typography.bodyMedium,
            )
            MetricRowM("Earth field |R|", "%.1f µT".format(s.fieldRUt))
            MetricRowM("RMS residual", "%.1f µT".format(s.residualUt))
            MetricRowM("Coverage", "${s.coverageBins} / 32 wedges")
            MetricRowM("Offset |c|", "%.1f µT".format(s.centerMagnitudeUt))
            if (s.centerWarning != MagCalStatus.CenterWarning.OK) {
                Text(
                    "Bias is approaching the chip's ±4915 µT OFFSET-register " +
                        "range — it may not fully subtract. Re-calibrate; if it " +
                        "persists the board may have unusual magnetics.",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFFFFA000),
                )
            }
            MetricRowM("Offset X/Y/Z (raw)", "${s.offsetX} / ${s.offsetY} / ${s.offsetZ}")
            Text(
                "0.15 µT per LSB. The chip subtracts these from every sample once accepted.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
    if (ok) {
        Button(onClick = onVerify, modifier = Modifier.fillMaxWidth()) { Text("Verify") }
        OutlinedButton(onClick = onForceApply, modifier = Modifier.fillMaxWidth()) {
            Text("Save and apply (skip verify)")
        }
        Text(
            "Verify programs the offset into the chip and confirms with a live " +
                "|B| readout — the recommended path.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
    OutlinedButton(onClick = onRetry, modifier = Modifier.fillMaxWidth()) { Text("Retry") }
    OutlinedButton(onClick = onAbort, modifier = Modifier.fillMaxWidth()) {
        Text("Abort", color = MaterialTheme.colorScheme.error)
    }
}

@Composable
private fun VerifyingSection(
    status: MagCalStatus?,
    verifyMin: Float?,
    verifyMax: Float?,
    onAcceptSave: (allGood: Boolean) -> Unit,
    onRetryVerify: () -> Unit,
    onRerunCal: () -> Unit,
    onAbort: () -> Unit,
) {
    val s = status ?: return
    val spread = if (verifyMin != null && verifyMax != null) verifyMax - verifyMin else 0f
    val inBand = (verifyMin ?: 0f) >= MagCalConst.VERIFY_MIN_UT &&
        (verifyMax ?: 0f) <= MagCalConst.VERIFY_MAX_UT
    val tight = spread <= MagCalConst.VERIFY_RANGE_UT
    val coverageMet = s.coverageBins >= MagCalConst.VERIFY_MIN_COVERAGE_BINS
    val samplesMet = s.sampleCount >= MagCalConst.VERIFY_MIN_SAMPLES
    val allGood = verifyMin != null && inBand && tight && coverageMet && samplesMet

    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text(
                if (allGood) "🛡✓ Verifying calibration" else "🛡 Verifying calibration",
                style = MaterialTheme.typography.titleMedium,
                color = if (allGood) Color(0xFF43A047) else MaterialTheme.colorScheme.onSurface,
            )
            Text(
                "Slowly rotate the rocket through every orientation. Live |B| " +
                    "should stay between 20 and 70 µT with a tight spread.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            MetricRowM("Current |B|", "%.1f µT".format(s.instantaneousFieldUt))
            GateRow("Min observed", verifyMin?.let { "%.1f µT".format(it) } ?: "—", verifyMin == null || (verifyMin >= MagCalConst.VERIFY_MIN_UT))
            GateRow("Max observed", verifyMax?.let { "%.1f µT".format(it) } ?: "—", verifyMax == null || (verifyMax <= MagCalConst.VERIFY_MAX_UT))
            GateRow("Spread", if (verifyMin == null) "—" else "%.1f µT".format(spread), verifyMin == null || tight)
            GateRow("Rotation coverage", "${s.coverageBins} / 32 (need ≥ ${MagCalConst.VERIFY_MIN_COVERAGE_BINS})", coverageMet)
            GateRow("Samples", "${s.sampleCount} / ${MagCalConst.VERIFY_MIN_SAMPLES}+", samplesMet)
        }
    }
    Button(onClick = { onAcceptSave(allGood) }, modifier = Modifier.fillMaxWidth()) {
        Text(if (allGood) "Accept and Save" else "Save anyway")
    }
    Text(
        if (allGood) {
            "All checks green — Accept and Save commits the cal to flight-computer memory."
        } else {
            "Keep rotating until every row is green. The gates are advisory — Save anyway skips them."
        },
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
    OutlinedButton(onClick = onRetryVerify, modifier = Modifier.fillMaxWidth()) { Text("Retry verification") }
    OutlinedButton(onClick = onRerunCal, modifier = Modifier.fillMaxWidth()) { Text("Re-run calibration") }
    OutlinedButton(onClick = onAbort, modifier = Modifier.fillMaxWidth()) {
        Text("Abort (restore prior cal)", color = MaterialTheme.colorScheme.error)
    }
}

@Composable
private fun AppliedSection(status: MagCalStatus?) {
    val s = status ?: return
    Card {
        Column(
            Modifier.fillMaxWidth().padding(20.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Text("✅ Saved", style = MaterialTheme.typography.headlineSmall, color = Color(0xFF43A047))
            Text(
                "Calibration written to flight-computer memory.\n" +
                    "Earth field locked at %.1f µT, residual %.1f µT."
                        .format(s.fieldRUt, s.residualUt),
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

/**
 * 32 firmware coverage bins as a 4×8 unrolled grid (bit i → row i/8,
 * col i%8): green covered, amber partial, outline uncovered.
 */
@Composable
private fun CoverageGrid(coverageMask: Long, partialMask: Long) {
    Canvas(Modifier.fillMaxWidth().height(120.dp)) {
        val cols = 8
        val rows = 4
        val gap = 3.dp.toPx()
        val cw = (size.width - gap * (cols - 1)) / cols
        val ch = (size.height - gap * (rows - 1)) / rows
        for (bit in 0 until 32) {
            val row = bit / cols
            val col = bit % cols
            val covered = (coverageMask shr bit) and 1L == 1L
            val partial = (partialMask shr bit) and 1L == 1L
            val color = when {
                covered -> Color(0xFF43A047)
                partial -> Color(0xFFFBC02D)
                else -> Color(0xFF757575).copy(alpha = 0.25f)
            }
            drawRect(
                color,
                topLeft = Offset(col * (cw + gap), row * (ch + gap)),
                size = Size(cw, ch),
            )
        }
    }
}

/** Dominant gravity axis from the low-g accel (≥4 m/s², iOS threshold). */
private fun dominantAxis(ax: Float?, ay: Float?, az: Float?): String? {
    if (ax == null || ay == null || az == null) return null
    val xa = abs(ax)
    val ya = abs(ay)
    val za = abs(az)
    val peak = max(xa, max(ya, za))
    if (peak < 4.0f) return null
    return when (peak) {
        xa -> if (ax > 0) "+X" else "−X"
        ya -> if (ay > 0) "+Y" else "−Y"
        else -> if (az > 0) "+Z" else "−Z"
    }
}

private fun rejectMessage(code: MagCalRejectCode): String = when (code) {
    MagCalRejectCode.OK -> "Fit accepted"
    MagCalRejectCode.R_TOO_LOW -> "Fitted field too low (< 20 µT) — likely a bad tumble"
    MagCalRejectCode.R_TOO_HIGH -> "Fitted field too high (> 80 µT) — magnetic interference?"
    MagCalRejectCode.HIGH_RESIDUAL -> "Poor sphere fit (high residual) — retry the tumble"
    MagCalRejectCode.LOW_COVERAGE -> "Not enough orientation coverage — keep tumbling"
    MagCalRejectCode.VERIFY_FAILED -> "Verification failed"
    MagCalRejectCode.VERIFY_TOO_HIGH -> "Verify: |B| too high"
    MagCalRejectCode.VERIFY_TOO_LOW -> "Verify: |B| too low"
    MagCalRejectCode.VERIFY_RANGE_WIDE -> "Verify: |B| spread too wide"
    MagCalRejectCode.VERIFY_LOW_COVERAGE -> "Verify: not enough rotation coverage"
    MagCalRejectCode.VERIFY_FEW_SAMPLES -> "Verify: not enough samples"
}

@Composable
private fun MetricRowM(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace)
    }
}

@Composable
private fun GateRow(label: String, value: String, pass: Boolean) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(
            value,
            fontFamily = FontFamily.Monospace,
            color = if (pass) Color(0xFF43A047) else Color(0xFFE53935),
        )
    }
}
