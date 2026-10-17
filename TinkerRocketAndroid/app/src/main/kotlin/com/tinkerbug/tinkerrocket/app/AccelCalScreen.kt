package com.tinkerbug.tinkerrocket.app

import androidx.activity.compose.BackHandler
import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.tween
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
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.CheckCircle
import androidx.compose.material.icons.filled.RadioButtonUnchecked
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearProgressIndicator
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
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.session.AccelCalReadout
import com.tinkerbug.tinkerrocket.session.AccelSixFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.DeviceSession
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch

/**
 * The six-face accelerometer calibration (#1629) — the port of iOS
 * AccelCalibrationView, section for section and word for word
 * ([AccelCalReadout] holds the words).  A dashboard tool beside the mag and
 * pad calibrations, also opened from Settings → Calibration as on iOS.
 *
 * tools/accel_cal_six_face.py's flow: read the stored offset (77), measure
 * the faces in any order (78: a 3 s window each, the face told from the
 * reading), solve, save (76) and read it back (77).  The rules and the
 * numbers are [AccelSixFace]'s; the round trips are [DeviceSession]'s
 * readAccelCal / measureAccelFace / applyAccelCal.  This screen is the
 * checklist, the buttons and the report.
 *
 * The offset is a property of the board, stored on it.  Nothing here touches
 * the rocket profile, and nothing pushes it on connect: this screen is the one
 * place that writes it.
 */
@Composable
fun AccelCalScreen(
    session: DeviceSession,
    deviceKey: String,
    onBack: () -> Unit,
) {
    val tr = TrTheme.colors
    val scope = rememberCoroutineScope()
    val connected by session.isConnected.collectAsState()
    val status by session.accelCalStatus.collectAsState()
    val simLaunched by session.simLaunched.collectAsState()
    // The gate's telemetry inputs at the rate they change, not at frame rate.
    val gate by remember(session) {
        session.telemetry.map { AccelCalGate(it.pwrPinOn, it.state, it.fcBootStepRaw != null) }
            .distinctUntilChanged()
    }.collectAsState(
        initial = session.telemetry.value.let { AccelCalGate(it.pwrPinOn, it.state, it.fcBootStepRaw != null) },
    )
    val access = AccelSixFace.access(
        connected, session.isBaseStation, gate.pwrPinOn, gate.state, gate.booting, simLaunched,
    )
    val canRun = access == AccelSixFace.Access.READY

    // The checklist is the board's, so it survives a reconnect (a new session
    // for the same device) but not a different rocket.
    var faces by remember(deviceKey) { mutableStateOf<Map<Face, AccelCalStatus>>(emptyMap()) }
    var lastFace by remember(deviceKey) { mutableStateOf<AccelFaceReport?>(null) }
    var lastWrite by remember(deviceKey) { mutableStateOf<AccelSixFace.WriteReport?>(null) }
    // One round trip at a time: the answers carry no request id.
    var op by remember { mutableStateOf<AccelCalOp?>(null) }
    var readUnanswered by remember { mutableStateOf(false) }
    var windowOpen by remember { mutableStateOf(false) }
    var confirmClear by remember { mutableStateOf(false) }
    var opened by remember { mutableStateOf(false) }

    fun read() {
        if (op != null) return
        op = AccelCalOp.READING
        readUnanswered = false
        scope.launch {
            try {
                readUnanswered = session.readAccelCal() == null
            } finally {
                op = null
            }
        }
    }

    // iOS: the read goes out on opening when the gate is open, and again when
    // the gate opens later with nothing read yet (the rocket came up, or it
    // reconnected, with the screen open).
    LaunchedEffect(session, canRun) {
        if (canRun && (!opened || session.accelCalStatus.value == null)) read()
        opened = true
    }

    fun measure() {
        if (op != null) return
        op = AccelCalOp.MEASURING
        windowOpen = false
        lastWrite = null   // the faces are changing under it
        scope.launch {
            try {
                val m = session.measureAccelFace(onWindowOpen = { windowOpen = true })
                lastFace = if (m is AccelSixFace.Measurement.Finished) {
                    val v = AccelSixFace.judge(m.window, faces.keys)
                    if (v is AccelSixFace.Verdict.Accepted) faces = faces + (v.face to m.window)
                    AccelFaceReport(AccelCalReadout.verdict(v), AccelCalReadout.detail(m.window))
                } else {
                    AccelCalReadout.notMeasured(m)?.let { AccelFaceReport(it, detail = null) }
                }
            } finally {
                op = null
                windowOpen = false
            }
        }
    }

    val solution = remember(faces) {
        if (faces.size == Face.entries.size) {
            AccelSixFace.solve(faces.mapValues { (_, w) -> w.faceMean.map { it.toDouble() } })
        } else {
            null
        }
    }

    /** A save ([clear] false, [sent] the offset) or a clear: 76, then the read-back. */
    fun write(clear: Boolean, sent: List<Float>) {
        if (op != null) return
        op = if (clear) AccelCalOp.CLEARING else AccelCalOp.SAVING
        lastWrite = null
        scope.launch {
            try {
                val a = session.applyAccelCal(sent[0], sent[1], sent[2])
                lastWrite = AccelSixFace.WriteReport(clear, sent, a.final)
            } finally {
                op = null
            }
        }
    }

    /** After a write that got no answer: read it back again. */
    fun readItBack(w: AccelSixFace.WriteReport) {
        if (op != null) return
        op = if (w.isClear) AccelCalOp.CLEARING else AccelCalOp.SAVING
        lastWrite = null
        scope.launch {
            try {
                lastWrite = w.copy(readBack = session.readAccelCal())
            } finally {
                op = null
            }
        }
    }

    fun startOver() {
        if (op != null) return
        faces = emptyMap()
        lastFace = null
        lastWrite = null
    }

    // The 3 s the board must be held still, from the window's MEASURING frame.
    val hold = remember { Animatable(0f) }
    LaunchedEffect(windowOpen) {
        hold.snapTo(0f)
        if (windowOpen) {
            hold.animateTo(1f, tween(AccelSixFace.FACE_WINDOW_MS.toInt(), easing = LinearEasing))
        }
    }

    BackHandler { onBack() }

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("← Dashboard") }
            Text(AccelCalReadout.TITLE, style = MaterialTheme.typography.titleLarge)
        }

        // ── Why ─────────────────────────────────────────────────────────
        Section(AccelCalReadout.WHY_TITLE) {
            Caption(AccelCalReadout.WHY_TILT)
            Caption(AccelCalReadout.WHY_FACES)
        }

        // ── Stored Offset ───────────────────────────────────────────────
        Section(AccelCalReadout.STORED_TITLE) {
            val s = status
            when {
                s != null -> {
                    if (s.stored) {
                        for (i in 0..2) ValueRow(AXIS_NAMES[i], AccelCalReadout.mg(s.offset[i].toDouble()))
                    }
                    Text(
                        AccelCalReadout.use(s),
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                op == AccelCalOp.READING -> Busy(AccelCalReadout.READING)
                readUnanswered -> {
                    ReadoutText(AccelCalReadout.Line(AccelCalReadout.NO_READ, AccelCalReadout.Tone.WARN))
                    TextButton(onClick = ::read, enabled = canRun && op == null) {
                        Text(AccelCalReadout.READ_AGAIN)
                    }
                }
                else -> Text(
                    AccelCalReadout.NOT_READ,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            AccelCalReadout.blocked(access)?.let {
                Text(it, style = MaterialTheme.typography.bodySmall, color = tr.statusWarn)
            }
        }

        // ── Six Faces ───────────────────────────────────────────────────
        Section(AccelCalReadout.FACES_TITLE) {
            Caption(AccelCalReadout.FACES_HOW)
            Face.entries.forEach { FaceRow(it, faces[it]) }
            if (op == AccelCalOp.MEASURING) {
                if (windowOpen) {
                    Text(
                        if (hold.value < 1f) AccelCalReadout.HOLD_STILL else AccelCalReadout.FINISHING,
                        style = MaterialTheme.typography.bodyMedium,
                    )
                    LinearProgressIndicator(progress = { hold.value }, modifier = Modifier.fillMaxWidth())
                } else {
                    Busy(AccelCalReadout.STARTING)
                }
            } else {
                Button(onClick = ::measure, enabled = canRun && op == null, modifier = Modifier.fillMaxWidth()) {
                    Text(
                        if (faces.size == Face.entries.size) AccelCalReadout.MEASURE_AGAIN
                        else AccelCalReadout.MEASURE,
                    )
                }
                lastFace?.let { r ->
                    ReadoutText(r.line)
                    r.detail?.let { MonoCaption(it) }
                }
            }
            if (faces.isNotEmpty()) {
                TextButton(onClick = ::startOver, enabled = op == null) { Text(AccelCalReadout.START_OVER) }
            }
        }

        // ── Offset, once all six are in ─────────────────────────────────
        solution?.let { s ->
            Section(AccelCalReadout.RESULT_TITLE) {
                for (i in 0..2) {
                    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                        Text(AXIS_NAMES[i], style = MaterialTheme.typography.bodyLarge, modifier = Modifier.weight(1f))
                        Column(horizontalAlignment = Alignment.End) {
                            Text(
                                AccelCalReadout.mg(s.offset[i]),
                                style = MaterialTheme.typography.bodyLarge,
                                fontFamily = FontFamily.Monospace,
                            )
                            MonoCaption(AccelCalReadout.scale(s, i))
                        }
                    }
                }
                Caption(AccelCalReadout.RESIDUALS)
                for (axis in 0..2) {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                        for (up in listOf(1, -1)) {
                            val face = Face.of(axis, up)
                            Text(face.label, style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
                            Text(
                                AccelCalReadout.mg(s.residual.getValue(face)),
                                style = MaterialTheme.typography.bodySmall,
                                fontFamily = FontFamily.Monospace,
                                textAlign = TextAlign.End,
                                modifier = Modifier.weight(1f),
                            )
                        }
                    }
                }
                AccelCalReadout.tooLarge(s)?.let { ReadoutText(it) }
                AccelCalReadout.residualNote(s)?.let { ReadoutText(it, small = true) }
                if (op == AccelCalOp.SAVING) {
                    Busy(AccelCalReadout.SAVING)
                } else {
                    Button(
                        onClick = { write(clear = false, sent = s.wireOffset) },
                        enabled = canRun && op == null && s.saveAllowed,
                        modifier = Modifier.fillMaxWidth(),
                    ) { Text(AccelCalReadout.SAVE) }
                }
                lastWrite?.takeIf { !it.isClear }?.let { w ->
                    WriteReportRows(w, canReadBack = canRun && op == null) { readItBack(w) }
                }
                Caption(AccelCalReadout.RESULT_FOOTER)
            }
        }

        // ── Clear: out of the way, and behind a confirmation ────────────
        Section(title = null) {
            if (op == AccelCalOp.CLEARING) {
                Busy(AccelCalReadout.CLEARING)
            } else {
                TextButton(
                    onClick = { confirmClear = true },
                    enabled = canRun && op == null && status?.stored == true,
                    colors = ButtonDefaults.textButtonColors(contentColor = tr.statusBad),
                ) { Text(AccelCalReadout.CLEAR) }
            }
            lastWrite?.takeIf { it.isClear }?.let { w ->
                WriteReportRows(w, canReadBack = canRun && op == null) { readItBack(w) }
            }
            Caption(AccelCalReadout.CLEAR_FOOTER)
        }
    }

    if (confirmClear) {
        AlertDialog(
            onDismissRequest = { confirmClear = false },
            title = { Text(AccelCalReadout.CLEAR_CONFIRM_TITLE) },
            text = { Text(AccelCalReadout.CLEAR_CONFIRM_TEXT) },
            confirmButton = {
                TextButton(
                    onClick = {
                        confirmClear = false
                        write(clear = true, sent = listOf(0f, 0f, 0f))
                    },
                    colors = ButtonDefaults.textButtonColors(contentColor = tr.statusBad),
                ) { Text(AccelCalReadout.CLEAR_CONFIRM) }
            },
            dismissButton = {
                TextButton(onClick = { confirmClear = false }) { Text(AccelCalReadout.CANCEL) }
            },
        )
    }
}

private val AXIS_NAMES = listOf("X", "Y", "Z")

/** The round trip in flight; the screen runs one at a time. */
private enum class AccelCalOp { READING, MEASURING, SAVING, CLEARING }

/** The gate's telemetry inputs: power, state, and whether a boot step is still reported. */
private data class AccelCalGate(val pwrPinOn: Boolean, val state: String, val booting: Boolean)

/** The last face measurement: its result line, and the window's numbers when there was one. */
private data class AccelFaceReport(val line: AccelCalReadout.Line, val detail: String?)

/** One of the screen's cards, its title inside like Settings' sections. */
@Composable
private fun Section(title: String?, content: @Composable () -> Unit) {
    Card {
        Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            title?.let { Text(it, style = MaterialTheme.typography.titleMedium) }
            content()
        }
    }
}

/** One checklist row: the face, what it means on the board, and its tilt once captured. */
@Composable
private fun FaceRow(face: Face, window: AccelCalStatus?) {
    val tr = TrTheme.colors
    val done = window != null
    Row(
        Modifier.fillMaxWidth().padding(vertical = 2.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Icon(
            if (done) Icons.Filled.CheckCircle else Icons.Filled.RadioButtonUnchecked,
            contentDescription = if (done) "Measured" else "Not measured",
            tint = if (done) tr.statusOk else MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Column(Modifier.padding(start = 12.dp).weight(1f)) {
            Text(face.label, style = MaterialTheme.typography.bodyMedium)
            Caption(AccelCalReadout.howTo(face))
        }
        window?.let { w ->
            MonoCaption(AccelCalReadout.captured(AccelSixFace.identify(w.faceMean.map { it.toDouble() }).tiltDeg))
        }
    }
}

/** A label and its value, the value at the end: iOS's LabeledContent. */
@Composable
private fun ValueRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(label, style = MaterialTheme.typography.bodyLarge, modifier = Modifier.weight(1f))
        Text(value, style = MaterialTheme.typography.bodyLarge, fontFamily = FontFamily.Monospace)
    }
}

/** A save's or a clear's outcome; with no answer, a way to read it back. */
@Composable
private fun WriteReportRows(w: AccelSixFace.WriteReport, canReadBack: Boolean, onReadBack: () -> Unit) {
    ReadoutText(AccelCalReadout.write(w))
    AccelCalReadout.readBack(w)?.let { MonoCaption(it) }
    if (w.readBack == null) {
        TextButton(onClick = onReadBack, enabled = canReadBack) { Text(AccelCalReadout.READ_IT_BACK) }
    }
}

@Composable
private fun ReadoutText(line: AccelCalReadout.Line, small: Boolean = false) {
    val tr = TrTheme.colors
    Text(
        line.text,
        style = if (small) MaterialTheme.typography.bodySmall else MaterialTheme.typography.bodyMedium,
        color = when (line.tone) {
            AccelCalReadout.Tone.OK -> tr.statusOk
            AccelCalReadout.Tone.WARN -> tr.statusWarn
            AccelCalReadout.Tone.BAD -> tr.statusBad
            AccelCalReadout.Tone.NORMAL -> MaterialTheme.colorScheme.onSurface
        },
    )
}

@Composable
private fun Caption(text: String) {
    Text(text, style = MaterialTheme.typography.bodySmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
}

@Composable
private fun MonoCaption(text: String) {
    Text(
        text,
        style = MaterialTheme.typography.bodySmall,
        fontFamily = FontFamily.Monospace,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

@Composable
private fun Busy(text: String) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        CircularProgressIndicator(Modifier.size(16.dp), strokeWidth = 2.dp)
        Spacer(Modifier.width(10.dp))
        Text(text, style = MaterialTheme.typography.bodyMedium)
    }
}
