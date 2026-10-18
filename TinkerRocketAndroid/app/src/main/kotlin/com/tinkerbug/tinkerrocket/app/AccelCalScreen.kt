package com.tinkerbug.tinkerrocket.app

import androidx.activity.compose.BackHandler
import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.foundation.shape.RoundedCornerShape
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
import androidx.compose.ui.draw.clip
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.selected
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.session.AccelCalReadout
import com.tinkerbug.tinkerrocket.session.AccelSixFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGWrite
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
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
 * The offset is a property of the board, stored on it.  It is in no rocket
 * profile, and nothing pushes it on connect: this screen is the one place that
 * writes it.
 *
 * The same windows carry the high-g's reading (a flight and out computer that
 * send it): the screen fits it to the low-g ([AccelSixFace.solveHighG]), shows
 * its scale, offset, cross-axis, fit and its own gravity fit's agreement, and
 * saves it after the offset (80, read back with 77).  The pad calibration
 * then keeps the alignment and re-measures only the high-g's bias; a rocket
 * profile holding this board's sensor cal gets the new bias ([syncer]'s
 * adoptHighGBias), so its connect-time push does not put the old one back.
 */
@Composable
fun AccelCalScreen(
    session: DeviceSession,
    deviceKey: String,
    syncer: ActiveRocketSyncer? = null,
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
    // Which face the board is on now, from each telemetry frame — mapped to
    // what the screen shows of it, so it recomposes when that changes (the
    // tilt in whole degrees), not at frame rate.
    val live by remember(session) {
        session.accelLiveFace.map { AccelLive.of(it) }.distinctUntilChanged()
    }.collectAsState(initial = AccelLive.of(AccelSixFace.LiveFace.Waiting))

    // The checklist is the board's, so it survives a reconnect (a new session
    // for the same device) but not a different rocket.
    var faces by remember(deviceKey) { mutableStateOf<Map<Face, AccelCalStatus>>(emptyMap()) }
    var lastFace by remember(deviceKey) { mutableStateOf<AccelFaceReport?>(null) }
    var lastWrite by remember(deviceKey) { mutableStateOf<AccelSixFace.WriteReport?>(null) }
    // The high-g's part of the last save or clear; null when it had none.
    var lastHighGWrite by remember(deviceKey) { mutableStateOf<AccelSixFace.HighGWriteReport?>(null) }
    // The row last tapped: its placement line shows under the checklist.
    var placementFace by remember(deviceKey) { mutableStateOf<Face?>(null) }
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
        placementFace = null
        lastWrite = null   // the faces are changing under it
        lastHighGWrite = null
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

    // The high-g's alignment to the low-g, once all six are in.
    val highGFit = remember(faces, solution) { solution?.let { AccelSixFace.solveHighG(faces, it) } }

    // A high-g alignment is stored on the board.
    val highGStored = status?.highG?.stored == true

    /**
     * A confirmed high-g save: the active profile's sensor cal (this board's)
     * takes the new bias, so the connect-time push keeps it.
     */
    fun highGWritten(r: AccelSixFace.HighGWriteReport) {
        val w = r.write as? HighGWrite.Save ?: return
        if (r.confirmed) syncer?.adoptHighGBias(session.identity.value.unitId, w.bias)
    }

    /**
     * A save ([clear] false, [sent] the offset) or a clear: 76, then the
     * read-back; and with [highG], 80 and its read-back after it.
     */
    fun write(clear: Boolean, sent: List<Float>, highG: HighGWrite?) {
        if (op != null) return
        op = if (clear) AccelCalOp.CLEARING else AccelCalOp.SAVING
        lastWrite = null
        lastHighGWrite = null
        scope.launch {
            try {
                val w = session.writeAccelCal(clear, sent, highG) { offset ->
                    lastWrite = offset
                    if (highG != null && !clear) op = AccelCalOp.SAVING_HIGH_G
                }
                w.highG?.let {
                    lastHighGWrite = it
                    highGWritten(it)
                }
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

    /** After a high-g write that got no answer: read it back again. */
    fun readHighGBack(w: AccelSixFace.HighGWriteReport) {
        if (op != null) return
        op = if (w.write == HighGWrite.Clear) AccelCalOp.CLEARING else AccelCalOp.SAVING_HIGH_G
        lastHighGWrite = null
        scope.launch {
            try {
                val r = w.copy(readBack = session.readAccelCal())
                lastHighGWrite = r
                highGWritten(r)
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
        lastHighGWrite = null
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
                    val h = s.highG
                    if (h != null) {
                        Text(
                            AccelCalReadout.hgAlignment(h),
                            style = MaterialTheme.typography.bodyMedium,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                        if (h.hasBias) {
                            MonoCaption(AccelCalReadout.hgBiasRow(h))
                            Caption(AccelCalReadout.hgSourceRow(h))
                        }
                    } else {
                        Caption(AccelCalReadout.HG_NOT_SENT)
                    }
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
            Caption(AccelCalReadout.FACES_NO_PICK)
            ReadoutText(live.line)
            Face.entries.forEach { face ->
                FaceRow(face, faces[face], isLive = live.face == face, onTap = { placementFace = face })
            }
            placementFace?.let {
                Text(AccelCalReadout.placement(it), style = MaterialTheme.typography.bodyMedium)
            }
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
                    Text(AccelCalReadout.measureLabel(live.onFace, allCaptured = faces.size == Face.entries.size))
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
                Caption(AccelCalReadout.SOLVED_TOGETHER)
                AccelCalReadout.cannotSave(s)?.let { ReadoutText(it) }
                if (op == AccelCalOp.SAVING) {
                    Busy(AccelCalReadout.SAVING)
                } else {
                    Button(
                        onClick = {
                            write(clear = false, sent = s.wireOffset, highG = AccelSixFace.highGAfterSave(highGFit))
                        },
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

        // ── The high-g, aligned to the low-g ────────────────────────────
        highGFit?.let { r ->
            Section(AccelCalReadout.HG_TITLE) {
                if (r is AccelSixFace.HighGResult.Solved) {
                    val h = r.solution
                    for (i in 0..2) {
                        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                            Text(AXIS_NAMES[i], style = MaterialTheme.typography.bodyLarge, modifier = Modifier.weight(1f))
                            Column(horizontalAlignment = Alignment.End) {
                                Text(
                                    AccelCalReadout.hgScale(h, i),
                                    style = MaterialTheme.typography.bodyLarge,
                                    fontFamily = FontFamily.Monospace,
                                )
                                MonoCaption(AccelCalReadout.hgOffset(h, i))
                            }
                        }
                    }
                    MonoCaption(AccelCalReadout.hgCross(h))
                    MonoCaption(AccelCalReadout.hgResidual(h))
                    // Agreeing is said quietly, as iOS's secondary line; a
                    // disagreement in the warn color.
                    val agreement = AccelCalReadout.hgAgreement(h)
                    Text(
                        agreement.text,
                        style = MaterialTheme.typography.bodyMedium,
                        color = if (agreement.tone == AccelCalReadout.Tone.WARN) tr.statusWarn
                        else MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                    ReadoutText(AccelCalReadout.hgVerdict(h))
                } else {
                    ReadoutText(AccelCalReadout.Line(AccelCalReadout.hgUnavailable(r) ?: "", AccelCalReadout.Tone.WARN))
                }
                if (op == AccelCalOp.SAVING_HIGH_G) Busy(AccelCalReadout.HG_SAVING)
                lastHighGWrite?.takeIf { it.write != HighGWrite.Clear }?.let { w ->
                    HighGWriteReportRows(w, canReadBack = canRun && op == null) { readHighGBack(w) }
                }
                Caption(AccelCalReadout.HG_FOOTER)
            }
        }

        // ── Clear: out of the way, and behind a confirmation ────────────
        Section(title = null) {
            if (op == AccelCalOp.CLEARING) {
                Busy(AccelCalReadout.CLEARING)
            } else {
                TextButton(
                    onClick = { confirmClear = true },
                    enabled = canRun && op == null && (status?.stored == true || highGStored),
                    colors = ButtonDefaults.textButtonColors(contentColor = tr.statusBad),
                ) { Text(if (highGStored) AccelCalReadout.CLEAR_ALL else AccelCalReadout.CLEAR) }
            }
            lastWrite?.takeIf { it.isClear }?.let { w ->
                WriteReportRows(w, canReadBack = canRun && op == null) { readItBack(w) }
            }
            lastHighGWrite?.takeIf { it.write == HighGWrite.Clear }?.let { w ->
                HighGWriteReportRows(w, canReadBack = canRun && op == null) { readHighGBack(w) }
            }
            Caption(if (highGStored) AccelCalReadout.CLEAR_ALL_FOOTER else AccelCalReadout.CLEAR_FOOTER)
        }
    }

    if (confirmClear) {
        AlertDialog(
            onDismissRequest = { confirmClear = false },
            title = {
                Text(if (highGStored) AccelCalReadout.CLEAR_ALL_CONFIRM_TITLE else AccelCalReadout.CLEAR_CONFIRM_TITLE)
            },
            text = {
                Text(if (highGStored) AccelCalReadout.CLEAR_ALL_CONFIRM_TEXT else AccelCalReadout.CLEAR_CONFIRM_TEXT)
            },
            confirmButton = {
                TextButton(
                    onClick = {
                        confirmClear = false
                        write(clear = true, sent = listOf(0f, 0f, 0f), highG = AccelSixFace.highGAfterClear(status))
                    },
                    colors = ButtonDefaults.textButtonColors(contentColor = tr.statusBad),
                ) { Text(if (highGStored) AccelCalReadout.CLEAR_ALL_CONFIRM else AccelCalReadout.CLEAR_CONFIRM) }
            },
            dismissButton = {
                TextButton(onClick = { confirmClear = false }) { Text(AccelCalReadout.CANCEL) }
            },
        )
    }
}

private val AXIS_NAMES = listOf("X", "Y", "Z")

/**
 * The round trip in flight; the screen runs one at a time.  A clear is
 * CLEARING through its high-g part too; a save's high-g part is its own, as
 * on iOS, where the offset's Save button comes back (disabled) while the
 * high-g section says it is saving.
 */
private enum class AccelCalOp { READING, MEASURING, SAVING, SAVING_HIGH_G, CLEARING }

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

/** What the screen shows of the live face: its line, its row, and the face to name on Measure. */
private data class AccelLive(
    val line: AccelCalReadout.Line,
    /** The row to highlight: the live face, square or askew. */
    val face: Face?,
    /** The face to name on the Measure button: only while on it (square, or under an exact mounting). */
    val onFace: Face?,
) {
    companion object {
        fun of(l: AccelSixFace.LiveFace): AccelLive = AccelLive(
            AccelCalReadout.live(l),
            l.face,
            when (l) {
                is AccelSixFace.LiveFace.On -> l.face
                is AccelSixFace.LiveFace.OnNoTilt -> l.face
                else -> null
            },
        )
    }
}

/**
 * One checklist row: the face, what it means on the board, its tilt once
 * captured, and — while the board is on it — the app's selection tint and a
 * "Now" tag.  A tap shows how to place the board for it.
 */
@Composable
private fun FaceRow(face: Face, window: AccelCalStatus?, isLive: Boolean, onTap: () -> Unit) {
    val tr = TrTheme.colors
    val accent = MaterialTheme.colorScheme.primary
    val shape = RoundedCornerShape(8.dp)
    val done = window != null
    Row(
        Modifier
            .fillMaxWidth()
            .clip(shape)
            .then(
                if (isLive) Modifier.background(accent.copy(alpha = 0.12f)).border(1.dp, accent, shape)
                else Modifier,
            )
            .semantics { selected = isLive }
            .clickable(onClickLabel = AccelCalReadout.ROW_ACTION, role = Role.Button, onClick = onTap)
            .padding(horizontal = 6.dp, vertical = 4.dp),
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
        if (isLive) {
            Text(
                AccelCalReadout.NOW,
                style = MaterialTheme.typography.labelMedium,
                color = accent,
                modifier = Modifier
                    .padding(start = 8.dp)
                    .background(accent.copy(alpha = 0.15f), RoundedCornerShape(12.dp))
                    .padding(horizontal = 8.dp, vertical = 2.dp),
            )
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

/** A high-g save's or clear's outcome; with no answer, a way to read it back. */
@Composable
private fun HighGWriteReportRows(w: AccelSixFace.HighGWriteReport, canReadBack: Boolean, onReadBack: () -> Unit) {
    ReadoutText(AccelCalReadout.hgWrite(w))
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
