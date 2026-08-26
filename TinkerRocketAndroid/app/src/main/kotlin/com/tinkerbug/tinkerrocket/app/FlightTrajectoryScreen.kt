package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.protocol.FlightCsvData
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import com.tinkerbug.tinkerrocket.protocol.flightTracks
import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.V3

/**
 * FlightTrajectory 2D (plan §1: the SceneKit 3D view projects to a 2D
 * Canvas; the orbit-camera 3D version trails to Phase 9): top-down EKF
 * ground track (Position East vs North), north-up with equal-aspect
 * scaling, path colored by altitude (blue ground → red apogee), launch
 * (green) and landing (red) markers, and a round-number scale bar.
 */
@Composable
fun TrajectoryCanvas(data: FlightCsvData) {
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    // BOTH position solutions — shared with the 3D view (#838 item 3).
    val tracks = remember(data) { flightTracks(data) }
    // Markers and the altitude ramp follow the primary solution: the EKF
    // where it exists, the GNSS track on a base-station LoRa log, which has
    // no EKF columns at all.
    val track = tracks.primary

    val textMeasurer = rememberTextMeasurer()
    val labelColor = MaterialTheme.colorScheme.onSurfaceVariant

    Canvas(Modifier.fillMaxSize().padding(12.dp)) {
        if (tracks.isEmpty) {
            val msg = textMeasurer.measure(
                // Was "No EKF position data in this log" — technically true of
                // every base-station LoRa log, and useless: those logs carry a
                // perfectly good lat/lon track, which iOS drew and this screen
                // refused to (#838 item 3).
                "No position data in this log",
                TextStyle(fontSize = 12.sp, color = labelColor),
            )
            drawText(
                msg,
                topLeft = Offset((size.width - msg.size.width) / 2, size.height / 2),
            )
            return@Canvas
        }

        // Extents span BOTH tracks — scaling to one clips the other, and the
        // gap between them is the thing worth seeing.
        val extent = tracks.all
        val minE = extent.minOf { it.e }
        val maxE = extent.maxOf { it.e }
        val minN = extent.minOf { it.n }
        val maxN = extent.maxOf { it.n }
        val minU = track.minOf { it.u }
        val maxU = track.maxOf { it.u }

        val spanE = max(maxE - minE, 1.0)
        val spanN = max(maxN - minN, 1.0)
        // Equal aspect: meters map to the same pixels on both axes.
        val scale = min(size.width / spanE, size.height / spanN).toFloat() * 0.9f
        val cx = size.width / 2
        val cy = size.height / 2
        val midE = (minE + maxE) / 2
        val midN = (minN + maxN) / 2

        fun px(e: Double, n: Double): Offset = Offset(
            cx + ((e - midE) * scale).toFloat(),
            cy - ((n - midN) * scale).toFloat(), // north up
        )

        // The OTHER solution first, so the primary draws on top of it: a thin
        // muted line, visually secondary but present.  Only when both exist —
        // on a LoRa log the GNSS track IS the primary and drawing it twice
        // would just thicken it.
        val secondary = if (tracks.ekf.size >= 2 && tracks.gnss.size >= 2) tracks.gnss else emptyList()
        for (i in 1 until secondary.size) {
            val a = secondary[i - 1]
            val b = secondary[i]
            drawLine(
                color = labelColor.copy(alpha = 0.55f),
                start = px(a.e, a.n),
                end = px(b.e, b.n),
                strokeWidth = 1.5.dp.toPx(),
                cap = StrokeCap.Round,
            )
        }

        // Path segments colored by altitude (blue ground → red apogee).
        val uSpan = max(maxU - minU, 1e-6)
        for (i in 1 until track.size) {
            val p0 = track[i - 1]
            val p1 = track[i]
            val t = ((p1.u - minU) / uSpan).toFloat().coerceIn(0f, 1f)
            drawLine(
                color = trajectoryAltitudeColor(t),
                start = px(p0.e, p0.n),
                end = px(p1.e, p1.n),
                strokeWidth = 3.dp.toPx(),
                cap = StrokeCap.Round,
            )
        }

        // Launch (red) + landing (green) markers, matching iOS's 2D map pins
        // (.systemRed / .systemGreen, FlightTrajectoryView.swift:189/:195) via
        // tokens.  Was green/red here, the opposite of iOS: a field-confusion
        // hazard with two phones out.
        //
        // The 2D/3D split is deliberate and easy to undo by accident: iOS's 3D
        // SceneKit views use hand-tuned literals against a near-black backdrop,
        // its 2D views use system colors.  This is 2D, so it takes roles; the
        // scene literals in Trajectory3DCanvas stay hardcoded.
        drawCircle(tr.launchSite, 6.dp.toPx(), px(track.first().e, track.first().n))
        drawCircle(
            tr.landingSite, 6.dp.toPx(),
            px(track.last().e, track.last().n),
        )

        // The GNSS landing, hollow, when the two solutions actually differ.
        // #741 measured them 81 m apart on the CENJARS flight — far enough to
        // send someone to the wrong end of a field, and invisible until both
        // were drawn on one view.
        if (secondary.isNotEmpty()) {
            val gLast = secondary.last()
            drawCircle(
                labelColor, 6.dp.toPx(), px(gLast.e, gLast.n),
                style = Stroke(width = 2.dp.toPx()),
            )
            val legend = textMeasurer.measure(
                "━ EKF   ┄ GNSS",
                TextStyle(fontSize = 10.sp, color = labelColor),
            )
            drawText(legend, topLeft = Offset(8.dp.toPx(), 4.dp.toPx()))
        }

        // Scale bar: a round meter length ≈ a third of the width.
        val targetM = (size.width / 3f) / scale
        val niceM = niceRound(targetM.toDouble())
        val barPx = (niceM * scale).toFloat()
        val y0 = size.height - 10.dp.toPx()
        drawLine(labelColor, Offset(8.dp.toPx(), y0), Offset(8.dp.toPx() + barPx, y0), strokeWidth = 2.dp.toPx())
        val barLabel = textMeasurer.measure(
            if (niceM >= 1000) "%.0f km".format(niceM / 1000) else "%.0f m".format(niceM),
            TextStyle(fontSize = 10.sp, color = labelColor),
        )
        drawText(barLabel, topLeft = Offset(8.dp.toPx(), y0 - barLabel.size.height - 2.dp.toPx()))

        // North-up marker.
        val nLabel = textMeasurer.measure("N ↑", TextStyle(fontSize = 11.sp, color = labelColor))
        drawText(nLabel, topLeft = Offset(size.width - nLabel.size.width - 4.dp.toPx(), 4.dp.toPx()))
    }
}


/** 1/2/5 × 10ⁿ round-down for the scale bar. */
private fun niceRound(v: Double): Double {
    if (v <= 0) return 1.0
    val exp = kotlin.math.floor(kotlin.math.log10(v))
    val base = 10.0.pow(exp)
    val m = v / base
    return when {
        m >= 5 -> 5 * base
        m >= 2 -> 2 * base
        else -> base
    }
}
