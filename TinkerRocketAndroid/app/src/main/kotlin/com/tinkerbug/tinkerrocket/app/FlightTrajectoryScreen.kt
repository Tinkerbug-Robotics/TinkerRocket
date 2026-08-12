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
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.protocol.FlightCsvData
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

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
    // Clean finite (E, N, U) triples — shared with the 3D view.
    val track = remember(data) { ekfTrack(data) }

    val textMeasurer = rememberTextMeasurer()
    val labelColor = MaterialTheme.colorScheme.onSurfaceVariant

    Canvas(Modifier.fillMaxSize().padding(12.dp)) {
        if (track.size < 2) {
            val msg = textMeasurer.measure(
                "No EKF position data in this log",
                TextStyle(fontSize = 12.sp, color = labelColor),
            )
            drawText(
                msg,
                topLeft = Offset((size.width - msg.size.width) / 2, size.height / 2),
            )
            return@Canvas
        }

        val minE = track.minOf { it.first }
        val maxE = track.maxOf { it.first }
        val minN = track.minOf { it.second }
        val maxN = track.maxOf { it.second }
        val minU = track.minOf { it.third }
        val maxU = track.maxOf { it.third }

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

        // Path segments colored by altitude (blue ground → red apogee).
        val uSpan = max(maxU - minU, 1e-6)
        for (i in 1 until track.size) {
            val (e0, n0, _) = track[i - 1]
            val (e1, n1, u1) = track[i]
            val t = ((u1 - minU) / uSpan).toFloat().coerceIn(0f, 1f)
            drawLine(
                color = trajectoryAltitudeColor(t),
                start = px(e0, n0),
                end = px(e1, n1),
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
        drawCircle(tr.launchSite, 6.dp.toPx(), px(track.first().first, track.first().second))
        drawCircle(
            tr.landingSite, 6.dp.toPx(),
            px(track.last().first, track.last().second),
        )

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
