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
    // Clean finite (E, N, U) triples, EKF position columns.
    val track = remember(data) {
        val e = data.columns["Position East (m)"] ?: emptyList()
        val n = data.columns["Position North (m)"] ?: emptyList()
        val u = data.columns["Position Up (m)"]
            ?: data.columns["Pressure Altitude (m)"] ?: emptyList()
        val rows = minOf(e.size, n.size, u.size)
        (0 until rows).mapNotNull { i ->
            if (e[i].isFinite() && n[i].isFinite() && u[i].isFinite()) {
                Triple(e[i], n[i], u[i])
            } else {
                null
            }
        }
    }

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
                color = lerpColor(Color(0xFF1E88E5), Color(0xFFE53935), t),
                start = px(e0, n0),
                end = px(e1, n1),
                strokeWidth = 3.dp.toPx(),
                cap = StrokeCap.Round,
            )
        }

        // Launch (green) + landing (red) markers.
        drawCircle(Color(0xFF43A047), 6.dp.toPx(), px(track.first().first, track.first().second))
        drawCircle(
            Color(0xFFE53935), 6.dp.toPx(),
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

private fun lerpColor(a: Color, b: Color, t: Float): Color = Color(
    red = a.red + (b.red - a.red) * t,
    green = a.green + (b.green - a.green) * t,
    blue = a.blue + (b.blue - a.blue) * t,
    alpha = 1f,
)

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
