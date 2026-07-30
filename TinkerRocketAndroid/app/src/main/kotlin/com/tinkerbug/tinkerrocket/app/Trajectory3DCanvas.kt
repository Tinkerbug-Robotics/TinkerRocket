package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableDoubleStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.protocol.FlightCsvData
import com.tinkerbug.tinkerrocket.protocol.Trajectory3D
import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.V3
import kotlin.math.ceil
import kotlin.math.floor

/**
 * Orbit-camera 3D flight trajectory — the Phase 9 port of iOS's SceneKit
 * `FlightScene3DView`, rendered per plan §1 as a 2D-projected Compose Canvas
 * (no 3D engine; the value is orientation + shape, not shading).  All camera
 * and projection math is [Trajectory3D] in `:core:protocol`, where the axis
 * conventions are pinned by tests — a mirrored trajectory looks plausible,
 * which is exactly why the math doesn't live in this file.
 *
 * Gestures: one-finger drag orbits (yaw/pitch), pinch zooms, double-tap
 * resets to the iOS-parity initial framing (broadside to the climb, 1.8×
 * extent, 15° elevation).
 *
 * Scene, matching iOS: ground grid, altitude-colored path (painter-sorted
 * far→near), apogee drop line, launch/apogee/landing markers + labels with
 * the apogee altitude callout.
 */
@Composable
fun Trajectory3DCanvas(data: FlightCsvData) {
    val track = remember(data) {
        Trajectory3D.downsample(ekfTrack(data).map { V3(it.first, it.second, it.third) }, 700)
    }
    val initial = remember(track) { Trajectory3D.initialCamera(track) }

    var yaw by remember(track) { mutableDoubleStateOf(initial.yawRad) }
    var pitch by remember(track) { mutableDoubleStateOf(initial.pitchRad) }
    var dist by remember(track) { mutableDoubleStateOf(initial.distance) }

    val extent = remember(track) { Trajectory3D.extent(track) }
    val textMeasurer = rememberTextMeasurer()
    val labelColor = MaterialTheme.colorScheme.onSurfaceVariant
    val gridColor = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.25f)

    Canvas(
        Modifier
            .fillMaxSize()
            .pointerInput(track) {
                detectTransformGestures { _, pan, zoom, _ ->
                    yaw -= pan.x * 0.006
                    pitch = (pitch + pan.y * 0.006)
                        .coerceIn(Math.toRadians(5.0), Math.toRadians(85.0))
                    if (zoom != 0f) {
                        dist = (dist / zoom).coerceIn(extent * 0.3, extent * 8.0)
                    }
                }
            }
            .pointerInput(track) {
                detectTapGestures(onDoubleTap = {
                    yaw = initial.yawRad
                    pitch = initial.pitchRad
                    dist = initial.distance
                })
            },
    ) {
        if (track.size < 2) {
            val msg = textMeasurer.measure(
                "No EKF position data in this log",
                TextStyle(fontSize = 12.sp, color = labelColor),
            )
            drawText(msg, topLeft = Offset((size.width - msg.size.width) / 2, size.height / 2))
            return@Canvas
        }

        val cam = initial.copy(yawRad = yaw, pitchRad = pitch, distance = dist)
        val w = size.width.toDouble()
        val h = size.height.toDouble()
        fun proj(p: V3): Trajectory3D.Projected? = Trajectory3D.project(p, cam, w, h)
        fun Trajectory3D.Projected.o() = Offset(x.toFloat(), y.toFloat())

        val lm = Trajectory3D.landmarks(track)!!
        val groundU = track.minOf { it.u }

        // ── Ground grid ──────────────────────────────────────────────────
        val spacing = Trajectory3D.niceGridSpacing(extent)
        val half = extent * 1.5
        val c = cam.center
        val lines = mutableListOf<Pair<V3, V3>>()
        var g = floor((c.e - half) / spacing) * spacing
        while (g <= ceil((c.e + half) / spacing) * spacing) {
            lines.add(V3(g, c.n - half, groundU) to V3(g, c.n + half, groundU))
            g += spacing
        }
        g = floor((c.n - half) / spacing) * spacing
        while (g <= ceil((c.n + half) / spacing) * spacing) {
            lines.add(V3(c.e - half, g, groundU) to V3(c.e + half, g, groundU))
            g += spacing
        }
        for ((a, b) in lines) {
            val pa = proj(a) ?: continue
            val pb = proj(b) ?: continue
            drawLine(gridColor, pa.o(), pb.o(), strokeWidth = 1f)
        }

        // ── Apogee drop line (under the path) ────────────────────────────
        val apoGround = V3(lm.apogee.e, lm.apogee.n, groundU)
        val pApo = proj(lm.apogee)
        val pApoGround = proj(apoGround)
        if (pApo != null && pApoGround != null) {
            drawLine(
                Color(0xFF64B5F6).copy(alpha = 0.35f), pApo.o(), pApoGround.o(),
                strokeWidth = 1.5.dp.toPx(),
            )
        }

        // ── Path, altitude-colored, painter-sorted far→near ──────────────
        val minU = track.minOf { it.u }
        val maxU = track.maxOf { it.u }
        val uSpan = (maxU - minU).coerceAtLeast(1e-6)
        data class Seg(val a: Offset, val b: Offset, val t: Float, val depth: Double)
        val segs = mutableListOf<Seg>()
        var prev: Trajectory3D.Projected? = proj(track[0])
        for (i in 1 until track.size) {
            val cur = proj(track[i])
            val p0 = prev
            prev = cur
            if (p0 == null || cur == null) continue   // culled vertex splits the line
            val midU = (track[i - 1].u + track[i].u) / 2
            segs.add(Seg(
                p0.o(), cur.o(),
                ((midU - minU) / uSpan).toFloat().coerceIn(0f, 1f),
                (p0.depth + cur.depth) / 2,
            ))
        }
        segs.sortByDescending { it.depth }
        for (s in segs) {
            drawLine(
                trajectoryAltitudeColor(s.t), s.a, s.b,
                strokeWidth = 3.dp.toPx(), cap = StrokeCap.Round,
            )
        }

        // ── Markers + labels (iOS colors: launch red, apogee blue, landing
        // green — the 2D track was aligned to the same palette in this
        // change) ────────────────────────────────────────────────────────
        fun marker(p: V3, color: Color, label: String, r: Float) {
            val pp = proj(p) ?: return
            drawCircle(color, r, pp.o())
            val t = textMeasurer.measure(label, TextStyle(fontSize = 11.sp, color = labelColor))
            drawText(t, topLeft = Offset(pp.x.toFloat() - t.size.width / 2, pp.y.toFloat() - r - t.size.height - 2))
        }
        marker(lm.launch, Color(0xFFFF6B6B), "Launch", 5.dp.toPx().coerceAtLeast(1f))
        marker(lm.landing, Color(0xFF52CF66), "Landing", 5.dp.toPx())
        marker(lm.apogee, Color(0xFF4DABF7), "Apogee", 6.dp.toPx())
        pApo?.let {
            val altText = textMeasurer.measure(
                "%.0f m AGL".format(lm.apogee.u - groundU),
                TextStyle(fontSize = 10.sp, color = labelColor),
            )
            drawText(altText, topLeft = Offset(it.x.toFloat() - altText.size.width / 2, it.y.toFloat() + 8.dp.toPx()))
        }

        // Hint + north cue.
        val hint = textMeasurer.measure(
            "drag to orbit · pinch to zoom · double-tap to reset",
            TextStyle(fontSize = 10.sp, color = labelColor),
        )
        drawText(hint, topLeft = Offset(8.dp.toPx(), size.height - hint.size.height - 6.dp.toPx()))
        val north = proj(V3(c.e, c.n + extent * 1.4, groundU))
        north?.let {
            val nl = textMeasurer.measure("N", TextStyle(fontSize = 12.sp, color = labelColor))
            drawText(nl, topLeft = Offset(it.x.toFloat() - nl.size.width / 2, it.y.toFloat() - nl.size.height / 2))
        }
    }
}

/** Blue ground → red apogee, shared with the 2D track. */
internal fun trajectoryAltitudeColor(t: Float): Color = Color(
    red = 0.12f + (0.90f - 0.12f) * t,
    green = 0.53f + (0.22f - 0.53f) * t,
    blue = 0.90f + (0.21f - 0.90f) * t,
    alpha = 1f,
)

/** Clean finite EKF (E, N, U) triples — the shared source for 2D and 3D. */
internal fun ekfTrack(data: FlightCsvData): List<Triple<Double, Double, Double>> {
    val e = data.columns["Position East (m)"] ?: emptyList()
    val n = data.columns["Position North (m)"] ?: emptyList()
    val u = data.columns["Position Up (m)"]
        ?: data.columns["Pressure Altitude (m)"] ?: emptyList()
    val rows = minOf(e.size, n.size, u.size)
    return (0 until rows).mapNotNull { i ->
        if (e[i].isFinite() && n[i].isFinite() && u[i].isFinite()) {
            Triple(e[i], n[i], u[i])
        } else {
            null
        }
    }
}
