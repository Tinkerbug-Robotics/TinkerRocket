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
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
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
import com.tinkerbug.tinkerrocket.session.GuidanceResult
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor

/**
 * The ONE shared orbit-camera scene (plan §1: all SceneKit views project to
 * Compose Canvas through a single component).  Consumers: the 3D flight
 * trajectory (EKF track) and DriftCast 3D (planned descent).  All camera and
 * projection math is [Trajectory3D] in `:core:protocol`, where the axis
 * conventions are pinned by tests — a mirrored scene looks plausible, which
 * is exactly why the math doesn't live in this file.
 *
 * Gestures: one-finger drag orbits (pitch clamped 5°–85°), pinch zooms
 * (0.3×–8× extent), double-tap resets to the iOS-parity initial framing
 * (broadside to the main displacement, 1.8× extent, 15° elevation).
 */
internal data class SceneMarker(
    val pos: V3,
    val color: Color,
    val label: String,
    val sublabel: String? = null,
    val radiusDp: Double = 5.0,
)

internal data class SceneLine(
    val a: V3,
    val b: V3,
    val color: Color,
    val widthDp: Double = 1.5,
)

@Composable
internal fun OrbitSceneCanvas(
    track: List<V3>,
    trackColor: (Float) -> Color,
    markers: List<SceneMarker>,
    extraLines: List<SceneLine> = emptyList(),
    /** Included in framing (extent + camera) but NOT drawn as track — e.g.
     *  the launch pad when the drawn track is only the descent. */
    frameAlso: List<V3> = emptyList(),
    emptyMessage: String = "No track data",
) {
    val framing = remember(track, frameAlso) { track + frameAlso }
    val initial = remember(framing) { Trajectory3D.initialCamera(framing) }
    var yaw by remember(framing) { mutableDoubleStateOf(initial.yawRad) }
    var pitch by remember(framing) { mutableDoubleStateOf(initial.pitchRad) }
    var dist by remember(framing) { mutableDoubleStateOf(initial.distance) }

    val extent = remember(framing) { Trajectory3D.extent(framing) }
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
                emptyMessage,
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

        val groundU = framing.minOf { it.u }

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

        // ── Extra lines (boost/drop) under the track ─────────────────────
        for (l in extraLines) {
            val pa = proj(l.a) ?: continue
            val pb = proj(l.b) ?: continue
            drawLine(l.color, pa.o(), pb.o(), strokeWidth = l.widthDp.dp.toPx())
        }

        // ── Track, painter-sorted far→near ───────────────────────────────
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
                trackColor(s.t), s.a, s.b,
                strokeWidth = 3.dp.toPx(), cap = StrokeCap.Round,
            )
        }

        // ── Markers + labels ─────────────────────────────────────────────
        // Labels stack instead of overprinting.  Two markers at the same spot
        // used to stamp their text at the same pixel, and "Launch Pad" over
        // "Landing" renders as unreadable mush.  That is not an edge case
        // here: a reverse drift cast lands you where you launched, so the two
        // coincide on every normal run, and a bench log or a pad abort does
        // the same to Apogee/Landing on the flight trajectory.
        val placed = mutableListOf<Rect>()
        fun place(candidate: Rect): Rect {
            var r = candidate
            val step = r.height + 2.dp.toPx()
            // Walk upward until clear.  Bounded: markers are few, and running
            // off the top is better than an illegible pile.
            var guard = 0
            while (placed.any { it.overlaps(r) } && guard++ < markers.size * 2) {
                r = r.translate(0f, -step)
            }
            placed += r
            return r
        }

        for (m in markers) {
            val pp = proj(m.pos) ?: continue
            val r = m.radiusDp.dp.toPx()
            drawCircle(m.color, r, pp.o())
            val t = textMeasurer.measure(m.label, TextStyle(fontSize = 11.sp, color = labelColor))
            val slot = place(
                Rect(
                    Offset(pp.x.toFloat() - t.size.width / 2, pp.y.toFloat() - r - t.size.height - 2),
                    Size(t.size.width.toFloat(), t.size.height.toFloat()),
                ),
            )
            drawText(t, topLeft = slot.topLeft)
            m.sublabel?.let { sub ->
                val st = textMeasurer.measure(sub, TextStyle(fontSize = 10.sp, color = labelColor))
                val subSlot = place(
                    Rect(
                        Offset(pp.x.toFloat() - st.size.width / 2, pp.y.toFloat() + 8.dp.toPx()),
                        Size(st.size.width.toFloat(), st.size.height.toFloat()),
                    ),
                )
                drawText(st, topLeft = subSlot.topLeft)
            }
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

// ── Flight-trajectory wrapper (EKF track from the CSV) ───────────────────

@Composable
fun Trajectory3DCanvas(data: FlightCsvData) {
    val track = remember(data) {
        Trajectory3D.downsample(ekfTrack(data).map { V3(it.first, it.second, it.third) }, 700)
    }
    val lm = remember(track) { Trajectory3D.landmarks(track) }
    val groundU = remember(track) { track.minOfOrNull { it.u } ?: 0.0 }

    val markers = lm?.let {
        listOf(
            SceneMarker(it.launch, Color(0xFFFF6B6B), "Launch"),
            SceneMarker(it.landing, Color(0xFF52CF66), "Landing"),
            SceneMarker(
                it.apogee, Color(0xFF4DABF7), "Apogee",
                sublabel = "%.0f m AGL".format(it.apogee.u - groundU), radiusDp = 6.0,
            ),
        )
    } ?: emptyList()
    val extra = lm?.let {
        listOf(SceneLine(
            it.apogee, V3(it.apogee.e, it.apogee.n, groundU),
            Color(0xFF4DABF7).copy(alpha = 0.25f),
        ))
    } ?: emptyList()

    OrbitSceneCanvas(
        track = track,
        trackColor = ::trajectoryAltitudeColor,
        markers = markers,
        extraLines = extra,
        emptyMessage = "No EKF position data in this log",
    )
}

// ── DriftCast wrapper (planned descent from the guidance result) ─────────

/**
 * DriftCast 3D (Phase 9, iOS `Trajectory3DView` port): launch pad, the
 * apogee guidance point with its AGL callout, the wind-driven descent track
 * (purple, constant color — altitude is not the story here, the wind path
 * is), the straight boost line, and the verified landing.  Coordinates are
 * local ENU meters around the launch pad, same earth model as the planner's
 * own forwardProject.
 */
@Composable
fun DriftCast3DCanvas(r: GuidanceResult) {
    val apogeeM = r.apogeeFt * 0.3048
    fun enu(lat: Double, lon: Double, altM: Double): V3 {
        val rad = Math.PI / 180.0
        val earth = 6_371_000.0
        return V3(
            earth * cos(r.launchLat * rad) * (lon - r.launchLon) * rad,
            earth * (lat - r.launchLat) * rad,
            altM,
        )
    }

    val track = remember(r) {
        r.descentTrack.map { enu(it.lat, it.lon, it.altAglFt * 0.3048) }
    }
    val launch = V3(0.0, 0.0, 0.0)
    val guidance = enu(r.guidanceLat, r.guidanceLon, apogeeM)
    val landing = enu(r.forwardLandingLat, r.forwardLandingLon, 0.0)

    OrbitSceneCanvas(
        track = track,                 // descent only — purple
        frameAlso = listOf(launch),    // pad frames the scene, boost is an extraLine
        trackColor = { Color(0xFF9775FA) },
        markers = listOf(
            SceneMarker(launch, Color(0xFFFF6B6B), "Launch Pad"),
            SceneMarker(landing, Color(0xFF52CF66), "Landing"),
            SceneMarker(
                guidance, Color(0xFF4DABF7), "Guidance",
                sublabel = "%.0f m AGL".format(apogeeM), radiusDp = 6.0,
            ),
        ),
        extraLines = listOf(
            SceneLine(launch, guidance, Color(0xFF4DABF7).copy(alpha = 0.5f), widthDp = 2.0),
            SceneLine(guidance, V3(guidance.e, guidance.n, 0.0),
                Color(0xFF4DABF7).copy(alpha = 0.25f), widthDp = 1.0),
        ),
        emptyMessage = "No descent track",
    )
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
