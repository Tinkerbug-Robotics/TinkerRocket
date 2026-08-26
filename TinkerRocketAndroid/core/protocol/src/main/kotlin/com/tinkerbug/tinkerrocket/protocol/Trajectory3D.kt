package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Orbit-camera math for the 3D flight-trajectory view (plan §1: all SceneKit
 * views project to Compose Canvas through ONE shared orbit-camera component;
 * this is that component's math, kept pure-JVM so the projection, culling and
 * camera conventions are testable without a Canvas).
 *
 * Coordinates are local ENU meters.  The EKF solution comes straight from the
 * CSV's `Position East/North/Up (m)`; the GNSS solution is converted into the
 * same frame by [geodeticToEnu].
 *
 * Both are drawn (#838 item 3).  This used to read "same rendered shape,
 * better source" and plot the EKF track alone — but the two solutions are not
 * the same shape: #741 measured them landing 81 m apart on the CENJARS
 * flight, far enough to send someone to the wrong end of a field.  The EKF
 * track IS smoother and does survive a GNSS outage, which is why it stays the
 * primary; it is not a substitute for showing the other one.  A base-station
 * LoRa log has no EKF columns at all, and used to render as the literal text
 * "No EKF position data in this log".
 *
 * Conventions (pinned by Trajectory3DTest):
 *  - Camera azimuth `yawRad` is the direction of the CAMERA from the center,
 *    compass-style (0 = camera due north of center, π = due south, looking
 *    back at the center either way).  `pitchRad` elevates it.
 *  - Screen: +x right, +y down (Compose Canvas).  A point due east of center
 *    appears screen-RIGHT when the camera sits south looking north; higher
 *    altitude appears screen-UP.
 *  - `project` returns null for points at or behind the near plane — a
 *    culled vertex splits the polyline rather than smearing across it.
 */
public object Trajectory3D {

    /** ENU meters. */
    public data class V3(val e: Double, val n: Double, val u: Double)

    public data class Landmarks(
        val launch: V3,
        val apogee: V3,
        val landing: V3,
        val apogeeIndex: Int,
    )

    public fun landmarks(track: List<V3>): Landmarks? {
        if (track.isEmpty()) return null
        var apogeeIndex = 0
        track.forEachIndexed { i, p -> if (p.u > track[apogeeIndex].u) apogeeIndex = i }
        return Landmarks(track.first(), track[apogeeIndex], track.last(), apogeeIndex)
    }

    /** Largest axis span across E/N/U, floored at 100 m (iOS parity). */
    public fun extent(track: List<V3>): Double {
        if (track.isEmpty()) return 100.0
        val spanE = track.maxOf { it.e } - track.minOf { it.e }
        val spanN = track.maxOf { it.n } - track.minOf { it.n }
        val spanU = track.maxOf { it.u } - track.minOf { it.u }
        return max(max(spanE, spanN), max(spanU, 100.0))
    }

    public fun center(track: List<V3>): V3 {
        if (track.isEmpty()) return V3(0.0, 0.0, 0.0)
        return V3(
            (track.maxOf { it.e } + track.minOf { it.e }) / 2,
            (track.maxOf { it.n } + track.minOf { it.n }) / 2,
            (track.maxOf { it.u } + track.minOf { it.u }) / 2,
        )
    }

    /**
     * Stride-downsample to at most [maxCount] points, always keeping the
     * first, the last, AND the apogee sample — the one point a stride can
     * shave that a human will notice missing (the chart LOD learned the same
     * lesson; its test pins apogee surviving LTTB).
     */
    /**
     * Mean Earth radius, metres — the value the firmware's own forward
     * conversion uses, so a track converted here lands in the same metric as
     * the EKF track it is drawn beside.
     */
    private const val R_EARTH_M: Double = 6_371_000.0

    /**
     * Convert a geodetic track to the local ENU frame the EKF track lives in
     * (#838 item 3).
     *
     * The two solutions genuinely disagree — #741 measured the nav filter and
     * GNSS landing **81 m apart** on the CENJARS flight after a 1.9 s boost
     * satellite outage that the filter dead-reckoned through and never
     * reconverged from. The standing rule from that investigation is to draw
     * both and say so, never to average them or silently prefer one.
     *
     * ANCHORING. The EKF's ENU origin is a running mean of pad GNSS fixes
     * frozen 120 s after the first fix, and the firmware writes it to no frame
     * and no sidecar (#741) — so it cannot be recovered from the log. Instead
     * of reconstructing it, this pins the geodetic track's first valid fix to
     * [anchorEast]/[anchorNorth], normally the EKF track's first sample. #741
     * measured the two solutions 0.7 m apart at launch, so that alignment
     * costs at most that, and every metre of divergence after it is real.
     *
     * Rows with no fix (lat or lon exactly 0, or non-finite) are dropped, and
     * negative altitudes are clamped to 0 — both matching iOS
     * `extractTrackPoints`, which drops them to avoid underground artefacts
     * from sensor noise.
     */
    public fun geodeticToEnu(
        lat: List<Double>,
        lon: List<Double>,
        alt: List<Double>,
        anchorEast: Double = 0.0,
        anchorNorth: Double = 0.0,
    ): List<V3> {
        val rows = minOf(lat.size, lon.size, alt.size)
        var originLat = Double.NaN
        var originLon = Double.NaN
        var metresPerDegree = 0.0
        var cosLat = 0.0
        val out = ArrayList<V3>(rows)

        for (i in 0 until rows) {
            val la = lat[i]
            val lo = lon[i]
            val al = alt[i]
            // "No fix" is exactly 0/0 in these logs, not a plausible position.
            if (!la.isFinite() || !lo.isFinite() || la == 0.0 || lo == 0.0) continue
            if (!al.isFinite()) continue

            if (originLat.isNaN()) {
                originLat = la
                originLon = lo
                // Scale by R + altitude and by the ORIGIN latitude's cosine,
                // mirroring the firmware's own forward conversion rather than
                // a more correct one, so the two tracks share a metric.
                metresPerDegree = (R_EARTH_M + maxOf(al, 0.0)) * PI / 180.0
                cosLat = cos(la * PI / 180.0)
            }

            out.add(
                V3(
                    e = anchorEast + (lo - originLon) * cosLat * metresPerDegree,
                    n = anchorNorth + (la - originLat) * metresPerDegree,
                    u = maxOf(al, 0.0),
                ),
            )
        }
        return out
    }

    public fun downsample(track: List<V3>, maxCount: Int): List<V3> {
        if (track.size <= maxCount || maxCount < 3) return track
        val lm = landmarks(track) ?: return track
        val stride = (track.size + maxCount - 1) / maxCount
        val out = ArrayList<V3>(maxCount + 2)
        var lastKept = -1
        for (i in track.indices step stride) {
            if (i == lm.apogeeIndex) {
                out.add(track[i]); lastKept = i
            } else {
                // Keep apogee if the stride would step over it.
                if (lastKept < lm.apogeeIndex && i > lm.apogeeIndex) out.add(track[lm.apogeeIndex])
                out.add(track[i]); lastKept = i
            }
        }
        if (lastKept < lm.apogeeIndex) out.add(track[lm.apogeeIndex])
        if (out.last() != track.last()) out.add(track.last())
        return out
    }

    public data class Camera(
        val yawRad: Double,
        val pitchRad: Double,
        val distance: Double,
        val center: V3,
        val fovDeg: Double = 50.0,
    )

    /**
     * iOS parity: side view perpendicular to the launch→apogee horizontal
     * bearing, at 1.8× extent, elevated ~15° so the ground plane reads as a
     * plane.  With no horizontal travel (a perfectly vertical flight) any
     * side works; use east.
     */
    public fun initialCamera(track: List<V3>): Camera {
        val lm = landmarks(track)
        val ext = extent(track)
        val c = center(track)
        val yaw = if (lm != null) {
            val de = lm.apogee.e - lm.launch.e
            val dn = lm.apogee.n - lm.launch.n
            if (abs(de) < 1e-6 && abs(dn) < 1e-6) PI / 2
            else atan2(de, dn) + PI / 2   // bearing of travel + 90° = broadside
        } else {
            PI / 2
        }
        return Camera(
            yawRad = yaw,
            pitchRad = Math.toRadians(15.0),
            distance = ext * 1.8,
            center = c,
        )
    }

    public fun cameraPosition(cam: Camera): V3 {
        val ch = cos(cam.pitchRad) * cam.distance
        return V3(
            cam.center.e + ch * sin(cam.yawRad),
            cam.center.n + ch * cos(cam.yawRad),
            cam.center.u + sin(cam.pitchRad) * cam.distance,
        )
    }

    /** Projected point: screen fraction of viewport (0..1 each axis when on
     *  screen) + camera-space depth for painter's ordering. */
    public data class Projected(val x: Double, val y: Double, val depth: Double)

    /**
     * World → screen.  [x]/[y] are in PIXELS for a viewport of
     * [widthPx]×[heightPx]; null when the point is at/behind the near plane.
     */
    public fun project(p: V3, cam: Camera, widthPx: Double, heightPx: Double): Projected? {
        val camPos = cameraPosition(cam)

        // View basis: forward toward the center, world-up = +U.
        val f = norm(sub(cam.center, camPos)) ?: return null
        val upWorld = V3(0.0, 0.0, 1.0)
        val r = norm(cross(f, upWorld)) ?: return null
        val up = cross(r, f)

        val d = sub(p, camPos)
        val cx = dot(d, r)
        val cy = dot(d, up)
        val cz = dot(d, f)

        val near = max(cam.distance * 0.001, 0.1)
        if (cz <= near) return null

        val focal = (heightPx / 2.0) / tan(Math.toRadians(cam.fovDeg) / 2.0)
        return Projected(
            x = widthPx / 2.0 + cx * focal / cz,
            y = heightPx / 2.0 - cy * focal / cz,
            depth = cz,
        )
    }

    /** 1/2/5 × 10ⁿ round-down (shared convention with the 2D scale bar). */
    public fun niceGridSpacing(extent: Double): Double {
        val target = extent / 4.0
        if (target <= 0) return 25.0
        val exp = kotlin.math.floor(kotlin.math.log10(target))
        val base = Math.pow(10.0, exp)
        val m = target / base
        return when {
            m >= 5 -> 5 * base
            m >= 2 -> 2 * base
            else -> base
        }
    }

    // ── Vector helpers ───────────────────────────────────────────────────

    private fun sub(a: V3, b: V3) = V3(a.e - b.e, a.n - b.n, a.u - b.u)
    private fun dot(a: V3, b: V3) = a.e * b.e + a.n * b.n + a.u * b.u
    private fun cross(a: V3, b: V3) = V3(
        a.n * b.u - a.u * b.n,
        a.u * b.e - a.e * b.u,
        a.e * b.n - a.n * b.e,
    )
    private fun norm(v: V3): V3? {
        val len = sqrt(dot(v, v))
        if (len < 1e-9) return null
        return V3(v.e / len, v.n / len, v.u / len)
    }
}
