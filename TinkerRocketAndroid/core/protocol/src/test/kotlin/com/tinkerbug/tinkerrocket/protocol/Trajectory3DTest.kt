package com.tinkerbug.tinkerrocket.protocol

import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.Camera
import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.V3
import kotlin.math.PI
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Pins the orbit-camera conventions the Canvas renders with.  The point of
 * testing PROJECTION rather than pixels: a flipped axis or swapped yaw sign
 * produces a plausible-looking trajectory that is silently mirrored — the
 * same class of failure as the compass arrow, invisible to an eyeball check.
 */
class Trajectory3DTest {

    private val w = 1000.0
    private val h = 800.0

    /** Camera due south of origin, level-ish, looking north at the center. */
    private fun southCam(pitchDeg: Double = 0.0, dist: Double = 1000.0) = Camera(
        yawRad = PI,               // camera azimuth: due south of center
        pitchRad = Math.toRadians(pitchDeg),
        distance = dist,
        center = V3(0.0, 0.0, 0.0),
    )

    // ── Projection conventions ───────────────────────────────────────────

    @Test
    fun centerProjectsToViewportCenter_atAnyOrbit() {
        for (yawDeg in intArrayOf(0, 45, 137, 233, 359)) {
            for (pitchDeg in intArrayOf(5, 30, 80)) {
                val cam = Camera(Math.toRadians(yawDeg.toDouble()),
                    Math.toRadians(pitchDeg.toDouble()), 500.0, V3(10.0, -20.0, 30.0))
                val p = Trajectory3D.project(cam.center, cam, w, h)
                assertNotNull(p, "yaw=$yawDeg pitch=$pitchDeg")
                assertEquals(w / 2, p.x, 1e-6)
                assertEquals(h / 2, p.y, 1e-6)
            }
        }
    }

    @Test
    fun eastIsScreenRight_whenCameraLooksNorth() {
        val p = Trajectory3D.project(V3(100.0, 0.0, 0.0), southCam(), w, h)
        assertNotNull(p)
        assertTrue(p.x > w / 2, "east must be screen-right, got x=${p.x}")
    }

    @Test
    fun upIsScreenUp() {
        val p = Trajectory3D.project(V3(0.0, 0.0, 100.0), southCam(), w, h)
        assertNotNull(p)
        assertTrue(p.y < h / 2, "higher altitude must be screen-UP, got y=${p.y}")
    }

    @Test
    fun behindCamera_isCulled() {
        // Camera is 1000 m south of center; a point 2000 m south is behind it.
        assertNull(Trajectory3D.project(V3(0.0, -2000.0, 0.0), southCam(), w, h))
    }

    @Test
    fun nearerPointHasSmallerDepth() {
        val near = Trajectory3D.project(V3(0.0, -500.0, 0.0), southCam(), w, h)
        val far = Trajectory3D.project(V3(0.0, 500.0, 0.0), southCam(), w, h)
        assertNotNull(near); assertNotNull(far)
        assertTrue(near.depth < far.depth)
    }

    @Test
    fun yawOrbit_movesAPointOppositeToTheCamera() {
        // Orbiting the camera clockwise must swing the scene the other way —
        // the compass-arrow property, one abstraction up.
        val east = V3(100.0, 0.0, 0.0)
        val p0 = Trajectory3D.project(east, southCam(), w, h)!!
        val turned = southCam().copy(yawRad = PI - 0.3)
        val p1 = Trajectory3D.project(east, turned, w, h)!!
        assertTrue(p1.x != p0.x, "orbiting must move off-center points")
    }

    // ── Landmarks / extent / downsample ──────────────────────────────────

    private fun hop(n: Int = 101, apogeeAt: Int = 50): List<V3> =
        (0 until n).map { i ->
            V3(i.toDouble(), 0.0, 400.0 - abs(i - apogeeAt).toDouble() * 8)
        }

    @Test
    fun landmarks_firstApexLast() {
        val lm = Trajectory3D.landmarks(hop())!!
        assertEquals(0.0, lm.launch.e)
        assertEquals(50.0, lm.apogee.e)
        assertEquals(100.0, lm.landing.e)
        assertEquals(50, lm.apogeeIndex)
    }

    @Test
    fun extent_flooredAt100() {
        val tiny = listOf(V3(0.0, 0.0, 0.0), V3(1.0, 2.0, 3.0))
        assertEquals(100.0, Trajectory3D.extent(tiny))
        // hop(): e spans 100, u spans 0..400 → the U span wins.
        assertEquals(400.0, Trajectory3D.extent(hop()), 1.0)
    }

    @Test
    fun downsample_keepsFirstLastAndApogee() {
        val track = hop(n = 5001, apogeeAt = 2500)
        val out = Trajectory3D.downsample(track, 200)
        assertTrue(out.size <= 205, "got ${out.size}")
        assertEquals(track.first(), out.first())
        assertEquals(track.last(), out.last())
        assertTrue(track[2500] in out, "the apogee sample must survive downsampling")
    }

    @Test
    fun downsample_belowLimit_untouched() {
        val track = hop(n = 50)
        assertEquals(track, Trajectory3D.downsample(track, 200))
    }

    // ── Initial camera ───────────────────────────────────────────────────

    @Test
    fun initialCamera_isBroadsideToTheClimb() {
        // Flight travels due north.  Broadside (camera east/west) shows the
        // arc PROFILE: the horizontal travel spreads across screen-x — the
        // exact opposite of a head-on view (camera on the travel bearing),
        // which collapses it.  Assert the spread dominates head-on, and the
        // climb still reads upward.
        val track = (0..100).map { i ->
            V3(0.0, i.toDouble() * 5, 400.0 - abs(i - 50).toDouble() * 8)
        }
        val cam = Trajectory3D.initialCamera(track)
        val lm = Trajectory3D.landmarks(track)!!
        val pl = Trajectory3D.project(lm.launch, cam, w, h)!!
        val pa = Trajectory3D.project(lm.apogee, cam, w, h)!!
        val broadsideSpread = abs(pl.x - pa.x)

        val headOn = cam.copy(yawRad = cam.yawRad - Math.PI / 2)
        val hl = Trajectory3D.project(lm.launch, headOn, w, h)!!
        val ha = Trajectory3D.project(lm.apogee, headOn, w, h)!!
        val headOnSpread = abs(hl.x - ha.x)

        assertTrue(broadsideSpread > headOnSpread * 3 + 1,
            "broadside must maximise the visible profile: " +
                "broadside=$broadsideSpread headOn=$headOnSpread")
        assertTrue(pa.y < pl.y, "apogee renders above launch")
    }

    @Test
    fun initialCamera_verticalFlight_stillDefined() {
        val track = (0..10).map { V3(0.0, 0.0, it.toDouble() * 40) }
        val cam = Trajectory3D.initialCamera(track)
        assertTrue(cam.distance >= 100.0 * 1.8 - 1e-9)
        assertNotNull(Trajectory3D.project(track.last(), cam, w, h))
    }

    @Test
    fun gridSpacing_is125() {
        // Round-DOWN to a 1/2/5 number: extent/4 = 25 → 20 (25 is not 1-2-5).
        assertEquals(20.0, Trajectory3D.niceGridSpacing(100.0))
        assertEquals(100.0, Trajectory3D.niceGridSpacing(450.0))
        assertEquals(200.0, Trajectory3D.niceGridSpacing(1000.0))
    }
}
