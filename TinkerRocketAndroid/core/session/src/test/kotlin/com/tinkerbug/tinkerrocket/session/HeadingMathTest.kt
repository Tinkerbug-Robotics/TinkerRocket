package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Compass arithmetic for the direction-to-rocket arrow (Checkpoint A Group 7).
 *
 * The point of these is the wrap cases. An arrow 359° wrong looks exactly like
 * an arrow 1° wrong on a phone screen, so the physical 8-heading walk-around
 * cannot catch a sign or modulo error — only this can.
 */
class HeadingMathTest {

    // ── true heading = magnetic + declination ────────────────────────────

    @Test
    fun trueHeading_appliesDeclinationAndNormalises() {
        // Easterly declination (positive) rotates clockwise.
        assertEquals(10.0, HeadingMath.trueHeading(0.0, 10.0), 1e-9)
        assertEquals(100.0, HeadingMath.trueHeading(90.0, 10.0), 1e-9)
    }

    @Test
    fun trueHeading_westerlyDeclinationPastNorthStaysPositive() {
        // The bug this guards: Kotlin's `%` keeps the dividend's sign, so
        // 2 + (-13) would be -11 instead of 349.  New Jersey's declination is
        // about -13°, so this is the everyday case, not an edge case.
        assertEquals(349.0, HeadingMath.trueHeading(2.0, -13.0), 1e-9)
        assertEquals(347.0, HeadingMath.trueHeading(0.0, -13.0), 1e-9)
    }

    @Test
    fun trueHeading_easterlyDeclinationPastNorthWrapsDown() {
        assertEquals(5.0, HeadingMath.trueHeading(355.0, 10.0), 1e-9)
    }

    @Test
    fun trueHeading_isAlwaysInRange() {
        for (mag in 0 until 360) {
            for (dec in listOf(-25.0, -13.0, 0.0, 7.5, 25.0)) {
                val h = HeadingMath.trueHeading(mag.toDouble(), dec)
                assertTrue(h >= 0.0 && h < 360.0, "mag=$mag dec=$dec gave $h")
            }
        }
    }

    // ── angular delta / publish guard ────────────────────────────────────

    @Test
    fun angularDelta_acrossNorthIsSmall() {
        // THE case the guard exists for: 359.5 -> 0.5 is a 1° move, not 359°.
        assertEquals(1.0, HeadingMath.angularDeltaDeg(0.5, 359.5), 1e-9)
        assertEquals(1.0, HeadingMath.angularDeltaDeg(359.5, 0.5), 1e-9)
        assertEquals(2.0, HeadingMath.angularDeltaDeg(1.0, 359.0), 1e-9)
    }

    @Test
    fun angularDelta_isSymmetricAndBounded() {
        val pairs = listOf(0.0 to 90.0, 90.0 to 0.0, 10.0 to 200.0, 350.0 to 170.0, 45.0 to 45.0)
        for ((a, b) in pairs) {
            val d = HeadingMath.angularDeltaDeg(a, b)
            assertEquals(HeadingMath.angularDeltaDeg(b, a), d, 1e-9, "asymmetric for $a/$b")
            assertTrue(d in 0.0..180.0, "$a/$b gave $d")
        }
    }

    @Test
    fun angularDelta_antipodalIs180() {
        assertEquals(180.0, HeadingMath.angularDeltaDeg(0.0, 180.0), 1e-9)
        assertEquals(180.0, HeadingMath.angularDeltaDeg(90.0, 270.0), 1e-9)
    }

    @Test
    fun shouldPublish_holdsStillNearNorthInsteadOfSpamming() {
        // Sitting still pointing north, jittering either side of 0: without a
        // wrap-aware delta every one of these would look like a ~359° move and
        // publish, which is the over-publishing the guard was added to stop.
        assertFalse(HeadingMath.shouldPublish(newDeg = 0.2, currentDeg = 359.8))
        assertFalse(HeadingMath.shouldPublish(newDeg = 359.8, currentDeg = 0.2))
        // A real 1° move does publish.
        assertTrue(HeadingMath.shouldPublish(newDeg = 1.0, currentDeg = 0.0))
        assertTrue(HeadingMath.shouldPublish(newDeg = 0.0, currentDeg = 359.0))
    }

    // ── arrow rotation ───────────────────────────────────────────────────

    @Test
    fun arrowAngle_pointsAheadWhenFacingTheRocket() {
        assertEquals(0.0, HeadingMath.arrowAngleDeg(bearingToRocketDeg = 90.0, phoneHeadingDeg = 90.0), 1e-9)
    }

    @Test
    fun arrowAngle_takesTheShortWayRound() {
        // Rocket just east of north, phone pointing just west of north: the
        // arrow should nudge 20° RIGHT, not 340° left.
        assertEquals(20.0, HeadingMath.arrowAngleDeg(10.0, 350.0), 1e-9)
        // And the mirror case: 20° LEFT, not 340° right.
        assertEquals(-20.0, HeadingMath.arrowAngleDeg(350.0, 10.0), 1e-9)
    }

    @Test
    fun arrowAngle_signIsRightRelativeToTheNose() {
        // Phone facing north, rocket due east → arrow right (+90).
        assertEquals(90.0, HeadingMath.arrowAngleDeg(90.0, 0.0), 1e-9)
        // Phone facing north, rocket due west → arrow left (-90).
        assertEquals(-90.0, HeadingMath.arrowAngleDeg(270.0, 0.0), 1e-9)
        // Directly behind lands on the half-open end: -180, NOT +180.  Same
        // direction for a rotation, but the range is [-180, 180).
        assertEquals(-180.0, HeadingMath.arrowAngleDeg(180.0, 0.0), 1e-9)
    }

    @Test
    fun arrowAngle_isAlwaysInRange() {
        for (bear in 0 until 360 step 7) {
            for (head in 0 until 360 step 11) {
                val a = HeadingMath.arrowAngleDeg(bear.toDouble(), head.toDouble())
                assertTrue(a >= -180.0 && a < 180.0, "bear=$bear head=$head gave $a")
            }
        }
    }

    @Test
    fun arrowAngle_counterRotatesWithThePhone() {
        // The property the physical walk-around actually checks: turning the
        // phone by D must swing the arrow by -D, so it keeps pointing at the
        // same patch of ground.
        val bearing = 42.0
        var prev = HeadingMath.arrowAngleDeg(bearing, 0.0)
        for (step in 1..11) {
            val head = step * 30.0
            val now = HeadingMath.arrowAngleDeg(bearing, head)
            val swing = HeadingMath.angularDeltaDeg(prev, now)
            assertEquals(30.0, swing, 1e-9, "phone turned 30° at head=$head; arrow swung $swing")
            prev = now
        }
    }
}
