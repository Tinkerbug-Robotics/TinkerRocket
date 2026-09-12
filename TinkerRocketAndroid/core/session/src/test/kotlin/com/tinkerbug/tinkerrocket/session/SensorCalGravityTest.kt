package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/** #1059: the post-cal 1 g check, iOS OnPadCalibrationView's numbers. */
class SensorCalGravityTest {
    @Test
    fun aRocketSittingStillPasses() {
        assertNull(SensorCalGravity.check(0f, 0f, -9.81f))
        // 2.9 % low, still inside the 3 % band.
        assertNull(SensorCalGravity.check(0f, 0f, 9.52f))
    }

    @Test
    fun aMagnitudeOffByMoreThanThreePercentWarnsWithTheNumbers() {
        val w = assertNotNull(SensorCalGravity.check(0f, 0f, 10.3f))
        assertEquals(10.3f, w.magnitudeMps2, 1e-4f)
        assertEquals(5.03f, w.errorPct, 0.01f)
        // The magnitude is what is judged, not any one axis.
        assertNotNull(SensorCalGravity.check(6f, 6f, 6f))
    }

    @Test
    fun noAccelDataIsNotAWarning() {
        assertNull(SensorCalGravity.check(null, null, null))
        assertNull(SensorCalGravity.check(0f, 0f, 0f))
        assertNull(SensorCalGravity.check(0.05f, 0.05f, 0.05f), "under the 0.1 m/s² floor")
    }
}
