package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * [TouchdownPredictor]: the model, and the last 20 s of three real flights
 * whose landing call never came (2026-10-24 BARC). iOS twin:
 * `TouchdownPredictorTests`.
 */
class TouchdownPredictorTest {

    /** The 2026-10-24 Rolly Polly V setup: main charge at 800 ft on descent. */
    private val dualDeploy = RecoveryProfile(
        drogueMps = 18.0, mainMps = 6.5, mainDeployAglM = 213.0, pyroMainAglM = 243.84,
    )

    /** A motor-ejection rocket whose recovery fields were left at the
     *  defaults: 700 ft main, but no pyro channel fires on altitude. */
    private val singleDeploy = RecoveryProfile(drogueMps = 18.3, mainMps = 3.66, mainDeployAglM = 213.0)

    // ── The model ────────────────────────────────────────────────────────

    @Test
    fun singleRate_isHeightOverRate() {
        assertEquals(5.0, TouchdownPredictor.secondsToTouchdown(30.0, 6.0, null), 1e-9)
    }

    @Test
    fun atOrBelowPadLevel_isNow() {
        assertEquals(0.0, TouchdownPredictor.secondsToTouchdown(-12.0, 6.0, null))
        assertEquals(0.0, TouchdownPredictor.secondsToTouchdown(0.0, 6.0, null))
    }

    /** Under drogue above the main: the observed rate down to the main
     *  altitude, the profile's main rate below it. */
    @Test
    fun dualDeploy_underDrogue_addsTheMainLeg() {
        assertEquals(
            (400 - 243.84) / 16 + 243.84 / 6.5,
            TouchdownPredictor.secondsToTouchdown(400.0, 16.0, dualDeploy), 1e-9,
        )
    }

    /** Below the main altitude but still falling at drogue speed: the main
     *  has fired and not yet opened (04 at 200 m). */
    @Test
    fun dualDeploy_belowMainAltitudeStillFast_usesTheMainRate() {
        assertEquals(200 / 6.5, TouchdownPredictor.secondsToTouchdown(200.0, 16.0, dualDeploy), 1e-9)
    }

    @Test
    fun dualDeploy_underMain_isHeightOverRate() {
        assertEquals(100 / 6.0, TouchdownPredictor.secondsToTouchdown(100.0, 6.0, dualDeploy), 1e-9)
    }

    /** The recovery fields always hold a main altitude, so they cannot be
     *  what says a main is coming. */
    @Test
    fun noAltitudeChannel_isHeightOverRateEvenWhenFast() {
        assertEquals(400 / 9.0, TouchdownPredictor.secondsToTouchdown(400.0, 9.0, singleDeploy), 1e-9)
    }

    @Test
    fun rate_isTheMeanOfTheLastThreeSeconds() {
        val p = TouchdownPredictor()
        p.observe(0, 100f, -20f, null)        // drops out of the window
        p.observe(1_000, 90f, -4f, null)
        p.observe(2_000, 85f, -6f, null)
        p.observe(4_000, 80f, -8f, null)
        assertEquals(4_000 + (80 / 6.0 * 1000).toLong(), p.predictedTouchdownMs)
    }

    /** Just past apogee the rocket is not yet falling: nothing is predicted. */
    @Test
    fun hoveringAtApogee_predictsNothing() {
        val p = TouchdownPredictor()
        p.observe(0, 300f, 0f, null)
        p.observe(500, 300f, -1f, null)
        assertNull(p.predictedTouchdownMs)
        assertNull(p.callDueMs)
    }

    @Test
    fun callDue_waitsForTheLinkToGoQuiet() {
        val p = TouchdownPredictor()
        p.observe(0, 2f, -8f, null)
        assertEquals(1_500L, p.callDueMs, "touchdown is 0.25 s out, the call waits for silence")
        // Higher up the touchdown itself is the later of the two.
        val high = TouchdownPredictor()
        high.observe(0, 120f, -6f, null)
        assertEquals(20_000L, high.callDueMs)
    }

    /** RecoveryProfile.from reads the main altitude off the pyro channels. */
    @Test
    fun recoveryProfile_mainAltitudeFromThePyroChannels() {
        val base = RocketProfile(name = "Rolly Polly V", createdAtMs = 0, updatedAtMs = 0)
        assertNull(RecoveryProfile.from(base).pyroMainAglM, "no channel enabled")
        val dual = base.copy(
            pyro1Enabled = true, pyro1TriggerMode = 0, pyro1TriggerValue = 1f,          // apogee + 1 s
            pyro2Enabled = true, pyro2TriggerMode = 1, pyro2TriggerValue = 243.84f,     // 800 ft
            pyro3Enabled = false, pyro3TriggerMode = 1, pyro3TriggerValue = 400f,       // off
        )
        assertEquals(243.84, RecoveryProfile.from(dual).pyroMainAglM!!, 1e-3)
    }

    // ── Real flights ─────────────────────────────────────────────────────

    /** When the call comes on each, relative to the real touchdown, s. */
    private fun callTime(frames: List<Triple<Double, Float, Float>>): Double? {
        val p = TouchdownPredictor()
        for ((t, alt, rate) in frames) p.observe((t * 1000).toLong(), alt, rate, null)
        return p.callDueMs?.let { it / 1000.0 }
    }

    @Test
    fun realFlights_callWithinSecondsOfTouchdown() {
        assertEquals(1.1, assertNotNull(callTime(GOBLIN_01)), 0.1)
        assertEquals(1.0, assertNotNull(callTime(ROLL_CTRL_03)), 0.1)
        assertEquals(3.0, assertNotNull(callTime(PIERCER_05)), 0.1)
    }

    /** Had the link died higher up on 03, the prediction from that frame. */
    @Test
    fun realFlight_cutHigher_predictsTheTouchdown() {
        for (cut in listOf(100f, 60f, 30f)) {
            val p = TouchdownPredictor()
            for ((t, alt, rate) in ROLL_CTRL_03.takeWhile { it.second >= cut }) {
                p.observe((t * 1000).toLong(), alt, rate, null)
            }
            assertEquals(0.0, assertNotNull(p.predictedTouchdownMs) / 1000.0, 2.0, "cut at $cut m")
        }
    }

    internal companion object {
        private fun f(vararg v: Double): List<Triple<Double, Float, Float>> =
            v.toList().chunked(3).map { (t, a, r) -> Triple(t, a.toFloat(), r.toFloat()) }

        /**
         * Live LoRa frames from the base-station logs: seconds from the flight
         * computer's own touchdown, pressure altitude (m), altitude rate
         * (m/s). In all three the link died at the ground and the landed
         * frame arrived minutes later. Same numbers as the iOS test.
         */
        val GOBLIN_01 = f(-19.92, 331.0, -23.0, -19.42, 325.0, -14.0, -18.96, 325.0, -14.0, -17.92, 299.0, -15.0, -17.42, 282.0, -35.0, -16.92, 273.0, -22.0, -16.42, 263.0, -29.0, -15.97, 263.0, -29.0, -15.42, 243.0, -22.0, -14.92, 235.0, -22.0, -14.42, 227.0, -15.0, -13.92, 215.0, -25.0, -13.42, 206.0, -22.0, -12.96, 206.0, -22.0, -12.42, 188.0, -13.0, -11.92, 187.0, 4.0, -11.42, 166.0, -29.0, -10.92, 159.0, -14.0, -10.42, 150.0, -15.0, -9.96, 150.0, -15.0, -8.92, 117.0, -15.0, -8.42, 101.0, -42.0, -7.92, 98.0, -7.0, -7.42, 94.0, -7.0, -6.97, 94.0, -7.0, -6.42, 85.0, -11.0, -5.92, 79.0, -9.0, -5.42, 72.0, -15.0, -4.92, 68.0, -9.0, -4.42, 61.0, -13.0, -3.97, 61.0, -13.0, -3.42, 48.0, -14.0, -2.92, 42.0, -13.0, -2.42, 34.0, -14.0, -1.92, 26.0, -19.0, -1.42, 19.0, -14.0, -0.96, 19.0, -14.0, -0.42, 4.0, -18.0)
        val ROLL_CTRL_03 = f(-19.50, 162.0, -6.0, -19.00, 157.0, -10.0, -18.50, 152.0, -10.0, -17.99, 148.0, -8.0, -17.50, 142.0, -12.0, -17.04, 142.0, -12.0, -16.50, 133.0, -10.0, -15.99, 129.0, -6.0, -15.50, 124.0, -10.0, -15.00, 119.0, -8.0, -14.50, 115.0, -9.0, -14.04, 115.0, -9.0, -13.50, 105.0, -9.0, -13.00, 100.0, -9.0, -12.50, 96.0, -8.0, -12.00, 92.0, -9.0, -11.50, 89.0, -8.0, -11.04, 89.0, -8.0, -10.50, 80.0, -7.0, -10.00, 76.0, -8.0, -9.50, 72.0, -6.0, -9.00, 67.0, -9.0, -8.50, 63.0, -8.0, -8.04, 63.0, -8.0, -7.50, 55.0, -9.0, -7.00, 52.0, -8.0, -6.50, 47.0, -10.0, -6.00, 44.0, -6.0, -5.50, 39.0, -10.0, -5.04, 39.0, -10.0, -4.50, 32.0, -6.0, -4.00, 28.0, -5.0, -3.50, 24.0, -8.0, -3.00, 20.0, -7.0, -2.50, 16.0, -6.0, -2.04, 16.0, -6.0, -1.50, 9.0, -6.0, -1.00, 6.0, -7.0, -0.50, 2.0, -7.0)
        /** Landed below pad level: the frames reach -12 m still descending. */
        val PIERCER_05 = f(-19.52, 100.0, -7.0, -19.02, 98.0, -4.0, -18.52, 95.0, -7.0, -18.02, 92.0, -6.0, -17.52, 89.0, -6.0, -17.06, 89.0, -6.0, -16.52, 83.0, -5.0, -16.02, 81.0, -4.0, -15.52, 78.0, -6.0, -15.02, 75.0, -5.0, -14.52, 72.0, -5.0, -14.06, 72.0, -5.0, -13.52, 67.0, -4.0, -13.02, 65.0, -5.0, -12.52, 62.0, -6.0, -12.02, 59.0, -5.0, -11.52, 57.0, -6.0, -11.06, 57.0, -6.0, -10.52, 52.0, -4.0, -10.02, 49.0, -5.0, -9.52, 47.0, -4.0, -9.02, 44.0, -5.0, -8.52, 41.0, -6.0, -8.06, 41.0, -6.0, -7.52, 35.0, -4.0, -7.02, 32.0, -7.0, -6.52, 29.0, -5.0, -6.02, 27.0, -4.0, -5.52, 24.0, -6.0, -4.52, 19.0, -5.0, -4.02, 16.0, -7.0, -3.52, 14.0, -3.0, -3.02, 10.0, -7.0, -2.52, 8.0, -4.0, -2.06, 8.0, -4.0, -1.52, 4.0, -6.0, -1.02, 1.0, -6.0, -0.52, -2.0, -5.0, -0.02, -4.0, -4.0, 0.48, -7.0, -5.0, 0.94, -7.0, -5.0, 1.48, -12.0, -6.0)
    }
}
