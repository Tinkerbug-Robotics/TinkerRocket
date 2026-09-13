package com.tinkerbug.tinkerrocket.session

import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #552: `hacc` chooses how far back to difference GNSS positions.  These pin
 * the behaviour the four 2026-08-29 BARC L2 flights were measured against.
 */
class GnssVelocityCheckTest {

    private val mPerDegLat = 111_320.0

    /** A rocket moving due north at [velN] m/s, sampled every [stepMs]. */
    private fun track(
        n: Int,
        velN: Double,
        hAccM: Int?,
        stepMs: Long = 500,
        truthVelN: Double = velN,
        lat0: Double = 40.0,
    ): List<GnssVelocityCheck.Sample> = (0 until n).map { i ->
        val tS = i * stepMs / 1000.0
        GnssVelocityCheck.Sample(
            tMs = i * stepMs,
            // Position advances at the TRUE rate; velE/velN are what the
            // filter claims. Equal by default, so they agree.
            latDeg = lat0 + truthVelN * tS / mPerDegLat,
            lonDeg = -105.0,
            hAccM = hAccM,
            velE = 0.0, velN = velN,
        )
    }

    @Test
    fun `a filter that agrees with its own fixes reads near zero`() {
        val r = assertNotNull(GnssVelocityCheck.evaluate(track(n = 10, velN = 30.0, hAccM = 0)))
        assertTrue(r.disagreementMps < 0.5, "agreement should read ~0, got ${r.disagreementMps}")
    }

    @Test
    fun `a filter inventing 40 m per s is caught at that size`() {
        // Position says 5 m/s, the filter says 45 -- the RIM-66 shape.
        val h = track(n = 10, velN = 45.0, hAccM = 0, truthVelN = 5.0)
        val r = assertNotNull(GnssVelocityCheck.evaluate(h))
        assertEquals(40.0, r.disagreementMps, 1.0)
    }

    @Test
    fun `good fixes difference at the frame rate`() {
        val r = assertNotNull(GnssVelocityCheck.evaluate(track(n = 10, velN = 30.0, hAccM = 0)))
        assertEquals(0.5, r.baselineS, 1e-9)
    }

    @Test
    fun `a 29 m fix lengthens the baseline instead of trusting it`() {
        // Rolly Polly V: four satellites, 29 m accuracy. sqrt(2)*29/3 = 13.7 s
        // of baseline would be needed, so this clamps at MAX_BASELINE_S.
        val r = assertNotNull(GnssVelocityCheck.evaluate(track(n = 20, velN = 30.0, hAccM = 29)))
        assertEquals(GnssVelocityCheck.MAX_BASELINE_S, r.baselineS, 1e-9)
    }

    @Test
    fun `the baseline grows with hacc rather than jumping`() {
        val baselines = listOf(0, 1, 3, 6, 12, 29).map { h ->
            GnssVelocityCheck.evaluate(track(n = 30, velN = 30.0, hAccM = h))!!.baselineS
        }
        assertEquals(baselines.sorted(), baselines, "baseline must be monotone in hacc")
        assertTrue(baselines.first() < baselines.last())
    }

    @Test
    fun `the longer baseline is what cuts the noise floor`() {
        // The point of the whole exercise: at the frame rate a 29 m fix gives
        // a velocity noise floor far above any real disagreement, so the
        // estimate would be mostly noise.
        // n = 2 is the only history that FORCES the frame interval: with more
        // samples the code already walks back as far as it is allowed to.
        val short = GnssVelocityCheck.evaluate(track(n = 2, velN = 30.0, hAccM = 29))!!
        val long = GnssVelocityCheck.evaluate(track(n = 20, velN = 30.0, hAccM = 29))!!
        assertEquals(0.5, short.baselineS, 1e-9)
        assertTrue(short.noiseFloorMps > 50.0, "0.5 s on a 29 m fix is ~82 m/s, got ${short.noiseFloorMps}")
        assertTrue(long.noiseFloorMps < 12.0, "4 s should bring it under 12 m/s, got ${long.noiseFloorMps}")
    }

    @Test
    fun `an absent hacc keeps the old short baseline`() {
        // Legacy firmware and the mini. Absent must not silently become a
        // LONG baseline either -- that would lag the estimate on every flight
        // that never had the field.
        val r = assertNotNull(GnssVelocityCheck.evaluate(track(n = 10, velN = 30.0, hAccM = null)))
        assertEquals(0.5, r.baselineS, 1e-9)
    }

    @Test
    fun `hacc zero and hacc absent agree here, and neither claims zero noise`() {
        val absent = GnssVelocityCheck.evaluate(track(n = 10, velN = 30.0, hAccM = null))!!
        val zero = GnssVelocityCheck.evaluate(track(n = 10, velN = 30.0, hAccM = 0))!!
        assertEquals(zero.baselineS, absent.baselineS, 1e-9)
        // 5 dp of wire precision is still 1.11 m of rounding; a claim of zero
        // noise would be arithmetic, not measurement.
        assertTrue(zero.noiseFloorMps > 0.5, "quantization alone is ~0.9 m/s, got ${zero.noiseFloorMps}")
    }

    @Test
    fun `one sample cannot be differenced`() {
        assertNull(GnssVelocityCheck.evaluate(track(n = 1, velN = 30.0, hAccM = 0)))
        assertNull(GnssVelocityCheck.evaluate(emptyList()))
    }

    @Test
    fun `history older than the window is dropped`() {
        var h = emptyList<GnssVelocityCheck.Sample>()
        for (i in 0 until 200) {
            h = GnssVelocityCheck.trimmed(
                h,
                GnssVelocityCheck.Sample(i * 500L, 40.0, -105.0, 0, 0.0, 0.0),
            )
        }
        val spanS = (h.last().tMs - h.first().tMs) / 1000.0
        assertTrue(spanS <= GnssVelocityCheck.HISTORY_S + 0.5, "kept $spanS s")
        assertTrue(spanS >= GnssVelocityCheck.MAX_BASELINE_S, "must still cover the longest baseline")
    }

    @Test
    fun `a backwards clock jump does not leave a stale sample to difference against`() {
        var h = emptyList<GnssVelocityCheck.Sample>()
        h = GnssVelocityCheck.trimmed(h, GnssVelocityCheck.Sample(100_000L, 40.0, -105.0, 0, 0.0, 0.0))
        h = GnssVelocityCheck.trimmed(h, GnssVelocityCheck.Sample(1_000L, 40.0, -105.0, 0, 0.0, 0.0))
        assertEquals(1, h.size, "the future-stamped sample must not survive")
        assertNull(GnssVelocityCheck.evaluate(h))
    }

    @Test
    fun `the filter average is compared against the chord, not one endpoint`() {
        // A filter whose velocity ramps 0 -> 40 while the rocket truly moves
        // at the mean of that ramp is NOT disagreeing; comparing the chord
        // against the latest instantaneous value would read 20 m/s.
        // Needs the LONG baseline to be meaningful -- over one frame interval
        // the "average" is just the two endpoints. hacc 29 forces the 4 s
        // window, which here spans the whole ramp.
        val n = 9
        val step = 500L
        val ramp = (0 until n).map { it * 5.0 }          // 0..40, mean 20
        val samples = (0 until n).map { i ->
            GnssVelocityCheck.Sample(
                tMs = i * step,
                latDeg = 40.0 + 20.0 * (i * step / 1000.0) / mPerDegLat,
                lonDeg = -105.0, hAccM = 29,
                velE = 0.0, velN = ramp[i],
            )
        }
        val r = assertNotNull(GnssVelocityCheck.evaluate(samples))
        assertEquals(GnssVelocityCheck.MAX_BASELINE_S, r.baselineS, 1e-9)
        assertTrue(
            abs(r.disagreementMps) < 3.0,
            "a ramp about the true mean is agreement, got ${r.disagreementMps}",
        )
    }

    // ── The other half: the disagreement has to reach the radius ────────
    //
    // Everything above pins the ESTIMATOR. These pin the WIRING, because a
    // spread term that silently returned 0 would satisfy every test above.

    private fun profile() = RocketProfile(
        name = "test", createdAtMs = 0L, updatedAtMs = 0L,
    )

    @Test
    fun `no disagreement means no velocity term`() {
        val nominal = LandingCast.simulateAscentThenDescent(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
        ).first.last()
        val spread = LandingCast.ascentVelocitySpreadMeters(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
            nominalLanding = nominal, disagreementMps = 0.0,
        )
        assertEquals(0.0, spread, 1e-9)
    }

    @Test
    fun `a 40 m per s disagreement moves the landing point by a useful distance`() {
        val nominal = LandingCast.simulateAscentThenDescent(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
        ).first.last()
        val spread = LandingCast.ascentVelocitySpreadMeters(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
            nominalLanding = nominal, disagreementMps = 40.0,
        )
        // Assert the PHYSICS rather than a number: the spread per m/s of
        // disagreement is an effective lever arm in seconds, and that lever is
        // time-to-APOGEE, not time-to-ground -- the predictor integrates the
        // rocket's own velocity only until vu <= 0 and then hands over to the
        // wind drift cast. Climbing at 100 m/s that is 7-8 s, and these
        // flights run 19-40 s to the ground, so a regression that restored the
        // to-the-ground reading (the mistake made while diagnosing #552) would
        // blow the upper bound rather than hide.
        val leverS = spread / 40.0
        assertTrue(leverS > 4.0, "40 m/s should be clearly visible, lever ${leverS}s")
        assertTrue(leverS < 12.0, "lever must be time-to-apogee, not to ground: ${leverS}s")
    }

    @Test
    fun `the velocity term grows with the disagreement`() {
        val nominal = LandingCast.simulateAscentThenDescent(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
        ).first.last()
        val spreads = listOf(0.0, 5.0, 15.0, 40.0).map { d ->
            LandingCast.ascentVelocitySpreadMeters(
                startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
                velE = 20.0, velN = 0.0, velU = 100.0,
                profile = profile(), dragK = 5e-4, wind = null,
                nominalLanding = nominal, disagreementMps = d,
            )
        }
        assertEquals(spreads.sorted(), spreads, "must be monotone in the disagreement")
        assertTrue(spreads.last() > spreads.first())
    }

    @Test
    fun `a nonsense disagreement is refused rather than propagated`() {
        val nominal = LandingCast.simulateAscentThenDescent(
            startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
            velE = 20.0, velN = 0.0, velU = 100.0,
            profile = profile(), dragK = 5e-4, wind = null,
        ).first.last()
        for (bad in listOf(Double.NaN, Double.POSITIVE_INFINITY, -1.0)) {
            assertEquals(
                0.0,
                LandingCast.ascentVelocitySpreadMeters(
                    startLat = 40.0, startLon = -105.0, currentAltAglFt = 1000.0,
                    velE = 20.0, velN = 0.0, velU = 100.0,
                    profile = profile(), dragK = 5e-4, wind = null,
                    nominalLanding = nominal, disagreementMps = bad,
                ),
                1e-9, "disagreement $bad must not produce a radius",
            )
        }
    }
}
