package com.tinkerbug.tinkerrocket.session

import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Mirror of iOS LandingPredictorTests (all 15 cases) — the algorithm is
 * validated in depth by the Python harness against the 2026-05-17 flights;
 * these guard the Kotlin binding the same way the Swift suite guards the
 * Swift one.  Plus Kotlin-only pins for interpAngle wraparound and the
 * Open-Meteo parser rules.
 */
class LandingCastTest {

    private val launchLat = 39.000
    private val launchLon = -76.105

    private fun profile(mutate: (RocketProfile) -> RocketProfile = { it }): RocketProfile =
        mutate(RocketProfile.makeDefault("T", 0L))

    /** FROM 250°, ~10 kts layered — like the 2026-05-17 Eagle Claw flight. */
    private fun sampleWind() = WindProfile(
        layers = listOf(
            WindLayer(0.0, 6.0, 250.0),
            WindLayer(500.0, 8.0, 250.0),
            WindLayer(2000.0, 10.0, 250.0),
        ),
        groundElevFt = 0.0, fetchTime = "", lat = launchLat, lon = launchLon,
    )

    @Test
    fun descentReturnsTrackEndingAtGround() {
        val track = LandingCast.simulateDescentForLanding(
            launchLat, launchLon, 1500.0, -20.0,
            profile { it.copy(mainDeployAltAglFt = 700.0, drogueRateFps = 60.0, mainRateFps = 12.0) },
            sampleWind(),
        )
        assertTrue(track.size > 2)
        assertEquals(0.0, track.last().altAglFt, 0.5)
    }

    @Test
    fun descentDriftsDownwindFromFromDirection() {
        // FROM 250° blows TO 70° (ENE) — landing must sit east of launch.
        val track = LandingCast.simulateDescentForLanding(
            launchLat, launchLon, 1000.0, -5.0, profile(), sampleWind(),
        )
        assertTrue(track.last().lon > launchLon)
    }

    @Test
    fun observedFallRateOverridesProfileRate() {
        val p = profile { it.copy(drogueRateFps = 60.0) }
        val slow = LandingCast.simulateDescentForLanding(launchLat, launchLon, 1500.0, -3.0, p, sampleWind())
        val fast = LandingCast.simulateDescentForLanding(launchLat, launchLon, 1500.0, -25.0, p, sampleWind())
        assertTrue(
            abs(slow.last().lon - launchLon) > abs(fast.last().lon - launchLon),
            "slower observed descent must drift further",
        )
    }

    @Test
    fun noWindMeansNoDrift() {
        val track = LandingCast.simulateDescentForLanding(
            launchLat, launchLon, 1500.0, -15.0, profile(), wind = null,
        )
        assertEquals(launchLat, track.last().lat, 1e-6)
        assertEquals(launchLon, track.last().lon, 1e-6)
    }

    // ── Uncertainty model (#191 item 2) ──────────────────────────────────

    @Test
    fun uncertaintyIsFractionOfMeanWindTimesDescentTime() {
        val wind = WindProfile(
            listOf(WindLayer(0.0, 10.0, 250.0)), 0.0, "", launchLat, launchLon,
        )
        val track = listOf(
            TrackPoint(launchLat, launchLon, 1000.0, 0.0),
            TrackPoint(launchLat, launchLon, 0.0, 100.0),
        )
        assertEquals(
            LandingCast.SNAPSHOT_POSITION_ERROR_METERS + 0.2 * 5.14444 * 100,
            LandingCast.landingUncertainty(track, wind),
            0.1,
        )
    }

    @Test
    fun noWindDataChargesAssumedWindBound() {
        val track = listOf(
            TrackPoint(launchLat, launchLon, 1000.0, 0.0),
            TrackPoint(launchLat, launchLon, 0.0, 100.0),
        )
        assertEquals(
            LandingCast.SNAPSHOT_POSITION_ERROR_METERS + 200.0,
            LandingCast.landingUncertainty(track, null),
            1e-9,
        )
    }

    @Test
    fun uncertaintyFloorsAtGnssErrorNotZero() {
        val atGround = listOf(
            TrackPoint(launchLat, launchLon, 0.0, 0.0),
            TrackPoint(launchLat, launchLon, 0.0, 0.0),
        )
        assertEquals(LandingCast.SNAPSHOT_POSITION_ERROR_METERS, LandingCast.landingUncertainty(atGround, sampleWind()), 1e-9)
        assertEquals(LandingCast.SNAPSHOT_POSITION_ERROR_METERS, LandingCast.landingUncertainty(atGround, null), 1e-9)
        assertEquals(LandingCast.SNAPSHOT_POSITION_ERROR_METERS, LandingCast.landingUncertainty(emptyList(), sampleWind()), 1e-9)
    }

    @Test
    fun uncertaintyShrinksMonotonicallyTowardTheFloor() {
        val p = profile { it.copy(drogueRateFps = 60.0) }
        val radii = listOf(3000.0, 2000.0, 1000.0, 300.0, 50.0).map { alt ->
            LandingCast.landingUncertainty(
                LandingCast.simulateDescentForLanding(launchLat, launchLon, alt, -15.0, p, sampleWind()),
                sampleWind(),
            )
        }
        radii.zipWithNext().forEach { (hi, lo) -> assertTrue(hi > lo) }
        assertTrue(radii.last() > LandingCast.SNAPSHOT_POSITION_ERROR_METERS)
    }

    @Test
    fun uncertaintyGrowsWithTimeAloft() {
        val p = profile { it.copy(drogueRateFps = 60.0) }
        val slow = LandingCast.simulateDescentForLanding(launchLat, launchLon, 1500.0, -3.0, p, sampleWind())
        val fast = LandingCast.simulateDescentForLanding(launchLat, launchLon, 1500.0, -25.0, p, sampleWind())
        assertTrue(
            LandingCast.landingUncertainty(slow, sampleWind()) >
                LandingCast.landingUncertainty(fast, sampleWind()),
        )
    }

    // ── Ascent ballistic (#191 item 1) ───────────────────────────────────

    @Test
    fun ascentGravityOnlyMatchesClosedForm() {
        val track = LandingCast.simulateAscentToApogee(
            launchLat, launchLon, 100.0, velE = 0.0, velN = 0.0, velU = 50.0, dragK = 0.0,
        )
        val apogee = track.last()
        val expectedFt = 100 + DriftCast.mToFt(50.0 * 50.0 / (2.0 * 9.80665))
        assertEquals(expectedFt, apogee.altAglFt, 10.0)
        assertEquals(50.0 / 9.80665, apogee.timeS, 0.1)
        assertEquals(launchLat, apogee.lat, 1e-9)
        assertEquals(launchLon, apogee.lon, 1e-9)
    }

    @Test
    fun ascentDragLowersApogee() {
        val ballistic = LandingCast.simulateAscentToApogee(launchLat, launchLon, 0.0, 0.0, 0.0, 100.0, 0.0)
        val dragged = LandingCast.simulateAscentToApogee(launchLat, launchLon, 0.0, 0.0, 0.0, 100.0, 5e-4)
        assertTrue(dragged.last().altAglFt < ballistic.last().altAglFt - 200.0)
    }

    @Test
    fun ascentCarriesDownrange() {
        val track = LandingCast.simulateAscentToApogee(
            launchLat, launchLon, 500.0, velE = 30.0, velN = 0.0, velU = 60.0, dragK = 5e-4,
        )
        assertTrue(track.last().lon > launchLon)
        assertEquals(launchLat, track.last().lat, 1e-6)
    }

    @Test
    fun ascentThenDescentIsContinuousAndLands() {
        val (track, descent) = LandingCast.simulateAscentThenDescent(
            launchLat, launchLon, 800.0, velE = 10.0, velN = 5.0, velU = 80.0,
            profile = profile(), dragK = 5e-4, wind = sampleWind(),
        )
        assertEquals(0.0, track.last().altAglFt, 0.5)
        track.zipWithNext().forEach { (a, b) ->
            assertTrue(b.timeS >= a.timeS, "time continuous across the stitch")
        }
        val maxAlt = track.maxOf { it.altAglFt }
        assertTrue(maxAlt > 800.0, "apogee must exceed the snapshot altitude")
        assertEquals(maxAlt, descent.first().altAglFt, 0.5)
    }

    @Test
    fun ascentDragSpreadPositiveWhenDragMatters() {
        val p = profile()
        val (track, _) = LandingCast.simulateAscentThenDescent(
            launchLat, launchLon, 500.0, 40.0, 0.0, 100.0, p, 5e-4, sampleWind(),
        )
        val spread = LandingCast.ascentDragSpreadMeters(
            launchLat, launchLon, 500.0, 40.0, 0.0, 100.0, p, 5e-4, sampleWind(), track.lastOrNull(),
        )
        assertTrue(spread > 10.0, "±50% drag-k on a 100 m/s coast must move the pin (was $spread)")

        val (track0, _) = LandingCast.simulateAscentThenDescent(
            launchLat, launchLon, 500.0, 40.0, 0.0, 100.0, p, 0.0, sampleWind(),
        )
        val spread0 = LandingCast.ascentDragSpreadMeters(
            launchLat, launchLon, 500.0, 40.0, 0.0, 100.0, p, 0.0, sampleWind(), track0.lastOrNull(),
        )
        assertEquals(0.0, spread0, 0.001)
    }

    @Test
    fun uncertaintyShrinksAsAltitudeFalls() {
        val p = profile()
        val high = LandingCast.simulateDescentForLanding(launchLat, launchLon, 1500.0, -15.0, p, sampleWind())
        val low = LandingCast.simulateDescentForLanding(launchLat, launchLon, 400.0, -15.0, p, sampleWind())
        assertTrue(
            LandingCast.landingUncertainty(high, sampleWind()) >
                LandingCast.landingUncertainty(low, sampleWind()),
        )
    }

    // ── Reverse drift-cast guidance point ────────────────────────────────

    @Test
    fun guidancePoint_roundTripsThroughForwardVerification() {
        // Reverse (upwind walk) then forward (downwind drift) through the
        // SAME layered wind must land back near the target — the layer
        // discretization is identical in both directions, so the error is
        // meters, not the drift scale (~km).
        val r = computeGuidancePoint(
            launchLat = launchLat, launchLon = launchLon,
            landingLat = launchLat, landingLon = launchLon,
            apogeeAglFt = 10_000.0,
            drogueRateFps = 60.0, mainRateFps = 18.0, mainDeployAglFt = 700.0,
            windProfile = sampleWind(),
        )
        assertTrue(r.landingErrorM < 20.0, "round-trip error ${r.landingErrorM} m")
        assertTrue(r.totalDriftM > 100.0, "10k ft under 6-10 kts must drift far")
        // FROM 250° → guidance point must sit upwind (west) of the target.
        assertTrue(r.guidanceLon < launchLon)
        assertTrue(r.totalDescentTimeS > 100.0)
    }

    @Test
    fun guidancePoint_feasibilityGate() {
        // Strong wind + low apogee → guidance point far off-pad → infeasible.
        val gale = WindProfile(
            listOf(WindLayer(0.0, 40.0, 250.0)), 0.0, "", launchLat, launchLon,
        )
        val r = computeGuidancePoint(
            launchLat, launchLon, launchLat, launchLon,
            apogeeAglFt = 1000.0,
            drogueRateFps = 30.0, mainRateFps = 15.0, mainDeployAglFt = 500.0,
            windProfile = gale,
        )
        assertTrue(!r.feasible)
        assertNotNull(r.infeasibleReason)
        assertTrue(r.steeringAngleDeg > 25.0)

        // Zero wind: guidance point IS the landing point, trivially feasible.
        val calm = WindProfile(listOf(WindLayer(0.0, 0.0, 0.0)), 0.0, "", launchLat, launchLon)
        val r0 = computeGuidancePoint(
            launchLat, launchLon, launchLat, launchLon,
            apogeeAglFt = 1000.0,
            drogueRateFps = 30.0, mainRateFps = 15.0, mainDeployAglFt = 500.0,
            windProfile = calm,
        )
        assertTrue(r0.feasible)
        assertEquals(launchLat, r0.guidanceLat, 1e-9)
        assertEquals(launchLon, r0.guidanceLon, 1e-9)
        assertEquals(0.0, r0.steeringAngleDeg, 1e-9)
    }

    // ── Kotlin-only pins ─────────────────────────────────────────────────

    @Test
    fun interpAngle_handlesWraparound() {
        // 350° → 10° midpoint is 0°, not 180°.
        assertEquals(0.0, DriftCast.interpAngle(350.0, 10.0, 0.5), 1e-9)
        assertEquals(355.0, DriftCast.interpAngle(350.0, 10.0, 0.25), 1e-9)
        // Antipodal is ambiguous; the Swift formula resolves 90→270 via the
        // -180 branch, midpoint 0 — pin the SAME choice, not a "nicer" one.
        assertEquals(0.0, DriftCast.interpAngle(90.0, 270.0, 0.5), 1e-9)
    }

    @Test
    fun openMeteoParser_buildsLayers_skipsBelowGround_synthesizesSurface() {
        // Two levels: 1000 hPa (~111 m ASL) and 850 hPa (~1457 m ASL) with a
        // 500 m ground elevation → the 1000 hPa layer is below ground
        // (-389 m AGL < -100 ft… actually -1276 ft) and must be skipped;
        // the surviving 850 hPa layer sits well above 100 ft so a surface
        // layer is synthesized at 0.7× its speed.
        val json = """
        {
          "elevation": 500.0,
          "hourly": {
            "time": ["2026-07-27T00:00"],
            "wind_speed_1000hPa": [12.0],
            "wind_direction_1000hPa": [180.0],
            "wind_speed_850hPa": [20.0],
            "wind_direction_850hPa": [250.0]
          }
        }
        """.trimIndent()
        val wp = parseOpenMeteoWinds(json, hourUtc = 0, lat = 39.0, lon = -76.0)
        assertNotNull(wp)
        assertEquals(2, wp.layers.size, "below-ground layer skipped + surface synthesized")
        assertEquals(0.0, wp.layers[0].altFt, 1e-9)
        assertEquals(20.0 * 0.7, wp.layers[0].speedKts, 1e-9)
        assertEquals(250.0, wp.layers[0].directionDeg, 1e-9)
        assertEquals(250.0, wp.layers[1].directionDeg, 1e-9)
        assertTrue(wp.layers[1].altFt > 2000.0, "850 hPa over 500 m ground is ~3100 ft AGL")

        // Malformed JSON → null, never a throw.
        assertNull(parseOpenMeteoWinds("{oops", 0, 0.0, 0.0))
    }
}
