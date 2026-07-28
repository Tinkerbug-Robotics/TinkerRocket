package com.tinkerbug.tinkerrocket.session

import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Drift-cast descent engine — port of iOS DriftCastEngine.swift (which
 * itself ports the Python driftcast tool).  Pure math: the wind FETCH
 * (Open-Meteo) lives behind a seam in the predictor so everything here is
 * JVM-testable.  Formulas verbatim from Swift; the algorithm is validated
 * in depth by the Python harness against the 2026-05-17 flights.
 */

public data class WindLayer(
    val altFt: Double,        // altitude AGL (feet)
    val speedKts: Double,     // wind speed (knots)
    val directionDeg: Double, // wind FROM direction (degrees, 0=N)
)

public data class WindProfile(
    val layers: List<WindLayer>,
    val groundElevFt: Double,
    val fetchTime: String,
    val lat: Double,
    val lon: Double,
) {
    /** Interpolate wind speed (kts) + direction (deg) at an AGL altitude. */
    public fun interpolate(altAglFt: Double): Pair<Double, Double> {
        if (layers.isEmpty()) return 0.0 to 0.0
        if (altAglFt <= layers.first().altFt) {
            return layers.first().speedKts to layers.first().directionDeg
        }
        if (altAglFt >= layers.last().altFt) {
            return layers.last().speedKts to layers.last().directionDeg
        }
        for (i in 0 until layers.size - 1) {
            val lo = layers[i]
            val hi = layers[i + 1]
            if (lo.altFt <= altAglFt && altAglFt <= hi.altFt) {
                val t = if (hi.altFt == lo.altFt) 0.0 else (altAglFt - lo.altFt) / (hi.altFt - lo.altFt)
                val speed = lo.speedKts + t * (hi.speedKts - lo.speedKts)
                return speed to DriftCast.interpAngle(lo.directionDeg, hi.directionDeg, t)
            }
        }
        return layers.last().speedKts to layers.last().directionDeg
    }
}

public data class TrackPoint(
    val lat: Double,
    val lon: Double,
    val altAglFt: Double,
    val timeS: Double,
)

public object DriftCast {

    private const val EARTH_RADIUS_M = 6_371_000.0

    // ── Geo / units ──────────────────────────────────────────────────────

    /** Project a point along a great-circle arc (Vincenty direct on sphere). */
    public fun forwardProject(lat: Double, lon: Double, bearingDeg: Double, distanceM: Double): Pair<Double, Double> {
        val rlat = Math.toRadians(lat)
        val rlon = Math.toRadians(lon)
        val rbrng = Math.toRadians(bearingDeg)
        val dOverR = distanceM / EARTH_RADIUS_M

        val lat2 = asin(sin(rlat) * cos(dOverR) + cos(rlat) * sin(dOverR) * cos(rbrng))
        val lon2 = rlon + atan2(
            sin(rbrng) * sin(dOverR) * cos(rlat),
            cos(dOverR) - sin(rlat) * sin(lat2),
        )
        return Math.toDegrees(lat2) to Math.toDegrees(lon2)
    }

    /** Haversine distance in meters (iOS LocationManager.haversineDistance). */
    public fun haversineM(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val a = min(
            1.0,
            sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2),
        )
        return EARTH_RADIUS_M * 2 * atan2(sqrt(a), sqrt(1 - a))
    }

    /** Initial bearing in degrees (0=N, 90=E) from point 1 to point 2. */
    public fun bearingDeg(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val lat1r = Math.toRadians(lat1)
        val lat2r = Math.toRadians(lat2)
        val dLon = Math.toRadians(lon2 - lon1)
        val y = sin(dLon) * cos(lat2r)
        val x = cos(lat1r) * sin(lat2r) - sin(lat1r) * cos(lat2r) * cos(dLon)
        return Math.toDegrees(atan2(y, x))
    }

    public fun ftToM(ft: Double): Double = ft * 0.3048
    public fun mToFt(m: Double): Double = m / 0.3048
    public fun ktsToMps(kts: Double): Double = kts * 0.514444

    // ── Descent simulation ───────────────────────────────────────────────

    private const val LAYER_THICKNESS_FT = 1000.0

    /** Altitude step boundaries low→high, split at the deploy altitude. */
    private fun buildAltSteps(lowFt: Double, highFt: Double, stepFt: Double, deployFt: Double): List<Pair<Double, Double>> {
        val steps = mutableListOf<Pair<Double, Double>>()
        var a = lowFt
        while (a < highFt) {
            val nextA = min(a + stepFt, highFt)
            if (a < deployFt && deployFt < nextA) {
                steps += a to deployFt
                steps += deployFt to nextA
            } else {
                steps += a to nextA
            }
            a = nextA
        }
        return steps
    }

    /**
     * Simulate parachute descent through wind layers.
     * [forward] true = apogee → ground drifting downwind; false = reverse
     * (ground → apogee, walking upwind — the guidance-point cast).
     */
    public fun simulateDescent(
        startLat: Double,
        startLon: Double,
        apogeeAglFt: Double,
        drogueRateFps: Double,
        mainRateFps: Double,
        mainDeployAglFt: Double,
        windProfile: WindProfile,
        forward: Boolean = true,
    ): List<TrackPoint> {
        if (drogueRateFps <= 0.1 || mainRateFps <= 0.1) {
            return listOf(TrackPoint(startLat, startLon, apogeeAglFt, 0.0))
        }

        val track = mutableListOf<TrackPoint>()
        var timeS = 0.0
        var lat = startLat
        var lon = startLon
        val steps = buildAltSteps(0.0, apogeeAglFt, LAYER_THICKNESS_FT, mainDeployAglFt)

        if (forward) {
            track += TrackPoint(lat, lon, apogeeAglFt, timeS)
            for ((stepLo, stepHi) in steps.reversed()) {
                val midAlt = (stepLo + stepHi) / 2.0
                val descentRate = if (midAlt > mainDeployAglFt) drogueRateFps else mainRateFps
                val dt = (stepHi - stepLo) / descentRate
                val (speedKts, fromDir) = windProfile.interpolate(midAlt)
                val driftM = ktsToMps(speedKts) * dt
                // Forward: drift downwind (FROM + 180).
                val (nlat, nlon) = forwardProject(lat, lon, (fromDir + 180.0).mod(360.0), driftM)
                lat = nlat
                lon = nlon
                timeS += dt
                track += TrackPoint(lat, lon, stepLo, timeS)
            }
        } else {
            track += TrackPoint(lat, lon, 0.0, timeS)
            for ((stepLo, stepHi) in steps) {
                val midAlt = (stepLo + stepHi) / 2.0
                val descentRate = if (midAlt > mainDeployAglFt) drogueRateFps else mainRateFps
                val dt = (stepHi - stepLo) / descentRate
                val (speedKts, fromDir) = windProfile.interpolate(midAlt)
                val driftM = ktsToMps(speedKts) * dt
                // Reverse: move upwind (INTO the wind = the FROM direction).
                val (nlat, nlon) = forwardProject(lat, lon, fromDir, driftM)
                lat = nlat
                lon = nlon
                timeS += dt
                track += TrackPoint(lat, lon, stepHi, timeS)
            }
        }
        return track
    }

    /** Linear angle interpolation with wraparound (degrees). */
    internal fun interpAngle(a1: Double, a2: Double, t: Double): Double {
        var diff = (a2 - a1 + 180).mod(360.0) - 180
        if (diff < -180) diff += 360
        return (a1 + t * diff).mod(360.0)
    }
}

// ── Landing-prediction cast layer (iOS LandingPredictor.swift free funcs) ─

/** Snapshot-source tag for the staleness badge and replay logs. */
public enum class LandingSnapshotSource { EKF, GNSS, LATCHED }

/** The published forecast (iOS LandingPrediction). */
public data class LandingPrediction(
    val landingLat: Double,
    val landingLon: Double,
    val descentTrack: List<TrackPoint>,
    val snapshotLat: Double,
    val snapshotLon: Double,
    val snapshotAltAglFt: Double,
    val snapshotSource: LandingSnapshotSource,
    val computedAtMs: Long,
    /** When the underlying telemetry sample arrived — staleness = now − this. */
    val sampleAtMs: Long,
    /** Uncertainty radius (m), FIXED at compute time (#191 item 2). */
    val uncertaintyMeters: Double,
)

public object LandingCast {

    // ── Uncertainty model (#191 item 2) ──────────────────────────────────

    /** ~20% of mean wind × descent time is the issue-#191 spec. */
    private const val WIND_UNCERTAINTY_FRACTION = 0.2

    /** Wind (m/s) charged when no profile was fetched (light breeze). */
    private const val ASSUMED_WIND_WHEN_UNKNOWN_MPS = 2.0

    /**
     * GNSS floor (m): hAcc is NOT on the telemetry wire, so this is the
     * typical 3D-fix figure rather than a per-packet value (#551).
     */
    public const val SNAPSHOT_POSITION_ERROR_METERS: Double = 3.0

    private const val ASCENT_DT_S = 0.05
    private const val ASCENT_MAX_COAST_S = 120.0
    private const val G_MPS2 = 9.80665

    /**
     * Descent-branch wrapper: drop from the current altitude with the
     * observed fall rate overriding the profile rate for the matching
     * layer (drogue above main-deploy alt, main below) — issue #156 spec.
     */
    public fun simulateDescentForLanding(
        startLat: Double,
        startLon: Double,
        currentAltAglFt: Double,
        observedVerticalRateMps: Double,
        profile: RocketProfile,
        wind: WindProfile?,
    ): List<TrackPoint> {
        val windToUse = wind ?: WindProfile(
            layers = listOf(WindLayer(0.0, 0.0, 0.0)),
            groundElevFt = 0.0, fetchTime = "", lat = startLat, lon = startLon,
        )

        var drogueRate = profile.drogueRateFps
        var mainRate = profile.mainRateFps
        val mainAlt = profile.mainDeployAltAglFt

        val fallRateFps = -observedVerticalRateMps * 3.28084
        if (fallRateFps > 0.5) {
            if (currentAltAglFt > mainAlt) drogueRate = fallRateFps else mainRate = fallRateFps
        }

        return DriftCast.simulateDescent(
            startLat = startLat, startLon = startLon,
            apogeeAglFt = max(currentAltAglFt, 1.0),
            drogueRateFps = drogueRate, mainRateFps = mainRate,
            mainDeployAglFt = mainAlt, windProfile = windToUse, forward = true,
        )
    }

    /**
     * Uncertainty radius, fixed at compute time: GNSS floor + wind term
     * over the remaining descent.  Does NOT grow with staleness once
     * latched — the badge, not the radius, carries "how old is this".
     */
    public fun landingUncertainty(track: List<TrackPoint>, wind: WindProfile?): Double {
        if (track.size < 2) return SNAPSHOT_POSITION_ERROR_METERS
        val descentS = max(0.0, track.last().timeS - track.first().timeS)
        if (wind == null) {
            return SNAPSHOT_POSITION_ERROR_METERS + ASSUMED_WIND_WHEN_UNKNOWN_MPS * descentS
        }
        val meanMps = track.map { DriftCast.ktsToMps(wind.interpolate(it.altAglFt).first) }.average()
        return SNAPSHOT_POSITION_ERROR_METERS + WIND_UNCERTAINTY_FRACTION * meanMps * descentS
    }

    // ── Ascent ballistic (#191 item 1) ───────────────────────────────────

    /**
     * Post-burnout coast to apogee under gravity + quadratic drag
     * (a = −g·ẑ − k·|v|·v).  Semi-implicit Euler like the Python
     * reference; last point has vU ≤ 0.  dragK = 0 → gravity-only.
     */
    public fun simulateAscentToApogee(
        startLat: Double,
        startLon: Double,
        currentAltAglFt: Double,
        velE: Double,
        velN: Double,
        velU: Double,
        dragK: Double,
    ): List<TrackPoint> {
        var lat = startLat
        var lon = startLon
        var uM = DriftCast.ftToM(currentAltAglFt)
        var ve = velE
        var vn = velN
        var vu = velU
        var t = 0.0
        val track = mutableListOf(TrackPoint(lat, lon, currentAltAglFt, 0.0))
        val maxSteps = (ASCENT_MAX_COAST_S / ASCENT_DT_S).toInt()
        repeat(maxSteps) {
            val vMag = sqrt(ve * ve + vn * vn + vu * vu)
            ve += -dragK * vMag * ve * ASCENT_DT_S
            vn += -dragK * vMag * vn * ASCENT_DT_S
            vu += (-G_MPS2 - dragK * vMag * vu) * ASCENT_DT_S

            val dE = ve * ASCENT_DT_S
            val dN = vn * ASCENT_DT_S
            uM += vu * ASCENT_DT_S
            t += ASCENT_DT_S
            val dist = sqrt(dE * dE + dN * dN)
            if (dist > 0) {
                val bearing = Math.toDegrees(atan2(dE, dN))
                val (nlat, nlon) = DriftCast.forwardProject(lat, lon, bearing, dist)
                lat = nlat
                lon = nlon
            }
            track += TrackPoint(lat, lon, DriftCast.mToFt(uM), t)
            if (vu <= 0) return track
        }
        return track
    }

    /**
     * Coast to apogee, then the standard descent cast from the apogee
     * point (profile rates — no observed fall rate exists at a future
     * apogee).  Returns the time-continuous stitched track plus the raw
     * descent segment (the uncertainty wind term charges descent only).
     */
    public fun simulateAscentThenDescent(
        startLat: Double,
        startLon: Double,
        currentAltAglFt: Double,
        velE: Double,
        velN: Double,
        velU: Double,
        profile: RocketProfile,
        dragK: Double,
        wind: WindProfile?,
    ): Pair<List<TrackPoint>, List<TrackPoint>> {
        val ascent = simulateAscentToApogee(startLat, startLon, currentAltAglFt, velE, velN, velU, dragK)
        val apogee = ascent.lastOrNull() ?: return emptyList<TrackPoint>() to emptyList()

        val descent = simulateDescentForLanding(
            startLat = apogee.lat, startLon = apogee.lon,
            currentAltAglFt = apogee.altAglFt,
            observedVerticalRateMps = 0.0,
            profile = profile, wind = wind,
        )
        val t0 = apogee.timeS
        val stitched = ascent + descent.drop(1).map { it.copy(timeS = it.timeS + t0) }
        return stitched to descent
    }

    /**
     * Drag-model term of the ascent uncertainty (#191 spec: k ± 50%):
     * re-run the cast at 0.5k and 1.5k, take the largest displacement
     * from the nominal landing.  k = 0 degenerates to 0 spread.
     */
    public fun ascentDragSpreadMeters(
        startLat: Double,
        startLon: Double,
        currentAltAglFt: Double,
        velE: Double,
        velN: Double,
        velU: Double,
        profile: RocketProfile,
        dragK: Double,
        wind: WindProfile?,
        nominalLanding: TrackPoint?,
    ): Double {
        val nominal = nominalLanding ?: return 0.0
        var worst = 0.0
        for (kVariant in listOf(dragK * 0.5, dragK * 1.5)) {
            val (track, _) = simulateAscentThenDescent(
                startLat, startLon, currentAltAglFt, velE, velN, velU, profile, kVariant, wind,
            )
            val l = track.lastOrNull() ?: continue
            worst = max(worst, DriftCast.haversineM(l.lat, l.lon, nominal.lat, nominal.lon))
        }
        return worst
    }
}
