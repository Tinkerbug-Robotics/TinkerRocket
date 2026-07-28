package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.net.HttpURLConnection
import java.net.URL
import java.time.Instant
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import kotlin.math.abs
import kotlin.math.pow

/**
 * Live in-flight landing-point prediction (issue #156) — port of iOS
 * LandingPredictor.swift.  Watches the active session's telemetry,
 * classifies flight phase, and drift-casts forward to where the rocket
 * will land; if LoRa drops mid-flight the last prediction stays pinned so
 * the recovery walk has a target.
 *
 * Descent (alt_apo or vU ≤ 0.5): cast from the current GNSS position with
 * the profile's rates + cached wind.  Post-burnout coast (#191 item 1):
 * ballistic-with-drag on the EKF velocity to apogee, then the descent
 * cast.  Boost never predicts — no thrust model.
 *
 * The wind fetch is a constructor seam so the class tests without
 * network; production uses [fetchOpenMeteoWinds].
 */
public class LandingPredictor(
    private val scope: CoroutineScope,
    private val windFetcher: suspend (lat: Double, lon: Double) -> WindProfile? = { lat, lon ->
        fetchOpenMeteoWinds(lat, lon, System.currentTimeMillis())
    },
    private val clock: () -> Long = System::currentTimeMillis,
) {
    private val _prediction = MutableStateFlow<LandingPrediction?>(null)
    public val prediction: StateFlow<LandingPrediction?> = _prediction.asStateFlow()

    private val _windProfile = MutableStateFlow<WindProfile?>(null)
    public val windProfile: StateFlow<WindProfile?> = _windProfile.asStateFlow()

    private val _windFetchError = MutableStateFlow<String?>(null)
    public val windFetchError: StateFlow<String?> = _windFetchError.asStateFlow()

    private var session: DeviceSession? = null
    private var profileStore: RocketProfileStore? = null
    private var jobs = mutableListOf<Job>()

    private var lastWindFetchAtMs: Long? = null
    private var lastWindFetchLat = 0.0
    private var lastWindFetchLon = 0.0
    private var landed = false

    /** Refetch the wind profile if older than this and still on the pad. */
    private val windRefetchAfterMs = 3_600_000L

    public fun attach(session: DeviceSession, profileStore: RocketProfileStore) {
        detach()
        this.session = session
        this.profileStore = profileStore
        landed = false
        jobs += scope.launch {
            session.telemetry.collect { handleTelemetry(it) }
        }
    }

    public fun detach() {
        jobs.forEach { it.cancel() }
        jobs.clear()
        session = null
        profileStore = null
        _prediction.value = null
        _windProfile.value = null
        _windFetchError.value = null
        landed = false
        lastWindFetchAtMs = null
    }

    // ── Telemetry handling (iOS handleTelemetry verbatim) ────────────────

    internal fun handleTelemetry(t: TelemetryData) {
        // No prediction before anything resembling a GPS position — the
        // latched lastValidRocketFix is the recovery anchor already.
        val lat = t.latitude ?: return
        val lon = t.longitude ?: return
        if (t.numSats < 4) return

        val now = clock()
        prefetchWindIfNeeded(t, lat, lon, now)

        // Once LANDED is asserted, freeze the last prediction so the pin
        // doesn't twitch on ground noise.
        if (t.landedFlag) {
            if (!landed) {
                val last = _prediction.value
                landed = true
                if (last != null) {
                    _prediction.value = last.copy(
                        snapshotSource = LandingSnapshotSource.LATCHED,
                        computedAtMs = now,
                    )
                }
            }
            return
        }

        val vU = (t.altitudeRate ?: 0f).toDouble()
        val descending = t.altApo || vU <= 0.5
        val profile = profileStore?.activeProfile ?: return

        // pressure_alt is baro; operators tare to launch → treat as AGL.
        val altAglFt = DriftCast.mToFt((t.pressureAlt ?: 0f).toDouble())

        val track: List<TrackPoint>
        val source: LandingSnapshotSource
        val uncertainty: Double

        if (descending) {
            track = LandingCast.simulateDescentForLanding(
                startLat = lat, startLon = lon,
                currentAltAglFt = altAglFt,
                observedVerticalRateMps = vU,
                profile = profile, wind = _windProfile.value,
            )
            source = LandingSnapshotSource.GNSS // num_sats >= 4 already required
            uncertainty = LandingCast.landingUncertainty(track, _windProfile.value)
        } else if (t.launchFlag && t.burnoutFlag && t.velE != null && t.velN != null) {
            // Post-burnout coast: position anchors on GNSS, velocity on the
            // EKF (GNSS velocity is the boost casualty; pre-#191 firmware
            // sends no ve/vn, so this branch never runs there).
            val vu = t.velU?.toDouble() ?: vU
            // Apogee straddle: baro climbing, EKF not → next packet is descent.
            if (vu <= 0.5) return
            val ve = t.velE!!.toDouble()
            val vn = t.velN!!.toDouble()
            val k = profile.ballisticDragK

            val (stitched, descent) = LandingCast.simulateAscentThenDescent(
                startLat = lat, startLon = lon,
                currentAltAglFt = altAglFt,
                velE = ve, velN = vn, velU = vu,
                profile = profile, dragK = k, wind = _windProfile.value,
            )
            track = stitched
            source = LandingSnapshotSource.EKF
            uncertainty = LandingCast.landingUncertainty(descent, _windProfile.value) +
                LandingCast.ascentDragSpreadMeters(
                    startLat = lat, startLon = lon,
                    currentAltAglFt = altAglFt,
                    velE = ve, velN = vn, velU = vu,
                    profile = profile, dragK = k, wind = _windProfile.value,
                    nominalLanding = stitched.lastOrNull(),
                )
        } else {
            return
        }

        val landing = track.lastOrNull() ?: return
        _prediction.value = LandingPrediction(
            landingLat = landing.lat, landingLon = landing.lon,
            descentTrack = track,
            snapshotLat = lat, snapshotLon = lon,
            snapshotAltAglFt = altAglFt,
            snapshotSource = source,
            computedAtMs = now, sampleAtMs = now,
            uncertaintyMeters = uncertainty,
        )
    }

    // ── Wind prefetch (pad only — never on the live flight path) ─────────

    private fun prefetchWindIfNeeded(t: TelemetryData, lat: Double, lon: Double, now: Long) {
        val preflight = t.state == "READY" || t.state == "PRELAUNCH" || t.state == "UNKNOWN"
        if (!preflight) return

        val last = lastWindFetchAtMs
        if (last != null && now - last < windRefetchAfterMs &&
            abs(lastWindFetchLat - lat) < 0.01 && abs(lastWindFetchLon - lon) < 0.01 &&
            _windProfile.value != null
        ) {
            return
        }
        lastWindFetchAtMs = now
        lastWindFetchLat = lat
        lastWindFetchLon = lon

        jobs += scope.launch {
            runCatching { windFetcher(lat, lon) }
                .onSuccess { wp ->
                    if (wp != null) {
                        _windProfile.value = wp
                        _windFetchError.value = null
                    }
                }
                .onFailure { _windFetchError.value = it.message ?: "wind fetch failed" }
        }
    }
}

// ── Open-Meteo wind fetch (iOS fetchWinds) ──────────────────────────────

private val PRESSURE_LEVELS_HPA =
    listOf(1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200)

// Standard atmosphere (troposphere) — pressure → geometric altitude.
private const val STD_P0 = 101_325.0
private const val STD_T0 = 288.15
private const val STD_L = 0.0065
private const val STD_G = 9.80665
private const val STD_M = 0.0289644
private const val STD_R = 8.31447

internal fun pressureToAltitudeM(pHpa: Double): Double {
    val exponent = STD_R * STD_L / (STD_G * STD_M)
    return (STD_T0 / STD_L) * (1.0 - ((pHpa * 100.0) / STD_P0).pow(exponent))
}

/**
 * Parse an Open-Meteo pressure-level response into a WindProfile — pure,
 * so the layer-building rules (below-ground skip, surface-layer synthesis
 * at 0.7×, altitude sort) are unit-testable without HTTP.
 */
public fun parseOpenMeteoWinds(
    jsonText: String,
    hourUtc: Int,
    lat: Double,
    lon: Double,
    groundElevFtOverride: Double? = null,
): WindProfile? {
    val root = runCatching { Json.parseToJsonElement(jsonText).jsonObject }.getOrNull() ?: return null
    val hourly = root["hourly"]?.jsonObject ?: return null
    val times = hourly["time"]?.jsonArray ?: return null
    if (times.isEmpty()) return null

    val apiElevM = root["elevation"]?.jsonPrimitive?.doubleOrNull ?: 0.0
    val groundElevFt = groundElevFtOverride ?: DriftCast.mToFt(apiElevM)
    val groundElevM = groundElevFt * 0.3048
    val idx = minOf(hourUtc, times.size - 1)

    val layers = mutableListOf<WindLayer>()
    for (p in PRESSURE_LEVELS_HPA) {
        val speedArr = hourly["wind_speed_${p}hPa"]?.jsonArray ?: continue
        val dirArr = hourly["wind_direction_${p}hPa"]?.jsonArray ?: continue
        if (idx >= speedArr.size || idx >= dirArr.size) continue
        val speed = speedArr[idx].jsonPrimitive.doubleOrNull ?: continue
        val dir = dirArr[idx].jsonPrimitive.doubleOrNull ?: continue
        if (speed < 0 || speed >= 500 || dir < 0 || dir > 360) continue

        val altAglFt = DriftCast.mToFt(pressureToAltitudeM(p.toDouble()) - groundElevM)
        if (altAglFt < -100) continue // below ground
        layers += WindLayer(altFt = maxOf(0.0, altAglFt), speedKts = speed, directionDeg = dir)
    }
    layers.sortBy { it.altFt }

    // Ensure a surface layer exists (0.7× the lowest aloft layer).
    layers.firstOrNull()?.let { first ->
        if (first.altFt > 100) {
            layers.add(0, WindLayer(0.0, first.speedKts * 0.7, first.directionDeg))
        }
    }

    return WindProfile(
        layers = layers, groundElevFt = groundElevFt,
        fetchTime = "", lat = lat, lon = lon,
    )
}

/** Fetch the wind profile from the Open-Meteo pressure-level API. */
public fun fetchOpenMeteoWinds(lat: Double, lon: Double, nowMs: Long): WindProfile? {
    val utc = Instant.ofEpochMilli(nowMs).atZone(ZoneOffset.UTC)
    val dateStr = utc.format(DateTimeFormatter.ISO_LOCAL_DATE)
    val hour = utc.hour

    val speedVars = PRESSURE_LEVELS_HPA.joinToString(",") { "wind_speed_${it}hPa" }
    val dirVars = PRESSURE_LEVELS_HPA.joinToString(",") { "wind_direction_${it}hPa" }
    val url = "https://api.open-meteo.com/v1/forecast" +
        "?latitude=$lat&longitude=$lon" +
        "&hourly=$speedVars,$dirVars" +
        "&wind_speed_unit=kn" +
        "&start_date=$dateStr&end_date=$dateStr" +
        "&timezone=UTC"

    return runCatching {
        val conn = URL(url).openConnection() as HttpURLConnection
        conn.setRequestProperty("User-Agent", "TinkerRocket-DriftCast/1.0")
        conn.connectTimeout = 15_000
        conn.readTimeout = 15_000
        try {
            if (conn.responseCode != 200) return@runCatching null
            parseOpenMeteoWinds(
                conn.inputStream.readBytes().decodeToString(),
                hourUtc = hour, lat = lat, lon = lon,
            )?.copy(fetchTime = "${dateStr}T%02d:00Z".format(hour))
        } finally {
            conn.disconnect()
        }
    }.getOrNull()
}
