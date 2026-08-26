package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * #838 item 3 — Android plotted the EKF ENU track where iOS plotted the GNSS
 * track, and rendered base-station LoRa logs as literal text.
 *
 * `ekfTrack` read only `Position East/North/Up (m)`, the nav-filter solution.
 * Two consequences:
 *
 *  (a) On the same rocket `flight_*.csv` the two apps drew different paths and
 *      different landing markers. Not a rounding difference: #741 measured the
 *      nav filter and GNSS landing **81 m apart** on the CENJARS flight, after
 *      a 1.9 s boost satellite outage the filter dead-reckoned through and
 *      never reconverged from. An operator walking to the Android marker
 *      searches the wrong spot.
 *
 *  (b) The base-station CSV header is
 *      `time_ms,state,num_sats,pdop,lat,lon,alt_m,…` — no `Position East (m)`
 *      column exists, so the reader returned an empty list and the screen
 *      rendered "No EKF position data in this log" for EVERY LoRa log, while
 *      iOS drew the full track.
 *
 * Both views now draw both solutions, which is the rule #741 set: never
 * average them, never silently prefer one.
 */
class FlightTracksTest {

    private val padLat = 40.0
    private val padLon = -105.0

    private fun csv(cols: Map<String, List<Double>>) =
        FlightCsvData(headers = cols.keys.toList(), columns = cols,
                      rowCount = cols.values.firstOrNull()?.size ?: 0)

    /** A rocket flight: both solutions present. */
    private fun rocketCsv() = csv(mapOf(
        "Position East (m)" to listOf(0.0, 10.0, 20.0),
        "Position North (m)" to listOf(0.0, 0.0, 0.0),
        "Position Up (m)" to listOf(0.0, 50.0, 0.0),
        "Latitude (deg)" to listOf(padLat, padLat, padLat),
        "Longitude (deg)" to listOf(padLon, padLon + 0.001, padLon + 0.002),
        "Pressure Altitude (m)" to listOf(0.0, 50.0, 0.0),
    ))

    /** A base-station LoRa log: short names, and NO EKF columns at all. */
    private fun loraCsv() = csv(mapOf(
        "time_ms" to listOf(0.0, 100.0, 200.0),
        "lat" to listOf(padLat, padLat + 0.0005, padLat + 0.001),
        "lon" to listOf(padLon, padLon, padLon),
        "pressure_alt" to listOf(0.0, 40.0, 0.0),
    ))

    // ── (b) the blank screen ────────────────────────────────────────────

    /** THE regression: a LoRa log has a track, and it must be found. */
    @Test
    fun loraLogYieldsATrackDespiteHavingNoEkfColumns() {
        val tracks = flightTracks(loraCsv())
        assertTrue(tracks.ekf.isEmpty(), "a LoRa log has no EKF columns")
        assertEquals(3, tracks.gnss.size, "the lat/lon track was not read")
        assertTrue(!tracks.isEmpty, "the screen would still say 'no position data'")
        assertEquals(tracks.gnss, tracks.primary,
                     "with no EKF solution the GNSS track must carry the markers")
    }

    @Test
    fun aLogWithNeitherSolutionIsStillEmpty() {
        val tracks = flightTracks(csv(mapOf("time_ms" to listOf(0.0, 1.0))))
        assertTrue(tracks.isEmpty)
    }

    // ── (a) both solutions, and the gap between them ────────────────────

    @Test
    fun aRocketFlightYieldsBothSolutions() {
        val tracks = flightTracks(rocketCsv())
        assertEquals(3, tracks.ekf.size)
        assertEquals(3, tracks.gnss.size)
        assertEquals(tracks.ekf, tracks.primary, "the EKF track carries the markers")
    }

    /**
     * The two are anchored at their first common sample — the EKF origin is
     * never logged (#741), so it cannot be recovered, and #741 measured the
     * solutions 0.7 m apart at launch, so pinning them there costs at most
     * that and leaves every later metre of divergence intact.
     */
    @Test
    fun theTwoSolutionsStartTogether() {
        val tracks = flightTracks(rocketCsv())
        assertEquals(tracks.ekf.first().e, tracks.gnss.first().e, 1e-9)
        assertEquals(tracks.ekf.first().n, tracks.gnss.first().n, 1e-9)
    }

    /** A divergence after the anchor survives — that is the whole point. */
    @Test
    fun divergenceAfterTheAnchorIsPreserved() {
        val tracks = flightTracks(rocketCsv())
        val gap = abs(tracks.ekf.last().e - tracks.gnss.last().e)
        assertTrue(gap > 50.0, "the solutions were flattened together (gap ${gap}m)")
    }

    /** Extent covers both, or the scaling clips one off the canvas. */
    @Test
    fun extentSpansBothSolutions() {
        val tracks = flightTracks(rocketCsv())
        assertEquals(tracks.ekf.size + tracks.gnss.size, tracks.all.size)
    }

    // ── the geodetic conversion ─────────────────────────────────────────

    @Test
    fun oneDegreeOfLatitudeIsAboutOneHundredAndElevenKm() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(padLat, padLat + 1.0),
            lon = listOf(padLon, padLon),
            alt = listOf(0.0, 0.0),
        )
        assertEquals(111_195.0, enu[1].n, 50.0)
        assertEquals(0.0, enu[1].e, 1e-6)
    }

    @Test
    fun eastAndNorthPointTheRightWay() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(padLat, padLat + 0.001, padLat),
            lon = listOf(padLon, padLon, padLon + 0.001),
            alt = listOf(0.0, 0.0, 0.0),
        )
        assertTrue(enu[1].n > 0, "increasing latitude must go north")
        assertEquals(0.0, enu[1].e, 1e-6)
        assertTrue(enu[2].e > 0, "increasing longitude must go east")
        assertEquals(0.0, enu[2].n, 1e-6)
    }

    /** Longitude degrees shrink with latitude's cosine. */
    @Test
    fun eastingUsesTheOriginLatitudesCosine() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(padLat, padLat),
            lon = listOf(padLon, padLon + 1.0),
            alt = listOf(0.0, 0.0),
        )
        // cos(40°) ≈ 0.766 → ~85.2 km, not 111 km.
        assertEquals(85_200.0, enu[1].e, 500.0)
    }

    /** "No fix" is exactly 0/0 in these logs, and must not plot as Null Island. */
    @Test
    fun rowsWithNoFixAreDropped() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(0.0, padLat, 0.0, padLat + 0.001),
            lon = listOf(0.0, padLon, padLon, padLon),
            alt = listOf(0.0, 0.0, 0.0, 0.0),
        )
        assertEquals(2, enu.size, "a no-fix row was plotted")
        assertEquals(0.0, enu.first().e, 1e-9)
    }

    @Test
    fun nonFiniteRowsAreDropped() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(padLat, Double.NaN, padLat + 0.001, padLat + 0.002),
            lon = listOf(padLon, padLon, Double.NaN, padLon),
            alt = listOf(0.0, 0.0, 0.0, Double.NaN),
        )
        assertEquals(1, enu.size)
    }

    /** Negative altitude is sensor noise, and drives the track underground. */
    @Test
    fun negativeAltitudeIsClampedToGround() {
        val enu = Trajectory3D.geodeticToEnu(
            lat = listOf(padLat, padLat),
            lon = listOf(padLon, padLon),
            alt = listOf(-3.5, 12.0),
        )
        assertEquals(0.0, enu[0].u)
        assertEquals(12.0, enu[1].u)
    }

    @Test
    fun anchoringTranslatesTheWholeTrack() {
        val plain = Trajectory3D.geodeticToEnu(
            listOf(padLat, padLat + 0.001), listOf(padLon, padLon), listOf(0.0, 0.0),
        )
        val moved = Trajectory3D.geodeticToEnu(
            listOf(padLat, padLat + 0.001), listOf(padLon, padLon), listOf(0.0, 0.0),
            anchorEast = 100.0, anchorNorth = -250.0,
        )
        assertEquals(plain[1].n + -250.0, moved[1].n, 1e-9)
        assertEquals(plain[1].e + 100.0, moved[1].e, 1e-9)
    }

    @Test
    fun anEmptyTrackIsEmpty() {
        assertEquals(emptyList(), Trajectory3D.geodeticToEnu(emptyList(), emptyList(), emptyList()))
    }

    /** GNSS altitude is MSL and drives the track underground; prefer baro. */
    @Test
    fun pressureAltitudeIsPreferredOverGnssAltitude() {
        val data = csv(mapOf(
            "Latitude (deg)" to listOf(padLat, padLat),
            "Longitude (deg)" to listOf(padLon, padLon),
            "Pressure Altitude (m)" to listOf(0.0, 100.0),
            "GNSS Altitude (m)" to listOf(1600.0, 1700.0),
        ))
        assertEquals(listOf(0.0, 100.0), gnssTrack(data).map { it.u })
    }

    @Test
    fun gnssAltitudeIsUsedWhenThereIsNoBarometer() {
        val data = csv(mapOf(
            "Latitude (deg)" to listOf(padLat, padLat),
            "Longitude (deg)" to listOf(padLon, padLon),
            "GNSS Altitude (m)" to listOf(1600.0, 1700.0),
        ))
        assertEquals(listOf(1600.0, 1700.0), gnssTrack(data).map { it.u })
    }
}
