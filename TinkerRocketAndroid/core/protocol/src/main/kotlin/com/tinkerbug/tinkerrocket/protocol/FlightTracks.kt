package com.tinkerbug.tinkerrocket.protocol

import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.V3
import com.tinkerbug.tinkerrocket.protocol.Trajectory3D.geodeticToEnu

// Position-track extraction for the flight-trajectory views.  Lives here
// rather than beside the Compose canvases because :app has no test source set
// — and the base-station column handling below is exactly what went untested
// and wrong (#838 item 3).

/**
 * Both position solutions for a flight, in the same local ENU frame
 * (#838 item 3).
 *
 * They genuinely disagree: #741 measured the nav filter and GNSS landing
 * **81 m apart** on the CENJARS flight, after a 1.9 s boost satellite outage
 * the filter dead-reckoned through and never reconverged from. Drawing one
 * silently sends an operator to the wrong end of a field, so both views draw
 * both and label them — the rule the #741 investigation set.
 *
 * [gnss] is also the ONLY track a base-station LoRa log has: its CSV header is
 * `time_ms,state,num_sats,pdop,lat,lon,alt_m,…` with no `Position East (m)`
 * column at all, so the EKF-only reader returned an empty list and the screen
 * rendered the literal text "No EKF position data in this log" for every LoRa
 * log — while iOS drew the full track from lat/lon.
 */
public data class FlightTracks(
    public val ekf: List<V3>,
    public val gnss: List<V3>,
) {
    /** Nothing meaningful to draw — neither solution has a path.
     *
     * #1092: "a path" is size >= 2 AND non-degenerate. A log where the EKF
     * never initialized carries all-zero Position East/North/Up columns
     * (main.cpp fills them only under `if (ekf_initialized)`), which passed the
     * old `size < 2` test and drew a zero-extent line with Launch/Landing/
     * Apogee stacked on one point. It now reads as empty, so the screen falls
     * through to "No position data" — matching iOS's "No GPS data in this
     * flight". NOT a per-row `e == 0 && n == 0` reject: an ENU (0,0) is the
     * launch pad, a legitimate first sample (see rocketCsv in the tests). */
    public val isEmpty: Boolean get() = !ekf.hasPath() && !gnss.hasPath()

    /** The track that carries the flight's markers: EKF when it has a path,
     *  else GNSS. A degenerate (zero-extent) EKF track no longer wins over a
     *  real GNSS one, which is why this checks hasPath() rather than size. */
    public val primary: List<V3>
        get() = if (ekf.hasPath()) ekf else gnss

    /** Every point either track contributes, for extent/scaling. */
    public val all: List<V3> get() = ekf + gnss
}

// #1092: a track "has a path" when it has at least two points AND spans more
// than a hair on some axis. A track that is two-or-more identical points (the
// all-zero EKF columns of an uninitialised-filter log) has no path — drawing
// it stacks every marker on one point instead of saying there is no data.
private const val TRACK_SPAN_EPS_M: Double = 1e-6

private fun List<V3>.hasPath(): Boolean {
    if (size < 2) return false
    var minE = this[0].e; var maxE = minE
    var minN = this[0].n; var maxN = minN
    var minU = this[0].u; var maxU = minU
    for (p in this) {
        if (p.e < minE) minE = p.e; if (p.e > maxE) maxE = p.e
        if (p.n < minN) minN = p.n; if (p.n > maxN) maxN = p.n
        if (p.u < minU) minU = p.u; if (p.u > maxU) maxU = p.u
    }
    return (maxE - minE) > TRACK_SPAN_EPS_M ||
        (maxN - minN) > TRACK_SPAN_EPS_M ||
        (maxU - minU) > TRACK_SPAN_EPS_M
}

/** Both solutions, GNSS anchored to the EKF track's first sample. */
public fun flightTracks(data: FlightCsvData): FlightTracks {
    val ekf = ekfTrack(data)
    val anchor = ekf.firstOrNull()
    return FlightTracks(
        ekf = ekf,
        gnss = gnssTrack(data, anchorEast = anchor?.e ?: 0.0,
                         anchorNorth = anchor?.n ?: 0.0),
    )
}

/**
 * Clean finite GNSS (E, N, U) triples, in the EKF track's frame.
 *
 * Reads the rocket CSV's `Latitude (deg)`/`Longitude (deg)` or the
 * base-station LoRa log's `lat`/`lon`, preferring pressure altitude over GNSS
 * altitude in both — GNSS altitude is MSL and drives the track underground,
 * which is the same preference iOS `extractTrackPoints` makes.
 */
public fun gnssTrack(
    data: FlightCsvData,
    anchorEast: Double = 0.0,
    anchorNorth: Double = 0.0,
): List<V3> {
    val lat = data.columns["Latitude (deg)"] ?: data.columns["lat"] ?: return emptyList()
    val lon = data.columns["Longitude (deg)"] ?: data.columns["lon"] ?: return emptyList()
    val alt = data.columns["Pressure Altitude (m)"]
        ?: data.columns["pressure_alt"]
        ?: data.columns["GNSS Altitude (m)"]
        ?: data.columns["alt_m"]
        ?: return emptyList()
    return geodeticToEnu(lat, lon, alt, anchorEast, anchorNorth)
        
}

/** A geodetic fix, degrees. */
public data class GeoFix(val lat: Double, val lon: Double)

/**
 * The first fix [gnssTrack] accepts — same columns, same rejection of the
 * 0/0 "no fix" rows and of non-finite altitude — so it is the geodetic twin
 * of `gnssTrack(data).first()`.  The 3D scenes centre their ground texture on
 * it (#1092 item 1; iOS uses `trackPoints.first`).  Null when the log has no
 * position columns or no usable fix.
 */
public fun firstGnssFix(data: FlightCsvData): GeoFix? {
    val lat = data.columns["Latitude (deg)"] ?: data.columns["lat"] ?: return null
    val lon = data.columns["Longitude (deg)"] ?: data.columns["lon"] ?: return null
    val alt = data.columns["Pressure Altitude (m)"]
        ?: data.columns["pressure_alt"]
        ?: data.columns["GNSS Altitude (m)"]
        ?: data.columns["alt_m"]
        ?: return null
    val rows = minOf(lat.size, lon.size, alt.size)
    for (i in 0 until rows) {
        val la = lat[i]
        val lo = lon[i]
        if (!la.isFinite() || !lo.isFinite() || la == 0.0 || lo == 0.0) continue
        if (!alt[i].isFinite()) continue
        return GeoFix(la, lo)
    }
    return null
}

/** Clean finite EKF (E, N, U) triples — the shared source for 2D and 3D. */
public fun ekfTrack(data: FlightCsvData): List<V3> {
    val e = data.columns["Position East (m)"] ?: emptyList()
    val n = data.columns["Position North (m)"] ?: emptyList()
    val u = data.columns["Position Up (m)"]
        ?: data.columns["Pressure Altitude (m)"] ?: emptyList()
    val rows = minOf(e.size, n.size, u.size)
    return (0 until rows).mapNotNull { i ->
        if (e[i].isFinite() && n[i].isFinite() && u[i].isFinite()) {
            V3(e[i], n[i], u[i])
        } else {
            null
        }
    }
}
