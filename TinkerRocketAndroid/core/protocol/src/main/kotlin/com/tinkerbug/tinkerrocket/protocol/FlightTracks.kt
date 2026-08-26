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
    /** Nothing to draw at all — neither solution is present. */
    public val isEmpty: Boolean get() = ekf.size < 2 && gnss.size < 2

    /** The track that carries the flight's markers: EKF when it exists. */
    public val primary: List<V3>
        get() = if (ekf.size >= 2) ekf else gnss

    /** Every point either track contributes, for extent/scaling. */
    public val all: List<V3> get() = ekf + gnss
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
