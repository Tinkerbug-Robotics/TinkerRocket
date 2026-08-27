package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData

/**
 * Synthesizes [TelemetryData.pastApogee] — the latched "this flight is past
 * apogee" phase that the wire cannot carry (#968).
 *
 * `altApo` and `velApo` are LIVE VOTES from a leaky counter whose baro test
 * requires `alt_est > 15.0f`, so both CLEAR below ~15 m AGL. Consumers that
 * read them as a phase latch flipped back to "pre-apogee" for the last seconds
 * of every flight — measured 130.96 s -> 138.84 s on the 2026-08-27 sim
 * flight. That is what let a stale burnout be announced just before landing
 * (#964), and the same clearing caused #235.
 *
 * The firmware HAS a latched master vote (`NSF2_MASTER_APOGEE`) but does not
 * transmit it, and there is no room to add it: LoRa `flags_state` is fully
 * allocated and `num_sats` has only 6 bits left against a real 0-40 range. So
 * the app latches it itself, into the bit position the firmware would use — if
 * a future protocol version does send it, this reads through unchanged.
 *
 * Not thread-safe: [apply] runs on the session's single telemetry path.
 */
internal class PastApogeeLatch {

    /**
     * Keyed by relayed rocket id (0 = the session's own direct link) because a
     * base station can relay two flights at once and one rocket reaching
     * apogee must not mark the other (#390).
     */
    private val seen = mutableMapOf<Int, Boolean>()

    /**
     * Returns [data] with the latched bit set once this rocket has passed
     * apogee, otherwise unchanged.
     *
     * Rises the first time EITHER live vote fires — `velApo` typically leads
     * `altApo` by a few hundred ms — and holds until the next flight arms.
     * Clearing on PRELAUNCH/READY/INITIALIZATION matches
     * `FlightAnnouncer.resetFlightState()`, so a second flight on the same
     * connection starts clean.
     */
    fun apply(data: TelemetryData): TelemetryData {
        val key = data.sourceRocketId ?: 0
        if (data.state == "PRELAUNCH" || data.state == "READY" ||
            data.state == "INITIALIZATION"
        ) {
            seen[key] = false
        }
        if (data.altApo || data.velApo) seen[key] = true
        return if (seen[key] == true) {
            data.copy(
                flightStatusBits =
                    data.flightStatusBits or TelemetryData.PAST_APOGEE_BIT,
            )
        } else {
            data
        }
    }
}
