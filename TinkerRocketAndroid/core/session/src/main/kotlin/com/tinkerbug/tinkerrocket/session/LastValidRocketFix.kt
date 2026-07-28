package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData

/**
 * Fix-cache / roster key: (networkID, rocketID).  Rocket IDs are only unique
 * per network — two BS/rocket pairs on different networks may reuse an id
 * (#390).  Both are wire u8s (0..255); 0 means unset/unknown for either.
 * Port of iOS `RocketKey` (RocketRoster.swift).
 */
public data class RocketKey(
    val networkId: Int,
    val rocketId: Int,
)

/**
 * Latched copy of the most recent usable rocket GPS fix.  Issue #140: the BS
 * keeps BLE-streaming to the phone even when its LoRa link to the rocket goes
 * quiet or carries a packet with no fresh GPS — those packets arrive with
 * lat/lon = null, which would blank the map marker.  A separate latched fix
 * only advances on a usable solution, so the marker stays parked on the last
 * reported position until a better one arrives.
 *
 * Semantics-exact port of iOS `LastValidRocketFix.swift`.
 */
public data class LastValidRocketFix(
    val rocketId: Int,
    val latitude: Double,
    val longitude: Double,
    val numSats: Int,
    /** When the fix was latched (injected clock — epoch millis). */
    val fixEpochMillis: Long,
) {
    public companion object {
        /**
         * A 3D GNSS solution needs four satellites; below this the position
         * can wander by hundreds of metres and isn't worth latching onto as
         * the rocket's "last known" location.  Tunable — lower if recovery
         * flights routinely produce sparser fixes.
         */
        public const val MIN_SATS_FOR_VALID_FIX: Int = 4

        /**
         * Build a fix from a freshly parsed telemetry packet if its GPS
         * fields look usable.  Returns null for packets missing GPS, sitting
         * at the (0, 0) origin, below the sat-count floor, or with no known
         * rocketID — those must not advance the cache.
         */
        public fun fromTelemetry(
            telemetry: TelemetryData,
            rocketId: Int,
            nowMillis: Long,
        ): LastValidRocketFix? {
            if (rocketId <= 0) return null
            val lat = telemetry.latitude ?: return null
            val lon = telemetry.longitude ?: return null
            if (lat == 0.0 && lon == 0.0) return null
            if (telemetry.numSats < MIN_SATS_FOR_VALID_FIX) return null
            return LastValidRocketFix(
                rocketId = rocketId,
                latitude = lat,
                longitude = lon,
                numSats = telemetry.numSats,
                fixEpochMillis = nowMillis,
            )
        }
    }
}
