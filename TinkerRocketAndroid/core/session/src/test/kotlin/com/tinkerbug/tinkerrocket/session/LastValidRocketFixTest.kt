package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * Issue #140: the map must keep showing the last reported rocket position
 * even when a fresh telemetry packet arrives without GPS (LoRa loss or BLE
 * drop during flight).  Port of ALL iOS LastValidRocketFixTests cases —
 * the validity check plus the fleet-level cache-preservation behavior.
 */
class LastValidRocketFixTest {

    private fun telemetry(
        lat: Double? = null,
        lon: Double? = null,
        sats: Int = 0,
    ) = TelemetryData(latitude = lat, longitude = lon, numSats = sats)

    /** Fix-cache keys are network-scoped (#390) — nid 5 is arbitrary. */
    private fun key(rid: Int, nid: Int = 5) = RocketKey(networkId = nid, rocketId = rid)

    // ------------------------------------------------ fromTelemetry validity

    @Test
    fun fromTelemetry_validFix_returned() {
        val fix = assertNotNull(
            LastValidRocketFix.fromTelemetry(telemetry(33.7, -118.4, 8), rocketId = 1, nowMillis = 42L),
        )
        assertEquals(33.7, fix.latitude, 1e-9)
        assertEquals(-118.4, fix.longitude, 1e-9)
        assertEquals(8, fix.numSats)
        assertEquals(1, fix.rocketId)
        assertEquals(42L, fix.fixEpochMillis)
    }

    @Test
    fun fromTelemetry_missingLat_rejected() {
        // The BS firmware omits NAN lat/lon from JSON to save MTU, so a real
        // "no fix" packet arrives with latitude == null.  Must not advance
        // the cache — the whole point of issue #140.
        assertNull(LastValidRocketFix.fromTelemetry(telemetry(lon = -118.4, sats = 8), 1, 0L))
    }

    @Test
    fun fromTelemetry_missingLon_rejected() {
        assertNull(LastValidRocketFix.fromTelemetry(telemetry(lat = 33.7, sats = 8), 1, 0L))
    }

    @Test
    fun fromTelemetry_zeroOrigin_rejected() {
        // (0, 0) is in the Gulf of Guinea — an unrealistic rocket position
        // that almost always means "no fix" leaking through.
        assertNull(LastValidRocketFix.fromTelemetry(telemetry(0.0, 0.0, 8), 1, 0L))
    }

    @Test
    fun fromTelemetry_tooFewSats_rejected() {
        assertNull(LastValidRocketFix.fromTelemetry(telemetry(33.7, -118.4, 3), 1, 0L))
    }

    @Test
    fun fromTelemetry_minSatsBoundary_accepted() {
        // The 4-sat floor is the boundary — anything >= 4 must pass so a
        // freshly acquired 3D fix can populate the cache.
        assertNotNull(
            LastValidRocketFix.fromTelemetry(
                telemetry(33.7, -118.4, LastValidRocketFix.MIN_SATS_FOR_VALID_FIX), 1, 0L,
            ),
        )
    }

    @Test
    fun fromTelemetry_zeroRocketId_rejected() {
        // rocketID == 0 means "unknown rocket" (the default) — never cache
        // against that key or all rockets share the same slot.
        assertNull(LastValidRocketFix.fromTelemetry(telemetry(33.7, -118.4, 8), 0, 0L))
    }

    // -------------------------------- FleetManager.recordRocketFix behavior

    @Test
    fun recordRocketFix_validPacket_populatesCache() = runTest {
        val h = fleetHarness()
        assertNotNull(h.fleet.recordRocketFix(telemetry(33.7, -118.4, 8), key(1)))
        val cached = assertNotNull(h.fleet.lastValidRocketFix(key(1)))
        assertEquals(33.7, cached.latitude, 1e-9)
    }

    @Test
    fun recordRocketFix_invalidPacketWithCached_preservesCache() = runTest {
        // THE bug fix for #140.  Once a valid fix is cached, an incoming
        // GPS-less packet must not blow it away — the map must keep showing
        // the last reported position.
        val h = fleetHarness()
        h.fleet.recordRocketFix(telemetry(33.7, -118.4, 8), key(1))

        val result = assertNotNull(
            h.fleet.recordRocketFix(telemetry(), key(1)),
            "A GPS-less packet must return the pre-existing fix",
        )
        assertEquals(33.7, result.latitude, 1e-9)
        assertEquals(33.7, assertNotNull(h.fleet.lastValidRocketFix(key(1))).latitude, 1e-9)
    }

    @Test
    fun recordRocketFix_newerValidReplacesOlder() = runTest {
        // Newer always wins so a recovery-grade fix isn't stuck on a
        // pre-launch one even if both meet the minimum quality bar.
        val h = fleetHarness()
        h.fleet.recordRocketFix(telemetry(33.7, -118.4, 4), key(1))
        h.fleet.recordRocketFix(telemetry(34.0, -118.5, 10), key(1))

        val cached = assertNotNull(h.fleet.lastValidRocketFix(key(1)))
        assertEquals(34.0, cached.latitude, 1e-9)
        assertEquals(10, cached.numSats)
    }

    @Test
    fun recordRocketFix_multipleRockets_independentSlots() = runTest {
        val h = fleetHarness()
        h.fleet.recordRocketFix(telemetry(33.7, -118.4, 8), key(1))
        h.fleet.recordRocketFix(telemetry(47.6, -122.3, 6), key(2))

        assertEquals(33.7, assertNotNull(h.fleet.lastValidRocketFix(key(1))).latitude, 1e-9)
        assertEquals(47.6, assertNotNull(h.fleet.lastValidRocketFix(key(2))).latitude, 1e-9)
    }

    @Test
    fun recordRocketFix_sameRidDifferentNetwork_independentSlots() = runTest {
        // #390 two-pair support: "rocket 1" on network 5 and "rocket 1" on
        // network 9 are different physical rockets and must never share a
        // cache slot.
        val h = fleetHarness()
        h.fleet.recordRocketFix(telemetry(33.7, -118.4, 8), key(1, nid = 5))
        h.fleet.recordRocketFix(telemetry(47.6, -122.3, 6), key(1, nid = 9))

        assertEquals(33.7, assertNotNull(h.fleet.lastValidRocketFix(key(1, nid = 5))).latitude, 1e-9)
        assertEquals(47.6, assertNotNull(h.fleet.lastValidRocketFix(key(1, nid = 9))).latitude, 1e-9)
    }

    @Test
    fun recordRocketFix_emptyCacheAndInvalidPacket_staysEmpty() = runTest {
        val h = fleetHarness()
        assertNull(h.fleet.recordRocketFix(telemetry(), key(1)))
        assertNull(h.fleet.lastValidRocketFix(key(1)))
    }

    @Test
    fun recordRocketFix_usesInjectedClock() = runTest {
        val h = fleetHarness()
        testScheduler.advanceTimeBy(1234)
        val fix = assertNotNull(h.fleet.recordRocketFix(telemetry(33.7, -118.4, 8), key(1)))
        assertEquals(1234L, fix.fixEpochMillis, "No wall-clock reads — virtual time only")
    }
}
