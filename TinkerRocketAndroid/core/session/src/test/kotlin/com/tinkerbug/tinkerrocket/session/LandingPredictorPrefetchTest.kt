package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import java.io.File
import java.io.IOException
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * The wind-prefetch and attach/detach lifecycle of [LandingPredictor]
 * (#1044 #1051 #1056). The cast arithmetic is covered by LandingCastTest;
 * these pin the guard that used to fire a network request on every
 * preflight frame until one succeeded, and the attach() that used to wipe
 * the pinned prediction and cached wind on every reconnect.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class LandingPredictorPrefetchTest {
    private fun pad(lat: Double = 39.0, lon: Double = -76.1): TelemetryData =
        assertNotNull(TelemetryData.decode("""{"st":"READY","lat":$lat,"lon":$lon,"nsat":8}"""))

    private fun wind() = WindProfile(
        layers = listOf(WindLayer(0.0, 6.0, 250.0)),
        groundElevFt = 0.0, fetchTime = "", lat = 39.0, lon = -76.1,
    )

    private class MemActive : ActiveProfileStorage {
        var id: String? = null
        override fun loadActiveId() = id
        override fun saveActiveId(id: String?) { this.id = id }
    }

    private fun tempStore(): RocketProfileStore {
        val dir = File.createTempFile("profiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
        return RocketProfileStore(dir, MemActive()) { 1000 }
    }

    @Test
    fun oneFetchInFlight_furtherFramesDoNotLaunchAnother() = runTest {
        var calls = 0
        val gate = CompletableDeferred<WindProfile?>()
        val p = LandingPredictor(backgroundScope, windFetcher = { _, _ -> calls++; gate.await() }, clock = { currentTime })
        repeat(5) { p.handleTelemetry(pad()); runCurrent() }
        assertEquals(1, calls, "#1051: one request while one is in flight")
        gate.complete(wind())
        runCurrent()
        assertNotNull(p.windProfile.value)
        assertNull(p.windFetchError.value)
        repeat(5) { advanceTimeBy(1_000); p.handleTelemetry(pad()); runCurrent() }
        assertEquals(1, calls, "a fresh profile at this location suppresses the refetch for an hour")
    }

    @Test
    fun failure_backsOff_thenRetriesOnce() = runTest {
        var calls = 0
        val p = LandingPredictor(
            backgroundScope,
            windFetcher = { _, _ -> calls++; throw IOException("no route to host") },
            clock = { currentTime },
        )
        p.handleTelemetry(pad()); runCurrent()
        assertEquals(1, calls)
        assertEquals("no route to host", p.windFetchError.value)
        // The old guard retried on EVERY frame here (no profile yet).
        repeat(10) { advanceTimeBy(1_000); p.handleTelemetry(pad()); runCurrent() }
        assertEquals(1, calls, "no retry inside the 45 s backoff")
        advanceTimeBy(40_000); p.handleTelemetry(pad()); runCurrent()
        assertEquals(2, calls, "exactly one retry once the backoff has passed")
    }

    @Test
    fun nullResult_isSurfacedAndBackedOffLikeAFailure() = runTest {
        var calls = 0
        val p = LandingPredictor(backgroundScope, windFetcher = { _, _ -> calls++; null }, clock = { currentTime })
        p.handleTelemetry(pad()); runCurrent()
        assertNotNull(p.windFetchError.value, "a silent parse/HTTP failure is reported")
        repeat(5) { advanceTimeBy(1_000); p.handleTelemetry(pad()); runCurrent() }
        assertEquals(1, calls)
    }

    @Test
    fun attach_keepsTheWindAcrossASessionSwap_andResetsForAnotherDevice() = runTest {
        var calls = 0
        val store = tempStore()
        val p = LandingPredictor(backgroundScope, windFetcher = { _, _ -> calls++; wind() }, clock = { currentTime })
        val fw1 = FakeFirmware(backgroundScope)
        val s1 = DeviceSession(scope = backgroundScope, transport = fw1, connectedDeviceName = "TR-R-One", clock = { currentTime })
        s1.start()
        p.attach(s1, store, deviceKey = "dev:1")
        runCurrent()
        fw1.emitTelemetryJson("""{"st":"READY","lat":39.0,"lon":-76.1,"nsat":8}""")
        runCurrent()
        assertNotNull(p.windProfile.value)
        assertEquals(1, calls)
        // Reconnect: a NEW session object for the same device (the ladder
        // builds one per reconnect). #1056: the wind must survive, and the
        // fresh session must not trigger a refetch.
        p.detach()
        val fw2 = FakeFirmware(backgroundScope)
        val s2 = DeviceSession(scope = backgroundScope, transport = fw2, connectedDeviceName = "TR-R-One", clock = { currentTime })
        s2.start()
        p.attach(s2, store, deviceKey = "dev:1")
        runCurrent()
        assertNotNull(p.windProfile.value, "survives the session swap")
        fw2.emitTelemetryJson("""{"st":"READY","lat":39.0,"lon":-76.1,"nsat":8}""")
        runCurrent()
        assertEquals(1, calls, "no refetch after the swap")
        // A DIFFERENT device is a different rocket: start clean.
        val fw3 = FakeFirmware(backgroundScope)
        val s3 = DeviceSession(scope = backgroundScope, transport = fw3, connectedDeviceName = "TR-R-Two", clock = { currentTime })
        s3.start()
        p.attach(s3, store, deviceKey = "dev:2")
        runCurrent()
        assertNull(p.windProfile.value, "another device resets the retained state")
    }
}
