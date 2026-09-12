package com.tinkerbug.tinkerrocket.maps

import java.io.File
import kotlin.math.abs
import kotlin.math.cos
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1092 item 1 — the ground texture under the 3D scenes, the twin of iOS
 * `SceneGroundTexture` and of its tests.  The endpoint, the key and the
 * cache-first order are the parity; the bbox numbers are the geometry the
 * app's textured quad relies on.
 */
class SceneGroundTextureTest {
    private val padLat = 40.1
    private val padLon = -105.2

    private fun tempCache() = OfflineTileCache(
        File.createTempFile("blobs", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } },
    )

    /** THE regression iOS pinned: the endpoint is the public-domain USGS one. */
    @Test
    fun usesTheUsgsEndpointNotEsri() {
        val req = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 500.0))
        assertTrue(req.url.startsWith(
            "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export",
        ), "not the USGS endpoint: ${req.url}")
        assertTrue("arcgisonline.com" !in req.url, "Esri World Imagery cannot be cached offline under its free terms")
        assertTrue("World_Imagery" !in req.url)
        assertTrue("&size=1024,1024&format=png&f=image" in req.url)
    }

    @Test
    fun theBboxIsOneAndAHalfExtentsEitherSideOfTheReference() {
        // Same numbers as iOS: 110 540 m per degree of latitude, 111 320 m
        // per degree of longitude at the equator, scaled by cos(lat).
        val req = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 500.0))
        assertEquals(750.0, req.halfM)
        val parts = req.url.substringAfter("bbox=").substringBefore("&").split(",").map { it.toDouble() }
        val (west, south, east, north) = parts
        assertEquals(padLat - 750.0 / 110_540.0, south, 1e-9)
        assertEquals(padLat + 750.0 / 110_540.0, north, 1e-9)
        val mPerDegLon = 111_320.0 * cos(Math.toRadians(padLat))
        assertEquals(padLon - 750.0 / mPerDegLon, west, 1e-9)
        assertEquals(padLon + 750.0 / mPerDegLon, east, 1e-9)
        assertTrue(abs((east - west) * mPerDegLon - 1500.0) < 1e-6, "the square is 3 extents wide in metres")
    }

    /** The key is the bbox alone, so two views of one site share one texture. */
    @Test
    fun theKeyIsStableAndSharedAcrossViewsOfOneSite() {
        val a = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 800.0))
        val b = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 800.0))
        assertEquals(a.cacheKey, b.cacheKey)
        assertTrue(a.cacheKey.startsWith("3d_"), "iOS key shape: ${a.cacheKey}")
        assertEquals("3d_" + a.url.substringAfter("bbox=").substringBefore("&"), a.cacheKey)
    }

    @Test
    fun differentSitesAndDifferentExtentsGetDifferentKeys() {
        val here = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 500.0))
        val there = assertNotNull(SceneGroundTexture.request(39.0, -104.0, extent = 500.0))
        val wider = assertNotNull(SceneGroundTexture.request(padLat, padLon, extent = 2000.0))
        assertNotEquals(here.cacheKey, there.cacheKey)
        assertNotEquals(here.cacheKey, wider.cacheKey, "zooming out is a different texture, not the same one stretched")
    }

    @Test
    fun refusesWhatItCannotPlace() {
        assertNull(SceneGroundTexture.request(Double.NaN, padLon, 500.0))
        assertNull(SceneGroundTexture.request(padLat, Double.POSITIVE_INFINITY, 500.0))
        assertNull(SceneGroundTexture.request(padLat, padLon, 0.0))
        // (cos(90°) is 6e-17 in floating point, not zero — the pole is not
        // refused on either platform; nobody launches there.)
    }

    /** Cache first: a scene viewed once renders offline later, and never re-downloads. */
    @Test
    fun loadIsCacheFirstThenFetchThenStore() {
        val cache = tempCache()
        val req = assertNotNull(SceneGroundTexture.request(padLat, padLon, 500.0))
        var fetches = 0
        val fetch: (String) -> ByteArray? = { url -> fetches++; assertEquals(req.url, url); byteArrayOf(9, 8, 7) }

        val first = SceneGroundTexture.load(cache, req, fetch)
        assertEquals(listOf<Byte>(9, 8, 7), first?.toList())
        assertEquals(1, fetches)
        assertEquals(listOf<Byte>(9, 8, 7), cache.blob(req.cacheKey)?.toList(), "stored under the key")

        val second = SceneGroundTexture.load(cache, req) { error("must not fetch: it is cached") }
        assertEquals(listOf<Byte>(9, 8, 7), second?.toList())
        assertEquals(1, fetches)
    }

    /** No imagery is no imagery: nothing stored, the scene keeps its grid. */
    @Test
    fun aFailedFetchStoresNothing() {
        val cache = tempCache()
        val req = assertNotNull(SceneGroundTexture.request(padLat, padLon, 500.0))
        assertNull(SceneGroundTexture.load(cache, req) { null })
        assertNull(cache.blob(req.cacheKey))
    }
}
