package com.tinkerbug.tinkerrocket.maps

import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class TileMathTest {

    @Test
    fun tileXY_goldenValues() {
        // Reference values from an independent Python slippy-map implementation.
        // (39.97154, -74.93655) is the bench board's GNSS fix.
        assertEquals(1195, TileMath.tileX(-74.93655, 12))
        assertEquals(1551, TileMath.tileY(39.97154, 12))
        assertEquals(19126, TileMath.tileX(-74.93655, 16))
        assertEquals(24817, TileMath.tileY(39.97154, 16))
        assertEquals(164, TileMath.tileX(-122.0, 10)) // the HIL sim fix
        assertEquals(394, TileMath.tileY(38.0, 10))
        assertEquals(1, TileMath.tileX(0.0, 1))
        assertEquals(1, TileMath.tileY(0.0, 1))
        // Near-pole / antimeridian extremes stay in range.
        assertEquals(15, TileMath.tileX(179.9, 4))
        assertEquals(0, TileMath.tileY(85.0, 4))
        assertEquals(0, TileMath.tileX(-179.9, 4))
        assertEquals(15, TileMath.tileY(-85.0, 4))
    }

    @Test
    fun cacheableSourcesDeclareTheDepthTheyActuallyServe() {
        // The save-area slider clamps to this.  USGS ArcGIS stops at 16:
        // z17/z18 404 every tile, and the downloader counts a 404 as done
        // with 0 bytes — so an over-deep range silently padded the estimate
        // with tiles that could never arrive.  If a source ever gains depth,
        // update it here and the slider follows.
        TileSource.entries.filter { it.cacheable }.forEach { s ->
            assertEquals(16, s.maxZoom, "${s.key} max zoom")
        }
    }

    @Test
    fun regionTiles_countMatchesGolden_andListMatchesCount() {
        val region = RegionSpec(
            centerLat = 39.97154, centerLon = -74.93655,
            radiusMeters = 800.0, minZoom = 10, maxZoom = 14,
        )
        assertEquals(8, TileMath.tileCount(region))
        val tiles = TileMath.tiles(region)
        assertEquals(TileMath.tileCount(region), tiles.size)
        assertEquals(tiles.size, tiles.toSet().size, "no duplicate tiles")
        // Every listed tile contains-or-borders the center's own tile column/row.
        for (z in 10..14) {
            val zt = tiles.filter { it.z == z }
            assertTrue(zt.isNotEmpty(), "zoom $z missing")
            val cx = TileMath.tileX(-74.93655, z)
            val cy = TileMath.tileY(39.97154, z)
            assertTrue(zt.any { it.x == cx && it.y == cy }, "center tile missing at z=$z")
        }
    }
}

class OfflineTileCacheTest {

    private fun tempRoot(): File =
        File.createTempFile("tiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }

    @Test
    fun diskLayout_isSourceZXY() {
        // THE transposition pin: the ArcGIS URL is z/y/x but disk is z/x/y —
        // the exact layout iOS writes (<src>/<z>/<x>/<y>.tile), so a future
        // cache import/export stays byte-compatible.
        val root = tempRoot()
        val cache = OfflineTileCache(root)
        cache.store(byteArrayOf(1, 2, 3), "usgsImageryTopo", z = 12, x = 1195, y = 1551)
        val expected = File(root, "usgsImageryTopo/12/1195/1551.tile")
        assertTrue(expected.isFile, "tile not at iOS-identical path")
        assertEquals(3, expected.length())
    }

    @Test
    fun roundTrip_byteCount_remove() {
        val cache = OfflineTileCache(tempRoot())
        assertNull(cache.tileData("s", 1, 2, 3))
        cache.store(ByteArray(100) { it.toByte() }, "s", 1, 2, 3)
        cache.store(ByteArray(50), "s", 1, 2, 4)
        assertEquals(100, cache.tileData("s", 1, 2, 3)?.size)
        assertEquals(150, cache.byteCount("s"))
        assertEquals(0, cache.byteCount("other"))
        cache.removeTiles("s", listOf(TileXYZ(1, 2, 3)))
        assertNull(cache.tileData("s", 1, 2, 3))
        assertEquals(50, cache.byteCount("s"))
    }
}
