package com.tinkerbug.tinkerrocket.maps

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withTimeout
import java.io.File
import java.net.InetAddress
import java.net.ServerSocket
import java.util.UUID
import java.util.concurrent.atomic.AtomicInteger
import kotlin.test.AfterTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

private fun tempDir(): File =
    File.createTempFile("maps", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }

/** Serves fixed bytes for every tile request; counts hits; 404s on demand. */
private class FakeTileUpstream : AutoCloseable {
    val socket = ServerSocket(0, 50, InetAddress.getLoopbackAddress())
    val hits = AtomicInteger(0)
    @Volatile var respond404 = false
    val body = byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 1, 2, 3, 4)

    val template = "http://127.0.0.1:${socket.localPort}/t/{z}/{y}/{x}"

    init {
        Thread {
            while (!socket.isClosed) {
                val client = try { socket.accept() } catch (_: Exception) { break }
                Thread {
                    client.use { c ->
                        c.getInputStream().bufferedReader(Charsets.ISO_8859_1).readLine() ?: return@use
                        hits.incrementAndGet()
                        val out = c.getOutputStream()
                        if (respond404) {
                            out.write("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".toByteArray())
                        } else {
                            out.write(
                                ("HTTP/1.1 200 OK\r\nContent-Length: ${body.size}\r\n" +
                                    "Connection: close\r\n\r\n").toByteArray(),
                            )
                            out.write(body)
                        }
                        out.flush()
                    }
                }.apply { isDaemon = true }.start()
            }
        }.apply { isDaemon = true; start() }
    }

    override fun close() { socket.close() }
}

class TileDownloaderTest {

    private val upstream = FakeTileUpstream()
    private val cache = OfflineTileCache(tempDir())
    private val downloader = TileDownloader(
        cache = cache,
        scope = CoroutineScope(Dispatchers.IO),
        upstreamTemplates = mapOf("src" to upstream.template),
    )
    // Small region: 8 tiles across z10–14 at the bench fix (pinned earlier).
    private val region = RegionSpec(
        centerLat = 39.97154, centerLon = -74.93655,
        radiusMeters = 800.0, minZoom = 10, maxZoom = 14,
    )

    @AfterTest
    fun tearDown() {
        upstream.close()
    }

    private fun awaitTerminal() = runBlocking {
        withTimeout(15_000) {
            while (downloader.phase.value == TileDownloader.Phase.DOWNLOADING) delay(20)
        }
    }

    @Test
    fun downloadsRegion_progressCompletes_tilesPersisted() {
        downloader.start(region, "src")
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FINISHED, downloader.phase.value)
        assertEquals(8, downloader.total.value)
        assertEquals(8, downloader.done.value)
        assertEquals(8L * upstream.body.size, downloader.bytes.value)
        // Every tile of the region is now on disk.
        TileMath.tiles(region).forEach { t ->
            assertContentEquals(upstream.body, cache.tileData("src", t.z, t.x, t.y), "missing $t")
        }
    }

    @Test
    fun resumable_cachedTilesCountWithoutUpstreamHits() {
        // Pre-seed every tile; a re-download must not touch upstream.
        TileMath.tiles(region).forEach { cache.store(upstream.body, "src", it.z, it.x, it.y) }
        downloader.start(region, "src")
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FINISHED, downloader.phase.value)
        assertEquals(8, downloader.done.value)
        assertEquals(0, upstream.hits.get(), "cached tiles must not re-fetch")
    }

    @Test
    fun missingTiles_countAsZeroBytes_progressStillCompletes() {
        upstream.respond404 = true
        downloader.start(region, "src")
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FINISHED, downloader.phase.value)
        assertEquals(8, downloader.done.value)
        assertEquals(0L, downloader.bytes.value)
        assertNull(cache.tileData("src", 12, 1195, 1551), "404 must not be cached")
    }

    @Test
    fun unknownSource_isIgnored() {
        downloader.start(region, "nosuch")
        assertEquals(TileDownloader.Phase.IDLE, downloader.phase.value)
    }
}

class OfflineRegionStoreTest {

    private fun region(name: String = "Pad A", savedAtMs: Long = 1_753_600_000_000L) = OfflineRegion(
        name = name,
        lat = 39.97154, lon = -74.93655,
        radiusMeters = 800.0, minZoom = 10, maxZoom = 14,
        source = "usgsImageryTopo", tileCount = 8, bytes = 200_000L,
        savedAtMs = savedAtMs,
    )

    @Test
    fun roundTrip_iosSchema() {
        val dir = tempDir()
        val cache = OfflineTileCache(tempDir())
        val store = OfflineRegionStore(dir, cache)
        val r = region()
        store.add(r)

        // Schema pins: uppercase UUID string + Apple-epoch savedAt seconds.
        val text = File(dir, "offline_regions.json").readText()
        assertTrue(r.id.toString().uppercase() in text, "uppercase UUID")
        val appleSeconds = r.savedAtMs / 1000.0 - 978_307_200.0
        assertTrue(text.contains("\"savedAt\":${appleSeconds}"), "Apple-epoch savedAt in: $text")

        val reloaded = OfflineRegionStore(dir, cache)
        assertEquals(listOf(r), reloaded.regions.value)
        assertEquals(200_000L, reloaded.totalBytes)
    }

    @Test
    fun add_dedupsById_insertsNewestFirst() {
        val store = OfflineRegionStore(tempDir(), OfflineTileCache(tempDir()))
        val a = region("A")
        store.add(a)
        store.add(region("B"))
        assertEquals(listOf("B", "A"), store.regions.value.map { it.name })
        store.add(a.copy(name = "A2"))
        assertEquals(listOf("A2", "B"), store.regions.value.map { it.name })
    }

    @Test
    fun delete_removesTilesAndManifestEntry() {
        val cacheDir = tempDir()
        val cache = OfflineTileCache(cacheDir)
        val store = OfflineRegionStore(tempDir(), cache)
        val r = region()
        TileMath.tiles(r.spec).forEach { cache.store(byteArrayOf(1), r.source, it.z, it.x, it.y) }
        store.add(r)

        store.delete(r)
        assertEquals(emptyList(), store.regions.value)
        assertEquals(0L, cache.byteCount(r.source), "tiles must be reclaimed")
    }

    @Test
    fun corruptManifest_startsEmpty() {
        val dir = tempDir()
        File(dir, "offline_regions.json").writeText("{not json[")
        val store = OfflineRegionStore(dir, OfflineTileCache(tempDir()))
        assertEquals(emptyList(), store.regions.value)
    }

    @Test
    fun regionIdRoundTrip() {
        // decode(encode(r)) preserves the id exactly even though encode
        // uppercases — UUID.fromString is case-insensitive.
        val dir = tempDir()
        val cache = OfflineTileCache(tempDir())
        val id = UUID.fromString("a1b2c3d4-e5f6-4a01-9b23-456789abcdef")
        OfflineRegionStore(dir, cache).add(region().copy(id = id))
        assertEquals(id, OfflineRegionStore(dir, cache).regions.value.single().id)
    }
}
