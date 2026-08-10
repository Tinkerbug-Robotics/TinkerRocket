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
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

private fun tempDir(): File =
    File.createTempFile("maps", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }

/** Serves fixed bytes for every tile request; counts hits; 404s on demand. */
private class FakeTileUpstream : AutoCloseable {
    val socket = ServerSocket(0, 50, InetAddress.getLoopbackAddress())
    val hits = AtomicInteger(0)
    @Volatile var respond404 = false
    /** Stall each response, so a run stays alive long enough to cancel it. */
    @Volatile var delayMs = 0L
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
                        if (delayMs > 0) Thread.sleep(delayMs)
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
    fun everyTileMissing_isFAILED_notAQuietlyEmptyRegion() {
        // The no-signal case this feature exists for.  It used to reach
        // FINISHED with 0 bytes and save a 0 MB area that looked real in the
        // list — the emptiness only surfaced at the field.
        upstream.respond404 = true
        var recorded = false
        downloader.start(region, "src") { _, _ -> recorded = true }
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FAILED, downloader.phase.value)
        assertEquals(8, downloader.done.value, "progress still completes")
        assertEquals(8, downloader.failed.value)
        assertEquals(0L, downloader.bytes.value)
        assertEquals(false, recorded, "a run that fetched nothing must not save a region")
        assertNull(cache.tileData("src", 12, 1195, 1551), "404 must not be cached")
    }

    @Test
    fun someTilesMissing_stillFinishes_butCountsTheFailures() {
        // A few edge tiles 404ing is normal and must not block the save; the
        // count is what tells the operator the area is partly covered.
        val all = TileMath.tiles(region)
        cache.store(upstream.body, "src", all[0].z, all[0].x, all[0].y)
        upstream.respond404 = true
        var recordedTiles = -1
        downloader.start(region, "src") { tiles, _ -> recordedTiles = tiles }
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FINISHED, downloader.phase.value)
        assertEquals(7, downloader.failed.value, "every uncached tile failed")
        assertEquals(8, recordedTiles, "the region is still recorded")
    }

    @Test
    fun unknownSource_isIgnored() {
        downloader.start(region, "nosuch")
        assertEquals(TileDownloader.Phase.IDLE, downloader.phase.value)
    }

    @Test
    fun cancel_removesOnlyWhatThisRunWrote() {
        // One tile is already on disk from an earlier, saved region. A cancel
        // must undo THIS run without punching a hole in that one.
        val all = TileMath.tiles(region)
        val preExisting = all.first()
        cache.store(upstream.body, "src", preExisting.z, preExisting.x, preExisting.y)

        upstream.delayMs = 40      // keep the run alive long enough to cancel
        downloader.start(region, "src")
        runBlocking { delay(60) }
        downloader.cancel()
        awaitTerminal()

        assertEquals(TileDownloader.Phase.CANCELLED, downloader.phase.value)
        assertContentEquals(
            upstream.body,
            cache.tileData("src", preExisting.z, preExisting.x, preExisting.y),
            "a cancel must not delete a tile it did not create",
        )
        // Nothing this run fetched is left behind: no region records those
        // tiles, so anything kept here could never be found or reclaimed.
        val leaked = all.filter { it != preExisting }
            .count { cache.tileData("src", it.z, it.x, it.y) != null }
        assertEquals(0, leaked, "cancelled run leaked $leaked tiles")
    }

    @Test
    fun finish_handsTotalsToTheCaller_soTheRegionOutlivesTheScreen() {
        // The downloader records the region itself; the screen only navigates.
        // When the UI owned this, backing out mid-run finished the download
        // into a manifest nobody wrote.
        var savedTiles = -1
        var savedBytes = -1L
        downloader.start(region, "src") { tiles, bytes -> savedTiles = tiles; savedBytes = bytes }
        awaitTerminal()

        assertEquals(TileDownloader.Phase.FINISHED, downloader.phase.value)
        assertEquals(8, savedTiles)
        assertEquals(8L * upstream.body.size, savedBytes)
    }

    @Test
    fun cancel_doesNotRecordARegion() {
        upstream.delayMs = 40
        var recorded = false
        downloader.start(region, "src") { _, _ -> recorded = true }
        runBlocking { delay(60) }
        downloader.cancel()
        awaitTerminal()

        assertEquals(TileDownloader.Phase.CANCELLED, downloader.phase.value)
        assertEquals(false, recorded, "a cancelled run must not save a region")
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
    fun delete_keepsTilesAnOverlappingRegionStillNeeds() {
        // Two saved areas over the same site share tile paths.  Deleting one
        // used to take the shared tiles with it, leaving the survivor quietly
        // incomplete — discovered offline at the field, which is the only
        // place it matters.
        val cache = OfflineTileCache(tempDir())
        val store = OfflineRegionStore(tempDir(), cache)
        val big = region(name = "Wide").copy(radiusMeters = 3_000.0)
        val small = region(name = "Tight").copy(radiusMeters = 800.0)
        val sharedTiles = TileMath.tiles(small.spec).toSet()
        assertTrue(
            TileMath.tiles(big.spec).toSet().containsAll(sharedTiles),
            "test premise: the small region is inside the big one",
        )
        (TileMath.tiles(big.spec) + TileMath.tiles(small.spec)).toSet().forEach {
            cache.store(byteArrayOf(1), big.source, it.z, it.x, it.y)
        }
        store.add(big)
        store.add(small)

        store.delete(big)

        // Everything the survivor covers is still on disk...
        sharedTiles.forEach {
            assertNotNull(
                cache.tileData(small.source, it.z, it.x, it.y),
                "deleting the wide area punched a hole in the tight one at $it",
            )
        }
        // ...and everything only the deleted area covered is gone.
        val onlyBig = TileMath.tiles(big.spec).filterNot { it in sharedTiles }
        assertTrue(onlyBig.isNotEmpty(), "test premise: the big region is strictly larger")
        assertEquals(
            0,
            onlyBig.count { cache.tileData(big.source, it.z, it.x, it.y) != null },
            "tiles no surviving region needs must be reclaimed",
        )
    }

    @Test
    fun delete_ignoresOverlapFromADifferentSource() {
        // Sources are separate directories, so identical z/x/y in another
        // source is a different file and must not protect anything.
        val cache = OfflineTileCache(tempDir())
        val store = OfflineRegionStore(tempDir(), cache)
        val mine = region(name = "Imagery")
        val otherSource = region(name = "Topo").copy(source = "usgsTopo")
        TileMath.tiles(mine.spec).forEach { cache.store(byteArrayOf(1), mine.source, it.z, it.x, it.y) }
        store.add(mine)
        store.add(otherSource)

        store.delete(mine)
        assertEquals(0L, cache.byteCount(mine.source), "another source must not pin these tiles")
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
