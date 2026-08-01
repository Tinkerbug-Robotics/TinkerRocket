package com.tinkerbug.tinkerrocket.maps

import java.io.File
import java.net.HttpURLConnection
import java.net.InetAddress
import java.net.ServerSocket
import java.net.URL
import java.util.concurrent.atomic.AtomicInteger
import kotlin.test.AfterTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Fake Map Tiles API endpoint: POST /v1/createSession mints tok-N and
 * rotates the accepted token; tile/viewport GETs 403 any other token.
 * Per-connection threads — the proxy pool fetches concurrently.
 */
private class FakeGoogleApi : AutoCloseable {
    val socket = ServerSocket(0, 10, InetAddress.getLoopbackAddress())
    val sessionCalls = AtomicInteger(0)
    val tileCalls = AtomicInteger(0)
    val viewportCalls = AtomicInteger(0)
    @Volatile var failSessions = false
    @Volatile var currentToken = "tok-0" // pre-rotation: nothing accepted yet
    val tileBody = byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 1, 2, 3) // JPEG magic
    val copyright = "Imagery ©2026 TerraMetrics"

    val baseUrl: String get() = "http://127.0.0.1:${socket.localPort}"

    /** Server-side invalidation: previously issued tokens now 403. */
    fun invalidateSessions() { currentToken = "tok-invalidated" }

    val thread = Thread {
        while (!socket.isClosed) {
            val client = try { socket.accept() } catch (_: Exception) { break }
            Thread {
                client.use { c ->
                    val reader = c.getInputStream().bufferedReader(Charsets.ISO_8859_1)
                    val requestLine = reader.readLine() ?: return@use
                    var contentLength = 0
                    while (true) {
                        val line = reader.readLine() ?: break
                        if (line.isEmpty()) break
                        if (line.startsWith("Content-Length:", ignoreCase = true)) {
                            contentLength = line.substringAfter(':').trim().toInt()
                        }
                    }
                    if (contentLength > 0) {
                        val buf = CharArray(contentLength)
                        var read = 0
                        while (read < contentLength) {
                            val n = reader.read(buf, read, contentLength - read)
                            if (n < 0) break
                            read += n
                        }
                    }
                    val out = c.getOutputStream()
                    fun send(code: Int, body: ByteArray) {
                        val status = if (code == 200) "200 OK" else "$code Error"
                        out.write(
                            ("HTTP/1.1 $status\r\nContent-Length: ${body.size}\r\n" +
                                "Connection: close\r\n\r\n").toByteArray(),
                        )
                        out.write(body)
                        out.flush()
                    }

                    val path = requestLine.split(" ").getOrNull(1).orEmpty()
                    val sessionParam = Regex("""[?&]session=([^&]+)""").find(path)?.groupValues?.get(1)
                    when {
                        requestLine.startsWith("POST /v1/createSession") -> {
                            if (failSessions) {
                                send(500, "boom".toByteArray())
                            } else {
                                val tok = "tok-${sessionCalls.incrementAndGet()}"
                                currentToken = tok
                                send(
                                    200,
                                    ("""{"session":"$tok","expiry":"9999999999",""" +
                                        """"tileWidth":256,"tileHeight":256,"imageFormat":"jpeg"}""").toByteArray(),
                                )
                            }
                        }
                        path.startsWith("/v1/2dtiles/") -> {
                            tileCalls.incrementAndGet()
                            if (sessionParam == currentToken) send(200, tileBody) else send(403, ByteArray(0))
                        }
                        path.startsWith("/tile/v1/viewport") -> {
                            viewportCalls.incrementAndGet()
                            if (sessionParam == currentToken) {
                                send(200, """{"copyright":"$copyright","maxZoomRects":[]}""".toByteArray())
                            } else {
                                send(403, ByteArray(0))
                            }
                        }
                        else -> send(404, ByteArray(0))
                    }
                }
            }.apply { isDaemon = true }.start()
        }
    }.apply { isDaemon = true; start() }

    override fun close() { socket.close() }
}

private fun upstreamFor(api: FakeGoogleApi): GoogleTileUpstream =
    GoogleTileUpstream(apiKey = "test-key", baseUrl = api.baseUrl, renewMinIntervalMs = 0)

class GoogleTileUpstreamTest {

    private val api = FakeGoogleApi()

    @AfterTest
    fun tearDown() = api.close()

    @Test
    fun sessionCreatedLazily_thenReusedAcrossTiles() {
        val upstream = upstreamFor(api)
        assertEquals(0, api.sessionCalls.get(), "no session before first tile")
        repeat(3) { i -> assertContentEquals(api.tileBody, upstream.fetchTile(5, 4, i)) }
        assertEquals(1, api.sessionCalls.get(), "one session shared by all tiles")
    }

    @Test
    fun deadSession_renewsOnceAndRetries() {
        val upstream = upstreamFor(api)
        assertContentEquals(api.tileBody, upstream.fetchTile(5, 4, 3))
        api.invalidateSessions()
        assertContentEquals(api.tileBody, upstream.fetchTile(5, 4, 4), "renew-and-retry must recover")
        assertEquals(2, api.sessionCalls.get())
    }

    @Test
    fun createSessionFailure_yieldsNullTile() {
        api.failSessions = true
        val upstream = upstreamFor(api)
        assertNull(upstream.fetchTile(5, 4, 3))
        // Recovery once the API comes back.
        api.failSessions = false
        assertContentEquals(api.tileBody, upstream.fetchTile(5, 4, 3))
    }

    @Test
    fun attribution_fetchedOnce_thenMemoized() {
        val upstream = upstreamFor(api)
        assertEquals(api.copyright, upstream.attribution())
        assertEquals(api.copyright, upstream.attribution())
        assertEquals(1, api.viewportCalls.get(), "second call must be memoized")
    }
}

/** googleSatellite through the proxy: online-only in both directions. */
class TileProxyServerGoogleTest {

    private val api = FakeGoogleApi()
    private val root = File.createTempFile("tiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
    private val cache = OfflineTileCache(root)
    private val proxy = TileProxyServer(
        cache = cache,
        upstreamTemplates = emptyMap(),
        googleUpstream = upstreamFor(api),
    ).apply { start() }

    @AfterTest
    fun tearDown() {
        proxy.stop()
        api.close()
    }

    private fun get(path: String): Triple<Int, ByteArray, String?> {
        val conn = URL("http://127.0.0.1:${proxy.port}$path").openConnection() as HttpURLConnection
        conn.connectTimeout = 5000
        conn.readTimeout = 5000
        val code = conn.responseCode
        val body = (if (code == 200) conn.inputStream else conn.errorStream)?.readBytes() ?: ByteArray(0)
        val cacheControl = conn.getHeaderField("Cache-Control")
        conn.disconnect()
        return Triple(code, body, cacheControl)
    }

    @Test
    fun googleTile_served_noStore_andNeverPersisted() {
        val (code, body, cacheControl) = get("/tile/googleSatellite/12/1195/1551")
        assertEquals(200, code)
        assertContentEquals(api.tileBody, body)
        assertEquals("no-store", cacheControl, "MapLibre's HTTP cache must be told not to persist")
        assertNull(cache.tileData("googleSatellite", 12, 1195, 1551), "provider terms: never cached")
        assertFalse(File(root, "googleSatellite").exists(), "no cache dir may even be created")
    }

    @Test
    fun staleCachedGoogleTile_isIgnoredNotServed() {
        // Hostile/legacy state: something put a googleSatellite tile on disk.
        val stale = byteArrayOf(0x89.toByte(), 'P'.code.toByte(), 9, 9)
        cache.store(stale, "googleSatellite", 5, 4, 3)
        val (_, body, _) = get("/tile/googleSatellite/5/4/3")
        assertContentEquals(api.tileBody, body, "must come from upstream, not the cache")
    }

    @Test
    fun googleUpstreamFailure_servesPlaceholderNoStore() {
        api.failSessions = true
        val (code, body, cacheControl) = get("/tile/googleSatellite/5/4/3")
        assertEquals(200, code)
        assertEquals(0x89.toByte(), body[0], "hatch placeholder on failure")
        assertEquals("no-store", cacheControl)
    }

    @Test
    fun attributionRoute_servesCopyright() {
        val (code, body, _) = get("/meta/googleSatellite/attribution")
        assertEquals(200, code)
        assertEquals(api.copyright, body.decodeToString())
    }

    @Test
    fun withoutGoogleUpstream_placeholderAndNoAttribution() {
        val bare = TileProxyServer(cache = cache, upstreamTemplates = emptyMap()).apply { start() }
        try {
            val conn = URL("http://127.0.0.1:${bare.port}/tile/googleSatellite/1/0/0")
                .openConnection() as HttpURLConnection
            assertEquals(200, conn.responseCode)
            assertEquals(0x89.toByte(), conn.inputStream.readBytes()[0])
            conn.disconnect()
            val meta = URL("http://127.0.0.1:${bare.port}/meta/googleSatellite/attribution")
                .openConnection() as HttpURLConnection
            assertEquals(404, meta.responseCode)
            meta.disconnect()
        } finally {
            bare.stop()
        }
    }
}

/** The online-only source must be structurally excluded from offline paths. */
class GoogleSourcePolicyTest {

    @Test
    fun enumPins() {
        val g = TileSource.GOOGLE_SATELLITE
        assertEquals("googleSatellite", g.key)
        assertNull(g.urlTemplate)
        assertFalse(g.cacheable)
        assertEquals(22, g.maxZoom)
        assertEquals(g, TileSource.fromKey("googleSatellite"))
        TileSource.entries.filter { it != g }.forEach {
            assertTrue(it.cacheable, "${it.key} must stay cacheable")
        }
    }

    @Test
    fun downloader_defaultTemplates_excludeGoogle() {
        val root = File.createTempFile("tiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
        val downloader = TileDownloader(OfflineTileCache(root))
        val region = RegionSpec(centerLat = 39.0, centerLon = -95.0, radiusMeters = 500.0, maxZoom = 12)
        downloader.start(region, TileSource.GOOGLE_SATELLITE.key)
        assertEquals(TileDownloader.Phase.IDLE, downloader.phase.value, "google region download must no-op")
    }
}
