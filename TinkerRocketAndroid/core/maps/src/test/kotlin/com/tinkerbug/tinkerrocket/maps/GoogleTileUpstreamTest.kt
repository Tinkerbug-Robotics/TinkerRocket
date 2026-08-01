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
import kotlin.test.assertIs
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
    val sessionAttempts = AtomicInteger(0)
    val tileCalls = AtomicInteger(0)
    val viewportCalls = AtomicInteger(0)
    val lastSessionHeaders = java.util.concurrent.ConcurrentHashMap<String, String>()
    val lastTileHeaders = java.util.concurrent.ConcurrentHashMap<String, String>()
    @Volatile var failSessions = false
    @Volatile var rejectAllTiles = false
    @Volatile var omitExpiry = false
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
                    val headers = mutableMapOf<String, String>()
                    while (true) {
                        val line = reader.readLine() ?: break
                        if (line.isEmpty()) break
                        val name = line.substringBefore(':').trim().lowercase()
                        headers[name] = line.substringAfter(':').trim()
                        if (name == "content-length") contentLength = headers[name]!!.toInt()
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
                            sessionAttempts.incrementAndGet()
                            lastSessionHeaders.clear()
                            lastSessionHeaders.putAll(headers)
                            if (failSessions) {
                                send(500, "boom".toByteArray())
                            } else {
                                val tok = "tok-${sessionCalls.incrementAndGet()}"
                                currentToken = tok
                                val expiryField = if (omitExpiry) "" else ""","expiry":"9999999999""""
                                send(
                                    200,
                                    ("""{"session":"$tok"$expiryField,""" +
                                        """"tileWidth":256,"tileHeight":256,"imageFormat":"jpeg"}""").toByteArray(),
                                )
                            }
                        }
                        path.startsWith("/v1/2dtiles/") -> {
                            tileCalls.incrementAndGet()
                            lastTileHeaders.clear()
                            lastTileHeaders.putAll(headers)
                            if (sessionParam == currentToken && !rejectAllTiles) {
                                send(200, tileBody)
                            } else {
                                send(403, ByteArray(0))
                            }
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

private fun okBytes(f: GoogleTileUpstream.TileFetch): ByteArray =
    assertIs<GoogleTileUpstream.TileFetch.Ok>(f).bytes

class GoogleTileUpstreamTest {

    private val api = FakeGoogleApi()

    @AfterTest
    fun tearDown() = api.close()

    @Test
    fun sessionCreatedLazily_thenReusedAcrossTiles() {
        val upstream = upstreamFor(api)
        assertEquals(0, api.sessionCalls.get(), "no session before first tile")
        repeat(3) { i -> assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, i))) }
        assertEquals(1, api.sessionCalls.get(), "one session shared by all tiles")
    }

    @Test
    fun deadSession_renewsOnceAndRetries() {
        val upstream = upstreamFor(api)
        assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, 3)))
        api.invalidateSessions()
        assertContentEquals(
            api.tileBody,
            okBytes(upstream.fetchTile(5, 4, 4)),
            "renew-and-retry must recover",
        )
        assertEquals(2, api.sessionCalls.get())
    }

    @Test
    fun createSessionFailure_isRetryThenRecovers() {
        api.failSessions = true
        val upstream = upstreamFor(api)
        assertIs<GoogleTileUpstream.TileFetch.Retry>(upstream.fetchTile(5, 4, 3))
        // Recovery once the API comes back.
        api.failSessions = false
        assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, 3)))
    }

    @Test
    fun concurrentFirstFetch_allWaitForOneSession() {
        // The first viewport: MapLibre fires many tile requests at once.
        // All must ride ONE createSession and return real imagery — the
        // losers wait on the latch instead of degrading to placeholders.
        val upstream = upstreamFor(api)
        val results = java.util.Collections.synchronizedList(
            mutableListOf<GoogleTileUpstream.TileFetch>(),
        )
        val threads = (0 until 6).map { i ->
            Thread { results.add(upstream.fetchTile(5, 4, i)) }.apply { start() }
        }
        threads.forEach { it.join(15_000) }
        assertEquals(6, results.size)
        results.forEach { assertContentEquals(api.tileBody, okBytes(it)) }
        assertEquals(1, api.sessionAttempts.get(), "one createSession shared by all racers")
    }

    @Test
    fun attribution_fetchedOnce_thenMemoized() {
        val upstream = upstreamFor(api)
        assertEquals(api.copyright, upstream.attribution())
        assertEquals(api.copyright, upstream.attribution())
        assertEquals(1, api.viewportCalls.get(), "second call must be memoized")
    }

    @Test
    fun attribution_renewsOnDeadSession() {
        val upstream = upstreamFor(api)
        assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, 3)))
        api.invalidateSessions()
        assertEquals(api.copyright, upstream.attribution(), "must renew-and-retry like fetchTile")
    }

    @Test
    fun noSession_createAttemptsHonorFloor_noSpamAndNoWedge() {
        // Dead network at the field: create attempts must be bounded by the
        // floor and every throttled call must return Retry IMMEDIATELY —
        // queued proxy workers may not stack behind serialized createSession.
        api.failSessions = true
        val upstream = GoogleTileUpstream(
            apiKey = "test-key",
            baseUrl = api.baseUrl,
            renewMinIntervalMs = 60_000,
        )
        repeat(10) { assertIs<GoogleTileUpstream.TileFetch.Retry>(upstream.fetchTile(5, 4, it)) }
        assertEquals(1, api.sessionAttempts.get(), "floor must bound createSession attempts")
    }

    @Test
    fun persistentTileRejection_renewsExactlyOnce() {
        // Revoked/misrestricted key: every token 403s.  fetchTile must do
        // initial + one renewed retry, then give up — never loop.
        api.rejectAllTiles = true
        val upstream = upstreamFor(api)
        assertIs<GoogleTileUpstream.TileFetch.Retry>(upstream.fetchTile(5, 4, 3))
        assertEquals(2, api.tileCalls.get(), "exactly initial + one retry")
        assertEquals(2, api.sessionCalls.get(), "exactly initial + one renewal")
    }

    @Test
    fun appAttestationHeaders_sentOnSessionAndTiles_whenConfigured() {
        val upstream = GoogleTileUpstream(
            apiKey = "test-key",
            baseUrl = api.baseUrl,
            renewMinIntervalMs = 0,
            appPackage = "com.tinkerbug.tinkerrocket",
            appCertSha1 = "E0DAC586432101BC48A5158E65908ECD15E2B432",
        )
        assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, 3)))
        assertEquals("com.tinkerbug.tinkerrocket", api.lastSessionHeaders["x-android-package"])
        assertEquals("E0DAC586432101BC48A5158E65908ECD15E2B432", api.lastSessionHeaders["x-android-cert"])
        assertEquals("com.tinkerbug.tinkerrocket", api.lastTileHeaders["x-android-package"])
        assertEquals("E0DAC586432101BC48A5158E65908ECD15E2B432", api.lastTileHeaders["x-android-cert"])
    }

    @Test
    fun appAttestationHeaders_absentByDefault() {
        val upstream = upstreamFor(api)
        okBytes(upstream.fetchTile(5, 4, 3))
        assertNull(api.lastTileHeaders["x-android-package"], "unconfigured upstream must not attest")
    }

    @Test
    fun missingExpiry_sessionStillReused() {
        // An absent expiry must fall back to a sane TTL, not "always
        // stale" — which would silently bill a createSession per tile.
        api.omitExpiry = true
        val upstream = upstreamFor(api)
        repeat(3) { assertContentEquals(api.tileBody, okBytes(upstream.fetchTile(5, 4, it))) }
        assertEquals(1, api.sessionCalls.get())
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
    fun googleUpstreamFailure_serves503NoStore() {
        // Transient failure → 503 so MapLibre retries with backoff; a 200
        // placeholder would stick for the whole style session.
        api.failSessions = true
        val (code, _, cacheControl) = get("/tile/googleSatellite/5/4/3")
        assertEquals(503, code)
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
