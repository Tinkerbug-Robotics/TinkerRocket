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
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Read-through semantics over a fake upstream: first hit fetches upstream
 * with the TRANSPOSED z/y/x path and persists at z/x/y; second hit serves
 * from disk; upstream failure serves the hatch placeholder and caches
 * nothing.
 */
class TileProxyServerTest {

    /** Minimal upstream: serves fixed bytes, records paths, 404s on demand. */
    private class FakeUpstream : AutoCloseable {
        val socket = ServerSocket(0, 10, InetAddress.getLoopbackAddress())
        val hits = AtomicInteger(0)
        @Volatile var lastPath: String? = null
        @Volatile var respond404 = false
        val body = byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 5, 6, 7) // JPEG magic

        val thread = Thread {
            while (!socket.isClosed) {
                val client = try { socket.accept() } catch (_: Exception) { break }
                client.use { c ->
                    val line = c.getInputStream().bufferedReader(Charsets.ISO_8859_1).readLine() ?: return@use
                    hits.incrementAndGet()
                    lastPath = line.split(" ").getOrNull(1)
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
            }
        }.apply { isDaemon = true; start() }

        override fun close() { socket.close() }
    }

    private val upstream = FakeUpstream()
    private val root = File.createTempFile("tiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
    private val cache = OfflineTileCache(root)
    private val proxy = TileProxyServer(
        cache = cache,
        upstreamTemplates = mapOf(
            "testsrc" to "http://127.0.0.1:${upstream.socket.localPort}/arcgis/tile/{z}/{y}/{x}",
        ),
    ).apply { start() }

    @AfterTest
    fun tearDown() {
        proxy.stop()
        upstream.close()
    }

    private fun get(path: String): Pair<Int, ByteArray> {
        val conn = URL("http://127.0.0.1:${proxy.port}$path").openConnection() as HttpURLConnection
        conn.connectTimeout = 5000
        conn.readTimeout = 5000
        val code = conn.responseCode
        val body = (if (code == 200) conn.inputStream else conn.errorStream)?.readBytes() ?: ByteArray(0)
        conn.disconnect()
        return code to body
    }

    @Test
    fun readThrough_fetchTransposesAndPersists_thenServesFromDisk() {
        val (code, body) = get("/tile/testsrc/12/1195/1551")
        assertEquals(200, code)
        assertContentEquals(upstream.body, body)
        // Upstream saw z/y/x (ArcGIS order)…
        assertEquals("/arcgis/tile/12/1551/1195", upstream.lastPath)
        // …but disk stores slippy z/x/y, iOS-identical.
        assertTrue(File(root, "testsrc/12/1195/1551.tile").isFile)

        val before = upstream.hits.get()
        val (code2, body2) = get("/tile/testsrc/12/1195/1551")
        assertEquals(200, code2)
        assertContentEquals(upstream.body, body2)
        assertEquals(before, upstream.hits.get(), "second hit must not touch upstream")
    }

    @Test
    fun upstreamFailure_servesPlaceholder_cachesNothing() {
        upstream.respond404 = true
        val (code, body) = get("/tile/testsrc/9/8/7")
        assertEquals(200, code)
        // PNG magic = the hatch placeholder, not upstream bytes.
        assertEquals(0x89.toByte(), body[0])
        assertNull(cache.tileData("testsrc", 9, 8, 7), "placeholder must never be cached")

        // Once upstream recovers, the real tile fills in.
        upstream.respond404 = false
        val (_, body2) = get("/tile/testsrc/9/8/7")
        assertContentEquals(upstream.body, body2)
        assertContentEquals(upstream.body, cache.tileData("testsrc", 9, 8, 7))
    }

    @Test
    fun unknownSource_servesPlaceholder() {
        val (code, body) = get("/tile/nosuch/1/0/0")
        assertEquals(200, code)
        assertEquals(0x89.toByte(), body[0]) // hatch placeholder
    }

    @Test
    fun malformedPath_is404() {
        assertEquals(404, get("/nope").first)
        assertEquals(404, get("/tile/testsrc/1/2").first)
    }
}
