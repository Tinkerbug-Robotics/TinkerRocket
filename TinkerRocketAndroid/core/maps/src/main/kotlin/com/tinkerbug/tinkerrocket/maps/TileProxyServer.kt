package com.tinkerbug.tinkerrocket.maps

import java.io.BufferedReader
import java.io.InputStreamReader
import java.net.HttpURLConnection
import java.net.InetAddress
import java.net.ServerSocket
import java.net.Socket
import java.net.URL
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Localhost read-through tile proxy (android-port plan §1 Maps row).
 *
 * MapLibre's raster sources fetch over HTTP with a slippy `{z}/{x}/{y}`
 * template; iOS's MKTileOverlay called straight into OfflineTileCache.  This
 * proxy is the Android bridge: it serves `GET /tile/<source>/<z>/<x>/<y>`
 * from the byte-identical flat-file cache, and on a miss fetches upstream
 * (ArcGIS `z/y/x` order — the transposition lives HERE and nowhere else),
 * persists, and returns.  Misses with no upstream (offline, 404 past max
 * zoom) return the same neutral hatch placeholder iOS renders, so offline
 * gaps read as intentional rather than a broken map.
 *
 * Loopback-only bind; one daemon accept thread + a small worker pool
 * (MapLibre requests many tiles concurrently).  Connection: close per
 * request — localhost, tile-sized bodies; keep-alive isn't worth the state.
 */
public class TileProxyServer(
    private val cache: OfflineTileCache,
    /** source key → upstream URL template; injectable for tests. */
    private val upstreamTemplates: Map<String, String> =
        TileSource.entries.mapNotNull { s -> s.urlTemplate?.let { s.key to it } }.toMap(),
    private val userAgent: String = "TinkerRocketApp/1.0 (offline map cache)",
    /**
     * Session-token upstream for [TileSource.GOOGLE_SATELLITE]; null when no
     * API key is configured — the source then serves placeholders.  Online-
     * only by provider terms: this path never reads or writes [cache].
     */
    private val googleUpstream: GoogleTileUpstream? = null,
) {
    private var serverSocket: ServerSocket? = null
    private val running = AtomicBoolean(false)
    private val pool = Executors.newFixedThreadPool(8) { r ->
        Thread(r, "tile-proxy").apply { isDaemon = true }
    }

    /** The bound port, valid after [start]. */
    public var port: Int = 0
        private set

    /** Neutral "not downloaded / no imagery" tile — same hatch as iOS. */
    private val placeholder: ByteArray =
        checkNotNull(javaClass.getResourceAsStream("/placeholder_tile.png")) {
            "placeholder_tile.png missing from resources"
        }.readBytes()

    public fun start() {
        if (running.getAndSet(true)) return
        // Explicit IPv4 loopback: on Android, getLoopbackAddress() can
        // resolve to ::1, which listens on IPv6 only — while the style
        // template (and MapLibre's OkHttp) dials 127.0.0.1.  Found live on
        // the Pixel as connection-refused with a bound socket.
        val loopback4 = InetAddress.getByAddress(byteArrayOf(127, 0, 0, 1))
        val socket = ServerSocket(0, 50, loopback4)
        serverSocket = socket
        port = socket.localPort
        Thread({
            while (running.get()) {
                val client = try {
                    socket.accept()
                } catch (_: Exception) {
                    break // socket closed by stop()
                }
                pool.execute { handle(client) }
            }
        }, "tile-proxy-accept").apply { isDaemon = true }.start()
    }

    public fun stop() {
        running.set(false)
        runCatching { serverSocket?.close() }
        pool.shutdownNow()
    }

    /** MapLibre raster-source template for a source, valid after [start]. */
    public fun templateFor(source: TileSource): String =
        "http://127.0.0.1:$port/tile/${source.key}/{z}/{x}/{y}"

    // ── Request handling ─────────────────────────────────────────────────

    private fun handle(client: Socket) {
        client.use { sock ->
            val reader = BufferedReader(InputStreamReader(sock.getInputStream(), Charsets.ISO_8859_1))
            val requestLine = reader.readLine() ?: return
            // Drain headers (ignored) up to the blank line.
            while (true) {
                val line = reader.readLine() ?: break
                if (line.isEmpty()) break
            }

            if (ATTRIBUTION_RE.matches(requestLine)) {
                val attr = googleUpstream?.attribution()
                if (attr != null) {
                    respond(sock, 200, "text/plain; charset=utf-8", attr.toByteArray(), noStore = true)
                } else {
                    respond(sock, 404, "text/plain", "no attribution".toByteArray())
                }
                return
            }

            val m = REQUEST_RE.matchEntire(requestLine)
            if (m == null) {
                respond(sock, 404, "text/plain", "not found".toByteArray())
                return
            }
            val (source, zs, xs, ys) = m.destructured
            val z = zs.toInt()
            val x = xs.toInt()
            val y = ys.toInt()

            // Online-only sources (provider terms forbid persistence) bypass
            // the offline cache in BOTH directions — a stale cached tile from
            // any earlier state must never serve — and carry no-store so
            // MapLibre's own HTTP cache doesn't persist them either.
            if (TileSource.fromKey(source)?.cacheable == false) {
                val fetched = if (source == TileSource.GOOGLE_SATELLITE.key) {
                    googleUpstream?.fetchTile(z, x, y)
                } else {
                    null
                }
                if (fetched != null) {
                    respond(sock, 200, contentType(fetched), fetched, noStore = true)
                } else {
                    respond(sock, 200, "image/png", placeholder, noStore = true)
                }
                return
            }

            cache.tileData(source, z, x, y)?.let {
                respond(sock, 200, contentType(it), it)
                return
            }

            val fetched = fetchUpstream(source, z, x, y)
            if (fetched != null) {
                cache.store(fetched, source, z, x, y)
                respond(sock, 200, contentType(fetched), fetched)
            } else {
                // Offline miss, 404 past max zoom, … → neutral placeholder
                // (200 so MapLibre renders it; never cached — a later online
                // pass must be able to fill the real tile).
                respond(sock, 200, "image/png", placeholder)
            }
        }
    }

    private fun fetchUpstream(source: String, z: Int, x: Int, y: Int): ByteArray? {
        val template = upstreamTemplates[source] ?: return null
        val url = template
            .replace("{z}", z.toString())
            .replace("{y}", y.toString())
            .replace("{x}", x.toString())
        return runCatching {
            val conn = URL(url).openConnection() as HttpURLConnection
            conn.setRequestProperty("User-Agent", userAgent)
            conn.connectTimeout = 10_000
            conn.readTimeout = 15_000
            try {
                if (conn.responseCode != 200) return@runCatching null
                conn.inputStream.readBytes()
            } finally {
                conn.disconnect()
            }
        }.getOrNull()
    }

    private fun respond(
        sock: Socket,
        code: Int,
        type: String,
        body: ByteArray,
        noStore: Boolean = false,
    ) {
        runCatching {
            val out = sock.getOutputStream()
            val status = if (code == 200) "200 OK" else "$code Error"
            val cacheControl = if (noStore) "Cache-Control: no-store\r\n" else ""
            out.write(
                (
                    "HTTP/1.1 $status\r\n" +
                        "Content-Type: $type\r\n" +
                        "Content-Length: ${body.size}\r\n" +
                        cacheControl +
                        "Access-Control-Allow-Origin: *\r\n" +
                        "Connection: close\r\n\r\n"
                    ).toByteArray(),
            )
            out.write(body)
            out.flush()
        }
    }

    /** Sniff PNG/JPEG magic — ArcGIS serves JPEG for imagery, PNG for topo. */
    private fun contentType(data: ByteArray): String = when {
        data.size >= 4 && data[0] == 0x89.toByte() && data[1] == 'P'.code.toByte() -> "image/png"
        data.size >= 2 && data[0] == 0xFF.toByte() && data[1] == 0xD8.toByte() -> "image/jpeg"
        else -> "application/octet-stream"
    }

    private companion object {
        val REQUEST_RE = Regex("""GET /tile/([A-Za-z0-9_]+)/(\d+)/(\d+)/(\d+) HTTP/1\.[01]""")

        /** Copyright string for the active Google session (display policy). */
        val ATTRIBUTION_RE = Regex("""GET /meta/googleSatellite/attribution HTTP/1\.[01]""")
    }
}
