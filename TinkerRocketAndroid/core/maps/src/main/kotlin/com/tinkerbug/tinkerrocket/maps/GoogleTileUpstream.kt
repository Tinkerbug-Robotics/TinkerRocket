package com.tinkerbug.tinkerrocket.maps

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.net.HttpURLConnection
import java.net.URL

/**
 * Authenticated upstream for [TileSource.GOOGLE_SATELLITE] — Google Map
 * Tiles API (2D satellite tiles).  Unlike the template sources, Google
 * requires a session token: `POST /v1/createSession` once (valid ~2 weeks),
 * then `GET /v1/2dtiles/{z}/{x}/{y}?session=…&key=…` per tile.
 *
 * Provider terms forbid persisting these tiles, so this class is the ONLY
 * fetch path for the source and [TileProxyServer] never lets its responses
 * near [OfflineTileCache].  The API key is compiled into the app (any
 * client-side Maps key ships in the APK) and is API-restricted server-side;
 * forks without a key simply never construct this class.
 *
 * [baseUrl] is injectable for tests, same idiom as the proxy's template map.
 */
public class GoogleTileUpstream(
    private val apiKey: String,
    private val baseUrl: String = "https://tile.googleapis.com",
    private val userAgent: String = "TinkerRocketApp/1.0 (offline map cache)",
    /** Floor between forced renewals — bounds createSession spam when a run
     *  of tiles genuinely 4xxes.  Tests pass 0 for deterministic renewal. */
    private val renewMinIntervalMs: Long = 5_000,
) {
    private class Session(val token: String, val expiryEpochSec: Long)

    private val lock = Any()
    private var session: Session? = null
    private var lastCreateAttemptMs: Long = 0
    private var cachedAttribution: String? = null
    private var createLatch: java.util.concurrent.CountDownLatch? = null

    /** Outcome of a tile fetch — the proxy maps these to HTTP semantics. */
    public sealed interface TileFetch {
        public class Ok(public val bytes: ByteArray) : TileFetch

        /**
         * Transient — session pending, network hiccup, upstream 5xx.  The
         * proxy answers 503 so MapLibre retries with backoff; a 200
         * placeholder would be sticky for the whole style session.
         */
        public object Retry : TileFetch

        /** Definitive no-imagery (404) — render the no-data placeholder. */
        public object NoData : TileFetch
    }

    /**
     * Fetch one satellite tile (slippy z/x/y).  A 4xx that looks like a
     * dead session triggers ONE renew-and-retry, never a loop.
     */
    public fun fetchTile(z: Int, x: Int, y: Int): TileFetch {
        val token = ensureSession(force = false) ?: return TileFetch.Retry
        val first = getTile(z, x, y, token)
        first.body?.let { return TileFetch.Ok(it) }
        if (first.code in SESSION_DEAD_CODES) {
            val renewed = ensureSession(force = true) ?: return TileFetch.Retry
            val second = getTile(z, x, y, renewed)
            second.body?.let { return TileFetch.Ok(it) }
            return if (second.code == 404) TileFetch.NoData else TileFetch.Retry
        }
        return if (first.code == 404) TileFetch.NoData else TileFetch.Retry
    }

    /**
     * The copyright string the Map Tiles API display policy requires while
     * these tiles are shown.  Fetched from the viewport endpoint at a
     * world-spanning bound and memoized; null until a fetch succeeds.
     * Mirrors [fetchTile]'s one renew-and-retry on a dead session.
     */
    public fun attribution(): String? {
        synchronized(lock) { cachedAttribution?.let { return it } }
        val token = ensureSession(force = false) ?: return null
        var result = getViewport(token)
        if (result.body == null && result.code in SESSION_DEAD_CODES) {
            val renewed = ensureSession(force = true) ?: return null
            result = getViewport(renewed)
        }
        val body = result.body ?: return null
        val copyright = runCatching {
            Json.parseToJsonElement(body.decodeToString())
                .jsonObject["copyright"]?.jsonPrimitive?.content
        }.getOrNull() ?: return null
        synchronized(lock) { cachedAttribution = copyright }
        return copyright
    }

    // ── Session lifecycle ────────────────────────────────────────────────

    /**
     * Returns a usable token, creating a session if absent, expired, or
     * [force]d.  createSession runs OUTSIDE the lock and single-flight:
     * one thread creates while concurrent callers WAIT on a latch, bounded
     * by [SESSION_WAIT_MS] — on a healthy network the whole first viewport
     * of tiles just waits ~300 ms for the one createSession and renders
     * real imagery, while on a dead network waiters unblock quickly and
     * the proxy's fixed worker pool can't wedge for minutes.  EVERY create
     * attempt (not just forced renewals) honors the [renewMinIntervalMs]
     * floor, so a dead network can't spam billed createSession calls.
     */
    private fun ensureSession(force: Boolean): String? {
        val now = System.currentTimeMillis()
        var awaitLatch: java.util.concurrent.CountDownLatch? = null
        synchronized(lock) {
            val current = session
            val fresh = current != null && now / 1000 < current.expiryEpochSec - EXPIRY_MARGIN_SEC
            if (fresh && !force) return current.token
            val inFlight = createLatch
            if (inFlight != null) {
                awaitLatch = inFlight
            } else {
                if (now - lastCreateAttemptMs < renewMinIntervalMs) {
                    return if (fresh) current?.token else null
                }
                lastCreateAttemptMs = now
                createLatch = java.util.concurrent.CountDownLatch(1)
            }
        }
        awaitLatch?.let { latch ->
            latch.await(SESSION_WAIT_MS, java.util.concurrent.TimeUnit.MILLISECONDS)
            synchronized(lock) {
                val current = session
                val fresh = current != null &&
                    System.currentTimeMillis() / 1000 < current.expiryEpochSec - EXPIRY_MARGIN_SEC
                return if (fresh) current.token else null
            }
        }
        try {
            val created = createSession()
            synchronized(lock) { if (created != null) session = created }
            return created?.token
        } finally {
            synchronized(lock) {
                createLatch?.countDown()
                createLatch = null
            }
        }
    }

    private fun createSession(): Session? = runCatching {
        val conn = URL("$baseUrl/v1/createSession?key=$apiKey").openConnection() as HttpURLConnection
        conn.requestMethod = "POST"
        conn.setRequestProperty("User-Agent", userAgent)
        conn.setRequestProperty("Content-Type", "application/json")
        conn.connectTimeout = 10_000
        conn.readTimeout = 15_000
        conn.doOutput = true
        try {
            conn.outputStream.use { it.write(SESSION_REQUEST_BODY.toByteArray()) }
            if (conn.responseCode != 200) return@runCatching null
            val obj = Json.parseToJsonElement(conn.inputStream.readBytes().decodeToString()).jsonObject
            val token = obj["session"]?.jsonPrimitive?.content ?: return@runCatching null
            // Docs serialize expiry as a string of epoch seconds; be lenient —
            // an absent/unparseable expiry must NOT read as "already stale"
            // (that would silently pair every tile with a billed
            // createSession), so fall back to a conservative week (the
            // documented lifetime is ~two weeks).
            val expiry = obj["expiry"]?.jsonPrimitive?.content?.toLongOrNull()
                ?: (System.currentTimeMillis() / 1000 + FALLBACK_TTL_SEC)
            Session(token, expiry)
        } finally {
            conn.disconnect()
        }
    }.getOrNull()

    private class TileResult(val code: Int, val body: ByteArray?)

    private fun getTile(z: Int, x: Int, y: Int, token: String): TileResult = runCatching {
        httpGet("$baseUrl/v1/2dtiles/$z/$x/$y?session=$token&key=$apiKey") { conn ->
            val code = conn.responseCode
            TileResult(code, if (code == 200) conn.inputStream.readBytes() else null)
        }
    }.getOrElse { TileResult(-1, null) }

    private fun getViewport(token: String): TileResult = runCatching {
        httpGet(
            "$baseUrl/tile/v1/viewport?session=$token&key=$apiKey" +
                "&zoom=1&north=85&south=-85&east=180&west=-180",
        ) { conn ->
            val code = conn.responseCode
            TileResult(code, if (code == 200) conn.inputStream.readBytes() else null)
        }
    }.getOrElse { TileResult(-1, null) }

    private fun <T> httpGet(url: String, read: (HttpURLConnection) -> T): T {
        val conn = URL(url).openConnection() as HttpURLConnection
        conn.setRequestProperty("User-Agent", userAgent)
        conn.connectTimeout = 10_000
        conn.readTimeout = 15_000
        try {
            return read(conn)
        } finally {
            conn.disconnect()
        }
    }

    private companion object {
        const val SESSION_REQUEST_BODY =
            """{"mapType":"satellite","language":"en-US","region":"US"}"""

        /** Renew this long before the server-reported expiry. */
        const val EXPIRY_MARGIN_SEC = 3_600L

        /** Session TTL assumed when the response omits a parseable expiry. */
        const val FALLBACK_TTL_SEC = 7L * 24 * 3_600

        /** Longest a tile fetch waits for another thread's createSession. */
        const val SESSION_WAIT_MS = 6_000L

        /**
         * Codes that plausibly mean "session token dead" (expired/revoked).
         * 404 is excluded — that's missing imagery, not a dead session.
         */
        val SESSION_DEAD_CODES = setOf(400, 401, 403)
    }
}
