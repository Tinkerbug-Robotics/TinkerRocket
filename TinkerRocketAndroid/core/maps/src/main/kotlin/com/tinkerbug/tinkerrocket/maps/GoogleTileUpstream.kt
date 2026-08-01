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
    private var lastCreateMs: Long = 0
    private var cachedAttribution: String? = null

    /**
     * Fetch one satellite tile (slippy z/x/y).  Null on any failure — the
     * proxy then serves its placeholder, matching template-source semantics.
     * A 4xx that looks like a dead session triggers ONE renew-and-retry.
     */
    public fun fetchTile(z: Int, x: Int, y: Int): ByteArray? {
        val token = ensureSession(force = false) ?: return null
        val first = getTile(z, x, y, token)
        if (first.body != null) return first.body
        if (first.code !in SESSION_DEAD_CODES) return null
        val renewed = ensureSession(force = true) ?: return null
        return getTile(z, x, y, renewed).body
    }

    /**
     * The copyright string the Map Tiles API display policy requires while
     * these tiles are shown.  Fetched once from the viewport endpoint at a
     * world-spanning bound and memoized; null until a fetch succeeds.
     */
    public fun attribution(): String? {
        synchronized(lock) { cachedAttribution?.let { return it } }
        val token = ensureSession(force = false) ?: return null
        val url = "$baseUrl/tile/v1/viewport?session=$token&key=$apiKey" +
            "&zoom=1&north=85&south=-85&east=180&west=-180"
        val body = runCatching {
            httpGet(url) { conn ->
                if (conn.responseCode != 200) null else conn.inputStream.readBytes()
            }
        }.getOrNull() ?: return null
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
     * [force]d.  Single-flight under [lock]; forced renewals are throttled
     * so a run of genuine 4xx tiles can't spam createSession.
     */
    private fun ensureSession(force: Boolean): String? = synchronized(lock) {
        val now = System.currentTimeMillis()
        val current = session
        val fresh = current != null && now / 1000 < current.expiryEpochSec - EXPIRY_MARGIN_SEC
        if (fresh && !force) return current.token
        if (force && now - lastCreateMs < renewMinIntervalMs) return current?.token
        lastCreateMs = now
        val created = createSession()
        session = created ?: session
        created?.token
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
            // Docs serialize expiry as a string of epoch seconds; be lenient.
            val expiry = obj["expiry"]?.jsonPrimitive?.content?.toLongOrNull() ?: 0L
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

        /**
         * Codes that plausibly mean "session token dead" (expired/revoked).
         * 404 is excluded — that's missing imagery, not a dead session.
         */
        val SESSION_DEAD_CODES = setOf(400, 401, 403)
    }
}
