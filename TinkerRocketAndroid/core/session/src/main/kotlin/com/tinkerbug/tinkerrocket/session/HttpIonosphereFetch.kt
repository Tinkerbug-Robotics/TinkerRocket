package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.ByteArrayOutputStream
import java.net.HttpURLConnection
import java.net.URI

/**
 * The network half of [IonosphereSource], on plain JVM: an anonymous HTTPS GET
 * of NOAA's CORS archive (#1600 phase 5).  iOS uses `URLSession` for the same.
 *
 * No credentials of any kind, as for [HttpFirmwareFetch]: the archive is
 * public and needs no login.
 */
public object HttpIonosphereFetch {

    /** The day's file is 30-70 KB of gzip; nothing that is it comes near this. */
    private const val MAX_BODY_BYTES: Int = 1024 * 1024

    public const val TIMEOUT_MS: Int = 15_000

    public val fetch: suspend (String) -> ByteArray? = { url ->
        withContext(Dispatchers.IO) {
            var conn: HttpURLConnection? = null
            try {
                conn = (URI(url).toURL().openConnection() as HttpURLConnection).apply {
                    connectTimeout = TIMEOUT_MS
                    readTimeout = TIMEOUT_MS
                    instanceFollowRedirects = true
                    // The file's own gzip is what IonosphereSource inflates.
                    // Left to itself, Android's stack offers gzip content
                    // encoding and silently inflates a response that takes
                    // it up, which would hand back plain text here.
                    setRequestProperty("Accept-Encoding", "identity")
                }
                if (conn.responseCode !in 200..299) {
                    null   // 404: the day's file is not up yet
                } else {
                    val out = ByteArrayOutputStream()
                    conn.inputStream.use { copyCapped(it, out, MAX_BODY_BYTES) }
                    out.toByteArray()
                }
            } catch (_: Exception) {
                // Offline, a timeout, a TLS failure: the caller treats them
                // all alike, trying yesterday's file and then its cache.
                null
            } finally {
                conn?.disconnect()
            }
        }
    }
}
