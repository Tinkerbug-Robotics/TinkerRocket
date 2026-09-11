package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.net.HttpURLConnection
import java.net.URI
import java.security.MessageDigest
import java.util.Locale

/**
 * The network and hash a [com.tinkerbug.tinkerrocket.protocol.FirmwareRepository]
 * needs, on plain JVM. iOS uses `URLSession` and `CryptoKit` for the same two.
 *
 * No credentials of any kind: the repository is public, and an anonymous read
 * is the one thing a phone at a launch site can always do. Adding a token here
 * would also put it in every APK.
 */
public object HttpFirmwareFetch {

    /** Refuse a body larger than any image could be, so a wrong URL cannot fill the heap. */
    private const val MAX_BODY_BYTES: Int = 8 * 1024 * 1024

    public const val TIMEOUT_MS: Int = 30_000

    public val fetch: suspend (String) -> ByteArray? = { url ->
        withContext(Dispatchers.IO) {
            var conn: HttpURLConnection? = null
            try {
                conn = (URI(url).toURL().openConnection() as HttpURLConnection).apply {
                    connectTimeout = TIMEOUT_MS
                    readTimeout = TIMEOUT_MS
                    // Pins the response shape even if GitHub's default changes.
                    setRequestProperty("Accept", "application/vnd.github+json")
                    // Asset downloads answer a redirect to the releases host;
                    // following it is what keeps them off the API's 60/hour
                    // anonymous budget.
                    instanceFollowRedirects = true
                }
                if (conn.responseCode !in 200..299) {
                    null
                } else {
                    conn.inputStream.use { it.readNBytes(MAX_BODY_BYTES) }
                }
            } catch (_: Exception) {
                // Every transport failure is the same answer to the caller:
                // no bytes. The repository turns that into "Unreachable",
                // which is what an operator can act on.
                null
            } finally {
                conn?.disconnect()
            }
        }
    }

    public val sha256: (ByteArray) -> String = { bytes ->
        MessageDigest.getInstance("SHA-256").digest(bytes)
            .joinToString("") { String.format(Locale.ROOT, "%02x", it) }
    }
}
