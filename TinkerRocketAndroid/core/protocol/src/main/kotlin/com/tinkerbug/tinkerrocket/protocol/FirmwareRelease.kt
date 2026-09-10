package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonArray

/**
 * Finding the firmware release on GitHub, and fetching from it (#773 step 4).
 *
 * WHERE THE BINARIES LIVE: GitHub Releases on this repository, which is public,
 * so an anonymous client can read both the API and the assets — no token, no
 * account. Verified 2026-09-10 against a real release asset.
 *
 * WHY NOT `/releases/latest`: that endpoint returns the newest release of ANY
 * kind, and this repo also tags board and Android releases. At the time of
 * writing it answers `rocket-computer-mini-v1.0.1` — a gerber zip. Firmware has
 * to be found by tag prefix, which is what this does.
 *
 * RATE LIMIT: listing releases is ONE unauthenticated API call, and the
 * anonymous budget is 60/hour per IP. Asset downloads do not touch
 * api.github.com at all — they come from the releases/download host — so
 * downloading eleven images costs one API call, not twelve.
 *
 * iOS twin: `Models/FirmwareRelease.swift`.
 */
public data class FirmwareRelease(
    val tag: String,
    val isPrerelease: Boolean,
    /** Asset name to download URL. */
    val assets: Map<String, String>,
) {
    public val manifestUrl: String? get() = assets[MANIFEST_ASSET]

    public fun urlFor(image: FirmwareImage): String? = assets[image.file]

    /**
     * This release in the shape [FirmwareReleaseLocator.firmwareReleases]
     * parses — a one-element `/releases` response.
     *
     * A cache writes this and reads it back through the same parser the
     * network path uses, so there is one codec for release JSON rather than
     * two that can disagree. Round-tripped by test.
     */
    public fun toListingJson(): String {
        val assetsJson = assets.entries.joinToString(",") { (name, url) ->
            """{"name":${quote(name)},"browser_download_url":${quote(url)}}"""
        }
        return """[{"tag_name":${quote(tag)},"prerelease":$isPrerelease,"assets":[$assetsJson]}]"""
    }

    public companion object {
        public const val MANIFEST_ASSET: String = "manifest.json"

        private fun quote(s: String): String = buildString {
            append('"')
            for (c in s) when (c) {
                '"' -> append("\\\"")
                '\\' -> append("\\\\")
                '\n' -> append("\\n")
                '\r' -> append("\\r")
                '\t' -> append("\\t")
                else -> if (c < ' ') append("\\u%04x".format(c.code)) else append(c)
            }
            append('"')
        }
    }
}

public object FirmwareReleaseLocator {
    /** Tag scheme from `.github/workflows/firmware-release.yml`. */
    public const val TAG_PREFIX: String = "fw-v"

    public const val RELEASES_URL: String =
        "https://api.github.com/repos/Tinkerbug-Robotics/TinkerRocket/releases?per_page=30"

    private val lenient = Json { ignoreUnknownKeys = true; isLenient = true }

    private fun str(o: JsonObject, k: String): String? =
        (o[k] as? JsonPrimitive)?.content?.takeIf { it.isNotEmpty() && it != "null" }

    /**
     * Every firmware release in a `/releases` response, newest first.
     *
     * Ordered by the version in the tag rather than by the API's own ordering.
     * The API sorts by creation date, which is usually the same thing and
     * occasionally is not — a re-cut tag, or a release edited later, moves in
     * that ordering and would silently become "newest".
     *
     * A release with no `manifest.json` asset is skipped: it is either from
     * before the manifest existed or a failed publish, and either way there is
     * nothing the app can act on.
     */
    public fun firmwareReleases(json: String, includePrereleases: Boolean = false):
        List<FirmwareRelease> = try {
        val arr: JsonArray = lenient.parseToJsonElement(json).jsonArray
        arr.mapNotNull { el ->
            val o = el as? JsonObject ?: return@mapNotNull null
            val tag = str(o, "tag_name") ?: return@mapNotNull null
            if (!tag.startsWith(TAG_PREFIX)) return@mapNotNull null
            val pre = ((o["prerelease"] as? JsonPrimitive)?.content == "true")
            if (pre && !includePrereleases) return@mapNotNull null
            val assets = (o["assets"] as? JsonArray).orEmpty().mapNotNull { a ->
                val ao = a as? JsonObject ?: return@mapNotNull null
                val name = str(ao, "name") ?: return@mapNotNull null
                val url = str(ao, "browser_download_url") ?: return@mapNotNull null
                name to url
            }.toMap()
            if (!assets.containsKey(FirmwareRelease.MANIFEST_ASSET)) return@mapNotNull null
            FirmwareRelease(tag, pre, assets)
        }.sortedWith { a, b -> compareKeys(versionKey(b.tag), versionKey(a.tag)) }
    } catch (_: Exception) {
        emptyList()
    }

    public fun newest(json: String, includePrereleases: Boolean = false): FirmwareRelease? =
        firmwareReleases(json, includePrereleases).firstOrNull()

    /**
     * Comparable key from a `fw-v1.2.3` tag. Non-numeric tails sort below an
     * otherwise-equal release, so `fw-v1.0.0` beats `fw-v1.0.0-rc1`.
     */
    /** Lexicographic compare of two equal-length version keys. */
    internal fun compareKeys(a: List<Int>, b: List<Int>): Int {
        for (i in a.indices) {
            val c = a[i].compareTo(b[i])
            if (c != 0) return c
        }
        return 0
    }

    internal fun versionKey(tag: String): List<Int> {
        val body = tag.removePrefix(TAG_PREFIX)
        val core = body.takeWhile { it.isDigit() || it == '.' }
        val parts = core.split('.').mapNotNull { it.toIntOrNull() }
        val padded = (parts + listOf(0, 0, 0)).take(3)
        // A suffix ("-rc1", "-dryrun") ranks below the bare version.
        return padded + listOf(if (body.length > core.length) 0 else 1)
    }
}

/** What a download attempt produced. */
public sealed class FirmwareFetch {
    public data class Ok(val bytes: ByteArray) : FirmwareFetch() {
        override fun equals(other: Any?): Boolean =
            other is Ok && bytes.contentEquals(other.bytes)
        override fun hashCode(): Int = bytes.contentHashCode()
    }
    /** The transport failed — no bytes, or the host said no. */
    public data class Unreachable(val reason: String) : FirmwareFetch()
    /**
     * Bytes arrived and are NOT what the manifest said they would be. Never
     * flashed, never cached: a truncated download and a tampered file look
     * identical here, and neither belongs on a flight computer.
     */
    public data class Corrupt(val reason: String) : FirmwareFetch()
}

/**
 * Fetching, with the network injected so the policy is testable without one.
 *
 * [fetch] returns the body, or null when the request failed.
 * [sha256] hashes bytes — supplied by the platform, since neither Kotlin/JVM
 * nor Swift has one in the common surface this module targets.
 */
public data class FetchedCatalog(
    val release: FirmwareRelease,
    val manifest: FirmwareManifest,
    /**
     * Exactly the bytes [manifest] was parsed from, so a cache stores what it
     * actually verified rather than a re-serialization that could differ.
     */
    val manifestJson: String,
)

public class FirmwareRepository(
    private val fetch: suspend (String) -> ByteArray?,
    private val sha256: (ByteArray) -> String,
) {
    public suspend fun latestManifest(includePrereleases: Boolean = false): FetchedCatalog? {
        val listing = fetch(FirmwareReleaseLocator.RELEASES_URL) ?: return null
        val release = FirmwareReleaseLocator.newest(
            listing.decodeToString(), includePrereleases,
        ) ?: return null
        val url = release.manifestUrl ?: return null
        val body = fetch(url) ?: return null
        val text = body.decodeToString()
        val manifest = FirmwareManifest.parse(text) ?: return null
        return FetchedCatalog(release, manifest, text)
    }

    /**
     * Download one image and prove it is the one the manifest described.
     *
     * Size is checked before the hash purely so a truncated download reports as
     * truncated rather than as a hash mismatch — same refusal, clearer reason.
     */
    public suspend fun download(release: FirmwareRelease, image: FirmwareImage): FirmwareFetch {
        val url = release.urlFor(image)
            ?: return FirmwareFetch.Unreachable("${image.file} is not in release ${release.tag}")
        val bytes = fetch(url)
            ?: return FirmwareFetch.Unreachable("could not download ${image.file}")
        if (bytes.size.toLong() != image.sizeBytes) {
            return FirmwareFetch.Corrupt(
                "${image.file} is ${bytes.size} bytes, manifest says ${image.sizeBytes}",
            )
        }
        val got = sha256(bytes).lowercase()
        if (got != image.sha256) {
            return FirmwareFetch.Corrupt("${image.file} checksum does not match the manifest")
        }
        return FirmwareFetch.Ok(bytes)
    }
}
