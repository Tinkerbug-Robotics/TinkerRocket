package com.tinkerbug.tinkerrocket.session

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive

/**
 * In-app update check against GitHub Releases (plan §1: sideload APK via
 * GitHub Releases + trivial update check → open browser). Android-only BY
 * DESIGN — iOS updates ride the App Store; ledger records the divergence.
 *
 * Pure logic: parsing and version ordering live here so they're JVM-testable;
 * the network fetch and throttle are the app's thin `UpdateChecker`.
 *
 * Release tags look like `android-v0.9.1` — the `android-` prefix keeps app
 * releases distinct from any future firmware releases in the same repo, so
 * everything else (drafts, prereleases, foreign tags) is ignored rather than
 * compared.
 */
public object UpdateCheck {

    public data class ReleaseCandidate(
        val versionName: String,   // "0.9.1" — tag minus the android-v prefix
        val tag: String,           // "android-v0.9.1"
        val htmlUrl: String,       // release page; the browser handles the APK
    )

    /**
     * Version ordering for `major.minor.patch[-suffix]` names.
     *
     * Semver's one non-obvious rule matters here: a suffixed version precedes
     * its unsuffixed base — `0.1.0-phase3` < `0.1.0`. The phase builds carried
     * suffixes, so getting this backwards would offer "updates" to older
     * releases. Missing numeric parts read as 0 (`1.0` == `1.0.0`); two
     * suffixes compare lexically, which is as much precision as our tag
     * history needs.
     */
    public fun compareVersions(a: String, b: String): Int {
        val (na, sa) = split(a)
        val (nb, sb) = split(b)
        for (i in 0 until 3) {
            val d = na[i].compareTo(nb[i])
            if (d != 0) return d
        }
        return when {
            sa == sb -> 0
            sa.isEmpty() -> 1     // release > its own pre-release
            sb.isEmpty() -> -1
            else -> sa.compareTo(sb)
        }
    }

    private fun split(v: String): Pair<IntArray, String> {
        val dash = v.indexOf('-')
        val numeric = if (dash >= 0) v.substring(0, dash) else v
        val suffix = if (dash >= 0) v.substring(dash + 1) else ""
        val parts = numeric.split('.')
        val nums = IntArray(3) { parts.getOrNull(it)?.toIntOrNull() ?: 0 }
        return nums to suffix
    }

    /**
     * Parse the GitHub `/releases` JSON array into candidates, keeping only
     * published (non-draft, non-prerelease) `android-v*` tags. Anything
     * malformed is skipped, never thrown — an update check must not be able
     * to break the app over API drift.
     */
    public fun parseReleases(json: String): List<ReleaseCandidate> = try {
        Json.parseToJsonElement(json).jsonArray.mapNotNull { el ->
            try {
                val o = el.jsonObject
                if (o["draft"]?.jsonPrimitive?.booleanOrNull() != false) return@mapNotNull null
                if (o["prerelease"]?.jsonPrimitive?.booleanOrNull() != false) return@mapNotNull null
                val tag = o["tag_name"]?.jsonPrimitive?.content ?: return@mapNotNull null
                if (!tag.startsWith("android-v")) return@mapNotNull null
                val url = o["html_url"]?.jsonPrimitive?.content ?: return@mapNotNull null
                ReleaseCandidate(tag.removePrefix("android-v"), tag, url)
            } catch (_: Exception) {
                null
            }
        }
    } catch (_: Exception) {
        emptyList()
    }

    /** The newest applicable release strictly ahead of [currentVersionName], or null. */
    public fun updateAvailable(
        currentVersionName: String,
        releasesJson: String,
    ): ReleaseCandidate? =
        parseReleases(releasesJson)
            .maxWithOrNull { x, y -> compareVersions(x.versionName, y.versionName) }
            ?.takeIf { compareVersions(it.versionName, currentVersionName) > 0 }

    private fun kotlinx.serialization.json.JsonPrimitive.booleanOrNull(): Boolean? =
        content.toBooleanStrictOrNull()
}
