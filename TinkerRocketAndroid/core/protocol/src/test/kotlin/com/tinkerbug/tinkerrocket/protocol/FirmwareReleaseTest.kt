package com.tinkerbug.tinkerrocket.protocol

import kotlin.coroutines.Continuation
import kotlin.coroutines.EmptyCoroutineContext
import kotlin.coroutines.startCoroutine
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #773 step 4: finding the firmware release on GitHub and fetching from it.
 * iOS twin: `FirmwareReleaseTests.swift`.
 */
class FirmwareReleaseTest {

    /**
     * Drives a suspend function to completion on this thread.
     *
     * `:core:protocol` deliberately carries no coroutines dependency (see its
     * build file), so there is no `runTest`/`runBlocking` here — and none is
     * needed: every `fetch` below answers from a map, so the coroutine never
     * actually suspends and has finished before `startCoroutine` returns. If a
     * future test hands `FirmwareRepository` a fetch that really suspends,
     * this helper will throw rather than hang, which is the failure you want.
     *
     * Every caller writes `(): Unit =` explicitly. Without it a body ending in
     * `assertIs` — which returns the narrowed value, not Unit — gives the test
     * method a non-void return type, and JUnit 5 then does not run it at all:
     * two cases here vanished from the report that way, silently green.
     */
    private fun <T> runSync(block: suspend () -> T): T {
        var out: Result<T>? = null
        block.startCoroutine(Continuation(EmptyCoroutineContext) { out = it })
        return checkNotNull(out) { "the block suspended; this driver cannot resume it" }
            .getOrThrow()
    }

    private fun rel(tag: String, prerelease: Boolean = false,
                    assets: List<String> = listOf("manifest.json", "flight_computer-V9.bin")) =
        """
        {"tag_name":"$tag","prerelease":$prerelease,"assets":[${
            assets.joinToString(",") {
                """{"name":"$it","browser_download_url":"https://example.test/$tag/$it"}"""
            }
        }]}
        """.trimIndent()

    private fun listing(vararg releases: String) = "[${releases.joinToString(",")}]"

    @Test
    fun `board and app releases are not firmware releases`() {
        // The trap this exists for. GitHub's /releases/latest returns the newest
        // release of ANY kind, and this repo also tags board and Android
        // releases — at the time of writing it answers a gerber zip. Firmware
        // is found by tag prefix, never by "latest".
        val real = javaClass.getResourceAsStream("/github_releases_golden.json")!!
            .bufferedReader().readText()
        assertTrue(real.contains("rocket-computer-mini-v1.0.1"))
        assertNull(FirmwareReleaseLocator.newest(real))
        assertTrue(FirmwareReleaseLocator.firmwareReleases(real).isEmpty())
    }

    @Test
    fun `picks the newest firmware release by version, not by API order`() {
        // The API sorts by creation date, which is usually the same thing and
        // occasionally is not — a re-cut or edited tag moves in that ordering.
        val json = listing(rel("fw-v1.0.0"), rel("fw-v1.10.0"), rel("fw-v1.9.0"))
        assertEquals("fw-v1.10.0", FirmwareReleaseLocator.newest(json)?.tag)
    }

    @Test
    fun `prereleases are skipped unless asked for`() {
        val json = listing(rel("fw-v2.0.0", prerelease = true), rel("fw-v1.0.0"))
        assertEquals("fw-v1.0.0", FirmwareReleaseLocator.newest(json)?.tag)
        assertEquals("fw-v2.0.0",
            FirmwareReleaseLocator.newest(json, includePrereleases = true)?.tag)
    }

    @Test
    fun `a release with no manifest is skipped`() {
        // Either it predates the manifest or the publish failed. Either way
        // there is nothing the app can act on.
        val json = listing(rel("fw-v2.0.0", assets = listOf("flight_computer-V9.bin")),
                           rel("fw-v1.0.0"))
        assertEquals("fw-v1.0.0", FirmwareReleaseLocator.newest(json)?.tag)
    }

    @Test
    fun `a bare version outranks the same version with a suffix`() {
        val json = listing(rel("fw-v1.0.0-rc1"), rel("fw-v1.0.0"))
        assertEquals("fw-v1.0.0", FirmwareReleaseLocator.newest(json)?.tag)
    }

    @Test
    fun `junk is an empty list, not a crash`() {
        assertTrue(FirmwareReleaseLocator.firmwareReleases("").isEmpty())
        assertTrue(FirmwareReleaseLocator.firmwareReleases("not json").isEmpty())
        assertTrue(FirmwareReleaseLocator.firmwareReleases("{}").isEmpty())
        assertNull(FirmwareReleaseLocator.newest("[]"))
    }

    // ── fetching ────────────────────────────────────────────────────────

    private val bytes = ByteArray(64) { it.toByte() }
    private val goodSha = "a".repeat(64)

    private fun image(sha: String = goodSha, size: Long = 64) = FirmwareImage(
        file = "flight_computer-V9.bin", project = "flight_computer",
        version = "abc-v9+1", chipId = 18, chip = "ESP32-P4",
        sizeBytes = size, sha256 = sha, board = "v9",
    )

    private fun repo(
        responses: Map<String, ByteArray?>,
        sha: (ByteArray) -> String = { goodSha },
    ) = FirmwareRepository(fetch = { responses[it] }, sha256 = sha)

    @Test
    fun `latestManifest walks the listing then the manifest`(): Unit = runSync {
        val manifestJson = """
            {"manifest_version":1,"tag":"fw-v1.0.0","images":[
             {"file":"flight_computer-V9.bin","project":"flight_computer",
              "size":64,"sha256":"$goodSha","board":"v9","chip":"ESP32-P4","chip_id":18}]}
        """.trimIndent()
        val r = repo(mapOf(
            FirmwareReleaseLocator.RELEASES_URL to listing(rel("fw-v1.0.0")).encodeToByteArray(),
            "https://example.test/fw-v1.0.0/manifest.json" to manifestJson.encodeToByteArray(),
        )).latestManifest()
        assertNotNull(r)
        assertEquals("fw-v1.0.0", r.release.tag)
        assertEquals(1, r.manifest.images.size)
        // The raw text comes back too, so a cache stores what it verified
        // rather than a re-serialization that could differ from it.
        assertEquals(manifestJson, r.manifestJson)
    }

    @Test
    fun `a release round-trips through its own listing JSON`(): Unit = runSync {
        // The cache writes this and reads it back through the SAME parser the
        // network path uses, so there is one codec for release JSON rather
        // than two that can disagree. Quoting matters: an asset name or URL
        // with a quote or backslash in it must survive.
        val original = FirmwareRelease(
            tag = "fw-v1.0.0",
            isPrerelease = true,
            assets = mapOf(
                "manifest.json" to "https://example.test/a b/manifest.json",
                "odd\"name.bin" to "https://example.test/x\\y.bin",
            ),
        )
        val back = FirmwareReleaseLocator
            .firmwareReleases(original.toListingJson(), includePrereleases = true)
            .single()
        assertEquals(original, back)
    }

    @Test
    fun `no network is null, not an exception`(): Unit = runSync {
        assertNull(repo(emptyMap()).latestManifest())
    }

    @Test
    fun `a verified download returns the bytes`(): Unit = runSync {
        val release = FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0")))!!
        val got = repo(mapOf(
            "https://example.test/fw-v1.0.0/flight_computer-V9.bin" to bytes,
        )).download(release, image())
        assertIs<FirmwareFetch.Ok>(got)
        assertTrue(got.bytes.contentEquals(bytes))
    }

    @Test
    fun `a truncated download is refused, and says so`(): Unit = runSync {
        // Size is checked before the hash purely so this reports as truncated
        // rather than as a checksum mismatch. Same refusal, clearer reason.
        val release = FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0")))!!
        val got = repo(mapOf(
            "https://example.test/fw-v1.0.0/flight_computer-V9.bin" to ByteArray(10),
        )).download(release, image())
        assertIs<FirmwareFetch.Corrupt>(got)
        assertTrue(got.reason.contains("10 bytes"))
    }

    @Test
    fun `a checksum mismatch is refused and never returned`(): Unit = runSync {
        // A tampered file and a corrupted one look identical here. Neither
        // belongs on a flight computer, so neither is handed back.
        val release = FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0")))!!
        val got = repo(
            mapOf("https://example.test/fw-v1.0.0/flight_computer-V9.bin" to bytes),
            sha = { "b".repeat(64) },
        ).download(release, image())
        assertIs<FirmwareFetch.Corrupt>(got)
        assertTrue(got.reason.contains("checksum"))
    }

    @Test
    fun `an image not in the release is unreachable, not corrupt`(): Unit = runSync {
        val release = FirmwareReleaseLocator.newest(
            listing(rel("fw-v1.0.0", assets = listOf("manifest.json"))),
        )!!
        val got = repo(emptyMap()).download(release, image())
        assertIs<FirmwareFetch.Unreachable>(got)
    }

    @Test
    fun `a failed transfer is unreachable, not corrupt`(): Unit = runSync {
        // The distinction matters to the operator: one is "try again on better
        // signal", the other is "do not flash this".
        val release = FirmwareReleaseLocator.newest(listing(rel("fw-v1.0.0")))!!
        val got = repo(mapOf(
            "https://example.test/fw-v1.0.0/flight_computer-V9.bin" to null,
        )).download(release, image())
        assertIs<FirmwareFetch.Unreachable>(got)
    }
}
