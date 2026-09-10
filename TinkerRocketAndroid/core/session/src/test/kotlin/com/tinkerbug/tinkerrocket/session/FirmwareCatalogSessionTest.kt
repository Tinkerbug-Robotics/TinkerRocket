package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.EspImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareReleaseLocator
import com.tinkerbug.tinkerrocket.protocol.FirmwareRepository
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #773 step 4c: the sequencing between "check for updates" and bytes an
 * operator can flash. iOS twin: `FirmwareCatalogSessionTests.swift`.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class FirmwareCatalogSessionTest {

    private val sha = "a".repeat(64)
    private val imageBytes = ByteArray(64) { (it % 251).toByte() }

    private fun realSha(b: ByteArray): String =
        java.security.MessageDigest.getInstance("SHA-256").digest(b)
            .joinToString("") { "%02x".format(it) }

    private fun imageJson(
        project: String,
        board: String?,
        size: Int,
        version: String,
        imageSha: String = sha,
    ) =
        """{"file":"$project${board?.let { "-$it" } ?: ""}.bin","project":"$project",
           "version":"$version","chip_id":9,"chip":"ESP32-S3","size":$size,
           "sha256":"$imageSha"${board?.let { ",\"board\":\"$it\"" } ?: ""}}"""

    private fun manifest(vararg images: String) =
        """{"manifest_version":1,"tag":"fw-v1.0.0","images":[${images.joinToString(",")}]}"""

    private fun listing(tag: String = "fw-v1.0.0") =
        """[{"tag_name":"$tag","prerelease":false,"assets":[
            {"name":"manifest.json","browser_download_url":"https://x.test/$tag/manifest.json"},
            {"name":"out_computer-v9.bin","browser_download_url":"https://x.test/$tag/oc.bin"},
            {"name":"base_station-v2.bin","browser_download_url":"https://x.test/$tag/bs.bin"}]}]"""

    private fun rig(
        responses: Map<String, ByteArray?>,
        sha256: (ByteArray) -> String = { sha },
        cache: FirmwareCache? = null,
    ): Pair<FirmwareCatalogSession, TestScope> {
        val scope = TestScope()
        return FirmwareCatalogSession(
            FirmwareRepository(fetch = { responses[it] }, sha256 = sha256), scope, cache,
        ) to scope
    }

    private fun ok(manifestBody: String, assets: Map<String, ByteArray?> = emptyMap()) =
        mapOf(
            FirmwareReleaseLocator.RELEASES_URL to listing().encodeToByteArray(),
            "https://x.test/fw-v1.0.0/manifest.json" to manifestBody.encodeToByteArray(),
        ) + assets

    @Test
    fun `a release with an image for this unit is offered, best first`() = runTest {
        val (s, scope) = rig(ok(manifest(
            imageJson("out_computer", "v8", 64, "abc-v8+1"),
            imageJson("out_computer", "v9", 64, "abc-v9+1"),
        )))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertEquals("fw-v1.0.0", st.release.tag)
        assertEquals(2, st.images.size)
        assertEquals("v9", st.best?.board, "the unit's own revision leads")
        assertTrue(!st.alreadyRunning)
    }

    @Test
    fun `an unprovisioned unit is identified by the firmware it is running`() = runTest {
        // THE BENCH CASE, 2026-09-10. A real V9 out computer against
        // fw-v0.1.0: no provisioned board (it predates #773 step 2, as every
        // board in the field does), three board-specific images, and the
        // catalog recommended `m1` — the ROCKET-COMPUTER-MINI image — with a
        // tick beside it, because with nothing to rank on the sort fell back
        // to the board string and `m1` beats `v8` and `v9` on spelling.
        //
        // The unit had been saying which board it was the whole time, in the
        // version string the screen displays two rows above.
        val (s, scope) = rig(ok(manifest(
            imageJson("out_computer", "m1", 64, "abc-m1+1"),
            imageJson("out_computer", "v8", 64, "abc-v8+1"),
            imageJson("out_computer", "v9", 64, "abc-v9+1"),
        )))
        s.check(
            EspImage.PROJECT_OC,
            provisionedBoard = null,
            runningVersion = "e1a4bee4-v9+20260910-1112",
        )
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertEquals("v9", st.best?.board, "the running version says v9, so v9 it is")
        assertTrue(st.boardKnown)
    }

    @Test
    fun `a unit that says nothing about its board gets no recommendation`() = runTest {
        // Nothing provisioned and nothing readable in the version — a pre-#8
        // image, or a board flashed with a suffixless build. Ranking is
        // impossible, so recommend nothing rather than the alphabet.
        val (s, scope) = rig(ok(manifest(
            imageJson("out_computer", "m1", 64, "abc-m1+1"),
            imageJson("out_computer", "v9", 64, "abc-v9+1"),
        )))
        s.check(EspImage.PROJECT_OC, provisionedBoard = null, runningVersion = "abc123+20260910")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertNull(st.best, "no basis to choose, so no tick")
        assertEquals(2, st.images.size, "still listed for a deliberate choice")
        assertTrue(!st.boardKnown, "and the screen can say WHY there is no default")
    }

    @Test
    fun `a provisioned board still wins over the running version`() = runTest {
        // Provisioning is the board's own answer; the version is the image's
        // claim about itself, and a wrongly flashed board claims the wrong
        // thing until it is flashed again. EspImage.check has always ordered
        // them this way and the catalog now matches.
        val (s, scope) = rig(ok(manifest(
            imageJson("out_computer", "v8", 64, "abc-v8+1"),
            imageJson("out_computer", "v9", 64, "abc-v9+1"),
        )))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v8", runningVersion = "abc-v9+1")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertEquals("v8", st.best?.board, "the board's answer, not the image's claim")
    }

    @Test
    fun `a phone that fetched at home still lists at a field with no signal`() = runTest {
        // #773's acceptance line: "can pre-download at home for a field with
        // no signal". Images alone would not do it — with no network the app
        // cannot LIST anything, so it could not offer what it already holds.
        val dir = java.nio.file.Files.createTempDirectory("fwsess").toFile()
        val store = FirmwareCache(dir) { realSha(it) }
        val body = manifest(imageJson("out_computer", "v9", 64, "abc-v9+1"))

        // At home, with a signal.
        val (online, s1) = rig(ok(body), sha256 = { realSha(it) }, cache = store)
        online.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); s1.runCurrent()
        assertIs<FirmwareCatalogSession.State.Ready>(online.state.value)

        // At the field, with none at all.
        val (offline, s2) = rig(emptyMap(), sha256 = { realSha(it) }, cache = store)
        offline.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); s2.runCurrent()

        val st = offline.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertEquals("fw-v1.0.0", st.release.tag)
        assertEquals("v9", st.best?.board)
        assertTrue(st.offline, "and it says so, because the catalog may be stale")
    }

    @Test
    fun `no signal and nothing cached says both halves`() = runTest {
        // Distinct from the above: telling someone to check their connection
        // when they never downloaded anything sends them to fix the wrong
        // thing, and vice versa.
        val (s, scope) = rig(emptyMap())
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("nothing has been downloaded" in st.reason, st.reason)
        assertTrue("before leaving" in st.reason, st.reason)
    }

    @Test
    fun `a cached image is used without touching the network`() = runTest {
        val dir = java.nio.file.Files.createTempDirectory("fwsess").toFile()
        val store = FirmwareCache(dir) { realSha(it) }
        val body = manifest(
            imageJson("out_computer", "v9", 64, "abc-v9+1", imageSha = realSha(imageBytes)),
        )

        val (online, s1) = rig(
            ok(body, mapOf("https://x.test/fw-v1.0.0/oc.bin" to imageBytes)),
            sha256 = { realSha(it) }, cache = store,
        )
        online.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); s1.runCurrent()
        val ready = online.state.value as FirmwareCatalogSession.State.Ready
        online.download(ready.best!!); s1.runCurrent()
        assertIs<FirmwareCatalogSession.State.Downloaded>(online.state.value)

        // Now with the asset unreachable — the cache must carry it.
        val (offline, s2) = rig(ok(body), sha256 = { realSha(it) }, cache = store)
        offline.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); s2.runCurrent()
        val r2 = offline.state.value as FirmwareCatalogSession.State.Ready
        assertTrue(r2.held.contains(realSha(imageBytes)), "the list shows it is held")
        offline.download(r2.best!!); s2.runCurrent()

        val st = offline.state.value
        assertIs<FirmwareCatalogSession.State.Downloaded>(st)
        assertTrue(st.bytes.contentEquals(imageBytes))
    }

    @Test
    fun `prefetch stores what it can and reports what it could not`() = runTest {
        // Three of four onto the phone before leaving beats an all-or-nothing
        // refusal, so one bad image does not sink the run.
        val dir = java.nio.file.Files.createTempDirectory("fwsess").toFile()
        val store = FirmwareCache(dir) { realSha(it) }
        val body = manifest(
            imageJson("out_computer", "v9", 64, "abc-v9+1", imageSha = realSha(imageBytes)),
            imageJson("out_computer", "v8", 64, "abc-v8+1"),
        )
        val (s, scope) = rig(
            ok(body, mapOf(
                "https://x.test/fw-v1.0.0/oc.bin" to imageBytes,   // v9 asset present
            )),
            sha256 = { realSha(it) }, cache = store,
        )
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); scope.runCurrent()
        val ready = s.state.value as FirmwareCatalogSession.State.Ready
        s.prefetch(ready.images); scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Prefetched>(st)
        assertEquals(1, st.stored, "the one whose asset was reachable")
        assertEquals(1, st.failed.size, "and the one that was not is named")
        assertTrue(st.bytesHeld > 0)
    }

    @Test
    fun `no network says so, and does not blame the firmware`() = runTest {
        // At a launch site the overwhelmingly likely cause is no signal.
        // Telling an operator their firmware is missing sends them looking in
        // the wrong place.
        val (s, scope) = rig(emptyMap())
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("connection" in st.reason, st.reason)
    }

    @Test
    fun `a release carrying nothing for this unit says which release`() = runTest {
        val (s, scope) = rig(ok(manifest(imageJson("base_station", "v2", 64, "abc-v2+1"))))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("fw-v1.0.0" in st.reason, st.reason)
        assertTrue("out_computer" in st.reason, st.reason)
    }

    @Test
    fun `a board with no matching image still gets the list, just no default`() = runTest {
        // The catalog refuses to GUESS a default when the board is known and
        // nothing matches — but refusing to guess is not refusing to show.
        val (s, scope) = rig(ok(manifest(imageJson("out_computer", "v8", 64, "abc-v8+1"))))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v12")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertEquals(1, st.images.size, "still offered for a deliberate choice")
        assertNull(st.best, "but never defaulted to a revision this is not")
    }

    @Test
    fun `an image the unit already runs is flagged rather than hidden`() = runTest {
        val (s, scope) = rig(ok(manifest(imageJson("out_computer", "v9", 64, "abc-v9+1"))))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9", runningVersion = "abc-v9+1")
        scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Ready>(st)
        assertTrue(st.alreadyRunning, "re-flashing is allowed, but say it is a re-flash")
    }

    @Test
    fun `a verified download hands the bytes over`() = runTest {
        val bytes = ByteArray(64) { it.toByte() }
        val (s, scope) = rig(ok(
            manifest(imageJson("out_computer", "v9", 64, "abc-v9+1")),
            mapOf("https://x.test/fw-v1.0.0/oc.bin" to bytes),
        ))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); scope.runCurrent()
        val ready = s.state.value as FirmwareCatalogSession.State.Ready
        s.download(ready.best!!); scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Downloaded>(st)
        assertTrue(st.bytes.contentEquals(bytes))
    }

    @Test
    fun `a corrupt download is refused and never surfaces bytes`() = runTest {
        val bytes = ByteArray(64) { it.toByte() }
        val (s, scope) = rig(
            ok(manifest(imageJson("out_computer", "v9", 64, "abc-v9+1")),
               mapOf("https://x.test/fw-v1.0.0/oc.bin" to bytes)),
            sha256 = { "b".repeat(64) },
        )
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); scope.runCurrent()
        val ready = s.state.value as FirmwareCatalogSession.State.Ready
        s.download(ready.best!!); scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("does not match" in st.reason, st.reason)
    }

    @Test
    fun `a failed transfer reads as try again, not as a bad image`() = runTest {
        val (s, scope) = rig(ok(
            manifest(imageJson("out_computer", "v9", 64, "abc-v9+1")),
            mapOf("https://x.test/fw-v1.0.0/oc.bin" to null),
        ))
        s.check(EspImage.PROJECT_OC, provisionedBoard = "v9"); scope.runCurrent()
        val ready = s.state.value as FirmwareCatalogSession.State.Ready
        s.download(ready.best!!); scope.runCurrent()

        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("did not finish" in st.reason, st.reason)
    }

    @Test
    fun `downloading before checking is refused rather than crashing`() = runTest {
        val (s, _) = rig(emptyMap())
        s.download(
            FirmwareImage("x.bin", "out_computer", "v", 9, "ESP32-S3", 64, sha, "v9"),
        )
        val st = s.state.value
        assertIs<FirmwareCatalogSession.State.Failed>(st)
        assertTrue("check for updates" in st.reason, st.reason)
    }
}
