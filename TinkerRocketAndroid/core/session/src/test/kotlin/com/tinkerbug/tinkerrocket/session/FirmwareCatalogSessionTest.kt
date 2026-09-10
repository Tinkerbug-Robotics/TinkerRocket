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

    private fun imageJson(project: String, board: String?, size: Int, version: String) =
        """{"file":"$project${board?.let { "-$it" } ?: ""}.bin","project":"$project",
           "version":"$version","chip_id":9,"chip":"ESP32-S3","size":$size,
           "sha256":"$sha"${board?.let { ",\"board\":\"$it\"" } ?: ""}}"""

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
    ): Pair<FirmwareCatalogSession, TestScope> {
        val scope = TestScope()
        return FirmwareCatalogSession(
            FirmwareRepository(fetch = { responses[it] }, sha256 = sha256), scope,
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
