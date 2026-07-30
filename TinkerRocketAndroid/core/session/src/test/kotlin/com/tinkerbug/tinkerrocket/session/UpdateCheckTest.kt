package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

class UpdateCheckTest {

    // ── version ordering ─────────────────────────────────────────────────

    @Test
    fun compare_numericTriples() {
        assertTrue(UpdateCheck.compareVersions("0.9.1", "0.9.0") > 0)
        assertTrue(UpdateCheck.compareVersions("0.10.0", "0.9.9") > 0)
        assertTrue(UpdateCheck.compareVersions("1.0.0", "0.99.99") > 0)
        assertEquals(0, UpdateCheck.compareVersions("1.2.3", "1.2.3"))
    }

    @Test
    fun compare_suffixPrecedesItsBase() {
        // THE rule that matters here: the phase builds carried suffixes, and
        // getting this backwards would offer "updates" to older releases.
        assertTrue(UpdateCheck.compareVersions("0.1.0", "0.1.0-phase3") > 0)
        assertTrue(UpdateCheck.compareVersions("0.1.0-phase3", "0.1.0") < 0)
        // A newer numeric beats any suffix state of an older one.
        assertTrue(UpdateCheck.compareVersions("0.9.1", "0.1.0-phase3") > 0)
    }

    @Test
    fun compare_missingPartsReadAsZero() {
        assertEquals(0, UpdateCheck.compareVersions("1.0", "1.0.0"))
        assertTrue(UpdateCheck.compareVersions("1.0.1", "1.0") > 0)
    }

    // ── release JSON parsing ─────────────────────────────────────────────

    private val fixture = """
        [
          {"tag_name":"android-v0.9.1","html_url":"https://x/r/android-v0.9.1",
           "draft":false,"prerelease":false},
          {"tag_name":"android-v0.9.2","html_url":"https://x/r/android-v0.9.2",
           "draft":true,"prerelease":false},
          {"tag_name":"android-v0.9.3-rc1","html_url":"https://x/r/android-v0.9.3-rc1",
           "draft":false,"prerelease":true},
          {"tag_name":"fw-v2.0.0","html_url":"https://x/r/fw-v2.0.0",
           "draft":false,"prerelease":false},
          {"tag_name":"android-v0.8.0","html_url":"https://x/r/android-v0.8.0",
           "draft":false,"prerelease":false}
        ]
    """.trimIndent()

    @Test
    fun parse_keepsOnlyPublishedAndroidTags() {
        val got = UpdateCheck.parseReleases(fixture)
        // Draft 0.9.2 out, prerelease 0.9.3-rc1 out, firmware tag out.
        assertEquals(listOf("0.9.1", "0.8.0"), got.map { it.versionName })
        assertEquals("android-v0.9.1", got[0].tag)
    }

    @Test
    fun parse_neverThrows() {
        assertEquals(emptyList(), UpdateCheck.parseReleases("not json at all"))
        assertEquals(emptyList(), UpdateCheck.parseReleases("{}"))
        assertEquals(emptyList(), UpdateCheck.parseReleases("[{\"tag_name\":42}]"))
    }

    // ── the decision ─────────────────────────────────────────────────────

    @Test
    fun update_offeredWhenBehind() {
        val got = UpdateCheck.updateAvailable("0.1.0-phase3", fixture)
        assertEquals("0.9.1", got?.versionName)
    }

    @Test
    fun update_silentWhenCurrentOrAhead() {
        assertNull(UpdateCheck.updateAvailable("0.9.1", fixture))
        assertNull(UpdateCheck.updateAvailable("1.0.0", fixture))
    }

    @Test
    fun update_silentOnGarbage() {
        assertNull(UpdateCheck.updateAvailable("0.1.0", "service unavailable"))
    }
}
