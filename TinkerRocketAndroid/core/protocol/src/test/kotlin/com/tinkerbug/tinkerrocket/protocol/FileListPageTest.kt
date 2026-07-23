package com.tinkerbug.tinkerrocket.protocol

import java.time.Instant
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * File-list page parse (strict, whole-page-or-nothing like iOS Codable), the
 * page-of-5 hasMore heuristic, and the FileInfo.flightDate port with its
 * pinned quirks.
 */
class FileListPageTest {

    private fun entries(n: Int): String =
        (0 until n).joinToString(",", prefix = "[", postfix = "]") {
            """{"name":"flight_2026072${it}_101010.bin","size":${1000 + it}}"""
        }

    // ── page-of-5 / hasMore edges ─────────────────────────────────────────

    @Test
    fun `empty page parses with no more files`() {
        val page = FileListPage.decode("[]")
        assertNotNull(page)
        assertEquals(0, page.files.size)
        assertFalse(page.hasMore)
    }

    @Test
    fun `partial page of 4 means no more files`() {
        val page = FileListPage.decode(entries(4))
        assertNotNull(page)
        assertEquals(4, page.files.size)
        assertFalse(page.hasMore)
    }

    @Test
    fun `full page of 5 means more files assumed`() {
        val page = FileListPage.decode(entries(5))
        assertNotNull(page)
        assertEquals(5, page.files.size)
        assertTrue(page.hasMore)
        assertEquals(5, FileListPage.FILES_PER_PAGE)
    }

    @Test
    fun `fields decode name and size`() {
        val page = FileListPage.decode("""[{"name":"a.bin","size":4294967295}]""")
        assertNotNull(page)
        assertEquals("a.bin", page.files[0].name)
        assertEquals(0xFFFF_FFFFL, page.files[0].size)   // full u32 range
    }

    // ── strict whole-page failure (iOS Codable semantics) ─────────────────

    @Test
    fun `any malformed entry fails the whole page`() {
        // Missing size.
        assertNull(FileListPage.decode("""[{"name":"a.bin"}]"""))
        // Missing name.
        assertNull(FileListPage.decode("""[{"size":10}]"""))
        // Negative / fractional / overflowing / stringy / boolean sizes.
        assertNull(FileListPage.decode("""[{"name":"a.bin","size":-1}]"""))
        assertNull(FileListPage.decode("""[{"name":"a.bin","size":5.5}]"""))
        assertNull(FileListPage.decode("""[{"name":"a.bin","size":4294967296}]"""))
        assertNull(FileListPage.decode("""[{"name":"a.bin","size":"5"}]"""))
        assertNull(FileListPage.decode("""[{"name":"a.bin","size":true}]"""))
        // Non-string name.
        assertNull(FileListPage.decode("""[{"name":7,"size":10}]"""))
        // Non-object element / non-array top level / garbage.
        assertNull(FileListPage.decode("""[42]"""))
        assertNull(FileListPage.decode("""{"name":"a.bin","size":10}"""))
        assertNull(FileListPage.decode("not json"))
        // One bad entry poisons the good ones (page-or-nothing).
        assertNull(
            FileListPage.decode("""[{"name":"a.bin","size":10},{"name":"b.bin"}]"""),
        )
    }

    @Test
    fun `exact-integral float size passes like NSNumber exactly-unboxing`() {
        val page = FileListPage.decode("""[{"name":"a.bin","size":5.0}]""")
        assertNotNull(page)
        assertEquals(5L, page.files[0].size)
    }

    // ── FileInfo.flightDate port ──────────────────────────────────────────

    private fun date(name: String): Instant? = FileInfo(name = name, size = 0).flightDateUtc

    @Test
    fun `flight and lora names parse as UTC`() {
        assertEquals(
            Instant.parse("2026-07-16T16:36:43Z"),
            date("flight_20260716_163643.bin"),
        )
        assertEquals(
            Instant.parse("2026-02-24T02:13:06Z"),
            date("lora_20260224_021306.csv"),
        )
    }

    @Test
    fun `separator character is skipped without being checked - pinned quirk`() {
        // iOS skips the char between date and time blindly; any single
        // character passes.
        assertEquals(
            Instant.parse("2026-07-16T16:36:43Z"),
            date("flight_20260716X163643.bin"),
        )
    }

    @Test
    fun `trailing characters after the time digits are ignored`() {
        assertEquals(
            Instant.parse("2026-07-16T16:36:43Z"),
            date("flight_20260716_163643_extra_stuff.bin"),
        )
    }

    @Test
    fun `unknown prefixes and short names return null`() {
        assertNull(date("rocket_data_000.bin"))
        // "flight_" but below the 26-char minimum-length gate.
        assertNull(date("flight_20260716_1636.bin"))
        // "lora_" but below its 23-char gate.
        assertNull(date("lora_20260716_1636.cv"))
    }

    @Test
    fun `non-digit fields and impossible dates return null`() {
        assertNull(date("flight_2026071x_163643.bin"))
        assertNull(date("flight_20260716_16364x.bin"))
        // Feb 30 — iOS's non-lenient DateFormatter rejects; STRICT resolver
        // must too.
        assertNull(date("flight_20260230_163643.bin"))
        // Month 13.
        assertNull(date("flight_20261316_163643.bin"))
        // Hour 25.
        assertNull(date("flight_20260716_253643.bin"))
    }
}
