package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #773 step 4. The JSON here is the real shape `image_info.py --manifest`
 * emits, taken from a run against images built out of this repo.
 *
 * iOS twin: `FirmwareManifestTests.swift`, case for case.
 */
class FirmwareManifestTest {

    private fun img(project: String, board: String?, file: String = "$project.bin",
                    sha: String = "a".repeat(64), size: Long = 644336) = """
        {"file":"$file","project":"$project","version":"3d1f5ca2-dirty+1",
         "chip_id":9,"chip":"ESP32-S3","min_chip_rev_full":0,"max_chip_rev_full":99,
         "idf_version":"v6.0.1","build_date":"Sep  9 2026","build_time":"20:11:03",
         "size":$size,"sha256":"$sha"${if (board == null) "" else ""","board":"$board""""}}
    """.trimIndent()

    private fun manifest(vararg images: String, version: Int = 1, tag: String = "fw-v1.0.0") =
        """{"manifest_version":$version,"tag":"$tag","images":[${images.joinToString(",")}]}"""

    @Test
    fun `parses the shape image_info emits`() {
        val m = FirmwareManifest.parse(
            manifest(img("flight_computer", "v9"), img("out_computer", "v9")),
        )
        assertNotNull(m)
        assertEquals(1, m.manifestVersion)
        assertEquals("fw-v1.0.0", m.tag)
        assertEquals(2, m.images.size)
        assertEquals("flight_computer", m.images[0].project)
        assertEquals("v9", m.images[0].board)
        assertEquals(644336L, m.images[0].sizeBytes)
        assertEquals(64, m.images[0].sha256.length)
    }

    @Test
    fun `a build with no board suffix parses with a null board`() {
        val m = FirmwareManifest.parse(manifest(img("rocket_computer_mini", null)))
        assertNotNull(m)
        assertNull(m.images[0].board)
    }

    @Test
    fun `a newer manifest version is refused rather than half-read`() {
        // A field this build does not know could be the one saying an image is
        // unsafe for this unit, so guessing is not an option.
        assertNull(FirmwareManifest.parse(manifest(img("flight_computer", "v9"), version = 2)))
        assertNull(FirmwareManifest.parse(manifest(img("flight_computer", "v9"), version = 0)))
    }

    @Test
    fun `an image the app could not verify is dropped, not shown`() {
        // No sha, a short sha, or no size: a catalog row that cannot be checked
        // against the downloaded bytes is worse than a missing one.
        val noSha = """{"file":"x.bin","project":"flight_computer","size":100}"""
        val shortSha = """{"file":"x.bin","project":"flight_computer","size":100,"sha256":"abc"}"""
        val zeroSize = """{"file":"x.bin","project":"flight_computer","size":0,"sha256":"${"a".repeat(64)}"}"""
        assertNull(FirmwareManifest.parse(manifest(noSha)))
        assertNull(FirmwareManifest.parse(manifest(shortSha)))
        assertNull(FirmwareManifest.parse(manifest(zeroSize)))
        // ...but one bad row among good ones only drops that row.
        val m = FirmwareManifest.parse(manifest(img("flight_computer", "v9"), noSha))
        assertNotNull(m)
        assertEquals(1, m.images.size)
    }

    @Test
    fun `junk is null, not a crash`() {
        assertNull(FirmwareManifest.parse(""))
        assertNull(FirmwareManifest.parse("not json"))
        assertNull(FirmwareManifest.parse("[]"))
        assertNull(FirmwareManifest.parse("""{"manifest_version":1}"""))
    }

    // ── selection ────────────────────────────────────────────────────────

    private val full = FirmwareManifest.parse(
        manifest(
            img("flight_computer", "v7"), img("flight_computer", "v8"),
            img("flight_computer", "v9"), img("flight_computer", "m1"),
            img("out_computer", "v9"), img("base_station", "v3"),
            img("rocket_computer_mini", null),
        ),
    )!!

    @Test
    fun `project is the hard filter`() {
        // The case #1310 exists for: base_station and out_computer are both
        // ESP32-S3 with byte-identical app slots, so nothing else separates them.
        val oc = FirmwareCatalog.forUnit(full, EspImage.PROJECT_OC)
        assertEquals(1, oc.size)
        assertEquals("out_computer", oc[0].project)
    }

    @Test
    fun `the provisioned board sorts first, and everything is still offered`() {
        val list = FirmwareCatalog.forUnit(full, EspImage.PROJECT_FC, provisionedBoard = "V8")
        assertEquals("v8", list.first().board)
        // Not filtered away: a board provisioned wrongly, or not at all, must
        // not be left unable to flash anything.
        assertEquals(4, list.size)
    }

    @Test
    fun `best returns the matching board`() {
        assertEquals("v9", FirmwareCatalog.best(full, EspImage.PROJECT_FC, "v9")?.board)
        assertEquals("m1", FirmwareCatalog.best(full, EspImage.PROJECT_FC, "M1")?.board)
    }

    @Test
    fun `best refuses to guess when the board is known and nothing matches`() {
        // Offering the wrong revision as the default is how a wrong flash
        // happens. The full list is still there for a deliberate choice.
        assertNull(FirmwareCatalog.best(full, EspImage.PROJECT_FC, "v12"))
        assertTrue(FirmwareCatalog.forUnit(full, EspImage.PROJECT_FC, "v12").isNotEmpty())
    }

    @Test
    fun `an unknown board gets no default when every image is board-specific`() {
        // This test used to assert the opposite — "a circular guess beats
        // nothing, and EspImage.check still warns at flash time". The bench
        // disproved the first half on 2026-09-10: against fw-v0.1.0 on a real
        // V9 out computer, the guess was the ROCKET-COMPUTER-MINI image,
        // because with no board to match the sort falls back to the board
        // string and `m1` beats `v8` and `v9` on spelling alone. It was
        // presented with a tick, as the recommendation.
        //
        // A wrong recommendation is worse than none: it is the one an operator
        // in a hurry takes. The warning at flash time is a backstop, not a
        // reason to point at the wrong file first — and it is only a warning.
        //
        // Boards provisioned before #773 step 2 report no revision, so this is
        // the ordinary state of existing hardware rather than an edge case.
        assertNull(FirmwareCatalog.best(full, EspImage.PROJECT_FC, null))
        assertNull(FirmwareCatalog.best(full, EspImage.PROJECT_FC, "  "))
        // ...and the list is still there, which is the whole point: refusing
        // to guess is not refusing to show.
        assertEquals(4, FirmwareCatalog.forUnit(full, EspImage.PROJECT_FC, null).size)
    }

    @Test
    fun `an unknown board still gets a suffixless image`() {
        // A project shipping exactly one build applies everywhere by
        // construction, so there is nothing to get wrong and no reason to
        // withhold it.
        assertEquals(
            "rocket_computer_mini",
            FirmwareCatalog.best(full, EspImage.PROJECT_MINI, null)?.project,
        )
    }

    @Test
    fun `a version string is enough to identify the board`() {
        // What the app had all along and was not using: the unit's running
        // firmware says which board it is. EspImage.boardSuffix reads it, and
        // that is what turns the unknown-board case above back into a match.
        assertEquals("v9", EspImage.boardSuffix("e1a4bee4-v9+20260910-1112"))
        assertEquals("m1", EspImage.boardSuffix("f204e64-m1+20260910-1230"))
        assertNull(EspImage.boardSuffix("f204e64+20260910-1230"))
        assertNull(EspImage.boardSuffix(null))
        assertEquals(
            "v9",
            FirmwareCatalog.best(
                full, EspImage.PROJECT_FC,
                EspImage.boardSuffix("e1a4bee4-v9+20260910-1112"),
            )?.board,
        )
    }

    @Test
    fun `a suffixless image applies to any board`() {
        assertEquals(
            "rocket_computer_mini",
            FirmwareCatalog.best(full, EspImage.PROJECT_MINI, "v9")?.project,
        )
    }

    @Test
    fun `nothing for this unit is null, not an arbitrary image`() {
        assertNull(FirmwareCatalog.best(full, "radio_board", "v3"))
        assertTrue(FirmwareCatalog.forUnit(full, "radio_board").isEmpty())
    }

    @Test
    fun `the summary line names what a catalog row is`() {
        val i = full.images.first { it.board == "v9" && it.project == "flight_computer" }
        assertTrue(i.summary.contains("flight_computer"))
        assertTrue(i.summary.contains("v9"))
        assertTrue(i.summary.contains("ESP32-S3"))
    }

    @Test
    fun `the golden manifest from image_info py parses`() {
        // Not a hand-written fixture: this file was produced by
        // `image_info.py --manifest` against images actually built from this
        // tree. It is the contract between the release workflow (#1322) and
        // this parser, and it is the one that would catch either side drifting.
        val text = javaClass.getResourceAsStream("/firmware_manifest_golden.json")!!
            .bufferedReader().readText()
        val m = FirmwareManifest.parse(text)
        assertNotNull(m)
        assertEquals("fw-v1.0.0", m.tag)
        assertEquals(3, m.images.size)

        val fc = m.images.first { it.project == "flight_computer" }
        assertEquals("v9", fc.board)
        assertEquals("ESP32-P4", fc.chip)
        assertEquals(18, fc.chipId)
        assertTrue(fc.sizeBytes > 100_000)
        assertEquals(64, fc.sha256.length)

        // The mini's single-MCU build carries no board suffix — the real case
        // the null board exists for, not a synthetic one.
        assertNull(m.images.first { it.project == "rocket_computer_mini" }.board)

        // And selection works on it end to end.
        assertEquals("v9", FirmwareCatalog.best(m, EspImage.PROJECT_FC, "v9")?.board)
        assertNull(FirmwareCatalog.best(m, EspImage.PROJECT_BS, "v3"))
    }
}
