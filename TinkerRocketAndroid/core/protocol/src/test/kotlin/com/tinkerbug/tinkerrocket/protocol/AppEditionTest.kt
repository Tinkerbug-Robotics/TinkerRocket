package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * The store / dev edition rules (AppEdition.kt). iOS twin:
 * `AppEditionTests.swift`, case for case. The edition is passed in, so the
 * store rules are checked from any build.
 */
class AppEditionTest {

    // ── Board family ────────────────────────────────────────────────────

    @Test
    fun `the revision letter names the family`() {
        assertEquals(BoardFamily.BEETLE, BoardFamily.fromRevision("M2"))
        assertEquals(BoardFamily.BEETLE, BoardFamily.fromRevision("M2 (Tinker-Beetle V2)"))
        assertEquals(BoardFamily.BEETLE, BoardFamily.fromRevision("m3"))
        assertEquals(BoardFamily.MANTIS, BoardFamily.fromRevision("V10"))
        assertEquals(BoardFamily.MANTIS, BoardFamily.fromRevision(" v9 "))
    }

    @Test
    fun `text that names no board is unknown`() {
        assertNull(BoardFamily.fromRevision(null))
        assertNull(BoardFamily.fromRevision(""))
        assertNull(BoardFamily.fromRevision("unknown"))
        assertNull(BoardFamily.fromRevision("M"))        // a letter with no revision
        assertNull(BoardFamily.fromRevision("Mantis"))
        assertNull(BoardFamily.fromRevision("B1"))       // not a family this app gates on
    }

    @Test
    fun `the provisioned board beats the firmware build`() {
        // A Beetle flashed with a V10 build still says what it is.
        assertEquals(
            BoardFamily.BEETLE,
            BoardFamily.of("M2", null, "3d1f5ca2-v10+20261120", null),
        )
        assertEquals(
            BoardFamily.MANTIS,
            BoardFamily.of(null, "V10", "", "3d1f5ca2-m2+20261120"),
        )
    }

    @Test
    fun `an unprovisioned board falls back to its firmware build`() {
        assertEquals(BoardFamily.BEETLE, BoardFamily.of(null, "", "3d1f5ca2-m2+20261120", ""))
        assertEquals(BoardFamily.MANTIS, BoardFamily.of(null, null, "", "d7017c0-v10+20260827-1919"))
        assertNull(BoardFamily.of(null, null, "unknown", "3d1f5ca2-dirty"))
    }

    // ── Features ────────────────────────────────────────────────────────

    @Test
    fun `the store edition has no fin or camera screens for any board`() {
        for (board in listOf(BoardFamily.BEETLE, BoardFamily.MANTIS, null)) {
            val f = AppFeatures(AppEdition.STORE, board)
            assertFalse(f.finControl, "$board")
            assertFalse(f.camera, "$board")
        }
    }

    @Test
    fun `the dev edition follows the board`() {
        assertFalse(AppFeatures(AppEdition.DEV, BoardFamily.BEETLE).finControl)
        assertFalse(AppFeatures(AppEdition.DEV, BoardFamily.BEETLE).camera)
        assertTrue(AppFeatures(AppEdition.DEV, BoardFamily.MANTIS).finControl)
        assertTrue(AppFeatures(AppEdition.DEV, BoardFamily.MANTIS).camera)
        // Cannot tell (nothing connected, a relay, an unprovisioned board):
        // the dev edition shows everything.
        assertTrue(AppFeatures(AppEdition.DEV, null).finControl)
        assertTrue(AppFeatures(AppEdition.DEV, null).camera)
    }

    @Test
    fun `the store edition supports the Beetle only`() {
        assertTrue(AppFeatures(AppEdition.STORE, BoardFamily.BEETLE).boardSupported)
        assertFalse(AppFeatures(AppEdition.STORE, BoardFamily.MANTIS).boardSupported)
        // A Beetle reads as unknown until its identity arrives: no warning yet.
        assertTrue(AppFeatures(AppEdition.STORE, null).boardSupported)
        for (board in listOf(BoardFamily.BEETLE, BoardFamily.MANTIS, null)) {
            assertTrue(AppFeatures(AppEdition.DEV, board).boardSupported)
        }
    }

    // ── Firmware catalog ────────────────────────────────────────────────

    private fun img(project: String, board: String?) = """
        {"file":"$project-${board ?: "any"}.bin","project":"$project",
         "version":"3d1f5ca2+1","chip_id":9,"chip":"ESP32-S3","size":644336,
         "sha256":"${"a".repeat(64)}"${if (board == null) "" else ""","board":"$board""""}}
    """.trimIndent()

    private val manifest: FirmwareManifest = FirmwareManifest.parse(
        """{"manifest_version":1,"tag":"fw-v1.0.0","images":[${
            listOf(
                img("flight_computer", "v10"), img("flight_computer", "m2"),
                img("flight_computer", null),
                img("out_computer", "v10"), img("out_computer", "m2"),
                img("base_station", "v3"), img("base_station", "v4"), img("base_station", "v5"),
            ).joinToString(",")
        }]}""",
    )!!

    private fun boards(project: String, edition: AppEdition, provisioned: String? = null) =
        FirmwareCatalog.forUnit(manifest, project, provisioned, edition).map { it.board }

    @Test
    fun `the store catalog offers only Beetle rocket images`() {
        assertEquals(listOf("m2", null), boards("flight_computer", AppEdition.STORE, "m2"))
        assertEquals(listOf<String?>("m2"), boards("out_computer", AppEdition.STORE, "m2"))
        // A Mantis connected to the store app is offered nothing of its own.
        assertEquals(listOf<String?>("m2"), boards("out_computer", AppEdition.STORE, "v10"))
        assertNull(FirmwareCatalog.best(manifest, "out_computer", "v10", AppEdition.STORE))
    }

    @Test
    fun `the store catalog offers every base station`() {
        assertEquals(setOf<String?>("v3", "v4", "v5"), boards("base_station", AppEdition.STORE).toSet())
    }

    @Test
    fun `the dev catalog is unchanged`() {
        assertEquals(listOf("v10", null, "m2"), boards("flight_computer", AppEdition.DEV, "v10"))
        assertEquals(setOf<String?>("m2", "v10"), boards("out_computer", AppEdition.DEV).toSet())
    }
}
