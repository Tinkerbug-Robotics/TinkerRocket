package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject

/**
 * The firmware manifest published beside the images on an `fw-v*` release
 * (#773 step 4; the release itself is #773 step 3).
 *
 * Written by `tinkerrocket-idf/tools/image_info.py`, which builds it by READING
 * each image's `esp_app_desc_t` rather than from the build matrix that produced
 * it. So every field here is the image's own account of itself, and the app can
 * check a downloaded file against it byte for byte.
 *
 * iOS twin: `Models/FirmwareManifest.swift`. Keep the two in step.
 */
public data class FirmwareImage(
    val file: String,
    val project: String,
    val version: String,
    val chipId: Int,
    val chip: String,
    val sizeBytes: Long,
    val sha256: String,
    /** Board revision the image asserts, or null when the build carries none. */
    val board: String?,
    val idfVersion: String = "",
    val buildDate: String = "",
) {
    /** A human line for a catalog row: what it is and when it was built. */
    public val summary: String
        get() = buildString {
            append(project)
            board?.let { append(" · ").append(it) }
            append(" · ").append(chip)
            if (buildDate.isNotEmpty()) append(" · ").append(buildDate)
        }
}

public data class FirmwareManifest(
    val manifestVersion: Int,
    val tag: String,
    val images: List<FirmwareImage>,
) {
    public companion object {
        /** The version this app understands. A newer manifest is refused rather
         *  than half-read: a field it does not know could be the one that says
         *  an image is unsafe for this unit. */
        public const val SUPPORTED_VERSION: Int = 1

        private val lenient = Json { ignoreUnknownKeys = true; isLenient = true }

        private fun str(o: JsonObject, k: String): String? =
            (o[k] as? JsonPrimitive)?.content?.takeIf { it.isNotEmpty() && it != "null" }

        private fun num(o: JsonObject, k: String): Long? =
            (o[k] as? JsonPrimitive)?.content?.toDoubleOrNull()?.toLong()

        /**
         * Parse a manifest, or null if it is not one we can act on.
         *
         * Deliberately strict about the things that decide whether a file may
         * be flashed — an image with no project, no sha or no size is dropped
         * rather than shown, because a catalog row the app cannot verify is
         * worse than a missing one.
         */
        public fun parse(text: String): FirmwareManifest? = try {
            val root = lenient.parseToJsonElement(text).jsonObject
            val version = num(root, "manifest_version")?.toInt() ?: 0
            if (version != SUPPORTED_VERSION) null
            else {
                val tag = str(root, "tag") ?: return null
                val images = root["images"]?.jsonArray.orEmpty().mapNotNull { el ->
                    val o = el as? JsonObject ?: return@mapNotNull null
                    val file = str(o, "file") ?: return@mapNotNull null
                    val project = str(o, "project") ?: return@mapNotNull null
                    val sha = str(o, "sha256")?.lowercase() ?: return@mapNotNull null
                    val size = num(o, "size") ?: return@mapNotNull null
                    if (sha.length != 64 || size <= 0L) return@mapNotNull null
                    FirmwareImage(
                        file = file,
                        project = project,
                        version = str(o, "version") ?: "",
                        chipId = num(o, "chip_id")?.toInt() ?: -1,
                        chip = str(o, "chip") ?: "",
                        sizeBytes = size,
                        sha256 = sha,
                        board = str(o, "board")?.lowercase(),
                        idfVersion = str(o, "idf_version") ?: "",
                        buildDate = str(o, "build_date") ?: "",
                    )
                }
                if (images.isEmpty()) null else FirmwareManifest(version, tag, images)
            }
        } catch (_: Exception) {
            null
        }
    }
}

/**
 * Which images in a manifest belong on the unit in front of you.
 *
 * The hard filter is `project`, exactly as in [EspImage.check] — it is the only
 * field that separates a base station from an out computer, both ESP32-S3 with
 * byte-identical app slots.
 *
 * Board is a RANKING, not a filter, and the order is deliberate:
 *  1. an image whose board matches what this board says it is
 *  2. an image with no board suffix at all, which applies everywhere
 *  3. everything else for the same project
 *
 * A non-matching board is offered LAST rather than hidden, because hiding it
 * would leave an operator with a board provisioned wrongly, or not at all, with
 * no way to flash anything — and the flash button still refuses a wrong
 * project, which is the case that actually matters.
 */
public object FirmwareCatalog {

    public fun forUnit(
        manifest: FirmwareManifest,
        expectedProject: String,
        provisionedBoard: String? = null,
    ): List<FirmwareImage> {
        val want = provisionedBoard?.trim()?.lowercase()?.ifEmpty { null }
        return manifest.images
            .filter { it.project == expectedProject }
            .sortedWith(
                compareBy(
                    { img ->
                        when {
                            want != null && img.board == want -> 0
                            img.board == null -> 1
                            else -> 2
                        }
                    },
                    { it.board ?: "" },
                    { it.file },
                ),
            )
    }

    /**
     * The single image to offer by default, or null when this manifest holds
     * nothing that can be recommended for this unit.
     *
     * Null rather than a guess in BOTH of the cases where a guess would be
     * one: when the board is known and no image matches it, and — the one the
     * bench found — when the board is NOT known and every candidate is
     * board-specific.
     *
     * That second case used to return the first candidate, which after the
     * board-agnostic sort means whichever board sorts first alphabetically.
     * On a real V9 out computer against fw-v0.1.0 that recommended the
     * ROCKET-COMPUTER-MINI image, `m1` beating `v8` and `v9` on nothing but
     * spelling, with a tick beside it (#773, 2026-09-10). Boards provisioned
     * before #773 step 2 report no revision at all, so this is the ordinary
     * state of existing hardware, not an edge case.
     *
     * An unsuffixed image IS still recommended to an unknown board: a project
     * that ships exactly one build (radio_board, the mini's own) applies
     * everywhere by construction, and there is nothing to get wrong.
     *
     * The full list is always still there for a deliberate choice — refusing
     * to guess is not the same as refusing to show.
     */
    public fun best(
        manifest: FirmwareManifest,
        expectedProject: String,
        provisionedBoard: String? = null,
    ): FirmwareImage? {
        val want = provisionedBoard?.trim()?.lowercase()?.ifEmpty { null }
        val candidates = forUnit(manifest, expectedProject, want)
        if (candidates.isEmpty()) return null
        val head = candidates.first()
        // head.board == null means the sort found a universal image and put it
        // first, which is a legitimate recommendation either way.
        if (want == null) return if (head.board == null) head else null
        return if (head.board == want || head.board == null) head else null
    }
}
