package com.tinkerbug.tinkerrocket.protocol

/**
 * ESP-IDF application-image header, and whether a picked `.bin` belongs on the
 * unit the user is about to flash (#773).
 *
 * Today neither app looks at the image at all: the file is chosen by hand, and
 * `TR_OTA` on the far end validates only size and SHA-256 — nothing on either
 * side inspects what the image IS. The out computer and the base station are
 * both ESP32-S3 with byte-identical app slots, so a base-station image pushed
 * to an out computer is accepted by both ends and only rollback catches it,
 * and only if the wrong image happens to fail to boot.
 *
 * The image already carries the answer. ESP-IDF writes an `esp_app_desc_t` at
 * a fixed offset containing the CMake `project()` name, so "is this the right
 * program for this box" is a string comparison — no manifest, no firmware
 * change, no CI change.
 *
 * Layout (esp_image_format.h), verified against real built images:
 *   0   esp_image_header_t, 24 bytes; [0] magic 0xE9, [12..13] chip_id (LE u16),
 *       [15..16] min_chip_rev_full, [17..18] max_chip_rev_full
 *   24  esp_image_segment_header_t, 8 bytes
 *   32  esp_app_desc_t: magic 0xABCD5432, then at +16 version[32],
 *       +48 project_name[32], +80 time[16], +96 date[16], +112 idf_ver[32]
 */
public data class EspAppImage(
    val chipId: Int,
    val minChipRevFull: Int,
    val maxChipRevFull: Int,
    val version: String,
    val projectName: String,
    val buildTime: String,
    val buildDate: String,
    val idfVersion: String,
) {
    /** Human name for [chipId], or "chip 0x..." for one this app does not know. */
    public val chipName: String get() = CHIP_NAMES[chipId] ?: "chip 0x%04X".format(chipId)

    /**
     * Board revision the image asserts, from the `-v9` in a version string like
     * `537dc3ff-dirty-v9+20260909-1835`. Null when the build carries no suffix
     * (the mini's single-MCU project does not).
     *
     * This is what the image CLAIMS, which is not the same as what the board
     * is — a wrongly flashed board reports the wrong revision forever. Good
     * enough to warn on, never to decide on.
     */
    public val boardSuffix: String? get() =
        Regex("-([vV]\\d+)(?:[+\\-]|$)").find(version)?.groupValues?.get(1)?.lowercase()

    public companion object {
        public val CHIP_NAMES: Map<Int, String> = mapOf(
            0x0000 to "ESP32", 0x0002 to "ESP32-S2", 0x0005 to "ESP32-C3",
            0x0009 to "ESP32-S3", 0x000C to "ESP32-C2", 0x000D to "ESP32-C6",
            0x0010 to "ESP32-H2", 0x0012 to "ESP32-P4",
        )
    }
}

/** What the app should do with a picked file. */
public sealed class EspImageVerdict {
    /** Belongs here. */
    public data class Ok(val image: EspAppImage) : EspImageVerdict()

    /**
     * Belongs here, but something is worth saying out loud first — a chip that
     * is not the one this unit usually runs, or a board suffix that disagrees
     * with the firmware currently on the box.
     */
    public data class Warn(val image: EspAppImage, val reason: String) : EspImageVerdict()

    /** Do not flash. [reason] is written for the operator, not the log. */
    public data class Refuse(val image: EspAppImage?, val reason: String) : EspImageVerdict()
}

public object EspImage {
    private const val IMAGE_MAGIC = 0xE9
    private const val APP_DESC_MAGIC = 0xABCD5432.toInt()
    private const val APP_DESC_OFFSET = 32
    private const val MIN_LEN = APP_DESC_OFFSET + 256

    // CMake project() names, which is what ESP-IDF stamps into the image.
    public const val PROJECT_FC: String = "flight_computer"
    public const val PROJECT_OC: String = "out_computer"
    public const val PROJECT_BS: String = "base_station"
    public const val PROJECT_MINI: String = "rocket_computer_mini"

    private fun u16(b: ByteArray, o: Int): Int =
        (b[o].toInt() and 0xFF) or ((b[o + 1].toInt() and 0xFF) shl 8)

    private fun u32(b: ByteArray, o: Int): Int =
        (b[o].toInt() and 0xFF) or ((b[o + 1].toInt() and 0xFF) shl 8) or
            ((b[o + 2].toInt() and 0xFF) shl 16) or ((b[o + 3].toInt() and 0xFF) shl 24)

    /** NUL-terminated fixed-width ASCII field; anything unprintable ends it. */
    private fun str(b: ByteArray, o: Int, len: Int): String {
        val sb = StringBuilder()
        for (i in o until minOf(o + len, b.size)) {
            val c = b[i].toInt() and 0xFF
            if (c == 0 || c < 0x20 || c > 0x7E) break
            sb.append(c.toChar())
        }
        return sb.toString()
    }

    /** Parse an ESP-IDF app image, or null if this is not one. */
    public fun parse(bytes: ByteArray): EspAppImage? {
        if (bytes.size < MIN_LEN) return null
        if ((bytes[0].toInt() and 0xFF) != IMAGE_MAGIC) return null
        if (u32(bytes, APP_DESC_OFFSET) != APP_DESC_MAGIC) return null
        return EspAppImage(
            chipId = u16(bytes, 12),
            minChipRevFull = u16(bytes, 15),
            maxChipRevFull = u16(bytes, 17),
            version = str(bytes, APP_DESC_OFFSET + 16, 32),
            projectName = str(bytes, APP_DESC_OFFSET + 48, 32),
            buildTime = str(bytes, APP_DESC_OFFSET + 80, 16),
            buildDate = str(bytes, APP_DESC_OFFSET + 96, 16),
            idfVersion = str(bytes, APP_DESC_OFFSET + 112, 32),
        )
    }

    /**
     * Decide whether [bytes] may be flashed to a unit running [expectedProject].
     *
     * [expectedChipId] and [runningVersion] are advisory: the flight computer
     * is an ESP32-P4 on the V9 board and an ESP32-S3 on the mini, so a chip
     * mismatch is a warning rather than a refusal, and the running version is
     * the box's own claim about itself.
     */
    /**
     * #773 step 2: [provisionedBoard] is what the BOARD says it is, read from
     * its own NVS and untouched by an OTA. When present it WINS over
     * [runningVersion], because the running version is the image's claim and a
     * wrongly flashed board repeats that wrong claim forever. The fallback is
     * kept for a board that has never been provisioned, where the circular
     * value is still better than nothing — but the warning says which source it
     * used, so nobody reads a fallback comparison as authoritative.
     */
    public fun check(
        bytes: ByteArray,
        expectedProject: String,
        expectedChipId: Int? = null,
        runningVersion: String? = null,
        provisionedBoard: String? = null,
    ): EspImageVerdict {
        val img = parse(bytes)
            ?: return EspImageVerdict.Refuse(
                null,
                "This file is not an ESP-IDF firmware image. Pick the .bin " +
                    "produced by the build, not a .zip, an .elf or a log."
            )
        if (img.projectName != expectedProject) {
            val what = if (img.projectName.isEmpty()) "an unnamed program"
                       else "\"${img.projectName}\""
            return EspImageVerdict.Refuse(
                img,
                "This image is $what, but you are updating " +
                    "\"$expectedProject\". Flashing it would leave that unit " +
                    "running the wrong program."
            )
        }
        val warnings = mutableListOf<String>()
        if (expectedChipId != null && img.chipId != expectedChipId) {
            warnings += "built for ${img.chipName}, but this unit is normally " +
                (EspAppImage.CHIP_NAMES[expectedChipId] ?: "chip 0x%04X".format(expectedChipId))
        }
        val provisioned = provisionedBoard?.trim()?.lowercase()?.ifEmpty { null }
        val fromVersion = runningVersion?.let {
            Regex("-([vV]\\d+)(?:[+\\-]|$)").find(it)?.groupValues?.get(1)?.lowercase()
        }
        val picked = img.boardSuffix
        if (picked != null) {
            if (provisioned != null && provisioned != picked) {
                warnings += "built for board $picked, but this board is provisioned as $provisioned"
            } else if (provisioned == null && fromVersion != null && fromVersion != picked) {
                warnings += "built for board $picked, but this unit's firmware " +
                    "reports $fromVersion (board not provisioned, so this is the " +
                    "image's own claim)"
            }
        }
        return if (warnings.isEmpty()) EspImageVerdict.Ok(img)
        else EspImageVerdict.Warn(img, warnings.joinToString("; "))
    }
}
