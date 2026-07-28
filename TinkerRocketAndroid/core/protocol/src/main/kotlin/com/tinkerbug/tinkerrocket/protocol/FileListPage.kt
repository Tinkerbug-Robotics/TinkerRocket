package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.time.format.ResolverStyle
import java.util.Locale
import kotlin.math.floor

/**
 * One entry of the on-device file list — port of iOS `FileInfo`
 * (BLETypes.swift): `{"name": String, "size": UInt32}`.
 */
public data class FileInfo(
    val name: String,
    /** Wire u32 (device-reported byte size), widened to Long. */
    val size: Long,
) {
    /**
     * UTC timestamp parsed from "flight_YYYYMMDD?HHMMSS…" / "lora_…" device
     * names — a port of iOS `FileInfo.flightDate` including its quirks:
     *  - the minimum-length gates are 26 for "flight_" and 23 for "lora_"
     *    (full-name length, extension included — NOT prefix+15);
     *  - the separator char between date and time is skipped WITHOUT being
     *    checked (any single character passes);
     *  - trailing characters after the 6 time digits are ignored;
     *  - impossible dates (month 13, Feb 30) are rejected (iOS non-lenient
     *    DateFormatter → STRICT resolver here, same as FlightFiles).
     *
     * NOTE: this is the DEVICE-LIST variant; the cached-file variant with its
     * ".csv"/".bin.csv" stripping lives in [FlightFiles.flightTimestampUtc].
     * The localized display string (iOS `displayTitle`) is a UI concern.
     */
    public val flightDateUtc: Instant?
        get() {
            val prefixLen = when {
                name.startsWith("flight_") && name.length >= 26 -> 7
                name.startsWith("lora_") && name.length >= 23 -> 5
                else -> return null
            }
            val dateStr = name.substring(prefixLen, prefixLen + 8)
            val timeStr = name.substring(prefixLen + 9, prefixLen + 15) // separator unchecked
            if (!dateStr.all { it.isDigit() } || !timeStr.all { it.isDigit() }) return null
            return try {
                LocalDateTime.parse(dateStr + timeStr, UTC_TIMESTAMP_FORMAT)
                    .toInstant(ZoneOffset.UTC)
            } catch (_: Exception) {
                null
            }
        }

    private companion object {
        val UTC_TIMESTAMP_FORMAT: DateTimeFormatter =
            DateTimeFormatter.ofPattern("uuuuMMddHHmmss", Locale.ROOT)
                .withResolverStyle(ResolverStyle.STRICT)
    }
}

/**
 * A decoded page of the device file list (cmd-2 readback on the file_ops
 * characteristic).  The firmware pages [FILES_PER_PAGE] entries at a time;
 * iOS infers "more pages exist" from a FULL page — a device with an exact
 * multiple of 5 files therefore serves one final empty page.  That heuristic
 * is pinned here as [hasMore].
 */
public data class FileListPage(
    val files: List<FileInfo>,
) {
    /** iOS: `hasMoreFiles = (fileList.count == 5)`. */
    public val hasMore: Boolean get() = files.size == FILES_PER_PAGE

    public companion object {
        /** Single source of truth is [FilePageNavigator.FILES_PER_PAGE]. */
        public const val FILES_PER_PAGE: Int = FilePageNavigator.FILES_PER_PAGE

        /**
         * Strict parse of the JSON array, mirroring iOS
         * `JSONDecoder().decode([FileInfo].self, ...)`: any malformed entry
         * fails the WHOLE page (iOS logs and keeps the previous list).
         * Requirements per entry: "name" must be a JSON string, "size" an
         * exact-integral JSON number within u32 (5.0 passes, 5.5 / -1 /
         * 2^32 / "5" / true all fail — `UInt32(exactly:)` unboxing).
         */
        public fun decode(bytes: ByteArray): FileListPage? = decode(bytes.decodeToString())

        public fun decode(text: String): FileListPage? {
            val root = try {
                Json.parseToJsonElement(text)
            } catch (_: Exception) {
                return null
            }
            val arr = root as? JsonArray ?: return null
            val files = ArrayList<FileInfo>(arr.size)
            for (el in arr) {
                val obj = el as? JsonObject ?: return null
                val nameEl = obj["name"] as? JsonPrimitive ?: return null
                if (!nameEl.isString) return null
                val size = u32Exactly(obj["size"] as? JsonPrimitive ?: return null) ?: return null
                files.add(FileInfo(name = nameEl.content, size = size))
            }
            return FileListPage(files)
        }

        /** JSONDecoder `UInt32` unboxing: exact-integral number in 0..2^32-1. */
        private fun u32Exactly(p: JsonPrimitive): Long? {
            if (p.isString) return null
            when (p.content) {
                "true", "false" -> return null   // CFBoolean → typeMismatch on iOS
            }
            val l = p.content.toLongOrNull() ?: run {
                val d = p.content.toDoubleOrNull() ?: return null
                if (!d.isFinite() || d != floor(d)) return null
                d.toLong().takeIf { it.toDouble() == d } ?: return null
            }
            return l.takeIf { it in 0..0xFFFF_FFFFL }
        }
    }
}
