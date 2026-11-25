package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.content.Intent
import androidx.core.content.FileProvider
import java.io.File
import com.tinkerbug.tinkerrocket.protocol.FlightFiles
import com.tinkerbug.tinkerrocket.protocol.FlightSettingsData
import com.tinkerbug.tinkerrocket.protocol.FlightSource
import com.tinkerbug.tinkerrocket.protocol.FlightSummary
import com.tinkerbug.tinkerrocket.protocol.MessageParser
import com.tinkerbug.tinkerrocket.protocol.MsgType

/**
 * On-phone flight storage — the iOS FileCache mirror.
 *
 * `filesDir/BinaryCache` and `filesDir/CSVCache`: durable app files, NEVER
 * `cacheDir`, because the OS may purge that under storage pressure and this is
 * field data that only exists here until it's exported.
 *
 * Extracted from FilesScreen for #635: nothing about reading these needs a BLE
 * connection, but they used to be private to the screen that does. Saved
 * Flights browses the same layout with no device attached.
 */
internal object FlightCache {

    fun binDir(context: Context): File =
        File(context.filesDir, "BinaryCache").apply { mkdirs() }

    fun csvDir(context: Context): File =
        File(context.filesDir, "CSVCache").apply { mkdirs() }

    fun binFileFor(context: Context, name: String): File =
        File(binDir(context), name)

    // #1062: a legacy base-station log is ALREADY a .csv (pre-a53d337 boards
    // wrote ASCII); appending a second extension pointed hasCsv, Share and
    // Chart at a "lora_003.csv.csv" that was never written.
    fun csvFileFor(context: Context, name: String): File =
        File(csvDir(context), if (name.endsWith(".csv")) name else name.removeSuffix(".bin") + ".csv")

    fun summaryFileFor(context: Context, name: String): File =
        File(csvDir(context), name.removeSuffix(".bin") + ".json")

    private fun isUnitId(id: String): Boolean =
        id.length in 1..16 && id.all { it in '0'..'9' || it in 'a'..'f' || it in 'A'..'F' }

    /**
     * The name a board's flight is cached under on this phone. Boards name logs
     * on their own — `flight_<n>.bin` when no phone has synced the time — so
     * two boards can both write `flight_3.bin`, and caching by that name alone
     * let the second download replace the first board's flight. The unit ID
     * keeps them apart: "flight_3.bin" from a1cae150 is cached as
     * "flight_3__a1cae150.bin", and its CSV and summary follow. Only rocket
     * flights (`flight_*`); base-station logs keep their names. An empty or
     * malformed unit ID keeps the board's name, as before. Same as iOS
     * `FileCache.cacheName(for:unitID:)`.
     */
    fun cacheName(boardName: String, unitId: String): String {
        if (!boardName.startsWith("flight_") || !isUnitId(unitId)) return boardName
        val dot = boardName.lastIndexOf('.')
        val stem = if (dot > 0) boardName.substring(0, dot) else boardName
        val ext = if (dot > 0) boardName.substring(dot) else ""
        if (stem.endsWith("__$unitId")) return boardName
        return "${stem}__$unitId$ext"
    }

    /** The unit ID a cache name carries ("flight_3__a1cae150.bin" -> "a1cae150"), or null. */
    fun unitIdInCacheName(name: String): String? {
        val stem = name.substringBeforeLast('.')
        val i = stem.lastIndexOf("__")
        if (i < 0) return null
        return stem.substring(i + 2).takeIf(::isUnitId)
    }

    /**
     * The name this unit's copy of a board file is cached under: the
     * unit-qualified name, or the board's own name for a copy downloaded before
     * unit IDs were added that is not known to be another unit's (its summary's
     * `source.unit_id`). An old copy with no `source` keeps reading as
     * downloaded, as it did before. Same as iOS `BLEDevice.cachedName(for:)`.
     */
    fun cachedName(context: Context, boardName: String, unitId: String): String {
        val qualified = cacheName(boardName, unitId)
        if (qualified == boardName) return qualified
        if (binFileFor(context, qualified).exists() || csvFileFor(context, qualified).exists()) return qualified
        val legacy = binFileFor(context, boardName).exists() || csvFileFor(context, boardName).exists()
        if (!legacy) return qualified
        val owner = summaryFileFor(context, boardName).takeIf { it.exists() }
            ?.let { runCatching { FlightSummary.sourceFromJson(it.readText()) }.getOrNull() }
            ?.unitId
        return if (owner == null || owner == unitId) boardName else qualified
    }

    /**
     * #1658: the FC's board for a flight whose summary predates
     * `settings.board`, read from the first flight-settings frame in its log.
     * null when the log predates the field (v9, #413) or cannot be read. It
     * reads the whole .bin, so call it off the main thread.
     */
    fun boardFromLog(bin: File): String? = runCatching {
        MessageParser.firstFrame(bin.readBytes(), MsgType.FLIGHT_SETTINGS)
            ?.let { FlightSettingsData.decode(it.payload) }
            ?.boardRevName
    }.getOrNull()

    /**
     * One downloaded flight as it exists on the phone.
     *
     * A flight can be half-present: the download writes the `.bin` first and
     * keeps it even if CSV conversion throws, deliberately, so a converter bug
     * never costs the data. So [csv] is nullable and the UI has to cope with a
     * `.bin` that can't be charted yet.
     */
    data class SavedFlight(
        val name: String,          // "flight_20260729_163740.bin"
        val bin: File?,
        val csv: File?,
        val sizeBytes: Long,
        val lastModified: Long,
        /** #1091 item 4: the headline numbers from the `.json` sidecar, when present. */
        val summary: FlightSummary? = null,
        /**
         * #1658: the rocket and computer it was downloaded from (summary
         * `source`); null for flights downloaded before the app recorded it.
         */
        val source: FlightSource? = null,
        /**
         * #1658: the FC's board from the log (summary `settings.board`). null
         * when the summary predates the field; the row then reads the log.
         */
        val fcBoard: String? = null,
    ) {
        /** A unit-qualified cache name reads "flight_3 · a1cae150". */
        val displayName: String get() {
            val stem = name.removeSuffix(".bin")
            val unit = unitIdInCacheName(name) ?: return stem
            return stem.dropLast(unit.length + 2) + " · " + unit
        }
        val hasCsv: Boolean get() = csv != null
        /** #1067: a bin-only flight (converter failed) is still exportable. */
        val hasAnyFile: Boolean get() = bin != null || csv != null
    }

    /**
     * Everything downloaded to this phone, newest first.
     *
     * Unions both directories rather than listing one: a `.bin` whose CSV
     * conversion failed still belongs in the list (it can be shared and
     * re-converted), and a `.csv` whose `.bin` was cleared is still chartable.
     */
    fun listSavedFlights(context: Context): List<SavedFlight> {
        val bins = binDir(context).listFiles()?.filter { it.isFile && it.name.endsWith(".bin") }
            ?: emptyList()
        val csvs = csvDir(context).listFiles()?.filter { it.isFile && it.name.endsWith(".csv") }
            ?: emptyList()

        val names = LinkedHashSet<String>()
        bins.forEach { names += it.name }
        csvs.forEach { names += it.name.removeSuffix(".csv") + ".bin" }

        return names.map { name ->
            val bin = bins.firstOrNull { it.name == name }
            val csv = csvs.firstOrNull { it.name == name.removeSuffix(".bin") + ".csv" }
            // #1658: one read, three independent decodes (#1077): a block that
            // fails to parse never blanks another.
            val summaryText = summaryFileFor(context, name).takeIf { it.exists() }
                ?.let { runCatching { it.readText() }.getOrNull() }
            SavedFlight(
                name = name,
                bin = bin,
                csv = csv,
                // Prefer the .bin's size — it's what came off the board, and
                // what a "how big was this flight" glance means.
                sizeBytes = bin?.length() ?: csv?.length() ?: 0L,
                lastModified = maxOf(bin?.lastModified() ?: 0L, csv?.lastModified() ?: 0L),
                summary = summaryText?.let(FlightSummary::fromJson),
                source = summaryText?.let(FlightSummary::sourceFromJson),
                fcBoard = summaryText?.let(FlightSummary::boardFromJson),
            )
        }.sortedByDescending { it.lastModified }
    }

    /**
     * #1067: share a flight as every file that exists for it — the raw `.bin`,
     * the converted `.csv` and the `.json` summary — through the app's
     * FileProvider, as iOS does. The post-flight tools take the `.bin` as
     * their only input; sharing the CSV alone stranded every Android log on
     * the phone, and once the board copy was deleted that was the only copy.
     * No-op if none of the files exist. `file_paths.xml` already exposes all
     * of filesDir, so BinaryCache needs no manifest change.
     */
    /**
     * #1080: delete one downloaded flight from THIS PHONE — the .bin, the
     * .csv and the summary sidecar together, the way iOS's swipe-to-delete
     * does. Both platforms store downloads in durable storage on purpose (a
     * purgeable cache dir would lose a flight the operator has not exported
     * yet), so nothing reclaims the space on its own: before this the only
     * remedy on Android was Settings -> Clear storage, which also takes the
     * profiles, the checklists and the offline map tiles with it.
     *
     * Deliberately NOT a device delete: the board keeps its copy (BLE cmd 3
     * is the other verb, on the Files screen).
     */
    fun deleteSavedFlight(context: Context, flight: SavedFlight) {
        listOfNotNull(
            flight.bin,
            flight.csv,
            summaryFileFor(context, flight.name).takeIf { it.exists() },
        ).forEach { runCatching { it.delete() } }
    }

    fun shareFlight(context: Context, files: List<File?>) {
        val present = files.filterNotNull().filter { it.exists() }
        if (present.isEmpty()) return
        val uris = ArrayList(present.map {
            FileProvider.getUriForFile(context, "${context.packageName}.files", it)
        })
        val label = present.firstOrNull { it.name.endsWith(".bin") }?.name ?: present.first().name
        context.startActivity(
            Intent.createChooser(
                Intent(Intent.ACTION_SEND_MULTIPLE).apply {
                    type = "*/*"
                    putParcelableArrayListExtra(Intent.EXTRA_STREAM, uris)
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                },
                "Share ${label.removeSuffix(".bin")}",
            ),
        )
    }

    /**
     * #854 item 2: "already downloaded" used to be `csv.exists()`, so a short
     * transfer that got written rendered as complete forever, with no way to
     * re-pull until the cache was cleared. The download keeps the raw bytes
     * (`binFileFor`), so when the board advertises a size, the cached copy
     * counts only if it matches — the same rule as iOS's
     * `isFlightCached(_:expectedSize:)`. With no advertised size, existence
     * is all there is to go on.
     */
    fun isCachedComplete(context: Context, name: String, expectedSize: Long): Boolean {
        if (!csvFileFor(context, name).exists()) return false
        if (expectedSize <= 0L) return true
        val raw = binFileFor(context, name)
        return raw.exists() &&
            FlightFiles.cachedSizeSatisfiesListing(raw.length(), expectedSize, shortDownload(context, name))
    }

    private const val SHORT_DOWNLOADS = "flight_cache_short_downloads"

    /**
     * A download the board ended itself short of its listing, kept (see
     * DownloadResult.Success.isShortOfListing): remember both sizes so the row
     * reads as downloaded for THAT listing instead of offering the same short
     * download forever. iOS: FileCache.recordShortDownload.
     */
    fun recordShortDownload(context: Context, name: String, listedSize: Long, receivedSize: Long) {
        context.getSharedPreferences(SHORT_DOWNLOADS, Context.MODE_PRIVATE).edit()
            .putString(name, "$listedSize:$receivedSize").apply()
    }

    fun shortDownload(context: Context, name: String): FlightFiles.ShortDownload? {
        val v = context.getSharedPreferences(SHORT_DOWNLOADS, Context.MODE_PRIVATE)
            .getString(name, null) ?: return null
        val parts = v.split(':')
        val listed = parts.getOrNull(0)?.toLongOrNull() ?: return null
        val received = parts.getOrNull(1)?.toLongOrNull() ?: return null
        return FlightFiles.ShortDownload(listed, received)
    }
}
