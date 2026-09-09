package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.content.Intent
import androidx.core.content.FileProvider
import java.io.File
import com.tinkerbug.tinkerrocket.protocol.FlightSummary

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
    ) {
        val displayName: String get() = name.removeSuffix(".bin")
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
            SavedFlight(
                name = name,
                bin = bin,
                csv = csv,
                // Prefer the .bin's size — it's what came off the board, and
                // what a "how big was this flight" glance means.
                sizeBytes = bin?.length() ?: csv?.length() ?: 0L,
                lastModified = maxOf(bin?.lastModified() ?: 0L, csv?.lastModified() ?: 0L),
                summary = summaryFileFor(context, name).takeIf { it.exists() }
                    ?.let { runCatching { FlightSummary.fromJson(it.readText()) }.getOrNull() },
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
        return raw.exists() && raw.length() == expectedSize
    }
}
