package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.content.Intent
import androidx.core.content.FileProvider
import java.io.File

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

    fun csvFileFor(context: Context, name: String): File =
        File(csvDir(context), name.removeSuffix(".bin") + ".csv")

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
    ) {
        val displayName: String get() = name.removeSuffix(".bin")
        val hasCsv: Boolean get() = csv != null
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
            )
        }.sortedByDescending { it.lastModified }
    }

    /** Share a converted CSV through the app's FileProvider. No-op if absent. */
    fun shareCsv(context: Context, csv: File?) {
        if (csv == null || !csv.exists()) return
        val uri = FileProvider.getUriForFile(context, "${context.packageName}.files", csv)
        context.startActivity(
            Intent.createChooser(
                Intent(Intent.ACTION_SEND).apply {
                    type = "text/csv"
                    putExtra(Intent.EXTRA_STREAM, uri)
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                },
                "Share ${csv.name}",
            ),
        )
    }
}
