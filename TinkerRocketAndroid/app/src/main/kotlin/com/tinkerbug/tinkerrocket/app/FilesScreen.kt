package com.tinkerbug.tinkerrocket.app

import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Checkbox
import androidx.compose.material3.TextButton
import androidx.compose.foundation.clickable
import android.content.Context
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.tinkerbug.tinkerrocket.protocol.CsvGenerator
import com.tinkerbug.tinkerrocket.protocol.FileInfo
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.DownloadResult
import com.tinkerbug.tinkerrocket.session.FleetDevice
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File
import java.util.Locale
import androidx.compose.foundation.background
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ChevronLeft
import androidx.compose.material.icons.filled.ChevronRight
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Description
import androidx.compose.material.icons.filled.Download
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material.icons.filled.Share
import androidx.compose.material.icons.filled.ShowChart
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import com.tinkerbug.tinkerrocket.protocol.FilePageNavigator

/**
 * Phase 4 slice — the post-flight loop: paged device file list (5/page,
 * FILES_PER_PAGE contract), download over the session engine, on-device
 * bin→CSV via the golden-pinned CsvGenerator, storage in the iOS-mirror
 * layout (filesDir/BinaryCache + CSVCache — durable app files, NEVER
 * cacheDir), and share via FileProvider.
 */
@Composable
fun FilesScreen(device: FleetDevice<DeviceSession>, fleetScope: CoroutineScope) {
    // Post-flight chart route: set = show the chart for that cached CSV.
    var chartCsv by androidx.compose.runtime.remember {
        androidx.compose.runtime.mutableStateOf<java.io.File?>(null)
    }
    chartCsv?.let { csv ->
        FlightChartScreen(csvFile = csv, onBack = { chartCsv = null })
        return
    }
    val session = device.session
    val files by session.files.collectAsState()
    val page by session.currentPage.collectAsState()
    val hasMore by session.hasMoreFiles.collectAsState()
    val downloadState by session.downloadState.collectAsState()
    val context = LocalContext.current

    var status by remember { mutableStateOf<String?>(null) }

    // #634 multi-select, mirroring iOS FileManagerView: keyed by file NAME, not
    // index, so a selection survives paging — you can select across pages and
    // the count reflects the whole set even when some rows aren't visible.
    var selecting by remember { mutableStateOf(false) }
    var selection by remember { mutableStateOf<Set<String>>(emptySet()) }
    var confirmBulkDelete by remember { mutableStateOf(false) }

    // Fetch page 0 on entry (once per session shown).
    androidx.compose.runtime.LaunchedEffect(session) {
        if (files.isEmpty()) session.requestFileList(0)
    }

    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors

    Column(
        Modifier.fillMaxSize().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        // Storage bar first (iOS StorageBarView sits directly under the nav
        // bar): lives on the Files screen like iOS, not the dashboard (moved
        // 2026-07-31).  0xCC on rocket links, 0xCD on BS links.
        val rocketStorage by session.rocketStorage.collectAsState()
        val bsStorage by session.bsStorage.collectAsState()
        StorageCard(session.isBaseStation, rocketStorage, bsStorage)

        // iOS totalPages: rocket flightCount is authoritative (also kills the
        // phantom next page an exactly-full single page would imply); the BS
        // reports no count → discovered mode.
        val totalPages = rocketStorage
            ?.takeIf { !session.isBaseStation && it.initialized }
            ?.let { FilePageNavigator.totalPages(it.flightCount, FilePageNavigator.FILES_PER_PAGE) }

        // iOS header row: title + page/count caption left, Select/Cancel and
        // the blue Refresh pill right.
        Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.weight(1f)) {
                Text(
                    if (session.isBaseStation) "LoRa Logs" else "Flights",
                    style = MaterialTheme.typography.titleLarge,
                )
                Text(
                    when {
                        (totalPages ?: 0) > 1 -> "Page ${page + 1} of $totalPages"
                        page > 0 || hasMore -> "Page ${page + 1}"
                        session.isBaseStation -> "${files.size} logs"
                        else -> "${files.size} flights"
                    },
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            if (selecting) {
                TextButton(onClick = { selecting = false; selection = emptySet() }) {
                    Text("Cancel", color = tr.savedFlights)
                }
            } else {
                if (files.isNotEmpty()) {
                    TextButton(
                        onClick = { selecting = true },
                        enabled = !downloadState.active,
                    ) { Text("Select", color = tr.savedFlights) }
                }
                Row(
                    Modifier
                        .background(tr.savedFlights, RoundedCornerShape(8.dp))
                        .clickable { session.requestFileList(page) }
                        .padding(horizontal = 12.dp, vertical = 6.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(4.dp),
                ) {
                    Icon(
                        Icons.Filled.Refresh, contentDescription = null,
                        tint = Color.White, modifier = Modifier.size(16.dp),
                    )
                    Text("Refresh", color = Color.White, style = MaterialTheme.typography.bodyMedium)
                }
            }
        }

        if (downloadState.active) {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Row(Modifier.fillMaxWidth()) {
                    Text(
                        "Downloading ${downloadState.filename ?: "file"}...",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.weight(1f),
                    )
                    Text(
                        "${(downloadState.progress * 100).toInt()}%",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                LinearProgressIndicator(
                    progress = { downloadState.progress.toFloat() },
                    modifier = Modifier.fillMaxWidth(),
                )
            }
        }
        // Android-only transient status line (download/CSV/delete results) —
        // iOS reports CSV progress with a dedicated banner instead; our
        // conversion is synchronous inside downloadAndConvert.
        status?.let {
            Text(
                it,
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }

        if (confirmBulkDelete) {
            val noun = if (session.isBaseStation) "LoRa Log" else "Flight"
            val nounLower = if (session.isBaseStation) "log" else "flight"
            val where = if (session.isBaseStation) "base station" else "rocket"
            val n = selection.size
            val s = if (n == 1) "" else "s"
            AlertDialog(
                onDismissRequest = { confirmBulkDelete = false },
                title = { Text("Delete $n $noun$s?") },
                text = {
                    Text(
                        "Are you sure you want to delete $n $nounLower$s from the $where? " +
                            "This cannot be undone.",
                    )
                },
                confirmButton = {
                    TextButton(onClick = {
                        // The name-keyed selection survives paging by design —
                        // delete the WHOLE set, not just the current page.
                        session.deleteFiles(selection.toList())
                        status = "Deleted $n file$s"
                        confirmBulkDelete = false
                        selecting = false
                        selection = emptySet()
                        // Resync: a bulk delete can empty the current page or
                        // shift the total, so go back to page 0 (iOS does the
                        // same in deleteSelected()).
                        session.requestFileList(0)
                    }) { Text("Delete") }
                },
                dismissButton = {
                    TextButton(onClick = { confirmBulkDelete = false }) { Text("Cancel") }
                },
            )
        }

        // iOS empty state: icon + guidance instead of a blank list.
        if (files.isEmpty() && !downloadState.active) {
            Column(
                Modifier.fillMaxWidth().padding(vertical = 32.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Icon(
                    Icons.Filled.Description, contentDescription = null,
                    modifier = Modifier.size(48.dp),
                    tint = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Text(
                    if (session.isBaseStation) "No LoRa logs found" else "No flights found",
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Text(
                    "Tap Refresh to load files",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }

        var fileToDelete by remember { mutableStateOf<FileInfo?>(null) }
        fileToDelete?.let { doomed ->
            AlertDialog(
                onDismissRequest = { fileToDelete = null },
                title = { Text(if (session.isBaseStation) "Delete LoRa Log?" else "Delete Flight?") },
                text = {
                    Text(
                        "Are you sure you want to delete \"${displayTitle(doomed)}\" from the " +
                            (if (session.isBaseStation) "base station" else "rocket") +
                            "? This cannot be undone.",
                    )
                },
                confirmButton = {
                    TextButton(onClick = {
                        session.deleteFiles(listOf(doomed.name))
                        status = "Deleted ${displayTitle(doomed)}"
                        fileToDelete = null
                        session.requestFileList(0)
                    }) { Text("Delete") }
                },
                dismissButton = {
                    TextButton(onClick = { fileToDelete = null }) { Text("Cancel") }
                },
            )
        }

        LazyColumn(Modifier.weight(1f, fill = false), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            items(files, key = { it.name }) { file ->
                FileRow(
                    file = file,
                    busy = downloadState.active,
                    onDownload = {
                        status = null
                        fleetScope.launch {
                            status = downloadAndConvert(context, session, file)
                        }
                    },
                    onShare = { shareCsvIfPresent(context, file) },
                    onChart = { chartCsv = csvFileFor(context, file.name) },
                    hasCsv = csvFileFor(context, file.name).exists(),
                    selecting = selecting,
                    selected = file.name in selection,
                    onToggleSelected = {
                        selection = if (file.name in selection) selection - file.name
                                    else selection + file.name
                    },
                    onDelete = { fileToDelete = file },
                )
            }
        }

        // iOS FilePageNavigator: chevrons + numbered circular pills below the
        // list (fixed 1..N on rocket links, discovered pages + "…" on BS).
        FilePageNavigatorBar(
            currentPage = page,
            totalPages = totalPages,
            hasMore = hasMore,
            onSelect = { session.requestFileList(it) },
        )

        // iOS selection action bar at the bottom: Select all/Deselect all +
        // the red Delete pill.  Deletes are irreversible and the files are
        // the only copy until downloaded, so this always confirms and always
        // names the count.
        if (selecting) {
            val allOnPageSelected =
                files.isNotEmpty() && files.all { it.name in selection }
            Row(
                Modifier.fillMaxWidth().padding(vertical = 4.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TextButton(onClick = {
                    // iOS: acts on the CURRENT page's names only; the
                    // selection itself survives paging.
                    val pageNames = files.map { it.name }
                    selection = if (allOnPageSelected) selection - pageNames.toSet()
                                else selection + pageNames
                }) {
                    Text(
                        if (allOnPageSelected) "Deselect all" else "Select all",
                        color = tr.savedFlights,
                    )
                }
                Box(Modifier.weight(1f))
                val deleteEnabled = selection.isNotEmpty() && !downloadState.active
                Row(
                    Modifier
                        .background(
                            if (deleteEnabled) tr.driftCast else tr.statusIdle,
                            RoundedCornerShape(8.dp),
                        )
                        .let { if (deleteEnabled) it.clickable { confirmBulkDelete = true } else it }
                        .padding(horizontal = 16.dp, vertical = 8.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    Icon(
                        Icons.Filled.Delete, contentDescription = null,
                        tint = Color.White, modifier = Modifier.size(16.dp),
                    )
                    Text(
                        if (selection.isEmpty()) "Delete" else "Delete (${selection.size})",
                        color = Color.White,
                        style = MaterialTheme.typography.bodyMedium,
                    )
                }
            }
        }
    }
}

/**
 * iOS FilePageNavigator twin (logic in :core:protocol FilePageNavigator).
 * Hidden entirely for a single known page; discovered mode trails a "…"
 * while the wire hasMore flag stays set.
 */
@Composable
private fun FilePageNavigatorBar(
    currentPage: Int,
    totalPages: Int?,
    hasMore: Boolean,
    onSelect: (Int) -> Unit,
) {
    val show = if (totalPages != null && totalPages > 0) totalPages > 1
               else currentPage > 0 || hasMore
    if (!show) return
    val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
    val pages = FilePageNavigator.pageIndices(currentPage, totalPages, hasMore)
    val canNext = FilePageNavigator.canGoNext(currentPage, totalPages, hasMore)

    @Composable
    fun chevron(icon: androidx.compose.ui.graphics.vector.ImageVector, enabled: Boolean, target: Int) {
        Box(
            Modifier
                .size(34.dp)
                .background(if (enabled) tr.savedFlights else tr.statusIdle, CircleShape)
                .let { if (enabled) it.clickable { onSelect(target) } else it },
            contentAlignment = Alignment.Center,
        ) {
            Icon(icon, contentDescription = null, tint = Color.White)
        }
    }

    Row(
        Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        chevron(Icons.Filled.ChevronLeft, currentPage > 0, currentPage - 1)
        Row(
            Modifier.weight(1f).horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            pages.forEach { p ->
                val current = p == currentPage
                Box(
                    Modifier
                        .size(34.dp)
                        .background(
                            if (current) tr.savedFlights else tr.cardSecondary,
                            CircleShape,
                        )
                        .let { if (current) it else it.clickable { onSelect(p) } },
                    contentAlignment = Alignment.Center,
                ) {
                    Text(
                        "${p + 1}",
                        color = if (current) Color.White else MaterialTheme.colorScheme.onSurface,
                        style = MaterialTheme.typography.bodyMedium.copy(
                            fontWeight = if (current) FontWeight.SemiBold else FontWeight.Normal,
                        ),
                    )
                }
            }
            if (totalPages == null && hasMore) {
                Text("…", color = MaterialTheme.colorScheme.onSurfaceVariant)
            }
        }
        chevron(Icons.Filled.ChevronRight, canNext, currentPage + 1)
    }
}

/** iOS FileInfo.displayTitle: the parsed flight/lora timestamp in the local
 *  zone, short date + short time; raw filename when unparseable. */
private fun displayTitle(file: FileInfo): String {
    val instant = file.flightDateUtc ?: return file.name
    return java.time.format.DateTimeFormatter
        .ofLocalizedDateTime(java.time.format.FormatStyle.SHORT, java.time.format.FormatStyle.SHORT)
        .withZone(java.time.ZoneId.systemDefault())
        .format(instant)
}

/** iOS formatFileSize: KiB math under KB/MB labels — ported exactly. */
private fun formatFileSize(bytes: Long): String {
    val kb = bytes / 1024.0
    return if (kb < 1024) String.format(Locale.ROOT, "%.1f KB", kb)
    else String.format(Locale.ROOT, "%.2f MB", kb / 1024.0)
}

@Composable
private fun FileRow(
    file: FileInfo,
    busy: Boolean,
    hasCsv: Boolean,
    onDownload: () -> Unit,
    onShare: () -> Unit,
    onChart: () -> Unit,
    selecting: Boolean = false,
    selected: Boolean = false,
    onToggleSelected: () -> Unit = {},
    onDelete: () -> Unit = {},
) {
    Card(
        Modifier.fillMaxWidth().let {
            // Whole row toggles while selecting — a checkbox alone is a
            // thumb-miss target, the same finding that moved the chart column
            // picker to row-click.
            if (selecting) it.clickable(onClick = onToggleSelected) else it
        },
    ) {
        Row(
            Modifier.fillMaxWidth().padding(14.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            if (selecting) {
                Checkbox(checked = selected, onCheckedChange = { onToggleSelected() })
            }
            Column(Modifier.weight(1f)) {
                // iOS FileRow: localized flight date as the title (raw name
                // only when unparseable), KB/MB size caption.
                Text(displayTitle(file), style = MaterialTheme.typography.bodyLarge)
                Text(
                    formatFileSize(file.size),
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            // Per-row actions would fight the selection gesture, so they step
            // aside while selecting.
            if (selecting) {
                // nothing — the action bar owns the verbs in this mode
            } else {
                // iOS-style trailing icon actions.  Android keeps its richer
                // post-download verbs (chart + share) alongside iOS's
                // download/trash pair.
                val tr = com.tinkerbug.tinkerrocket.app.theme.TrTheme.colors
                if (hasCsv) {
                    IconButton(onClick = onChart) {
                        Icon(
                            Icons.Filled.ShowChart, contentDescription = "Chart",
                            tint = tr.savedFlights,
                        )
                    }
                    IconButton(onClick = onShare) {
                        Icon(
                            Icons.Filled.Share, contentDescription = "Share",
                            tint = tr.savedFlights,
                        )
                    }
                } else {
                    IconButton(onClick = onDownload, enabled = !busy) {
                        Icon(
                            Icons.Filled.Download, contentDescription = "Download",
                            tint = if (busy) tr.statusIdle else tr.savedFlights,
                        )
                    }
                }
                IconButton(onClick = onDelete, enabled = !busy) {
                    Icon(
                        Icons.Filled.Delete, contentDescription = "Delete",
                        tint = tr.driftCast,
                    )
                }
            }
        }
    }
}

// iOS FileCache mirror.  Moved to FlightCache for #635 so Saved Flights can
// browse the same layout with no device attached; these stay as thin aliases
// to keep this screen readable.
private fun binFileFor(context: Context, name: String) = FlightCache.binFileFor(context, name)

private fun csvFileFor(context: Context, name: String) = FlightCache.csvFileFor(context, name)

private fun summaryFileFor(context: Context, name: String) =
    FlightCache.summaryFileFor(context, name)

/** Runs on the fleet dispatcher (downloadFile contract); CSV work hops to Default. */
private suspend fun downloadAndConvert(
    context: Context,
    session: DeviceSession,
    file: FileInfo,
): String = when (val result = session.downloadFile(file.name)) {
    is DownloadResult.Success -> withContext(Dispatchers.Default) {
        binFileFor(context, file.name).writeBytes(result.bytes)
        try {
            val (csv, summary) = CsvGenerator().writeCsv(result.bytes)
            csvFileFor(context, file.name).writeText(csv)
            summaryFileFor(context, file.name).writeText(summary.toJson())
            "Saved ${file.name} (${result.bytes.size / 1000} kB) + CSV"
        } catch (e: Exception) {
            // Keep the .bin either way — the data survives a converter bug.
            "Saved .bin; CSV failed: ${e.message}"
        }
    }
    else -> "Download failed: $result"
}

private fun shareCsvIfPresent(context: Context, file: FileInfo) =
    FlightCache.shareCsv(context, csvFileFor(context, file.name))
