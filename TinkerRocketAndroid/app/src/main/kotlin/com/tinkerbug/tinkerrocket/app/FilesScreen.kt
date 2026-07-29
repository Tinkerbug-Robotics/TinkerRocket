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

    Column(
        Modifier.fillMaxSize().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Text("Flight logs", style = MaterialTheme.typography.titleLarge)

        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            OutlinedButton(
                onClick = { session.requestFileList(page - 1) },
                enabled = page > 0,
            ) { Text("Prev") }
            Text("Page ${page + 1}", style = MaterialTheme.typography.bodyMedium)
            OutlinedButton(
                onClick = { session.requestFileList(page + 1) },
                enabled = hasMore,
            ) { Text("Next") }
            OutlinedButton(onClick = { session.requestFileList(0) }) { Text("Refresh") }
        }

        if (downloadState.active) {
            Column {
                Text(
                    "Downloading ${downloadState.filename ?: ""}…",
                    style = MaterialTheme.typography.bodyMedium,
                )
                LinearProgressIndicator(
                    progress = { downloadState.progress.toFloat() },
                    modifier = Modifier.fillMaxWidth(),
                )
            }
        }
        status?.let { Text(it, style = MaterialTheme.typography.bodyMedium) }

        // #634 selection action bar.  Deletes are irreversible and the files
        // are the only copy until they're downloaded, so this always confirms
        // and always names the count.
        Row(
            Modifier.fillMaxWidth().padding(vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            if (!selecting) {
                OutlinedButton(
                    onClick = { selecting = true },
                    enabled = files.isNotEmpty() && !downloadState.active,
                ) { Text("Select") }
            } else {
                Text("${selection.size} selected", style = MaterialTheme.typography.bodyMedium)
                OutlinedButton(
                    onClick = { selection = files.map { it.name }.toSet() },
                    enabled = selection.size < files.size,
                ) { Text("All") }
                OutlinedButton(
                    onClick = { confirmBulkDelete = true },
                    enabled = selection.isNotEmpty() && !downloadState.active,
                ) { Text("Delete") }
                TextButton(onClick = { selecting = false; selection = emptySet() }) {
                    Text("Cancel")
                }
            }
        }

        if (confirmBulkDelete) {
            AlertDialog(
                onDismissRequest = { confirmBulkDelete = false },
                title = { Text("Delete ${selection.size} file${if (selection.size == 1) "" else "s"}?") },
                text = {
                    Text(
                        "This erases them from the rocket. Anything already downloaded stays " +
                            "on this phone under Saved Flights.",
                    )
                },
                confirmButton = {
                    TextButton(onClick = {
                        val doomed = files.map { it.name }.filter { it in selection }
                        session.deleteFiles(doomed)
                        status = "Deleted ${doomed.size} file(s)"
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

        LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
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
                )
            }
        }
    }
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
                Text(file.name, style = MaterialTheme.typography.titleSmall)
                Text(
                    String.format(Locale.ROOT, "%.1f kB", file.size / 1000.0),
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            // Per-row actions would fight the selection gesture, so they step
            // aside while selecting.
            if (selecting) {
                // nothing — the action bar owns the verbs in this mode
            } else if (hasCsv) {
                OutlinedButton(onClick = onChart) { Text("Chart") }
                OutlinedButton(
                    onClick = onShare,
                    modifier = Modifier.padding(start = 6.dp),
                ) { Text("Share") }
            } else {
                Button(onClick = onDownload, enabled = !busy) { Text("Download") }
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
