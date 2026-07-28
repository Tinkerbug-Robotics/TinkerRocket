package com.tinkerbug.tinkerrocket.app

import android.content.Context
import android.content.Intent
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
import androidx.core.content.FileProvider
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
) {
    Card(Modifier.fillMaxWidth()) {
        Row(
            Modifier.fillMaxWidth().padding(14.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Column(Modifier.weight(1f)) {
                Text(file.name, style = MaterialTheme.typography.titleSmall)
                Text(
                    String.format(Locale.ROOT, "%.1f kB", file.size / 1000.0),
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            if (hasCsv) {
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

/** iOS FileCache mirror: durable app files, shallow flight_* naming. */
private fun binFileFor(context: Context, name: String) =
    File(File(context.filesDir, "BinaryCache").apply { mkdirs() }, name)

private fun csvFileFor(context: Context, name: String) =
    File(
        File(context.filesDir, "CSVCache").apply { mkdirs() },
        name.removeSuffix(".bin") + ".csv",
    )

private fun summaryFileFor(context: Context, name: String) =
    File(
        File(context.filesDir, "CSVCache").apply { mkdirs() },
        name.removeSuffix(".bin") + ".json",
    )

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

private fun shareCsvIfPresent(context: Context, file: FileInfo) {
    val csv = csvFileFor(context, file.name)
    if (!csv.exists()) return
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
