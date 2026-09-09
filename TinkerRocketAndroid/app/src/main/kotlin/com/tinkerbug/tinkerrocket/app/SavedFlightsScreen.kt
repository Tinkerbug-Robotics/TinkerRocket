package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import androidx.compose.material3.AlertDialog

/**
 * Flights already downloaded to this phone — browsable with nothing connected.
 *
 * #635: the chart and share paths read purely from `filesDir`, but they lived
 * only inside FilesScreen, which is mounted in the connected-device tab row.
 * That made the normal post-flight workflow impossible: download logs at the
 * pad, drive home, and the data was unreachable without powering a board and
 * reconnecting to it. iOS has had "Saved Flights" on its main screen all along
 * (`DashboardView.swift:92`).
 *
 * Deliberately does NOT list the device's own files — that half genuinely needs
 * a session. This is the local cache only, which is the half that never did.
 */
@Composable
fun SavedFlightsScreen(onBack: () -> Unit) {
    val context = LocalContext.current

    // Re-listed on every entry: a download that happened while connected must
    // show up here without an app restart.
    // #1080: a delete has to re-list, so this is state rather than a one-shot
    // remember.
    var flights by remember { mutableStateOf(FlightCache.listSavedFlights(context)) }
    var flightToDelete by remember { mutableStateOf<FlightCache.SavedFlight?>(null) }

    var chartCsv by remember { mutableStateOf<File?>(null) }
    chartCsv?.let { csv ->
        FlightChartScreen(csvFile = csv, onBack = { chartCsv = null })
        return
    }

    Column(Modifier.fillMaxSize().padding(12.dp)) {
        Row(
            Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            TextButton(onClick = onBack) { Text("← Back") }
            Text("Saved Flights", style = MaterialTheme.typography.titleMedium)
        }

        if (flights.isEmpty()) {
            Text(
                "No flights downloaded yet.\n\nConnect to a rocket, open Files, and download a " +
                    "flight log — it will appear here afterwards, with or without a connection.",
                style = MaterialTheme.typography.bodyMedium,
                modifier = Modifier.padding(top = 16.dp),
            )
            return@Column
        }

        Text(
            "${flights.size} flight${if (flights.size == 1) "" else "s"} on this phone",
            style = MaterialTheme.typography.bodySmall,
            modifier = Modifier.padding(bottom = 8.dp),
        )

        LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            items(flights, key = { it.name }) { flight ->
                SavedFlightRow(
                    flight = flight,
                    onChart = { flight.csv?.let { chartCsv = it } },
                    onShare = { FlightCache.shareFlight(context, listOf(flight.bin, flight.csv, FlightCache.summaryFileFor(context, flight.name))) },
                    onDelete = { flightToDelete = flight },
                )
            }
        }
    }

    // #1080: confirm first — this is the only copy once the board's has been
    // deleted, and the wording says WHERE it is being deleted from (the Files
    // screen's Delete is the device one).
    flightToDelete?.let { doomed ->
        AlertDialog(
            onDismissRequest = { flightToDelete = null },
            title = { Text("Delete from this phone?") },
            text = {
                Text(
                    "\"${doomed.displayName}\" and its CSV will be removed from this " +
                        "phone. The copy on the rocket is not affected — delete that from " +
                        "the Files screen while connected.",
                )
            },
            confirmButton = {
                TextButton(onClick = {
                    FlightCache.deleteSavedFlight(context, doomed)
                    flights = FlightCache.listSavedFlights(context)
                    flightToDelete = null
                }) { Text("Delete") }
            },
            dismissButton = {
                TextButton(onClick = { flightToDelete = null }) { Text("Cancel") }
            },
        )
    }
}

@Composable
private fun SavedFlightRow(
    flight: FlightCache.SavedFlight,
    onChart: () -> Unit,
    onShare: () -> Unit,
    onDelete: () -> Unit,
) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp)) {
            Text(flight.displayName, style = MaterialTheme.typography.bodyLarge)
            Text(
                "${formatSize(flight.sizeBytes)} · ${formatDate(flight.lastModified)}" +
                    if (flight.hasCsv) "" else " · .bin only (not converted)",
                style = MaterialTheme.typography.bodySmall,
            )
            // #1091 item 4: the same four numbers iOS's Flight Summary shows.
            flight.summary?.let { s ->
                val parts = listOfNotNull(
                    s.maxAltitudeM?.let { String.format(Locale.US, "Max alt %.0f m", it) },
                    s.maxSpeedMps?.let { String.format(Locale.US, "Max speed %.0f m/s", it) },
                    s.burnoutTimeS?.let { String.format(Locale.US, "Burnout %.1f s", it) },
                    s.apogeeTimeS?.let { String.format(Locale.US, "Apogee %.1f s", it) },
                )
                if (parts.isNotEmpty()) {
                    Text(parts.joinToString(" · "), style = MaterialTheme.typography.bodySmall)
                }
            }
            Row(
                Modifier.fillMaxWidth().padding(top = 8.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                // Both need the CSV: the chart parses it and the share sends it.
                // A .bin whose conversion failed is still listed, so the row has
                // to say why its buttons are dead rather than just disabling them.
                OutlinedButton(onClick = onChart, enabled = flight.hasCsv) { Text("Chart") }
                OutlinedButton(onClick = onShare, enabled = flight.hasAnyFile) { Text("Share") }   // #1067: bin-only flights export too
                // #1080: the third verb — see FlightCache.deleteSavedFlight.
                OutlinedButton(onClick = onDelete) { Text("Delete") }
            }
        }
    }
}

// #1091 item 1: one unit base on both phones — iOS's binary KB/MB
// (FlightLogsView.formatFileSize), not 1000-based kB.
private fun formatSize(bytes: Long): String = when {
    bytes >= 1024L * 1024L -> String.format(Locale.US, "%.1f MB", bytes / (1024.0 * 1024.0))
    bytes >= 1024L -> String.format(Locale.US, "%.1f KB", bytes / 1024.0)
    else -> "$bytes B"
}

private fun formatDate(millis: Long): String =
    if (millis <= 0) "—"
    else SimpleDateFormat("MMM d, HH:mm", Locale.US).format(Date(millis))
