package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Checkbox
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.TextMeasurer
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.tinkerbug.tinkerrocket.protocol.ChartWindow
import com.tinkerbug.tinkerrocket.protocol.ChartWindow.Domain
import com.tinkerbug.tinkerrocket.protocol.CsvParser
import com.tinkerbug.tinkerrocket.protocol.FlightCsvData
import com.tinkerbug.tinkerrocket.protocol.LttbPoint
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import java.io.File

/**
 * Post-flight data chart — port of iOS FlightChartView on a custom Compose
 * Canvas (plan §1: no chart library — the LTTB/zoom windowing is app-owned
 * and a library's zoom model fights it).  All window math lives in
 * ChartWindow (:core:protocol), golden-pinned; this file is drawing +
 * gestures only.  Pinch zooms about the window center, drag pans, double
 * tap resets; re-decimation runs debounced after the window settles.
 */
@Composable
fun FlightChartScreen(csvFile: File, onBack: () -> Unit) {
    var csvData by remember { mutableStateOf<FlightCsvData?>(null) }
    var error by remember { mutableStateOf<String?>(null) }
    var selected by remember { mutableStateOf<Set<String>>(emptySet()) }
    var showPicker by remember { mutableStateOf(false) }

    // Full-resolution clean series (name → x/y), kept for re-decimation.
    var fullSeries by remember { mutableStateOf<List<FullCol>>(emptyList()) }
    var chartSeries by remember { mutableStateOf<List<Pair<String, List<LttbPoint>>>>(emptyList()) }
    var fullX by remember { mutableStateOf(Domain(0.0, 1.0)) }
    var visibleX by remember { mutableStateOf(Domain(0.0, 1.0)) }

    LaunchedEffect(csvFile) {
        try {
            val parsed = withContext(Dispatchers.Default) { CsvParser.parse(csvFile.readText()) }
            csvData = parsed
            selected = when {
                parsed.columns.containsKey("Pressure Altitude (m)") -> setOf("Pressure Altitude (m)")
                parsed.columns.containsKey("pressure_alt") -> setOf("pressure_alt")
                else -> emptySet()
            }
        } catch (e: Exception) {
            error = e.message ?: "CSV parse failed"
        }
    }

    // Rebuild full + decimated series when the selection changes.
    LaunchedEffect(csvData, selected) {
        val data = csvData ?: return@LaunchedEffect
        val timeMs = data.columns["Time (ms)"] ?: data.columns["time_ms"] ?: return@LaunchedEffect
        val timeS = timeMs.map { it / 1000.0 }
        val cols = selected.sorted().mapNotNull { name ->
            val values = data.columns[name] ?: return@mapNotNull null
            val x = mutableListOf<Double>()
            val y = mutableListOf<Double>()
            for (i in 0 until minOf(timeS.size, values.size)) {
                if (timeS[i].isFinite() && values[i].isFinite()) {
                    x += timeS[i]
                    y += values[i]
                }
            }
            if (x.isEmpty()) null else FullCol(name, x, y)
        }
        fullSeries = cols
        val allMin = cols.mapNotNull { it.x.firstOrNull() }.minOrNull()
        val allMax = cols.mapNotNull { it.x.lastOrNull() }.maxOrNull()
        fullX = if (allMin != null && allMax != null && allMin < allMax) {
            ChartWindow.safeDomain(allMin, allMax)
        } else {
            Domain(0.0, 1.0)
        }
        visibleX = fullX
        chartSeries = cols.map { it.name to ChartWindow.redecimate(it.x, it.y, fullX, MAX_POINTS) }
    }

    // Progressive-LOD re-decimation, debounced until the window settles.
    LaunchedEffect(visibleX) {
        delay(200)
        if (fullSeries.isNotEmpty()) {
            chartSeries = fullSeries.map {
                it.name to ChartWindow.redecimate(it.x, it.y, visibleX, MAX_POINTS)
            }
        }
    }

    Column(Modifier.fillMaxSize()) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 8.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            TextButton(onClick = onBack) { Text("← Files") }
            Text(csvFile.nameWithoutExtension, style = MaterialTheme.typography.titleSmall)
            TextButton(onClick = { showPicker = true }) { Text("Columns") }
        }

        when {
            error != null -> Text(
                error!!,
                color = MaterialTheme.colorScheme.error,
                modifier = Modifier.padding(16.dp),
            )
            csvData == null -> Row(
                Modifier.padding(16.dp),
                horizontalArrangement = Arrangement.spacedBy(10.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                CircularProgressIndicator(Modifier.size(22.dp))
                Text("Parsing flight data…")
            }
            chartSeries.isEmpty() -> Column(
                Modifier.fillMaxWidth().padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Text("Select a column to plot", style = MaterialTheme.typography.titleMedium)
                TextButton(onClick = { showPicker = true }) { Text("Choose Columns") }
            }
            else -> {
                // Zoom indicator (only when zoomed).
                if (ChartWindow.isZoomed(fullX, visibleX)) {
                    Row(
                        Modifier.fillMaxWidth().padding(horizontal = 12.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            "🔍 %.1fs – %.1fs".format(visibleX.lower, visibleX.upper),
                            style = MaterialTheme.typography.labelSmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                        TextButton(onClick = { visibleX = fullX }) { Text("Reset") }
                    }
                }
                // Legend
                Row(
                    Modifier.fillMaxWidth().horizontalScroll(rememberScrollState())
                        .padding(horizontal = 12.dp),
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    chartSeries.forEachIndexed { i, (name, _) ->
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Box(Modifier.size(10.dp).background(SERIES_COLORS[i % SERIES_COLORS.size], CircleShape))
                            Text(
                                " $name",
                                style = MaterialTheme.typography.labelSmall,
                            )
                        }
                    }
                }

                val textMeasurer = rememberTextMeasurer()
                val labelColor = MaterialTheme.colorScheme.onSurfaceVariant
                val gridColor = MaterialTheme.colorScheme.onSurfaceVariant.copy(alpha = 0.2f)
                val density = LocalDensity.current

                Canvas(
                    Modifier
                        .fillMaxSize()
                        .padding(start = 8.dp, end = 8.dp, bottom = 8.dp)
                        .pointerInputTransform(
                            onGesture = { pan, zoom, widthPx ->
                                var window = visibleX
                                if (zoom != 1.0f) {
                                    window = ChartWindow.zoomAround(window, fullX, zoom.toDouble())
                                }
                                if (pan != 0f && ChartWindow.isZoomed(fullX, window)) {
                                    val effectiveWidth = maxOf(widthPx - with(density) { 48.dp.toPx() }, 1f)
                                    val dataPerPx = window.span / effectiveWidth
                                    window = ChartWindow.pan(window, fullX, -pan * dataPerPx)
                                }
                                visibleX = window
                            },
                            onDoubleTap = { visibleX = fullX },
                        ),
                ) {
                    val yDomain = ChartWindow.visibleYRange(chartSeries.map { it.second }, visibleX)
                    drawChart(
                        series = chartSeries,
                        xDomain = visibleX,
                        yDomain = yDomain,
                        textMeasurer = textMeasurer,
                        labelColor = labelColor,
                        gridColor = gridColor,
                    )
                }
            }
        }
    }

    if (showPicker && csvData != null) {
        ColumnPickerDialog(
            data = csvData!!,
            selected = selected,
            onDismiss = { showPicker = false },
            onChange = { selected = it },
        )
    }
}

private const val MAX_POINTS = 2000

private val SERIES_COLORS = listOf(
    Color(0xFF1E88E5), Color(0xFFD32F2F), Color(0xFF43A047),
    Color(0xFFFB8C00), Color(0xFF8E24AA), Color(0xFF00ACC1),
)

private class FullCol(val name: String, val x: List<Double>, val y: List<Double>)

/** Transform (pinch zoom + drag pan) and double-tap-reset gestures. */
private fun Modifier.pointerInputTransform(
    onGesture: (panX: Float, zoom: Float, widthPx: Float) -> Unit,
    onDoubleTap: () -> Unit,
): Modifier = this
    .pointerInput(Unit) {
        detectTapGestures(onDoubleTap = { onDoubleTap() })
    }
    .pointerInput(Unit) {
        detectTransformGestures { _, pan, zoom, _ ->
            onGesture(pan.x, zoom, size.width.toFloat())
        }
    }

// ── Drawing ─────────────────────────────────────────────────────────────

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawChart(
    series: List<Pair<String, List<LttbPoint>>>,
    xDomain: Domain,
    yDomain: Domain,
    textMeasurer: TextMeasurer,
    labelColor: Color,
    gridColor: Color,
) {
    val leftPad = 48.dp.toPx()
    val bottomPad = 24.dp.toPx()
    val plotW = size.width - leftPad
    val plotH = size.height - bottomPad
    if (plotW <= 0 || plotH <= 0) return

    fun px(x: Double): Float = leftPad + ((x - xDomain.lower) / xDomain.span * plotW).toFloat()
    fun py(y: Double): Float = (plotH - (y - yDomain.lower) / yDomain.span * plotH).toFloat()

    val labelStyle = TextStyle(fontSize = 9.sp, color = labelColor)

    // Grid + labels: 5 x-ticks, 5 y-ticks.
    for (i in 0..4) {
        val xVal = xDomain.lower + xDomain.span * i / 4
        val xPx = px(xVal)
        drawLine(gridColor, Offset(xPx, 0f), Offset(xPx, plotH), strokeWidth = 1f)
        val label = textMeasurer.measure("%.1f".format(xVal), labelStyle)
        drawText(
            label,
            topLeft = Offset(
                (xPx - label.size.width / 2).coerceIn(0f, size.width - label.size.width),
                plotH + 4.dp.toPx(),
            ),
        )
        val yVal = yDomain.lower + yDomain.span * i / 4
        val yPx = py(yVal)
        drawLine(gridColor, Offset(leftPad, yPx), Offset(size.width, yPx), strokeWidth = 1f)
        val yLabel = textMeasurer.measure(compactNumber(yVal), labelStyle)
        drawText(
            yLabel,
            topLeft = Offset(
                leftPad - yLabel.size.width - 4.dp.toPx(),
                (yPx - yLabel.size.height / 2).coerceIn(0f, size.height - yLabel.size.height),
            ),
        )
    }

    series.forEachIndexed { idx, (_, points) ->
        if (points.isEmpty()) return@forEachIndexed
        val color = SERIES_COLORS[idx % SERIES_COLORS.size]
        val path = Path()
        var started = false
        for (p in points) {
            val xPx = px(p.x)
            val yPx = py(p.y)
            if (!started) {
                path.moveTo(xPx, yPx)
                started = true
            } else {
                path.lineTo(xPx, yPx)
            }
        }
        drawPath(path, color, style = Stroke(width = 2.dp.toPx()))
        // Point markers only when zoomed in enough that samples matter.
        if (points.size <= 500) {
            for (p in points) {
                drawCircle(color, radius = 2.5.dp.toPx() / 2, center = Offset(px(p.x), py(p.y)))
            }
        }
    }
}

private fun compactNumber(v: Double): String = when {
    kotlin.math.abs(v) >= 10_000 -> "%.0f".format(v)
    kotlin.math.abs(v) >= 100 -> "%.1f".format(v)
    else -> "%.2f".format(v)
}

// ── Column picker ───────────────────────────────────────────────────────

@Composable
private fun ColumnPickerDialog(
    data: FlightCsvData,
    selected: Set<String>,
    onDismiss: () -> Unit,
    onChange: (Set<String>) -> Unit,
) {
    val groups = FlightCsvData.columnGroupsFor(data.headers)
    val plottable = data.headers.filter { it != "Time (ms)" && it != "time_ms" }.toSet()
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Columns") },
        text = {
            Column(
                Modifier.fillMaxWidth()
                    .heightIn(max = 420.dp)
                    .verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(2.dp),
            ) {
                groups.forEach { group ->
                    val inFile = group.columns.filter { it in plottable }
                    if (inFile.isEmpty()) return@forEach
                    Text(group.name, style = MaterialTheme.typography.titleSmall)
                    inFile.forEach { col ->
                        // Whole row toggles — a checkbox-only hit target is
                        // a thumb-miss on a phone.
                        Row(
                            Modifier.fillMaxWidth().clickable {
                                onChange(if (col in selected) selected - col else selected + col)
                            },
                            verticalAlignment = Alignment.CenterVertically,
                        ) {
                            Checkbox(
                                checked = col in selected,
                                onCheckedChange = { on ->
                                    onChange(if (on) selected + col else selected - col)
                                },
                            )
                            Text(col, style = MaterialTheme.typography.bodySmall)
                        }
                    }
                }
            }
        },
        confirmButton = { TextButton(onClick = onDismiss) { Text("Done") } },
    )
}
