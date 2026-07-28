package com.tinkerbug.tinkerrocket.app

import android.text.format.DateUtils
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableDoubleStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.tinkerbug.tinkerrocket.maps.ESTIMATED_TILE_BYTES
import com.tinkerbug.tinkerrocket.maps.OfflineRegion
import com.tinkerbug.tinkerrocket.maps.RegionSpec
import com.tinkerbug.tinkerrocket.maps.TileDownloader
import com.tinkerbug.tinkerrocket.maps.TileMath
import com.tinkerbug.tinkerrocket.maps.TileSource
import com.tinkerbug.tinkerrocket.session.DeviceSession
import kotlinx.coroutines.launch
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.style.layers.FillLayer
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.PropertyFactory.fillColor
import org.maplibre.android.style.layers.PropertyFactory.fillOpacity
import org.maplibre.android.style.layers.PropertyFactory.lineColor
import org.maplibre.android.style.layers.PropertyFactory.lineWidth
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.geojson.Feature
import org.maplibre.geojson.Point
import org.maplibre.geojson.Polygon
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/**
 * Map tab router: rocket map → Offline Maps manager → Save Area sheet
 * (iOS: OfflineMapsView + SaveAreaView, shared entry from the Rocket Map).
 */
@Composable
fun MapTab(container: AppContainer, session: DeviceSession?) {
    var route by remember { mutableStateOf("map") }

    // Landing predictor follows the map's lifecycle (iOS: attach onAppear,
    // detach onDisappear) — telemetry-rate work only runs while it's shown.
    val predictor = remember { com.tinkerbug.tinkerrocket.session.LandingPredictor(container.fleetScope) }
    DisposableEffect(session) {
        session?.let { predictor.attach(it, container.profileStore) }
        onDispose { predictor.detach() }
    }

    when (route) {
        "map" -> Box(Modifier.fillMaxSize()) {
            MapScreen(proxy = container.tileProxy, session = session, predictor = predictor)
            TextButton(
                onClick = { route = "offline" },
                modifier = Modifier.align(Alignment.BottomEnd).padding(8.dp),
            ) { Text("Offline maps") }
        }
        "offline" -> OfflineMapsScreen(
            container = container,
            onBack = { route = "map" },
            onAdd = { route = "save" },
        )
        else -> SaveAreaScreen(
            container = container,
            initialCenter = session?.lastValidRocketFix?.value
                ?.let { LatLng(it.latitude, it.longitude) }
                ?: LatLng(39.8283, -98.5795), // continental-US fallback
            onDone = { route = "offline" },
        )
    }
}

// ── Offline Maps manager ────────────────────────────────────────────────

@Composable
fun OfflineMapsScreen(container: AppContainer, onBack: () -> Unit, onAdd: () -> Unit) {
    val regions by container.regionStore.regions.collectAsState()

    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                TextButton(onClick = onBack) { Text("← Map") }
                Text("Offline Maps", style = MaterialTheme.typography.titleLarge)
            }
            OutlinedButton(onClick = onAdd) { Text("＋ Save area") }
        }

        if (regions.isEmpty()) {
            Text(
                "No offline areas saved yet. Save imagery for a launch site " +
                    "while you have signal.",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        regions.forEach { r ->
            Card {
                Row(
                    Modifier.fillMaxWidth().padding(12.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Column(Modifier.weight(1f)) {
                        Text(r.name, style = MaterialTheme.typography.bodyLarge)
                        Text(
                            "${"%.1f".format(r.radiusMeters / 1000)} km · z${r.minZoom}–${r.maxZoom} · " +
                                "${formatMb(r.bytes)} · ${r.tileSource?.displayName ?: r.source}",
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                        Text(
                            "Saved ${DateUtils.getRelativeTimeSpanString(r.savedAtMs)}",
                            style = MaterialTheme.typography.labelSmall,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                        )
                    }
                    TextButton(onClick = {
                        container.fleetScope.launch { container.regionStore.delete(r) }
                    }) { Text("Delete", color = MaterialTheme.colorScheme.error) }
                }
            }
        }
        if (regions.isNotEmpty()) {
            Text(
                "Total offline storage · ${formatMb(container.regionStore.totalBytes)} " +
                    "across ${regions.size} region${if (regions.size == 1) "" else "s"}",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

private fun formatMb(bytes: Long): String {
    val mb = bytes / 1_048_576.0
    return if (mb < 0.1) "0 MB" else "%.0f MB".format(mb)
}

// ── Save Area ───────────────────────────────────────────────────────────

@Composable
fun SaveAreaScreen(container: AppContainer, initialCenter: LatLng, onDone: () -> Unit) {
    val context = androidx.compose.ui.platform.LocalContext.current
    remember {
        org.maplibre.android.MapLibre.getInstance(context)
        org.maplibre.android.MapLibre.setConnected(true)
    }

    var source by remember { mutableStateOf(TileSource.USGS_IMAGERY_TOPO) }
    var radiusKm by remember { mutableDoubleStateOf(5.0) }
    var maxZoom by remember { mutableDoubleStateOf(16.0) }
    var name by remember { mutableStateOf("") }
    var centerLat by remember { mutableDoubleStateOf(initialCenter.latitude) }
    var centerLon by remember { mutableDoubleStateOf(initialCenter.longitude) }

    val downloader = remember { TileDownloader(container.tileCache) }
    val phase by downloader.phase.collectAsState()
    val done by downloader.done.collectAsState()
    val total by downloader.total.collectAsState()
    val bytes by downloader.bytes.collectAsState()

    val spec = RegionSpec(
        centerLat = centerLat, centerLon = centerLon,
        radiusMeters = radiusKm * 1000, minZoom = 10, maxZoom = maxZoom.toInt(),
    )
    val tileCount = TileMath.tileCount(spec)
    val estMb = tileCount * ESTIMATED_TILE_BYTES / 1_048_576.0
    val tooBig = estMb > 200
    val defaultName = "Area %.3f, %.3f".format(centerLat, centerLon)

    // Save the region once the download finishes (iOS onChange(.finished)).
    LaunchedEffect(phase) {
        if (phase == TileDownloader.Phase.FINISHED) {
            container.fleetScope.launch {
                container.regionStore.add(
                    OfflineRegion(
                        name = name.ifBlank { defaultName },
                        lat = centerLat, lon = centerLon,
                        radiusMeters = radiusKm * 1000,
                        minZoom = 10, maxZoom = maxZoom.toInt(),
                        source = source.key, tileCount = total, bytes = bytes,
                        savedAtMs = System.currentTimeMillis(),
                    ),
                )
            }
            onDone()
        }
    }

    var mapRef by remember { mutableStateOf<MapLibreMap?>(null) }
    val mapView = remember {
        MapView(context).apply {
            onCreate(null)
            getMapAsync { map ->
                map.moveCamera(CameraUpdateFactory.newLatLngZoom(initialCenter, 12.0))
                map.addOnCameraIdleListener {
                    centerLat = map.cameraPosition.target?.latitude ?: centerLat
                    centerLon = map.cameraPosition.target?.longitude ?: centerLon
                }
                mapRef = map
            }
        }
    }
    DisposableEffect(Unit) {
        mapView.onStart(); mapView.onResume()
        onDispose { mapView.onPause(); mapView.onStop(); mapView.onDestroy() }
    }

    // Style (re)install on source change — circle layers re-added each time.
    LaunchedEffect(mapRef, source) {
        val map = mapRef ?: return@LaunchedEffect
        map.setStyle(Style.Builder().fromJson(saveAreaStyleJson(container, source))) { style ->
            val src = GeoJsonSource(CIRCLE_SOURCE)
            src.setGeoJson(circleFeature(centerLat, centerLon, radiusKm * 1000))
            style.addSource(src)
            style.addLayer(
                FillLayer(CIRCLE_FILL, CIRCLE_SOURCE)
                    .withProperties(fillColor("#1E88E5"), fillOpacity(0.15f)),
            )
            style.addLayer(
                LineLayer(CIRCLE_LINE, CIRCLE_SOURCE)
                    .withProperties(lineColor("#1E88E5"), lineWidth(2f)),
            )
        }
    }
    // Circle tracks center/radius (diff-based source update, no reload).
    LaunchedEffect(mapRef, centerLat, centerLon, radiusKm) {
        mapRef?.style?.getSourceAs<GeoJsonSource>(CIRCLE_SOURCE)
            ?.setGeoJson(circleFeature(centerLat, centerLon, radiusKm * 1000))
    }

    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState())) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 8.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            TextButton(onClick = onDone, enabled = phase != TileDownloader.Phase.DOWNLOADING) {
                Text("Cancel")
            }
            Text("Save area offline", style = MaterialTheme.typography.titleMedium)
            OutlinedButton(onClick = {
                source = when (source) {
                    TileSource.USGS_IMAGERY_TOPO -> TileSource.USGS_TOPO
                    TileSource.USGS_TOPO -> TileSource.USGS_IMAGERY
                    TileSource.USGS_IMAGERY -> TileSource.USGS_IMAGERY_TOPO
                }
            }) { Text(source.displayName, style = MaterialTheme.typography.labelSmall) }
        }

        Box(Modifier.fillMaxWidth().height(280.dp)) {
            AndroidView(factory = { mapView }, modifier = Modifier.fillMaxSize())
            // Center crosshair = the region center used for the download.
            Text(
                "⌖",
                style = MaterialTheme.typography.headlineLarge,
                color = Color(0xFF1E88E5),
                modifier = Modifier.align(Alignment.Center),
            )
        }

        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            SliderRow("Radius", "%.1f km".format(radiusKm), radiusKm, 1.0..20.0) { radiusKm = (it * 2).toInt() / 2.0 }
            SliderRow("Detail (max zoom)", "z${maxZoom.toInt()}", maxZoom, 13.0..18.0) { maxZoom = it.toInt().toDouble() }

            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Text("Estimate", color = MaterialTheme.colorScheme.onSurfaceVariant)
                Text(
                    "~$tileCount tiles · ~%.0f MB".format(estMb),
                    fontFamily = FontFamily.Monospace,
                    color = if (tooBig) Color(0xFFFFA000) else MaterialTheme.colorScheme.onSurface,
                )
            }
            if (tooBig) {
                Text(
                    "Large download — consider a smaller radius or lower max zoom.",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFFFFA000),
                )
            }

            OutlinedTextField(
                value = name,
                onValueChange = { name = it },
                label = { Text(defaultName) },
                singleLine = true,
                modifier = Modifier.fillMaxWidth(),
            )

            if (phase == TileDownloader.Phase.DOWNLOADING) {
                LinearProgressIndicator(
                    progress = { if (total > 0) done.toFloat() / total else 0f },
                    modifier = Modifier.fillMaxWidth(),
                )
                Text(
                    "$done / $total tiles · %.0f MB".format(bytes / 1_048_576.0),
                    style = MaterialTheme.typography.bodySmall,
                    fontFamily = FontFamily.Monospace,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                OutlinedButton(
                    onClick = { downloader.cancel() },
                    modifier = Modifier.fillMaxWidth(),
                ) { Text("Cancel download") }
            } else {
                Button(
                    onClick = { downloader.start(spec, source.key) },
                    modifier = Modifier.fillMaxWidth(),
                ) { Text("Download $tileCount tiles") }
            }
        }
    }
}

@Composable
private fun SliderRow(
    title: String,
    valueLabel: String,
    value: Double,
    range: ClosedFloatingPointRange<Double>,
    onChange: (Double) -> Unit,
) {
    Column {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(title, color = MaterialTheme.colorScheme.onSurfaceVariant)
            Text(valueLabel, fontFamily = FontFamily.Monospace)
        }
        Slider(
            value = value.toFloat(),
            onValueChange = { onChange(it.toDouble()) },
            valueRange = range.start.toFloat()..range.endInclusive.toFloat(),
        )
    }
}

private const val CIRCLE_SOURCE = "region-circle-src"
private const val CIRCLE_FILL = "region-circle-fill"
private const val CIRCLE_LINE = "region-circle-line"

private fun saveAreaStyleJson(container: AppContainer, source: TileSource): String = """
{
  "version": 8,
  "sources": {
    "usgs": {
      "type": "raster",
      "tiles": ["${container.tileProxy.templateFor(source)}"],
      "tileSize": 256,
      "maxzoom": ${source.maxZoom}
    }
  },
  "layers": [
    {"id": "bg", "type": "background", "paint": {"background-color": "#ededed"}},
    {"id": "usgs", "type": "raster", "source": "usgs"}
  ]
}
"""

/** Geodesic-enough circle for km radii (same small-angle math as RegionSpec). */
private fun circleFeature(lat: Double, lon: Double, radiusM: Double, points: Int = 64): Feature {
    val dLat = radiusM / 111_320.0
    val dLon = radiusM / (111_320.0 * max(0.01, cos(Math.toRadians(lat))))
    val ring = (0..points).map { i ->
        val a = 2.0 * Math.PI * i / points
        Point.fromLngLat(lon + dLon * sin(a), lat + dLat * cos(a))
    }
    return Feature.fromGeometry(Polygon.fromLngLats(listOf(ring)))
}
