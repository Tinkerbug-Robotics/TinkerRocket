package com.tinkerbug.tinkerrocket.app

import android.content.Context
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
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.tinkerbug.tinkerrocket.maps.TileSource
import com.tinkerbug.tinkerrocket.session.GuidanceResult
import com.tinkerbug.tinkerrocket.session.computeGuidancePoint
import com.tinkerbug.tinkerrocket.session.fetchOpenMeteoWinds
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.style.layers.CircleLayer
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.PropertyFactory.circleColor
import org.maplibre.android.style.layers.PropertyFactory.circleRadius
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeColor
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeWidth
import org.maplibre.android.style.layers.PropertyFactory.lineColor
import org.maplibre.android.style.layers.PropertyFactory.lineDasharray
import org.maplibre.android.style.layers.PropertyFactory.lineWidth
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.geojson.Feature
import org.maplibre.geojson.LineString
import org.maplibre.geojson.Point

/**
 * Reverse Drift Cast planner (iOS DriftCastView, 2D scope — the 3D
 * trajectory view trails to Phase 9): pick launch + landing points on the
 * map, enter apogee/rates, fetch winds, and compute the apogee guidance
 * point with feasibility + forward verification.  The send flow (cmd 28)
 * also trails — guided flight is gated behind FIRST-GUIDED-FLIGHT anyway.
 *
 * Inputs persist across sessions (iOS @AppStorage); valid descent-field
 * edits write back into the active profile (#191), with a same-value
 * guard against the seed echo.
 */
@Composable
fun DriftCastScreen(container: AppContainer, onBack: () -> Unit) {
    val context = LocalContext.current
    val prefs = remember { context.getSharedPreferences("driftcast", Context.MODE_PRIVATE) }
    remember {
        org.maplibre.android.MapLibre.getInstance(context)
        org.maplibre.android.MapLibre.setConnected(true)
    }

    // Phone GPS backs Use GPS + the first-open auto-seed.
    DisposableEffect(Unit) {
        container.phoneLocation.start()
        onDispose { container.phoneLocation.stop() }
    }
    val phoneFix by container.phoneLocation.location.collectAsState()

    fun pref(key: String, def: String) = prefs.getString(key, def) ?: def
    var launchLat by remember { mutableStateOf(pref("launchLat", "")) }
    var launchLon by remember { mutableStateOf(pref("launchLon", "")) }
    var landingLat by remember { mutableStateOf(pref("landingLat", "")) }
    var landingLon by remember { mutableStateOf(pref("landingLon", "")) }
    var apogeeFt by remember { mutableStateOf(pref("apogeeFt", "10000")) }
    var drogueRate by remember { mutableStateOf(pref("drogueRate", "60")) }
    var mainRate by remember { mutableStateOf(pref("mainRate", "18")) }
    var mainDeploy by remember { mutableStateOf(pref("mainDeploy", "700")) }
    var maxSteering by remember { mutableStateOf(pref("maxSteering", "25")) }
    var tapModeLaunch by remember { mutableStateOf(true) }
    var didAutoSeed by remember { mutableStateOf(false) }

    var result by remember { mutableStateOf<GuidanceResult?>(null) }
    var computing by remember { mutableStateOf(false) }
    var errorMessage by remember { mutableStateOf<String?>(null) }
    val scope = rememberCoroutineScope()

    fun persist() {
        prefs.edit()
            .putString("launchLat", launchLat).putString("launchLon", launchLon)
            .putString("landingLat", landingLat).putString("landingLon", landingLon)
            .putString("apogeeFt", apogeeFt)
            .putString("drogueRate", drogueRate).putString("mainRate", mainRate)
            .putString("mainDeploy", mainDeploy).putString("maxSteering", maxSteering)
            .apply()
    }

    // Seed the descent fields from the active profile once (iOS
    // seedFromActiveProfile); the write-back guard below swallows the echo.
    LaunchedEffect(Unit) {
        container.profileStore.activeProfile?.let { p ->
            drogueRate = fmtD(p.drogueRateFps)
            mainRate = fmtD(p.mainRateFps)
            mainDeploy = fmtD(p.mainDeployAltAglFt)
        }
    }

    // #191: valid descent-field edits flow back into the active profile.
    fun writeBack(value: String, get: (Double) -> Boolean, set: (Double) -> Unit) {
        val v = value.toDoubleOrNull() ?: return
        if (v <= 0) return
        if (get(v)) return // same-value guard
        set(v)
    }
    LaunchedEffect(drogueRate) {
        writeBack(drogueRate, { container.profileStore.activeProfile?.drogueRateFps == it }) { v ->
            container.profileStore.activeProfile?.let { p ->
                container.fleetScope.launch { container.profileStore.update(p.id) { it.copy(drogueRateFps = v) } }
            }
        }
    }
    LaunchedEffect(mainRate) {
        writeBack(mainRate, { container.profileStore.activeProfile?.mainRateFps == it }) { v ->
            container.profileStore.activeProfile?.let { p ->
                container.fleetScope.launch { container.profileStore.update(p.id) { it.copy(mainRateFps = v) } }
            }
        }
    }
    LaunchedEffect(mainDeploy) {
        writeBack(mainDeploy, { container.profileStore.activeProfile?.mainDeployAltAglFt == it }) { v ->
            container.profileStore.activeProfile?.let { p ->
                container.fleetScope.launch { container.profileStore.update(p.id) { it.copy(mainDeployAltAglFt = v) } }
            }
        }
    }

    // First GPS fix seeds blank points — never clobbers typed/restored values.
    LaunchedEffect(phoneFix) {
        val f = phoneFix ?: return@LaunchedEffect
        if (didAutoSeed) return@LaunchedEffect
        didAutoSeed = true
        if (launchLat.isBlank()) { launchLat = "%.5f".format(f.lat); launchLon = "%.5f".format(f.lon) }
        if (landingLat.isBlank()) { landingLat = "%.5f".format(f.lat); landingLon = "%.5f".format(f.lon) }
        persist()
    }

    // ── Map ──────────────────────────────────────────────────────────────
    var mapRef by remember { mutableStateOf<MapLibreMap?>(null) }
    val mapView = remember {
        MapView(context).apply {
            onCreate(null)
            getMapAsync { map ->
                val lat = launchLat.toDoubleOrNull()
                val lon = launchLon.toDoubleOrNull()
                if (lat != null && lon != null) {
                    map.moveCamera(CameraUpdateFactory.newLatLngZoom(LatLng(lat, lon), 12.0))
                }
                mapRef = map
            }
        }
    }
    DisposableEffect(Unit) {
        mapView.onStart(); mapView.onResume()
        onDispose { mapView.onPause(); mapView.onStop(); mapView.onDestroy() }
    }

    fun refreshOverlays() {
        val style = mapRef?.style ?: return
        fun pt(latS: String, lonS: String): Point? {
            val la = latS.toDoubleOrNull() ?: return null
            val lo = lonS.toDoubleOrNull() ?: return null
            return Point.fromLngLat(lo, la)
        }
        pt(launchLat, launchLon)?.let {
            style.getSourceAs<GeoJsonSource>(DC_LAUNCH_SRC)?.setGeoJson(Feature.fromGeometry(it))
        }
        pt(landingLat, landingLon)?.let {
            style.getSourceAs<GeoJsonSource>(DC_LANDING_SRC)?.setGeoJson(Feature.fromGeometry(it))
        }
        val r = result
        if (r != null) {
            style.getSourceAs<GeoJsonSource>(DC_GUIDANCE_SRC)?.setGeoJson(
                Feature.fromGeometry(Point.fromLngLat(r.guidanceLon, r.guidanceLat)),
            )
            style.getSourceAs<GeoJsonSource>(DC_TRACK_SRC)?.setGeoJson(
                Feature.fromGeometry(
                    LineString.fromLngLats(r.descentTrack.map { Point.fromLngLat(it.lon, it.lat) }),
                ),
            )
            pt(launchLat, launchLon)?.let { l ->
                style.getSourceAs<GeoJsonSource>(DC_BOOST_SRC)?.setGeoJson(
                    Feature.fromGeometry(
                        LineString.fromLngLats(
                            listOf(l, Point.fromLngLat(r.guidanceLon, r.guidanceLat)),
                        ),
                    ),
                )
            }
        }
    }

    LaunchedEffect(mapRef) {
        val map = mapRef ?: return@LaunchedEffect
        map.setStyle(
            Style.Builder().fromJson(
                driftCastStyleJson(container, TileSource.USGS_IMAGERY_TOPO),
            ),
        ) { style ->
            installDriftCastLayers(style)
            refreshOverlays()
        }
        map.addOnMapClickListener { latLng ->
            if (tapModeLaunch) {
                launchLat = "%.5f".format(latLng.latitude)
                launchLon = "%.5f".format(latLng.longitude)
            } else {
                landingLat = "%.5f".format(latLng.latitude)
                landingLon = "%.5f".format(latLng.longitude)
            }
            persist()
            refreshOverlays()
            true
        }
    }
    LaunchedEffect(mapRef, result, launchLat, landingLat, launchLon, landingLon) {
        refreshOverlays()
    }

    val inputsValid = listOf(launchLat, launchLon, landingLat, landingLon)
        .all { it.toDoubleOrNull() != null } &&
        listOf(apogeeFt, drogueRate, mainRate, mainDeploy, maxSteering)
            .all { (it.toDoubleOrNull() ?: 0.0) > 0 }

    // ── UI ───────────────────────────────────────────────────────────────
    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState())) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 8.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            TextButton(onClick = onBack) { Text("← Map") }
            Text("Reverse Drift Cast", style = MaterialTheme.typography.titleMedium)
            TextButton(onClick = {
                phoneFix?.let { f ->
                    if (tapModeLaunch) {
                        launchLat = "%.5f".format(f.lat); launchLon = "%.5f".format(f.lon)
                    } else {
                        landingLat = "%.5f".format(f.lat); landingLon = "%.5f".format(f.lon)
                    }
                    persist()
                }
            }) { Text("Use GPS") }
        }

        Row(
            Modifier.fillMaxWidth().padding(horizontal = 16.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            SegChip("Set Launch", tapModeLaunch) { tapModeLaunch = true }
            SegChip("Set Landing", !tapModeLaunch) { tapModeLaunch = false }
        }

        Box(Modifier.fillMaxWidth().height(300.dp).padding(top = 6.dp)) {
            AndroidView(factory = { mapView }, modifier = Modifier.fillMaxSize())
        }

        Column(Modifier.fillMaxWidth().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DcField("Launch lat", launchLat, Modifier.weight(1f)) { launchLat = it; persist() }
                DcField("Launch lon", launchLon, Modifier.weight(1f)) { launchLon = it; persist() }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DcField("Landing lat", landingLat, Modifier.weight(1f)) { landingLat = it; persist() }
                DcField("Landing lon", landingLon, Modifier.weight(1f)) { landingLon = it; persist() }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DcField("Apogee ft", apogeeFt, Modifier.weight(1f)) { apogeeFt = it; persist() }
                DcField("Max steer °", maxSteering, Modifier.weight(1f)) { maxSteering = it; persist() }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DcField("Drogue ft/s", drogueRate, Modifier.weight(1f)) { drogueRate = it; persist() }
                DcField("Main ft/s", mainRate, Modifier.weight(1f)) { mainRate = it; persist() }
                DcField("Main alt ft", mainDeploy, Modifier.weight(1f)) { mainDeploy = it; persist() }
            }

            if (computing) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    CircularProgressIndicator(Modifier.height(22.dp))
                    Text("Fetching winds + casting…")
                }
            } else {
                Button(
                    enabled = inputsValid,
                    onClick = {
                        computing = true
                        errorMessage = null
                        scope.launch {
                            val r = withContext(Dispatchers.IO) {
                                val wind = fetchOpenMeteoWinds(
                                    landingLat.toDouble(), landingLon.toDouble(),
                                    System.currentTimeMillis(),
                                ) ?: return@withContext null
                                computeGuidancePoint(
                                    launchLat = launchLat.toDouble(),
                                    launchLon = launchLon.toDouble(),
                                    landingLat = landingLat.toDouble(),
                                    landingLon = landingLon.toDouble(),
                                    apogeeAglFt = apogeeFt.toDouble(),
                                    drogueRateFps = drogueRate.toDouble(),
                                    mainRateFps = mainRate.toDouble(),
                                    mainDeployAglFt = mainDeploy.toDouble(),
                                    windProfile = wind,
                                    maxSteeringDeg = maxSteering.toDouble(),
                                )
                            }
                            computing = false
                            if (r == null) {
                                errorMessage = "Wind fetch failed — check connectivity."
                            } else {
                                result = r
                            }
                        }
                    },
                    modifier = Modifier.fillMaxWidth(),
                ) { Text("Calculate guidance point") }
            }
            errorMessage?.let {
                Text(it, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall)
            }

            result?.let { r ->
                Card {
                    Column(Modifier.fillMaxWidth().padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text(
                            if (r.feasible) "FEASIBLE" else "NOT FEASIBLE",
                            style = MaterialTheme.typography.titleMedium,
                            color = if (r.feasible) Color(0xFF43A047) else Color(0xFFFFA000),
                        )
                        r.infeasibleReason?.let {
                            Text(it, style = MaterialTheme.typography.bodySmall, color = Color(0xFFFFA000))
                        }
                        MetricRow("Steering", "%.1f° @ %.0f°".format(r.steeringAngleDeg, r.steeringBearingDeg))
                        MetricRow("Guidance point", "%.5f, %.5f".format(r.guidanceLat, r.guidanceLon))
                        MetricRow("Verification error", "%.0f m".format(r.landingErrorM))
                        MetricRow("Total drift", "%.0f m".format(r.totalDriftM))
                        MetricRow("Descent time", "%.0f s".format(r.totalDescentTimeS))
                        MetricRow("Wind layers", "${r.windProfile.layers.size}")
                    }
                }
                // DriftCast 3D (Phase 9): the planned cast as an orbitable
                // scene — pad, boost line, guidance point, wind-driven
                // descent, verified landing.  Bounded height inside the
                // scrolling column; drags on the canvas orbit instead of
                // scrolling, which is the point of the view.
                Card {
                    Box(Modifier.fillMaxWidth().height(380.dp)) {
                        DriftCast3DCanvas(r)
                    }
                }
            }
        }
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

@Composable
private fun SegChip(label: String, selected: Boolean, onClick: () -> Unit) {
    if (selected) {
        Button(onClick = onClick) { Text(label) }
    } else {
        OutlinedButton(onClick = onClick) { Text(label) }
    }
}

@Composable
private fun DcField(label: String, value: String, modifier: Modifier = Modifier, onChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = onChange,
        label = { Text(label) },
        singleLine = true,
        modifier = modifier,
    )
}

@Composable
private fun MetricRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace)
    }
}

private fun fmtD(d: Double): String =
    if (d == d.toLong().toDouble()) d.toLong().toString() else d.toString()

private const val DC_LAUNCH_SRC = "dc-launch-src"
private const val DC_LANDING_SRC = "dc-landing-src"
private const val DC_GUIDANCE_SRC = "dc-guidance-src"
private const val DC_TRACK_SRC = "dc-track-src"
private const val DC_BOOST_SRC = "dc-boost-src"

/** Launch blue · landing red · guidance purple · boost solid · descent dashed. */
private fun installDriftCastLayers(style: Style) {
    listOf(DC_BOOST_SRC, DC_TRACK_SRC, DC_LAUNCH_SRC, DC_LANDING_SRC, DC_GUIDANCE_SRC)
        .forEach { style.addSource(GeoJsonSource(it)) }
    style.addLayer(
        LineLayer("dc-boost", DC_BOOST_SRC)
            .withProperties(lineColor("#7B1FA2"), lineWidth(2.5f)),
    )
    style.addLayer(
        LineLayer("dc-track", DC_TRACK_SRC)
            .withProperties(lineColor("#43A047"), lineWidth(2.5f), lineDasharray(arrayOf(2f, 2f))),
    )
    fun pin(id: String, src: String, color: String) = style.addLayer(
        CircleLayer(id, src).withProperties(
            circleRadius(8f), circleColor(color),
            circleStrokeWidth(2.5f), circleStrokeColor("#FFFFFF"),
        ),
    )
    pin("dc-launch", DC_LAUNCH_SRC, "#1E88E5")
    pin("dc-landing", DC_LANDING_SRC, "#D32F2F")
    pin("dc-guidance", DC_GUIDANCE_SRC, "#7B1FA2")
}

private fun driftCastStyleJson(container: AppContainer, source: TileSource): String = """
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
