package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.MyLocation
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import com.tinkerbug.tinkerrocket.app.theme.TrMapIconButton
import com.tinkerbug.tinkerrocket.app.theme.TrMapPlate
import com.tinkerbug.tinkerrocket.app.theme.TrSpacing
import com.tinkerbug.tinkerrocket.app.theme.TrTheme
import com.tinkerbug.tinkerrocket.maps.TileProxyServer
import com.tinkerbug.tinkerrocket.maps.TileSource
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.LandingPredictor
import com.tinkerbug.tinkerrocket.session.LandingPrediction
import com.tinkerbug.tinkerrocket.session.LandingSnapshotSource
import kotlinx.coroutines.delay
import org.maplibre.android.MapLibre
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.style.layers.CircleLayer
import org.maplibre.android.style.layers.FillLayer
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.PropertyFactory.circleColor
import org.maplibre.android.style.layers.PropertyFactory.circleRadius
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeColor
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeWidth
import org.maplibre.android.style.layers.PropertyFactory.fillColor
import org.maplibre.android.style.layers.PropertyFactory.fillOpacity
import org.maplibre.android.style.layers.PropertyFactory.lineColor
import org.maplibre.android.style.layers.PropertyFactory.lineDasharray
import org.maplibre.android.style.layers.PropertyFactory.lineWidth
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.geojson.Feature
import org.maplibre.geojson.LineString
import org.maplibre.geojson.Point
import org.maplibre.geojson.Polygon
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/**
 * Rocket map — Phase 6 S2 spike shape, kept as the RocketMapView slot.
 * MapLibre raster-only over the localhost tile proxy: the style's tile
 * template points at 127.0.0.1 so every tile ride-throughs the offline
 * cache (view online once → renders offline forever).
 *
 * The two behaviors the spike exists to prove:
 *  - style swap = FULL style reload → every overlay layer must be
 *    re-installed in the setStyle callback (nothing survives the swap);
 *  - camera-reason gating: a REASON_API_GESTURE move breaks follow mode,
 *    programmatic recenter does not.
 *
 * #140: the marker reads the LATCHED lastValidRocketFix, never per-frame
 * telemetry lat/lon — after landing, frames stop carrying fresh GPS and a
 * per-frame read would blank the marker.
 */
@Composable
fun MapScreen(
    proxy: TileProxyServer,
    session: DeviceSession?,
    predictor: LandingPredictor? = null,
    phoneLocation: PhoneLocationManager? = null,
    /** True when the build carries a Map Tiles API key (AppContainer). */
    googleAvailable: Boolean = false,
    /** Opens the offline-areas manager from the basemap menu (iOS parity). */
    onManageOffline: (() -> Unit)? = null,
) {
    // Phone dot runs with the map (ref-counted manager).
    if (phoneLocation != null) {
        androidx.compose.runtime.DisposableEffect(Unit) {
            phoneLocation.start()
            onDispose { phoneLocation.stop() }
        }
    }
    val context = LocalContext.current
    remember {
        MapLibre.getInstance(context)
        // MapLibre fails requests FAST when the OS reports offline — it
        // never dials, so the (always-reachable) localhost proxy would go
        // blank in airplane mode.  Force "connected": every source URL is
        // loopback, and the PROXY owns real offline semantics (cache hit →
        // tile, upstream fail → hatch placeholder).  Found in the S2 spike.
        MapLibre.setConnected(true)
    }

    val fix by (
        session?.lastValidRocketFix
            ?: kotlinx.coroutines.flow.MutableStateFlow(null)
        ).collectAsState()
    val prediction by (
        predictor?.prediction
            ?: kotlinx.coroutines.flow.MutableStateFlow<LandingPrediction?>(null)
        ).collectAsState()
    val phoneFix by (
        phoneLocation?.location
            ?: kotlinx.coroutines.flow.MutableStateFlow(null)
        ).collectAsState()

    // Basemap-decision (plan §8 #2): USGS raster = DEFAULT + the only
    // offline/cacheable path; OpenFreeMap Liberty (keyless vector, ODbL)
    // and Google Satellite (Map Tiles API, key'd builds) are the
    // online-only options.  One state, not a source + an openFreeMap flag:
    // the picker is a list of five, and a flag beside an enum can express
    // combinations that aren't on it.
    var basemap by remember { mutableStateOf(Basemap.USGS_IMAGERY_TOPO) }
    var googleAttribution by remember { mutableStateOf<String?>(null) }
    var follow by remember { mutableStateOf(true) }

    // The map handle once ready; state so recomposition sees it.
    var mapRef by remember { mutableStateOf<MapLibreMap?>(null) }

    val mapView = remember {
        MapView(context).apply {
            onCreate(null)
            getMapAsync { map ->
                map.addOnCameraMoveStartedListener { reason ->
                    // Gesture pans break follow; API/animation moves don't.
                    if (reason == MapLibreMap.OnCameraMoveStartedListener.REASON_API_GESTURE) {
                        follow = false
                    }
                }
                mapRef = map
            }
        }
    }

    DisposableEffect(Unit) {
        mapView.onStart()
        mapView.onResume()
        onDispose {
            mapView.onPause()
            mapView.onStop()
            mapView.onDestroy()
        }
    }

    // Style install/swap: a swap is a FULL reload — overlays are re-added in
    // the callback every time (the layer-reinstall pattern the spike proves).
    LaunchedEffect(mapRef, basemap) {
        val map = mapRef ?: return@LaunchedEffect
        val raster = basemap.tileSource
        val builder = if (raster == null) {
            Style.Builder().fromUri(OPENFREEMAP_STYLE_URI)
        } else {
            Style.Builder().fromJson(rasterStyleJson(proxy, raster))
        }
        map.setStyle(builder) { style ->
            if (raster != null && !raster.cacheable) {
                // Online-only source: MapLibre's native Cache-Control parser
                // reads only max-age/must-revalidate, so the proxy's
                // no-store alone would NOT keep tiles out of the on-disk
                // ambient cache — volatile is the per-source "never
                // persist" switch.  The ambient cache is also disabled
                // app-wide in AppContainer; this is defense in depth.
                style.getSource("usgs")?.isVolatile = true
            }
            installPredictionLayers(style, prediction)
            installPhoneDot(style, phoneFix)
            installRocketMarker(style, fix?.latitude, fix?.longitude)
        }
    }

    // Prediction overlays track re-predictions (diff-based source updates).
    LaunchedEffect(mapRef, prediction) {
        val style = mapRef?.style ?: return@LaunchedEffect
        updatePredictionSources(style, prediction)
    }

    // Phone dot tracks the fused location.
    LaunchedEffect(mapRef, phoneFix) {
        val f = phoneFix ?: return@LaunchedEffect
        mapRef?.style?.getSourceAs<GeoJsonSource>(PHONE_SOURCE)
            ?.setGeoJson(Feature.fromGeometry(Point.fromLngLat(f.lon, f.lat)))
    }

    // Marker + follow tracking on fix updates (diff-based: update the source,
    // don't rebuild the style).
    LaunchedEffect(mapRef, fix) {
        val map = mapRef ?: return@LaunchedEffect
        val f = fix ?: return@LaunchedEffect
        map.style?.getSourceAs<GeoJsonSource>(ROCKET_SOURCE)?.setGeoJson(
            Feature.fromGeometry(Point.fromLngLat(f.longitude, f.latitude)),
        )
        if (follow) {
            // First latch with the camera still at world zoom → zoom in;
            // afterwards track position only, preserving the user's zoom.
            if (map.cameraPosition.zoom < 10.0) {
                map.animateCamera(
                    CameraUpdateFactory.newLatLngZoom(LatLng(f.latitude, f.longitude), 15.0),
                )
            } else {
                map.animateCamera(CameraUpdateFactory.newLatLng(LatLng(f.latitude, f.longitude)))
            }
        }
    }

    Box(Modifier.fillMaxSize()) {
        AndroidView(factory = { mapView }, modifier = Modifier.fillMaxSize())

        // Prediction staleness badge (top-left): color-graded so a frozen
        // prediction can't read as live; ticks once a second.
        prediction?.let { p ->
            var nowMs by remember { mutableStateOf(System.currentTimeMillis()) }
            LaunchedEffect(p) {
                while (true) {
                    nowMs = System.currentTimeMillis()
                    delay(1000)
                }
            }
            val ageS = max(0L, (nowMs - p.sampleAtMs) / 1000)
            val color = when {
                p.snapshotSource == LandingSnapshotSource.LATCHED -> Color(0xFFFFA000)
                ageS < 5 -> Color(0xFF43A047)
                ageS < 30 -> Color(0xFFFBC02D)
                else -> Color(0xFFE53935)
            }
            TrMapPlate(Modifier.align(Alignment.TopStart).padding(TrSpacing.rowSpacing)) {
                Text(
                    "⚑ landing ±%.0f m · T+%ds ago".format(p.uncertaintyMeters, ageS),
                    style = MaterialTheme.typography.labelMedium,
                    color = color,
                    modifier = Modifier.padding(horizontal = 8.dp, vertical = 5.dp),
                )
            }
        }

        // Floating controls, iOS MapView.swift arrangement: source menu over
        // recenter, top-trailing, each a 44dp glyph on a plate.
        Column(
            Modifier.align(Alignment.TopEnd).padding(TrSpacing.rowSpacing),
            verticalArrangement = Arrangement.spacedBy(TrSpacing.rowSpacing),
        ) {
            BasemapMenuButton(
                selected = basemap,
                options = Basemap.options(googleAvailable),
                onSelect = { basemap = it },
                onManageOffline = onManageOffline,
            )
            // Only while follow is broken — recentering while already
            // following is a no-op, and iOS's always-visible button is the
            // divergence this keeps (see the parity ledger).
            if (!follow) {
                TrMapIconButton(
                    icon = Icons.Filled.MyLocation,
                    contentDescription = "Recenter on the rocket",
                    tint = TrTheme.colors.mapControl,
                    onClick = {
                        follow = true
                        val f = fix
                        val map = mapRef
                        if (f != null && map != null) {
                            map.animateCamera(
                                CameraUpdateFactory.newLatLngZoom(
                                    LatLng(f.latitude, f.longitude),
                                    15.0,
                                ),
                            )
                        }
                    },
                )
            }
        }

        // Active source + attribution (iOS bottom-leading plate).  The source
        // name lives HERE, not on the button that changes it: this plate is
        // already the most readable thing on the screen, and it frees the
        // control up top to be a 44dp glyph like its iOS twin.
        TrMapPlate(Modifier.align(Alignment.BottomStart).padding(TrSpacing.rowSpacing)) {
            Column(Modifier.padding(horizontal = 8.dp, vertical = 5.dp)) {
                Text(
                    basemap.label,
                    style = MaterialTheme.typography.labelMedium,
                    fontWeight = FontWeight.Bold,
                )
                Text(
                    basemap.attribution(googleAttribution),
                    fontSize = 9.sp,
                    lineHeight = 12.sp,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }

    // Map Tiles display policy wants the live copyright string alongside the
    // Google name; the proxy memoizes it.  Keep retrying while the source is
    // active — a first fetch racing a slow createSession must not pin the
    // static fallback for the whole session.  Cancelled on source switch.
    LaunchedEffect(basemap) {
        while (basemap == Basemap.GOOGLE_SATELLITE && googleAttribution == null) {
            googleAttribution = fetchGoogleAttribution(proxy)
            if (googleAttribution == null) kotlinx.coroutines.delay(10_000)
        }
    }

    // First camera: the rocket's latched fix when we already have one, else
    // the PHONE — the map is opened on the pad, before the rocket has a fix,
    // and opening on the whole planet when the phone knows exactly where it
    // is helps nobody.  Fires on the first fix of either kind (a cold GPS
    // lands well after the map does), once only, and never against a user
    // who has already panned (`follow` clears on gesture).  A rocket fix
    // arriving later still takes the camera — that's the follow effect above,
    // which pans at the zoom the user is on.
    var initialCentered by remember { mutableStateOf(false) }
    LaunchedEffect(mapRef, fix, phoneFix) {
        val map = mapRef ?: return@LaunchedEffect
        if (initialCentered || !follow) return@LaunchedEffect
        val target = fix?.let { LatLng(it.latitude, it.longitude) }
            ?: phoneFix?.let { LatLng(it.lat, it.lon) }
            ?: return@LaunchedEffect
        map.moveCamera(CameraUpdateFactory.newLatLngZoom(target, 15.0))
        initialCentered = true
    }
}

/** Live Google copyright via the proxy's memoized /meta route. */
private suspend fun fetchGoogleAttribution(proxy: TileProxyServer): String? =
    kotlinx.coroutines.withContext(kotlinx.coroutines.Dispatchers.IO) {
        runCatching {
            val conn = java.net.URL(
                "http://127.0.0.1:${proxy.port}/meta/googleSatellite/attribution",
            ).openConnection() as java.net.HttpURLConnection
            conn.connectTimeout = 5_000
            conn.readTimeout = 10_000
            try {
                if (conn.responseCode == 200) {
                    conn.inputStream.readBytes().decodeToString()
                } else {
                    null
                }
            } finally {
                conn.disconnect()
            }
        }.getOrNull()
    }

private const val ROCKET_SOURCE = "rocket-src"
private const val ROCKET_LAYER = "rocket-layer"

/** Raster style over the proxy; background matches the hatch placeholder. */
private fun rasterStyleJson(proxy: TileProxyServer, source: TileSource): String = """
{
  "version": 8,
  "sources": {
    "usgs": {
      "type": "raster",
      "tiles": ["${proxy.templateFor(source)}"],
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

private const val PRED_PIN_SOURCE = "pred-pin-src"
private const val PRED_PIN_LAYER = "pred-pin-layer"
private const val PRED_TRACK_SOURCE = "pred-track-src"
private const val PRED_TRACK_LAYER = "pred-track-layer"
private const val PRED_UNCERT_SOURCE = "pred-uncert-src"
private const val PRED_UNCERT_LAYER = "pred-uncert-layer"

/**
 * Prediction overlays (#156/#191), iOS renderer colors: green pin, dashed
 * green descent track ("this is a prediction"), near-transparent green
 * uncertainty disc.  Order: disc under track under pin; the rocket marker
 * is installed after these so it stays on top.
 */
private fun installPredictionLayers(style: Style, prediction: LandingPrediction?) {
    style.addSource(GeoJsonSource(PRED_UNCERT_SOURCE))
    style.addSource(GeoJsonSource(PRED_TRACK_SOURCE))
    style.addSource(GeoJsonSource(PRED_PIN_SOURCE))
    style.addLayer(
        FillLayer(PRED_UNCERT_LAYER, PRED_UNCERT_SOURCE)
            .withProperties(fillColor("#43A047"), fillOpacity(0.12f)),
    )
    style.addLayer(
        LineLayer(PRED_TRACK_LAYER, PRED_TRACK_SOURCE).withProperties(
            lineColor("#43A047"),
            lineWidth(2.5f),
            lineDasharray(arrayOf(2f, 2f)),
        ),
    )
    style.addLayer(
        CircleLayer(PRED_PIN_LAYER, PRED_PIN_SOURCE).withProperties(
            circleRadius(8f),
            circleColor("#43A047"),
            circleStrokeWidth(2.5f),
            circleStrokeColor("#FFFFFF"),
        ),
    )
    updatePredictionSources(style, prediction)
}

private fun updatePredictionSources(style: Style, prediction: LandingPrediction?) {
    val p = prediction ?: return
    style.getSourceAs<GeoJsonSource>(PRED_PIN_SOURCE)
        ?.setGeoJson(Feature.fromGeometry(Point.fromLngLat(p.landingLon, p.landingLat)))
    if (p.descentTrack.size >= 2) {
        style.getSourceAs<GeoJsonSource>(PRED_TRACK_SOURCE)?.setGeoJson(
            Feature.fromGeometry(
                LineString.fromLngLats(p.descentTrack.map { Point.fromLngLat(it.lon, it.lat) }),
            ),
        )
    }
    if (p.uncertaintyMeters > 0) {
        style.getSourceAs<GeoJsonSource>(PRED_UNCERT_SOURCE)?.setGeoJson(
            Feature.fromGeometry(uncertaintyPolygon(p.landingLat, p.landingLon, p.uncertaintyMeters)),
        )
    }
}

/** Small-angle circle polygon, fine for the ≤km radii the model produces. */
private fun uncertaintyPolygon(lat: Double, lon: Double, radiusM: Double, points: Int = 64): Polygon {
    val dLat = radiusM / 111_320.0
    val dLon = radiusM / (111_320.0 * max(0.01, cos(Math.toRadians(lat))))
    val ring = (0..points).map { i ->
        val a = 2.0 * Math.PI * i / points
        Point.fromLngLat(lon + dLon * sin(a), lat + dLat * cos(a))
    }
    return Polygon.fromLngLats(listOf(ring))
}

private const val PHONE_SOURCE = "phone-src"
private const val PHONE_LAYER = "phone-layer"

/** Blue phone-position dot (the operator), under the rocket marker. */
private fun installPhoneDot(style: Style, fix: PhoneLocationManager.PhoneFix?) {
    val src = GeoJsonSource(PHONE_SOURCE)
    fix?.let { src.setGeoJson(Feature.fromGeometry(Point.fromLngLat(it.lon, it.lat))) }
    style.addSource(src)
    style.addLayer(
        CircleLayer(PHONE_LAYER, PHONE_SOURCE).withProperties(
            circleRadius(7f),
            circleColor("#1E88E5"),
            circleStrokeWidth(2.5f),
            circleStrokeColor("#FFFFFF"),
        ),
    )
}

private fun installRocketMarker(style: Style, lat: Double?, lon: Double?) {
    val feature = if (lat != null && lon != null) {
        Feature.fromGeometry(Point.fromLngLat(lon, lat))
    } else {
        null
    }
    val src = GeoJsonSource(ROCKET_SOURCE)
    feature?.let { src.setGeoJson(it) }
    style.addSource(src)
    style.addLayer(
        CircleLayer(ROCKET_LAYER, ROCKET_SOURCE).withProperties(
            circleRadius(9f),
            circleColor("#D32F2F"),
            circleStrokeWidth(2.5f),
            circleStrokeColor("#FFFFFF"),
        ),
    )
}
