package com.tinkerbug.tinkerrocket.app

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
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
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.tinkerbug.tinkerrocket.maps.TileProxyServer
import com.tinkerbug.tinkerrocket.maps.TileSource
import com.tinkerbug.tinkerrocket.session.DeviceSession
import org.maplibre.android.MapLibre
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.style.layers.CircleLayer
import org.maplibre.android.style.layers.PropertyFactory.circleColor
import org.maplibre.android.style.layers.PropertyFactory.circleRadius
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeColor
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeWidth
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.geojson.Feature
import org.maplibre.geojson.Point

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
) {
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

    var source by remember { mutableStateOf(TileSource.USGS_IMAGERY_TOPO) }
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
    LaunchedEffect(mapRef, source) {
        val map = mapRef ?: return@LaunchedEffect
        map.setStyle(Style.Builder().fromJson(rasterStyleJson(proxy, source))) { style ->
            installRocketMarker(style, fix?.latitude, fix?.longitude)
        }
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

        Column(Modifier.align(Alignment.TopEnd).padding(8.dp)) {
            OutlinedButton(onClick = {
                source = when (source) {
                    TileSource.USGS_IMAGERY_TOPO -> TileSource.USGS_TOPO
                    TileSource.USGS_TOPO -> TileSource.USGS_IMAGERY
                    TileSource.USGS_IMAGERY -> TileSource.USGS_IMAGERY_TOPO
                }
            }) { Text(source.displayName) }
            if (!follow) {
                OutlinedButton(onClick = {
                    follow = true
                    val f = fix
                    val map = mapRef
                    if (f != null && map != null) {
                        map.animateCamera(
                            CameraUpdateFactory.newLatLngZoom(LatLng(f.latitude, f.longitude), 15.0),
                        )
                    }
                }) { Text("Recenter") }
            }
        }

        Surface(
            modifier = Modifier.align(Alignment.BottomStart).padding(6.dp),
            tonalElevation = 2.dp,
        ) {
            Row {
                Text(
                    TileSource.ATTRIBUTION,
                    style = MaterialTheme.typography.labelSmall,
                    modifier = Modifier.padding(horizontal = 6.dp, vertical = 2.dp),
                )
            }
        }
    }

    // First camera: latched fix if we have one, else the last map position.
    LaunchedEffect(mapRef) {
        val map = mapRef ?: return@LaunchedEffect
        val f = fix
        if (f != null) {
            map.moveCamera(CameraUpdateFactory.newLatLngZoom(LatLng(f.latitude, f.longitude), 15.0))
        }
    }
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
