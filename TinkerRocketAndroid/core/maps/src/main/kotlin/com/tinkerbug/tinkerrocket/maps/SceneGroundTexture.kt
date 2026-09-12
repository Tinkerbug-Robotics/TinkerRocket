package com.tinkerbug.tinkerrocket.maps

import java.net.HttpURLConnection
import java.net.URL
import kotlin.math.PI
import kotlin.math.cos

/**
 * The satellite texture under a 3D scene's ground plane, cached for offline
 * use — the twin of iOS `SceneGroundTexture` (#1092 item 1).
 *
 * iOS paints a 1024×1024 public-domain USGS export on the ground plane of
 * both 3D scenes (the flight path and Drift Cast) and drops the wireframe
 * grid; Android drew the grid only, so the scene had no terrain reference.
 * Same bbox maths, same endpoint, same `3d_<bbox>` key shape, so the two
 * platforms share one design and two views of one site share one texture.
 *
 * Pure JVM: this module has no Android in it, so the bytes come back as
 * bytes and the caller decodes them — the app turns them into a bitmap, a
 * test can look at them as a file.  The network fetch is a function
 * parameter for the same reason.
 *
 * USGS `USGSImageryOnly` is US-only; outside the US the export comes back
 * blank and the scene keeps its grid — #231 records why there is no global
 * cacheable source yet.
 */
public object SceneGroundTexture {

    /** Public-domain USGS imagery export — no key, the 2D offline tiles' source. */
    public const val ENDPOINT: String =
        "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export"

    /** Pixels per side of the export. */
    public const val SIZE_PX: Int = 1024

    /**
     * What one scene asks for.  [halfM] is the half-width of the square the
     * image covers, in metres from the reference point, so the caller can put
     * the quad exactly under the bbox rather than stretching it to something.
     */
    public data class Request(val url: String, val cacheKey: String, val halfM: Double)

    /**
     * The bounding box, request URL and cache key for a scene centred on
     * [refLat]/[refLon] with the scene's [extent] (its framing radius, the
     * same number the ground grid uses).  Pure, so the endpoint and the key
     * can be pinned by a test.  The key is derived from the bbox alone, so
     * two views showing the same site share one cached texture.
     */
    public fun request(refLat: Double, refLon: Double, extent: Double): Request? {
        val halfM = extent * 1.5
        val mPerDegLat = 110_540.0
        val mPerDegLon = 111_320.0 * cos(refLat * PI / 180.0)
        if (!halfM.isFinite() || halfM <= 0.0 || !refLat.isFinite() || !refLon.isFinite() ||
            mPerDegLon == 0.0
        ) {
            return null
        }
        val south = refLat - halfM / mPerDegLat
        val north = refLat + halfM / mPerDegLat
        val west = refLon - halfM / mPerDegLon
        val east = refLon + halfM / mPerDegLon
        val bbox = "$west,$south,$east,$north"
        val url = ENDPOINT +
            "?bbox=$bbox&bboxSR=4326&imageSR=4326" +
            "&size=$SIZE_PX,$SIZE_PX&format=png&f=image"
        return Request(url, "3d_$bbox", halfM)
    }

    /**
     * The texture bytes for [req]: cache first, so a scene viewed once renders
     * offline later; on a miss, [fetch] it and store it.  Null means no
     * imagery — offline and never cached, or the fetch failed — and the
     * caller keeps its grid.  Blocking: call it off the main thread.
     */
    public fun load(
        cache: OfflineTileCache,
        req: Request,
        fetch: (String) -> ByteArray? = ::fetchHttp,
    ): ByteArray? {
        cache.blob(req.cacheKey)?.let { return it }
        val bytes = fetch(req.url) ?: return null
        cache.storeBlob(bytes, req.cacheKey)
        return bytes
    }

    /** One GET; null on anything but a 200 with a body. */
    public fun fetchHttp(url: String): ByteArray? = try {
        val conn = URL(url).openConnection() as HttpURLConnection
        try {
            conn.connectTimeout = 10_000
            conn.readTimeout = 20_000
            conn.setRequestProperty("User-Agent", "TinkerRocket-Android")
            if (conn.responseCode != 200) null
            else conn.inputStream.use { it.readBytes() }.takeIf { it.isNotEmpty() }
        } finally {
            conn.disconnect()
        }
    } catch (_: Exception) {
        null
    }
}
