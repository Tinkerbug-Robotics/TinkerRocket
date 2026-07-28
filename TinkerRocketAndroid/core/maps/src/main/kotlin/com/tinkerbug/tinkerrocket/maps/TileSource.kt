package com.tinkerbug.tinkerrocket.maps

/**
 * Selectable map imagery sources — port of iOS TileSource.swift.  The Apple
 * basemap cases have no Android equivalent and are omitted; USGS tiles are
 * public domain and are the only sources cached for offline use (the point
 * of the feature — see the iOS file for the Esri/Google rationale).
 *
 * The `key` doubles as the on-disk cache directory name and MUST match the
 * iOS rawValue so a future cache import/export stays byte-compatible.
 */
public enum class TileSource(
    public val key: String,
    public val displayName: String,
    /**
     * Upstream URL template.  ArcGIS REST tile endpoints are ordered
     * `z/y/x` — NOT the slippy-map `z/x/y` used on disk and by MapLibre
     * templates; the proxy owns that transposition.
     */
    public val urlTemplate: String,
    /** Provider's max native zoom; the map may overzoom beyond it. */
    public val maxZoom: Int,
) {
    USGS_IMAGERY(
        key = "usgsImagery",
        displayName = "USGS Imagery",
        urlTemplate = "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{z}/{y}/{x}",
        maxZoom = 16,
    ),
    USGS_IMAGERY_TOPO(
        key = "usgsImageryTopo",
        displayName = "USGS Imagery + Topo",
        urlTemplate = "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryTopo/MapServer/tile/{z}/{y}/{x}",
        maxZoom = 16,
    ),
    USGS_TOPO(
        key = "usgsTopo",
        displayName = "USGS Topo",
        urlTemplate = "https://basemap.nationalmap.gov/arcgis/rest/services/USGSTopo/MapServer/tile/{z}/{y}/{x}",
        maxZoom = 16,
    ),
    ;

    public companion object {
        public fun fromKey(key: String): TileSource? = entries.firstOrNull { it.key == key }

        /** Attribution shown while a USGS source is active (provider terms). */
        public const val ATTRIBUTION: String = "USGS · The National Map"
    }
}
