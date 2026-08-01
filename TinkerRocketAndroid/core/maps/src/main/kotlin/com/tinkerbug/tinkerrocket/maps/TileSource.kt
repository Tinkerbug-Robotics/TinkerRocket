package com.tinkerbug.tinkerrocket.maps

/**
 * Selectable map imagery sources — port of iOS TileSource.swift.  The Apple
 * basemap cases have no Android equivalent and are omitted; USGS tiles are
 * public domain and are the only sources cached for offline use (the point
 * of the feature — see the iOS file for the Esri/Google rationale).
 *
 * GOOGLE_SATELLITE plays the iOS "Apple basemap" role on Android: a global,
 * online-only imagery layer whose provider terms forbid persistence.  It is
 * `cacheable = false` with a null template — the proxy fetches it through
 * [GoogleTileUpstream] (Map Tiles API session tokens) and never touches the
 * offline cache in either direction.
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
     * templates; the proxy owns that transposition.  Null for sources
     * fetched via a dedicated authenticated upstream instead of template
     * substitution — null also drops the source from every default
     * template map, so it can never be region-downloaded.
     */
    public val urlTemplate: String?,
    /** Provider's max native zoom; the map may overzoom beyond it. */
    public val maxZoom: Int,
    /**
     * Whether tiles may be persisted to the offline cache (mirrors iOS
     * `isCacheable`).  False = online-only by provider terms: the proxy
     * neither reads nor writes the cache for this source and stamps
     * responses `Cache-Control: no-store`.
     */
    public val cacheable: Boolean = true,
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
    GOOGLE_SATELLITE(
        key = "googleSatellite",
        displayName = "Google Satellite",
        urlTemplate = null,
        maxZoom = 22,
        cacheable = false,
    ),
    ;

    public companion object {
        public fun fromKey(key: String): TileSource? = entries.firstOrNull { it.key == key }

        /** Attribution shown while a USGS source is active (provider terms). */
        public const val ATTRIBUTION: String = "USGS · The National Map"

        /**
         * Fallback attribution while Google Satellite is active; the proxy's
         * `/meta/googleSatellite/attribution` route serves the live copyright
         * string (Map Tiles API display requirement) once a session is up.
         */
        public const val GOOGLE_ATTRIBUTION: String = "© Google"
    }
}
