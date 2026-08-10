package com.tinkerbug.tinkerrocket.maps

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlinx.serialization.json.put
import java.io.File
import java.util.UUID

/**
 * A saved offline region — port of iOS OfflineRegion.  The manifest JSON is
 * SCHEMA-IDENTICAL to the iOS Codable output (uppercase UUID strings,
 * `savedAt` in Apple-epoch seconds) so a future backup/restore or cache
 * import can move between platforms.
 */
public data class OfflineRegion(
    val id: UUID = UUID.randomUUID(),
    val name: String,
    val lat: Double,
    val lon: Double,
    val radiusMeters: Double,
    val minZoom: Int,
    val maxZoom: Int,
    /** TileSource key (iOS rawValue). */
    val source: String,
    val tileCount: Int,
    val bytes: Long,
    val savedAtMs: Long,
) {
    public val spec: RegionSpec
        get() = RegionSpec(
            centerLat = lat, centerLon = lon,
            radiusMeters = radiusMeters, minZoom = minZoom, maxZoom = maxZoom,
        )

    public val tileSource: TileSource? get() = TileSource.fromKey(source)
}

/**
 * Persisted store of saved regions (iOS OfflineRegionStore) — manifest JSON
 * `offline_regions.json` in [directory].  Single-writer: mutate from one
 * dispatcher.
 */
public class OfflineRegionStore(
    directory: File,
    private val cache: OfflineTileCache,
) {
    private val file = File(directory.apply { mkdirs() }, "offline_regions.json")

    private val _regions = MutableStateFlow<List<OfflineRegion>>(emptyList())
    public val regions: StateFlow<List<OfflineRegion>> = _regions.asStateFlow()

    init {
        load()
    }

    public val totalBytes: Long get() = _regions.value.sumOf { it.bytes }

    public fun add(region: OfflineRegion) {
        _regions.value = listOf(region) + _regions.value.filterNot { it.id == region.id }
        save()
    }

    /**
     * Remove a region from the manifest and delete the tiles no SURVIVING
     * region still needs.
     *
     * Deleting by geometry alone punched holes in overlapping regions: two
     * areas over the same launch site share tile paths, so dropping one took
     * the shared tiles with it and left the other quietly incomplete.  The
     * old comment shrugged that a lost tile "will simply re-download next
     * time it's viewed online" — which is true at home and exactly wrong at
     * a field with no signal, where these regions are the entire point.
     *
     * Only same-source regions can share tiles (the source is a directory in
     * the path), so the keep-set is built from those alone.  Tiles cached
     * passively by panning the map are not claimed by any region and are
     * still removed if they fall inside — they cost one re-fetch online, and
     * tracking them would mean refcounting the whole cache.
     */
    public fun delete(region: OfflineRegion) {
        val survivors = _regions.value.filterNot { it.id == region.id }
        val keep = survivors
            .filter { it.source == region.source }
            .flatMapTo(HashSet()) { TileMath.tiles(it.spec) }
        cache.removeTiles(region.source, TileMath.tiles(region.spec).filterNot { it in keep })
        _regions.value = survivors
        save()
    }

    // ── Persistence (iOS-schema JSON) ────────────────────────────────────

    private fun load() {
        val loaded = runCatching {
            Json.parseToJsonElement(file.readText()).jsonArray.map { el ->
                decodeRegion(el.jsonObject)
            }
        }.getOrNull() ?: return
        _regions.value = loaded
    }

    private fun save() {
        val json = buildJsonArray {
            _regions.value.forEach { r -> add(encodeRegion(r)) }
        }
        val tmp = File(file.path + ".tmp")
        tmp.writeText(json.toString())
        if (!tmp.renameTo(file)) {
            tmp.copyTo(file, overwrite = true)
            tmp.delete()
        }
    }

    private fun encodeRegion(r: OfflineRegion): JsonObject = buildJsonObject {
        put("id", r.id.toString().uppercase())
        put("name", r.name)
        put("lat", r.lat)
        put("lon", r.lon)
        put("radiusMeters", r.radiusMeters)
        put("minZoom", r.minZoom)
        put("maxZoom", r.maxZoom)
        put("source", r.source)
        put("tileCount", r.tileCount)
        put("bytes", r.bytes)
        put("savedAt", r.savedAtMs / 1000.0 - APPLE_EPOCH_OFFSET_S)
    }

    private fun decodeRegion(o: JsonObject): OfflineRegion = OfflineRegion(
        id = UUID.fromString(o.getValue("id").jsonPrimitive.content),
        name = o.getValue("name").jsonPrimitive.content,
        lat = o.getValue("lat").jsonPrimitive.double,
        lon = o.getValue("lon").jsonPrimitive.double,
        radiusMeters = o.getValue("radiusMeters").jsonPrimitive.double,
        minZoom = o.getValue("minZoom").jsonPrimitive.int,
        maxZoom = o.getValue("maxZoom").jsonPrimitive.int,
        source = o.getValue("source").jsonPrimitive.content,
        tileCount = o.getValue("tileCount").jsonPrimitive.int,
        bytes = o.getValue("bytes").jsonPrimitive.long,
        savedAtMs = ((o.getValue("savedAt").jsonPrimitive.double + APPLE_EPOCH_OFFSET_S) * 1000.0).toLong(),
    )

    private companion object {
        /** Swift Date reference epoch (2001-01-01) — same pin as RocketProfileCodec. */
        const val APPLE_EPOCH_OFFSET_S = 978_307_200.0
    }
}
