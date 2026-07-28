package com.tinkerbug.tinkerrocket.maps

import kotlin.math.asinh
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

/** A single web-mercator tile coordinate. */
public data class TileXYZ(val z: Int, val x: Int, val y: Int)

/**
 * What to download: a circle (center + radius) over a span of zoom levels —
 * port of iOS TileMath.swift RegionSpec.
 */
public data class RegionSpec(
    val centerLat: Double,
    val centerLon: Double,
    val radiusMeters: Double,
    val minZoom: Int = 10,
    val maxZoom: Int,
) {
    /**
     * Lat/lon bounding box of the circle (small-angle approximation — fine
     * for the few-km radii used here).
     */
    public val bbox: Bbox
        get() {
            val dLat = radiusMeters / 111_320.0
            val cosLat = max(0.01, cos(Math.toRadians(centerLat)))
            val dLon = radiusMeters / (111_320.0 * cosLat)
            return Bbox(centerLat - dLat, centerLat + dLat, centerLon - dLon, centerLon + dLon)
        }
}

public data class Bbox(val latMin: Double, val latMax: Double, val lonMin: Double, val lonMax: Double)

/** Slippy-map tile math — port of iOS TileMath.swift, formulas verbatim. */
public object TileMath {

    public fun tileX(lon: Double, z: Int): Int {
        val n = (1 shl z).toDouble()
        return floor((lon + 180.0) / 360.0 * n).toInt()
    }

    public fun tileY(lat: Double, z: Int): Int {
        val n = (1 shl z).toDouble()
        val r = Math.toRadians(lat)
        return floor((1.0 - asinh(tan(r)) / Math.PI) / 2.0 * n).toInt()
    }

    private fun clamp(v: Int, lo: Int, hi: Int) = min(max(v, lo), hi)

    /** Inclusive tile rectangle for a bbox at one zoom level. */
    private fun rect(b: Bbox, z: Int): IntArray {
        val n = 1 shl z
        // North (latMax) maps to the smaller y; south (latMin) to the larger y.
        return intArrayOf(
            clamp(tileX(b.lonMin, z), 0, n - 1),
            clamp(tileX(b.lonMax, z), 0, n - 1),
            clamp(tileY(b.latMax, z), 0, n - 1),
            clamp(tileY(b.latMin, z), 0, n - 1),
        )
    }

    /** Total tile count for the region (for the size estimate; no allocation). */
    public fun tileCount(region: RegionSpec): Int {
        val b = region.bbox
        var count = 0
        for (z in min(region.minZoom, region.maxZoom)..region.maxZoom) {
            val (xMin, xMax, yMin, yMax) = rect(b, z)
            count += (xMax - xMin + 1) * (yMax - yMin + 1)
        }
        return count
    }

    /** The full list of tiles to download for the region. */
    public fun tiles(region: RegionSpec): List<TileXYZ> {
        val b = region.bbox
        val out = mutableListOf<TileXYZ>()
        for (z in min(region.minZoom, region.maxZoom)..region.maxZoom) {
            val (xMin, xMax, yMin, yMax) = rect(b, z)
            for (x in xMin..xMax) {
                for (y in yMin..yMax) {
                    out += TileXYZ(z, x, y)
                }
            }
        }
        return out
    }

    private operator fun IntArray.component1() = this[0]
    private operator fun IntArray.component2() = this[1]
    private operator fun IntArray.component3() = this[2]
    private operator fun IntArray.component4() = this[3]
}
