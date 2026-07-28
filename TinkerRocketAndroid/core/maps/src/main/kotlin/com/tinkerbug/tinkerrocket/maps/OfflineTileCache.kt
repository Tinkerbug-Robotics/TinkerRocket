package com.tinkerbug.tinkerrocket.maps

import java.io.File

/**
 * On-disk read-through cache for map tiles — port of iOS OfflineTileCache.
 * Layout is BYTE-IDENTICAL to iOS: `<root>/<source>/<z>/<x>/<y>.tile`
 * (slippy z/x/y on disk, even though the ArcGIS URL is z/y/x — the
 * transposition test pins this).  On Android the root is
 * `noBackupFilesDir/OfflineTiles` — the analogue of iOS Application Support
 * + excluded-from-backup: never a cache dir the OS purges under pressure,
 * which would silently delete the field data the user pre-downloaded.
 */
public class OfflineTileCache(private val root: File) {

    init {
        root.mkdirs()
    }

    /** Cached bytes for a tile, or null if not stored. */
    public fun tileData(source: String, z: Int, x: Int, y: Int): ByteArray? =
        fileFor(source, z, x, y).takeIf { it.isFile }?.readBytes()

    /** Persist tile bytes atomically, creating intermediate directories. */
    public fun store(data: ByteArray, source: String, z: Int, x: Int, y: Int) {
        val file = fileFor(source, z, x, y)
        file.parentFile?.mkdirs()
        val tmp = File(file.path + ".tmp")
        tmp.writeBytes(data)
        if (!tmp.renameTo(file)) {
            tmp.copyTo(file, overwrite = true)
            tmp.delete()
        }
    }

    /** Bytes on disk for a source (region manager / debug). */
    public fun byteCount(source: String): Long =
        File(root, source).walkBottomUp().filter { it.isFile }.sumOf { it.length() }

    /**
     * Delete the given tiles for a source from disk (used when a saved
     * region is removed).  Best-effort.
     */
    public fun removeTiles(source: String, tiles: List<TileXYZ>) {
        tiles.forEach { fileFor(source, it.z, it.x, it.y).delete() }
    }

    private fun fileFor(source: String, z: Int, x: Int, y: Int) =
        File(root, "$source/$z/$x/$y.tile")
}
