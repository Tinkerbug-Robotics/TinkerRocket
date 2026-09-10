package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.FetchedCatalog
import com.tinkerbug.tinkerrocket.protocol.FirmwareImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareManifest
import com.tinkerbug.tinkerrocket.protocol.FirmwareRelease
import com.tinkerbug.tinkerrocket.protocol.FirmwareReleaseLocator
import java.io.File

/**
 * Firmware kept on the phone so a launch site with no signal is survivable
 * (#773). iOS twin: `Models/FirmwareCache.swift`.
 *
 * WHAT IS STORED, AND WHY THE MANIFEST TOO. Images alone are not enough: with
 * no network the app cannot LIST anything, so it cannot offer what it already
 * holds. The catalog — the release and the manifest exactly as fetched — is
 * cached alongside, and an offline check falls back to it.
 *
 * CONTENT-ADDRESSED. An image is stored under its own SHA-256, which is the
 * name the manifest gives it. Two releases sharing an unchanged image share
 * one file, a re-download of something already held is free, and there is no
 * separate index to fall out of step with the directory.
 *
 * VERIFIED ON READ, ALWAYS. The file name is a claim about the contents, not
 * proof of them — a half-written file from a killed app has the right name and
 * the wrong bytes. Every read re-hashes and discards anything that does not
 * match. Cheap next to a flash, and the alternative is flashing a rocket from
 * a file nobody checked.
 */
public class FirmwareCache(
    private val dir: File,
    private val sha256: (ByteArray) -> String,
) {
    private val imagesDir: File get() = File(dir, "images")
    private val releaseFile: File get() = File(dir, "release.json")
    private val manifestFile: File get() = File(dir, "manifest.json")

    // ── the catalog ──────────────────────────────────────────────────────

    /** Remember a fetched catalog so a later offline check can still list it. */
    public fun putCatalog(catalog: FetchedCatalog): Boolean = try {
        dir.mkdirs()
        // Manifest first: a reader requires BOTH, so a crash between the two
        // leaves a stale-but-consistent catalog rather than a torn new one.
        manifestFile.writeText(catalog.manifestJson)
        releaseFile.writeText(catalog.release.toListingJson())
        true
    } catch (_: Exception) {
        false
    }

    /**
     * The last catalog stored, or null if there is none or it is unreadable.
     *
     * Read back through the same parsers the network path uses, so there is
     * one codec rather than two that can disagree.
     */
    public fun catalog(): Pair<FirmwareRelease, FirmwareManifest>? = try {
        val release = FirmwareReleaseLocator
            .firmwareReleases(releaseFile.readText(), includePrereleases = true)
            .firstOrNull()
        val manifest = FirmwareManifest.parse(manifestFile.readText())
        if (release != null && manifest != null) release to manifest else null
    } catch (_: Exception) {
        null
    }

    // ── the images ───────────────────────────────────────────────────────

    private fun fileFor(image: FirmwareImage) = File(imagesDir, image.sha256)

    /**
     * Is this image held? A cheap check for a badge in a list — existence and
     * length only, no hashing. [image] is what proves it before use.
     */
    public fun hasImage(image: FirmwareImage): Boolean =
        fileFor(image).let { it.isFile && it.length() == image.sizeBytes }

    /** Store bytes that have already been proven against the manifest. */
    public fun putImage(image: FirmwareImage, bytes: ByteArray): Boolean = try {
        imagesDir.mkdirs()
        // Write beside, then rename: a rename is atomic on every filesystem
        // this runs on, so a killed app cannot leave a half-file under a name
        // that claims to be a whole one.
        val tmp = File(imagesDir, "${image.sha256}.part")
        tmp.writeBytes(bytes)
        tmp.renameTo(fileFor(image))
    } catch (_: Exception) {
        false
    }

    /**
     * The cached bytes for [image], or null when they are absent or wrong.
     *
     * Anything that fails the check is DELETED rather than left to be found
     * again: a file that did not match once will not match later, and leaving
     * it invites a second attempt to trust it.
     */
    public fun image(image: FirmwareImage): ByteArray? {
        val f = fileFor(image)
        return try {
            if (!f.isFile) return null
            val bytes = f.readBytes()
            if (bytes.size.toLong() != image.sizeBytes ||
                sha256(bytes).lowercase() != image.sha256
            ) {
                f.delete()
                return null
            }
            bytes
        } catch (_: Exception) {
            f.delete()
            null
        }
    }

    /** Every SHA currently held, for a "what do I have offline" view. */
    public fun heldShas(): Set<String> =
        (imagesDir.listFiles() ?: emptyArray())
            .filter { it.isFile && !it.name.endsWith(".part") }
            .map { it.name }
            .toSet()

    /**
     * Drop everything not in [keep].
     *
     * Called with the current release's SHAs, so superseded images age out
     * instead of filling the phone one release at a time. Nothing here is
     * precious — anything dropped can be fetched again with a signal.
     */
    public fun prune(keep: Set<String>) {
        (imagesDir.listFiles() ?: emptyArray())
            .filter { it.isFile && it.name !in keep }
            .forEach { it.delete() }
    }

    public fun bytesHeld(): Long =
        (imagesDir.listFiles() ?: emptyArray()).filter { it.isFile }.sumOf { it.length() }
}
